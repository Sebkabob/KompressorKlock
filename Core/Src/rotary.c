#include "rotary.h"
#include "main.h"
#include "screens.h"
#include "settings.h"

/*
 * ================= INTERRUPT-DRIVEN QUADRATURE DECODING =================
 */

static const int8_t encoder_lut[16] = {
      0,     1,    -1,     0,
     -1,     0,     0,     1,
      1,     0,     0,    -1,
      0,    -1,     1,     0,
};

/* ================= TIMING ================= */
#define ENCODER_DEBOUNCE_MS   20u
#define SW_DEBOUNCE_MS        50u
#define DETENT_THRESHOLD       4
#define LONG_PRESS_MS       2000u

/* ================= STATE ================= */
static volatile uint8_t  enc_state  = 0;
static volatile int8_t   step_accum = 0;

/*
 * Persistent accumulator: ISR adds raw sub-steps to step_accum.
 * The main loop copies step_accum into pending_steps (atomically)
 * and only consumes full detents, carrying the remainder forward.
 * This prevents losing partial steps between poll cycles.
 */
static int8_t   pending_steps = 0;
static uint32_t last_step_tick = 0;

static bool     sw_last_raw    = true;
static bool     sw_stable      = true;
static uint32_t sw_change_tick = 0;
static bool     sw_pressed_flag = false;

/* Long-press tracking */
static bool     sw_held          = false;
static uint32_t sw_held_since    = 0;
static bool     sw_long_consumed = false;

/* ================= ISR ================= */

void Rotary_EXTI_Handler(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b = (HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    uint8_t cur = (a << 1) | b;
    uint8_t idx = (enc_state << 2) | cur;

    int8_t delta = encoder_lut[idx & 0x0Fu];
    if (delta != 0) {
        step_accum += delta;
    }

    enc_state = cur;
}

/* ================= INIT ================= */

void Rotary_Init(void)
{
    uint8_t a = (HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b = (HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    enc_state = (a << 1) | b;

    sw_last_raw    = (HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == GPIO_PIN_SET);
    sw_stable      = sw_last_raw;
    sw_change_tick = HAL_GetTick();
    sw_pressed_flag = false;
    sw_held         = false;
    sw_long_consumed = false;
    last_step_tick  = HAL_GetTick();
    step_accum      = 0;
    pending_steps   = 0;

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin  = ROT_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ROT_A_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = ROT_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ROT_B_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* ================= UPDATE (call from main loop ~5ms) ================= */

void Rotary_Update(void)
{
    uint32_t now = HAL_GetTick();
    bool in_settings = (Settings_GetState() != SETTINGS_STATE_INACTIVE);

    /* ---- Encoder: merge ISR steps into persistent accumulator ---- */
    {
        __disable_irq();
        int8_t new_steps = step_accum;
        step_accum = 0;
        __enable_irq();

        pending_steps += new_steps;

        if ((now - last_step_tick) >= ENCODER_DEBOUNCE_MS)
        {
            if (in_settings) {
                /* Consume one detent at a time, carry remainder */
                while (pending_steps >= DETENT_THRESHOLD) {
                    Settings_OnScroll(-1);  /* Negated to match physical direction */
                    Screen_MarkDirty();
                    pending_steps -= DETENT_THRESHOLD;
                    last_step_tick = now;
                }
                while (pending_steps <= -DETENT_THRESHOLD) {
                    Settings_OnScroll(1);   /* Negated to match physical direction */
                    Screen_MarkDirty();
                    pending_steps += DETENT_THRESHOLD;
                    last_step_tick = now;
                }
            } else {
                if (!Screen_IsTransitioning()) {
                    if (pending_steps >= DETENT_THRESHOLD) {
                        Screen_Next(TRANSITION_SLIDE_UP);
                        pending_steps = 0;  /* Consume all on screen transition */
                        last_step_tick = now;
                    } else if (pending_steps <= -DETENT_THRESHOLD) {
                        Screen_Prev(TRANSITION_SLIDE_DOWN);
                        pending_steps = 0;
                        last_step_tick = now;
                    }
                }
            }
        }
    }

    /* ---- Switch (active-low, pulled up) ---- */
    {
        bool raw = (HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == GPIO_PIN_SET);

        if (raw != sw_last_raw) {
            sw_last_raw    = raw;
            sw_change_tick = now;
        }

        if ((now - sw_change_tick) >= SW_DEBOUNCE_MS) {
            if (raw != sw_stable) {
                bool was_held = sw_held;
                sw_stable = raw;

                if (!sw_stable) {
                    /* Button just pressed down */
                    sw_held = true;
                    sw_held_since = now;
                    sw_long_consumed = false;

                    /* Immediate press for settings (fast OK confirm) */
                    if (in_settings) {
                        Settings_OnPress();
                        Screen_MarkDirty();
                    }
                } else {
                    /* Button just released */
                    if (was_held && !sw_long_consumed) {
                        if (!in_settings) {
                            sw_pressed_flag = true;
                        }
                    }
                    sw_held = false;
                }
            }
        }

        /* ---- Long press detection (while held) ---- */
        if (sw_held && !sw_long_consumed) {
            if ((now - sw_held_since) >= LONG_PRESS_MS) {
                sw_long_consumed = true;

                bool currently_in_settings = (Settings_GetState() != SETTINGS_STATE_INACTIVE);

                if (currently_in_settings) {
                    Settings_Exit();
                    Screen_ExitSettings();
                } else {
                    Settings_Enter();
                    Screen_EnterSettings();
                }
            }
        }
    }
}

/* ================= ACCESSORS ================= */

bool Rotary_SWPressed(void)
{
    if (sw_pressed_flag) {
        sw_pressed_flag = false;
        return true;
    }
    return false;
}
