#include "rotary.h"
#include "main.h"
#include "screens.h"
#include "settings.h"
#include "timer_app.h"

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
#define TIMER_HOLD_MS       1000u
#define LONG_PRESS_MS       2000u

/* ================= INTERACTIVE SCREEN TRACKING ================= */
static int stopwatch_screen_index = -1;
static int countdown_screen_index = -1;

void Rotary_SetStopwatchScreenIndex(int idx) { stopwatch_screen_index = idx; }
void Rotary_SetCountdownScreenIndex(int idx) { countdown_screen_index = idx; }

static bool on_stopwatch_screen(void)
{
    return (Screen_GetCurrent() == stopwatch_screen_index && stopwatch_screen_index >= 0);
}

static bool on_countdown_screen(void)
{
    return (Screen_GetCurrent() == countdown_screen_index && countdown_screen_index >= 0);
}

static bool on_timer_screen(void)
{
    return on_stopwatch_screen() || on_countdown_screen();
}

static bool scroll_claimed(void)
{
    if (on_countdown_screen()) {
        return (Countdown_GetState() == CD_STATE_SETTING);
    }
    return false;
}

/* ================= STATE ================= */
static volatile uint8_t  enc_state  = 0;
static volatile int8_t   step_accum = 0;

static int8_t   pending_steps = 0;
static uint32_t last_step_tick = 0;

static bool     sw_last_raw    = true;
static bool     sw_stable      = true;
static uint32_t sw_change_tick = 0;
static bool     sw_pressed_flag = false;

static bool     sw_held          = false;
static uint32_t sw_held_since    = 0;
static bool     sw_hold_consumed = false;

static uint8_t  bar_height = 0;

/* Helper: consume hold and clear bar in one place */
static void consume_hold(void)
{
    sw_hold_consumed = true;
    bar_height = 0;
    Screen_MarkDirty();
}

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
    sw_hold_consumed = false;
    last_step_tick  = HAL_GetTick();
    step_accum      = 0;
    pending_steps   = 0;
    bar_height      = 0;

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

    /* ---- Countdown alarm: force-jump to countdown screen ---- */
    if (countdown_screen_index >= 0 &&
        Countdown_GetState() == CD_STATE_FINISHED &&
        !in_settings) {
        if (Screen_GetCurrent() != countdown_screen_index) {
            Screen_SetCurrent(countdown_screen_index);
            Screen_MarkDirty();
        }
    }

    /* ---- Press bar: update hold height ---- */
    if (sw_held && !sw_hold_consumed) {
        uint32_t held_ms = now - sw_held_since;
        uint32_t fill_ms = on_timer_screen() ? TIMER_HOLD_MS : LONG_PRESS_MS;
        uint8_t h = (uint8_t)(held_ms * 8 / fill_ms);
        if (h > 7) h = 7;

        if (h != bar_height) {
            bar_height = h;
            Screen_MarkDirty();
        }
    } else if (bar_height != 0) {
        bar_height = 0;
        Screen_MarkDirty();
    }

    /* ---- Encoder ---- */
    {
        __disable_irq();
        int8_t new_steps = step_accum;
        step_accum = 0;
        __enable_irq();

        pending_steps += new_steps;

        if ((now - last_step_tick) >= ENCODER_DEBOUNCE_MS)
        {
            if (in_settings) {
                while (pending_steps >= DETENT_THRESHOLD) {
                    Settings_OnScroll(-1);
                    Screen_MarkDirty();
                    pending_steps -= DETENT_THRESHOLD;
                    last_step_tick = now;
                }
                while (pending_steps <= -DETENT_THRESHOLD) {
                    Settings_OnScroll(1);
                    Screen_MarkDirty();
                    pending_steps += DETENT_THRESHOLD;
                    last_step_tick = now;
                }
            } else if (scroll_claimed()) {
                while (pending_steps >= DETENT_THRESHOLD) {
                    Countdown_OnScroll(-1);
                    Screen_MarkDirty();
                    pending_steps -= DETENT_THRESHOLD;
                    last_step_tick = now;
                }
                while (pending_steps <= -DETENT_THRESHOLD) {
                    Countdown_OnScroll(1);
                    Screen_MarkDirty();
                    pending_steps += DETENT_THRESHOLD;
                    last_step_tick = now;
                }
            } else {
                if (!Screen_IsTransitioning()) {
                    if (pending_steps >= DETENT_THRESHOLD) {
                        Screen_Next(TRANSITION_SLIDE_UP);
                        pending_steps = 0;
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
                    sw_hold_consumed = false;
                    bar_height = 0;

                    if (in_settings && Settings_IsOnOK()) {
                        Settings_OnPress();
                        consume_hold();
                    }
                } else {
                    /* Button just released */
                    sw_held = false;
                    bar_height = 0;
                    Screen_MarkDirty();

                    if (was_held && !sw_hold_consumed) {
                        if (in_settings) {
                            Settings_OnPress();
                            Screen_MarkDirty();
                        } else if (on_stopwatch_screen()) {
                            Stopwatch_OnPress();
                            Screen_MarkDirty();
                        } else if (on_countdown_screen()) {
                            Countdown_OnPress();
                            Screen_MarkDirty();
                        } else {
                            sw_pressed_flag = true;
                        }
                    }
                }
            }
        }

        /* ---- Hold detection ---- */
        if (sw_held && !sw_hold_consumed) {
            uint32_t held_ms = now - sw_held_since;

            if (held_ms >= TIMER_HOLD_MS && held_ms < LONG_PRESS_MS) {
                if (!in_settings) {
                    if (on_stopwatch_screen()) {
                        Stopwatch_OnLongPress();
                        consume_hold();
                    } else if (on_countdown_screen()) {
                        CountdownState_t cds = Countdown_GetState();
                        if (cds == CD_STATE_SETTING || cds == CD_STATE_PAUSED ||
                            cds == CD_STATE_RUNNING) {
                            Countdown_OnLongPress();
                            consume_hold();
                        }
                    }
                }
            }
            else if (held_ms >= LONG_PRESS_MS) {
                bool timer_busy = false;
                if (on_stopwatch_screen()) {
                    StopwatchState_t sws = Stopwatch_GetState();
                    timer_busy = (sws == SW_STATE_RUNNING || sws == SW_STATE_PAUSED);
                }
                if (on_countdown_screen()) {
                    CountdownState_t cds = Countdown_GetState();
                    timer_busy = (cds == CD_STATE_SETTING || cds == CD_STATE_RUNNING ||
                                  cds == CD_STATE_PAUSED || cds == CD_STATE_FINISHED);
                }

                if (!timer_busy) {
                    if (in_settings) {
                        Settings_Exit();
                        Screen_ExitSettings();
                    } else {
                        Settings_Enter();
                        Screen_EnterSettings();
                    }
                }
                consume_hold();
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

uint8_t Rotary_GetBarHeight(void)
{
    return bar_height;
}
