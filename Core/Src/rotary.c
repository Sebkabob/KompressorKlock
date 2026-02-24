#include "rotary.h"
#include "main.h"
#include "screens.h"

/*
 * ================= INTERRUPT-DRIVEN QUADRATURE DECODING =================
 *
 * Both encoder pins (ROT_A on PA12, ROT_B on PB0) are configured as
 * EXTI inputs with rising+falling edge triggers. Every edge fires an
 * ISR that reads both pins, builds a 4-bit LUT index from
 * (prev_AB << 2 | cur_AB), and accumulates sub-steps.
 *
 * The main-loop Rotary_Update() then checks the accumulated steps
 * and fires screen transitions. This ensures no edges are missed
 * even at high rotation speeds.
 */

/* Lookup table: index = (prev_A << 3 | prev_B << 2 | cur_A << 1 | cur_B)
 *  1 = CW step
 * -1 = CCW step
 *  0 = invalid / no movement */
static const int8_t encoder_lut[16] = {
/*  0000   0001   0010   0011  */
      0,     1,    -1,     0,
/*  0100   0101   0110   0111  */
     -1,     0,     0,     1,
/*  1000   1001   1010   1011  */
      1,     0,     0,    -1,
/*  1100   1101   1110   1111  */
      0,    -1,     1,     0,
};

/* ================= TIMING ================= */
#define ENCODER_DEBOUNCE_MS   10u   /* Minimum ms between accepted detent actions */
#define SW_DEBOUNCE_MS        50u   /* Minimum ms for switch state to be stable   */
#define DETENT_THRESHOLD       1    /* Sub-steps per detent (most encoders = 4)   */

/* ================= STATE ================= */
static volatile uint8_t  enc_state  = 0;   /* Previous AB in bits 1:0 */
static volatile int8_t   step_accum = 0;   /* Accumulated from ISR    */

static uint32_t last_step_tick = 0;

static bool     sw_last_raw    = true;
static bool     sw_stable      = true;
static uint32_t sw_change_tick = 0;
static bool     sw_pressed_flag = false;

/* ================= ISR — called on every edge of ROT_A or ROT_B ================= */

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
    /* Snapshot initial pin states */
    uint8_t a = (HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b = (HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    enc_state = (a << 1) | b;

    sw_last_raw    = (HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == GPIO_PIN_SET);
    sw_stable      = sw_last_raw;
    sw_change_tick = HAL_GetTick();
    sw_pressed_flag = false;
    last_step_tick  = HAL_GetTick();
    step_accum      = 0;

    /*
     * Configure ROT_A (PA12) and ROT_B (PB0) as EXTI rising+falling edge.
     * The GPIO pins are already configured as inputs with pull-ups by MX_GPIO_Init().
     * We just need to re-init with interrupt mode here.
     */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* ROT_A = PA12 -> EXTI12 (handled by EXTI4_15_IRQHandler) */
    GPIO_InitStruct.Pin  = ROT_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ROT_A_GPIO_Port, &GPIO_InitStruct);

    /* ROT_B = PB0 -> EXTI0 (handled by EXTI0_1_IRQHandler) */
    GPIO_InitStruct.Pin  = ROT_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ROT_B_GPIO_Port, &GPIO_InitStruct);

    /* Enable NVIC for both EXTI lines — priority 1 (below TIM3 at 0) */
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* ================= UPDATE (call from main loop ~5ms) ================= */

void Rotary_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- Encoder: consume accumulated steps from ISR ---- */
    {
        /* Atomically read and clear the accumulator */
        __disable_irq();
        int8_t accum = step_accum;
        step_accum = 0;
        __enable_irq();

        if (accum != 0 &&
            (now - last_step_tick) >= ENCODER_DEBOUNCE_MS &&
            !Screen_IsTransitioning())
        {
            if (accum >= DETENT_THRESHOLD) {
                Screen_Next(TRANSITION_SLIDE_UP);
                last_step_tick = now;
            } else if (accum <= -DETENT_THRESHOLD) {
                Screen_Prev(TRANSITION_SLIDE_DOWN);
                last_step_tick = now;
            }
            /* If |accum| < DETENT_THRESHOLD, steps are lost — that's
             * intentional debouncing for partial/noisy detents. */
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
                sw_stable = raw;
                if (!sw_stable) {
                    sw_pressed_flag = true;
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
