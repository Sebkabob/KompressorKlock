#include "rotary.h"
#include "main.h"
#include "screens.h"

/*
 * ================= QUADRATURE DECODING =================
 *
 * A rotary encoder outputs two signals (A and B) that are 90° out of phase.
 * We track the last 2 bits (prev_A, prev_B) and the current 2 bits as a
 * 4-bit state: (prev_A << 3 | prev_B << 2 | cur_A << 1 | cur_B).
 *
 * Only certain transitions are valid for a real rotation. Invalid transitions
 * (caused by noise or bounce) are simply ignored — the encoder state only
 * updates on recognized sequences.
 *
 * Valid CW  transitions: 0b0001, 0b0111, 0b1110, 0b1000
 * Valid CCW transitions: 0b0010, 0b1011, 0b1101, 0b0100
 *
 * These correspond to the Gray code sequence:
 *   CW:  00 -> 01 -> 11 -> 10 -> 00 ...
 *   CCW: 00 -> 10 -> 11 -> 01 -> 00 ...
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
#define ENCODER_DEBOUNCE_MS   20u   /* Minimum ms between accepted encoder steps */
#define SW_DEBOUNCE_MS        50u   /* Minimum ms for switch state to be stable   */

/* ================= STATE ================= */
static uint8_t  enc_state      = 0;   /* Upper 2 bits = prev AB, lower 2 = current AB */
static uint32_t last_step_tick = 0;
static int8_t   step_accum     = 0;   /* Accumulate steps; act on full detent */

static bool     sw_last_raw    = true;   /* true = HIGH (not pressed, pulled up) */
static bool     sw_stable      = true;
static uint32_t sw_change_tick = 0;
static bool     sw_pressed_flag = false;

/* ================= INIT ================= */

void Rotary_Init(void)
{
    /* Snapshot initial pin states so the first Update() doesn't
     * see a false transition from 0,0. */
    uint8_t a = (HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t b = (HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    /* Store as both "previous" and "current" so the LUT sees 0 delta */
    enc_state = (a << 3) | (b << 2) | (a << 1) | b;

    sw_last_raw    = (HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == GPIO_PIN_SET);
    sw_stable      = sw_last_raw;
    sw_change_tick = HAL_GetTick();
    sw_pressed_flag = false;
    last_step_tick  = HAL_GetTick();
    step_accum      = 0;
}

/* ================= UPDATE ================= */

void Rotary_Update(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- Encoder ---- */
    {
        uint8_t a = (HAL_GPIO_ReadPin(ROT_A_GPIO_Port, ROT_A_Pin) == GPIO_PIN_SET) ? 1u : 0u;
        uint8_t b = (HAL_GPIO_ReadPin(ROT_B_GPIO_Port, ROT_B_Pin) == GPIO_PIN_SET) ? 1u : 0u;

        /* Shift previous state into upper bits, put new reading in lower bits */
        uint8_t idx = ((enc_state & 0x0Cu) << 1) | (a << 1) | b;
        /*            ^^^^^^^^^^^^^^^^^^^ keeps prev_A and prev_B             */

        int8_t delta = encoder_lut[idx & 0x0Fu];

        /* Save current AB as next iteration's "previous" */
        enc_state = (a << 3) | (b << 2);

        if (delta != 0) {
            step_accum += delta;

            if ((now - last_step_tick) >= ENCODER_DEBOUNCE_MS
                && (step_accum >= 4 || step_accum <= -4)  // raised from 2 to 4
                && !Screen_IsTransitioning())               // don't stack transitions
            {
                if (step_accum > 0) {
                    Screen_Next(TRANSITION_SLIDE_UP);
                } else {
                    Screen_Prev(TRANSITION_SLIDE_DOWN);
                }
                step_accum     = 0;
                last_step_tick = now;
            }
        } else if ((now - last_step_tick) > 200u) {
            step_accum = 0;
        }
    }

    /* ---- Switch (active-low, pulled up) ---- */
    {
        bool raw = (HAL_GPIO_ReadPin(ROT_SW_GPIO_Port, ROT_SW_Pin) == GPIO_PIN_SET);

        if (raw != sw_last_raw) {
            /* Pin changed — restart the debounce timer */
            sw_last_raw    = raw;
            sw_change_tick = now;
        }

        if ((now - sw_change_tick) >= SW_DEBOUNCE_MS) {
            if (raw != sw_stable) {
                sw_stable = raw;

                /* Falling edge (HIGH->LOW) = button pressed */
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
