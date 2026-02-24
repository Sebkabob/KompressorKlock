#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

/* ================= HARDWARE CONFIG ================= */
/*
 * Buzzer is driven by TIM2 Channel 3 on PC6 (AF2).
 * TIM2 clock = PCLK1 = 16 MHz (no PLL, HSI direct).
 *
 * To produce a tone at freq_hz:
 *   ARR  = (16_000_000 / freq_hz) - 1
 *   CCR3 = ARR / 2                       (50% duty cycle)
 *
 * To silence: stop PWM output (or set CCR3 = 0).
 *
 * MX_TIM2_Init() sets up TIM2 with prescaler=0, ARR=max.
 * HAL_TIM_MspPostInit() configures PC6 as TIM2_CH3 AF.
 * We reconfigure ARR/CCR3 dynamically for each note.
 */

extern TIM_HandleTypeDef htim2;

#define TIMER_CLOCK_HZ  16000000UL
#define MIN_FREQ_HZ     100
#define MAX_FREQ_HZ     10000

/* ================= MELODY SEQUENCER ================= */

#define MAX_MELODY_LEN  16

static BuzzerNote_t melody[MAX_MELODY_LEN];
static uint8_t  melody_len    = 0;
static uint8_t  melody_index  = 0;
static uint32_t note_start    = 0;
static bool     playing       = false;

/* ================= LOW-LEVEL TONE ================= */

static void tone_on(uint16_t freq_hz)
{
    if (freq_hz < MIN_FREQ_HZ || freq_hz > MAX_FREQ_HZ) {
        /* Out of range — treat as silence */
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        return;
    }

    uint32_t arr = (TIMER_CLOCK_HZ / (uint32_t)freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, arr / 2);  /* 50% duty */

    /* Reset counter to avoid glitch if new ARR < current CNT */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

static void tone_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

/* ================= PUBLIC API ================= */

void Buzzer_Init(void)
{
    /*
     * PC6 is already configured as TIM2_CH3 AF by HAL_TIM_MspPostInit().
     * Just start the PWM channel with zero duty (silent).
     */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    tone_off();
    playing = false;
}

void Buzzer_Update(void)
{
    if (!playing) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - note_start;

    if (elapsed >= melody[melody_index].duration_ms) {
        /* Advance to next note */
        melody_index++;

        if (melody_index >= melody_len) {
            /* Melody complete */
            tone_off();
            playing = false;
            return;
        }

        note_start = now;

        /* Start the new note */
        if (melody[melody_index].freq_hz == NOTE_REST) {
            tone_off();
        } else {
            tone_on(melody[melody_index].freq_hz);
        }
    }
}

void Buzzer_PlayMelody(const BuzzerNote_t *notes, uint8_t count)
{
    if (count == 0 || count > MAX_MELODY_LEN || notes == NULL) return;

    /* Copy into local buffer so caller doesn't need to keep it alive */
    for (uint8_t i = 0; i < count; i++) {
        melody[i] = notes[i];
    }

    melody_len   = count;
    melody_index = 0;
    note_start   = HAL_GetTick();
    playing      = true;

    /* Start first note */
    if (melody[0].freq_hz == NOTE_REST) {
        tone_off();
    } else {
        tone_on(melody[0].freq_hz);
    }
}

/* ================= PRESET PATTERNS ================= */

void Buzzer_BeepShort(void)
{
    static const BuzzerNote_t pat[] = {
        { NOTE_C5, 80 },
    };
    Buzzer_PlayMelody(pat, 1);
}

void Buzzer_BeepDouble(void)
{
    static const BuzzerNote_t pat[] = {
        { NOTE_C5,    80 },
        { NOTE_REST,  60 },
        { NOTE_E5,    80 },
    };
    Buzzer_PlayMelody(pat, 3);
}

void Buzzer_BeepLong(void)
{
    static const BuzzerNote_t pat[] = {
        { NOTE_A4, 500 },
    };
    Buzzer_PlayMelody(pat, 1);
}

void Buzzer_BeepAlarm(void)
{
    /*
     * Assertive alarm: descending urgent tone pattern.
     * Three quick high-pitched beeps followed by a longer low tone.
     */
    static const BuzzerNote_t pat[] = {
        { NOTE_A5,    120 },
        { NOTE_REST,   60 },
        { NOTE_A5,    120 },
        { NOTE_REST,   60 },
        { NOTE_A5,    120 },
        { NOTE_REST,   80 },
        { NOTE_E5,    300 },
    };
    Buzzer_PlayMelody(pat, 7);
}

void Buzzer_Stop(void)
{
    tone_off();
    playing = false;
}

bool Buzzer_IsPlaying(void)
{
    return playing;
}
