#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

/* ================= GPIO CONFIG ================= */
/*
 * PC6 drives an active-high MOSFET controlling the buzzer.
 * HIGH = buzzer on, LOW = buzzer off.
 *
 * We use simple GPIO toggling for the buzzer since we just need
 * on/off control (the buzzer itself oscillates at its resonant freq).
 */
#define BUZZER_PIN      GPIO_PIN_6
#define BUZZER_PORT     GPIOC

/* ================= PATTERN SEQUENCER ================= */
/*
 * A beep pattern is an array of durations (in ms).
 * Even indices = ON time, odd indices = OFF time.
 * Terminated by 0.
 *
 * Example: {100, 80, 100, 0} = beep 100ms, pause 80ms, beep 100ms, done.
 */

#define MAX_PATTERN_LEN  12

static uint16_t pattern[MAX_PATTERN_LEN];
static uint8_t  pattern_index = 0;
static uint8_t  pattern_len   = 0;
static uint32_t step_start    = 0;
static bool     playing       = false;

static void buzzer_on(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

static void buzzer_off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

static void start_pattern(const uint16_t *pat, uint8_t len)
{
    if (len == 0 || len > MAX_PATTERN_LEN) return;

    for (uint8_t i = 0; i < len; i++) {
        pattern[i] = pat[i];
    }
    pattern_len   = len;
    pattern_index = 0;
    step_start    = HAL_GetTick();
    playing       = true;

    /* Start first step (always ON) */
    buzzer_on();
}

/* ================= PUBLIC API ================= */

void Buzzer_Init(void)
{
    /* Enable GPIOC clock */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = BUZZER_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

    buzzer_off();
    playing = false;
}

void Buzzer_Update(void)
{
    if (!playing) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - step_start;

    if (elapsed >= pattern[pattern_index]) {
        /* Move to next step */
        pattern_index++;

        if (pattern_index >= pattern_len) {
            /* Pattern complete */
            buzzer_off();
            playing = false;
            return;
        }

        step_start = now;

        /* Even index = ON, odd index = OFF */
        if (pattern_index & 1) {
            buzzer_off();
        } else {
            buzzer_on();
        }
    }
}

void Buzzer_BeepShort(void)
{
    const uint16_t pat[] = {100};
    start_pattern(pat, 1);
}

void Buzzer_BeepDouble(void)
{
    /* beep 100ms, off 80ms, beep 100ms */
    const uint16_t pat[] = {100, 80, 100};
    start_pattern(pat, 3);
}

void Buzzer_BeepLong(void)
{
    const uint16_t pat[] = {500};
    start_pattern(pat, 1);
}

void Buzzer_BeepAlarm(void)
{
    /* 3 rapid beeps: beep-off-beep-off-beep */
    const uint16_t pat[] = {150, 100, 150, 100, 300};
    start_pattern(pat, 5);
}

void Buzzer_Stop(void)
{
    buzzer_off();
    playing = false;
}

bool Buzzer_IsPlaying(void)
{
    return playing;
}
