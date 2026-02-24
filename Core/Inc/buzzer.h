#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize buzzer on TIM2 CH3 (PC6) for PWM tone output.
 *        Call once at startup after MX_TIM2_Init / MX_GPIO_Init.
 */
void Buzzer_Init(void);

/**
 * @brief Non-blocking update — call from main loop (~5ms).
 *        Drives the current melody pattern step by step.
 */
void Buzzer_Update(void);

/* ================= NOTE DEFINITION ================= */

/**
 * A note is a frequency (Hz) + duration (ms).
 * freq_hz = 0 means silence (rest) for that duration.
 */
typedef struct {
    uint16_t freq_hz;       /* 0 = rest/silence */
    uint16_t duration_ms;
} BuzzerNote_t;

/* ================= COMMON FREQUENCIES ================= */
#define NOTE_REST   0
#define NOTE_C4     262
#define NOTE_D4     294
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_G4     392
#define NOTE_A4     440
#define NOTE_B4     494
#define NOTE_C5     523
#define NOTE_D5     587
#define NOTE_E5     659
#define NOTE_F5     698
#define NOTE_G5     784
#define NOTE_A5     880
#define NOTE_B5     988
#define NOTE_C6     1047

/* ================= PLAY A MELODY ================= */

/**
 * @brief Play a sequence of notes (non-blocking).
 *        The array is NOT copied — it must remain valid until playback ends.
 * @param notes  Pointer to note array
 * @param count  Number of notes in the array
 */
void Buzzer_PlayMelody(const BuzzerNote_t *notes, uint8_t count);

/* ================= PRESET PATTERNS ================= */

/** Short click confirmation (~100ms beep) */
void Buzzer_BeepShort(void);

/** Double beep (beep-pause-beep) */
void Buzzer_BeepDouble(void);

/** Long beep (~500ms) */
void Buzzer_BeepLong(void);

/** Countdown finished alarm — assertive repeating pattern */
void Buzzer_BeepAlarm(void);

/** Immediately silence the buzzer and cancel any pattern */
void Buzzer_Stop(void);

/** Check if a pattern is currently playing */
bool Buzzer_IsPlaying(void);

#endif // BUZZER_H
