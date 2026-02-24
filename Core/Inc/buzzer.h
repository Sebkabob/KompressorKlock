#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize buzzer GPIO (PC6, active-high MOSFET)
 *        Call once at startup after MX_GPIO_Init.
 */
void Buzzer_Init(void);

/**
 * @brief Non-blocking update — call from main loop (~5ms).
 *        Drives the current beep pattern.
 */
void Buzzer_Update(void);

/**
 * @brief Play a short single beep (~100ms)
 */
void Buzzer_BeepShort(void);

/**
 * @brief Play a double beep (beep-pause-beep)
 */
void Buzzer_BeepDouble(void);

/**
 * @brief Play a long beep (~500ms)
 */
void Buzzer_BeepLong(void);

/**
 * @brief Play the countdown-finished alarm (3 rapid beeps)
 */
void Buzzer_BeepAlarm(void);

/**
 * @brief Immediately silence the buzzer and cancel any pattern
 */
void Buzzer_Stop(void);

/**
 * @brief Check if a beep pattern is currently playing
 * @return true if buzzer is active
 */
bool Buzzer_IsPlaying(void);

#endif // BUZZER_H
