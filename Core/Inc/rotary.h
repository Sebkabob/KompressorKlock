#ifndef ROTARY_H
#define ROTARY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= PUBLIC API ================= */

void Rotary_Init(void);
void Rotary_Update(void);
bool Rotary_SWPressed(void);
void Rotary_EXTI_Handler(void);

/* ================= INTERACTIVE SCREEN REGISTRATION ================= */

void Rotary_SetStopwatchScreenIndex(int idx);
void Rotary_SetCountdownScreenIndex(int idx);

/* ================= PRESS INDICATOR BAR ================= */

/**
 * @brief Get how many pixels (0-7) the hold bar should show.
 *        0 = nothing, 1-7 = bar height from bottom.
 *        Fills over the hold duration relevant to the current context:
 *          - On stopwatch/countdown screens: 1 second (timer reset hold)
 *          - Elsewhere: 2 seconds (settings entry hold)
 *        Only shown during holds, not on short presses.
 */
uint8_t Rotary_GetBarHeight(void);

#endif // ROTARY_H
