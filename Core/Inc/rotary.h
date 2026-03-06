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
void Rotary_SetWorldClockScreenIndex(int idx);
void Rotary_SetBatteryScreenIndex(int idx);

/* ================= PRESS INDICATOR BAR ================= */

uint8_t Rotary_GetBarHeight(void);

#endif // ROTARY_H
