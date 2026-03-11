#ifndef SCREEN_IMPL_H
#define SCREEN_IMPL_H

#include "matrix.h"
#include <stdint.h>
#include <stdbool.h>

/* ================= SCREEN IMPLEMENTATIONS ================= */

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Time(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeDate(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_BatteryAlt(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

void Screen_Stopwatch(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Countdown(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

void Screen_TimeDateCompact(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/* ================= NEW SCREENS ================= */

void Screen_WorldClock(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_BigDigit(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_PixelRain(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

void Screen_Conway(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Snake(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Typewriter(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_PongClock(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/* ================= LOW BATTERY WARNING ================= */

/**
 * @brief Render the low-battery warning screen.
 *        Alternates between "Low Battery" (3s) and "Please Plug In" (4s).
 *        Text is centered on the display.
 */
void Screen_LowBatteryWarning(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/* ================= BATTERY SCREEN TOGGLE ================= */

void Screen_Battery_Toggle(void);
bool Screen_Battery_IsAlt(void);

/*
 * Called when the user navigates AWAY from the battery screen.
 * Resets the debug mode so it is not visible when they return.
 */
void Screen_Battery_ResetDebug(void);

/*
 * Record one button press while on the battery screen.
 * After 6 presses within the timeout window the debug overlay is shown
 * instead of the normal battery view.  No new screen slot is registered.
 */
void Screen_Battery_RecordPress(void);

/* ================= CONWAY SPEED CONTROL ================= */

void Screen_Conway_CycleSpeed(void);

#endif // SCREEN_IMPL_H
