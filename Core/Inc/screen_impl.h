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

/* ================= BATTERY SCREEN TOGGLE ================= */

void Screen_Battery_Toggle(void);
bool Screen_Battery_IsAlt(void);

/* ================= CONWAY SPEED CONTROL ================= */

void Screen_Conway_CycleSpeed(void);

#endif // SCREEN_IMPL_H
