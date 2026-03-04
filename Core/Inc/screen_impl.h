#ifndef SCREEN_IMPL_H
#define SCREEN_IMPL_H

#include "matrix.h"
#include <stdint.h>

/* ================= SCREEN IMPLEMENTATIONS ================= */

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Logo2(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Time(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeDate(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Battery2(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempBatt(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

void Screen_Stopwatch(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Countdown(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

void Screen_LightDebug(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeDateCompact(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/* ================= NEW SCREENS ================= */

void Screen_FuzzyTime(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_WorldClock(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_BigDigit(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_PixelRain(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

#endif // SCREEN_IMPL_H
