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
void Screen_ScrollMessage(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/**
 * @brief Stopwatch screen — counts up from 00:00:00
 *        Interactive: press=start/pause, scroll=reset when paused
 *        Shows state icon: > (running), || (paused), [] (stopped)
 */
void Screen_Stopwatch(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/**
 * @brief Countdown timer screen — set time then count down
 *        Interactive: scroll=set time/reset, press=start/pause/acknowledge
 *        Flashes and beeps when finished
 */
void Screen_Countdown(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/**
 * @brief Debug screen for brightness tuning
 */
void Screen_LightDebug(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

#endif // SCREEN_IMPL_H
