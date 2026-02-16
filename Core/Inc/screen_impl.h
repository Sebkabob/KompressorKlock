#ifndef SCREEN_IMPL_H
#define SCREEN_IMPL_H

#include "matrix.h"
#include <stdint.h>

/* ================= SCREEN IMPLEMENTATIONS ================= */
/*
 * Each screen function receives a zeroed buffer and draws content
 * using Matrix_*_Buf functions. The screen manager calls these
 * functions each frame, so content can be dynamic.
 */

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Logo2(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Time(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeDate(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_ScrollMessage(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

#endif // SCREEN_IMPL_H
