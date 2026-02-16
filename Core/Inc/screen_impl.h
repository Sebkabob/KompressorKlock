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
void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeTempBatt(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
void Screen_ScrollMessage(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/**
 * @brief Debug screen for brightness tuning
 *
 * Shows: R:xx (raw light%), F:xx (filtered%), M:xxx (mapped brightness), B:xxx (actual brightness)
 * Register this screen to see live values while tuning breakpoints.
 */
void Screen_LightDebug(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

#endif // SCREEN_IMPL_H
