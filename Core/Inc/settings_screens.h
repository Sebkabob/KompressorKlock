#ifndef SETTINGS_SCREENS_H
#define SETTINGS_SCREENS_H

#include <stdint.h>
#include <stdbool.h>
#include "matrix.h"

/* ================= TIME SET ================= */

typedef enum {
    TIME_FIELD_HOUR = 0,
    TIME_FIELD_MINUTE,
    TIME_FIELD_SECOND,
    TIME_FIELD_AMPM,
    TIME_FIELD_OK,
    TIME_FIELD_COUNT
} TimeField_t;

void TimeSetting_Enter(void);
void TimeSetting_OnScroll(int direction);
bool TimeSetting_OnPress(void);
void TimeSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool TimeSetting_NeedsRedraw(void);

/* ================= DATE SET ================= */

void DateSetting_Enter(void);
void DateSetting_OnScroll(int direction);
bool DateSetting_OnPress(void);
void DateSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool DateSetting_NeedsRedraw(void);

/* ================= BRIGHTNESS ================= */

void BrightnessSetting_Enter(void);
void BrightnessSetting_OnScroll(int direction);
bool BrightnessSetting_OnPress(void);
void BrightnessSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool BrightnessSetting_NeedsRedraw(void);

#endif // SETTINGS_SCREENS_H
