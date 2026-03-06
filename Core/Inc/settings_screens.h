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
bool TimeSetting_IsOnOK(void);

/* ================= DATE SET ================= */

void DateSetting_Enter(void);
void DateSetting_OnScroll(int direction);
bool DateSetting_OnPress(void);
void DateSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool DateSetting_NeedsRedraw(void);
bool DateSetting_IsOnOK(void);

/* ================= BRIGHTNESS ================= */

void BrightnessSetting_Enter(void);
void BrightnessSetting_OnScroll(int direction);
bool BrightnessSetting_OnPress(void);
void BrightnessSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool BrightnessSetting_NeedsRedraw(void);
bool BrightnessSetting_IsOnOK(void);

/* ================= TIMEZONE ================= */

void TimezoneSetting_Enter(void);
void TimezoneSetting_OnScroll(int direction);
bool TimezoneSetting_OnPress(void);
void TimezoneSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool TimezoneSetting_NeedsRedraw(void);
bool TimezoneSetting_IsOnOK(void);

#endif // SETTINGS_SCREENS_H
