#include "settings_screens.h"
#include "settings.h"
#include "sensor_manager.h"
#include "rtc_rv3032.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#define BLINK_INTERVAL_MS 500

/* =====================================================================
 *  TIME SET
 *
 *  Left-justified time fields, OK right-justified on same screen.
 *  When editing time fields, OK shows on the right.
 *  When on OK field, OK blinks.
 * =====================================================================*/

static uint8_t ts_hour;
static uint8_t ts_minute;
static uint8_t ts_second;
static bool    ts_is_pm;
static TimeField_t ts_field;

static uint32_t ts_blink_tick;
static bool     ts_blink_on;

void TimeSetting_Enter(void)
{
    const SensorData_t *data = SensorManager_GetData();

    if (Settings_Is24Hour()) {
        ts_hour   = data->hours_24;
        ts_is_pm  = false;
    } else {
        ts_hour   = data->hours_12;
        ts_is_pm  = (data->hours_24 >= 12);
    }
    ts_minute = data->minutes;
    ts_second = data->seconds;

    ts_field = TIME_FIELD_HOUR;
    ts_blink_tick = HAL_GetTick();
    ts_blink_on = true;
}

void TimeSetting_OnScroll(int direction)
{
    switch (ts_field) {
        case TIME_FIELD_HOUR:
            if (Settings_Is24Hour()) {
                if (direction > 0) {
                    ts_hour = (ts_hour >= 23) ? 0 : ts_hour + 1;
                } else {
                    ts_hour = (ts_hour == 0) ? 23 : ts_hour - 1;
                }
            } else {
                if (direction > 0) {
                    ts_hour++;
                    if (ts_hour > 12) ts_hour = 1;
                } else {
                    ts_hour--;
                    if (ts_hour < 1) ts_hour = 12;
                }
            }
            break;

        case TIME_FIELD_MINUTE:
            if (direction > 0) {
                ts_minute = (ts_minute >= 59) ? 0 : ts_minute + 1;
            } else {
                ts_minute = (ts_minute == 0) ? 59 : ts_minute - 1;
            }
            break;

        case TIME_FIELD_SECOND:
            if (direction > 0) {
                ts_second = (ts_second >= 59) ? 0 : ts_second + 1;
            } else {
                ts_second = (ts_second == 0) ? 59 : ts_second - 1;
            }
            break;

        case TIME_FIELD_AMPM:
            ts_is_pm = !ts_is_pm;
            break;

        case TIME_FIELD_OK:
            break;

        default:
            break;
    }

    ts_blink_on = true;
    ts_blink_tick = HAL_GetTick();
}

bool TimeSetting_OnPress(void)
{
    if (ts_field < TIME_FIELD_OK) {
        ts_field++;
        /* In 24hr mode, skip AM/PM field */
        if (Settings_Is24Hour() && ts_field == TIME_FIELD_AMPM) {
            ts_field = TIME_FIELD_OK;
        }
        ts_blink_on = true;
        ts_blink_tick = HAL_GetTick();
        return false;
    }

    /* OK pressed — commit time */
    uint8_t hours_24;
    if (Settings_Is24Hour()) {
        hours_24 = ts_hour;
    } else {
        if (ts_is_pm) {
            hours_24 = (ts_hour == 12) ? 12 : ts_hour + 12;
        } else {
            hours_24 = (ts_hour == 12) ? 0 : ts_hour;
        }
    }

    RV3032_SetTime(ts_second, ts_minute, hours_24,
                   RV3032_GetWeekday(), RV3032_GetDate(),
                   RV3032_GetMonth(), RV3032_GetYear());

    return true;
}

void TimeSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    char hour_str[4], min_str[4], sec_str[4];

    if (ts_field == TIME_FIELD_HOUR && !ts_blink_on) {
        sprintf(hour_str, "__");
    } else {
        sprintf(hour_str, "%02d", ts_hour);
    }

    if (ts_field == TIME_FIELD_MINUTE && !ts_blink_on) {
        sprintf(min_str, "__");
    } else {
        sprintf(min_str, "%02d", ts_minute);
    }

    if (ts_field == TIME_FIELD_SECOND && !ts_blink_on) {
        sprintf(sec_str, "__");
    } else {
        sprintf(sec_str, "%02d", ts_second);
    }

    if (Settings_Is24Hour()) {
        sprintf(str, "%s:%s:%s", hour_str, min_str, sec_str);
    } else {
        char ampm_str[4];
        if (ts_field == TIME_FIELD_AMPM && !ts_blink_on) {
            sprintf(ampm_str, "__");
        } else {
            sprintf(ampm_str, "%s", ts_is_pm ? "pm" : "am");
        }
        sprintf(str, "%s:%s:%s%s", hour_str, min_str, sec_str, ampm_str);
    }

    /* Left-justify the time */
    Matrix_DrawText_Buf(buf, 0, 0, str);

    /* Right-justify OK — blink when selected */
    if (ts_field == TIME_FIELD_OK) {
        if (ts_blink_on) {
            Matrix_DrawTextRight_Buf(buf, 0, "OK");
        }
    } else {
        Matrix_DrawTextRight_Buf(buf, 0, "OK");
    }
}

bool TimeSetting_NeedsRedraw(void)
{
    uint32_t now = HAL_GetTick();
    if (now - ts_blink_tick >= BLINK_INTERVAL_MS) {
        ts_blink_on = !ts_blink_on;
        ts_blink_tick = now;
        return true;
    }
    return false;
}

bool TimeSetting_IsOnOK(void)
{
    return ts_field == TIME_FIELD_OK;
}

/* =====================================================================
 *  DATE SET
 *
 *  No OK screen — auto-returns after the last field (year) is pressed.
 * =====================================================================*/

typedef enum {
    DATE_FIELD_WEEKDAY = 0,
    DATE_FIELD_MONTH,
    DATE_FIELD_DATE,
    DATE_FIELD_YEAR,
    DATE_FIELD_COUNT
} DateField_t;

static uint8_t  ds_weekday;
static uint8_t  ds_month;
static uint8_t  ds_date;
static uint16_t ds_year;
static DateField_t ds_field;

static uint32_t ds_blink_tick;
static bool     ds_blink_on;

static const char *weekday_names_short[] = {
    "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
};

static const char *month_names_short[] = {
    "", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

static uint8_t days_in_month(uint8_t month, uint16_t year)
{
    static const uint8_t dim[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    if (month == 2) {
        bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
        return leap ? 29 : 28;
    }
    if (month >= 1 && month <= 12) return dim[month];
    return 31;
}

static void clamp_date(void)
{
    uint8_t max = days_in_month(ds_month, ds_year);
    if (ds_date > max) ds_date = max;
    if (ds_date < 1) ds_date = 1;
}

void DateSetting_Enter(void)
{
    ds_weekday = RV3032_GetWeekday();
    ds_month   = RV3032_GetMonth();
    ds_date    = RV3032_GetDate();
    ds_year    = RV3032_GetYear();

    ds_field = DATE_FIELD_WEEKDAY;
    ds_blink_tick = HAL_GetTick();
    ds_blink_on = true;
}

void DateSetting_OnScroll(int direction)
{
    switch (ds_field) {
        case DATE_FIELD_WEEKDAY:
            if (direction > 0) {
                ds_weekday = (ds_weekday >= 6) ? 0 : ds_weekday + 1;
            } else {
                ds_weekday = (ds_weekday == 0) ? 6 : ds_weekday - 1;
            }
            break;

        case DATE_FIELD_MONTH:
            if (direction > 0) {
                ds_month = (ds_month >= 12) ? 1 : ds_month + 1;
            } else {
                ds_month = (ds_month <= 1) ? 12 : ds_month - 1;
            }
            clamp_date();
            break;

        case DATE_FIELD_DATE: {
            uint8_t max = days_in_month(ds_month, ds_year);
            if (direction > 0) {
                ds_date = (ds_date >= max) ? 1 : ds_date + 1;
            } else {
                ds_date = (ds_date <= 1) ? max : ds_date - 1;
            }
            break;
        }

        case DATE_FIELD_YEAR:
            if (direction > 0) {
                ds_year = (ds_year >= 2099) ? 2000 : ds_year + 1;
            } else {
                ds_year = (ds_year <= 2000) ? 2099 : ds_year - 1;
            }
            clamp_date();
            break;

        default:
            break;
    }

    ds_blink_on = true;
    ds_blink_tick = HAL_GetTick();
}

bool DateSetting_OnPress(void)
{
    if (ds_field < DATE_FIELD_YEAR) {
        ds_field++;
        ds_blink_on = true;
        ds_blink_tick = HAL_GetTick();
        return false;
    }

    /* Last field pressed — commit and return */
    const SensorData_t *data = SensorManager_GetData();
    RV3032_SetTime(data->seconds, data->minutes, data->hours_24,
                   ds_weekday, ds_date, ds_month, ds_year);

    return true;
}

void DateSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    char wday_str[5], mon_str[5], date_str[4], year_str[6];

    if (ds_field == DATE_FIELD_WEEKDAY && !ds_blink_on) {
        sprintf(wday_str, "___");
    } else {
        sprintf(wday_str, "%s", weekday_names_short[ds_weekday]);
    }

    if (ds_field == DATE_FIELD_MONTH && !ds_blink_on) {
        sprintf(mon_str, "___");
    } else {
        sprintf(mon_str, "%s", month_names_short[ds_month]);
    }

    if (ds_field == DATE_FIELD_DATE && !ds_blink_on) {
        sprintf(date_str, "__");
    } else {
        sprintf(date_str, "%02d", ds_date);
    }

    if (ds_field == DATE_FIELD_YEAR && !ds_blink_on) {
        sprintf(year_str, "____");
    } else {
        sprintf(year_str, "%4d", ds_year);
    }

    sprintf(str, "%s %s %s %s", wday_str, mon_str, date_str, year_str);
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

bool DateSetting_NeedsRedraw(void)
{
    uint32_t now = HAL_GetTick();
    if (now - ds_blink_tick >= BLINK_INTERVAL_MS) {
        ds_blink_on = !ds_blink_on;
        ds_blink_tick = now;
        return true;
    }
    return false;
}

bool DateSetting_IsOnOK(void)
{
    return (ds_field == DATE_FIELD_YEAR);
}

/* =====================================================================
 *  BRIGHTNESS
 *
 *  No OK screen. No blinking. Scroll to adjust, press to confirm.
 * =====================================================================*/

#define BRIGHT_AUTO  0

static int bright_value;

void BrightnessSetting_Enter(void)
{
    if (SensorManager_IsAutoBrightness()) {
        bright_value = BRIGHT_AUTO;
    } else {
        bright_value = SensorManager_GetManualBrightnessPercent();
    }
}

void BrightnessSetting_OnScroll(int direction)
{
    bright_value += direction;
    if (bright_value < 0) bright_value = 100;
    if (bright_value > 100) bright_value = 0;

    /* Live preview */
    if (bright_value == BRIGHT_AUTO) {
        SensorManager_SetAutoBrightness(true);
    } else {
        SensorManager_SetAutoBrightness(false);
        SensorManager_SetManualBrightnessPercent((uint8_t)bright_value);
    }
}

bool BrightnessSetting_OnPress(void)
{
    /* Already applied from live preview */
    return true;
}

void BrightnessSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    if (bright_value == BRIGHT_AUTO) {
        sprintf(str, "Auto");
    } else {
        sprintf(str, "%d%%", bright_value);
    }
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

bool BrightnessSetting_NeedsRedraw(void)
{
    return false;
}

bool BrightnessSetting_IsOnOK(void)
{
    return true;
}
