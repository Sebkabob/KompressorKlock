#include "settings_screens.h"
#include "sensor_manager.h"
#include "rtc_rv3032.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

#define BLINK_INTERVAL_MS 500

/* =====================================================================
 *  TIME SET
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

    ts_hour   = data->hours_12;
    ts_minute = data->minutes;
    ts_second = data->seconds;
    ts_is_pm  = (data->hours_24 >= 12);

    ts_field = TIME_FIELD_HOUR;
    ts_blink_tick = HAL_GetTick();
    ts_blink_on = true;
}

void TimeSetting_OnScroll(int direction)
{
    switch (ts_field) {
        case TIME_FIELD_HOUR:
            if (direction > 0) {
                ts_hour++;
                if (ts_hour > 12) ts_hour = 1;
            } else {
                ts_hour--;
                if (ts_hour < 1) ts_hour = 12;
            }
            break;

        case TIME_FIELD_MINUTE:
            if (direction > 0) {
                ts_minute++;
                if (ts_minute > 59) ts_minute = 0;
            } else {
                if (ts_minute == 0) ts_minute = 59;
                else ts_minute--;
            }
            break;

        case TIME_FIELD_SECOND:
            if (direction > 0) {
                ts_second++;
                if (ts_second > 59) ts_second = 0;
            } else {
                if (ts_second == 0) ts_second = 59;
                else ts_second--;
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
        ts_blink_on = true;
        ts_blink_tick = HAL_GetTick();
        return false;
    }

    uint8_t hours_24;
    if (ts_is_pm) {
        hours_24 = (ts_hour == 12) ? 12 : ts_hour + 12;
    } else {
        hours_24 = (ts_hour == 12) ? 0 : ts_hour;
    }

    RV3032_SetTime(ts_second, ts_minute, hours_24,
                   RV3032_GetWeekday(), RV3032_GetDate(),
                   RV3032_GetMonth(), RV3032_GetYear());

    return true;
}

void TimeSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    if (ts_field == TIME_FIELD_OK) {
        if (ts_blink_on) {
            Matrix_DrawTextCentered_Buf(buf, 0, "OK");
        } else {
            Matrix_DrawTextCentered_Buf(buf, 0, "__");
        }
        return;
    }

    char hour_str[4];
    char min_str[4];
    char sec_str[4];
    char ampm_display[4];

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

    if (ts_field == TIME_FIELD_AMPM && !ts_blink_on) {
        sprintf(ampm_display, "__");
    } else {
        sprintf(ampm_display, "%s", ts_is_pm ? "pm" : "am");
    }

    sprintf(str, "%s:%s:%s%s", hour_str, min_str, sec_str, ampm_display);
    Matrix_DrawTextCentered_Buf(buf, 0, str);
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
 * =====================================================================*/

typedef enum {
    DATE_FIELD_WEEKDAY = 0,
    DATE_FIELD_MONTH,
    DATE_FIELD_DATE,
    DATE_FIELD_YEAR,
    DATE_FIELD_OK,
    DATE_FIELD_COUNT
} DateField_t;

static uint8_t  ds_weekday;
static uint8_t  ds_month;
static uint8_t  ds_date;
static uint16_t ds_year;
static DateField_t ds_field;

static uint32_t ds_blink_tick;
static bool     ds_blink_on;

/* RV3032 weekday: 0=Sunday, 1=Monday, ... 6=Saturday */
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
                ds_weekday++;
                if (ds_weekday > 6) ds_weekday = 0;
            } else {
                if (ds_weekday == 0) ds_weekday = 6;
                else ds_weekday--;
            }
            break;

        case DATE_FIELD_MONTH:
            if (direction > 0) {
                ds_month++;
                if (ds_month > 12) ds_month = 1;
            } else {
                ds_month--;
                if (ds_month < 1) ds_month = 12;
            }
            clamp_date();
            break;

        case DATE_FIELD_DATE: {
            uint8_t max = days_in_month(ds_month, ds_year);
            if (direction > 0) {
                ds_date++;
                if (ds_date > max) ds_date = 1;
            } else {
                if (ds_date <= 1) ds_date = max;
                else ds_date--;
            }
            break;
        }

        case DATE_FIELD_YEAR:
            if (direction > 0) {
                ds_year++;
                if (ds_year > 2099) ds_year = 2000;
            } else {
                if (ds_year <= 2000) ds_year = 2099;
                else ds_year--;
            }
            clamp_date();
            break;

        case DATE_FIELD_OK:
            break;

        default:
            break;
    }

    ds_blink_on = true;
    ds_blink_tick = HAL_GetTick();
}

bool DateSetting_OnPress(void)
{
    if (ds_field < DATE_FIELD_OK) {
        ds_field++;
        ds_blink_on = true;
        ds_blink_tick = HAL_GetTick();
        return false;
    }

    const SensorData_t *data = SensorManager_GetData();
    RV3032_SetTime(data->seconds, data->minutes, data->hours_24,
                   ds_weekday, ds_date, ds_month, ds_year);

    return true;
}

void DateSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    if (ds_field == DATE_FIELD_OK) {
        if (ds_blink_on) {
            Matrix_DrawTextCentered_Buf(buf, 0, "OK");
        } else {
            Matrix_DrawTextCentered_Buf(buf, 0, "__");
        }
        return;
    }

    char wday_str[5];
    char mon_str[5];
    char date_str[4];
    char year_str[6];

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
    return ds_field == DATE_FIELD_OK;
}

/* =====================================================================
 *  BRIGHTNESS
 * =====================================================================*/

typedef enum {
    BRIGHT_FIELD_VALUE = 0,
    BRIGHT_FIELD_OK,
} BrightField_t;

#define BRIGHT_AUTO  0

static int bright_value;
static BrightField_t bright_field;

static uint32_t bright_blink_tick;
static bool     bright_blink_on;

void BrightnessSetting_Enter(void)
{
    if (SensorManager_IsAutoBrightness()) {
        bright_value = BRIGHT_AUTO;
    } else {
        bright_value = SensorManager_GetManualBrightnessPercent();
    }

    bright_field = BRIGHT_FIELD_VALUE;
    bright_blink_tick = HAL_GetTick();
    bright_blink_on = true;
}

void BrightnessSetting_OnScroll(int direction)
{
    if (bright_field == BRIGHT_FIELD_OK) return;

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

    bright_blink_on = true;
    bright_blink_tick = HAL_GetTick();
}

bool BrightnessSetting_OnPress(void)
{
    if (bright_field < BRIGHT_FIELD_OK) {
        bright_field++;
        bright_blink_on = true;
        bright_blink_tick = HAL_GetTick();
        return false;
    }

    /* Already applied from live preview */
    return true;
}

void BrightnessSetting_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    if (bright_field == BRIGHT_FIELD_OK) {
        if (bright_blink_on) {
            Matrix_DrawTextCentered_Buf(buf, 0, "OK");
        } else {
            Matrix_DrawTextCentered_Buf(buf, 0, "__");
        }
        return;
    }

    if (bright_blink_on) {
        if (bright_value == BRIGHT_AUTO) {
            sprintf(str, "Auto");
        } else {
            sprintf(str, "%03d%%", bright_value);
        }
    } else {
        sprintf(str, "____");
    }
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

bool BrightnessSetting_NeedsRedraw(void)
{
    uint32_t now = HAL_GetTick();
    if (now - bright_blink_tick >= BLINK_INTERVAL_MS) {
        bright_blink_on = !bright_blink_on;
        bright_blink_tick = now;
        return true;
    }
    return false;
}

bool BrightnessSetting_IsOnOK(void)
{
    return bright_field == BRIGHT_FIELD_OK;
}
