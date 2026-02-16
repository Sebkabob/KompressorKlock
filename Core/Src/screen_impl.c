#include "screen_impl.h"
#include "sensor_manager.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* ================= SCREEN IMPLEMENTATIONS ================= */

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(buf, kompressor_logo);
}

void Screen_Logo2(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(buf, buy_a_wd);
}

void Screen_Time(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str[32];

    sprintf(str, "%02d:%02d:%02d", data->hours_12, data->minutes, data->seconds);
    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

void Screen_TimeDate(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char time_str[32];
    char date_str[32];

    const char *weekdays_short[] = {
        "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
    };

    const char *day_suffix[] = {
        "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th",
        "th", "th", "th", "th", "th", "th", "th", "th", "th", "th",
        "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th",
        "th", "st"
    };

    char am_pm = (data->hours_24 < 12) ? 'a' : 'p';

    sprintf(time_str, "%02d:%02d%c", data->hours_12, data->minutes, am_pm);

    uint8_t weekday = RV3032_GetWeekday();
    uint8_t date = RV3032_GetDate();
    sprintf(date_str, "%s %d%s", weekdays_short[weekday], date, day_suffix[date]);

    Matrix_DrawText_Buf(buf, 0, 0, time_str);
    Matrix_DrawTextRight_Buf(buf, 0, date_str);
}

void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str[32];

    if (data->charger_fault_latchoff) {
        sprintf(str, "CHG FAULT!");
    } else if (data->charger_fault_recoverable) {
        sprintf(str, "CHG WARN!");
    } else if (data->current_mA >= 0) {
        sprintf(str, "%2d%% %3dmA \x02", data->soc_percent, data->current_mA);
    } else {
        sprintf(str, "%2d%% %3dmA", data->soc_percent, data->current_mA);
    }

    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str[32];

    sprintf(str, "  %2dF   %2d\x01%%", data->temp_f, data->humidity);
    Matrix_DrawText_Buf(buf, 0, 0, str);
}

void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32];
    char str2[32];

    sprintf(str1, "%02d:%02d:%02d", data->hours_12, data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d\x03 %2d\x01", data->temp_f, data->humidity);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32];
    char str2[32];

    sprintf(str1, "%02d:%02d:%02d", data->hours_12, data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d%%", data->light_percent);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

void Screen_TimeTempBatt(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32];
    char str2[32];

    sprintf(str1, "%02d:%02d:%02d", data->hours_12, data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d\x03 %2d%%", data->temp_f, data->soc_percent);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

/**
 * @brief Debug screen for tuning brightness — page 0 and page 1.
 *
 * Alternates every 2 seconds between two views:
 *
 * Page 0: "R:xx F:xx M:xxx B:xxx"
 *   R = Raw single-sample light %
 *   F = Filtered (trimmed mean) light %
 *   M = Mapped brightness (from breakpoint table)
 *   B = actual Brightness (after smoothing)
 *
 * Page 1: "C0:xxxxx C1:xxxxx"
 *   C0 = Raw channel 0 (visible + IR) from sensor
 *   C1 = Raw channel 1 (IR only) from sensor
 *   These are the actual 16-bit ADC counts — use them to
 *   calibrate LTR_329_MAX_RAW_VALUE and sensor gain.
 */
void Screen_LightDebug(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    static uint32_t last_page_switch = 0;
    static uint8_t page = 0;
    char str[48];

    uint32_t now = HAL_GetTick();
    if (now - last_page_switch >= 2000) {
        page = (page + 1) % 2;
        last_page_switch = now;
    }

    if (page == 0) {
        uint8_t raw = LTR_329_GetLightPercentRaw();
        uint8_t filtered = LTR_329_GetLightPercent();
        uint8_t mapped = SensorManager_GetMappedBrightness();
        uint8_t actual = SensorManager_GetCurrentBrightness();

        sprintf(str, "%02d %02d", raw, filtered);
        Matrix_DrawText_Buf(buf, 0, 0, str);

        sprintf(str, "%03d %03d", mapped, actual);
        Matrix_DrawTextRight_Buf(buf, 0, str);
    } else {
        uint16_t ch0 = LTR_329_GetChannel0();
        uint16_t ch1 = LTR_329_GetChannel1();

        sprintf(str, "%05u %05u", ch0, ch1);
        Matrix_DrawTextCentered_Buf(buf, 0, str);
    }
}

void Screen_ScrollMessage(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    static char message[128];
    static uint32_t last_update = 0;
    static int scroll_offset = 0;
    static uint32_t scroll_tick = 0;

    const SensorData_t *data = SensorManager_GetData();
    uint32_t now = HAL_GetTick();

    if (now - last_update >= 1000) {
        const char *months[] = {
            "January", "February", "March", "April", "May", "June",
            "July", "August", "September", "October", "November", "December"
        };

        const char *weekdays[] = {
            "Sunday", "Monday", "Tuesday", "Wednesday",
            "Thursday", "Friday", "Saturday"
        };

        const char *day_suffix[] = {
            "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th",
            "th", "th", "th", "th", "th", "th", "th", "th", "th", "th",
            "th", "st", "nd", "rd", "th", "th", "th", "th", "th", "th",
            "th", "st"
        };

        const char *greeting;
        if (data->hours_24 < 12) {
            greeting = "morning";
        } else if (data->hours_24 < 18) {
            greeting = "afternoon";
        } else {
            greeting = "evening";
        }

        sprintf(message,
            "Today is %s, %s %d%s, %d. It is a lovely %s. "
            "The time is %02d:%02d:%02d. Battery: %d%% Temp: %dF. Have a great day!",
            weekdays[RV3032_GetWeekday()],
            months[RV3032_GetMonth() - 1],
            RV3032_GetDate(),
            day_suffix[RV3032_GetDate()],
            RV3032_GetYear(),
            greeting,
            data->hours_12, data->minutes, data->seconds,
            data->soc_percent,
            data->temp_f
        );

        last_update = now;
    }

    Matrix_ScrollText_Buf(buf, 0, message, &scroll_offset, 1, &scroll_tick);
}
