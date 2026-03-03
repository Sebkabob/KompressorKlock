#include "screen_impl.h"
#include "sensor_manager.h"
#include "settings.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "timer_app.h"
#include "main.h"
#include "calorie_app.h"
#include <stdio.h>
#include <string.h>

/* ================= HELPER: get display hour ================= */
static uint8_t get_display_hour(const SensorData_t *data)
{
    return Settings_Is24Hour() ? data->hours_24 : data->hours_12;
}

/* ================= HELPER: get display temp ================= */
static int get_display_temp(const SensorData_t *data)
{
    return Settings_IsCelsius() ? data->temp_c : data->temp_f;
}

static char get_temp_unit_char(void)
{
    return Settings_IsCelsius() ? 'C' : 'F';
}

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
    sprintf(str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);
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

    if (Settings_Is24Hour()) {
        sprintf(time_str, "%02d:%02d", data->hours_24, data->minutes);
    } else {
        char am_pm = (data->hours_24 < 12) ? 'a' : 'p';
        sprintf(time_str, "%02d:%02d%c", data->hours_12, data->minutes, am_pm);
    }

    uint8_t weekday = RV3032_GetWeekday();
    uint8_t date = RV3032_GetDate();
    sprintf(date_str, "%s %d%s", weekdays_short[weekday], date, day_suffix[date]);

    Matrix_DrawText_Buf(buf, 0, 0, time_str);
    Matrix_DrawTextRight_Buf(buf, 0, date_str);
}

void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();

    #define ICON_W   15
    #define ICON_GAP  2

    uint8_t soc = (uint8_t)data->soc_percent;
    bool charging = (data->current_mA >= 0);

    /* Calculate filled bars (same logic as DrawBatteryIcon) */
    uint8_t filled_bars = 0;
    if (soc >= 5) {
        filled_bars = (uint8_t)((soc - 5) / 10) + 1;
        if (filled_bars > 10) filled_bars = 10;
    }

    /* While charging, blink the next unfilled bar */
    uint8_t blink_bar = 0xFF;  /* 0xFF = no blink */
    if (charging && filled_bars < 10) {
        bool blink_on = ((HAL_GetTick() / 500) % 2) == 0;
        if (blink_on) {
            blink_bar = filled_bars;  /* 0-indexed, the next bar */
        }
    }

    if (charging) {
        char str[4] = { '\x02', '\0' };
        int total_w = ICON_W + ICON_GAP + Matrix_TextPixelWidth(str);
        int start = (NUM_COLS - total_w) / 2;

        Matrix_DrawBatteryIcon_Blink_Buf(buf, start, soc, blink_bar);
        Matrix_DrawText_Buf(buf, 0, start + ICON_W + ICON_GAP, str);
    } else {
        char str[16];
        int days = (soc * 12 + 50) / 100;
        sprintf(str, "%dd left", days);

        int total_w = ICON_W + ICON_GAP + Matrix_TextPixelWidth(str);
        int start = (NUM_COLS - total_w) / 2;

        Matrix_DrawBatteryIcon_Buf(buf, start, soc);
        Matrix_DrawText_Buf(buf, 0, start + ICON_W + ICON_GAP, str);
    }
}

void Screen_Battery2(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str[32];
    sprintf(str, "%3dmA %4dmV %2d%%", data->current_mA, data->voltage_mV, data->soc_percent);
    Matrix_DrawText_Buf(buf, 0, 0, str);
}

void Screen_TempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str[32];
    sprintf(str, "  %2d%c   %2d\x01%%", get_display_temp(data), get_temp_unit_char(), data->humidity);
    Matrix_DrawText_Buf(buf, 0, 0, str);
}

void Screen_TimeTempHumid(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32], str2[32];

    sprintf(str1, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d\x03 %2d\x01", get_display_temp(data), data->humidity);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

void Screen_TimeLight(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32], str2[32];

    sprintf(str1, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d%%", data->light_percent);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

void Screen_TimeTempBatt(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char str1[32], str2[32];

    sprintf(str1, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, str1);

    sprintf(str2, "%2d\x03 %2d%%", get_display_temp(data), data->soc_percent);
    Matrix_DrawTextRight_Buf(buf, 0, str2);
}

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
        if (data->hours_24 < 12) greeting = "morning";
        else if (data->hours_24 < 18) greeting = "afternoon";
        else greeting = "evening";

        int temp = get_display_temp(data);
        char unit = get_temp_unit_char();

        sprintf(message,
            "Today is %s, %s %d%s, %d. It is a lovely %s. "
            "The time is %02d:%02d:%02d. Battery: %d%% Temp: %d%c. Have a great day!",
            weekdays[RV3032_GetWeekday()],
            months[RV3032_GetMonth() - 1],
            RV3032_GetDate(), day_suffix[RV3032_GetDate()],
            RV3032_GetYear(), greeting,
            get_display_hour(data), data->minutes, data->seconds,
            data->soc_percent, temp, unit
        );
        last_update = now;
    }

    Matrix_ScrollText_Buf(buf, 0, message, &scroll_offset, 1, &scroll_tick);
}

/* =====================================================================
 *  STOPWATCH SCREEN
 * =====================================================================*/

void Screen_Stopwatch(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];

    uint32_t total_ms = Stopwatch_GetTotalMs();
    StopwatchState_t state = Stopwatch_GetState();

    uint32_t total_secs = total_ms / 1000;
    uint8_t  cs    = (uint8_t)((total_ms / 10) % 100);
    uint8_t  secs  = (uint8_t)(total_secs % 60);
    uint8_t  mins  = (uint8_t)((total_secs / 60) % 60);
    uint16_t hours = (uint16_t)(total_secs / 3600);

    char time_part[24];
    if (hours > 0) {
        sprintf(time_part, "%d:%02d:%02d.%02d", (int)hours, mins, secs, cs);
    } else if (mins > 0) {
        sprintf(time_part, "%d:%02d.%02d", mins, secs, cs);
    } else {
        sprintf(time_part, "%02d.%02d", secs, cs);
    }

    if (state == SW_STATE_RUNNING) {
        sprintf(str, "> %s", time_part);
    } else if (state == SW_STATE_PAUSED) {
        sprintf(str, " \x04 %s", time_part);
    } else {
        sprintf(str, "%s", time_part);
    }

    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

/* =====================================================================
 *  COUNTDOWN SCREEN
 * =====================================================================*/

void Screen_Countdown(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];
    CountdownState_t state = Countdown_GetState();

    uint8_t h = Countdown_GetHours();
    uint8_t m = Countdown_GetMinutes();
    uint8_t s = Countdown_GetSeconds();

    /* Build compact time string (no leading zero groups) */
    char time_part[16];
    if (h > 0) {
        sprintf(time_part, "%d:%02d:%02d", h, m, s);
    } else if (m > 0) {
        sprintf(time_part, "%d:%02d", m, s);
    } else {
        sprintf(time_part, "0:%02d", s);
    }

    if (state == CD_STATE_IDLE) {
        sprintf(str, "%s", time_part);
        Matrix_DrawTextCentered_Buf(buf, 0, str);

    } else if (state == CD_STATE_SETTING) {
        CountdownField_t field = Countdown_GetField();
        bool blink_on = ((HAL_GetTick() / 500) % 2) == 0;

        char h_str[4], m_str[4], s_str[4];

        if (field == CD_FIELD_HOURS && !blink_on)
            sprintf(h_str, "__");
        else
            sprintf(h_str, "%02d", h);

        if (field == CD_FIELD_MINUTES && !blink_on)
            sprintf(m_str, "__");
        else
            sprintf(m_str, "%02d", m);

        if (field == CD_FIELD_SECONDS && !blink_on)
            sprintf(s_str, "__");
        else
            sprintf(s_str, "%02d", s);

        sprintf(str, "%s:%s:%s", h_str, m_str, s_str);
        Matrix_DrawTextCentered_Buf(buf, 0, str);

    } else if (state == CD_STATE_FINISHED) {
        bool flash_on = ((HAL_GetTick() / 300) % 2) == 0;
        if (flash_on) {
            Matrix_DrawTextCentered_Buf(buf, 0, "0:00");
        }

    } else {
        /* RUNNING or PAUSED */
        const char *icon = (state == CD_STATE_RUNNING) ? ">" : "\x04";
        sprintf(str, "%s %s", icon, time_part);
        Matrix_DrawTextCentered_Buf(buf, 0, str);
    }
}

/* =====================================================================
 *  CALORIE SCREEN
 * =====================================================================*/
void Screen_Calories(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    char str[32];
    uint16_t count = Calorie_GetCount();

    if (Calorie_GetState() == CAL_STATE_EDITING) {
        sprintf(str, "SET CAL %d", count);
    } else {
        sprintf(str, "CAL %d/1600", count);
    }

    Matrix_DrawTextCentered_Buf(buf, 0, str);
}

void Screen_TimeDateCompact(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    char time_str[16];
    char date_str[16];

    sprintf(time_str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);
    Matrix_DrawText_Buf(buf, 0, 0, time_str);

    uint8_t month = RV3032_GetMonth();
    uint8_t date = RV3032_GetDate();
    uint8_t year = (uint8_t)(RV3032_GetYear() - 2000);

    if (Settings_Is24Hour()) {
        sprintf(date_str, "%d/%d/%02d", date, month, year);
    } else {
        sprintf(date_str, "%d/%d/%02d", month, date, year);
    }

    Matrix_DrawTextRight_Buf(buf, 0, date_str);
}
