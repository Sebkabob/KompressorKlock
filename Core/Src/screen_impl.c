#include "screen_impl.h"
#include "sensor_manager.h"
#include "settings.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "timer_app.h"
#include "main.h"
#include "world_clock.h"
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

/* ================= EXISTING SCREEN IMPLEMENTATIONS ================= */

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

    static const char * const weekdays_short[] = {
        "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"
    };
    static const char * const day_suffix[] = {
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

    uint8_t filled_bars = 0;
    if (soc >= 5) {
        filled_bars = (uint8_t)((soc - 5) / 10) + 1;
        if (filled_bars > 10) filled_bars = 10;
    }

    uint8_t blink_bar = 0xFF;
    if (charging && filled_bars < 10) {
        bool blink_on = ((HAL_GetTick() / 500) % 2) == 0;
        if (blink_on) {
            blink_bar = filled_bars;
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
        const char *icon = (state == CD_STATE_RUNNING) ? ">" : "\x04";
        sprintf(str, "%s %s", icon, time_part);
        Matrix_DrawTextCentered_Buf(buf, 0, str);
    }
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

/* =====================================================================
 *  FUZZY TIME
 * =====================================================================*/

void Screen_FuzzyTime(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    static int scroll_offset = 0;
    static uint32_t scroll_tick = 0;
    static char last_phrase[48] = "";

    const SensorData_t *data = SensorManager_GetData();
    uint8_t h12 = data->hours_12;
    uint8_t m   = data->minutes;

    static const char * const hour_names[] = {
        "", "one", "two", "three", "four", "five",
        "six", "seven", "eight", "nine", "ten",
        "eleven", "twelve"
    };

    char phrase[48];

    if (data->hours_24 == 0 && m < 3) {
        sprintf(phrase, "midnight");
    } else if (data->hours_24 == 12 && m < 3) {
        sprintf(phrase, "noon");
    } else if (m < 3) {
        sprintf(phrase, "%s o'clock", hour_names[h12]);
    } else if (m < 8) {
        sprintf(phrase, "just after %s", hour_names[h12]);
    } else if (m < 13) {
        sprintf(phrase, "ten past %s", hour_names[h12]);
    } else if (m < 18) {
        sprintf(phrase, "quarter past %s", hour_names[h12]);
    } else if (m < 23) {
        sprintf(phrase, "twenty past %s", hour_names[h12]);
    } else if (m < 28) {
        sprintf(phrase, "almost half past %s", hour_names[h12]);
    } else if (m < 33) {
        sprintf(phrase, "half past %s", hour_names[h12]);
    } else if (m < 38) {
        uint8_t next_h = (h12 % 12) + 1;
        sprintf(phrase, "twenty five to %s", hour_names[next_h]);
    } else if (m < 43) {
        uint8_t next_h = (h12 % 12) + 1;
        sprintf(phrase, "twenty to %s", hour_names[next_h]);
    } else if (m < 48) {
        uint8_t next_h = (h12 % 12) + 1;
        sprintf(phrase, "quarter to %s", hour_names[next_h]);
    } else if (m < 53) {
        uint8_t next_h = (h12 % 12) + 1;
        sprintf(phrase, "ten to %s", hour_names[next_h]);
    } else {
        uint8_t next_h = (h12 % 12) + 1;
        sprintf(phrase, "almost %s", hour_names[next_h]);
    }

    if (strcmp(phrase, last_phrase) != 0) {
        strcpy(last_phrase, phrase);
        scroll_offset = 0;
        scroll_tick = HAL_GetTick();
    }

    int text_w = Matrix_TextPixelWidth(phrase);
    if (text_w <= NUM_COLS) {
        Matrix_DrawTextCentered_Buf(buf, 0, phrase);
    } else {
        Matrix_ScrollText_Buf(buf, 0, phrase, &scroll_offset, 80, &scroll_tick);
    }
}

/* =====================================================================
 *  WORLD CLOCK — uses tiny 3x5 font for city labels
 *
 *  Layout (84 columns):
 *    [tiny_label0] [space] [HH:MM]  gap  [tiny_label1] [space] [HH:MM]
 *
 *  Tiny label: 3 chars × 4px = 11px.  Time: 5 chars × 6px = 29px.
 *  One zone block = 11 + 1 + 29 = 41px.  Two blocks + 2px gap = 84px.
 *
 *  The tiny label is drawn at row 1 (vertically centered in 7 rows
 *  for a 5-row-tall glyph). The normal time is drawn at row 0.
 * =====================================================================*/

void Screen_WorldClock(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    WorldClockState_t state = WorldClock_GetState();

    uint8_t h0, m0, h1, m1;
    WorldClock_GetTime(0, &h0, &m0);
    WorldClock_GetTime(1, &h1, &m1);

    const char *label0 = WorldClock_GetLabel(0);
    const char *label1 = WorldClock_GetLabel(1);

    bool blink_on = WorldClock_IsBlinkOn();

    /* --- Left timezone --- */
    /* Tiny label at col 0, row 1 */
    if (state == WC_STATE_EDITING_TZ1 && !blink_on) {
        Matrix_DrawTinyText_Buf(buf, 1, 0, "___");
    } else {
        Matrix_DrawTinyText_Buf(buf, 1, 0, label0);
    }

    /* Normal-font time right after the label */
    {
        char t0[8];
        sprintf(t0, "%02d:%02d", h0, m0);
        Matrix_DrawText_Buf(buf, 0, 13, t0);
    }

    /* --- Right timezone --- */
    /* Mirror layout from right side: time ends at col 83, label before it */
    {
        char t1[8];
        sprintf(t1, "%02d:%02d", h1, m1);
        int time_w = Matrix_TextPixelWidth(t1);
        int time_x = NUM_COLS - time_w;
        Matrix_DrawText_Buf(buf, 0, time_x, t1);

        /* Tiny label just to the left of the time, with 1px gap */
        int label_x = time_x - Matrix_TinyTextPixelWidth(label1) - 2;
        if (state == WC_STATE_EDITING_TZ2 && !blink_on) {
            Matrix_DrawTinyText_Buf(buf, 1, label_x, "___");
        } else {
            Matrix_DrawTinyText_Buf(buf, 1, label_x, label1);
        }
    }
}

/* =====================================================================
 *  BIG DIGIT CLOCK
 * =====================================================================*/

static void draw_big_char(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int col, char c)
{
    uint8_t tmp[NUM_ROWS][TOTAL_BYTES];
    memset(tmp, 0, sizeof(tmp));
    Matrix_DrawChar_Buf(tmp, 0, 0, c);

    int max_col = 0;
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int cc = 0; cc < 8; cc++) {
            if (tmp[r][0] & (1 << cc)) {
                if (cc > max_col) max_col = cc;
            }
        }
    }

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int cc = 0; cc <= max_col; cc++) {
            if (tmp[r][cc / 8] & (1 << (cc % 8))) {
                int dst1 = col + cc * 2;
                int dst2 = col + cc * 2 + 1;
                if (dst1 >= 0 && dst1 < NUM_COLS)
                    buf[r][dst1 / 8] |= (1 << (dst1 % 8));
                if (dst2 >= 0 && dst2 < NUM_COLS)
                    buf[r][dst2 / 8] |= (1 << (dst2 % 8));
            }
        }
    }
}

static int big_char_width(char c)
{
    uint8_t tmp[NUM_ROWS][TOTAL_BYTES];
    memset(tmp, 0, sizeof(tmp));
    Matrix_DrawChar_Buf(tmp, 0, 0, c);

    int max_col = -1;
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int cc = 0; cc < 8; cc++) {
            if (tmp[r][0] & (1 << cc)) {
                if (cc > max_col) max_col = cc;
            }
        }
    }
    if (max_col < 0) return 4;
    return (max_col + 1) * 2 + 2;
}

static void draw_big_colon(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int col, bool visible)
{
    if (!visible) return;
    for (int r = 1; r <= 2; r++) {
        for (int c = col; c < col + 2 && c < NUM_COLS; c++) {
            if (c >= 0) buf[r][c / 8] |= (1 << (c % 8));
        }
    }
    for (int r = 4; r <= 5; r++) {
        for (int c = col; c < col + 2 && c < NUM_COLS; c++) {
            if (c >= 0) buf[r][c / 8] |= (1 << (c % 8));
        }
    }
}

void Screen_BigDigit(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    uint8_t h = get_display_hour(data);
    uint8_t m = data->minutes;

    char d[4];
    d[0] = '0' + (h / 10);
    d[1] = '0' + (h % 10);
    d[2] = '0' + (m / 10);
    d[3] = '0' + (m % 10);

    int w0 = big_char_width(d[0]);
    int w1 = big_char_width(d[1]);
    int colon_w = 4;
    int w2 = big_char_width(d[2]);
    int w3 = big_char_width(d[3]) - 2;

    int total_w = w0 + w1 + colon_w + w2 + w3;
    int x = (NUM_COLS - total_w) / 2;

    draw_big_char(buf, x, d[0]); x += w0;
    draw_big_char(buf, x, d[1]); x += w1;

    bool colon_on = ((HAL_GetTick() / 500) % 2) == 0;
    draw_big_colon(buf, x + 1, colon_on);
    x += colon_w;

    draw_big_char(buf, x, d[2]); x += w2;
    draw_big_char(buf, x, d[3]);
}

/* =====================================================================
 *  PIXEL RAIN — slower and sparser
 *
 *  Changes from original:
 *  - Speed range 2-5 (was 1-3) — everything falls slower
 *  - Only ~1 in 3 columns are active at a time (rest are dormant)
 *  - Longer pause before columns respawn after falling off
 * =====================================================================*/

#define RAIN_COLS  NUM_COLS

typedef struct {
    int8_t  head_row;
    uint8_t speed;
    uint8_t tick_count;
    uint8_t tail_len;
    uint8_t active;      /* 0 = dormant (invisible), 1 = falling */
} RainColumn_t;

static RainColumn_t rain[RAIN_COLS];
static bool rain_inited = false;

static uint16_t rain_rng = 0xBEEF;

static uint8_t rain_rand(void)
{
    rain_rng ^= rain_rng << 7;
    rain_rng ^= rain_rng >> 9;
    rain_rng ^= rain_rng << 8;
    return (uint8_t)(rain_rng & 0xFF);
}

static void rain_reset_column(int c)
{
    /* ~1 in 3 chance of being active; rest stay dormant longer */
    if ((rain_rand() % 3) == 0) {
        rain[c].active = 1;
        rain[c].head_row = -(int8_t)(rain_rand() % 12);
        rain[c].speed = 2 + (rain_rand() % 4);      /* 2-5 (slower) */
        rain[c].tail_len = 2 + (rain_rand() % 3);    /* 2-4 (shorter) */
    } else {
        rain[c].active = 0;
        rain[c].head_row = -(int8_t)(10 + (rain_rand() % 20)); /* long delay */
        rain[c].speed = 3;
        rain[c].tail_len = 0;
    }
    rain[c].tick_count = 0;
}

void Screen_PixelRain(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    if (!rain_inited) {
        rain_rng ^= (uint16_t)HAL_GetTick();
        for (int c = 0; c < RAIN_COLS; c++) {
            rain_reset_column(c);
            /* Extra stagger on init */
            rain[c].head_row -= (int8_t)(rain_rand() % 10);
        }
        rain_inited = true;
    }

    /* Advance columns */
    for (int c = 0; c < RAIN_COLS; c++) {
        rain[c].tick_count++;
        if (rain[c].tick_count >= rain[c].speed) {
            rain[c].tick_count = 0;
            rain[c].head_row++;

            if (rain[c].active) {
                if (rain[c].head_row - (int8_t)rain[c].tail_len >= NUM_ROWS) {
                    rain_reset_column(c);
                }
            } else {
                /* Dormant column: once head_row reaches 0, give it a chance */
                if (rain[c].head_row >= 0) {
                    rain_reset_column(c);
                }
            }
        }
    }

    /* Draw active rain drops */
    for (int c = 0; c < RAIN_COLS; c++) {
        if (!rain[c].active) continue;
        int head = rain[c].head_row;
        int tail = rain[c].tail_len;
        for (int i = 0; i < tail; i++) {
            int r = head - i;
            if (r >= 0 && r < NUM_ROWS) {
                buf[r][c / 8] |= (1 << (c % 8));
            }
        }
    }

    /* Overlay time text */
    const SensorData_t *data = SensorManager_GetData();
    char time_str[16];
    sprintf(time_str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);

    int text_w = Matrix_TextPixelWidth(time_str);
    int text_x = (NUM_COLS - text_w) / 2;

    /* Clear band behind text */
    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = text_x - 2; c < text_x + text_w + 2; c++) {
            if (c >= 0 && c < NUM_COLS) {
                buf[r][c / 8] &= ~(1 << (c % 8));
            }
        }
    }

    Matrix_DrawText_Buf(buf, 0, text_x, time_str);
}
