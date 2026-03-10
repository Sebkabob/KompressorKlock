#include "screen_impl.h"
#include "sensor_manager.h"
#include "settings.h"
#include "rtc_rv3032.h"
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

/* ================= EXISTING SCREEN IMPLEMENTATIONS ================= */

void Screen_Logo(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    Matrix_DrawBitmap_Buf(buf, kompressor_logo);
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

/* ================= BATTERY SCREEN TOGGLE STATE ================= */

static bool battery_show_alt = false;

void Screen_Battery_Toggle(void)
{
    battery_show_alt = !battery_show_alt;
}

bool Screen_Battery_IsAlt(void)
{
    return battery_show_alt;
}

/* ================= BATTERY SCREEN (NORMAL) ================= */

void Screen_Battery(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    if (battery_show_alt) {
        Screen_BatteryAlt(buf);
        return;
    }

    const SensorData_t *data = SensorManager_GetData();
    uint8_t soc = (uint8_t)data->soc_percent;
    bool charging = (data->current_mA >= 0);

    uint8_t filled_bars = 0;
    if (soc >= 5) {
        filled_bars = (uint8_t)((soc - 5) / 10) + 1;
        if (filled_bars > 10) filled_bars = 10;
    }

    uint8_t blink_bar = 0xFF;
    if (charging && filled_bars < 10 && ((HAL_GetTick() / 500) % 2) == 0)
        blink_bar = filled_bars;

    char soc_str[8];
    sprintf(soc_str, "%d%%", soc);
    int soc_w = Matrix_TextPixelWidth(soc_str);

    #define ICON_W   15
    #define ICON_GAP  2

    int total_w = ICON_W + ICON_GAP + soc_w;
    if (charging) total_w += ICON_GAP + 6;
    int start = (NUM_COLS - total_w) / 2;
    int text_x = start + ICON_W + ICON_GAP;

    if (charging)
        Matrix_DrawBatteryIcon_Blink_Buf(buf, start, soc, blink_bar);
    else
        Matrix_DrawBatteryIcon_Buf(buf, start, soc);

    Matrix_DrawText_Buf(buf, 0, text_x, soc_str);

    if (charging) {
        char bolt[2] = { '\x02', '\0' };
        Matrix_DrawText_Buf(buf, 0, text_x + soc_w + ICON_GAP, bolt);
    }
}

/* ================= BATTERY SCREEN (ALTERNATE — FULL-WIDTH BAR) ================= */

void Screen_BatteryAlt(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    uint8_t soc = (uint8_t)data->soc_percent;
    bool charging = (data->current_mA >= 0);

    int fill_cols = (int)(((uint32_t)soc * NUM_COLS + 50) / 100);
    if (fill_cols > NUM_COLS) fill_cols = NUM_COLS;

    if (charging && fill_cols < NUM_COLS) {
        if (((HAL_GetTick() / 500) % 2) == 0) fill_cols++;
    }

    char soc_str[8];
    sprintf(soc_str, "%d%%", soc);
    Matrix_DrawText_Buf(buf, 0, 1, soc_str);

    if (charging) {
        char bolt[2] = { '\x02', '\0' };
        int bw = Matrix_TextPixelWidth(bolt);
        Matrix_DrawText_Buf(buf, 0, NUM_COLS - bw - 1, bolt);
    }

    int full_bytes = fill_cols / 8;
    int rem_bits = fill_cols % 8;
    uint8_t rem_mask = (uint8_t)((1u << rem_bits) - 1);

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int b = 0; b < full_bytes && b < TOTAL_BYTES; b++)
            buf[r][b] ^= 0xFF;
        if (rem_bits > 0 && full_bytes < TOTAL_BYTES)
            buf[r][full_bytes] ^= rem_mask;
    }
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
 *  WORLD CLOCK — uses tiny 3x5 font for city labels
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

    if (state == WC_STATE_EDITING_TZ1 && !blink_on) {
        Matrix_DrawTinyText_Buf(buf, 1, 0, "___");
    } else {
        Matrix_DrawTinyText_Buf(buf, 1, 0, label0);
    }

    {
        char t0[8];
        sprintf(t0, "%02d:%02d", h0, m0);
        Matrix_DrawText_Buf(buf, 0, 13, t0);
    }

    {
        char t1[8];
        sprintf(t1, "%02d:%02d", h1, m1);
        int time_w = Matrix_TextPixelWidth(t1);
        int time_x = NUM_COLS - time_w;
        Matrix_DrawText_Buf(buf, 0, time_x, t1);

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
 *  PIXEL RAIN
 * =====================================================================*/

#define RAIN_COLS  NUM_COLS

typedef struct {
    int8_t  head_row;
    uint8_t speed;
    uint8_t tick_count;
    uint8_t tail_len;
    uint8_t active;
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
    if ((rain_rand() % 3) == 0) {
        rain[c].active = 1;
        rain[c].head_row = -(int8_t)(rain_rand() % 12);
        rain[c].speed = 2 + (rain_rand() % 4);
        rain[c].tail_len = 2 + (rain_rand() % 3);
    } else {
        rain[c].active = 0;
        rain[c].head_row = -(int8_t)(10 + (rain_rand() % 20));
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
            rain[c].head_row -= (int8_t)(rain_rand() % 10);
        }
        rain_inited = true;
    }

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
                if (rain[c].head_row >= 0) {
                    rain_reset_column(c);
                }
            }
        }
    }

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

    const SensorData_t *data = SensorManager_GetData();
    char time_str[16];
    sprintf(time_str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);

    int text_w = Matrix_TextPixelWidth(time_str);
    int text_x = (NUM_COLS - text_w) / 2;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = text_x - 2; c < text_x + text_w + 2; c++) {
            if (c >= 0 && c < NUM_COLS) {
                buf[r][c / 8] &= ~(1 << (c % 8));
            }
        }
    }

    Matrix_DrawText_Buf(buf, 0, text_x, time_str);
}

/* =====================================================================
 *  CONWAY'S GAME OF LIFE
 *
 *  Full 7x84 toroidal grid. Time overlaid in center.
 *
 *  Anti-stagnation strategy:
 *    - Tracks 3 previous generations to catch period-1/2/3 oscillators
 *    - Monitors population: if it barely changes for 8 gens, inject chaos
 *    - Hard age limit: full reseed after ~12 seconds of same generation
 *    - "Inject" mode: instead of full reseed, sprinkle random cells into
 *      a region to create new activity while preserving existing patterns
 *    - Population floor: reseed if population drops below 10
 *
 *  Click to cycle speed: Slow (250ms) -> Normal (150ms) -> Fast (60ms).
 * =====================================================================*/

#define GOL_ROWS  NUM_ROWS
#define GOL_COLS  NUM_COLS

static uint8_t gol_grid[GOL_ROWS][TOTAL_BYTES];
static uint8_t gol_hist[3][GOL_ROWS][TOTAL_BYTES];  /* 3 previous generations */
static bool    gol_inited = false;
static uint32_t gol_last_tick = 0;
static uint16_t gol_rng = 0xCAFE;

/* Stagnation tracking */
static uint16_t gol_gen_count = 0;       /* generations since last seed/inject */
static int      gol_prev_pop = 0;        /* population last gen */
static uint8_t  gol_stable_pop_count = 0;/* consecutive gens with similar pop */

/* Speed: 0=Slow, 1=Normal, 2=Fast */
static uint8_t gol_speed_idx = 1;
static const uint16_t gol_speed_ms[] = { 500, 150, 100 };
static const char * const gol_speed_names[] = { "Slow", "Norm", "Fast" };

#define GOL_SPEED_COUNT      3
#define GOL_POP_FLOOR        3    /* only reseed if nearly dead */
#define GOL_STABLE_LIMIT    16    /* be patient — let it settle before injecting */
#define GOL_MAX_AGE        200    /* full reseed after ~30s at normal speed */
#define GOL_INJECT_CELLS    12    /* gentle nudge, not a bomb */

/* Brief overlay when speed changes */
static uint32_t gol_speed_show_tick = 0;
#define GOL_SPEED_SHOW_MS  800

void Screen_Conway_CycleSpeed(void)
{
    gol_speed_idx = (gol_speed_idx + 1) % GOL_SPEED_COUNT;
    gol_speed_show_tick = HAL_GetTick();
}

static uint8_t gol_rand8(void)
{
    gol_rng ^= gol_rng << 7;
    gol_rng ^= gol_rng >> 9;
    gol_rng ^= gol_rng << 8;
    return (uint8_t)(gol_rng & 0xFF);
}

static inline uint8_t gol_get(const uint8_t grid[][TOTAL_BYTES], int r, int c)
{
    if (r < 0) r += GOL_ROWS;
    if (r >= GOL_ROWS) r -= GOL_ROWS;
    if (c < 0) c += GOL_COLS;
    if (c >= GOL_COLS) c -= GOL_COLS;
    return (grid[r][c / 8] >> (c % 8)) & 1;
}

static inline void gol_set(uint8_t grid[][TOTAL_BYTES], int r, int c, uint8_t v)
{
    if (v)
        grid[r][c / 8] |= (1 << (c % 8));
    else
        grid[r][c / 8] &= ~(1 << (c % 8));
}

static int gol_population(void)
{
    int count = 0;
    for (int r = 0; r < GOL_ROWS; r++) {
        for (int b = 0; b < TOTAL_BYTES; b++) {
            uint8_t v = gol_grid[r][b];
            while (v) { count++; v &= v - 1; }
        }
    }
    return count;
}

static void gol_seed(void)
{
    gol_rng ^= (uint16_t)HAL_GetTick();
    memset(gol_grid, 0, sizeof(gol_grid));
    for (int r = 0; r < GOL_ROWS; r++) {
        for (int c = 0; c < GOL_COLS; c++) {
            if ((gol_rand8() % 4) == 0) {
                gol_set(gol_grid, r, c, 1);
            }
        }
    }
    memset(gol_hist, 0, sizeof(gol_hist));
    gol_gen_count = 0;
    gol_prev_pop = 0;
    gol_stable_pop_count = 0;
}

/* Inject a small cluster of random cells to nudge the simulation */
static void gol_inject(void)
{
    gol_rng ^= (uint16_t)HAL_GetTick();
    /* Small cluster in a ~10-col band at a random spot */
    int center_c = (int)(gol_rand8() % GOL_COLS);
    for (int i = 0; i < GOL_INJECT_CELLS; i++) {
        int r = gol_rand8() % GOL_ROWS;
        int c = (center_c + (int)(gol_rand8() % 10) - 5 + GOL_COLS) % GOL_COLS;
        gol_set(gol_grid, r, c, 1);
    }
    gol_gen_count = 0;
    gol_stable_pop_count = 0;
}

static bool gol_matches_history(const uint8_t candidate[][TOTAL_BYTES])
{
    /* Check against current grid and 3 previous generations */
    if (memcmp(candidate, gol_grid, sizeof(gol_grid)) == 0) return true;
    for (int h = 0; h < 3; h++) {
        if (memcmp(candidate, gol_hist[h], sizeof(gol_grid)) == 0) return true;
    }
    return false;
}

static void gol_step(void)
{
    uint8_t next[GOL_ROWS][TOTAL_BYTES];
    memset(next, 0, sizeof(next));

    for (int r = 0; r < GOL_ROWS; r++) {
        for (int c = 0; c < GOL_COLS; c++) {
            int neighbors = 0;
            neighbors += gol_get(gol_grid, r-1, c-1);
            neighbors += gol_get(gol_grid, r-1, c  );
            neighbors += gol_get(gol_grid, r-1, c+1);
            neighbors += gol_get(gol_grid, r,   c-1);
            neighbors += gol_get(gol_grid, r,   c+1);
            neighbors += gol_get(gol_grid, r+1, c-1);
            neighbors += gol_get(gol_grid, r+1, c  );
            neighbors += gol_get(gol_grid, r+1, c+1);

            uint8_t alive = gol_get(gol_grid, r, c);
            if (alive) {
                if (neighbors == 2 || neighbors == 3)
                    gol_set(next, r, c, 1);
            } else {
                if (neighbors == 3)
                    gol_set(next, r, c, 1);
            }
        }
    }

    /* Shift history ring: [2] = [1], [1] = [0], [0] = current grid */
    memcpy(gol_hist[2], gol_hist[1], sizeof(gol_grid));
    memcpy(gol_hist[1], gol_hist[0], sizeof(gol_grid));
    memcpy(gol_hist[0], gol_grid, sizeof(gol_grid));

    /* Install next generation */
    memcpy(gol_grid, next, sizeof(next));
    gol_gen_count++;

    /* --- Stagnation detection --- */
    int pop = gol_population();

    /* 1. Dead or nearly dead → full reseed */
    if (pop < GOL_POP_FLOOR) {
        gol_seed();
        return;
    }

    /* 2. Exact match with any of the last 3 generations → oscillator detected */
    if (gol_matches_history(next)) {
        gol_inject();
        return;
    }

    /* 3. Population barely changing → stable still-lifes / oscillators */
    int pop_diff = pop - gol_prev_pop;
    if (pop_diff < 0) pop_diff = -pop_diff;
    if (pop_diff <= 2) {
        gol_stable_pop_count++;
    } else {
        gol_stable_pop_count = 0;
    }
    gol_prev_pop = pop;

    if (gol_stable_pop_count >= GOL_STABLE_LIMIT) {
        gol_inject();
        return;
    }

    /* 4. Hard age limit → full reseed to keep things fresh */
    if (gol_gen_count >= GOL_MAX_AGE) {
        gol_seed();
        return;
    }
}

void Screen_Conway(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    uint32_t now = HAL_GetTick();

    if (!gol_inited) {
        gol_seed();
        gol_last_tick = now;
        gol_inited = true;
    }

    if (now - gol_last_tick >= gol_speed_ms[gol_speed_idx]) {
        gol_step();
        gol_last_tick = now;
    }

    memcpy(buf, gol_grid, sizeof(gol_grid));

    /* Overlay time with cleared border */
    const SensorData_t *data = SensorManager_GetData();
    char time_str[16];
    sprintf(time_str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);

    int text_w = Matrix_TextPixelWidth(time_str);
    int text_x = (NUM_COLS - text_w) / 2;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = text_x - 2; c < text_x + text_w + 2; c++) {
            if (c >= 0 && c < NUM_COLS) {
                buf[r][c / 8] &= ~(1 << (c % 8));
            }
        }
    }

    Matrix_DrawText_Buf(buf, 0, text_x, time_str);

    /* Show speed name briefly after change */
    if (now - gol_speed_show_tick < GOL_SPEED_SHOW_MS) {
        const char *label = gol_speed_names[gol_speed_idx];
        int lw = Matrix_TinyTextPixelWidth(label);
        for (int r = 0; r < 6; r++) {
            for (int c = NUM_COLS - lw - 2; c < NUM_COLS; c++) {
                if (c >= 0 && c < NUM_COLS)
                    buf[r][c / 8] &= ~(1 << (c % 8));
            }
        }
        Matrix_DrawTinyText_Buf(buf, 1, NUM_COLS - lw, label);
    }
}

/* =====================================================================
 *  SNAKE
 *
 *  Autonomous AI snake on full 7x84 grid. Chases food, grows on eat.
 *  Time overlaid in center.
 * =====================================================================*/

#define SNAKE_MAX_LEN    120
#define SNAKE_INIT_LEN   4
#define SNAKE_STEP_MS    60
#define SNAKE_GROW_AMT   3

typedef struct {
    uint8_t r;
    uint8_t c;
} SnakePos_t;

static SnakePos_t snake_body[SNAKE_MAX_LEN];
static int snake_head_idx  = 0;
static int snake_len       = 0;
static int8_t snake_dr     = 0;
static int8_t snake_dc     = 1;
static uint8_t snake_food_r = 0;
static uint8_t snake_food_c = 0;
static int snake_grow       = 0;
static bool snake_inited    = false;
static uint32_t snake_last_tick = 0;
static uint16_t snake_rng   = 0xFACE;

static uint8_t snake_rand8(void)
{
    snake_rng ^= snake_rng << 7;
    snake_rng ^= snake_rng >> 9;
    snake_rng ^= snake_rng << 8;
    return (uint8_t)(snake_rng & 0xFF);
}

static bool snake_occupies(int r, int c)
{
    for (int i = 0; i < snake_len; i++) {
        int idx = (snake_head_idx - i + SNAKE_MAX_LEN) % SNAKE_MAX_LEN;
        if (snake_body[idx].r == r && snake_body[idx].c == c)
            return true;
    }
    return false;
}

static void snake_place_food(void)
{
    for (int attempts = 0; attempts < 200; attempts++) {
        int r = snake_rand8() % GOL_ROWS;
        int c = ((uint16_t)snake_rand8() * 3) % GOL_COLS;
        if (!snake_occupies(r, c)) {
            snake_food_r = (uint8_t)r;
            snake_food_c = (uint8_t)c;
            return;
        }
    }
    for (int r = 0; r < GOL_ROWS; r++) {
        for (int c = 0; c < GOL_COLS; c++) {
            if (!snake_occupies(r, c)) {
                snake_food_r = (uint8_t)r;
                snake_food_c = (uint8_t)c;
                return;
            }
        }
    }
}

static void snake_reset(void)
{
    snake_rng ^= (uint16_t)HAL_GetTick();
    snake_len = SNAKE_INIT_LEN;
    snake_head_idx = SNAKE_INIT_LEN - 1;
    snake_dr = 0;
    snake_dc = 1;
    snake_grow = 0;

    int start_r = 3;
    int start_c = 5;
    for (int i = 0; i < SNAKE_INIT_LEN; i++) {
        int idx = SNAKE_INIT_LEN - 1 - i;
        snake_body[idx].r = (uint8_t)start_r;
        snake_body[idx].c = (uint8_t)(start_c + i);
    }

    snake_place_food();
}

static bool snake_is_safe(int r, int c)
{
    if (r < 0 || r >= GOL_ROWS || c < 0 || c >= GOL_COLS) return false;
    return !snake_occupies(r, c);
}

static void snake_ai_choose_direction(void)
{
    SnakePos_t head = snake_body[snake_head_idx];
    int hr = head.r, hc = head.c;
    int fr = snake_food_r, fc = snake_food_c;

    static const int8_t dirs[4][2] = {{0,1},{0,-1},{-1,0},{1,0}};

    int best_dir = -1;
    int best_dist = 9999;

    for (int d = 0; d < 4; d++) {
        int nr = hr + dirs[d][0];
        int nc = hc + dirs[d][1];

        if (dirs[d][0] == -snake_dr && dirs[d][1] == -snake_dc && snake_len > 1)
            continue;

        if (!snake_is_safe(nr, nc)) continue;

        int dist = (nr - fr) * (nr - fr) + (nc - fc) * (nc - fc);
        if (dist < best_dist) {
            best_dist = dist;
            best_dir = d;
        }
    }

    if (best_dir < 0) {
        for (int d = 0; d < 4; d++) {
            int nr = hr + dirs[d][0];
            int nc = hc + dirs[d][1];
            if (dirs[d][0] == -snake_dr && dirs[d][1] == -snake_dc && snake_len > 1)
                continue;
            if (snake_is_safe(nr, nc)) {
                best_dir = d;
                break;
            }
        }
    }

    if (best_dir >= 0) {
        snake_dr = dirs[best_dir][0];
        snake_dc = dirs[best_dir][1];
    }
}

void Screen_Snake(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    uint32_t now = HAL_GetTick();

    if (!snake_inited) {
        snake_reset();
        snake_last_tick = now;
        snake_inited = true;
    }

    if (now - snake_last_tick >= SNAKE_STEP_MS) {
        snake_last_tick = now;

        snake_ai_choose_direction();

        SnakePos_t head = snake_body[snake_head_idx];
        int nr = head.r + snake_dr;
        int nc = head.c + snake_dc;

        if (nr < 0 || nr >= GOL_ROWS || nc < 0 || nc >= GOL_COLS || snake_occupies(nr, nc)) {
            snake_reset();
        } else {
            snake_head_idx = (snake_head_idx + 1) % SNAKE_MAX_LEN;
            snake_body[snake_head_idx].r = (uint8_t)nr;
            snake_body[snake_head_idx].c = (uint8_t)nc;

            if (snake_grow > 0) {
                snake_len++;
                if (snake_len > SNAKE_MAX_LEN) snake_len = SNAKE_MAX_LEN;
                snake_grow--;
            }

            if (nr == snake_food_r && nc == snake_food_c) {
                snake_grow += SNAKE_GROW_AMT;
                snake_place_food();
            }
        }
    }

    for (int i = 0; i < snake_len; i++) {
        int idx = (snake_head_idx - i + SNAKE_MAX_LEN) % SNAKE_MAX_LEN;
        int r = snake_body[idx].r;
        int c = snake_body[idx].c;
        if (r >= 0 && r < NUM_ROWS && c >= 0 && c < NUM_COLS) {
            buf[r][c / 8] |= (1 << (c % 8));
        }
    }

    if ((now / 200) % 2) {
        int r = snake_food_r;
        int c = snake_food_c;
        if (r >= 0 && r < NUM_ROWS && c >= 0 && c < NUM_COLS) {
            buf[r][c / 8] |= (1 << (c % 8));
        }
    }

    const SensorData_t *data = SensorManager_GetData();
    char time_str[16];
    sprintf(time_str, "%02d:%02d:%02d", get_display_hour(data), data->minutes, data->seconds);

    int text_w = Matrix_TextPixelWidth(time_str);
    int text_x = (NUM_COLS - text_w) / 2;

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = text_x - 2; c < text_x + text_w + 2; c++) {
            if (c >= 0 && c < NUM_COLS) {
                buf[r][c / 8] &= ~(1 << (c % 8));
            }
        }
    }

    Matrix_DrawText_Buf(buf, 0, text_x, time_str);
}

/* =====================================================================
 *  TYPEWRITER CLOCK
 *
 *  Each minute, the time "types" itself out one character at a time
 *  with a blinking cursor. Seconds shown in tiny font bottom-right.
 * =====================================================================*/

#define TW_CHAR_INTERVAL_MS  120
#define TW_CURSOR_BLINK_MS   400

static bool     tw_inited = false;
static uint8_t  tw_last_minute = 0xFF;
static uint8_t  tw_chars_shown = 0;
static uint8_t  tw_total_chars = 0;
static uint32_t tw_last_char_tick = 0;
static char     tw_text[20];

void Screen_Typewriter(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    const SensorData_t *data = SensorManager_GetData();
    uint32_t now = HAL_GetTick();

    uint8_t h = get_display_hour(data);
    uint8_t m = data->minutes;
    uint8_t s = data->seconds;

    if (!tw_inited || m != tw_last_minute) {
        tw_last_minute = m;
        tw_chars_shown = 0;

        if (Settings_Is24Hour()) {
            sprintf(tw_text, "%02d:%02d", h, m);
        } else {
            char ap = (data->hours_24 < 12) ? 'a' : 'p';
            sprintf(tw_text, "%02d:%02d%c", h, m, ap);
        }
        tw_total_chars = (uint8_t)strlen(tw_text);
        tw_last_char_tick = now;
        tw_inited = true;
    }

    if (tw_chars_shown < tw_total_chars) {
        if (now - tw_last_char_tick >= TW_CHAR_INTERVAL_MS) {
            tw_chars_shown++;
            tw_last_char_tick = now;
        }
    }

    char partial[20];
    memcpy(partial, tw_text, tw_chars_shown);
    partial[tw_chars_shown] = '\0';

    int text_w = Matrix_TextPixelWidth(tw_text);
    int start_x = (NUM_COLS - text_w) / 2;

    Matrix_DrawText_Buf(buf, 0, start_x, partial);

    int partial_w = Matrix_TextPixelWidth(partial);
    int cursor_x = start_x + partial_w;

    bool cursor_on;
    if (tw_chars_shown < tw_total_chars) {
        cursor_on = true;
    } else {
        cursor_on = ((now / TW_CURSOR_BLINK_MS) % 2) == 0;
    }

    if (cursor_on && cursor_x >= 0 && cursor_x < NUM_COLS) {
        for (int r = 0; r < NUM_ROWS; r++) {
            buf[r][cursor_x / 8] |= (1 << (cursor_x % 8));
        }
    }

    {
        char sec_str[4];
        sprintf(sec_str, "%02d", s);
        int sw = Matrix_TinyTextPixelWidth(sec_str);
        Matrix_DrawTinyText_Buf(buf, 2, NUM_COLS - sw, sec_str);
    }
}

/* =====================================================================
 *  PONG CLOCK
 *
 *  Two AI paddles play Pong. Left score = hours, right = minutes.
 *  When the real time changes, the appropriate side deliberately
 *  misses so the score updates to match the clock.
 *
 *  Fixed-point ball physics (8.8). Paddles are 3px tall.
 *  Ball serves at a strong diagonal angle and bounces off
 *  top/bottom walls and paddles with english.
 * =====================================================================*/

#define PONG_STEP_MS          35
#define PONG_PADDLE_H          3
#define PONG_LPAD_C            1   /* left paddle column */
#define PONG_RPAD_C           82   /* right paddle column */
#define PONG_RESET_DELAY_MS  600

/* 8.8 fixed-point helpers */
#define FP(x) ((int16_t)((x) * 256))
#define FP_INT(x) ((x) >> 8)

typedef struct {
    int16_t bx, by;         /* ball position  (8.8 fixed) */
    int16_t bdx, bdy;       /* ball velocity  (8.8 fixed) */

    int8_t  lpad, rpad;     /* top row of each paddle (0..NUM_ROWS-PADDLE_H) */

    uint8_t disp_h, disp_m; /* currently displayed score */

    bool     waiting;       /* pause after goal before re-serve */
    uint32_t wait_tick;

    bool     miss_left;     /* left paddle should let ball through */
    bool     miss_right;    /* right paddle should let ball through */

    uint32_t last_tick;
    bool     inited;
} PongState_t;

static PongState_t pg;
static uint16_t pg_rng = 0xD00D;

static int16_t pg_rand_sign(void)
{
    pg_rng ^= pg_rng << 7;
    pg_rng ^= pg_rng >> 9;
    pg_rng ^= pg_rng << 8;
    return (pg_rng & 1) ? 1 : -1;
}

static void pg_serve(bool toward_left)
{
    /* Start from center */
    pg.bx = FP(NUM_COLS / 2);
    pg.by = FP(NUM_ROWS / 2);

    /* Strong diagonal: ~1.0 px/step horizontal, ~0.6 px/step vertical */
    pg.bdx = toward_left ? -256 : 256;
    pg.bdy = pg_rand_sign() * (128 + (pg_rng % 64));  /* 0.5-0.75 px/step vertical */

    pg.waiting = false;
}

static void pg_init(void)
{
    const SensorData_t *d = SensorManager_GetData();
    pg.disp_h = get_display_hour(d);
    pg.disp_m = d->minutes;
    pg.lpad = (NUM_ROWS - PONG_PADDLE_H) / 2;
    pg.rpad = (NUM_ROWS - PONG_PADDLE_H) / 2;
    pg.miss_left = false;
    pg.miss_right = false;
    pg.waiting = false;
    pg.last_tick = HAL_GetTick();
    pg.inited = true;
    pg_rng ^= (uint16_t)HAL_GetTick();
    pg_serve(pg_rand_sign() > 0);
}

/*
 * Move paddle toward target row. speed = max pixels to move per step.
 * If 'miss' is true and ball is heading this way, move AWAY from ball.
 */
static void pg_move_pad(int8_t *pad, int target_r, int speed, bool miss)
{
    int center = *pad + PONG_PADDLE_H / 2;

    if (miss) {
        /* Move opposite to ball to guarantee a miss */
        if (target_r <= center) {
            /* Ball is above or at center — move down */
            for (int i = 0; i < speed; i++) {
                if (*pad < NUM_ROWS - PONG_PADDLE_H) (*pad)++;
            }
        } else {
            /* Ball is below center — move up */
            for (int i = 0; i < speed; i++) {
                if (*pad > 0) (*pad)--;
            }
        }
        return;
    }

    /* Normal tracking: move toward target, up to 'speed' pixels */
    for (int i = 0; i < speed; i++) {
        center = *pad + PONG_PADDLE_H / 2;
        if (center < target_r && *pad < NUM_ROWS - PONG_PADDLE_H) {
            (*pad)++;
        } else if (center > target_r && *pad > 0) {
            (*pad)--;
        }
    }
}

void Screen_PongClock(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    uint32_t now = HAL_GetTick();

    if (!pg.inited) pg_init();

    /* --- Check if real time changed → trigger a miss --- */
    const SensorData_t *data = SensorManager_GetData();
    uint8_t real_h = get_display_hour(data);
    uint8_t real_m = data->minutes;

    if (!pg.waiting) {
        if (real_h != pg.disp_h) {
            /* Need left score to change: ball must score on RIGHT side,
               so RIGHT paddle misses → ball goes past right → left "scores" */
            pg.miss_right = true;
        }
        if (real_m != pg.disp_m) {
            /* Need right score to change: LEFT paddle misses */
            pg.miss_left = true;
        }
    }

    /* --- Physics step --- */
    if (now - pg.last_tick >= PONG_STEP_MS) {
        pg.last_tick = now;

        if (pg.waiting) {
            if (now - pg.wait_tick >= PONG_RESET_DELAY_MS) {
                /* Serve toward the side that still needs to miss, or random */
                bool serve_left = pg.miss_left;
                if (!pg.miss_left && !pg.miss_right)
                    serve_left = (pg_rand_sign() > 0);
                pg_serve(serve_left);
            }
        } else {
            /* Move ball */
            pg.bx += pg.bdx;
            pg.by += pg.bdy;

            int br = FP_INT(pg.by);
            int bc = FP_INT(pg.bx);

            /* Top/bottom wall bounce */
            if (pg.by < 0) {
                pg.by = -pg.by;
                pg.bdy = -pg.bdy;
            }
            if (pg.by > FP(NUM_ROWS - 1)) {
                pg.by = FP(NUM_ROWS - 1) * 2 - pg.by;
                pg.bdy = -pg.bdy;
            }

            br = FP_INT(pg.by);
            bc = FP_INT(pg.bx);

            /* Left paddle check */
            if (bc <= PONG_LPAD_C + 1 && pg.bdx < 0) {
                if (br >= pg.lpad && br < pg.lpad + PONG_PADDLE_H) {
                    /* Paddle hit — bounce */
                    pg.bdx = -pg.bdx;
                    pg.bx = FP(PONG_LPAD_C + 2);

                    /* Add english: top of paddle = upward, bottom = downward */
                    int hit = br - pg.lpad;
                    if (hit == 0) pg.bdy -= 80;
                    else if (hit == PONG_PADDLE_H - 1) pg.bdy += 80;

                    /* Clamp vertical speed to keep things interesting */
                    if (pg.bdy > 220) pg.bdy = 220;
                    if (pg.bdy < -220) pg.bdy = -220;
                    /* Ensure minimum vertical motion */
                    if (pg.bdy > 0 && pg.bdy < 40) pg.bdy = 40;
                    if (pg.bdy < 0 && pg.bdy > -40) pg.bdy = -40;

                } else if (bc <= 0) {
                    /* Ball past left edge → right side (minutes) scores */
                    pg.disp_m = real_m;
                    pg.miss_left = false;
                    /* If hours also need updating and this was a left miss,
                       we'll catch it on next serve */
                    pg.waiting = true;
                    pg.wait_tick = now;
                }
            }

            /* Right paddle check */
            if (bc >= PONG_RPAD_C - 1 && pg.bdx > 0) {
                if (br >= pg.rpad && br < pg.rpad + PONG_PADDLE_H) {
                    pg.bdx = -pg.bdx;
                    pg.bx = FP(PONG_RPAD_C - 2);

                    int hit = br - pg.rpad;
                    if (hit == 0) pg.bdy -= 80;
                    else if (hit == PONG_PADDLE_H - 1) pg.bdy += 80;

                    if (pg.bdy > 220) pg.bdy = 220;
                    if (pg.bdy < -220) pg.bdy = -220;
                    if (pg.bdy > 0 && pg.bdy < 40) pg.bdy = 40;
                    if (pg.bdy < 0 && pg.bdy > -40) pg.bdy = -40;

                } else if (bc >= NUM_COLS - 1) {
                    /* Ball past right edge → left side (hours) scores */
                    pg.disp_h = real_h;
                    pg.miss_right = false;
                    pg.waiting = true;
                    pg.wait_tick = now;
                }
            }

            /* --- Move paddles --- */
            int ball_row = FP_INT(pg.by);

            /* Left paddle: miss if miss_left AND ball heading left */
            bool l_miss = pg.miss_left && (pg.bdx < 0);
            pg_move_pad(&pg.lpad, ball_row, 2, l_miss);

            /* Right paddle: miss if miss_right AND ball heading right */
            bool r_miss = pg.miss_right && (pg.bdx > 0);
            pg_move_pad(&pg.rpad, ball_row, 2, r_miss);
        }
    }

    /* ---- DRAW ---- */

    /* Dotted center line */
    {
        int cc = NUM_COLS / 2;
        for (int r = 0; r < NUM_ROWS; r += 2) {
            buf[r][cc / 8] |= (1 << (cc % 8));
        }
    }

    /* Left paddle */
    for (int i = 0; i < PONG_PADDLE_H; i++) {
        int r = pg.lpad + i;
        if (r >= 0 && r < NUM_ROWS) {
            buf[r][PONG_LPAD_C / 8] |= (1 << (PONG_LPAD_C % 8));
        }
    }

    /* Right paddle */
    for (int i = 0; i < PONG_PADDLE_H; i++) {
        int r = pg.rpad + i;
        if (r >= 0 && r < NUM_ROWS) {
            buf[r][PONG_RPAD_C / 8] |= (1 << (PONG_RPAD_C % 8));
        }
    }

    /* Ball (hidden during reset pause) */
    if (!pg.waiting) {
        int br = FP_INT(pg.by);
        int bc = FP_INT(pg.bx);
        if (br >= 0 && br < NUM_ROWS && bc >= 0 && bc < NUM_COLS) {
            buf[br][bc / 8] |= (1 << (bc % 8));
        }
    }

    /* Scores — hours left, minutes right */
    {
        char h_str[4], m_str[4];
        sprintf(h_str, "%02d", pg.disp_h);
        sprintf(m_str, "%02d", pg.disp_m);

        int lw = Matrix_TextPixelWidth(h_str);
        int lx = (NUM_COLS / 2 - lw) / 2;
        Matrix_DrawText_Buf(buf, 0, lx, h_str);

        int rw = Matrix_TextPixelWidth(m_str);
        int rx = NUM_COLS / 2 + (NUM_COLS / 2 - rw) / 2;
        Matrix_DrawText_Buf(buf, 0, rx, m_str);
    }
}
