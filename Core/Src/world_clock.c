#include "world_clock.h"
#include "sensor_manager.h"
#include "settings.h"
#include "rtc_rv3032.h"
#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

/* ================= TIMEZONE TABLE ================= */

static const Timezone_t tz_table[] = {
    { "BKR",  -48 },  /* UTC-12:00  Baker Island          */
    { "SST",  -44 },  /* UTC-11:00  American Samoa        */
    { "HNL",  -40 },  /* UTC-10:00  Honolulu              */
    { "ANC",  -36 },  /* UTC-9:00   Anchorage             */
    { "LAX",  -32 },  /* UTC-8:00   Los Angeles           */
    { "DEN",  -28 },  /* UTC-7:00   Denver                */
    { "CHI",  -24 },  /* UTC-6:00   Chicago               */
    { "NYC",  -20 },  /* UTC-5:00   New York              */
    { "CCS",  -16 },  /* UTC-4:00   Caracas               */
    { "GRU",  -12 },  /* UTC-3:00   Sao Paulo             */
    { "GSI",   -8 },  /* UTC-2:00   South Georgia Island  */
    { "CVT",   -4 },  /* UTC-1:00   Cape Verde            */
    { "LON",    0 },  /* UTC+0:00   London                */
    { "PAR",    4 },  /* UTC+1:00   Paris                 */
    { "CAI",    8 },  /* UTC+2:00   Cairo                 */
    { "MSK",   12 },  /* UTC+3:00   Moscow                */
    { "DXB",   16 },  /* UTC+4:00   Dubai                 */
    { "DEL",   22 },  /* UTC+5:30   Delhi                 */
    { "KTM",   23 },  /* UTC+5:45   Kathmandu             */
    { "DAC",   24 },  /* UTC+6:00   Dhaka                 */
    { "BKK",   28 },  /* UTC+7:00   Bangkok               */
    { "SGP",   32 },  /* UTC+8:00   Singapore             */
    { "TKY",   36 },  /* UTC+9:00   Tokyo                 */
    { "SYD",   40 },  /* UTC+10:00  Sydney                */
    { "NOU",   44 },  /* UTC+11:00  Noumea                */
    { "AKL",   48 },  /* UTC+12:00  Auckland              */
};

#define TZ_COUNT  (sizeof(tz_table) / sizeof(tz_table[0]))

/* ================= DST SUPPORT ================= */

typedef enum {
    DST_NONE = 0,
    DST_US,           /* 2nd Sun Mar - 1st Sun Nov */
    DST_EU,           /* Last Sun Mar - Last Sun Oct */
    DST_SOUTHERN      /* 1st Sun Oct - 1st Sun Apr (summer = Oct-Mar) */
} DSTRule_t;

static const DSTRule_t dst_rules[TZ_COUNT] = {
    DST_NONE,      /* BKR */
    DST_NONE,      /* SST */
    DST_NONE,      /* HNL */
    DST_US,        /* ANC */
    DST_US,        /* LAX */
    DST_US,        /* DEN */
    DST_US,        /* CHI */
    DST_US,        /* NYC */
    DST_NONE,      /* CCS */
    DST_NONE,      /* GRU */
    DST_NONE,      /* GSI */
    DST_NONE,      /* CVT */
    DST_EU,        /* LON */
    DST_EU,        /* PAR */
    DST_NONE,      /* CAI */
    DST_NONE,      /* MSK */
    DST_NONE,      /* DXB */
    DST_NONE,      /* DEL */
    DST_NONE,      /* KTM */
    DST_NONE,      /* DAC */
    DST_NONE,      /* BKK */
    DST_NONE,      /* SGP */
    DST_NONE,      /* TKY */
    DST_SOUTHERN,  /* SYD */
    DST_NONE,      /* NOU */
    DST_SOUTHERN,  /* AKL */
};

static int day_of_week(int y, int m, int d)
{
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y--;
    return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}

static int nth_weekday(int year, int month, int weekday, int n)
{
    int dow_1st = day_of_week(year, month, 1);
    int first = 1 + ((weekday - dow_1st + 7) % 7);
    return first + (n - 1) * 7;
}

static int last_weekday(int year, int month, int weekday)
{
    static const uint8_t dim[] = {0,31,28,31,30,31,30,31,31,30,31,30,31};
    int mdays = dim[month];
    if (month == 2) {
        bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
        if (leap) mdays = 29;
    }

    int dow_last = day_of_week(year, month, mdays);
    int diff = (dow_last - weekday + 7) % 7;
    return mdays - diff;
}

static bool date_on_or_after(int month, int day, int ref_month, int ref_day)
{
    if (month > ref_month) return true;
    if (month == ref_month && day >= ref_day) return true;
    return false;
}

static bool date_before(int month, int day, int ref_month, int ref_day)
{
    return !date_on_or_after(month, day, ref_month, ref_day);
}

static int8_t get_dst_offset_q(int tz_idx)
{
    DSTRule_t rule = dst_rules[tz_idx];
    if (rule == DST_NONE) return 0;

    uint8_t month = RV3032_GetMonth();
    uint8_t mday  = RV3032_GetDate();
    uint16_t year = RV3032_GetYear();

    int m = (int)month;
    int d = (int)mday;
    int y = (int)year;

    switch (rule) {
        case DST_US: {
            int start_day = nth_weekday(y, 3, 0, 2);
            int end_day   = nth_weekday(y, 11, 0, 1);

            if (date_on_or_after(m, d, 3, start_day) &&
                date_before(m, d, 11, end_day)) {
                return 4;
            }
            return 0;
        }

        case DST_EU: {
            int start_day = last_weekday(y, 3, 0);
            int end_day   = last_weekday(y, 10, 0);

            if (date_on_or_after(m, d, 3, start_day) &&
                date_before(m, d, 10, end_day)) {
                return 4;
            }
            return 0;
        }

        case DST_SOUTHERN: {
            int start_day = nth_weekday(y, 10, 0, 1);
            int end_day   = nth_weekday(y, 4, 0, 1);

            if (date_on_or_after(m, d, 10, start_day) ||
                date_before(m, d, 4, end_day)) {
                return 4;
            }
            return 0;
        }

        default:
            return 0;
    }
}

/* ================= STATE ================= */

static WorldClockState_t wc_state = WC_STATE_DISPLAY;
static int tz_index[2] = { 4, 22 };  /* Default: NYC (idx 7), LON (idx 12) */
static bool wc_dirty = true;

static uint32_t wc_blink_tick = 0;
static bool     wc_blink_on  = true;
#define WC_BLINK_MS  500

/* ================= PUBLIC API ================= */

void WorldClock_Init(void)
{
    wc_state = WC_STATE_DISPLAY;
    wc_dirty = true;
    wc_blink_tick = HAL_GetTick();
    wc_blink_on = true;
}

void WorldClock_OnLongPress(void)
{
    if (wc_state == WC_STATE_DISPLAY) {
        wc_state = WC_STATE_EDITING_TZ1;
        wc_blink_on = true;
        wc_blink_tick = HAL_GetTick();
        wc_dirty = true;
        Buzzer_BeepShort();
    }
}

void WorldClock_OnPress(void)
{
    if (wc_state == WC_STATE_EDITING_TZ1) {
        wc_state = WC_STATE_EDITING_TZ2;
        wc_blink_on = true;
        wc_blink_tick = HAL_GetTick();
        wc_dirty = true;
        Buzzer_BeepShort();
    } else if (wc_state == WC_STATE_EDITING_TZ2) {
        /* Done editing — persist to EEPROM */
        wc_state = WC_STATE_DISPLAY;
        wc_dirty = true;
        Buzzer_BeepDouble();

        Settings_MarkEEPROMDirty();
        Settings_SaveToEEPROM();
    }
}

void WorldClock_OnScroll(int direction)
{
    if (wc_state == WC_STATE_EDITING_TZ1) {
        tz_index[0] += direction;
        if (tz_index[0] < 0) tz_index[0] = (int)TZ_COUNT - 1;
        if (tz_index[0] >= (int)TZ_COUNT) tz_index[0] = 0;
        wc_blink_on = true;
        wc_blink_tick = HAL_GetTick();
        wc_dirty = true;
    } else if (wc_state == WC_STATE_EDITING_TZ2) {
        tz_index[1] += direction;
        if (tz_index[1] < 0) tz_index[1] = (int)TZ_COUNT - 1;
        if (tz_index[1] >= (int)TZ_COUNT) tz_index[1] = 0;
        wc_blink_on = true;
        wc_blink_tick = HAL_GetTick();
        wc_dirty = true;
    }
}

WorldClockState_t WorldClock_GetState(void)
{
    return wc_state;
}

bool WorldClock_NeedsRedraw(void)
{
    if (wc_state == WC_STATE_EDITING_TZ1 || wc_state == WC_STATE_EDITING_TZ2) {
        uint32_t now = HAL_GetTick();
        if (now - wc_blink_tick >= WC_BLINK_MS) {
            wc_blink_on = !wc_blink_on;
            wc_blink_tick = now;
            wc_dirty = true;
        }
    }

    bool d = wc_dirty;
    wc_dirty = false;
    return d;
}

const char* WorldClock_GetLabel(int slot)
{
    if (slot < 0 || slot > 1) return "???";
    return tz_table[tz_index[slot]].label;
}

void WorldClock_GetTime(int slot, uint8_t *hours, uint8_t *minutes)
{
    if (slot < 0 || slot > 1) { *hours = 0; *minutes = 0; return; }

    const SensorData_t *data = SensorManager_GetData();
    int32_t local_mins = (int32_t)data->hours_24 * 60 + (int32_t)data->minutes;

    /* Home timezone from settings (standard offset + DST) */
    int8_t home_offset_q = Settings_GetHomeTimezoneOffsetQ();
    uint8_t home_idx = Settings_GetHomeTimezoneIndex();
    int8_t home_dst = get_dst_offset_q((int)home_idx);
    int32_t home_total_q = (int32_t)home_offset_q + (int32_t)home_dst;

    /* Target timezone (standard offset + DST) */
    int8_t tz_offset_q = tz_table[tz_index[slot]].utc_offset_quarters;
    int8_t tz_dst = get_dst_offset_q(tz_index[slot]);
    int32_t tz_total_q = (int32_t)tz_offset_q + (int32_t)tz_dst;

    /* Difference in quarter-hours between target and home */
    int32_t diff_mins = ((int32_t)tz_total_q - home_total_q) * 15;
    int32_t tz_mins = local_mins + diff_mins;

    /* Wrap to 0-1439 */
    while (tz_mins < 0) tz_mins += 1440;
    while (tz_mins >= 1440) tz_mins -= 1440;

    *hours = (uint8_t)(tz_mins / 60);
    *minutes = (uint8_t)(tz_mins % 60);
}

bool WorldClock_IsBlinkOn(void)
{
    return wc_blink_on;
}

int WorldClock_GetEditingSlot(void)
{
    if (wc_state == WC_STATE_EDITING_TZ1) return 0;
    if (wc_state == WC_STATE_EDITING_TZ2) return 1;
    return -1;
}

int WorldClock_GetTimezoneCount(void)
{
    return (int)TZ_COUNT;
}

int WorldClock_GetTimezoneIndex(int slot)
{
    if (slot < 0 || slot > 1) return 0;
    return tz_index[slot];
}

/* ================= EEPROM SLOT ACCESSORS ================= */

void WorldClock_SetSlotIndex(int slot, int tz_idx)
{
    if (slot < 0 || slot > 1) return;
    if (tz_idx < 0 || tz_idx >= (int)TZ_COUNT) return;
    tz_index[slot] = tz_idx;
}

int WorldClock_GetSlotIndex(int slot)
{
    if (slot < 0 || slot > 1) return 0;
    return tz_index[slot];
}
