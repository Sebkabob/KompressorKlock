#include "world_clock.h"
#include "sensor_manager.h"
#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

/* ================= TIMEZONE TABLE ================= */
/*
 * UTC offsets stored in quarter-hours so we can represent
 * zones like UTC+5:30 (India) and UTC+5:45 (Nepal).
 *
 * Sorted roughly west-to-east for intuitive scrolling.
 */

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

/* ================= STATE ================= */

static WorldClockState_t wc_state = WC_STATE_DISPLAY;
static int tz_index[2] = { 7, 12 };  /* Default: NYC (idx 7), LON (idx 12) */
static bool wc_dirty = true;

/* Blink for editing */
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
        /* Enter editing: start with TZ1 */
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
        /* Advance to editing TZ2 */
        wc_state = WC_STATE_EDITING_TZ2;
        wc_blink_on = true;
        wc_blink_tick = HAL_GetTick();
        wc_dirty = true;
        Buzzer_BeepShort();
    } else if (wc_state == WC_STATE_EDITING_TZ2) {
        /* Done editing */
        wc_state = WC_STATE_DISPLAY;
        wc_dirty = true;
        Buzzer_BeepDouble();
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

    /* Get local time in total minutes since midnight */
    int32_t local_mins = (int32_t)data->hours_24 * 60 + (int32_t)data->minutes;

    /*
     * We need the user's local UTC offset to compute other zones.
     * Since we don't store it, we assume the device's RTC is set to
     * the user's local time. We'll use a reference offset that the
     * user can mentally calibrate by choosing their own city as one
     * of the two zones.
     *
     * The device time IS local time. To get another timezone:
     *   other_time = local_time + (other_offset - local_offset)
     *
     * We store a "device timezone" that defaults to NYC (-20 quarters = UTC-5).
     * The user sets their own zone as TZ1 or TZ2, matching their RTC.
     *
     * For simplicity: we compute relative to UTC using a fixed device offset.
     * The user should set one slot to their own city to keep it meaningful.
     *
     * Actually, the cleanest approach: store a device_tz_index and let
     * the user set it. But to keep it simple and match the spec,
     * we'll just compute the difference between the two selected zones
     * and the device's assumed UTC offset.
     *
     * Let's use a simpler model: device_tz is always slot 0's timezone
     * when first initialized (NYC). The user's RTC matches their local time.
     * We'll store a device offset that we can derive.
     *
     * SIMPLEST: assume device RTC = UTC. User picks two cities, we apply
     * their UTC offsets directly. This is actually the most flexible -
     * the user just needs to set their RTC to UTC, OR they pick their
     * own city as a reference and the other city will be correct relative
     * to it.
     *
     * Let's go with: device time = local time, and we need a "home" offset.
     * We'll default home to NYC (UTC-5) and store it. The world clock
     * computes: tz_time = local_time + (tz_offset - home_offset)
     */

    /* Home offset: use a static. Could be made configurable later. */
    static int8_t home_offset_quarters = -20; /* NYC = UTC-5 */

    int8_t tz_offset = tz_table[tz_index[slot]].utc_offset_quarters;

    /* Difference in quarter-hours */
    int32_t diff_quarters = (int32_t)tz_offset - (int32_t)home_offset_quarters;
    int32_t diff_mins = diff_quarters * 15;

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
