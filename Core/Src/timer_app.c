#include "timer_app.h"
#include "buzzer.h"
#include "main.h"
#include <stdbool.h>

/* =====================================================================
 *  STOPWATCH
 *
 *  Press = start / pause / resume
 *  Long press (1s) = reset to zero (any state)
 *  Scroll = ALWAYS navigates screens (never claimed)
 *
 *  Display format (handled by screen_impl):
 *    < 60s:   SS.cc          e.g. "05.23"
 *    < 60m:   M:SS.cc        e.g. "1:05.23"
 *    >= 60m:  H:MM:SS.cc     e.g. "1:00:00.23"
 *
 *  Updates every 10ms for centisecond display.
 * =====================================================================*/

static StopwatchState_t sw_state = SW_STATE_STOPPED;
static uint32_t sw_elapsed_ms   = 0;   /* accumulated while paused */
static uint32_t sw_start_tick   = 0;   /* HAL_GetTick when started/resumed */
static bool     sw_dirty        = true;
static uint8_t  sw_last_cs      = 0xFF; /* last displayed centisecond */

void Stopwatch_Init(void)
{
    sw_state      = SW_STATE_STOPPED;
    sw_elapsed_ms = 0;
    sw_start_tick = 0;
    sw_dirty      = true;
    sw_last_cs    = 0xFF;
}

uint32_t Stopwatch_GetTotalMs(void)
{
    if (sw_state == SW_STATE_RUNNING) {
        return sw_elapsed_ms + (HAL_GetTick() - sw_start_tick);
    }
    return sw_elapsed_ms;
}

void Stopwatch_Update(void)
{
    if (sw_state != SW_STATE_RUNNING) return;

    /* Redraw every 10ms (centisecond change) */
    uint32_t total = Stopwatch_GetTotalMs();
    uint8_t cs = (total / 10) % 100;

    if (cs != sw_last_cs) {
        sw_last_cs = cs;
        sw_dirty = true;
    }
}

void Stopwatch_OnPress(void)
{
    switch (sw_state) {
        case SW_STATE_STOPPED:
            sw_start_tick = HAL_GetTick();
            sw_state = SW_STATE_RUNNING;
            sw_dirty = true;
            break;

        case SW_STATE_RUNNING:
            sw_elapsed_ms += (HAL_GetTick() - sw_start_tick);
            sw_state = SW_STATE_PAUSED;
            sw_dirty = true;
            break;

        case SW_STATE_PAUSED:
            sw_start_tick = HAL_GetTick();
            sw_state = SW_STATE_RUNNING;
            sw_dirty = true;
            break;
    }
}

void Stopwatch_OnLongPress(void)
{
    /* Reset from any state */
    sw_elapsed_ms = 0;
    sw_start_tick = 0;
    sw_state = SW_STATE_STOPPED;
    sw_last_cs = 0xFF;
    sw_dirty = true;
    Buzzer_BeepShort();
}

StopwatchState_t Stopwatch_GetState(void) { return sw_state; }

bool Stopwatch_NeedsRedraw(void)
{
    bool d = sw_dirty;
    sw_dirty = false;
    return d;
}

/* =====================================================================
 *  COUNTDOWN
 *
 *  IDLE:     Shows previous set time (or 00:00:00 on first use).
 *            Press → SETTING (pre-populated with last set time).
 *  SETTING:  Blinking field editor. Scroll adjusts value, press
 *            advances to next field. After last field press → RUNNING.
 *            Long press → IDLE (cancel).
 *  RUNNING:  Counting down. Press → PAUSED. Scroll navigates away freely.
 *  PAUSED:   Frozen. Press → RUNNING (resume). Long press → IDLE (clear).
 *  FINISHED: Beeping/flashing. Scroll navigates away freely.
 *            Press → IDLE (dismiss alarm).
 * =====================================================================*/

static CountdownState_t cd_state = CD_STATE_IDLE;
static CountdownField_t cd_field = CD_FIELD_HOURS;

/* The "last set" values — remembered across resets */
static uint8_t cd_last_hours   = 0;
static uint8_t cd_last_minutes = 5;   /* Default 5 min on first use */
static uint8_t cd_last_seconds = 0;

/* Current editing values (copied from last_* on SETTING enter) */
static uint8_t cd_set_hours   = 0;
static uint8_t cd_set_minutes = 0;
static uint8_t cd_set_seconds = 0;

static uint32_t cd_remaining_ms = 0;
static uint32_t cd_last_tick    = 0;
static bool     cd_dirty        = true;
static uint8_t  cd_last_sec     = 0xFF;

static uint32_t cd_flash_tick      = 0;
static bool     cd_flash_on        = true;
static bool     cd_alarm_played    = false;
static uint32_t cd_last_alarm_tick = 0;

static uint32_t cd_blink_tick = 0;
static bool     cd_blink_on   = true;
#define CD_BLINK_MS  500

void Countdown_Init(void)
{
    cd_state        = CD_STATE_IDLE;
    cd_field        = CD_FIELD_HOURS;
    cd_set_hours    = cd_last_hours;
    cd_set_minutes  = cd_last_minutes;
    cd_set_seconds  = cd_last_seconds;
    cd_remaining_ms = 0;
    cd_dirty        = true;
    cd_alarm_played = false;
    cd_blink_tick   = HAL_GetTick();
    cd_blink_on     = true;
}

void Countdown_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (cd_state == CD_STATE_RUNNING) {
        uint32_t delta = now - cd_last_tick;
        cd_last_tick = now;

        if (delta >= cd_remaining_ms) {
            cd_remaining_ms = 0;
            cd_state = CD_STATE_FINISHED;
            cd_flash_tick = now;
            cd_flash_on = true;
            cd_alarm_played = false;
            cd_last_alarm_tick = now;
            cd_dirty = true;
        } else {
            cd_remaining_ms -= delta;

            uint32_t total_secs = (cd_remaining_ms + 999) / 1000;
            uint8_t sec = (uint8_t)(total_secs % 60);

            if (sec != cd_last_sec) {
                cd_last_sec = sec;
                cd_dirty = true;
            }
        }
    }

    if (cd_state == CD_STATE_FINISHED) {
        /* Flash display */
        if (now - cd_flash_tick >= 300) {
            cd_flash_on = !cd_flash_on;
            cd_flash_tick = now;
            cd_dirty = true;
        }

        /* Play alarm, repeat every 2s */
        if (!cd_alarm_played) {
            Buzzer_BeepAlarm();
            cd_alarm_played = true;
            cd_last_alarm_tick = now;
        } else if (now - cd_last_alarm_tick >= 2000) {
            if (!Buzzer_IsPlaying()) {
                Buzzer_BeepAlarm();
            }
            cd_last_alarm_tick = now;
        }
    }

    if (cd_state == CD_STATE_SETTING) {
        if (now - cd_blink_tick >= CD_BLINK_MS) {
            cd_blink_on = !cd_blink_on;
            cd_blink_tick = now;
            cd_dirty = true;
        }
    }
}

void Countdown_OnPress(void)
{
    switch (cd_state) {
        case CD_STATE_IDLE:
            /* Enter setting mode, pre-populate with last set time */
            cd_state = CD_STATE_SETTING;
            cd_field = CD_FIELD_HOURS;
            cd_set_hours   = cd_last_hours;
            cd_set_minutes = cd_last_minutes;
            cd_set_seconds = cd_last_seconds;
            cd_blink_on = true;
            cd_blink_tick = HAL_GetTick();
            cd_dirty = true;
            break;

        case CD_STATE_SETTING:
            if (cd_field < CD_FIELD_SECONDS) {
                /* Advance to next field */
                cd_field++;
                cd_blink_on = true;
                cd_blink_tick = HAL_GetTick();
                cd_dirty = true;
            } else {
                /* Last field — start the countdown */
                uint32_t total_ms = ((uint32_t)cd_set_hours * 3600 +
                                     (uint32_t)cd_set_minutes * 60 +
                                     (uint32_t)cd_set_seconds) * 1000;

                if (total_ms == 0) {
                    /* Can't start a zero timer — beep and reset to hours */
                    Buzzer_BeepDouble();
                    cd_field = CD_FIELD_HOURS;
                    cd_dirty = true;
                    return;
                }

                /* Save as "last set" for next time */
                cd_last_hours   = cd_set_hours;
                cd_last_minutes = cd_set_minutes;
                cd_last_seconds = cd_set_seconds;

                cd_remaining_ms = total_ms;
                cd_last_tick = HAL_GetTick();
                cd_last_sec = 0xFF;
                cd_state = CD_STATE_RUNNING;
                Buzzer_BeepShort();
                cd_dirty = true;
            }
            break;

        case CD_STATE_RUNNING:
            cd_state = CD_STATE_PAUSED;
            cd_dirty = true;
            break;

        case CD_STATE_PAUSED:
            cd_last_tick = HAL_GetTick();
            cd_state = CD_STATE_RUNNING;
            Buzzer_BeepShort();
            cd_dirty = true;
            break;

        case CD_STATE_FINISHED:
            /* Dismiss alarm */
            Buzzer_Stop();
            cd_state = CD_STATE_IDLE;
            cd_dirty = true;
            break;
    }
}

void Countdown_OnScroll(int direction)
{
    if (cd_state != CD_STATE_SETTING) return;

    switch (cd_field) {
        case CD_FIELD_HOURS:
            if (direction > 0) {
                cd_set_hours = (cd_set_hours >= 99) ? 0 : cd_set_hours + 1;
            } else {
                cd_set_hours = (cd_set_hours == 0) ? 99 : cd_set_hours - 1;
            }
            break;

        case CD_FIELD_MINUTES:
            if (direction > 0) {
                cd_set_minutes = (cd_set_minutes >= 59) ? 0 : cd_set_minutes + 1;
            } else {
                cd_set_minutes = (cd_set_minutes == 0) ? 59 : cd_set_minutes - 1;
            }
            break;

        case CD_FIELD_SECONDS:
            if (direction > 0) {
                cd_set_seconds = (cd_set_seconds >= 59) ? 0 : cd_set_seconds + 1;
            } else {
                cd_set_seconds = (cd_set_seconds == 0) ? 59 : cd_set_seconds - 1;
            }
            break;

        default:
            break;
    }

    cd_blink_on = true;
    cd_blink_tick = HAL_GetTick();
    cd_dirty = true;
}

void Countdown_OnLongPress(void)
{
    /* Clear back to idle from paused or running */
    if (cd_state == CD_STATE_PAUSED || cd_state == CD_STATE_RUNNING ||
        cd_state == CD_STATE_SETTING) {
        cd_state = CD_STATE_IDLE;
        cd_remaining_ms = 0;
        cd_dirty = true;
        Buzzer_BeepShort();
    }
}

CountdownState_t Countdown_GetState(void) { return cd_state; }
CountdownField_t Countdown_GetField(void) { return cd_field; }

uint8_t Countdown_GetHours(void)
{
    if (cd_state == CD_STATE_IDLE)
        return cd_last_hours;
    if (cd_state == CD_STATE_SETTING)
        return cd_set_hours;
    uint32_t total_secs = (cd_remaining_ms + 999) / 1000;
    return (uint8_t)((total_secs / 3600) % 100);
}

uint8_t Countdown_GetMinutes(void)
{
    if (cd_state == CD_STATE_IDLE)
        return cd_last_minutes;
    if (cd_state == CD_STATE_SETTING)
        return cd_set_minutes;
    uint32_t total_secs = (cd_remaining_ms + 999) / 1000;
    return (uint8_t)((total_secs / 60) % 60);
}

uint8_t Countdown_GetSeconds(void)
{
    if (cd_state == CD_STATE_IDLE)
        return cd_last_seconds;
    if (cd_state == CD_STATE_SETTING)
        return cd_set_seconds;
    uint32_t total_secs = (cd_remaining_ms + 999) / 1000;
    return (uint8_t)(total_secs % 60);
}

bool Countdown_NeedsRedraw(void)
{
    bool d = cd_dirty;
    cd_dirty = false;
    return d;
}
