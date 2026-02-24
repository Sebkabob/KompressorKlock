#ifndef TIMER_APP_H
#define TIMER_APP_H

#include <stdint.h>
#include <stdbool.h>

/* ================= STOPWATCH ================= */

typedef enum {
    SW_STATE_STOPPED,    /* Shows 00:00.0, press to start */
    SW_STATE_RUNNING,    /* Counting up, press to pause */
    SW_STATE_PAUSED      /* Frozen display, press to resume */
} StopwatchState_t;

void Stopwatch_Init(void);
void Stopwatch_Update(void);
void Stopwatch_OnPress(void);
void Stopwatch_OnLongPress(void);   /* Hold 1s = reset */
StopwatchState_t Stopwatch_GetState(void);
uint8_t  Stopwatch_GetMinutes(void);
uint8_t  Stopwatch_GetSeconds(void);
uint8_t  Stopwatch_GetTenths(void);  /* Deciseconds 0-9 */
uint16_t Stopwatch_GetTotalSeconds(void);
bool Stopwatch_NeedsRedraw(void);

/* ================= COUNTDOWN ================= */

typedef enum {
    CD_STATE_IDLE,       /* No time set, press to enter setting */
    CD_STATE_SETTING,    /* Editing H:M:S, scroll adjusts, press advances */
    CD_STATE_RUNNING,    /* Counting down, press to pause */
    CD_STATE_PAUSED,     /* Frozen, press to resume, hold 1s to clear */
    CD_STATE_FINISHED    /* Time's up, beeping/flashing, scroll away freely */
} CountdownState_t;

typedef enum {
    CD_FIELD_HOURS,
    CD_FIELD_MINUTES,
    CD_FIELD_SECONDS,
    CD_FIELD_COUNT
} CountdownField_t;

void Countdown_Init(void);
void Countdown_Update(void);
void Countdown_OnPress(void);
void Countdown_OnScroll(int direction);
void Countdown_OnLongPress(void);    /* Hold 1s = clear (when paused/setting) */
CountdownState_t Countdown_GetState(void);
CountdownField_t Countdown_GetField(void);
uint8_t Countdown_GetHours(void);
uint8_t Countdown_GetMinutes(void);
uint8_t Countdown_GetSeconds(void);
bool Countdown_NeedsRedraw(void);

#endif // TIMER_APP_H
