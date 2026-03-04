#ifndef WORLD_CLOCK_H
#define WORLD_CLOCK_H

#include <stdint.h>
#include <stdbool.h>

/* ================= TIMEZONE DEFINITION ================= */

typedef struct {
    const char *label;    /* 3-char city code e.g. "NYC", "LON", "TKY" */
    int8_t     utc_offset_quarters; /* UTC offset in 15-min increments (e.g. +20 = UTC+5:00) */
} Timezone_t;

/* ================= WORLD CLOCK STATES ================= */

typedef enum {
    WC_STATE_DISPLAY,     /* Showing two time zones, scroll changes screens */
    WC_STATE_EDITING_TZ1, /* Scrolling through timezone list for slot 1 */
    WC_STATE_EDITING_TZ2  /* Scrolling through timezone list for slot 2 */
} WorldClockState_t;

/* ================= PUBLIC API ================= */

void WorldClock_Init(void);
void WorldClock_OnPress(void);
void WorldClock_OnScroll(int direction);
void WorldClock_OnLongPress(void);
WorldClockState_t WorldClock_GetState(void);
bool WorldClock_NeedsRedraw(void);

/* Get the selected timezone label and computed hours/minutes for each slot */
const char* WorldClock_GetLabel(int slot); /* slot 0 or 1 */
void WorldClock_GetTime(int slot, uint8_t *hours, uint8_t *minutes);

/* For blink animation during editing */
bool WorldClock_IsBlinkOn(void);

/* Which slot (0 or 1) is being edited — only valid during editing states */
int WorldClock_GetEditingSlot(void);

/* Get total timezone count and current index for a slot */
int WorldClock_GetTimezoneCount(void);
int WorldClock_GetTimezoneIndex(int slot);

#endif // WORLD_CLOCK_H
