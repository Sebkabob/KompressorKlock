#ifndef SCREENS_H
#define SCREENS_H

#include <stdint.h>
#include <stdbool.h>
#include "matrix.h"

/* ================= TRANSITION TYPES ================= */
typedef enum {
    TRANSITION_NONE = 0,
    TRANSITION_SLIDE_LEFT,
    TRANSITION_SLIDE_RIGHT,
    TRANSITION_SLIDE_UP,
    TRANSITION_SLIDE_DOWN,
    TRANSITION_DISSOLVE,
    TRANSITION_COUNT
} TransitionType_t;

/* ================= SCREEN CALLBACK ================= */
typedef void (*ScreenRenderFunc_t)(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);

/* ================= CONFIGURATION ================= */
#define MAX_SCREENS              8
#define AUTO_CYCLE_INTERVAL_MS   1000

// Time-based transition speeds (in milliseconds)
// These are constant regardless of timer period / brightness
#define SLIDE_H_DURATION_MS      1300   // Horizontal slide total time
#define SLIDE_V_DURATION_MS      400   // Vertical slide total time
#define DISSOLVE_PHASE_MS        100   // Each dissolve phase (out + in)

/* ================= PUBLIC API ================= */

void Screen_Init(void);
int  Screen_Register(ScreenRenderFunc_t render_func);
void Screen_GoTo(int screen_index, TransitionType_t transition);
void Screen_Next(TransitionType_t transition);
void Screen_Prev(TransitionType_t transition);
void Screen_Update(void);
void Screen_SetAutoCycle(bool enabled);
void Screen_SetAutoCycleTransition(TransitionType_t transition);
int  Screen_GetCurrent(void);
bool Screen_IsTransitioning(void);
void Screen_MarkDirty(void);

#endif // SCREENS_H
