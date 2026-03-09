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
#define MAX_SCREENS              12
#define AUTO_CYCLE_INTERVAL_MS   10000

/* ================= TRANSITION SPEED ================= */
typedef enum {
    TRANSITION_SPEED_SLOW = 0,
    TRANSITION_SPEED_NORMAL,
    TRANSITION_SPEED_FAST,
    TRANSITION_SPEED_FASTEST,
    TRANSITION_SPEED_COUNT
} TransitionSpeed_t;

void Screen_SetTransitionSpeed(TransitionSpeed_t speed);
TransitionSpeed_t Screen_GetTransitionSpeed(void);

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

/**
 * @brief Enter settings mode with dissolve transition.
 *        Saves current screen index and switches to settings renderer.
 */
void Screen_EnterSettings(void);

/**
 * @brief Exit settings mode with dissolve transition.
 *        Restores the screen that was active before entering settings.
 */
void Screen_ExitSettings(void);

void Screen_SetCurrent(int index);

/**
 * @brief Boot dissolve: transitions from the boot logo into the
 *        current screen with a dissolve effect.
 *        Call after Screen_SetCurrent().
 */
void Screen_BootDissolve(void);


#endif // SCREENS_H
