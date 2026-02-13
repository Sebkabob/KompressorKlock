#include "screens.h"
#include "matrix.h"
#include "main.h"
#include <string.h>

/* ================= INTERNAL STATE ================= */

typedef enum {
    STATE_IDLE,
    STATE_TRANSITIONING
} ScreenState_t;

typedef enum {
    DISSOLVE_PHASE_OUT,
    DISSOLVE_PHASE_IN
} DissolvePhase_t;

#define TOTAL_PIXELS (NUM_ROWS * NUM_COLS)  // 588

typedef struct {
    ScreenState_t       state;

    ScreenRenderFunc_t  screens[MAX_SCREENS];
    int                 screen_count;
    int                 current_screen;
    int                 target_screen;

    // Transition
    TransitionType_t    active_transition;
    uint32_t            transition_start;
    int                 last_offset;        // Track last rendered offset to avoid redundant redraws
    DissolvePhase_t     dissolve_phase;
    uint32_t            dissolve_phase_start;
    int                 last_dissolve_count;

    // Auto-cycle
    bool                auto_cycle;
    TransitionType_t    auto_cycle_transition;
    uint32_t            last_cycle_time;

    // Dirty flag
    bool                dirty;

    // Offscreen buffers (only used during transitions)
    uint8_t             buf_current[NUM_ROWS][TOTAL_BYTES];
    uint8_t             buf_target[NUM_ROWS][TOTAL_BYTES];

    // Pre-shuffled dissolve order
    uint16_t            dissolve_order[TOTAL_PIXELS];
} ScreenManager_t;

static ScreenManager_t sm;

/* ================= DISSOLVE SHUFFLE ================= */

static uint16_t shuffle_rng = 0xACE1;

static uint16_t shuffle_rand(void)
{
    shuffle_rng ^= shuffle_rng << 7;
    shuffle_rng ^= shuffle_rng >> 9;
    shuffle_rng ^= shuffle_rng << 8;
    return shuffle_rng;
}

static void shuffle_dissolve_order(void)
{
    for (int i = 0; i < TOTAL_PIXELS; i++) {
        sm.dissolve_order[i] = (uint16_t)i;
    }
    for (int i = TOTAL_PIXELS - 1; i > 0; i--) {
        uint16_t j = shuffle_rand() % (i + 1);
        uint16_t tmp = sm.dissolve_order[i];
        sm.dissolve_order[i] = sm.dissolve_order[j];
        sm.dissolve_order[j] = tmp;
    }
}

/* ================= PIXEL HELPERS ================= */

static inline uint8_t get_pixel(const uint8_t buf[NUM_ROWS][TOTAL_BYTES], int r, int c)
{
    return (buf[r][c / 8] >> (c % 8)) & 1;
}

static inline void set_pixel(uint8_t buf[NUM_ROWS][TOTAL_BYTES], int r, int c, uint8_t val)
{
    int byte = c / 8;
    int bit  = c % 8;
    if (val)
        buf[r][byte] |= (1 << bit);
    else
        buf[r][byte] &= ~(1 << bit);
}

/* ================= TRANSITION RENDERERS ================= */

static void render_slide_horizontal(int offset, bool slide_left)
{
    uint8_t composite[NUM_ROWS][TOTAL_BYTES];
    memset(composite, 0, sizeof(composite));

    for (int r = 0; r < NUM_ROWS; r++) {
        for (int c = 0; c < NUM_COLS; c++) {
            int src_c;
            const uint8_t (*src_buf)[TOTAL_BYTES];

            if (slide_left) {
                int old_c = c + offset;
                int new_c = c - (NUM_COLS - offset);
                if (old_c < NUM_COLS) {
                    src_c = old_c; src_buf = sm.buf_current;
                } else if (new_c >= 0) {
                    src_c = new_c; src_buf = sm.buf_target;
                } else continue;
            } else {
                int old_c = c - offset;
                int new_c = c + (NUM_COLS - offset);
                if (old_c >= 0) {
                    src_c = old_c; src_buf = sm.buf_current;
                } else if (new_c < NUM_COLS) {
                    src_c = new_c; src_buf = sm.buf_target;
                } else continue;
            }

            if (get_pixel(src_buf, r, src_c)) {
                set_pixel(composite, r, c, 1);
            }
        }
    }

    Matrix_LoadBuffer(composite);
}

static void render_slide_vertical(int offset, bool slide_up)
{
    uint8_t composite[NUM_ROWS][TOTAL_BYTES];
    memset(composite, 0, sizeof(composite));

    for (int r = 0; r < NUM_ROWS; r++) {
        int src_r;
        const uint8_t (*src_buf)[TOTAL_BYTES];

        if (slide_up) {
            int old_r = r + offset;
            int new_r = r - (NUM_ROWS - offset);
            if (old_r < NUM_ROWS) {
                src_r = old_r; src_buf = sm.buf_current;
            } else if (new_r >= 0) {
                src_r = new_r; src_buf = sm.buf_target;
            } else continue;
        } else {
            int old_r = r - offset;
            int new_r = r + (NUM_ROWS - offset);
            if (old_r >= 0) {
                src_r = old_r; src_buf = sm.buf_current;
            } else if (new_r < NUM_ROWS) {
                src_r = new_r; src_buf = sm.buf_target;
            } else continue;
        }

        memcpy(composite[r], src_buf[src_r], TOTAL_BYTES);
    }

    Matrix_LoadBuffer(composite);
}

static bool render_dissolve(void)
{
    uint32_t elapsed = HAL_GetTick() - sm.dissolve_phase_start;
    uint8_t composite[NUM_ROWS][TOTAL_BYTES];

    // Calculate how many pixels should be processed based on time
    int target_count = (int)((uint32_t)elapsed * TOTAL_PIXELS / DISSOLVE_PHASE_MS);
    if (target_count > TOTAL_PIXELS) target_count = TOTAL_PIXELS;

    // Skip if nothing changed since last render
    if (target_count == sm.last_dissolve_count) return false;
    sm.last_dissolve_count = target_count;

    if (sm.dissolve_phase == DISSOLVE_PHASE_OUT) {
        // Start from current, black out pixels
        memcpy(composite, sm.buf_current, sizeof(composite));
        for (int i = 0; i < target_count; i++) {
            uint16_t idx = sm.dissolve_order[i];
            set_pixel(composite, idx / NUM_COLS, idx % NUM_COLS, 0);
        }
        Matrix_LoadBuffer(composite);

        if (target_count >= TOTAL_PIXELS) {
            sm.dissolve_phase = DISSOLVE_PHASE_IN;
            sm.dissolve_phase_start = HAL_GetTick();
            sm.last_dissolve_count = -1;
            shuffle_rng ^= (uint16_t)HAL_GetTick();
            shuffle_dissolve_order();
        }
        return false;

    } else {
        // Start from black, reveal target pixels
        memset(composite, 0, sizeof(composite));
        for (int i = 0; i < target_count; i++) {
            uint16_t idx = sm.dissolve_order[i];
            int r = idx / NUM_COLS;
            int c = idx % NUM_COLS;
            if (get_pixel(sm.buf_target, r, c)) {
                set_pixel(composite, r, c, 1);
            }
        }
        Matrix_LoadBuffer(composite);

        return (target_count >= TOTAL_PIXELS);
    }
}

/* ================= TRANSITION DRIVER ================= */

static void run_transition(void)
{
    uint32_t elapsed = HAL_GetTick() - sm.transition_start;
    bool done = false;

    switch (sm.active_transition) {
        case TRANSITION_SLIDE_LEFT:
        case TRANSITION_SLIDE_RIGHT: {
            // Time-based: calculate pixel offset from elapsed time
            int offset = (int)((uint32_t)elapsed * NUM_COLS / SLIDE_H_DURATION_MS);
            if (offset >= NUM_COLS) {
                offset = NUM_COLS;
                done = true;
            }
            // Only re-render if offset actually changed
            if (offset != sm.last_offset) {
                sm.last_offset = offset;
                render_slide_horizontal(offset,
                                        sm.active_transition == TRANSITION_SLIDE_LEFT);
            }
            break;
        }

        case TRANSITION_SLIDE_UP:
        case TRANSITION_SLIDE_DOWN: {
            int offset = (int)((uint32_t)elapsed * NUM_ROWS / SLIDE_V_DURATION_MS);
            if (offset >= NUM_ROWS) {
                offset = NUM_ROWS;
                done = true;
            }
            if (offset != sm.last_offset) {
                sm.last_offset = offset;
                render_slide_vertical(offset,
                                      sm.active_transition == TRANSITION_SLIDE_UP);
            }
            break;
        }

        case TRANSITION_DISSOLVE:
            done = render_dissolve();
            break;

        case TRANSITION_NONE:
        default:
            done = true;
            break;
    }

    if (done) {
        sm.state = STATE_IDLE;
        sm.current_screen = sm.target_screen;
        sm.last_cycle_time = HAL_GetTick();
        sm.dirty = true;
    }
}

/* ================= START A TRANSITION ================= */

static void begin_transition(int target, TransitionType_t transition)
{
    if (sm.state == STATE_TRANSITIONING) return;
    if (target < 0 || target >= sm.screen_count) return;
    if (target == sm.current_screen && transition != TRANSITION_NONE) return;

    sm.target_screen = target;
    sm.active_transition = transition;
    sm.state = STATE_TRANSITIONING;
    sm.transition_start = HAL_GetTick();
    sm.last_offset = -1;
    sm.last_dissolve_count = -1;

    memset(sm.buf_current, 0, sizeof(sm.buf_current));
    memset(sm.buf_target, 0, sizeof(sm.buf_target));

    if (sm.screens[sm.current_screen]) {
        sm.screens[sm.current_screen](sm.buf_current);
    }
    if (sm.screens[sm.target_screen]) {
        sm.screens[sm.target_screen](sm.buf_target);
    }

    if (transition == TRANSITION_DISSOLVE) {
        sm.dissolve_phase = DISSOLVE_PHASE_OUT;
        sm.dissolve_phase_start = HAL_GetTick();
        shuffle_rng = 0xACE1 ^ (uint16_t)HAL_GetTick();
        shuffle_dissolve_order();
    }
}

/* ================= PUBLIC API ================= */

void Screen_Init(void)
{
    memset(&sm, 0, sizeof(sm));
    sm.state = STATE_IDLE;
    sm.current_screen = 0;
    sm.auto_cycle = true;
    sm.auto_cycle_transition = TRANSITION_SLIDE_LEFT;
    sm.last_cycle_time = HAL_GetTick();
    sm.dirty = true;
}

int Screen_Register(ScreenRenderFunc_t render_func)
{
    if (sm.screen_count >= MAX_SCREENS) return -1;
    sm.screens[sm.screen_count] = render_func;
    return sm.screen_count++;
}

void Screen_GoTo(int screen_index, TransitionType_t transition)
{
    begin_transition(screen_index, transition);
}

void Screen_Next(TransitionType_t transition)
{
    if (sm.screen_count <= 1) return;
    int next = (sm.current_screen + 1) % sm.screen_count;
    begin_transition(next, transition);
}

void Screen_Prev(TransitionType_t transition)
{
    if (sm.screen_count <= 1) return;
    int prev = (sm.current_screen - 1 + sm.screen_count) % sm.screen_count;
    begin_transition(prev, transition);
}

void Screen_Update(void)
{
    if (sm.screen_count == 0) return;

    if (sm.state == STATE_TRANSITIONING) {
        run_transition();
        return;
    }

    if (sm.dirty) {
        uint8_t buf[NUM_ROWS][TOTAL_BYTES];
        memset(buf, 0, sizeof(buf));
        if (sm.screens[sm.current_screen]) {
            sm.screens[sm.current_screen](buf);
        }
        Matrix_LoadBuffer(buf);
        sm.dirty = false;
    }

    if (sm.auto_cycle && sm.screen_count > 1) {
        if ((HAL_GetTick() - sm.last_cycle_time) >= AUTO_CYCLE_INTERVAL_MS) {
            Screen_Next(sm.auto_cycle_transition);
        }
    }
}

void Screen_SetAutoCycle(bool enabled)
{
    sm.auto_cycle = enabled;
    sm.last_cycle_time = HAL_GetTick();
}

void Screen_SetAutoCycleTransition(TransitionType_t transition)
{
    sm.auto_cycle_transition = transition;
}

int Screen_GetCurrent(void)
{
    return sm.current_screen;
}

bool Screen_IsTransitioning(void)
{
    return sm.state == STATE_TRANSITIONING;
}

void Screen_MarkDirty(void)
{
    sm.dirty = true;
}
