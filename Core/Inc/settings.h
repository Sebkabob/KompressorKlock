#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include "matrix.h"

/* ================= SETTINGS STATES ================= */
typedef enum {
    SETTINGS_STATE_INACTIVE,      /* Normal clock face mode */
    SETTINGS_STATE_MENU,          /* Browsing settings list */
    SETTINGS_STATE_EDITING,       /* Inside a specific setting */
} SettingsState_t;

/* ================= SETTING IDS ================= */
typedef enum {
    SETTING_TIME_SET = 0,
    SETTING_DATE_SET,
    SETTING_BRIGHTNESS,
    SETTING_COUNT
} SettingID_t;

/* ================= PUBLIC API ================= */

void Settings_Init(void);
void Settings_Enter(void);
void Settings_Exit(void);
SettingsState_t Settings_GetState(void);
void Settings_OnScroll(int direction);
void Settings_OnPress(void);
void Settings_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
bool Settings_NeedsRedraw(void);

#endif // SETTINGS_H
