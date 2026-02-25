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
    SETTING_12_24HR,
    SETTING_TEMP_UNIT,
    SETTING_EXIT,
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

/**
 * @brief Check if the current setting editor is on its OK/confirm field.
 *        Used by rotary to decide whether to fire instant press-on-down.
 * @return true if editing and on the OK field, false otherwise
 */
bool Settings_IsOnOK(void);

/* ================= GLOBAL DISPLAY PREFERENCES ================= */

/**
 * @brief Check if display is in 24-hour mode
 * @return true if 24-hour, false if 12-hour
 */
bool Settings_Is24Hour(void);

/**
 * @brief Set 12/24 hour mode
 */
void Settings_Set24Hour(bool enabled);

/**
 * @brief Check if temperature should be displayed in Celsius
 * @return true if Celsius, false if Fahrenheit
 */
bool Settings_IsCelsius(void);

/**
 * @brief Set temperature unit
 */
void Settings_SetCelsius(bool enabled);

#endif // SETTINGS_H
