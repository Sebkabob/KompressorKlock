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
    SETTING_TIMEZONE,
    SETTING_TRANSITIONS,
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

bool Settings_Is24Hour(void);
void Settings_Set24Hour(bool enabled);
bool Settings_IsCelsius(void);
void Settings_SetCelsius(bool enabled);

/* ================= HOME TIMEZONE ================= */

/**
 * @brief Get the home (device) timezone UTC offset in quarter-hours.
 *        Used by world_clock.c to compute other zone times.
 */
int8_t Settings_GetHomeTimezoneOffsetQ(void);

/**
 * @brief Get the home timezone table index.
 */
uint8_t Settings_GetHomeTimezoneIndex(void);

/**
 * @brief Set the home timezone table index.
 */
void Settings_SetHomeTimezoneIndex(uint8_t idx);

/* ================= EEPROM PERSISTENCE ================= */
/*
 * Uses the RV-3032 User EEPROM (32 bytes at 0xCB-0xEA) to store
 * settings that survive full power cycles.
 *
 * Stored settings:
 *   - 12/24 hour mode
 *   - C / F temperature unit
 *   - Auto / manual brightness + manual percent
 *   - Last active screen index
 *   - Home timezone index
 *   - Transition speed
 *
 * Call Settings_LoadFromEEPROM() once at startup AFTER SensorManager_Init,
 * Screen_Init, and all Screen_Register calls.
 *
 * Call Settings_SaveToEEPROM() when exiting settings or when the active
 * screen changes. Only bytes that actually changed are written to protect
 * EEPROM endurance (~100k writes per byte).
 */

/**
 * @brief Load settings from RV-3032 User EEPROM and apply them.
 *        If EEPROM is blank or corrupt, writes current defaults.
 */
void Settings_LoadFromEEPROM(void);

/**
 * @brief Save current settings to EEPROM (only changed bytes).
 *        Safe to call frequently -- no-ops if nothing changed.
 */
void Settings_SaveToEEPROM(void);

/**
 * @brief Mark that something changed and needs saving.
 */
void Settings_MarkEEPROMDirty(void);

/**
 * @brief Check if there are unsaved changes.
 */
bool Settings_IsEEPROMDirty(void);

/**
 * @brief Store/retrieve the last active screen index.
 */
void Settings_SetSavedScreen(uint8_t index);
uint8_t Settings_GetSavedScreen(void);

#endif // SETTINGS_H
