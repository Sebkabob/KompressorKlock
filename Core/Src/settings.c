#include "settings.h"
#include "settings_screens.h"
#include "screens.h"
#include "matrix.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ================= GLOBAL DISPLAY PREFERENCES ================= */
static bool use_24hour = false;
static bool use_celsius = false;

/* ================= STATE ================= */
static SettingsState_t state = SETTINGS_STATE_INACTIVE;
static int menu_index = 0;

/* ================= SETTING DISPATCH TABLES ================= */

typedef void (*SettingEnterFunc)(void);
typedef void (*SettingScrollFunc)(int direction);
typedef bool (*SettingPressFunc)(void);
typedef void (*SettingRenderFunc)(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
typedef bool (*SettingNeedsRedrawFunc)(void);
typedef bool (*SettingIsOnOKFunc)(void);

static const SettingEnterFunc setting_enter[SETTING_COUNT] = {
    TimeSetting_Enter,
    DateSetting_Enter,
    BrightnessSetting_Enter,
    NULL,  /* 12/24hr — handled inline */
    NULL,  /* F/C — handled inline */
    NULL,  /* Exit — handled inline */
};

static const SettingScrollFunc setting_scroll[SETTING_COUNT] = {
    TimeSetting_OnScroll,
    DateSetting_OnScroll,
    BrightnessSetting_OnScroll,
    NULL,
    NULL,
    NULL,
};

static const SettingPressFunc setting_press[SETTING_COUNT] = {
    TimeSetting_OnPress,
    DateSetting_OnPress,
    BrightnessSetting_OnPress,
    NULL,
    NULL,
    NULL,
};

static const SettingRenderFunc setting_render[SETTING_COUNT] = {
    TimeSetting_Render,
    DateSetting_Render,
    BrightnessSetting_Render,
    NULL,
    NULL,
    NULL,
};

static const SettingNeedsRedrawFunc setting_needs_redraw[SETTING_COUNT] = {
    TimeSetting_NeedsRedraw,
    DateSetting_NeedsRedraw,
    BrightnessSetting_NeedsRedraw,
    NULL,
    NULL,
    NULL,
};

static const SettingIsOnOKFunc setting_is_on_ok[SETTING_COUNT] = {
    TimeSetting_IsOnOK,
    DateSetting_IsOnOK,
    BrightnessSetting_IsOnOK,
    NULL,
    NULL,
    NULL,
};

/* ================= HELPER: is this an inline-toggle setting? ================= */
static bool is_inline_setting(int idx)
{
    return (idx == SETTING_12_24HR || idx == SETTING_TEMP_UNIT || idx == SETTING_EXIT);
}

/* ================= HELPER: get dynamic menu label ================= */
static const char* get_menu_label(int idx)
{
    switch (idx) {
        case SETTING_TIME_SET:   return "Time Set";
        case SETTING_DATE_SET:   return "Date Set";
        case SETTING_BRIGHTNESS: return "Brightness";
        case SETTING_12_24HR:    return use_24hour ? "24hr" : "12hr";
        case SETTING_TEMP_UNIT:  return use_celsius ? "\x03""C" : "\x03""F";
        case SETTING_EXIT:       return "Exit";
        default:                 return "???";
    }
}

/* ================= PUBLIC API ================= */

void Settings_Init(void)
{
    state = SETTINGS_STATE_INACTIVE;
    menu_index = 0;
}

void Settings_Enter(void)
{
    state = SETTINGS_STATE_MENU;
    menu_index = 0;
}

void Settings_Exit(void)
{
    state = SETTINGS_STATE_INACTIVE;
}

SettingsState_t Settings_GetState(void)
{
    return state;
}

void Settings_OnScroll(int direction)
{
    if (state == SETTINGS_STATE_MENU) {
        menu_index += direction;
        if (menu_index < 0) menu_index = 0;
        if (menu_index >= SETTING_COUNT) menu_index = SETTING_COUNT - 1;
    } else if (state == SETTINGS_STATE_EDITING) {
        if (setting_scroll[menu_index]) {
            setting_scroll[menu_index](direction);
        }
    }
}

void Settings_OnPress(void)
{
    if (state == SETTINGS_STATE_MENU) {
        /* Handle inline toggles directly from the menu */
        if (menu_index == SETTING_12_24HR) {
            use_24hour = !use_24hour;
            /* Stay in menu — label updates on next render */
            return;
        }
        if (menu_index == SETTING_TEMP_UNIT) {
            use_celsius = !use_celsius;
            return;
        }
        if (menu_index == SETTING_EXIT) {
            Settings_Exit();
            Screen_ExitSettings();
            return;
        }

        /* Normal settings: enter editing mode */
        state = SETTINGS_STATE_EDITING;
        if (setting_enter[menu_index]) {
            setting_enter[menu_index]();
        }
    } else if (state == SETTINGS_STATE_EDITING) {
        if (setting_press[menu_index]) {
            bool done = setting_press[menu_index]();
            if (done) {
                state = SETTINGS_STATE_MENU;
            }
        }
    }
}

void Settings_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    if (state == SETTINGS_STATE_MENU) {
        Matrix_DrawTextCentered_Buf(buf, 0, get_menu_label(menu_index));

        if (menu_index > 0) {
            Matrix_DrawText_Buf(buf, 0, 0, "<");
        }

        if (menu_index < SETTING_COUNT - 1) {
            Matrix_DrawTextRight_Buf(buf, 0, ">");
        }
    } else if (state == SETTINGS_STATE_EDITING) {
        if (setting_render[menu_index]) {
            setting_render[menu_index](buf);
        }
    }
}

bool Settings_NeedsRedraw(void)
{
    if (state == SETTINGS_STATE_MENU) {
        return false;
    } else if (state == SETTINGS_STATE_EDITING) {
        if (setting_needs_redraw[menu_index]) {
            return setting_needs_redraw[menu_index]();
        }
    }
    return false;
}

bool Settings_IsOnOK(void)
{
    if (state == SETTINGS_STATE_MENU) {
        /* Inline toggles act like instant-press OK */
        return is_inline_setting(menu_index);
    }
    if (state == SETTINGS_STATE_EDITING) {
        if (setting_is_on_ok[menu_index]) {
            return setting_is_on_ok[menu_index]();
        }
    }
    return false;
}

/* ================= GLOBAL PREFERENCE ACCESSORS ================= */

bool Settings_Is24Hour(void)
{
    return use_24hour;
}

void Settings_Set24Hour(bool enabled)
{
    use_24hour = enabled;
}

bool Settings_IsCelsius(void)
{
    return use_celsius;
}

void Settings_SetCelsius(bool enabled)
{
    use_celsius = enabled;
}
