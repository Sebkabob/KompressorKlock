#include "settings.h"
#include "settings_screens.h"
#include "matrix.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* ================= SETTING NAMES ================= */
static const char *setting_names[SETTING_COUNT] = {
    "Time Set",
    "Date Set",
    "Brightness",
};

/* ================= STATE ================= */
static SettingsState_t state = SETTINGS_STATE_INACTIVE;
static int menu_index = 0;

/* ================= SETTING DISPATCH TABLES ================= */

typedef void (*SettingEnterFunc)(void);
typedef void (*SettingScrollFunc)(int direction);
typedef bool (*SettingPressFunc)(void);
typedef void (*SettingRenderFunc)(uint8_t buf[NUM_ROWS][TOTAL_BYTES]);
typedef bool (*SettingNeedsRedrawFunc)(void);

static const SettingEnterFunc setting_enter[SETTING_COUNT] = {
    TimeSetting_Enter,
    DateSetting_Enter,
    BrightnessSetting_Enter,
};

static const SettingScrollFunc setting_scroll[SETTING_COUNT] = {
    TimeSetting_OnScroll,
    DateSetting_OnScroll,
    BrightnessSetting_OnScroll,
};

static const SettingPressFunc setting_press[SETTING_COUNT] = {
    TimeSetting_OnPress,
    DateSetting_OnPress,
    BrightnessSetting_OnPress,
};

static const SettingRenderFunc setting_render[SETTING_COUNT] = {
    TimeSetting_Render,
    DateSetting_Render,
    BrightnessSetting_Render,
};

static const SettingNeedsRedrawFunc setting_needs_redraw[SETTING_COUNT] = {
    TimeSetting_NeedsRedraw,
    DateSetting_NeedsRedraw,
    BrightnessSetting_NeedsRedraw,
};

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
        setting_scroll[menu_index](direction);
    }
}

void Settings_OnPress(void)
{
    if (state == SETTINGS_STATE_MENU) {
        state = SETTINGS_STATE_EDITING;
        setting_enter[menu_index]();
    } else if (state == SETTINGS_STATE_EDITING) {
        bool done = setting_press[menu_index]();
        if (done) {
            state = SETTINGS_STATE_MENU;
        }
    }
}

void Settings_Render(uint8_t buf[NUM_ROWS][TOTAL_BYTES])
{
    if (state == SETTINGS_STATE_MENU) {
        Matrix_DrawTextCentered_Buf(buf, 0, setting_names[menu_index]);

        if (menu_index > 0) {
            Matrix_DrawText_Buf(buf, 0, 0, "<");
        }

        if (menu_index < SETTING_COUNT - 1) {
            Matrix_DrawTextRight_Buf(buf, 0, ">");
        }
    } else if (state == SETTINGS_STATE_EDITING) {
        setting_render[menu_index](buf);
    }
}

bool Settings_NeedsRedraw(void)
{
    if (state == SETTINGS_STATE_MENU) {
        return false;
    } else if (state == SETTINGS_STATE_EDITING) {
        return setting_needs_redraw[menu_index]();
    }
    return false;
}
