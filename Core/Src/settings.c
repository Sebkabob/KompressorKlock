#include "settings.h"
#include "settings_screens.h"
#include "screens.h"
#include "sensor_manager.h"
#include "rtc_rv3032.h"
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

/* =================================================================
 *  EEPROM PERSISTENCE
 *
 *  RV-3032 User EEPROM: 32 bytes at addresses 0xCB–0xEA.
 *  Survives full power loss (VBACKUP powered).
 *
 *  Layout (6 bytes used):
 *    0xCB  magic   (0xA5 = valid block)
 *    0xCC  flags   bit0=24hr, bit1=celsius, bit2=auto_brightness
 *    0xCD  manual_brightness_percent (1–100)
 *    0xCE  last_screen_index
 *    0xCF  version (0x01)
 *    0xD0  checksum (XOR of 0xCB–0xCF)
 *
 *  Single-byte EEPROM access per datasheet §4.6.5 / §4.6.6:
 *    1. EERD=1 (disable auto refresh)
 *    2. Wait EEbusy=0
 *    3. Set EEADDR, EEDATA, EECMD
 *    4. Wait EEbusy=0
 *    5. EERD=0 (re-enable auto refresh)
 * ================================================================= */

#define USER_EE_BASE        0xCB

#define ADDR_MAGIC          (USER_EE_BASE + 0)
#define ADDR_FLAGS          (USER_EE_BASE + 1)
#define ADDR_BRIGHTNESS     (USER_EE_BASE + 2)
#define ADDR_SCREEN         (USER_EE_BASE + 3)
#define ADDR_VERSION        (USER_EE_BASE + 4)
#define ADDR_CHECKSUM       (USER_EE_BASE + 5)

#define BLOCK_SIZE          6

#define EE_MAGIC            0xA5
#define EE_VERSION          0x01

#define FLAG_24HOUR         (1 << 0)
#define FLAG_CELSIUS        (1 << 1)
#define FLAG_AUTO_BRIGHT    (1 << 2)

/* RV-3032 registers used for EEPROM access */
#define REG_TEMP_LSB        0x0E   /* bit 2 = EEbusy */
#define REG_CONTROL1        0x10   /* bit 2 = EERD   */
#define REG_EEADDR          0x3D
#define REG_EEDATA          0x3E
#define REG_EECMD           0x3F

#define EERD_BIT            (1 << 2)
#define EEBUSY_BIT          (1 << 2)

#define EECMD_WRITE_BYTE    0x21
#define EECMD_READ_BYTE     0x22

/* Cached EEPROM state */
typedef struct {
    uint8_t magic;
    uint8_t flags;
    uint8_t brightness;
    uint8_t screen;
    uint8_t version;
    uint8_t checksum;
} EEBlock_t;

static EEBlock_t ee_cached  = {0};   /* what we want in EEPROM   */
static EEBlock_t ee_written = {0};   /* what's actually in EEPROM */
static bool ee_dirty = false;

/* ---------- low-level EEPROM helpers ---------- */

static bool ee_wait_not_busy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        if (!(RV3032_ReadRegister(REG_TEMP_LSB) & EEBUSY_BIT))
            return true;
        HAL_Delay(1);
    }
    return false;
}

static void ee_set_eerd(bool disable_refresh)
{
    uint8_t ctrl1 = RV3032_ReadRegister(REG_CONTROL1);
    if (disable_refresh)
        ctrl1 |= EERD_BIT;
    else
        ctrl1 &= ~EERD_BIT;
    RV3032_WriteRegister(REG_CONTROL1, ctrl1);
}

static uint8_t ee_read_byte(uint8_t addr)
{
    ee_set_eerd(true);
    if (!ee_wait_not_busy(100)) { ee_set_eerd(false); return 0xFF; }

    RV3032_WriteRegister(REG_EEADDR, addr);
    RV3032_WriteRegister(REG_EECMD, EECMD_READ_BYTE);

    if (!ee_wait_not_busy(10)) { ee_set_eerd(false); return 0xFF; }

    uint8_t data = RV3032_ReadRegister(REG_EEDATA);
    ee_set_eerd(false);
    return data;
}

static bool ee_write_byte(uint8_t addr, uint8_t data)
{
    ee_set_eerd(true);
    if (!ee_wait_not_busy(100)) { ee_set_eerd(false); return false; }

    RV3032_WriteRegister(REG_EEADDR, addr);
    RV3032_WriteRegister(REG_EEDATA, data);
    RV3032_WriteRegister(REG_EECMD, EECMD_WRITE_BYTE);

    if (!ee_wait_not_busy(20)) { ee_set_eerd(false); return false; }

    ee_set_eerd(false);
    return true;
}

/* ---------- block helpers ---------- */

static uint8_t ee_checksum(const EEBlock_t *b)
{
    const uint8_t *p = (const uint8_t *)b;
    uint8_t x = 0;
    for (int i = 0; i < BLOCK_SIZE - 1; i++) x ^= p[i];
    return x;
}

static bool ee_validate(const EEBlock_t *b)
{
    return (b->magic == EE_MAGIC &&
            b->version == EE_VERSION &&
            b->checksum == ee_checksum(b));
}

static void ee_read_block(EEBlock_t *b)
{
    uint8_t *p = (uint8_t *)b;
    for (int i = 0; i < BLOCK_SIZE; i++)
        p[i] = ee_read_byte(USER_EE_BASE + i);
}

static void ee_write_changed(void)
{
    ee_cached.checksum = ee_checksum(&ee_cached);

    uint8_t *src = (uint8_t *)&ee_cached;
    uint8_t *dst = (uint8_t *)&ee_written;
    for (int i = 0; i < BLOCK_SIZE; i++) {
        if (src[i] != dst[i]) {
            if (ee_write_byte(USER_EE_BASE + i, src[i]))
                dst[i] = src[i];
        }
    }
}

static void ee_force_write_all(void)
{
    ee_cached.checksum = ee_checksum(&ee_cached);

    uint8_t *p = (uint8_t *)&ee_cached;
    for (int i = 0; i < BLOCK_SIZE; i++)
        ee_write_byte(USER_EE_BASE + i, p[i]);

    memcpy(&ee_written, &ee_cached, sizeof(EEBlock_t));
}

static void ee_build_block(void)
{
    ee_cached.magic   = EE_MAGIC;
    ee_cached.version = EE_VERSION;

    ee_cached.flags = 0;
    if (use_24hour)                       ee_cached.flags |= FLAG_24HOUR;
    if (use_celsius)                      ee_cached.flags |= FLAG_CELSIUS;
    if (SensorManager_IsAutoBrightness()) ee_cached.flags |= FLAG_AUTO_BRIGHT;

    ee_cached.brightness = SensorManager_GetManualBrightnessPercent();
    /* screen field is set separately via Settings_SetSavedScreen */

    ee_cached.checksum = ee_checksum(&ee_cached);
}

static void ee_apply_block(const EEBlock_t *b)
{
    use_24hour = (b->flags & FLAG_24HOUR) != 0;
    use_celsius = (b->flags & FLAG_CELSIUS) != 0;

    bool auto_bright = (b->flags & FLAG_AUTO_BRIGHT) != 0;
    SensorManager_SetAutoBrightness(auto_bright);
    if (!auto_bright) {
        uint8_t pct = b->brightness;
        if (pct < 1) pct = 1;
        if (pct > 100) pct = 100;
        SensorManager_SetManualBrightnessPercent(pct);
    }
}

/* ================= EEPROM PUBLIC API ================= */

void Settings_LoadFromEEPROM(void)
{
    EEBlock_t block;
    ee_read_block(&block);

    if (ee_validate(&block)) {
        memcpy(&ee_cached, &block, sizeof(block));
        memcpy(&ee_written, &block, sizeof(block));
        ee_apply_block(&block);
    } else {
        /* First boot or corrupt — write current defaults */
        ee_build_block();
        ee_cached.screen = 0;
        ee_force_write_all();
    }

    ee_dirty = false;
}

void Settings_SaveToEEPROM(void)
{
    if (!ee_dirty) return;

    ee_build_block();
    ee_write_changed();
    ee_dirty = false;
}

void Settings_MarkEEPROMDirty(void)
{
    ee_dirty = true;
}

bool Settings_IsEEPROMDirty(void)
{
    return ee_dirty;
}

void Settings_SetSavedScreen(uint8_t index)
{
    if (ee_cached.screen != index) {
        ee_cached.screen = index;
        ee_dirty = true;
    }
}

uint8_t Settings_GetSavedScreen(void)
{
    return ee_cached.screen;
}

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
    NULL,
    NULL,
    NULL,
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

/* ================= HELPERS ================= */

static bool is_inline_setting(int idx)
{
    return (idx == SETTING_12_24HR || idx == SETTING_TEMP_UNIT || idx == SETTING_EXIT);
}

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

/* ================= CORE SETTINGS API ================= */

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

    /* Commit any pending changes to EEPROM on exit */
    Settings_SaveToEEPROM();
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
        if (menu_index == SETTING_12_24HR) {
            use_24hour = !use_24hour;
            ee_dirty = true;
            return;
        }
        if (menu_index == SETTING_TEMP_UNIT) {
            use_celsius = !use_celsius;
            ee_dirty = true;
            return;
        }
        if (menu_index == SETTING_EXIT) {
            Settings_Exit();
            Screen_ExitSettings();
            return;
        }

        state = SETTINGS_STATE_EDITING;
        if (setting_enter[menu_index]) {
            setting_enter[menu_index]();
        }
    } else if (state == SETTINGS_STATE_EDITING) {
        if (setting_press[menu_index]) {
            bool done = setting_press[menu_index]();
            if (done) {
                if (menu_index == SETTING_BRIGHTNESS) {
                    ee_dirty = true;
                }
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

bool Settings_Is24Hour(void)   { return use_24hour; }
void Settings_Set24Hour(bool enabled) { use_24hour = enabled; }
bool Settings_IsCelsius(void)  { return use_celsius; }
void Settings_SetCelsius(bool enabled) { use_celsius = enabled; }
