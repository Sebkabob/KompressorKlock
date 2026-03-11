#include "app.h"
#include "matrix.h"
#include "sensor_manager.h"
#include "screens.h"
#include "screen_impl.h"
#include "rotary.h"
#include "buzzer.h"
#include "timer_app.h"
#include "world_clock.h"
#include "settings.h"
#include <string.h>

/* ================= SCREEN INDICES ================= */
static int stopwatch_screen_index    = -1;
static int countdown_screen_index    = -1;
static int battery_screen_index      = -1;
static int worldclock_screen_index   = -1;
static int pixelrain_screen_index    = -1;
static int bigdigit_screen_index     = -1;
static int conway_screen_index       = -1;
static int snake_screen_index        = -1;
static int typewriter_screen_index   = -1;
static int pongclock_screen_index    = -1;

/* ================= EXTERN TIMERS ================= */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* ================= LOW BATTERY / OVERDISCHARGE STATE ================= */

typedef enum {
    POWER_STATE_NORMAL,       /* Normal operation */
    POWER_STATE_LOW_BATTERY,  /* SOC <= 1%, show warning, 10% brightness */
    POWER_STATE_OVERDISCHARGE /* Voltage < 3.1V, screen off */
} PowerState_t;

static PowerState_t power_state = POWER_STATE_NORMAL;

/* Brightness to restore when exiting low-battery mode */
static uint8_t saved_brightness = 255;
static bool    brightness_saved = false;

/* ================= BOOT SEQUENCE ================= */

static void App_BlankShiftRegisters(void)
{
    GPIOA->BSRR = A1_Pin | A2_Pin | A3_Pin | A4_Pin
                 | A5_Pin | A6_Pin | A7_Pin;

    GPIOA->BRR = DATA_Pin;
    for (int i = 0; i < 84; i++) {
        GPIOA->BSRR = SRCLK_Pin;
        GPIOA->BRR  = SRCLK_Pin;
    }
    GPIOA->BSRR = RCLK_Pin;
    GPIOA->BRR  = RCLK_Pin;
}

static void App_BootAnimation(void)
{
    Matrix_SetBrightness(0);

    uint8_t logo_buf[NUM_ROWS][TOTAL_BYTES];
    memset(logo_buf, 0, sizeof(logo_buf));
    Matrix_DrawBitmap_Buf(logo_buf, kompressor_logo);
    Matrix_LoadBuffer(logo_buf);

    HAL_Delay(250);

    uint32_t fade_start = HAL_GetTick();
    uint8_t last_b = 0;

    while (1) {
        uint32_t elapsed = HAL_GetTick() - fade_start;
        if (elapsed >= 500) break;

        uint8_t b = (uint8_t)((uint32_t)elapsed * 255 / 500);
        if (b != last_b) {
            last_b = b;
            Matrix_SetBrightness(b);
        }
        HAL_Delay(5);
    }
    Matrix_SetBrightness(255);

    HAL_Delay(1000);
}

static void App_RegisterScreens(void)
{
    Screen_Init();

    /* ---- Clock face screens ---- */
    Screen_Register(Screen_Time);
    Screen_Register(Screen_TimeDate);
    Screen_Register(Screen_TimeDateCompact);
    Screen_Register(Screen_TimeTempHumid);

    worldclock_screen_index  = Screen_Register(Screen_WorldClock);
    Rotary_SetWorldClockScreenIndex(worldclock_screen_index);
    WorldClock_Init();

    bigdigit_screen_index    = Screen_Register(Screen_BigDigit);
    typewriter_screen_index  = Screen_Register(Screen_Typewriter);
    pongclock_screen_index   = Screen_Register(Screen_PongClock);

    /* ---- Animated screens ---- */
    pixelrain_screen_index   = Screen_Register(Screen_PixelRain);
    conway_screen_index      = Screen_Register(Screen_Conway);
    Rotary_SetConwayScreenIndex(conway_screen_index);
    snake_screen_index       = Screen_Register(Screen_Snake);

    stopwatch_screen_index   = Screen_Register(Screen_Stopwatch);
    countdown_screen_index   = Screen_Register(Screen_Countdown);

    Rotary_SetStopwatchScreenIndex(stopwatch_screen_index);
    Rotary_SetCountdownScreenIndex(countdown_screen_index);

    /* ---- Utility screens ---- */
    battery_screen_index     = Screen_Register(Screen_Battery);
    Rotary_SetBatteryScreenIndex(battery_screen_index);

    /* NOTE: Battery debug is NOT registered as a screen.
     * It renders as an overlay inside Screen_Battery() when
     * the developer unlocks it via 6 button presses. */
}

/* ================= LOW BATTERY HELPERS ================= */

/**
 * @brief  Enter low-battery warning mode.
 *         Saves current brightness and drops to 10% (~25/255).
 */
static void App_EnterLowBattery(void)
{
    if (power_state == POWER_STATE_LOW_BATTERY) return;

    /* Save brightness only if coming from normal mode */
    if (power_state == POWER_STATE_NORMAL && !brightness_saved) {
        saved_brightness = Matrix_GetBrightness();
        brightness_saved = true;
    }

    power_state = POWER_STATE_LOW_BATTERY;

    /* 10% brightness ≈ 25 out of 255 */
    Matrix_SetBrightness(25);
}

/**
 * @brief  Enter overdischarge protection mode.
 *         Turns the screen completely off to save battery.
 */
static void App_EnterOverdischarge(void)
{
    if (power_state == POWER_STATE_OVERDISCHARGE) return;

    /* Save brightness if not already saved */
    if (!brightness_saved) {
        saved_brightness = Matrix_GetBrightness();
        brightness_saved = true;
    }

    power_state = POWER_STATE_OVERDISCHARGE;

    /* Turn display off */
    Matrix_SetBrightness(0);
}

/**
 * @brief  Resume normal operation.
 *         Restores brightness and returns to normal screen flow.
 */
static void App_ResumeNormal(void)
{
    if (power_state == POWER_STATE_NORMAL) return;

    power_state = POWER_STATE_NORMAL;

    /* Restore previous brightness */
    if (brightness_saved) {
        /* If auto brightness is on, just re-enable it — the sensor manager
         * will take over on the next update cycle. If manual, restore saved. */
        if (SensorManager_IsAutoBrightness()) {
            /* Setting any non-zero value kicks it back on; the auto system
             * will smoothly take over within ~30ms. */
            Matrix_SetBrightness(saved_brightness > 0 ? saved_brightness : 128);
        } else {
            Matrix_SetBrightness(saved_brightness > 0 ? saved_brightness : 128);
        }
        brightness_saved = false;
    } else {
        Matrix_SetBrightness(128);
    }

    Screen_MarkDirty();
}

/**
 * @brief  Check battery state and transition between power modes.
 *         Called every App_Update() cycle.
 */
static void App_CheckPowerState(void)
{
    const SensorData_t *data = SensorManager_GetData();
    bool charging = (data->current_mA >= 0);
    uint16_t voltage = (uint16_t)data->voltage_mV;
    uint16_t soc = (uint16_t)data->soc_percent;

    switch (power_state) {
        case POWER_STATE_NORMAL:
            /* Check for overdischarge first (higher priority) */
            if (!charging && voltage > 0 && voltage < 3100) {
                App_EnterOverdischarge();
            }
            /* Then check for low SOC */
            else if (!charging && soc <= 1) {
                App_EnterLowBattery();
            }
            break;

        case POWER_STATE_LOW_BATTERY:
            /* Overdischarge takes priority */
            if (!charging && voltage > 0 && voltage < 3100) {
                App_EnterOverdischarge();
            }
            /* Resume if charging or SOC recovered */
            else if (charging || soc > 1) {
                App_ResumeNormal();
            }
            break;

        case POWER_STATE_OVERDISCHARGE:
            /* Resume if plugged in */
            if (charging) {
                App_ResumeNormal();
            }
            /* If voltage recovers above threshold (with hysteresis) and SOC > 1 */
            else if (voltage >= 3200 && soc > 1) {
                App_ResumeNormal();
            }
            /* If voltage recovers but SOC still critical, go to warning mode */
            else if (voltage >= 3200 && soc <= 1) {
                App_EnterLowBattery();
            }
            break;
    }
}

/* ================= PUBLIC API ================= */

void App_Init(I2C_HandleTypeDef *hi2c)
{
    Matrix_Init();
    App_BlankShiftRegisters();

    Rotary_Init();
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

    SensorManager_Init(hi2c);

    App_BootAnimation();

    SensorManager_Update();

    App_RegisterScreens();

    Buzzer_Init();
    Stopwatch_Init();
    Countdown_Init();

    Settings_LoadFromEEPROM();

    uint8_t saved = Settings_GetSavedScreen();
    Screen_SetCurrent(saved);
    Screen_BootDissolve();

    power_state = POWER_STATE_NORMAL;
    brightness_saved = false;
}

void App_Update(void)
{
    SensorManager_Update();

    /* ---- Check battery power state ---- */
    App_CheckPowerState();

    /* ---- Overdischarge: screen off, skip all rendering ---- */
    if (power_state == POWER_STATE_OVERDISCHARGE) {
        /* Still update sensors so we detect charger plug-in,
         * but don't drive the display or process UI. */
        HAL_Delay(100);  /* Slow down loop to save power */
        return;
    }

    /* ---- Low battery: show warning screen only ---- */
    if (power_state == POWER_STATE_LOW_BATTERY) {
        /* Render the alternating warning message */
        uint8_t buf[NUM_ROWS][TOTAL_BYTES];
        memset(buf, 0, sizeof(buf));
        Screen_LowBatteryWarning(buf);
        Matrix_LoadBuffer(buf);

        /* Keep brightness at 10% — re-assert in case auto-brightness tried
         * to change it during SensorManager_Update() */
        Matrix_SetBrightness(25);

        HAL_Delay(50);  /* Slower update rate to save power */
        return;
    }

    /* ================= NORMAL OPERATION (unchanged) ================= */

    Stopwatch_Update();
    Countdown_Update();

    Buzzer_Update();

    if (Screen_GetCurrent() == stopwatch_screen_index && Stopwatch_NeedsRedraw())
        Screen_MarkDirty();
    if (Screen_GetCurrent() == countdown_screen_index && Countdown_NeedsRedraw())
        Screen_MarkDirty();
    if (Screen_GetCurrent() == worldclock_screen_index && WorldClock_NeedsRedraw())
        Screen_MarkDirty();

    Rotary_Update();

    /* Save screen index after dwelling 5 minutes */
    {
        static int tracked_screen = -1;
        static uint32_t screen_arrived = 0;
        static bool screen_saved = false;

        int cur = Screen_GetCurrent();

        if (cur != tracked_screen) {
            tracked_screen = cur;
            screen_arrived = HAL_GetTick();
            screen_saved = false;
        } else if (!screen_saved && (HAL_GetTick() - screen_arrived) >= 300000) {
            Settings_SetSavedScreen((uint8_t)cur);
            Settings_SaveToEEPROM();
            screen_saved = true;
        }
    }

    if (SensorManager_HasChanged())
        Screen_MarkDirty();

    /* Continuously animated screens */
    {
        int cur = Screen_GetCurrent();
        if (cur == pixelrain_screen_index    ||
            cur == bigdigit_screen_index     ||
            cur == conway_screen_index       ||
            cur == snake_screen_index        ||
            cur == typewriter_screen_index   ||
            cur == pongclock_screen_index) {
            Screen_MarkDirty();
        }
    }

    /* Battery screen: mark dirty when charging (animation) or debug active */
    if (battery_screen_index >= 0 && Screen_GetCurrent() == battery_screen_index) {
        const SensorData_t *batt_data = SensorManager_GetData();
        if (batt_data->current_mA >= 0) {
            Screen_MarkDirty();
        }
        if (Screen_Battery_IsAlt() && batt_data->current_mA >= 0) {
            Screen_MarkDirty();
        }
    }

    /* Auto-switch to battery screen when charger plugged in */
    {
        static bool was_charging = false;
        bool charging_now = (SensorManager_GetData()->current_mA >= 0);

        if (charging_now && !was_charging && battery_screen_index >= 0) {
            if (!Screen_IsTransitioning() &&
                Settings_GetState() == SETTINGS_STATE_INACTIVE) {
                Screen_GoTo(battery_screen_index, TRANSITION_DISSOLVE);
            }
        }
        was_charging = charging_now;
    }

    Screen_Update();

    HAL_Delay(5);
}
