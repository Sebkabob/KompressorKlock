#include "sensor_manager.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "sht40.h"
#include "battery.h"
#include "matrix.h"
#include "main.h"
#include <string.h>

/* ================= CHARGER STATUS (BQ25120) ================= */
// STAT1=HIGH, STAT2=HIGH -> Charge complete / sleep / disabled
// STAT1=HIGH, STAT2=LOW  -> Normal charging in progress
// STAT1=LOW,  STAT2=HIGH -> Recoverable fault (OVP, TS, etc.)
// STAT1=LOW,  STAT2=LOW  -> Non-recoverable latch-off fault

typedef enum {
    CHARGER_COMPLETE_OR_DISABLED,
    CHARGER_CHARGING,
    CHARGER_FAULT_RECOVERABLE,
    CHARGER_FAULT_LATCHOFF
} ChargerStatus_t;

/* ================= BRIGHTNESS SMOOTHING ================= */
/*
 * We smooth brightness transitions to prevent glitchy jumps.
 *
 * target_brightness: where the light sensor wants us to be (0-255)
 * current_brightness_x16: fixed-point (4 fractional bits) actual brightness
 *
 * Every 30ms we move current toward target by 1/8 of the difference.
 * This gives a smooth ~200ms fade that looks natural.
 * Using fixed-point avoids rounding to zero when the step is small.
 */
#define BRIGHTNESS_MIN       5     /* minimum visible brightness */
#define BRIGHTNESS_MAX       220   /* max brightness (LUT saturates above this) */
#define SMOOTH_INTERVAL_MS   30    /* how often we step toward target */
#define SMOOTH_DIVISOR       8     /* larger = slower fade (1/N of difference per step) */

static int16_t target_brightness = BRIGHTNESS_MAX;
static int16_t current_brightness_x16 = (int16_t)BRIGHTNESS_MAX << 4;
static uint32_t last_smooth_tick = 0;

/* ================= PRIVATE STATE ================= */
static SensorData_t sensor_data = {0};
static SensorData_t prev_data = {0};
static bool data_changed = false;
static I2C_HandleTypeDef *hi2c_handle = NULL;

/* ================= PRIVATE FUNCTIONS ================= */

static void update_charger_status(void)
{
    GPIO_PinState stat1 = HAL_GPIO_ReadPin(STAT1_GPIO_Port, STAT1_Pin);
    GPIO_PinState stat2 = HAL_GPIO_ReadPin(STAT2_GPIO_Port, STAT2_Pin);

    if (stat1 == GPIO_PIN_SET && stat2 == GPIO_PIN_SET) {
        sensor_data.charger_fault = false;
        sensor_data.charger_fault_recoverable = false;
        sensor_data.charger_fault_latchoff = false;
    } else if (stat1 == GPIO_PIN_SET && stat2 == GPIO_PIN_RESET) {
        sensor_data.charger_fault = false;
        sensor_data.charger_fault_recoverable = false;
        sensor_data.charger_fault_latchoff = false;
    } else if (stat1 == GPIO_PIN_RESET && stat2 == GPIO_PIN_SET) {
        sensor_data.charger_fault = true;
        sensor_data.charger_fault_recoverable = true;
        sensor_data.charger_fault_latchoff = false;
    } else {
        sensor_data.charger_fault = true;
        sensor_data.charger_fault_recoverable = false;
        sensor_data.charger_fault_latchoff = true;
    }
}

static void update_time(void)
{
    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_update < 200) return;
    last_update = now;

    if (RV3032_UpdateTime()) {
        sensor_data.hours_12 = RV3032_GetHours();
        sensor_data.minutes = RV3032_GetMinutes();
        sensor_data.seconds = RV3032_GetSeconds();

        // Get 24-hour format for logic/calculations
        sensor_data.hours_24 = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HOURS));
    }
}

static void update_temperature(void)
{
    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_update < 300) return;
    last_update = now;

    SHT40_UpdateReadings();
    sensor_data.temp_f = (int)SHT40_GetTemperatureF();
    sensor_data.humidity = SHT40_GetHumidity();
}

static void update_light(void)
{
    static uint32_t last_update = 0;
    static uint32_t last_sensor_read = 0;
    uint32_t now = HAL_GetTick();

    /* --- Read light sensor every 100ms and set target brightness --- */
    if (now - last_sensor_read >= 100) {
        LTR_329_UpdateReadings();
        last_sensor_read = now;

        int light = LTR_329_GetLightPercent();

        /*
         * Map light sensor (0-100%) to brightness (BRIGHTNESS_MIN - BRIGHTNESS_MAX).
         * Linear mapping is fine here because the gamma LUT in matrix.c
         * handles the perceptual correction.
         */
        if (light <= 0) {
            target_brightness = BRIGHTNESS_MIN;
        } else if (light >= 100) {
            target_brightness = BRIGHTNESS_MAX;
        } else {
            target_brightness = BRIGHTNESS_MIN +
                (light * (BRIGHTNESS_MAX - BRIGHTNESS_MIN)) / 100;
        }
    }

    /* --- Smooth brightness toward target every SMOOTH_INTERVAL_MS --- */
    if (now - last_smooth_tick >= SMOOTH_INTERVAL_MS) {
        last_smooth_tick = now;

        int16_t target_x16 = target_brightness << 4;
        int16_t diff = target_x16 - current_brightness_x16;

        if (diff == 0) {
            /* Already at target, nothing to do */
        } else if (diff > 0) {
            /* Getting brighter */
            int16_t step = diff / SMOOTH_DIVISOR;
            if (step < 1) step = 1;  /* always make progress */
            current_brightness_x16 += step;
            if (current_brightness_x16 > target_x16)
                current_brightness_x16 = target_x16;
        } else {
            /* Getting dimmer */
            int16_t step = (-diff) / SMOOTH_DIVISOR;
            if (step < 1) step = 1;
            current_brightness_x16 -= step;
            if (current_brightness_x16 < target_x16)
                current_brightness_x16 = target_x16;
        }

        /* Convert fixed-point back to 0-255 and apply */
        uint8_t brightness = (uint8_t)(current_brightness_x16 >> 4);
        Matrix_SetBrightness(brightness);
    }

    /* --- Update light percentage for display every 77ms --- */
    if (now - last_update >= 77) {
        last_update = now;
        sensor_data.light_percent = LTR_329_GetLightPercent();
    }
}

static void update_battery(void)
{
    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_update < 500) return;
    last_update = now;

    BATTERY_UpdateState();
    sensor_data.soc_percent = BATTERY_GetSOC();
    sensor_data.current_mA = BATTERY_GetCurrent();
    sensor_data.voltage_mV = BATTERY_GetVoltage();
}

static void check_if_changed(void)
{
    data_changed = (memcmp(&sensor_data, &prev_data, sizeof(SensorData_t)) != 0);
    if (data_changed) {
        memcpy(&prev_data, &sensor_data, sizeof(SensorData_t));
    }
}

/* ================= PUBLIC API ================= */

void SensorManager_Init(I2C_HandleTypeDef *hi2c)
{
    hi2c_handle = hi2c;

    // Initialize RTC
    if (RV3032_Init(hi2c)) {
        // Uncomment to set initial time:
        // RV3032_SetTime(0, 0, 12, 0, 15, 2, 2026);
    }

    // Initialize temperature/humidity sensor
    SHT40_Init(hi2c);

    // Initialize light sensor
    LTR_329_Init(hi2c);

    // Initialize battery gauge
    BATTERY_Init();

    // Initialize sensor data
    memset(&sensor_data, 0, sizeof(sensor_data));
    memset(&prev_data, 0, sizeof(prev_data));
    data_changed = false;

    // Initialize brightness to a sensible default
    target_brightness = BRIGHTNESS_MAX;
    current_brightness_x16 = (int16_t)BRIGHTNESS_MAX << 4;
    last_smooth_tick = HAL_GetTick();
    Matrix_SetBrightness(BRIGHTNESS_MAX);
}

void SensorManager_Update(void)
{
    update_time();
    update_temperature();
    update_light();
    update_battery();
    update_charger_status();
    check_if_changed();
}

const SensorData_t* SensorManager_GetData(void)
{
    return &sensor_data;
}

bool SensorManager_HasChanged(void)
{
    bool changed = data_changed;
    data_changed = false;  // Clear flag after reading
    return changed;
}
