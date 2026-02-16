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
    static uint32_t last_brightness_update = 0;
    uint32_t now = HAL_GetTick();

    // Update brightness from light sensor every 100ms
    if (now - last_brightness_update >= 100) {
        LTR_329_UpdateReadings();
        last_brightness_update = now;

        /*
         * Map light sensor to brightness (0-255).
         *
         * You'll want to tune this mapping to your light sensor's range.
         * LTR_329_GetLightPercent() returns 0-100.
         *
         * Example mapping:
         *   0% light  ->  brightness 3   (very dim but still visible)
         *   100% light -> brightness 255  (full brightness)
         *
         * The quadratic curve inside Matrix_SetBrightness() already handles
         * perceptual linearity, so a linear mapping here works well.
         */
        int light = LTR_329_GetLightPercent();

        /* Linear map: light 0-100 -> brightness 3-255 */
        uint8_t brightness;
        if (light <= 0) {
            brightness = 3;   /* minimum visible brightness */
        } else if (light >= 100) {
            brightness = 255;
        } else {
            brightness = (uint8_t)(3 + (light * 252) / 100);
        }

        Matrix_SetBrightness(brightness);
    }

    // Update light percentage for display every 77ms
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
