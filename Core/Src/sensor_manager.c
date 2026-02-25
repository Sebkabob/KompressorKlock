#include "sensor_manager.h"
#include "rtc_rv3032.h"
#include "ltr_329.h"
#include "sht40.h"
#include "battery.h"
#include "matrix.h"
#include "main.h"
#include <string.h>

/* ================= CHARGER STATUS (BQ25120) ================= */
typedef enum {
    CHARGER_COMPLETE_OR_DISABLED,
    CHARGER_CHARGING,
    CHARGER_FAULT_RECOVERABLE,
    CHARGER_FAULT_LATCHOFF
} ChargerStatus_t;

/* ================= BRIGHTNESS BREAKPOINT MAP ================= */
typedef struct {
    uint8_t light_percent;
    uint8_t brightness;
} BrightnessBreakpoint_t;

#define NUM_BREAKPOINTS 5

static bool auto_brightness_enabled = true;
static uint8_t manual_brightness_percent = 50;

static const BrightnessBreakpoint_t brightness_map[NUM_BREAKPOINTS] = {
    {    0,         1  },
    {   6,         70  },
    {   12,       130  },
    {   20,       255  },
    {  100,       255  },
};

static uint8_t map_brightness(uint8_t light_pct)
{
    if (light_pct <= brightness_map[0].light_percent) {
        return brightness_map[0].brightness;
    }
    if (light_pct >= brightness_map[NUM_BREAKPOINTS - 1].light_percent) {
        return brightness_map[NUM_BREAKPOINTS - 1].brightness;
    }
    for (int i = 0; i < NUM_BREAKPOINTS - 1; i++) {
        uint8_t x0 = brightness_map[i].light_percent;
        uint8_t x1 = brightness_map[i + 1].light_percent;
        if (light_pct >= x0 && light_pct <= x1) {
            uint8_t y0 = brightness_map[i].brightness;
            uint8_t y1 = brightness_map[i + 1].brightness;
            uint16_t result = y0 + ((uint16_t)(y1 - y0) * (light_pct - x0)) / (x1 - x0);
            return (uint8_t)result;
        }
    }
    return brightness_map[NUM_BREAKPOINTS - 1].brightness;
}

/* ================= BRIGHTNESS SMOOTHING ================= */
#define SMOOTH_INTERVAL_MS   30
#define SMOOTH_DIVISOR       8

static int16_t target_brightness = 220;
static int16_t current_brightness_x16 = (int16_t)220 << 4;
static uint32_t last_smooth_tick = 0;

static uint8_t debug_mapped_brightness = 0;
static uint8_t debug_current_brightness = 0;

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

    if (now - last_update < 50) return;
    last_update = now;

    if (RV3032_UpdateTime()) {
        sensor_data.hours_12 = RV3032_GetHours();
        sensor_data.minutes = RV3032_GetMinutes();
        sensor_data.seconds = RV3032_GetSeconds();

        sensor_data.hours_24 = RV3032_BCDtoDEC(RV3032_ReadRegister(RV3032_HOURS));
    }
}

static void update_temperature(void)
{
    static uint32_t last_update = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_update < 100) return;
    last_update = now;

    SHT40_UpdateReadings();

    #define TEMP_HYSTERESIS  0.25f
    #define HUM_HYSTERESIS   0.25f

    float raw_temp_f = SHT40_GetTemperatureF();
    float raw_temp_c = SHT40_GetTemperatureC();
    float raw_humidity = SHT40_GetHumidity();

    float temp_diff_f = raw_temp_f - (float)sensor_data.temp_f;
    if (temp_diff_f > TEMP_HYSTERESIS || temp_diff_f < -TEMP_HYSTERESIS) {
        sensor_data.temp_f = (int)raw_temp_f;
    }

    float temp_diff_c = raw_temp_c - (float)sensor_data.temp_c;
    if (temp_diff_c > TEMP_HYSTERESIS || temp_diff_c < -TEMP_HYSTERESIS) {
        sensor_data.temp_c = (int)raw_temp_c;
    }

    float hum_diff = raw_humidity - (float)sensor_data.humidity;
    if (hum_diff > HUM_HYSTERESIS || hum_diff < -HUM_HYSTERESIS) {
        sensor_data.humidity = (int)raw_humidity;
    }
}

static void update_light(void)
{
    static uint32_t last_update = 0;
    static uint32_t last_sensor_read = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_sensor_read >= 100) {
        LTR_329_UpdateReadings();
        last_sensor_read = now;

        if (auto_brightness_enabled) {
            uint8_t light = LTR_329_GetLightPercent();
            uint8_t mapped = map_brightness(light);
            target_brightness = mapped;
            debug_mapped_brightness = mapped;
        }
    }

    if (!auto_brightness_enabled) {
        if (now - last_update >= 77) {
            last_update = now;
            sensor_data.light_percent = LTR_329_GetLightPercent();
        }
        return;
    }

    if (now - last_smooth_tick >= SMOOTH_INTERVAL_MS) {
        last_smooth_tick = now;

        int16_t target_x16 = target_brightness << 4;
        int16_t diff = target_x16 - current_brightness_x16;

        if (diff != 0) {
            int16_t step;
            if (diff > 0) {
                step = diff / SMOOTH_DIVISOR;
                if (step < 1) step = 1;
                current_brightness_x16 += step;
                if (current_brightness_x16 > target_x16)
                    current_brightness_x16 = target_x16;
            } else {
                step = (-diff) / SMOOTH_DIVISOR;
                if (step < 1) step = 1;
                current_brightness_x16 -= step;
                if (current_brightness_x16 < target_x16)
                    current_brightness_x16 = target_x16;
            }
        }

        uint8_t brightness = (uint8_t)(current_brightness_x16 >> 4);
        debug_current_brightness = brightness;
        Matrix_SetBrightness(brightness);
    }

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

    RV3032_Init(hi2c);
    SHT40_Init(hi2c);
    LTR_329_Init(hi2c);
    BATTERY_Init();

    memset(&sensor_data, 0, sizeof(sensor_data));
    memset(&prev_data, 0, sizeof(prev_data));
    data_changed = false;

    uint8_t initial = brightness_map[NUM_BREAKPOINTS - 1].brightness;
    target_brightness = initial;
    current_brightness_x16 = (int16_t)initial << 4;
    last_smooth_tick = HAL_GetTick();
    Matrix_SetBrightness(initial);
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
    data_changed = false;
    return changed;
}

void SensorManager_SetAutoBrightness(bool enabled)
{
    auto_brightness_enabled = enabled;
    if (!enabled) {
        uint8_t brightness = (uint8_t)((uint16_t)manual_brightness_percent * 255 / 100);
        if (brightness < 1) brightness = 1;
        target_brightness = brightness;
        current_brightness_x16 = (int16_t)brightness << 4;
        Matrix_SetBrightness(brightness);
        debug_current_brightness = brightness;
        debug_mapped_brightness = brightness;
    }
}

bool SensorManager_IsAutoBrightness(void)
{
    return auto_brightness_enabled;
}

void SensorManager_SetManualBrightnessPercent(uint8_t percent)
{
    if (percent < 1) percent = 1;
    if (percent > 100) percent = 100;
    manual_brightness_percent = percent;

    if (!auto_brightness_enabled) {
        uint8_t brightness = (uint8_t)((uint16_t)percent * 255 / 100);
        if (brightness < 1) brightness = 1;
        target_brightness = brightness;
        current_brightness_x16 = (int16_t)brightness << 4;
        Matrix_SetBrightness(brightness);
        debug_current_brightness = brightness;
        debug_mapped_brightness = brightness;
    }
}

uint8_t SensorManager_GetManualBrightnessPercent(void)
{
    return manual_brightness_percent;
}

uint8_t SensorManager_GetMappedBrightness(void)
{
    return debug_mapped_brightness;
}

uint8_t SensorManager_GetCurrentBrightness(void)
{
    return debug_current_brightness;
}
