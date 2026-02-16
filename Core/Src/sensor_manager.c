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
/*
 * Tunable brightness curve using breakpoints.
 *
 * Each entry maps a light_percent to a brightness value (0-255).
 * The system linearly interpolates between adjacent breakpoints.
 *
 * To tune: adjust the brightness values to taste.
 * - Lower brightness values = dimmer at that light level
 * - The light_percent values define the "zones"
 *
 * Example: if your room at night reads ~5% light and you want the
 * display barely visible, set the first breakpoint brightness low.
 * If normal room lighting reads ~40%, set that breakpoint to your
 * preferred "comfortable reading" brightness.
 */
typedef struct {
    uint8_t light_percent;   /* Input: filtered light level (0-100) */
    uint8_t brightness;      /* Output: display brightness (0-255) */
} BrightnessBreakpoint_t;

#define NUM_BREAKPOINTS 5

static const BrightnessBreakpoint_t brightness_map[NUM_BREAKPOINTS] = {
    /*  light%   brightness
     *  ------   ----------  */
    {    0,         1  },   /* Pitch dark: very dim but still readable */
    {   6,         70  },   /* Dark room (nighttime, lights off nearby) */
    {   14,        95  },   /* Dim room (evening, indirect lighting) */
    {   35,       150  },   /* Normal room (daytime, overhead lights) */
    {  100,       255  },   /* Bright (direct light, window, lamp nearby) */
};

/**
 * @brief Map light percent to brightness using breakpoint interpolation
 * @param light_pct Filtered light percentage (0-100)
 * @return Brightness value (0-255)
 */
static uint8_t map_brightness(uint8_t light_pct)
{
    /* Below first breakpoint */
    if (light_pct <= brightness_map[0].light_percent) {
        return brightness_map[0].brightness;
    }

    /* Above last breakpoint */
    if (light_pct >= brightness_map[NUM_BREAKPOINTS - 1].light_percent) {
        return brightness_map[NUM_BREAKPOINTS - 1].brightness;
    }

    /* Find which two breakpoints we're between */
    for (int i = 0; i < NUM_BREAKPOINTS - 1; i++) {
        uint8_t x0 = brightness_map[i].light_percent;
        uint8_t x1 = brightness_map[i + 1].light_percent;

        if (light_pct >= x0 && light_pct <= x1) {
            /* Linear interpolation between breakpoints */
            uint8_t y0 = brightness_map[i].brightness;
            uint8_t y1 = brightness_map[i + 1].brightness;

            uint16_t result = y0 + ((uint16_t)(y1 - y0) * (light_pct - x0)) / (x1 - x0);
            return (uint8_t)result;
        }
    }

    /* Shouldn't reach here, but return max as fallback */
    return brightness_map[NUM_BREAKPOINTS - 1].brightness;
}

/* ================= BRIGHTNESS SMOOTHING ================= */
/*
 * Smooth transitions between brightness levels to prevent flicker.
 *
 * target_brightness: where the breakpoint map wants us (0-255)
 * current_brightness_x16: fixed-point (4 fractional bits) for smooth steps
 *
 * Every SMOOTH_INTERVAL_MS we move 1/SMOOTH_DIVISOR toward target.
 */
#define SMOOTH_INTERVAL_MS   30    /* How often we step toward target */
#define SMOOTH_DIVISOR       8     /* Larger = slower fade */

static int16_t target_brightness = 220;
static int16_t current_brightness_x16 = (int16_t)220 << 4;
static uint32_t last_smooth_tick = 0;

/* Debug: expose current mapped brightness for the debug screen */
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

    if (now - last_update < 200) return;
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

    /* --- Read light sensor every 100ms --- */
    if (now - last_sensor_read >= 100) {
        LTR_329_UpdateReadings();
        last_sensor_read = now;

        /* Get filtered light level and map through breakpoints */
        uint8_t light = LTR_329_GetLightPercent();
        uint8_t mapped = map_brightness(light);

        target_brightness = mapped;
        debug_mapped_brightness = mapped;
    }

    /* --- Smooth brightness toward target every SMOOTH_INTERVAL_MS --- */
    if (now - last_smooth_tick >= SMOOTH_INTERVAL_MS) {
        last_smooth_tick = now;

        int16_t target_x16 = target_brightness << 4;
        int16_t diff = target_x16 - current_brightness_x16;

        if (diff == 0) {
            /* Already at target */
        } else if (diff > 0) {
            int16_t step = diff / SMOOTH_DIVISOR;
            if (step < 1) step = 1;
            current_brightness_x16 += step;
            if (current_brightness_x16 > target_x16)
                current_brightness_x16 = target_x16;
        } else {
            int16_t step = (-diff) / SMOOTH_DIVISOR;
            if (step < 1) step = 1;
            current_brightness_x16 -= step;
            if (current_brightness_x16 < target_x16)
                current_brightness_x16 = target_x16;
        }

        uint8_t brightness = (uint8_t)(current_brightness_x16 >> 4);
        debug_current_brightness = brightness;
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

    RV3032_Init(hi2c);
    SHT40_Init(hi2c);
    LTR_329_Init(hi2c);
    BATTERY_Init();

    memset(&sensor_data, 0, sizeof(sensor_data));
    memset(&prev_data, 0, sizeof(prev_data));
    data_changed = false;

    /* Start at the bright end of the map */
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

/* ================= DEBUG ACCESSORS ================= */

uint8_t SensorManager_GetMappedBrightness(void)
{
    return debug_mapped_brightness;
}

uint8_t SensorManager_GetCurrentBrightness(void)
{
    return debug_current_brightness;
}
