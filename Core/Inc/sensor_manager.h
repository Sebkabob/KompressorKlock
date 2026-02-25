#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* ================= SENSOR DATA STRUCTURE ================= */
typedef struct {
    // Time
    uint8_t hours_12;      // 12-hour format for display
    uint8_t hours_24;      // 24-hour format for logic
    uint8_t minutes;
    uint8_t seconds;

    // Environment
    int temp_f;
    int temp_c;
    int humidity;
    int light_percent;

    // Battery
    int soc_percent;
    int current_mA;
    int voltage_mV;

    // Charger status
    bool charger_fault;
    bool charger_fault_recoverable;
    bool charger_fault_latchoff;
} SensorData_t;

/* ================= PUBLIC API ================= */

void SensorManager_Init(I2C_HandleTypeDef *hi2c);
void SensorManager_Update(void);
const SensorData_t* SensorManager_GetData(void);
bool SensorManager_HasChanged(void);

/* ================= DEBUG ACCESSORS ================= */

uint8_t SensorManager_GetMappedBrightness(void);
uint8_t SensorManager_GetCurrentBrightness(void);

/* ================= BRIGHTNESS MODE CONTROL ================= */

void SensorManager_SetAutoBrightness(bool enabled);
bool SensorManager_IsAutoBrightness(void);
void SensorManager_SetManualBrightnessPercent(uint8_t percent);
uint8_t SensorManager_GetManualBrightnessPercent(void);

#endif // SENSOR_MANAGER_H
