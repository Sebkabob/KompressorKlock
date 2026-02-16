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

/**
 * @brief Get the brightness value from breakpoint mapping (before smoothing)
 * @return Target brightness (0-255)
 */
uint8_t SensorManager_GetMappedBrightness(void);

/**
 * @brief Get the actual current brightness (after smoothing)
 * @return Current brightness (0-255)
 */
uint8_t SensorManager_GetCurrentBrightness(void);

#endif // SENSOR_MANAGER_H
