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

/**
 * @brief Initialize all sensors
 * @param hi2c Pointer to I2C handle
 */
void SensorManager_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Update all sensor readings (call from main loop)
 */
void SensorManager_Update(void);

/**
 * @brief Get pointer to current sensor data
 * @return Pointer to sensor data structure (read-only)
 */
const SensorData_t* SensorManager_GetData(void);

/**
 * @brief Check if sensor data has changed since last check
 * @return true if data changed, false otherwise
 */
bool SensorManager_HasChanged(void);

#endif // SENSOR_MANAGER_H
