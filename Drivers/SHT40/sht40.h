#ifndef SHT40_H
#define SHT40_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* SHT40 I2C Address (7-bit: 0x44, 8-bit write: 0x88, read: 0x89) */
#define SHT40_I2C_ADDR          0x44

/* SHT40 Commands */
#define SHT40_CMD_MEASURE_HIGH  0xFD  /* High precision measurement */
#define SHT40_CMD_MEASURE_MED   0xF6  /* Medium precision measurement */
#define SHT40_CMD_MEASURE_LOW   0xE0  /* Low precision measurement */
#define SHT40_CMD_READ_SERIAL   0x89  /* Read serial number */
#define SHT40_CMD_SOFT_RESET    0x94  /* Soft reset */

/**
 * @brief Initialize the SHT40 sensor
 * @param hi2c Pointer to I2C handle
 * @retval true if successful, false otherwise
 */
bool SHT40_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief Read temperature and humidity from SHT40
 * @param temp_c Pointer to store temperature in Celsius
 * @param humidity Pointer to store relative humidity in %
 * @retval true if successful, false otherwise
 */
bool SHT40_Read(float *temp_c, float *humidity);

/**
 * @brief Get temperature in Celsius
 * @retval Temperature in Celsius
 */
float SHT40_GetTemperatureC(void);

/**
 * @brief Get temperature in Fahrenheit
 * @retval Temperature in Fahrenheit
 */
float SHT40_GetTemperatureF(void);

/**
 * @brief Get relative humidity
 * @retval Relative humidity in %
 */
float SHT40_GetHumidity(void);

/**
 * @brief Update sensor readings (call periodically)
 * @retval true if successful, false otherwise
 */
bool SHT40_UpdateReadings(void);

/**
 * @brief Perform soft reset of the sensor
 * @retval true if successful, false otherwise
 */
bool SHT40_SoftReset(void);

/**
 * @brief Read the sensor serial number
 * @param serial Pointer to store 32-bit serial number
 * @retval true if successful, false otherwise
 */
bool SHT40_ReadSerial(uint32_t *serial);

#endif /* SHT40_H */
