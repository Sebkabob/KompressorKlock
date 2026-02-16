#ifndef LTR_329_H
#define LTR_329_H

#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* I2C Address */
#define LTR_329_I2C_ADDR                    (0x29 << 1)  /* 7-bit address shifted for HAL */

/* Register Addresses */
#define LTR_329_REG_CONTR                   0x80
#define LTR_329_REG_MEAS_RATE               0x85
#define LTR_329_REG_PART_ID                 0x86
#define LTR_329_REG_MANUFACTURER_ID         0x87
#define LTR_329_REG_DATA_CH1_0              0x88
#define LTR_329_REG_DATA_CH1_1              0x89
#define LTR_329_REG_DATA_CH0_0              0x8A
#define LTR_329_REG_DATA_CH0_1              0x8B
#define LTR_329_REG_STATUS                  0x8C

/* Gain Settings (bits 4:2 of CONTR register) */
typedef enum {
    LTR_329_GAIN_1X  = 0x00,  /* 1 lux to 64k lux */
    LTR_329_GAIN_2X  = 0x01,  /* 0.5 lux to 32k lux */
    LTR_329_GAIN_4X  = 0x02,  /* 0.25 lux to 16k lux */
    LTR_329_GAIN_8X  = 0x03,  /* 0.125 lux to 8k lux */
    LTR_329_GAIN_48X = 0x06,  /* 0.02 lux to 1.3k lux */
    LTR_329_GAIN_96X = 0x07   /* 0.01 lux to 600 lux */
} LTR_329_Gain_t;

/* Integration Time Settings (bits 5:3 of MEAS_RATE register) */
typedef enum {
    LTR_329_INT_TIME_50MS  = 0x01,
    LTR_329_INT_TIME_100MS = 0x00,  /* Default */
    LTR_329_INT_TIME_150MS = 0x04,
    LTR_329_INT_TIME_200MS = 0x02,
    LTR_329_INT_TIME_250MS = 0x05,
    LTR_329_INT_TIME_300MS = 0x06,
    LTR_329_INT_TIME_350MS = 0x07,
    LTR_329_INT_TIME_400MS = 0x03
} LTR_329_IntTime_t;

/* Measurement Rate Settings (bits 2:0 of MEAS_RATE register) */
typedef enum {
    LTR_329_MEAS_RATE_50MS   = 0x00,
    LTR_329_MEAS_RATE_100MS  = 0x01,
    LTR_329_MEAS_RATE_200MS  = 0x02,
    LTR_329_MEAS_RATE_500MS  = 0x03,  /* Default */
    LTR_329_MEAS_RATE_1000MS = 0x04,
    LTR_329_MEAS_RATE_2000MS = 0x05
} LTR_329_MeasRate_t;

/* Device Mode */
typedef enum {
    LTR_329_MODE_STANDBY = 0x00,
    LTR_329_MODE_ACTIVE  = 0x01
} LTR_329_Mode_t;

/* Measurement Data Structure */
typedef struct {
    uint16_t channel0;  /* Visible + IR */
    uint16_t channel1;  /* IR only */
} LTR_329_Measurement_t;

/* Status Structure */
typedef struct {
    uint8_t data_valid;   /* 0 = valid, 1 = invalid */
    uint8_t data_status;  /* 0 = old data, 1 = new data */
    uint8_t gain;         /* Current gain setting */
} LTR_329_Status_t;

/* Function Prototypes */

bool LTR_329_Init(I2C_HandleTypeDef *hi2c);
bool LTR_329_SetGain(I2C_HandleTypeDef *hi2c, LTR_329_Gain_t gain);
bool LTR_329_SetIntegrationTime(I2C_HandleTypeDef *hi2c, LTR_329_IntTime_t time);
bool LTR_329_SetMeasurementRate(I2C_HandleTypeDef *hi2c, LTR_329_MeasRate_t rate);
bool LTR_329_SetMode(I2C_HandleTypeDef *hi2c, LTR_329_Mode_t mode);
bool LTR_329_ReadMeasurement(I2C_HandleTypeDef *hi2c, LTR_329_Measurement_t *measurement);
bool LTR_329_ReadStatus(I2C_HandleTypeDef *hi2c, LTR_329_Status_t *status);
uint8_t LTR_329_ReadPartID(I2C_HandleTypeDef *hi2c);
uint8_t LTR_329_ReadManufacturerID(I2C_HandleTypeDef *hi2c);

/**
 * @brief Update sensor readings (call every 100ms for smooth averaging)
 * @return true if new data was read
 */
bool LTR_329_UpdateReadings(void);

/**
 * @brief Get light intensity as percentage (0-100), trimmed-mean filtered
 * @return Light intensity percentage
 */
uint8_t LTR_329_GetLightPercent(void);

/**
 * @brief Get the most recent unfiltered light percentage (for debug)
 * @return Raw single-sample light percentage
 */
uint8_t LTR_329_GetLightPercentRaw(void);

/**
 * @brief Get timer period based on averaged light level (legacy)
 * @return Timer period (110 at 0% light, 259 at 100% light)
 */
uint16_t LTR_329_GetTimerPeriod(void);

uint16_t LTR_329_GetChannel0(void);
uint16_t LTR_329_GetChannel1(void);

#endif /* LTR_329_H */
