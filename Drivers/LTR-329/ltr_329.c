#include "ltr_329.h"

/* Private variables */
static I2C_HandleTypeDef *ltr_329_hi2c = NULL;
static LTR_329_Measurement_t last_measurement = {0, 0};
static uint8_t current_gain = LTR_329_GAIN_1X;

/* Averaging buffer for smooth dimming */
#define AVG_SAMPLES 20  /* 20 samples at 100ms = 2 seconds */
static uint8_t light_buffer[AVG_SAMPLES];
static uint8_t buffer_index = 0;
static uint8_t buffer_filled = 0;  /* Track if buffer has been filled once */

/* I2C timeout in ms */
#define LTR_329_I2C_TIMEOUT  100

/* Maximum raw value for scaling (depends on gain and integration time) */
#define LTR_329_MAX_RAW_VALUE  40  /* Adjusted for high sensitivity */

/* Timer period range */
#define PERIOD_MIN  110  /* Period at 0% light */
#define PERIOD_MAX  259  /* Period at 100% light */

/* Private function prototypes */
static bool LTR_329_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value);
static bool LTR_329_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);
static bool LTR_329_ReadRegister16(I2C_HandleTypeDef *hi2c, uint8_t reg_low, uint16_t *value);

/**
 * @brief Write a single byte to a register
 */
static bool LTR_329_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(hi2c, LTR_329_I2C_ADDR, data, 2, LTR_329_I2C_TIMEOUT) == HAL_OK;
}

/**
 * @brief Read a single byte from a register
 */
static bool LTR_329_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    if (HAL_I2C_Master_Transmit(hi2c, LTR_329_I2C_ADDR, &reg, 1, LTR_329_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    return HAL_I2C_Master_Receive(hi2c, LTR_329_I2C_ADDR, value, 1, LTR_329_I2C_TIMEOUT) == HAL_OK;
}

/**
 * @brief Read 16-bit value from two consecutive registers (low byte first)
 */
static bool LTR_329_ReadRegister16(I2C_HandleTypeDef *hi2c, uint8_t reg_low, uint16_t *value)
{
    uint8_t data[2];

    if (HAL_I2C_Master_Transmit(hi2c, LTR_329_I2C_ADDR, &reg_low, 1, LTR_329_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }
    if (HAL_I2C_Master_Receive(hi2c, LTR_329_I2C_ADDR, data, 2, LTR_329_I2C_TIMEOUT) != HAL_OK) {
        return false;
    }

    *value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return true;
}

bool LTR_329_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t control_reg;
    uint16_t retry_count = 0;

    if (hi2c == NULL) {
        return false;
    }

    ltr_329_hi2c = hi2c;

    /* Initialize averaging buffer to 0 */
    for (int i = 0; i < AVG_SAMPLES; i++) {
        light_buffer[i] = 0;
    }
    buffer_index = 0;
    buffer_filled = 0;

    /* Check if device is present by reading manufacturer ID */
    uint8_t mfg_id = LTR_329_ReadManufacturerID(hi2c);
    if (mfg_id != 0x05) {
        return false;
    }

    /* Wait for device to be ready (check SW reset bit) */
    while (retry_count++ < 500) {
        if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_CONTR, &control_reg)) {
            return false;
        }

        /* Check if bit 1 (SW reset) is cleared */
        if ((control_reg & 0x02) == 0) {
            break;
        }

        HAL_Delay(1);
    }

    if (retry_count >= 500) {
        return false;
    }

    /* Set gain to 1x (good for indoor use) */
    if (!LTR_329_SetGain(hi2c, LTR_329_GAIN_1X)) {
        return false;
    }

    /* Set integration time to 100ms */
    if (!LTR_329_SetIntegrationTime(hi2c, LTR_329_INT_TIME_100MS)) {
        return false;
    }

    /* Set measurement rate to 100ms */
    if (!LTR_329_SetMeasurementRate(hi2c, LTR_329_MEAS_RATE_100MS)) {
        return false;
    }

    /* Activate the sensor */
    if (!LTR_329_SetMode(hi2c, LTR_329_MODE_ACTIVE)) {
        return false;
    }

    /* Wait for first measurement */
    HAL_Delay(110);

    return true;
}

bool LTR_329_SetGain(I2C_HandleTypeDef *hi2c, LTR_329_Gain_t gain)
{
    uint8_t control_reg;

    if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_CONTR, &control_reg)) {
        return false;
    }

    /* Clear gain bits (4:2) and set new gain */
    control_reg = (control_reg & 0xE3) | ((gain & 0x07) << 2);

    if (!LTR_329_WriteRegister(hi2c, LTR_329_REG_CONTR, control_reg)) {
        return false;
    }

    current_gain = gain;
    return true;
}

bool LTR_329_SetIntegrationTime(I2C_HandleTypeDef *hi2c, LTR_329_IntTime_t time)
{
    uint8_t rate_reg;

    if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_MEAS_RATE, &rate_reg)) {
        return false;
    }

    /* Clear integration time bits (5:3) and set new time */
    rate_reg = (rate_reg & 0xC7) | ((time & 0x07) << 3);

    return LTR_329_WriteRegister(hi2c, LTR_329_REG_MEAS_RATE, rate_reg);
}

bool LTR_329_SetMeasurementRate(I2C_HandleTypeDef *hi2c, LTR_329_MeasRate_t rate)
{
    uint8_t rate_reg;

    if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_MEAS_RATE, &rate_reg)) {
        return false;
    }

    /* Clear measurement rate bits (2:0) and set new rate */
    rate_reg = (rate_reg & 0xF8) | (rate & 0x07);

    return LTR_329_WriteRegister(hi2c, LTR_329_REG_MEAS_RATE, rate_reg);
}

bool LTR_329_SetMode(I2C_HandleTypeDef *hi2c, LTR_329_Mode_t mode)
{
    uint8_t control_reg;

    if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_CONTR, &control_reg)) {
        return false;
    }

    /* Clear mode bit (0) and set new mode */
    control_reg = (control_reg & 0xFE) | (mode & 0x01);

    return LTR_329_WriteRegister(hi2c, LTR_329_REG_CONTR, control_reg);
}

bool LTR_329_ReadMeasurement(I2C_HandleTypeDef *hi2c, LTR_329_Measurement_t *measurement)
{
    if (measurement == NULL) {
        return false;
    }

    /*
     * Important: The order of reading matters!
     * The sensor locks the data registers when reading CH1_0.
     * The lock is released after reading CH0_1.
     * So we must read: CH1_0, CH1_1, CH0_0, CH0_1
     */

    /* Read Channel 1 (IR) first */
    if (!LTR_329_ReadRegister16(hi2c, LTR_329_REG_DATA_CH1_0, &measurement->channel1)) {
        return false;
    }

    /* Read Channel 0 (Visible + IR) - this releases the lock */
    if (!LTR_329_ReadRegister16(hi2c, LTR_329_REG_DATA_CH0_0, &measurement->channel0)) {
        return false;
    }

    return true;
}

bool LTR_329_ReadStatus(I2C_HandleTypeDef *hi2c, LTR_329_Status_t *status)
{
    uint8_t status_reg;

    if (status == NULL) {
        return false;
    }

    if (!LTR_329_ReadRegister(hi2c, LTR_329_REG_STATUS, &status_reg)) {
        return false;
    }

    status->data_valid = (status_reg >> 7) & 0x01;
    status->gain = (status_reg >> 4) & 0x07;
    status->data_status = (status_reg >> 2) & 0x01;

    return true;
}

uint8_t LTR_329_ReadPartID(I2C_HandleTypeDef *hi2c)
{
    uint8_t part_id = 0;
    LTR_329_ReadRegister(hi2c, LTR_329_REG_PART_ID, &part_id);
    return part_id;
}

uint8_t LTR_329_ReadManufacturerID(I2C_HandleTypeDef *hi2c)
{
    uint8_t mfg_id = 0;
    LTR_329_ReadRegister(hi2c, LTR_329_REG_MANUFACTURER_ID, &mfg_id);
    return mfg_id;
}

bool LTR_329_UpdateReadings(void)
{
    if (ltr_329_hi2c == NULL) {
        return false;
    }

    /* Read measurement */
    if (!LTR_329_ReadMeasurement(ltr_329_hi2c, &last_measurement)) {
        return false;
    }

    /* Calculate current light percent */
    uint32_t visible;
    uint32_t percent;

    if (last_measurement.channel0 > last_measurement.channel1) {
        visible = last_measurement.channel0 - last_measurement.channel1;
    } else {
        visible = last_measurement.channel0;
    }

    percent = (visible * 100) / LTR_329_MAX_RAW_VALUE;
    if (percent > 100) {
        percent = 100;
    }

    /* Add to circular buffer */
    light_buffer[buffer_index] = (uint8_t)percent;
    buffer_index++;

    if (buffer_index >= AVG_SAMPLES) {
        buffer_index = 0;
        buffer_filled = 1;
    }

    return true;
}

uint8_t LTR_329_GetLightPercent(void)
{
    uint32_t sum = 0;
    uint8_t count;

    /* Use all samples if buffer has been filled, otherwise use what we have */
    if (buffer_filled) {
        count = AVG_SAMPLES;
    } else {
        count = buffer_index;
        if (count == 0) {
            return 0;
        }
    }

    for (uint8_t i = 0; i < count; i++) {
        sum += light_buffer[i];
    }

    return (uint8_t)(sum / count);
}

uint16_t LTR_329_GetTimerPeriod(void)
{
    uint8_t light_percent = LTR_329_GetLightPercent();

    /* Map 0-100% to PERIOD_MIN-PERIOD_MAX */
    /* At 0% light -> PERIOD_MIN (110) */
    /* At 100% light -> PERIOD_MAX (259) */
    uint16_t period = PERIOD_MIN + ((uint32_t)(PERIOD_MAX - PERIOD_MIN) * light_percent) / 100;

    return period;
}

uint16_t LTR_329_GetChannel0(void)
{
    return last_measurement.channel0;
}

uint16_t LTR_329_GetChannel1(void)
{
    return last_measurement.channel1;
}
