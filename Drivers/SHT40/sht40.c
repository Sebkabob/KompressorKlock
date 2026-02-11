#include "sht40.h"

/* Private variables */
static I2C_HandleTypeDef *sht40_i2c = NULL;
static float last_temp_c = 0.0f;
static float last_humidity = 0.0f;

/* Private function prototypes */
static uint8_t SHT40_CRC8(const uint8_t *data, uint16_t len);
static bool SHT40_CheckCRC(const uint8_t *data, uint16_t len, uint8_t crc);

/**
 * @brief Calculate CRC-8 checksum
 * @param data Pointer to data bytes
 * @param len Number of bytes
 * @retval CRC-8 checksum
 */
static uint8_t SHT40_CRC8(const uint8_t *data, uint16_t len)
{
    const uint8_t polynomial = 0x31;
    uint8_t crc = 0xFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ polynomial;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief Verify CRC-8 checksum
 * @param data Pointer to data bytes
 * @param len Number of bytes
 * @param crc Expected CRC value
 * @retval true if CRC matches, false otherwise
 */
static bool SHT40_CheckCRC(const uint8_t *data, uint16_t len, uint8_t crc)
{
    return (SHT40_CRC8(data, len) == crc);
}

/**
 * @brief Initialize the SHT40 sensor
 */
bool SHT40_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
        return false;

    sht40_i2c = hi2c;

    /* Check if device is present on the bus */
    if (HAL_I2C_IsDeviceReady(sht40_i2c, SHT40_I2C_ADDR << 1, 3, 100) != HAL_OK)
        return false;

    /* Perform soft reset */
    SHT40_SoftReset();
    HAL_Delay(10);

    /* Do an initial reading to verify communication */
    return SHT40_UpdateReadings();
}

/**
 * @brief Read temperature and humidity from SHT40
 */
bool SHT40_Read(float *temp_c, float *humidity)
{
    uint8_t cmd = SHT40_CMD_MEASURE_HIGH;
    uint8_t data[6];

    if (sht40_i2c == NULL)
        return false;

    /* Send measurement command */
    if (HAL_I2C_Master_Transmit(sht40_i2c, SHT40_I2C_ADDR << 1, &cmd, 1, 100) != HAL_OK)
        return false;

    /* Wait for measurement (high precision takes ~8.2ms, use 10ms to be safe) */
    HAL_Delay(10);

    /* Read 6 bytes: temp_msb, temp_lsb, temp_crc, hum_msb, hum_lsb, hum_crc */
    if (HAL_I2C_Master_Receive(sht40_i2c, SHT40_I2C_ADDR << 1, data, 6, 100) != HAL_OK)
        return false;

    /* Verify CRC for temperature */
    if (!SHT40_CheckCRC(&data[0], 2, data[2]))
        return false;

    /* Verify CRC for humidity */
    if (!SHT40_CheckCRC(&data[3], 2, data[5]))
        return false;

    /* Convert raw values to physical units */
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];

    /* Temperature conversion: T = -45 + 175 * (raw / 65535) */
    *temp_c = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);

    /* Humidity conversion: RH = -6 + 125 * (raw / 65535) */
    *humidity = -6.0f + 125.0f * ((float)hum_raw / 65535.0f);

    /* Clamp humidity to valid range */
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity < 0.0f) *humidity = 0.0f;

    return true;
}

/**
 * @brief Update sensor readings (stores internally)
 */
bool SHT40_UpdateReadings(void)
{
    return SHT40_Read(&last_temp_c, &last_humidity);
}

/**
 * @brief Get temperature in Celsius
 */
float SHT40_GetTemperatureC(void)
{
    return last_temp_c;
}

/**
 * @brief Get temperature in Fahrenheit
 */
float SHT40_GetTemperatureF(void)
{
    return (last_temp_c * 9.0f / 5.0f) + 32.0f;
}

/**
 * @brief Get relative humidity
 */
float SHT40_GetHumidity(void)
{
    return last_humidity;
}

/**
 * @brief Perform soft reset of the sensor
 */
bool SHT40_SoftReset(void)
{
    uint8_t cmd = SHT40_CMD_SOFT_RESET;

    if (sht40_i2c == NULL)
        return false;

    if (HAL_I2C_Master_Transmit(sht40_i2c, SHT40_I2C_ADDR << 1, &cmd, 1, 100) != HAL_OK)
        return false;

    /* Reset takes ~1ms */
    HAL_Delay(2);

    return true;
}

/**
 * @brief Read the sensor serial number
 */
bool SHT40_ReadSerial(uint32_t *serial)
{
    uint8_t cmd = SHT40_CMD_READ_SERIAL;
    uint8_t data[6];

    if (sht40_i2c == NULL || serial == NULL)
        return false;

    /* Send read serial command */
    if (HAL_I2C_Master_Transmit(sht40_i2c, SHT40_I2C_ADDR << 1, &cmd, 1, 100) != HAL_OK)
        return false;

    HAL_Delay(10);

    /* Read 6 bytes */
    if (HAL_I2C_Master_Receive(sht40_i2c, SHT40_I2C_ADDR << 1, data, 6, 100) != HAL_OK)
        return false;

    /* Verify CRCs */
    if (!SHT40_CheckCRC(&data[0], 2, data[2]))
        return false;

    if (!SHT40_CheckCRC(&data[3], 2, data[5]))
        return false;

    /* Combine into 32-bit serial */
    *serial = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
              ((uint32_t)data[3] << 8) | data[4];

    return true;
}
