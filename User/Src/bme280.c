/**
 * @file    bme280.c
 * @brief   BME280 sensor driver implementation.
 *
 * @details
 * This module implements a driver for the Bosch BME280 sensor.
 *
 * Responsibilities:
 *  - low-level register access via I2C
 *  - calibration parameter handling
 *  - raw data compensation
 *  - relative altitude calculation
 *
 * Architecture:
 *  - sensor driver layer
 *  - no application logic
 *  - no direct dependency on display or UI modules
 *
 * Notes:
 *  - compensation formulas are taken directly from
 *    Bosch BME280 datasheet
 *  - 64-bit arithmetic is used where required to avoid overflow
 */

#include "bme280.h"

// I2C3 GPIO configuration
#define I2C3_SCL_PIN                     GPIO_PIN_8
#define I2C3_SDA_PIN                     GPIO_PIN_9
#define I2C3_SCL_PORT                    GPIOA
#define I2C3_SDA_PORT                    GPIOC

// Default I2C address of BME280 (SDO = GND)
#define BME280_ADDR          (0x76 << 1)

// BME280 registers
#define BME280_REG_ID        0xD0
#define BME280_REG_RESET     0xE0
#define BME280_REG_CTRL_HUM  0xF2
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG    0xF5
#define BME280_REG_DATA      0xF7

// BME280 constants
#define BME280_SOFTRESET     0xB6
#define BME280_ID_VALUE      0x60

#define BME280_I2C_TIMEOUT   100

// I2C handle used for BME280 communication
static I2C_HandleTypeDef bme_i2c;

/**
 * @brief Calibration coefficients read from sensor.
 *
 * Used by compensation algorithms.
 * Must be read once during initialization.
 */
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5,
                dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;

// Fine temperature value used for pressure and humidity compensation
static int32_t t_fine;


static float base_pressure = 101325.0f;
static bool base_pressure_set = false;

/**
 * @brief Initialize I2C3 peripheral for BME280 communication.
 *
 * Performs:
 *  - GPIO configuration (SCL, SDA)
 *  - I2C3 peripheral initialization
 *
 * @retval true   I2C initialized successfully
 * @retval false  I2C initialization failed
 */
static bool BME280_I2C3_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

// Clocking resolution of peripherals
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_I2C3_CLK_ENABLE();

  bme_i2c.Instance = I2C3;
  bme_i2c.Init.ClockSpeed = 100000;
  bme_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
  bme_i2c.Init.OwnAddress1 = 0x0;
  bme_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  bme_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  bme_i2c.Init.OwnAddress2 = 0;
  bme_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  bme_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  GPIO_InitStruct.Pin = I2C3_SCL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;

  HAL_GPIO_Init(I2C3_SCL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2C3_SDA_PIN;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;

  HAL_GPIO_Init(I2C3_SDA_PORT, &GPIO_InitStruct);

  if (HAL_I2C_Init(&bme_i2c) != HAL_OK)
  {
      return false;
  }

  return true;
}

/**
 * @brief Read multiple registers from BME280.
 *
 * @param reg  Start register address
 * @param buf  Pointer to receive buffer
 * @param len  Number of bytes to read
 *
 * @retval true   Read operation successful
 * @retval false  Invalid parameters or I2C error
 */
static bool BME280_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0)
        return false;

    return (HAL_I2C_Mem_Read(&bme_i2c, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, BME280_I2C_TIMEOUT) == HAL_OK);
}

/**
 * @brief Write single register of BME280.
 *
 * @param reg Register address
 * @param val Value to write
 *
 * @retval true   Write operation successful
 * @retval false  I2C error
 */
static bool BME280_WriteReg(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, BME280_I2C_TIMEOUT) == HAL_OK);
}

/**
 * @brief Read calibration parameters from BME280.
 *
 * @retval true   Calibration data successfully read
 * @retval false  I2C communication error
 */
static bool BME280_ReadCalibration(void)
{
    uint8_t buf[26];
    if (!BME280_ReadRegs(0x88, buf, 26))
    	return false;

    dig_T1 = (buf[1] << 8) | buf[0];
    dig_T2 = (buf[3] << 8) | buf[2];
    dig_T3 = (buf[5] << 8) | buf[4];

    dig_P1 = (buf[7] << 8) | buf[6];
    dig_P2 = (buf[9] << 8) | buf[8];
    dig_P3 = (buf[11] << 8) | buf[10];
    dig_P4 = (buf[13] << 8) | buf[12];
    dig_P5 = (buf[15] << 8) | buf[14];
    dig_P6 = (buf[17] << 8) | buf[16];
    dig_P7 = (buf[19] << 8) | buf[18];
    dig_P8 = (buf[21] << 8) | buf[20];
    dig_P9 = (buf[23] << 8) | buf[22];

    dig_H1 = buf[25];

    uint8_t hbuf[7];
    if (!BME280_ReadRegs(0xE1, hbuf, 7))
    	return false;

    // Humidity calibration parameters have a non-aligned format
    // and must be reconstructed according to the datasheet
    dig_H2 = (hbuf[1] << 8) | hbuf[0];
    dig_H3 = hbuf[2];
    dig_H4 = (hbuf[3] << 4) | (hbuf[4] & 0x0F);
    dig_H5 = (hbuf[5] << 4) | (hbuf[4] >> 4);
    dig_H6 = (int8_t)hbuf[6];

    return true;
}

/**
 * @brief Calculate relative altitude from pressure.
 *
 * Uses the barometric formula and current base pressure reference.
 *
 * @param pressure Current pressure in Pa
 *
 * @return Relative altitude in meters
 */
static float BME280_CalcRelativeAltitude(uint32_t pressure)
{
    return 44330.0f * (1.0f - powf((float)pressure / base_pressure, 0.1903f));
}


/**
 * @brief Initialize BME280 sensor.
 *
 * Initialization sequence:
 *  - initialize I2C peripheral
 *  - verify sensor ID
 *  - perform soft reset
 *  - read calibration data
 *  - configure oversampling and measurement mode
 *
 * @retval true   Sensor initialized successfully
 * @retval false  Initialization failed
 */
bool BME280_Init(void)
{
	if (!BME280_I2C3_Init())
		return false;

    uint8_t id;
    if (!BME280_ReadRegs(BME280_REG_ID, &id, 1))
    	return false;
    if (id != BME280_ID_VALUE)
        return false;

    if (!BME280_WriteReg(BME280_REG_RESET, BME280_SOFTRESET))
    	return false;
    HAL_Delay(5);

    if (!BME280_ReadCalibration())
    	return false;

    // humidity oversampling x1
    if (!BME280_WriteReg(BME280_REG_CTRL_HUM, 0x01))
    	return false;
    // temp & pressure x1, normal mode
    if (!BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x27))
    	return false;

    return true;
}

/**
 * @brief Read compensated sensor data.
 *
 * Reads raw measurement data, applies compensation algorithms
 * and updates output structure.
 *
 * @param data Pointer to output data structure
 *
 * @retval true   Data successfully read and valid
 * @retval false  Communication error or invalid data
 */
bool BME280_Read(BME280_Data_t *data)
{
    if (!data) return false;

    uint8_t buf[8];
    if (!BME280_ReadRegs(BME280_REG_DATA, buf, 8))
    	return false;

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    int32_t adc_H = (buf[6] << 8)  | buf[7];

    // Temperature compensation (sets t_fine)
    int32_t var1T = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2T = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1T + var2T;
    data->temperature = (t_fine * 5 + 128) / 25600.0f;

    // Pressure compensation (64-bit arithmetic)
    int64_t p;
    int64_t var1P = ((int64_t)t_fine) - 128000;
    int64_t var2P = var1P * var1P * dig_P6;
    var2P = var2P + ((var1P * dig_P5) << 17);
    var2P = var2P + (((int64_t)dig_P4) << 35);
    var1P = ((var1P * var1P * dig_P3) >> 8) + ((var1P * dig_P2) << 12);
    var1P = (((((int64_t)1) << 47) + var1P) * dig_P1) >> 33;
    if (var1P == 0) return false;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2P) * 3125) / var1P;
    var1P = (dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2P = (dig_P8 * p) >> 19;
    p = ((p + var1P + var2P) >> 8) + (((int64_t)dig_P7) << 4);
    data->pressure = (uint32_t)(p >> 8);


    // Humidity compensation
    int32_t h = t_fine - 76800;
    h = (((((adc_H << 14) - (dig_H4 << 20) - (dig_H5 * h)) + 16384) >> 15) *
        (((((((h * dig_H6) >> 10) * (((h * dig_H3) >> 11) + 32768)) >> 10) + 2097152) *
          dig_H2 + 8192) >> 14));
    h = h - (((((h >> 15) * (h >> 15)) >> 7) * dig_H1) >> 4);
    if (h < 0) h = 0;
    if (h > 419430400) h = 419430400;
    data->humidity = (h >> 12) / 1024.0f;

    // Initialize base pressure on first successful measurement
    if (!base_pressure_set)
    {
        base_pressure = data->pressure;
        base_pressure_set = true;
    }

    data->altitude = BME280_CalcRelativeAltitude(data->pressure);

    return true;
}

/**
 * @brief Set reference pressure for relative altitude calculation.
 *
 * @param pressure Reference pressure in Pa
 */
void BME280_SetBasePressure(float pressure)
{
    base_pressure = pressure;
}

/**
 * @brief Get current reference pressure.
 *
 * @return Reference pressure in Pa
 */
float BME280_GetBasePressure(void)
{
    return base_pressure;
}

