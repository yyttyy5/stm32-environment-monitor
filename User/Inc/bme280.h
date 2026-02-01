/**
 * @file    bme280.h
 * @brief   BME280 sensor driver interface.
 *
 * @details
 * Driver for Bosch BME280 environmental sensor.
 *
 * Provides functionality to:
 *  - initialize the BME280 sensor
 *  - read temperature, pressure and humidity
 *  - calculate relative altitude based on pressure
 *
 * Design notes:
 *  - communication via I2C
 *  - blocking API (intended for periodic polling)
 *  - does not perform direct peripheral initialization
 *  - relies on lower-level I2C abstraction
 *
 * Units:
 *  - temperature : degrees Celsius (°C)
 *  - pressure    : Pascal (Pa)
 *  - humidity    : percent (%)
 *  - altitude    : meters (m)
 */

#ifndef BME280_H
#define BME280_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/**
 * @brief BME280 measured data.
 */
typedef struct {
    float temperature;
    float pressure;
    float humidity;
    float altitude;
} BME280_Data_t;


/**
 * @brief Initialize BME280 sensor.
 *
 * Performs:
 *  - sensor reset
 *  - calibration data read
 *  - configuration of oversampling and filter
 *
 * @retval true   Sensor initialized successfully
 * @retval false  Sensor not detected or configuration failed
 */
bool BME280_Init(void);


/**
 * @brief Read measured values from BME280.
 *
 * Reads raw sensor data, applies compensation algorithms
 * according to Bosch datasheet and updates output structure.
 *
 * @param  data  Pointer to structure for storing measured values
 *
 * @retval true   Data successfully read and valid
 * @retval false  Communication error or invalid data
 */
bool BME280_Read(BME280_Data_t *data);

// Setting the relative height reference point (Pa)
void BME280_SetBasePressure(float pressure);

/*
 * Getting the relative height reference point (Pa)
 * Is called from the Display_UpdateSensors() for display saved pressure level.
 */
float BME280_GetBasePressure(void);

#endif /* BME280_H */
