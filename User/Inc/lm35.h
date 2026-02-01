/**
 * @file    lm35.h
 * @brief   LM35 temperature sensor driver interface.
 *
 * @details
 * This module provides the interface to the LM35 temperature sensor.
 * It includes:
 *  - ADC initialization
 *  - Temperature reading
 *  - DMA handle access
 *
 * Notes:
 *  - Temperature is returned in Celsius.
 *  - Non-blocking read using DMA is supported.
 */

#ifndef LM35_H
#define LM35_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>


/**
 * @brief LM35 sensor data structure
 */
typedef struct {
    float temperature_c;
} LM35_Data_t;


/**
 * @brief Initialize LM35 sensor (ADC and DMA setup)
 *
 * @retval true  Initialization succeeded
 * @retval false Initialization failed
 */
bool LM35_Init(void);


/**
 * @brief Read the current temperature from LM35 sensor
 *
 * @param data Pointer to LM35_Data_t structure to store result
 * @retval true  Data successfully read and updated
 * @retval false Error occurred during reading
 */
bool LM35_Read(LM35_Data_t *data);

/**
 * @brief Get the DMA handle used by LM35 ADC
 *
 * calls in stm32f4xx_it.c
 *
 * @retval Pointer to DMA_HandleTypeDef associated with LM35 ADC
 *
 *
 */
DMA_HandleTypeDef* LM35_GetDmaHandle(void);

#endif /* LM35_H */
