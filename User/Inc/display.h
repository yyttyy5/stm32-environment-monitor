/**
 * @file    display.h
 * @brief   LCD display interface.
 *
 * @details
 * This module provides a high-level interface for displaying
 * sensor data on the STM32F429 Discovery LCD.
 *
 * Responsibilities:
 *  - LCD initialization
 *  - drawing static UI elements
 *  - updating dynamic sensor values
 *
 * Architecture:
 *  - presentation layer
 *  - no direct hardware access (uses BSP LCD driver)
 *  - no sensor logic or data acquisition
 *
 * Notes:
 *  - LCD handling is based on STM32F429 Discovery BSP
 *  - this module does not perform any timing or buffering
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include "bme280.h"
#include "lm35.h"

/**
 * @brief Conversion factor from Pascal to millimeters of mercury.
 */
#define PA_TO_MMHG    133.322f

/**
 * @brief Initialize LCD display subsystem.
 *
 * @retval true   LCD successfully initialized
 * @retval false  LCD initialization failed
 */
bool Display_Init(void);

/**
 * @brief Draw static UI elements.
 *
 * Draws labels and other non-changing screen elements.
 * Should be called once after successful display initialization.
 */
void Display_DrawStatic(void);

/**
 * @brief Update displayed sensor values.
 *
 * Updates dynamic fields on the LCD.
 * Passing NULL skips update for the corresponding sensor.
 *
 * @param bme   Pointer to BME280 sensor data (can be NULL)
 * @param lm35  Pointer to LM35 sensor data (can be NULL)
 */
void Display_UpdateSensors(const BME280_Data_t *bme,
                           const LM35_Data_t *lm35);

#endif /* DISPLAY_H */
