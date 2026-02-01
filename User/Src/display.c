/**
 * @file    display.c
 * @brief   LCD display driver implementation.
 *
 * @details
 * This module implements the interface defined in display.h.
 * It provides functionality for initializing the LCD, drawing static UI elements,
 * and updating dynamic sensor data from BME280 and LM35 sensors.
 *
 * Architecture:
 *  - presentation layer only
 *  - uses STM32F429 Discovery BSP LCD functions
 *  - does not handle sensor acquisition or timing
 *
 * Notes:
 *  - dynamic text fields are cleared before updating to avoid ghosting
 *  - all positions are hardcoded for the current layout
 */

#include "display.h"
#include "stm32f429i_discovery_lcd.h"
#include <stdio.h>

/** LCD configuration constants */
#define LCD_HEIGHT             320
#define LCD_FRAME_BUFFER_LAYER (uint32_t)(0xd0000000)
#define CLEAR_STRING           ("                      ")

/**
 * @brief Initialize the LCD display.
 *
 * @retval true   LCD initialized successfully
 * @retval false  Initialization failed
 */
bool Display_Init(void)
{
    if (BSP_LCD_Init() != LCD_OK)
    	return false;
    BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER_LAYER);
    BSP_LCD_Clear(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

    return true;
}

/**
 * @brief Draw static UI elements.
 *
 * Draws non-changing labels for LM35DZ and BME280 sensors.
 * This should be called once after successful display initialization.
 */
void Display_DrawStatic(void)
{
    BSP_LCD_SetFont(&Font20);

    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 117, "LM35DZ:", LEFT_MODE);

    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 72, "BME280:", LEFT_MODE);
}

/**
 * @brief Update dynamic sensor values on the LCD.
 *
 * Clears old values before drawing new values to prevent ghosting.
 *
 * @param bme   Pointer to BME280 sensor data (can be NULL)
 * @param lm35  Pointer to LM35 sensor data (can be NULL)
 */
void Display_UpdateSensors(const BME280_Data_t *bme,
                           const LM35_Data_t *lm35)
{
    uint8_t buf[64];  ///< Temporary buffer for formatted strings

    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);

    /** Update LM35 temperature */
    if (lm35)
    {
        snprintf(buf, sizeof(buf), "TEMP: %.2f C", lm35->temperature_c);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 97, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 97, buf, LEFT_MODE);
    }

    /** Update BME280 sensor values */
    if (bme)
    {
        snprintf(buf, sizeof(buf), "TEMP: %.2f C", bme->temperature);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 52, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 52, buf, LEFT_MODE);

        snprintf(buf, sizeof(buf), "PRESS: %.1f mmHg", bme->pressure / PA_TO_MMHG);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 34, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 34, buf, LEFT_MODE);

        snprintf(buf, sizeof(buf), "HUM: %.1f %%", bme->humidity);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 16, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 16, buf, LEFT_MODE);

        BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);

        snprintf(buf, sizeof(buf), "RELATIVE ALT:%.1f M", bme->altitude);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 159, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 159, buf, LEFT_MODE);

        snprintf(buf, sizeof(buf), "(press: %.1f mmHg)", BME280_GetBasePressure() / PA_TO_MMHG);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 140, CLEAR_STRING, LEFT_MODE);
        BSP_LCD_DisplayStringAt(0, LCD_HEIGHT - 140, buf, LEFT_MODE);
    }
}
