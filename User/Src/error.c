/**
 * @file    error.c
 * @brief   System error handling implementation.
 *
 * @details
 * Implements centralized error handling logic defined in error.h.
 * Uses LEDs for visual signaling and LCD for critical errors.
 *
 * Architecture:
 *  - error list table holds all error metadata
 *  - critical errors halt system
 *  - recoverable errors escalate if not cleared after a timeout
 *  - warning errors just blink green LED
 *
 * Notes:
 *  - HAL_GetTick() is used for timing
 *  - LCD messages are displayed for critical and escalated recoverable errors
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery_lcd.h"
#include "error.h"

/** @brief Structure for each error record */
typedef struct {
    ErrorType  type;        ///< severity
    uint8_t    *error_msg;  ///< message to display on LCD
    uint32_t   timestamp;   ///< time of activation
    bool       active;      ///< currently active?
    bool       time_over;   ///< recoverable error timeout exceeded?
} ErrorRecord;

/** LED configuration */
#define RED_LED_PIN           GPIO_PIN_14
#define GREEN_LED_PIN         GPIO_PIN_13
#define RED_LED_PORT          GPIOG
#define GREEN_LED_PORT        GPIOG

#define LED_TOGGLE_TIME       500    ///< ms
#define RECOVER_TIME          10000  ///< ms for recoverable errors

/** @brief Array holding metadata for all errors */
ErrorRecord error_list[]  = {
	[SYSTEM_INIT_ERROR]          = { ERROR_CRITICAL, "System init failed", 0, false, false },
	[LCD_INIT_ERROR]             = { ERROR_CRITICAL, "LCD init failed", 0, false, false },
	[BME280_INIT_ERROR]          = { ERROR_CRITICAL, "BME280 init failed", 0, false, false },
	[BME280_READ_VALUE_ERROR]    = { ERROR_RECOVERABLE, "BME280 read error", 0, false, false },
	[LM35_INIT_ERROR]            = { ERROR_CRITICAL, "LM35 init failed", 0, false, false },
	[LM35_READ_VALUE_ERROR]      = { ERROR_RECOVERABLE, "LM35 read error", 0, false, false },
	[GRAPH_INIT_ERROR]           = { ERROR_WARNING, "Graph init failed", 0, false, false },
};

/** @brief Handle critical errors by halting the system and displaying message */
static void Critical_Error_Handle(ErrorRecord *rec)
{
	  // Displaying on LCD
	  BSP_LCD_Clear(LCD_COLOR_RED);
	  BSP_LCD_SetBackColor(LCD_COLOR_RED);
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetFont(&Font16);
	  BSP_LCD_DisplayStringAt(0, 160, (uint8_t*)rec->error_msg, CENTER_MODE);

	  __disable_irq();

	  // LED indication
	  while (1) {
	      HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
	  }
}

/**
 * @brief Initialize error LEDs (red and green) on the board.
 *
 * Configures GPIO pins as outputs and turns off both LEDs.
 */
void ErrorLED_Init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = RED_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(RED_LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GREEN_LED_PIN;

    HAL_GPIO_Init(GREEN_LED_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_RESET);
}


/**
 * @brief Trigger an error.
 *
 * Activates an error record, sets its timestamp, and turns on the corresponding LED.
 * Critical errors or recoverable errors whose recovery time has elapsed will
 * stop the system and display the error message on the LCD.
 *
 * @param error  Error title to trigger
 */
void Error_Trigger(ErrorTitle error)
{
    if (error >= sizeof(error_list)/sizeof(error_list[0]))
        return;

    ErrorRecord *rec = &error_list[error];

    if (!rec->active)
    {
        rec->timestamp = HAL_GetTick();
        rec->active = true;
    }

    // Instant handling for critical errors
    if ((rec->type == ERROR_CRITICAL) || (rec->time_over))
    	Critical_Error_Handle(rec);
}


/**
 * @brief Clear a previously triggered error.
 *
 * Deactivates the error, resets its timestamp, and turns off the corresponding LED.
 *
 * @param error  Error title to clear
 */
void Error_Clear(ErrorTitle error)
{
    if (error >= sizeof(error_list)/sizeof(error_list[0]))
        return;

    ErrorRecord *rec = &error_list[error];

    if (rec->active)
    {
    	rec->active    = false;
    	rec->time_over = false;
    	rec->timestamp = 0;

    	HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    }
}

/**
 * @brief Periodic error handler.
 *
 * Toggles LEDs for active recoverable and warning errors and escalates recoverable
 * errors if their timeout has been reached.
 */
void Error_Handle(void)
{
    static uint32_t last_toggle = 0;
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < sizeof(error_list)/sizeof(error_list[0]); i++)
    {
        ErrorRecord *rec = &error_list[i];
        if (!rec->active)
            continue;

        /*
         * if the recoverable error is active for more than 10 seconds,
         * handling is considered critical.
         */
        if ((now - rec->timestamp >= RECOVER_TIME) && (rec->type == ERROR_RECOVERABLE)) {
        	rec->time_over = true;
        	Critical_Error_Handle(rec);
        }

        // LED flashing every 500 ms
        if (now - last_toggle >= LED_TOGGLE_TIME)
        {
            last_toggle = now;

            switch (rec->type)
            {
                case ERROR_RECOVERABLE:
                    HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
                    break;
                case ERROR_WARNING:
                    HAL_GPIO_TogglePin(GREEN_LED_PORT, GREEN_LED_PIN);
                    break;
                default:
                    break;
            }
        }
    }
}
