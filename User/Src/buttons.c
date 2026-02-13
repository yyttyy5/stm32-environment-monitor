/**
 * @file    buttons.c
 * @brief   Button handling module implementation.
 *
 * @details
 * This module implements event-based handling of physical buttons
 * using GPIO EXTI interrupts.
 *
 * Features:
 *  - GPIO and EXTI configuration for buttons
 *  - software debouncing
 *  - event latching in interrupt context
 *
 * Design notes:
 *  - STM32CubeMX code generation is NOT used
 *  - buttons are active-low or active-high depending on wiring
 *  - interrupt service routine performs minimal processing
 *
 * Architecture:
 *  - input / hardware abstraction layer
 *  - no application logic
 *  - events are consumed by the application layer
 */

#include "buttons.h"
#include "stm32f4xx_hal.h"

/* Button debounce time (milliseconds) */
#define DEBOUNCE_MS                      200

/* Button GPIO configuration */
#define BUTTON_SET_BASE_PRESSURE_PIN     GPIO_PIN_0
#define BUTTON_GRAPH_MODE_PIN            GPIO_PIN_2
#define BUTTON_SET_BASE_PRESSURE_PORT    GPIOA
#define BUTTON_GRAPH_MODE_PORT           GPIOC


static QueueHandle_t buttonQueue = NULL;


QueueHandle_t Buttons_GetQueue(void)
{
    return buttonQueue;
}

void Buttons_Queue_Init(void)
{
    buttonQueue = xQueueCreate(4, sizeof(ButtonEvent_t));
}
/**
 * @brief Initialize button subsystem.
 *
 * Configures GPIO pins and EXTI interrupts for all buttons.
 * Must be called once during system initialization.
 */
void Buttons_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks for GPIO ports and SYSCFG (EXTI)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* ===== BUTTON_GRAPH_MODE : PC2 ===== */
    GPIO_InitStruct.Pin  = BUTTON_GRAPH_MODE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BUTTON_GRAPH_MODE_PORT, &GPIO_InitStruct);

    // Configure NVIC for EXTI2 interrupt
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    /* ===== BUTTON_SET_BASE_PRESSURE : PA0 ===== */
    GPIO_InitStruct.Pin  = BUTTON_SET_BASE_PRESSURE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUTTON_SET_BASE_PRESSURE_PORT, &GPIO_InitStruct);

    // Configure NVIC for EXTI0 interrupt
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief GPIO EXTI interrupt callback.
 *
 * This function is called from HAL EXTI ISR context.
 * Performs software debouncing and latches button events.
 *
 * @param GPIO_Pin GPIO pin that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_tick[BUTTON_COUNT] = {0};
    uint32_t now = HAL_GetTick();

    ButtonEvent_t evt;

    if (GPIO_Pin == BUTTON_GRAPH_MODE_PIN)
    {
        if (now - last_tick[BUTTON_GRAPH_MODE] > DEBOUNCE_MS)
        {
            last_tick[BUTTON_GRAPH_MODE] = now;
            evt.id = BUTTON_GRAPH_MODE;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(buttonQueue, &evt, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if (GPIO_Pin == BUTTON_SET_BASE_PRESSURE_PIN)
    {
        if (now - last_tick[BUTTON_SET_BASE_PRESSURE] > DEBOUNCE_MS)
        {
            last_tick[BUTTON_SET_BASE_PRESSURE] = now;
            evt.id = BUTTON_SET_BASE_PRESSURE;
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(buttonQueue, &evt, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
