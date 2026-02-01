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

/* Latched button events (set in EXTI ISR, cleared on read) */
static volatile uint8_t button_events[BUTTON_COUNT];

/* Timestamp of last valid button event (for debouncing) */
static uint32_t last_tick[BUTTON_COUNT];


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
 * @brief Check and clear button event.
 *
 * Returns true if a button press event occurred
 * since the last call for the specified button.
 *
 * @param id Button identifier
 *
 * @retval true   Button event detected
 * @retval false  No button event or invalid ID
 */
bool Buttons_GetEvent(ButtonId id)
{
    if (id >= BUTTON_COUNT)
        return false;

    if (button_events[id])
    {
        button_events[id] = 0;
        return true;
    }
    return false;
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
    uint32_t now = HAL_GetTick();

    // BUTTON_GRAPH_MODE handling
    if (GPIO_Pin == BUTTON_GRAPH_MODE_PIN)
    {
        if (now - last_tick[BUTTON_GRAPH_MODE] > DEBOUNCE_MS)
        {
            last_tick[BUTTON_GRAPH_MODE] = now;
            button_events[BUTTON_GRAPH_MODE] = 1;
        }
    }
    // BUTTON_SET_BASE_PRESSURE handling
    else if (GPIO_Pin == BUTTON_SET_BASE_PRESSURE_PIN)
    {
        if (now - last_tick[BUTTON_SET_BASE_PRESSURE] > DEBOUNCE_MS)
        {
            last_tick[BUTTON_SET_BASE_PRESSURE] = now;
            button_events[BUTTON_SET_BASE_PRESSURE] = 1;
        }
    }
}
