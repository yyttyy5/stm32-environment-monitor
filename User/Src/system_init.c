/**
 * @file    system_init.c
 * @brief   System initialization implementation
 *
 * @details
 * This module initializes the STM32F4 system, including HAL,
 * system clock, and other fundamental hardware.
 * All functions return `true` if the operation was successful,
 * otherwise `false`.
 */

#include "system_init.h"
#include "stm32f429i_discovery_lcd.h"


/**
 * @brief Configure the system clock
 *
 * @details
 * Sets up the main internal regulator, initializes the HSE oscillator,
 * configures the PLL, enables over-drive mode, and sets CPU, AHB, and APB
 * bus clocks according to the desired frequency.
 *
 * @retval true   Clock configuration succeeded
 * @retval false  Clock configuration failed
 */
static bool SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		return false;

	/** Activate the Over-Drive mode
	*/
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
		return false;

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
										  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
		return false;

	return true;
}


/**
 * @brief Initialize the system
 *
 * @details
 * Initializes the HAL library and configures the system clock.
 * Returns false if any step fails.
 *
 * @retval true   System initialized successfully
 * @retval false  Initialization failed
 */
bool System_Init(void)
{
	if (HAL_Init() != HAL_OK)
		return false;

    if (!SystemClock_Config())
    	return false;

    return true;
}
