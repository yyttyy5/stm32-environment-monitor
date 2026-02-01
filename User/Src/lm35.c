/**
 * @file    lm35.c
 * @brief   LM35 temperature sensor driver implementation.
 *
 * @details
 * This module provides functions to interface with the LM35 analog temperature sensor.
 * It includes:
 *  - ADC1 initialization
 *  - DMA configuration for continuous ADC readings
 *  - Conversion from ADC value to Celsius
 *
 * Notes:
 *  - The ADC is configured in continuous mode with DMA.
 *  - Temperature readings are returned in Celsius.
 */

#include "lm35.h"

// Constants
#define LM35_ADC_MAX       4095.0f   /**< Maximum 12-bit ADC value */
#define LM35_VREF_MV       2930.0f   /**< Reference voltage in mV */
#define LM35_MV_PER_C      10.0f     /**< LM35 output: 10 mV per °C */

// Private variables
static ADC_HandleTypeDef lm35_hadc;
static DMA_HandleTypeDef hdma_adc1;
static uint32_t lm35_adc_dma_value;


/**
 * @brief Initialize DMA for ADC1
 *
 * @retval true  DMA successfully initialized
 * @retval false Initialization failed
 */
static bool ADC1_DMA_Init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();

	hdma_adc1.Instance = DMA2_Stream0;
	hdma_adc1.Init.Channel = DMA_CHANNEL_0;
	hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_adc1.Init.MemInc = DMA_MINC_DISABLE;
	hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_adc1.Init.Mode = DMA_CIRCULAR;
	hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;

	if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
		return false;

	__HAL_LINKDMA(&lm35_hadc, DMA_Handle, hdma_adc1);

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    return true;
}


/**
 * @brief Initialize ADC1 for LM35
 *
 * @retval true  ADC successfully initialized
 * @retval false Initialization failed
 */
static bool LM35_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO configuration
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  // ADC configuration
  __HAL_RCC_ADC1_CLK_ENABLE();

  lm35_hadc.Instance = ADC1;
  lm35_hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  lm35_hadc.Init.Resolution = ADC_RESOLUTION_12B;
  lm35_hadc.Init.ScanConvMode = DISABLE;
  lm35_hadc.Init.ContinuousConvMode = ENABLE;
  lm35_hadc.Init.DiscontinuousConvMode = DISABLE;
  lm35_hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  lm35_hadc.Init.NbrOfConversion = 1;
  lm35_hadc.Init.DMAContinuousRequests = ENABLE;
  lm35_hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&lm35_hadc) != HAL_OK)
	  return false;

  // ADC channel configuration
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&lm35_hadc, &sConfig) != HAL_OK)
	  return false;

  return true;
}

/**
 * @brief Convert ADC value to Celsius
 *
 * @param adc_value ADC reading
 * @retval Temperature in Celsius
 */
static float LM35_ConvertToCelsius(uint32_t adc_value)
{
    float voltage_mv = (adc_value * LM35_VREF_MV) / LM35_ADC_MAX;
    return voltage_mv / LM35_MV_PER_C;
}


/**
 * @brief Initialize LM35 sensor
 *
 * @retval true  Initialization succeeded
 * @retval false Initialization failed
 */
bool LM35_Init(void)
{
	if (!LM35_ADC1_Init())
		return false;
	if (!ADC1_DMA_Init())
		return false;
	if (HAL_ADC_Start_DMA(&lm35_hadc, &lm35_adc_dma_value, 1) != HAL_OK)
		return false;

    return true;
}


/**
 * @brief Read current temperature from LM35
 *
 * @param data Pointer to LM35_Data_t structure to store result
 * @retval true  Data successfully read
 * @retval false Error occurred
 */
bool LM35_Read(LM35_Data_t *data)
{
    if (!data)
        return false;

    data->temperature_c =
        LM35_ConvertToCelsius(lm35_adc_dma_value);

    return true;
}


/**
 * @brief Get the DMA handle for LM35 ADC
 *
 * calls in stm32f4xx_it.c
 *
 * @retval Pointer to DMA_HandleTypeDef
 */
DMA_HandleTypeDef* LM35_GetDmaHandle(void)
{
	return &hdma_adc1;
}
