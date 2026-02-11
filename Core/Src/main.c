/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"



void AppTask(void *argument)
{
    App_Init();

    for (;;)
    {
        App_Loop();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	xTaskCreate(
          AppTask,
          "App",
          1024,
          NULL,
          tskIDLE_PRIORITY + 2,
          NULL
	);

	vTaskStartScheduler();

	while (1) {}
}
