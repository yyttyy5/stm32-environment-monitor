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
#include "semphr.h"
#include "lm35.h"
#include "bme280.h"
#include "display.h"
#include "graph.h"
#include "buttons.h"
#include "ring_buffer.h"
#include "system_init.h"
#include "error.h"

float bme280_buffer_temp[GRAPH_POINTS];
float bme280_buffer_press[GRAPH_POINTS];
float bme280_buffer_hum[GRAPH_POINTS];
float lm35_buffer_temp[GRAPH_POINTS];

typedef struct
{
    BME280_Data_t bme;
    LM35_Data_t   lm35;

    RingBuffer bme_temp;
    RingBuffer bme_press;
    RingBuffer bme_hum;
    RingBuffer lm35_temp;
} AppState_t;

AppState_t gAppState;
SemaphoreHandle_t gAppStateMutex;


void SensorTask(void *arg)
{
	BME280_Data_t bme;
	LM35_Data_t lm35;

    for (;;)
    {
        if (LM35_Read(&lm35))
        {
            xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
            gAppState.lm35 = lm35;
            RB_Push(&gAppState.lm35_temp, lm35.temperature_c);
            xSemaphoreGive(gAppStateMutex);
        }


        if (BME280_Read(&bme))
        {
            xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
            gAppState.bme = bme;
            RB_Push(&gAppState.bme_temp, bme.temperature);
            RB_Push(&gAppState.bme_press, bme.pressure / PA_TO_MMHG);
            RB_Push(&gAppState.bme_hum, bme.humidity);
            xSemaphoreGive(gAppStateMutex);
        }


        // складываем данные
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void DisplayTask(void *arg)
{
    BME280_Data_t bme;
    LM35_Data_t lm35;

    for (;;)
    {
        xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
        bme  = gAppState.bme;
        lm35 = gAppState.lm35;
        xSemaphoreGive(gAppStateMutex);

        Display_UpdateSensors(&bme, &lm35);
        Graph_Draw();

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void ButtonTask(void *arg)
{
    ButtonEvent_t evt;
    QueueHandle_t queue;

    queue = Buttons_GetQueue();

    for (;;)
    {
        if (xQueueReceive(queue, &evt, portMAX_DELAY) == pdTRUE)
        {
            switch (evt.id)
            {
                case BUTTON_GRAPH_MODE:
                    // Сменить режим графика
                    Graph_SetMode((GraphMode)((Graph_GetMode() + 1) % 3));
                    break;

                case BUTTON_SET_BASE_PRESSURE:
                    xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
                    BME280_SetBasePressure(gAppState.bme.pressure);
                    xSemaphoreGive(gAppStateMutex);
                    break;

                default:
                    break;
            }
        }
    }
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    System_Init();
    ErrorLED_Init();
    Display_Init();
    LM35_Init();
    BME280_Init();
    Buttons_Init();
    Graph_Init(&(gAppState.lm35_temp), &(gAppState.bme_temp),
    		&(gAppState.bme_press), &(gAppState.bme_hum));

    Display_DrawStatic();

    RB_Init(&(gAppState.bme_temp), bme280_buffer_temp, sizeof(bme280_buffer_temp) / sizeof(float));
    RB_Init(&(gAppState.bme_press), bme280_buffer_press, sizeof(bme280_buffer_press) / sizeof(float));
	RB_Init(&(gAppState.bme_hum), bme280_buffer_hum, sizeof(bme280_buffer_hum) / sizeof(float));
	RB_Init(&(gAppState.lm35_temp), lm35_buffer_temp, sizeof(lm35_buffer_temp) / sizeof(float));

    gAppStateMutex = xSemaphoreCreateMutex();
    configASSERT(gAppStateMutex);

	xTaskCreate(DisplayTask, "DT", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(SensorTask, "ST", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(ButtonTask, "BT", 256, NULL, tskIDLE_PRIORITY + 3, NULL);  // выше display

	vTaskStartScheduler();

	while (1) {}
}
