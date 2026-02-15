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
#include <stdbool.h>
#include "main.h"
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

typedef struct
{
    BME280_Data_t bme;
    LM35_Data_t   lm35;
} SensorSnapshot_t;

typedef enum
{
    APP_EVENT_BUTTON_GRAPH_MODE,
    APP_EVENT_BUTTON_SET_BASE_PRESSURE,
} AppEvent_t;

typedef struct
{
    QueueHandle_t sensorSnapshotQ;
    QueueHandle_t appEventQ;
} AppContext;


void SensorTask(void *arg)
{
    SensorSnapshot_t snap;
    AppContext *ctx = arg;

    for (;;)
    {
        if (LM35_Read(&snap.lm35))
        {
        	Error_Clear(LM35_READ_VALUE_ERROR);
        	Graph_PushLM35(snap.lm35.temperature_c);
        }
        else
        {
        	// In case of an error, we write down a marker and signal
        	Graph_PushLM35(SENSOR_ERROR_VALUE);
        	Error_Trigger(LM35_READ_VALUE_ERROR);
        }


        if (BME280_Read(&snap.bme))
        {
        	Error_Clear(BME280_READ_VALUE_ERROR);
        	Graph_PushBME(snap.bme.temperature, snap.bme.pressure / PA_TO_MMHG, snap.bme.humidity);
        }
        else
        {
        	// In case of an error, we write down are markers and signal
        	Graph_PushBME(SENSOR_ERROR_VALUE, SENSOR_ERROR_VALUE, SENSOR_ERROR_VALUE);
        	Error_Trigger(BME280_READ_VALUE_ERROR);
        }

        xQueueOverwrite(ctx->sensorSnapshotQ, &snap);

        // складываем данные
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void DisplayTask(void *arg)
{
    SensorSnapshot_t snap;
    AppContext *ctx = arg;

    for (;;)
    {
    	if (xQueuePeek(ctx->sensorSnapshotQ, &snap, portMAX_DELAY))
    	{
    		Display_UpdateSensors(&snap.bme, &snap.lm35);
    		Graph_Draw();
    	}
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void ButtonTask(void *arg)
{
    ButtonEvent_t btn;
    AppEvent_t appEvt;
    AppContext *ctx = arg;

    QueueHandle_t btnQueue = Buttons_GetQueue();

    for (;;)
    {
        if (xQueueReceive(btnQueue, &btn, portMAX_DELAY) == pdTRUE)
        {
            switch (btn.id)
            {
                case BUTTON_GRAPH_MODE:
                    appEvt = APP_EVENT_BUTTON_GRAPH_MODE;
                    xQueueSend(ctx->appEventQ, &appEvt, 0);
                    break;

                case BUTTON_SET_BASE_PRESSURE:
                    appEvt = APP_EVENT_BUTTON_SET_BASE_PRESSURE;
                    xQueueSend(ctx->appEventQ, &appEvt, 0);
                    break;

                default:
                    break;
            }
        }
    }
}


void AppTask(void *arg)
{
    AppEvent_t evt;
    SensorSnapshot_t snap;
    AppContext *ctx = arg;

    for (;;)
    {
    	if (xQueueReceive(ctx->appEventQ, &evt, portMAX_DELAY) == pdTRUE)
        {
    		switch (evt)
    		{
            	case APP_EVENT_BUTTON_GRAPH_MODE:
                	Graph_SetMode((GraphMode)((Graph_GetMode() + 1) % GRAPH_MODE_COUNT));
                break;

                case APP_EVENT_BUTTON_SET_BASE_PRESSURE:
                	if (xQueuePeek(ctx->sensorSnapshotQ, &snap, 0))
                		BME280_SetBasePressure(snap.bme.pressure);
                break;
             }
        }
    }
}

void ErrorTask(void *arg)
{
    for (;;)
    {
        Error_Handle();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	static AppContext app;

	if (!System_Init())
		Error_Trigger(SYSTEM_INIT_ERROR);

	ErrorLED_Init();

	if (!Display_Init())
		Error_Trigger(LCD_INIT_ERROR);

    Buttons_Init();

    if (!LM35_Init())
    	Error_Trigger(LM35_INIT_ERROR);
    if (!BME280_Init())
    	Error_Trigger(BME280_INIT_ERROR);

    if (!Graph_Init())
    	Error_Trigger(GRAPH_INIT_ERROR);

    Display_DrawStatic();


    Buttons_Queue_Init();

    app.sensorSnapshotQ = xQueueCreate(1, sizeof(SensorSnapshot_t));
    app.appEventQ = xQueueCreate(8, sizeof(AppEvent_t));

    xTaskCreate(SensorTask,  "Sensor",  512, &app, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(AppTask,     "App",     512, &app, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(DisplayTask, "Display", 512, &app, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(ButtonTask,  "Button",  256, &app, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(ErrorTask, "Error", 256, NULL, tskIDLE_PRIORITY + 1, NULL);


	vTaskStartScheduler();

	while (1) {}
}
