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

typedef struct
{
    float lm35_temp_buf[GRAPH_POINTS];
    float bme_temp_buf[GRAPH_POINTS];
    float bme_press_buf[GRAPH_POINTS];
    float bme_hum_buf[GRAPH_POINTS];

    RingBuffer lm35_temp;
    RingBuffer bme_temp;
    RingBuffer bme_press;
    RingBuffer bme_hum;
} GraphState_t;

typedef enum
{
    APP_EVENT_BUTTON_GRAPH_MODE,
    APP_EVENT_BUTTON_SET_BASE_PRESSURE,
} AppEvent_t;


GraphState_t gGraphState;

QueueHandle_t sensorSnapshotQueue;
QueueHandle_t appEventQueue;


static void GraphState_Init(GraphState_t *s)
{
    RB_Init(&s->bme_temp,  s->bme_temp_buf,  GRAPH_POINTS);
    RB_Init(&s->bme_press, s->bme_press_buf, GRAPH_POINTS);
    RB_Init(&s->bme_hum,   s->bme_hum_buf,   GRAPH_POINTS);
    RB_Init(&s->lm35_temp, s->lm35_temp_buf, GRAPH_POINTS);
}


void SensorTask(void *arg)
{
    SensorSnapshot_t snap;

    for (;;)
    {
        if (LM35_Read(&snap.lm35))
        {
        	Error_Clear(LM35_READ_VALUE_ERROR);
            RB_Push(&gGraphState.lm35_temp, snap.lm35.temperature_c);

        }
        else
        {
        	// In case of an error, we write down a marker and signal
            RB_Push(&gGraphState.lm35_temp, SENSOR_ERROR_VALUE);
        	Error_Trigger(LM35_READ_VALUE_ERROR);
        }


        if (BME280_Read(&snap.bme))
        {
        	Error_Clear(BME280_READ_VALUE_ERROR);
            RB_Push(&gGraphState.bme_temp, snap.bme.temperature);
            RB_Push(&gGraphState.bme_press, snap.bme.pressure / PA_TO_MMHG);
            RB_Push(&gGraphState.bme_hum, snap.bme.humidity);
        }
        else
        {
        	// In case of an error, we write down are markers and signal
            RB_Push(&gGraphState.bme_temp, SENSOR_ERROR_VALUE);
            RB_Push(&gGraphState.bme_press, SENSOR_ERROR_VALUE);
            RB_Push(&gGraphState.bme_hum, SENSOR_ERROR_VALUE);
        	Error_Trigger(BME280_READ_VALUE_ERROR);
        }

        xQueueOverwrite(sensorSnapshotQueue, &snap);

        // складываем данные
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void DisplayTask(void *arg)
{
    SensorSnapshot_t snap;

    for (;;)
    {
    	if (xQueuePeek(sensorSnapshotQueue, &snap, portMAX_DELAY))
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

    QueueHandle_t btnQueue = Buttons_GetQueue();

    for (;;)
    {
        if (xQueueReceive(btnQueue, &btn, portMAX_DELAY) == pdTRUE)
        {
            switch (btn.id)
            {
                case BUTTON_GRAPH_MODE:
                    appEvt = APP_EVENT_BUTTON_GRAPH_MODE;
                    xQueueSend(appEventQueue, &appEvt, 0);
                    break;

                case BUTTON_SET_BASE_PRESSURE:
                    appEvt = APP_EVENT_BUTTON_SET_BASE_PRESSURE;
                    xQueueSend(appEventQueue, &appEvt, 0);
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

    for (;;)
    {
    	if (xQueueReceive(appEventQueue, &evt, portMAX_DELAY) == pdTRUE)
        {
    		switch (evt)
    		{
            	case APP_EVENT_BUTTON_GRAPH_MODE:
                	Graph_SetMode((GraphMode)((Graph_GetMode() + 1) % GRAPH_MODE_COUNT));
                break;

                case APP_EVENT_BUTTON_SET_BASE_PRESSURE:
                	if (xQueuePeek(sensorSnapshotQueue, &snap, 0))
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

    GraphState_Init(&gGraphState);

    if (!Graph_Init(&gGraphState.lm35_temp,
               &gGraphState.bme_temp,
               &gGraphState.bme_press,
               &gGraphState.bme_hum))
    {
    	Error_Trigger(GRAPH_INIT_ERROR);
    }

    Display_DrawStatic();


    Buttons_Queue_Init();

    appEventQueue = xQueueCreate(8, sizeof(AppEvent_t));
    configASSERT(appEventQueue);

    sensorSnapshotQueue = xQueueCreate(1, sizeof(SensorSnapshot_t));
    configASSERT(sensorSnapshotQueue);

    xTaskCreate(SensorTask,  "Sensor",  512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(AppTask,     "App",     512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(DisplayTask, "Display", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(ButtonTask,  "Button",  256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(ErrorTask, "Error", 256, NULL, tskIDLE_PRIORITY + 1, NULL);


	vTaskStartScheduler();

	while (1) {}
}
