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
    /* raw buffers */
    float bme_temp_buf[GRAPH_POINTS];
    float bme_press_buf[GRAPH_POINTS];
    float bme_hum_buf[GRAPH_POINTS];
    float lm35_temp_buf[GRAPH_POINTS];

    /* ring buffers */
    RingBuffer bme_temp;
    RingBuffer bme_press;
    RingBuffer bme_hum;
    RingBuffer lm35_temp;

    /* latest values */
    BME280_Data_t bme;
    LM35_Data_t   lm35;

} AppState_t;

typedef enum
{
    APP_EVENT_NONE = 0,
    APP_EVENT_BUTTON_GRAPH_MODE,
    APP_EVENT_BUTTON_SET_BASE_PRESSURE,
} AppEvent_t;

typedef enum {
    APP_STATE_RUNNING = 0,
    APP_STATE_ERROR
} AppStateMachine_t;


AppState_t gAppState;
SemaphoreHandle_t gAppStateMutex;
QueueHandle_t appEventQueue;


static void AppState_Init(AppState_t *s)
{
    RB_Init(&s->bme_temp,  s->bme_temp_buf,  GRAPH_POINTS);
    RB_Init(&s->bme_press, s->bme_press_buf, GRAPH_POINTS);
    RB_Init(&s->bme_hum,   s->bme_hum_buf,   GRAPH_POINTS);
    RB_Init(&s->lm35_temp, s->lm35_temp_buf, GRAPH_POINTS);
}

void App_HandleEvent(AppEvent_t evt)
{
	switch (evt)
	            {
	                case APP_EVENT_BUTTON_GRAPH_MODE:
	                    Graph_SetMode(
	                        (GraphMode)((Graph_GetMode() + 1) % GRAPH_MODE_COUNT)
	                    );
	                    break;

	                case APP_EVENT_BUTTON_SET_BASE_PRESSURE:
	                    xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
	                    BME280_SetBasePressure(gAppState.bme.pressure);
	                    xSemaphoreGive(gAppStateMutex);
	                    break;

	                default:
	                    break;
	            }
}


void SensorTask(void *arg)
{
	BME280_Data_t bme;
	LM35_Data_t lm35;

    for (;;)
    {
        if (LM35_Read(&lm35))
        {
        	Error_Clear(LM35_READ_VALUE_ERROR);
            xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
            gAppState.lm35 = lm35;
            RB_Push(&gAppState.lm35_temp, lm35.temperature_c);
            xSemaphoreGive(gAppStateMutex);

        }
        else
        {
        	// In case of an error, we write down a marker and signal
            RB_Push(&gAppState.lm35_temp, SENSOR_ERROR_VALUE);
        	Error_Trigger(LM35_READ_VALUE_ERROR);
        }


        if (BME280_Read(&bme))
        {
        	Error_Clear(BME280_READ_VALUE_ERROR);
            xSemaphoreTake(gAppStateMutex, portMAX_DELAY);
            gAppState.bme = bme;
            RB_Push(&gAppState.bme_temp, bme.temperature);
            RB_Push(&gAppState.bme_press, bme.pressure / PA_TO_MMHG);
            RB_Push(&gAppState.bme_hum, bme.humidity);
            xSemaphoreGive(gAppStateMutex);
        }
        else
        {
        	// In case of an error, we write down are markers and signal
            RB_Push(&gAppState.bme_temp, SENSOR_ERROR_VALUE);
            RB_Push(&gAppState.bme_press, SENSOR_ERROR_VALUE);
            RB_Push(&gAppState.bme_hum, SENSOR_ERROR_VALUE);
        	Error_Trigger(BME280_READ_VALUE_ERROR);
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
    static AppStateMachine_t state = APP_STATE_RUNNING;

    for (;;)
    {
        switch (state)
        {
            case APP_STATE_RUNNING:
                if (xQueueReceive(appEventQueue, &evt, portMAX_DELAY) == pdTRUE)
                {
                    App_HandleEvent(evt);
                }
                break;

            case APP_STATE_ERROR:
                break;
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

    AppState_Init(&gAppState);

    if (!Graph_Init(&gAppState.lm35_temp,
               &gAppState.bme_temp,
               &gAppState.bme_press,
               &gAppState.bme_hum))
    {
    	Error_Trigger(GRAPH_INIT_ERROR);
    }

    Display_DrawStatic();


    Buttons_Queue_Init();

    gAppStateMutex = xSemaphoreCreateMutex();
    configASSERT(gAppStateMutex);

    appEventQueue = xQueueCreate(8, sizeof(AppEvent_t));
    configASSERT(appEventQueue);

    xTaskCreate(SensorTask,  "Sensor",  512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(AppTask,     "App",     512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(DisplayTask, "Display", 512, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(ButtonTask,  "Button",  256, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(ErrorTask, "Error", 256, NULL, tskIDLE_PRIORITY + 1, NULL);


	vTaskStartScheduler();

	while (1) {}
}
