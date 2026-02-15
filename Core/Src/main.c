/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : RTOS-based sensor monitoring and display application.
  *
  * @details
  * Implements a FreeRTOS-based system for:
  *  - Periodic acquisition of LM35 and BME280 sensor data.
  *  - Storing historical data in ring buffers for graphing.
  *  - Displaying sensor readings and graphs on an LCD.
  *  - Handling button events for graph mode switching and base pressure adjustment.
  *  - Centralized error management.
  *
  * Communication between tasks is achieved via FreeRTOS queues to decouple
  * producer (sensors) and consumer (display / application logic) tasks.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lm35.h"
#include "bme280.h"
#include "display.h"
#include "graph.h"
#include "buttons.h"
#include "system_init.h"
#include "error.h"


/**
 * @brief Structure containing a snapshot of all sensor readings.
 *
 * @details
 * Holds the latest readings of LM35 temperature and BME280 temperature, pressure,
 * and humidity. Used to pass sensor data between SensorTask, DisplayTask, and
 * AppTask via queues.
 */
typedef struct
{
    BME280_Data_t bme;   /**< Latest BME280 sensor data */
    LM35_Data_t   lm35;  /**< Latest LM35 sensor data */
} SensorSnapshot_t;


/**
 * @brief Enumeration of application-level events triggered by buttons.
 *
 * @details
 * Events are sent from ButtonTask to AppTask via the appEvent queue.
 * APP_EVENT_BUTTON_GRAPH_MODE – cycle the graph display mode (temperature/pressure/humidity)
 * APP_EVENT_BUTTON_SET_BASE_PRESSURE – set the BME280 base pressure to current reading
 */
typedef enum
{
    APP_EVENT_BUTTON_GRAPH_MODE,         /**< Switch graph display mode */
    APP_EVENT_BUTTON_SET_BASE_PRESSURE,  /**< Set base pressure from BME280 */
} AppEvent_t;


/**
 * @brief Context structure passed to all tasks via the void* argument.
 *
 * @details
 * Contains FreeRTOS queues used for inter-task communication:
 *  - sensorSnapshotQ: queue of SensorSnapshot_t containing the latest sensor readings
 *  - appEventQ: queue of AppEvent_t containing user-triggered events
 */
typedef struct
{
    QueueHandle_t sensorSnapshotQ;  /**< Queue for sending sensor snapshots to DisplayTask and AppTask */
    QueueHandle_t appEventQ;        /**< Queue for sending button events to AppTask */
} AppContext;


/**
 * @brief Task: periodically read sensors and update graph buffers.
 *
 * @param arg Pointer to AppContext containing queues.
 *
 * @details
 * - Reads LM35 and BME280 every 500 ms.
 * - Pushes data into Graph ring buffers using Graph_PushLM35 and Graph_PushBME.
 * - Sends latest snapshot to sensorSnapshotQ via xQueueOverwrite.
 * - Triggers errors via Error_Trigger and marks readings with SENSOR_ERROR_VALUE on failure.
 */
void SensorTask(void *arg)
{
    SensorSnapshot_t snap;
    AppContext *ctx = arg;

    for (;;)
    {
        // Read LM35 sensor
        if (LM35_Read(&snap.lm35))
        {
        	Error_Clear(LM35_READ_VALUE_ERROR);
        	Graph_PushLM35(snap.lm35.temperature_c);
        }
        else
        {
        	Graph_PushLM35(SENSOR_ERROR_VALUE);
        	Error_Trigger(LM35_READ_VALUE_ERROR);
        }

        // Read BME280 sensor
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

        // Send latest snapshot to queue for display and app logic
        xQueueOverwrite(ctx->sensorSnapshotQ, &snap);

        // Wait for next sample (500 ms)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


/**
 * @brief Task: update the LCD with latest sensor data and graph.
 *
 * @param arg Pointer to AppContext containing queues.
 *
 * @details
 * - Waits for new sensor snapshot on sensorSnapshotQ using xQueuePeek.
 * - Updates numeric display via Display_UpdateSensors().
 * - Draws graph from internal ring buffers via Graph_Draw().
 * - Runs periodically every 500 ms.
 */
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


/**
 * @brief Task: handle button events and send corresponding application events.
 *
 * @param arg Pointer to AppContext containing queues.
 *
 * @details
 * - Waits for ButtonEvent_t from the Buttons module queue.
 * - Converts physical button presses into AppEvent_t and sends them to appEventQ.
 * - Uses non-blocking xQueueSend with a short timeout (10 ms) to avoid deadlocks.
 */
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
                    xQueueSend(ctx->appEventQ, &appEvt, pdMS_TO_TICKS(10));
                    break;

                case BUTTON_SET_BASE_PRESSURE:
                    appEvt = APP_EVENT_BUTTON_SET_BASE_PRESSURE;
                    xQueueSend(ctx->appEventQ, &appEvt, pdMS_TO_TICKS(10));
                    break;

                default:
                    break;
            }
        }
    }
}


/**
 * @brief Task: process application events received from buttons.
 *
 * @param arg Pointer to AppContext containing queues.
 *
 * @details
 * - Receives AppEvent_t from appEventQ.
 * - Performs actions:
 *      - APP_EVENT_BUTTON_GRAPH_MODE → cycles graph mode (temperature/pressure/humidity).
 *      - APP_EVENT_BUTTON_SET_BASE_PRESSURE → sets BME280 base pressure using latest sensor snapshot.
 */
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


/**
 * @brief Task: centralized error handling.
 *
 * @param arg Unused.
 *
 * @details
 * - Calls Error_Handle() periodically (every 100 ms) to process
 *   any errors triggered by other tasks (sensor failures, init failures).
 * - Ensures errors are logged, LED signals are updated, and system remains stable.
 */
void ErrorTask(void *arg)
{
    for (;;)
    {
        Error_Handle();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * @brief Main entry point: initializes system and starts FreeRTOS scheduler.
 *
 * @details
 * - Initializes system peripherals (clocks, GPIOs, LEDs, display, buttons).
 * - Initializes sensors (LM35, BME280) and graph buffers.
 * - Initializes FreeRTOS queues for inter-task communication.
 * - Creates RTOS tasks with appropriate stack sizes and priorities:
 *      SensorTask → data acquisition
 *      AppTask → application logic
 *      DisplayTask → LCD update
 *      ButtonTask → button event handling
 *      ErrorTask → error management
 * - Starts scheduler; function never returns.
 */
int main(void)
{
	static AppContext app;

	if (!System_Init())
		Error_Trigger(SYSTEM_INIT_ERROR);
	if (!Display_Init())
		Error_Trigger(LCD_INIT_ERROR);

	ErrorLED_GPIO_Init();
    Buttons_GPIO_Init();

    if (!LM35_Init())
    	Error_Trigger(LM35_INIT_ERROR);
    if (!BME280_Init())
    	Error_Trigger(BME280_INIT_ERROR);
    if (!Graph_Init())
    	Error_Trigger(GRAPH_INIT_ERROR);


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
