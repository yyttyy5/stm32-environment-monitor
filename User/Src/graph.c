/**
 * @file    graph.c
 * @brief   Graph rendering implementation for sensor data.
 *
 * @details
 * This module implements drawing of graphs for LM35 and BME280
 * sensor data stored in ring buffers. It supports temperature,
 * pressure, and humidity modes.
 *
 * Responsibilities:
 *  - Graph initialization with sensor ring buffers
 *  - Mode management (temperature, pressure, humidity)
 *  - Drawing graph lines and labels on the display
 *
 * Notes:
 *  - Uses a fixed number of points (GRAPH_POINTS)
 *  - Depends on Display module for rendering
 *  - Does not manage sensor acquisition
 */

#include <string.h>
#include "graph.h"
#include "stm32f429i_discovery_lcd.h"

/* Graph scale constants */
#define GRAPH_X0       30
#define GRAPH_Y0       20
#define GRAPH_WIDTH    180
#define GRAPH_HEIGHT   120

/* Graph value ranges */
#define TEMP_MIN       (-10.0f)
#define TEMP_MAX       ( 50.0f)
#define PRESS_MIN      (720.0f)
#define PRESS_MAX      (780.0f)
#define HUM_MIN        (0.0f)
#define HUM_MAX        (100.0f)

/**
 * @brief Structure describing graph scale and title.
 */
typedef struct {
	int32_t min;
	int32_t max;
    const uint8_t *title;
} GraphScale;

static const GraphScale graph_scales[] = {
    [GRAPH_MODE_TEMPERATURE] = { -10, 50, "temp,C" },
    [GRAPH_MODE_PRESSURE]    = { 720, 780, "press,mmHg" },
    [GRAPH_MODE_HUMIDITY]    = { 0, 100, "hum,%" }
};

/* Ring buffers for sensor data */
static const RingBuffer *temp_bme_rb  = NULL;
static const RingBuffer *temp_lm35_rb = NULL;
static const RingBuffer *press_rb     = NULL;
static const RingBuffer *hum_rb       = NULL;

/* Current graph mode */
static GraphMode current_mode = GRAPH_MODE_TEMPERATURE;

/* Flag indicating graph initialization */
static bool graph_initialized = false;


/**
 * @brief Convert a sensor value to the corresponding Y coordinate on the LCD.
 *
 * @param value Sensor value
 * @retval Y coordinate in pixels
 */
static uint16_t ValueToY(float value)
{
	if (current_mode > GRAPH_MODE_HUMIDITY)
		return GRAPH_Y0 + GRAPH_HEIGHT;

	if (value < graph_scales[current_mode].min) value = graph_scales[current_mode].min;
    if (value > graph_scales[current_mode].max) value = graph_scales[current_mode].max;
    return (GRAPH_Y0 + (GRAPH_HEIGHT * (1 -(value - graph_scales[current_mode].min) /
    		(graph_scales[current_mode].max - graph_scales[current_mode].min))));
}


/**
 * @brief Draw a curve based on the provided ring buffer.
 *
 * @param rb Pointer to the ring buffer containing sensor data
 */
static void DrawCurve(const RingBuffer *rb)
{
    if (!rb || rb->count < 2)
        return;

    float dx = (float)GRAPH_WIDTH / (GRAPH_POINTS - 1);

    for (uint16_t i = 1; i < rb->count; i++)
    {
        uint16_t index      = (rb->tail + i) % GRAPH_POINTS;
        uint16_t prev_index = (rb->tail + i - 1 + GRAPH_POINTS) % GRAPH_POINTS;

        uint16_t x0 = GRAPH_X0 + (uint16_t)((i - 1) * dx);
        uint16_t x1 = GRAPH_X0 + (uint16_t)(i * dx);

        uint16_t y0 = ValueToY(rb->buffer[prev_index]);
        uint16_t y1 = ValueToY(rb->buffer[index]);

        // Drawing transparent curve in case of sensor read error
        if (rb->buffer[prev_index] == SENSOR_ERROR_VALUE ||
        	rb->buffer[index] == SENSOR_ERROR_VALUE)
        {
            uint32_t prev_text_color = BSP_LCD_GetTextColor();
        	BSP_LCD_SetTextColor(LCD_COLOR_TRANSPARENT);
        	BSP_LCD_DrawLine(x0, graph_scales[current_mode].max, x1, graph_scales[current_mode].max);
        	BSP_LCD_SetTextColor(prev_text_color);
        	continue;
        }

        // Drawing curve
        BSP_LCD_DrawLine(x0, y0, x1, y1);
    }
}


/**
 * @brief Draw graph labels (min, max values, axis titles).
 */
static void DrawLabels(void)
{
	uint8_t buf[10];
	uint8_t digits_min, digits_max;

	BSP_LCD_SetFont(&Font12);

	/* UNIFIED rendering of marks */
	snprintf(buf, sizeof(buf), "%d", graph_scales[current_mode].min);
	digits_min = strlen(buf);
	BSP_LCD_DisplayStringAt(GRAPH_X0 - 7 * digits_min - 4, GRAPH_Y0 + GRAPH_HEIGHT - 5, buf, LEFT_MODE);

	snprintf(buf, sizeof(buf), "%d", graph_scales[current_mode].max);
	digits_max = strlen(buf);
	BSP_LCD_DisplayStringAt(GRAPH_X0 - 7 * digits_max - 4, GRAPH_Y0 - 4, buf, LEFT_MODE);

	BSP_LCD_DisplayStringAt(GRAPH_X0 + GRAPH_WIDTH - 20,
                        	GRAPH_Y0 + GRAPH_HEIGHT + 5, "t,s", LEFT_MODE);

	BSP_LCD_DisplayStringAt(0, 10, graph_scales[current_mode].title, CENTER_MODE);

    /* Draw unique marks depending on mode */
	switch (current_mode) {
	case GRAPH_MODE_TEMPERATURE:

		for (uint32_t i = 0; i < 10; i++)
		{
			BSP_LCD_DrawHLine(GRAPH_X0 - 3 + GRAPH_WIDTH / 10 * i, GRAPH_Y0 + GRAPH_HEIGHT/2, GRAPH_WIDTH / 20);
			BSP_LCD_DrawHLine(GRAPH_X0 - 3 + GRAPH_WIDTH / 10 * i, GRAPH_Y0 + 5 * GRAPH_HEIGHT/6, GRAPH_WIDTH / 20);
		}

		BSP_LCD_DisplayStringAt(GRAPH_X0 - 18,
                            	GRAPH_Y0 + GRAPH_HEIGHT/2 - 5, "20", LEFT_MODE);
		BSP_LCD_DisplayStringAt(GRAPH_X0 - 11,
                                GRAPH_Y0 + 5 * GRAPH_HEIGHT/6 - 5, "0", LEFT_MODE);

		break;

	case GRAPH_MODE_PRESSURE:

		for (uint32_t i = 0; i < 10; i++)
		{
			BSP_LCD_DrawHLine(GRAPH_X0 - 3 + GRAPH_WIDTH / 10 * i, GRAPH_Y0 + GRAPH_HEIGHT/2, GRAPH_WIDTH / 20);
			BSP_LCD_DrawHLine(GRAPH_X0 - 3 + GRAPH_WIDTH / 10 * i, GRAPH_Y0 + GRAPH_HEIGHT/6, GRAPH_WIDTH / 20);
		}

		BSP_LCD_DisplayStringAt(GRAPH_X0 - 25,
                                GRAPH_Y0 + GRAPH_HEIGHT/6 - 5, "770", LEFT_MODE);
		BSP_LCD_DisplayStringAt(GRAPH_X0 - 25,
                            	GRAPH_Y0 + GRAPH_HEIGHT/2 - 5, "750", LEFT_MODE);
		break;

	case GRAPH_MODE_HUMIDITY:

		for (uint32_t i = 0; i < 10; i++)
		{
			BSP_LCD_DrawHLine(GRAPH_X0 - 3 + GRAPH_WIDTH / 10 * i, GRAPH_Y0 + GRAPH_HEIGHT/2, GRAPH_WIDTH / 20);
		}

		BSP_LCD_DisplayStringAt(GRAPH_X0 - 18,
                            	GRAPH_Y0 + GRAPH_HEIGHT/2 - 5, "50", LEFT_MODE);
		break;

	}

}


/**
 * @brief Draw the X and Y axes for the graph.
 */
static void DrawAxes(void)
{
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    // Vertical axe
    BSP_LCD_DrawLine(GRAPH_X0, GRAPH_Y0,
                     GRAPH_X0, GRAPH_Y0 + GRAPH_HEIGHT);

    // Horizontal axe
    BSP_LCD_DrawLine(GRAPH_X0, GRAPH_Y0 + GRAPH_HEIGHT,
                     GRAPH_X0 + GRAPH_WIDTH, GRAPH_Y0 + GRAPH_HEIGHT);

    DrawLabels();
}


/**
 * @brief Set the current graph mode.
 *
 * @param mode Graph mode to set
 */
void Graph_SetMode(GraphMode mode)
{
	if (mode > GRAPH_MODE_HUMIDITY)
		return;
    current_mode = mode;
}

/**
 * @brief Get the current graph mode.
 *
 * @retval Current graph mode
 */
GraphMode Graph_GetMode(void)
{
    return current_mode;
}

/**
 * @brief Check if the graph module is initialized.
 *
 * @retval true  Graph initialized
 * @retval false Graph not initialized
 */
bool Is_Graph_Initialised(void)
{
    return graph_initialized;
}


/**
 * @brief Initialize the graph module with sensor ring buffers.
 *
 * @param lm35       Ring buffer for LM35 temperature
 * @param bme_temp   Ring buffer for BME280 temperature
 * @param bme_press  Ring buffer for BME280 pressure
 * @param bme_hum    Ring buffer for BME280 humidity
 *
 * @retval true  Initialization succeeded
 * @retval false Initialization failed
 */
bool Graph_Init(const RingBuffer *lm35, const RingBuffer *bme_temp,
				const RingBuffer *bme_press, const RingBuffer *bme_hum)
{
	if (!lm35 || !bme_temp || !bme_press || !bme_hum)
		return false;

    uint32_t prev_text_color = BSP_LCD_GetTextColor();

    temp_lm35_rb = lm35;
    temp_bme_rb  = bme_temp;
    press_rb = bme_press;
    hum_rb = bme_hum;

    DrawAxes();
    BSP_LCD_SetTextColor(prev_text_color);

    graph_initialized = true;
    return true;
}


/**
 * @brief Draw the graph on the LCD based on current mode and data buffers.
 */
void Graph_Draw(void)
{
	// Not draw if initialize error
	if (!graph_initialized)
		return;

	// Saving font and color for recovery in the end
    uint32_t prev_text_color = BSP_LCD_GetTextColor();
	sFONT *prev_font = BSP_LCD_GetFont();

	// Clearing graph with axes
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_FillRect(0, 0,
                     GRAPH_WIDTH + GRAPH_X0 + 10,
                     GRAPH_HEIGHT + GRAPH_Y0 + 10);

    DrawAxes();

    switch (current_mode)
    {
        case GRAPH_MODE_TEMPERATURE:
            if (temp_bme_rb)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                DrawCurve(temp_bme_rb);
            }

            if (temp_lm35_rb)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_RED);
                DrawCurve(temp_lm35_rb);
            }
            break;

        case GRAPH_MODE_PRESSURE:
            if (press_rb)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                DrawCurve(press_rb);
            }
            break;

        case GRAPH_MODE_HUMIDITY:
            if (hum_rb)
            {
                BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
                DrawCurve(hum_rb);
            }
            break;
    }

    // Recovering previous font and color
    BSP_LCD_SetTextColor(prev_text_color);
    BSP_LCD_SetFont(prev_font);
}
