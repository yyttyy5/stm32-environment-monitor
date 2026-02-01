/**
 * @file    graph.h
 * @brief   Graph rendering module for LM35 and BME280 sensor data.
 *
 * @details
 * This module provides functions to initialize and draw graphs
 * for LM35 and BME280 sensor data using ring buffers.
 * It supports switching between temperature, pressure, and humidity modes.
 *
 * Responsibilities:
 *  - Graph initialization with sensor ring buffers
 *  - Graph rendering on the display
 *  - Mode switching (temperature, pressure, humidity)
 *
 * Notes:
 *  - Uses a fixed number of points (GRAPH_POINTS)
 *  - Depends on RingBuffer for historical data storage
 */

#ifndef GRAPH_H
#define GRAPH_H

#include "ring_buffer.h"
#include <stdbool.h>

/// Number of points to display on the graph
#define GRAPH_POINTS   100

/// Graph display mode
typedef enum {
	GRAPH_MODE_TEMPERATURE,
	GRAPH_MODE_PRESSURE,
	GRAPH_MODE_HUMIDITY
} GraphMode;

/**
 * @brief Initialize the graph module with sensor data buffers.
 *
 * @param lm35       Ring buffer for LM35 temperature readings
 * @param bme_temp   Ring buffer for BME280 temperature readings
 * @param bme_press  Ring buffer for BME280 pressure readings
 * @param bme_hum    Ring buffer for BME280 humidity readings
 *
 * @retval true  Initialization succeeded
 * @retval false Initialization failed
 */
bool Graph_Init(const RingBuffer *lm35, const RingBuffer *bme_temp,
				const RingBuffer *bme_press, const RingBuffer *bme_hum);

/**
 * @brief Draw the graph on the display.
 *
 * Updates the graph according to the current mode and
 * the data stored in the ring buffers.
 */
void Graph_Draw(void);

/**
 * @brief Set the graph display mode.
 *
 * @param mode  New graph mode (temperature, pressure, or humidity)
 */
void Graph_SetMode(GraphMode mode);

/**
 * @brief Get the current graph display mode.
 *
 * @return Current GraphMode
 */
GraphMode Graph_GetMode(void);

/**
 * @brief Check if the graph module has been initialized.
 *
 * @retval true   Graph module initialized
 * @retval false  Graph module not initialized
 */
bool Is_Graph_Initialised(void);

#endif /* GRAPH_H */
