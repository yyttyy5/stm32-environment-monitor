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
	GRAPH_MODE_TEMPERATURE = 0,
	GRAPH_MODE_PRESSURE,
	GRAPH_MODE_HUMIDITY,
	GRAPH_MODE_COUNT
} GraphMode;

/**
 * @brief Initialize the graph module with sensor data buffers.
 *
 * @retval true  Initialization succeeded
 * @retval false Initialization failed
 */
bool Graph_Init(void);


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


/**
 * @brief Push a new LM35 temperature reading into the graph buffer.
 *
 * @details
 * Stores the latest LM35 sensor reading into the internal ring buffer
 * used for graph plotting. The value will be included in the next call
 * to Graph_Draw().
 *
 * The function is safe to call from a single task context. It does not
 * block or fail unless the internal buffer is not initialized.
 *
 * @param temp Temperature reading in degrees Celsius.
 */
void Graph_PushLM35(float temp);


/**
 * @brief Push a new BME280 sensor reading into the graph buffers.
 *
 * @details
 * Stores the latest BME280 sensor readings (temperature, pressure, humidity)
 * into their respective internal ring buffers used for graph plotting.
 * The values will be included in the next call to Graph_Draw().
 *
 * @param temp Temperature in degrees Celsius.
 * @param press Pressure in mmHg (or other pre-scaled unit as used by your system).
 * @param hum   Relative humidity in percent.
 */
void Graph_PushBME(float temp, float press, float hum);

#endif /* GRAPH_H */
