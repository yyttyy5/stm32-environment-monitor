/**
 * @file    ring_buffer.h
 * @brief   Ring buffer (circular buffer) implementation for sensor data.
 *
 * @details
 * This module provides a simple ring buffer to store sensor readings.
 * It supports:
 *  - Initialization of the buffer
 *  - Adding new values with automatic wrap-around
 *  - Storing a fixed number of most recent samples
 *
 * Notes:
 *  - `SENSOR_ERROR_VALUE` is used to mark invalid readings.
 *  - The buffer does not allocate memory internally; it uses a preallocated array.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define SENSOR_ERROR_VALUE     -500   /**< Value representing a sensor error */

typedef struct {
    float *buffer;   /**< Pointer to preallocated buffer array */
    uint16_t size;   /**< Maximum number of elements */
    uint16_t head;   /**< Index of next write */
    uint16_t tail;   /**< Index of oldest element */
    uint16_t count;  /**< Number of elements currently stored */
} RingBuffer;


/**
 * @brief Initialize a ring buffer
 *
 * @param rb   Pointer to RingBuffer structure
 * @param buf  Preallocated array for storing values
 * @param size Size of the array
 * @retval true  Initialization successful
 * @retval false Initialization failed (invalid parameters)
 */
bool RB_Init(RingBuffer *rb, const float *buf, uint16_t size);


/**
 * @brief Push a new value into the ring buffer
 *
 * @param rb    Pointer to RingBuffer
 * @param value Value to store
 * @retval true  Value successfully stored
 * @retval false Failed (e.g., invalid buffer)
 */
bool RB_Push(RingBuffer *rb, float value);

#endif /* RING_BUFFER_H */
