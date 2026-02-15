/**
 * @file    ring_buffer.c
 * @brief   Ring buffer (circular buffer) implementation for sensor data.
 *
 * @details
 * Implements a fixed-size circular buffer for storing float sensor readings.
 * Supports initialization and pushing new values with automatic wrap-around.
 * Uses `SENSOR_ERROR_VALUE` to mark invalid readings.
 */

#include "ring_buffer.h"


uint16_t RB_Count(const RingBuffer *rb)
{
    return rb ? rb->count : 0;
}

float RB_Get(const RingBuffer *rb, uint16_t index)
{
    if (!rb || index >= rb->count)
        return SENSOR_ERROR_VALUE;

    uint16_t real = (rb->tail + index) % rb->size;
    return rb->buffer[real];
}

/**
 * @brief Initialize a ring buffer
 *
 * @param rb   Pointer to RingBuffer structure
 * @param buf  Preallocated array for storing values
 * @param size Size of the array
 * @retval true  Initialization successful
 * @retval false Invalid parameters
 */
bool RB_Init(RingBuffer *rb, const float *buf, uint16_t size)
{
	if (!rb || !buf || size == 0)
		return false;

    rb->buffer = buf;
    rb->size = size;
    rb->head = rb->tail = rb->count = 0;

    return true;
}


/**
 * @brief Push a new value into the ring buffer
 *
 * @param rb    Pointer to RingBuffer
 * @param value Value to store
 * @retval true  Value successfully stored
 * @retval false Invalid buffer
 */
bool RB_Push(RingBuffer *rb, float value)
{
	if (!rb || !rb->buffer || rb->size == 0)
		return false;

    rb->buffer[rb->head] = value;
    rb->head = (rb->head + 1) % rb->size;

    if (rb->count < rb->size)
        rb->count++;
    else
        rb->tail = (rb->tail + 1) % rb->size;

    return true;
}
