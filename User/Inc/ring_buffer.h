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


/**
 * @brief Get the number of valid elements currently stored in the ring buffer.
 *
 * @details
 * Returns the count of elements that have been written to the buffer and
 * have not yet been overwritten. The returned value is always in the range
 * [0 .. buffer size].
 *
 * This function does not modify the buffer state and is safe to call
 * concurrently with a single writer task, assuming single-writer /
 * single-reader usage.
 *
 * @param rb Pointer to the ring buffer instance.
 *
 * @return
 *  - Number of valid elements in the buffer
 *  - 0 if @p rb is NULL
 */
uint16_t RB_Count(const RingBuffer *rb);


/**
 * @brief Retrieve an element from the ring buffer by logical index.
 *
 * @details
 * The index is interpreted relative to the oldest element currently stored
 * in the buffer:
 *  - index = 0 corresponds to the oldest element
 *  - index = RB_Count(rb) - 1 corresponds to the most recent element
 *
 * The function performs bounds checking and does not modify the buffer state.
 *
 * If the buffer pointer is NULL or the index is out of range, a special
 * error marker value (@ref SENSOR_ERROR_VALUE) is returned.
 *
 * @param rb    Pointer to the ring buffer instance.
 * @param index Logical index of the element to retrieve.
 *
 * @return
 *  - The requested element value
 *  - @ref SENSOR_ERROR_VALUE if @p rb is NULL or @p index is out of range
 */
float    RB_Get(const RingBuffer *rb, uint16_t index);


#endif /* RING_BUFFER_H */
