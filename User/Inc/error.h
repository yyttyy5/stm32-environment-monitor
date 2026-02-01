/**
 * @file    error.h
 * @brief   System error handling module.
 *
 * @details
 * This module provides centralized error handling for the system.
 * It defines error types, triggers and clears errors, and controls LEDs for visual feedback.
 *
 * Responsibilities:
 *  - define error titles and severity
 *  - initialize error LEDs
 *  - trigger and clear errors
 *  - handle critical and recoverable errors
 *
 * Notes:
 *  - critical errors halt the system and display message on LCD
 *  - recoverable errors are indicated with LED blinking and can escalate if not cleared
 */

#ifndef ERROR_H
#define ERROR_H

/** @enum ErrorTitle
 *  @brief List of all possible system errors.
 */
typedef enum {
	SYSTEM_INIT_ERROR,
	LCD_INIT_ERROR,
	BME280_INIT_ERROR,
	BME280_READ_VALUE_ERROR,
	LM35_INIT_ERROR,
	LM35_READ_VALUE_ERROR,
	GRAPH_INIT_ERROR
} ErrorTitle;

/** @enum ErrorType
 *  @brief Severity levels of errors.
 */
typedef enum {
	ERROR_CRITICAL,
	ERROR_RECOVERABLE,
	ERROR_WARNING
} ErrorType;

/** @brief Initialize error LEDs. */
void ErrorLED_Init(void);

/** @brief Trigger an error.
 *  @param error  Error title to trigger
 */
void Error_Trigger(ErrorTitle error);

/** @brief Clear a previously triggered error.
 *  @param error  Error title to clear
 */
void Error_Clear(ErrorTitle error);

/** @brief Handle active errors (LED toggling, escalation). */
void Error_Handle(void);

#endif /* ERROR_H */
