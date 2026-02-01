/**
 * @file    system_init.h
 * @brief   System initialization interface
 *
 * @details
 * This module provides a single function to initialize all
 * core hardware and peripheral components of the system.
 * Returns `true` if all components are successfully initialized,
 * otherwise returns `false`.
 */

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include <stdbool.h>


/**
 * @brief Initialize the system
 *
 * @retval true   All system components initialized successfully
 * @retval false  Initialization failed for one or more components
 */
bool System_Init(void);

#endif /* SYSTEM_INIT_H */
