/**
 * @file    app.h
 * @brief   Application layer interface.
 *
 * @details
 * The main module of the application.
 * Responsible for:
 * - initialization of all subsystems (sensors, display, graph, buttons)
 * - periodic polling of sensors
 * - processing of user events
 * - updating the UI and graph
 *
 * Architectural role:
 * - upper layer (Application layer)
 * - does not contain direct work with peripherals
 * - uses abstract module APIs
 *
 * Calls:
 *  - App_Init() - It must be called once at system startup.
 *  - App_Loop() - Called in the main loop
 */


#ifndef APP_H
#define APP_H

void App_Init(void);
void App_Loop(void);

#endif /* APP_H */
