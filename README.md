# STM32 Environment Monitor

Modular environmental monitoring system based on **STM32F429**.
The project measures temperature, pressure and humidity, displays values and graphs on an LCD,
and implements robust error handling.

## Features
- STM32F429 MCU (HAL, **no CubeMX code generation**)
- BME280 sensor (temperature, pressure, humidity)
- LM35 temperature sensor (ADC + DMA)
- LCD visualization (values + real-time graphs)
- Ring buffer based data storage
- Button-driven UI
- Centralized error handling system
- Modular and scalable architecture

## Architecture

App layer: 
- display
- graph
- buttons

Driver layer:
- bme280
- lm35

Service layer:
- ring_buffer
- error


## Hardware
- STM32F429 Discovery
- BME280 (I2C)
- LM35DZ (ADC)

## Build
- STM32CubeIDE
- GCC ARM Embedded

## Notes
This project is intentionally implemented **without STM32CubeMX code generation**
to demonstrate low-level understanding of HAL, peripherals and project architecture.
In *docs/images* you can find **demo photo**.
User created modules contains in */User* directory.
