# STM32 Environment Monitor

Modular environmental monitoring system based on **STM32F429**.  
The project measures **temperature, pressure, and humidity**, displays values and **real-time graphs** on an LCD, and implements robust **error handling using FreeRTOS**.

## Features
- **STM32F429 MCU** (HAL, **no CubeMX code generation**)  
- **BME280 sensor** (temperature, pressure, humidity)  
- **LM35 temperature sensor** (ADC + DMA)  
- **LCD visualization** (values + real-time graphs)  
- **Ring buffer** based data storage for historical sensor readings  
- **Button-driven UI** using FreeRTOS queues  
- **Centralized error handling system** running in its own task  
- **Modular and scalable RTOS architecture**

## Architecture

**Tasks / Application Layer:**  
- `SensorTask` – reads LM35 and BME280 sensors, pushes data to ring buffers and queues  
- `DisplayTask` – updates LCD with current sensor readings and graphs  
- `ButtonTask` – handles button input and posts events to `appEventQ`  
- `AppTask` – processes button events (switching graph mode, setting base pressure)  
- `ErrorTask` – handles errors asynchronously  

**Driver Layer:**  
- `bme280` – BME280 sensor driver (I2C)  
- `lm35` – LM35 sensor driver (ADC + DMA)  

**Service Layer:**  
- `ring_buffer` – stores historical sensor readings for smooth graph plotting  
- `error` – centralized error handling and status management  

**RTOS Communication:**  
- `sensorSnapshotQ` – queue for the latest sensor snapshot  
- `appEventQ` – queue for button-driven application events 

## Hardware
- **STM32F429 Discovery**  
- **BME280** (I2C)  
- **LM35DZ** (ADC)  

## Build
- **STM32CubeIDE**  
- **GCC ARM Embedded** 

## Notes
This project is intentionally implemented **without STM32CubeMX code generation** to demonstrate low-level understanding of HAL, peripherals, and **RTOS-based project architecture**.  
User-created modules are contained in the `*/User*` directory.  
Demo images are available in `*/docs/images*`.
