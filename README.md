# MAX30102_ST

## Overview
This project is a basic implementation of the MAX30102 heart rate and SpO2 sensor with STM32 microcontroller, using FreeRTOS for multi-threading. The project aims to acquire and process sensor data such as temperature, heart rate, and SpO2, then store or log the data for further analysis.

## Features
- Heart rate monitoring
- SpO2 measurement
- Interrupt Based Temperature reading from MAX30102
- FreeRTOS-based multi-threading for data acquisition and logging
- Data logging to SD card - giving mount error so remove from the main freertos code
- UART communication for debugging and data logging

## Prerequisites
- STM32 microcontroller (e.g., STM32F4, STM32L476)
- MAX30102 sensor module
- FreeRTOS
- STM32CubeMX (for configuration)
- STM32CubeIDE (for development and debugging)
- SD card module

## Installation
1. Clone the repository to your local machine:
   ```bash
   git clone git@github.com:vikrantdeveloper/MAX30102_ST.git

