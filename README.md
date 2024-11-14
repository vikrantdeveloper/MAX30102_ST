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



MAX30102_ST
This repository contains the firmware for the MAX30102 sensor module, utilizing FreeRTOS for task management. The project is designed to acquire sensor data and log it to an SD card or to the console, depending on the branch. It is structured into two branches:

main branch: Includes the integration of the SD card for logging, captures sensor data using FreeRTOS, and uses multiple tasks for efficient data handling.
develop branch: Contains an initial version where sensor data is acquired and logged within a single while loop (without task management).
Overview
The project aims to acquire data from the MAX30102 sensor and log it either to an SD card or the UART console, depending on the branch:

In the main branch:
The sensor data acquisition and logging have been separated into different FreeRTOS tasks.
One task is responsible for acquiring data from the MAX30102 sensor.
Another task is responsible for logging the acquired data to the SD card.
The SD card integration is used to store sensor data for later retrieval, and FreeRTOS handles the concurrent execution of tasks.
In the develop branch:
Sensor data acquisition and logging are performed in a single while loop.
This version does not utilize FreeRTOS tasks and runs in a blocking manner.
Branch Structure
1. main branch (Assignments 4 & 5 Combined):
This branch contains the code for both Assignment 4 and Assignment 5 combined. In this branch:

SD Card Integration: The SD card is used for storing sensor data, and the data is logged to it.
FreeRTOS Task Management: The code is refactored to use FreeRTOS with two separate tasks: one for sensor data acquisition and one for logging the data to the SD card.
2. develop branch (Assignments 1, 2, & 3):
This branch contains the code for Assignments 1, 2, and 3. The approach is different in this branch:

Sensor data acquisition and logging are done in a single while loop.
FreeRTOS is not used in this version, and the operations are executed sequentially without any multitasking.
Folder Structure
The project is organized into the following folders:

1. /src
Contains the main source code files:

main.c: The main program file that initializes peripherals, creates tasks (in main branch), and starts the scheduler.
max30102.c and max30102.h: Functions for interacting with the MAX30102 sensor.
sd_card.c and sd_card.h: Functions for interacting with the SD card, initializing it, and writing data to it.
uart.c and uart.h: Functions for handling UART communication to log data to the console.
2. /FreeRTOS
Contains the FreeRTOS kernel files and configuration for task management:

FreeRTOSConfig.h: Configuration for FreeRTOS parameters.
tasks.c: Contains the implementations of FreeRTOS tasks (sensor acquisition and logging).
semphr.c: Semaphore handling for synchronization between tasks.
3. /inc
Contains all the header files:

main.h: Declarations for global variables and function prototypes used in main.c.
max30102.h: Header file for MAX30102 sensor functions.
sd_card.h: Header file for SD card functions.
uart.h: Contains UART communication declarations.
4. /drivers
Includes low-level drivers for hardware peripherals:

gpio.c and gpio.h: GPIO configuration.
i2c.c and i2c.h: I2C configuration for MAX30102 communication.
usart.c and usart.h: USART configuration for UART communication.
5. /max30102
Contains files related to MAX30102 sensor data processing:

filters.c: Implements filtering algorithms for sensor data.
heartrate.c: Contains functions for calculating heart rate.
max30102.c: Functions for initializing and interacting with the MAX30102 sensor.
spo2.c: Implements functions for SpO2 calculation.
Header Files:
filters.h: Declares filter functions for data processing.
heartrate.h: Declares functions for heart rate calculation.
max30102.h: Declares sensor initialization and data reading functions.
spo2.h: Declares functions for SpO2 calculation.
6. /logs
Contains files for logging data to the console:

erlog.c: Implements functions for writing logs to the console using UART.
erlog.h: Declares functions for logging data and handling console output.
Currently, sensor data is logged to the console via erlog.c in case of SD card mounting issues. Once the SD card integration is stable, the logs will be redirected to the SD card.

Task Overview (for main branch)
Sensor Acquisition Task (fun_sensor_acq):
Continuously acquires data from the MAX30102 sensor (heart rate, SpO2).
The data is stored in global variables (temp, beatsPerMinute, spo2) for later processing.
Logging Task (fun_log_sd_card):
Logs the acquired data to an SD card.
Uses a binary semaphore to synchronize between the sensor acquisition and logging tasks.
SD Card Integration:
The SD card is initialized and used to log the sensor data in a file.
The SD card functions are implemented in sd_card.c and sd_card.h.
Semaphore Handling:
The tasks use semaphores for task synchronization, ensuring that the data is logged after it has been acquired by the sensor task.
