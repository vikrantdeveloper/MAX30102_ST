# MAX30102_ST

This project interfaces with the MAX30102 heart rate and SpO2 sensor using an STM32 microcontroller and FreeRTOS. It collects sensor data and logs it via UART, with SD card logging being temporarily removed due to mounting issues.

## Features
- **Heart rate monitoring**
- **SpO2 measurement**
- **Interrupt-based temperature reading** from MAX30102
- **FreeRTOS-based multi-threading** for data acquisition and logging
- **Data logging via UART** (SD card integration pending)

## Prerequisites
- **STM32 microcontroller** (e.g., STM32F4, STM32L476)
- **MAX30102 sensor module**
- **FreeRTOS**
- **STM32CubeMX/IDE** (for configuration and development)
- **SD card module** (for future integration)

## Installation
Clone the repository to your local machine:
```bash
git clone git@github.com:vikrantdeveloper/MAX30102_ST.git

Branches
main branch: Multi-tasking with FreeRTOS, SD card logging (pending due to mounting issues).
develop branch: Single while loop without FreeRTOS, UART logging.
Folder Structure
/src: Core code files (sensor, SD card, UART).
/FreeRTOS: FreeRTOS config and task management.
/inc: Header files for sensors and peripherals.
/drivers: Low-level hardware drivers.
/max30102: Sensor data processing (filters, heart rate, SpO2).
/logs: Console logging via UART.
Task Overview (Main Branch)
Sensor Task: Continuously acquires data from MAX30102.
Logging Task: Logs data to SD card (UART used for now).
Semaphore: Synchronizes tasks.
Troubleshooting SD Card Issues
Ensure the SD card is formatted to FAT32.
Verify wiring and power to the SD card module.
Conclusion
main branch: FreeRTOS multi-tasking, UART logging, SD card (pending).
develop branch: Single loop, no FreeRTOS, UART logging.