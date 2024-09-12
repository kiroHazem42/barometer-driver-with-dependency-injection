# BMP180 Driver generic 

## Overview

This project is a generic driver for the **BMP180** barometric pressure sensor, which can measure atmospheric pressure, temperature, and altitude. The driver is written in C and follows a modular, hardware-agnostic approach that can be integrated with various platforms.

The BMP180 driver is designed for efficient handling of I2C communication, reading and processing calibration data, and converting raw sensor data into meaningful pressure and temperature readings. It also includes altitude calculations based on pressure readings.

## Features

- **Supports all OSS modes**: Ultra-low power, Standard, High-resolution, and Ultra-high resolution modes.
- **Pressure and Temperature Measurement**: The driver calculates compensated pressure and temperature based on raw sensor data.
- **Altitude Calculation**: Uses pressure data to calculate altitude.
- **Hardware Abstraction Layer (HAL) Support**: Abstracts low-level I2C communication and delays using function pointers, allowing easy portability across different microcontroller platforms.
- **Modular Design**: The driver is divided into logical modules with a clean interface for ease of use.

## File Structure

- `BMP_180.h`: Header file containing the driver configuration structure, function prototypes, and definitions.
- `BMP_180.c`: Source file that contains the main implementation of the BMP180 driver, including sensor initialization, data reading, and processing.
- `BMP180 hardware interface.h`: Header file defining the hardware interface functions for I2C communication and delays.
- `BMP180 hardware interface.c`: Source file implementing the STM32-specific hardware interface functions using HAL.

## How It Works

The driver follows the BMP180 sensor's workflow as per the datasheet:

1. **Initialization**:
   - The driver reads the calibration data stored in the sensor's EEPROM and stores them in the configuration structure for later use.
  
2. **Raw Data Reading**:
   - The sensor provides raw, uncompensated temperature and pressure data, which are then processed using the calibration data.
  
3. **Compensated Temperature and Pressure**:
   - The raw values are converted to actual temperature and pressure using the sensor's calibration constants.

4. **Altitude Calculation**:
   - Based on the current pressure reading, the driver calculates the altitude using the barometric formula.

## Driver API

### 1. Initialization

```c
BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *B);
Initializes the BMP180 sensor and reads the calibration data from EEPROM.

###2. Reading Temperature

uint32_t bmp_180_read_temp(bmp_180_conf_t *B);
Reads the temperature in °C from the sensor.

###3. Reading Pressure
uint32_t bmp_180_read_press(bmp_180_conf_t *B);
Reads the pressure in Pa from the sensor.

###4. Reading Altitude

BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t *B, float *altitude);
Calculates the altitude based on the current pressure reading.

##Configuration Structure
The driver uses a configuration structure bmp_180_conf_t to store calibration data, sensor settings, and function pointers to hardware-specific functions (I2C write, read, and delay):
typedef struct {
    uint8_t I2C_Buffer[25];
    BMP180_oss_mode_t oss_mode;
    int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
    uint16_t AC4, AC5, AC6;
    int32_t UT, UP, T, p, B5_T_P;
    struct {
        uint8_t (*BMP180_write_parmeters)(uint8_t sla, uint8_t *Data, uint8_t length);
        uint8_t (*BMP180_read_parmeters)(uint8_t sla, uint8_t *Data, uint8_t length);
        uint8_t (*BMP180_delay)(uint8_t delay);
    } BMP180_AGhardware_interface_t;
} bmp_180_conf_t;
##Hardware Abstraction Layer (HAL)
The driver uses function pointers to interface with the underlying hardware for I2C communication and delays. This makes the driver agnostic to the specific microcontroller used. In this project, the STM32 HAL functions are implemented for these interfaces in BMP180 hardware interface.c:

###uint8_t BMP180_write_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length);
###uint8_t BMP180_read_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length);
###uint8_t BMP180_delay_hardware_interface(uint8_t delay);
##Dependencies
###STM32 HAL: The driver relies on STM32 HAL for I2C communication and delays. Ensure that the STM32CubeMX or STM32 HAL library is included in your project.
###Math Library: The driver uses the standard math library for calculations (e.g., pow, powf).

#include "BMP_180.h"
#include "BMP180 hardware interface.h"

// Create configuration object
bmp_180_conf_t BMP180;

##int main(void) {
    // Initialize HAL and hardware interface
    HAL_Init();
    // Initialize BMP180 with the configuration struct
    bmp_180_init(&BMP180);

    // Read temperature, pressure, and altitude
    uint32_t temp = bmp_180_read_temp(&BMP180);
    uint32_t pressure = bmp_180_read_press(&BMP180);
    float altitude;
    bmp_180_read_altitude(&BMP180, &altitude);
}
##License
This project is licensed under the MIT License - see the LICENSE file for details.

##Author
###Kirollos Hazem - September 2024

This `README.md` provides a professional overview of your BMP180 driver project, including how the driver 