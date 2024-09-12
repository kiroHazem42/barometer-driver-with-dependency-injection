
# BMP180 Driver generic
# Author: **Kirollos Hazem**  
This project provides a platform-agnostic driver for the BMP180 barometric pressure sensor, designed to be used with any microcontrollers. The driver is modular, easy to integrate, and provides functionality to read temperature, pressure, and altitude data from the sensor via the I2C interface.

## Features

- Read uncalibrated and calibrated temperature and pressure values from the BMP180 sensor.
- Compute altitude based on pressure readings.
- Configurable oversampling settings for pressure readings.
- Hardware abstraction allows for easy integration into any platform using custom I2C and delay implementations.
- Supports STM32 HAL (Hardware Abstraction Layer) for I2C communication.

## Project Structure

```plaintext
├── BMP_180.c
├── BMP_180.h
├── BMP180_hardware_interface.c
├── BMP180_hardware_interface.h
└── README.md
```

### Files

- **BMP_180.h**: Contains the driver configuration structures, enumerations, and function declarations to interface with the BMP180 sensor.
- **BMP_180.c**: Implements the BMP180 driver, including temperature, pressure, and altitude calculations, along with I2C communication.
- **BMP180_hardware_interface.c**: Provides hardware-specific functions (I2C and delay) for STM32 using the HAL library.
- **BMP180_hardware_interface.h**: Header file for the hardware interface implementations.

## Usage

### 1. Initialization

To use the BMP180 driver, you must initialize it by calling `bmp_180_init`. This function reads the EEPROM calibration data from the BMP180 sensor and prepares the driver for operation.

```c
bmp_180_conf_t bmp_config;
BMP180_checkstatus_t status = bmp_180_init(&bmp_config);
if (status == BMP180_success) {
    // Initialization successful
}
```

### 2. Reading Temperature and Pressure

After initialization, you can read the uncompensated temperature and pressure, which will then be compensated using the sensor's calibration data.

```c
uint32_t temperature = bmp_180_read_temp(&bmp_config);
uint32_t pressure = bmp_180_read_press(&bmp_config);
```

### 3. Reading Altitude

To calculate the altitude based on the pressure readings:

```c
float altitude;
BMP180_checkstatus_t status = bmp_180_read_altitude(&bmp_config, &altitude);
if (status == BMP180_success) {
    // Altitude successfully calculated
}
```

### 4. Oversampling Mode

The BMP180 sensor supports four oversampling modes to balance between accuracy and speed:

```c
typedef enum {
    oss_0 = 0, // Ultra low power
    oss_1 = 1, // Standard
    oss_2 = 2, // High resolution
    oss_3 = 3  // Ultra high resolution
} BMP180_oss_mode_t;

bmp_config.oss_mode = oss_1;  // Set standard mode
```

## Hardware Interface

The `BMP180_hardware_interface.c` provides HAL-based I2C and delay functions for STM32. These functions abstract the hardware-specific operations like I2C communication and delays, making the driver easily portable to other platforms.

```c
uint8_t BMP180_write_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length);
uint8_t BMP180_read_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length);
uint8_t BMP180_delay_hardware_interface(uint8_t delay);
```

### Example Hardware Implementation

```c
extern I2C_HandleTypeDef hi2c1;  // Obtained from STM32 HAL
uint8_t BMP180_write_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length) {
    HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(&hi2c1, sla << 1, Data, length, 100);
    return (ok == HAL_OK) ? 1 : 0;
}

uint8_t BMP180_read_parmeters_hardware_interface(uint8_t sla, uint8_t *Data, uint8_t length) {
    HAL_StatusTypeDef ok = HAL_I2C_Master_Receive(&hi2c1, sla << 1, Data, length, 100);
    return (ok == HAL_OK) ? 1 : 0;
}

uint8_t BMP180_delay_hardware_interface(uint8_t delay) {
    HAL_Delay(delay);
    return 1;
}
```

## Dependencies

- STM32 HAL Library (for I2C communication and delays)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

Author: **Kirollos Hazem**  
Date: September 2024
