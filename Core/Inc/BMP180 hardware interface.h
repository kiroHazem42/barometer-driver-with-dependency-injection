/*
 * BMP180 hardware interface.h
 *
 *  Created on: Sep 9, 2024
 *      Author: kirollous hazem
 */

#ifndef INC_BMP180_HARDWARE_INTERFACE_H_
#define INC_BMP180_HARDWARE_INTERFACE_H_

#include "stm32f1xx_hal.h"
uint8_t BMP180_write_parmeters_hardware_interface(uint8_t sla,uint8_t *Data,uint8_t length);
uint8_t BMP180_read_parmeters_hardware_interface(uint8_t sla,uint8_t *Data,uint8_t length);
uint8_t BMP180_delay_hardware_interface(uint8_t delay);

#endif /* INC_BMP180_HARDWARE_INTERFACE_H_ */
