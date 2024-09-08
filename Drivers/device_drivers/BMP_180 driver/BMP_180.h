/*
 * BMP_180.h
 *
 *  Created on: Sep 7, 2024
 *      Author: kirolous hazem
 */
//this file is made to be reachable by the user
#include <stdint.h>
#include "stm32f1xx_hal.h"
/*
 #define sla_address_read 0xEF //1110(E) 1111(F)
 #define sla_address_write 0xEE //1110(E) 1110(E)
 */
#define BMP180_SLA_ADDRESS 0xEF>>1

typedef enum {
	BMP180_ultra_low_power_mode,
	BMP180_standard_mode,
	BMP180_high_resolution_mode,
	BMP180_ultra_high_resolution_mode,
	BMP180_temp_measure,

} BMP180_mode_t;

typedef struct {

	uint8_t I2C_Buffer[25];
	//the number inside the buffer array relates to the number of registers that can be hold in this buffer
	// it was put to be 25 as the max number of registers that could be hold in one time is 22 registers off the
	// EEPROM as it has 11 output calibrations each output take 2 register bytes (2*11=22)
	//it was put 25 for comfort
	I2C_HandleTypeDef *I2C_BUS;	//da 3ashan a7aded anhy device if there is more than device and more than one I2c
	uint8_t BMP180_mode_t;
// the calculation constants not made as local variable at BMP_180.c to keep their values not changed as
//they are constants forever or it can be made as static BMP_180.c

} bmp_180_conf_t;

typedef enum {
	BMP180_success, //to check if the bus success

	BMP180_failed, //to check if the bus failed
} BMP180_checkstatus_t;

//userinteface functions
BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *conf, I2C_HandleTypeDef *I2C_BUS /* ???*/);

BMP180_checkstatus_t bmp_180_read_temp(bmp_180_conf_t *conf,I2C_HandleTypeDef *I2C_BUS );

