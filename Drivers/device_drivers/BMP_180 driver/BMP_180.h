/*
 * BMP_180.h
 *
 *  Created on: Sep 7, 2024
 *      Author: kirollos hazem
 */
//this file is made to be reachable by the user
#include <stdint.h>
/*
 #define sla_address_read 0xEF //1110(E) 1111(F)
 #define sla_address_write 0xEE //1110(E) 1110(E)
 */
#define BMP180_SLA_ADDRESS 0xEF>>1
typedef enum {
	oss_0 = 0, oss_1 = 1, oss_2 = 2, oss_3 = 3

} BMP180_oss_mode_t;

typedef struct {

	uint8_t I2C_Buffer[25];
	//the number inside the buffer array relates to the number of registers that can be hold in this buffer
	// it was put to be 25 as the max number of registers that could be hold in one time is 22 registers off the
	// EEPROM as it has 11 output calibrations each output take 2 register bytes (2*11=22)
	//it was put 25 for comfort
// the calculation constants not made as local variable at BMP_180.c to keep their values not changed as
//they are constants forever or it can be made as static BMP_180.c
//by putting it in struct that can be used as an argument in any function make them global and their value doesnt depend
//on the scope
	BMP180_oss_mode_t oss_mode;
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t UT;	//uncompensated temprature coefficient

	int32_t UP;   	//uncompensated pressure  coefficient
	int32_t p;
	int32_t T;

	int32_t B5_T_P; //its a shared variable between temp. and pres. calculations
    // i put this variable only here is to be global for all functions
    struct {
		uint8_t (*BMP180_write_parmeters)(uint8_t sla,uint8_t *Data,uint8_t length);
		uint8_t (*BMP180_read_parmeters)(uint8_t sla,uint8_t *Data,uint8_t length);
        uint8_t (*BMP180_delay)(uint8_t delay);


	}BMP180_AGhardware_interface_t;



} bmp_180_conf_t;



/*typedef enum {
 BMP180_ultra_low_power_mode,
 BMP180_standard_mode,
 BMP180_high_resolution_mode,
 BMP180_ultra_high_resolution_mode,
 BMP180_temp_measure,

 } BMP180_mode_t;*/
typedef enum {
	BMP180_failed, //to check if the bus failed

	BMP180_success, //to check if the bus success

} BMP180_checkstatus_t;

//userinteface functions

BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *B) ;
BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t *B, float *altitude);
