/*
 * BMP_180.h
 *
 *  Created on: Sep 7, 2024
 *      Author: kirollos hazem
 */
//this file is made to be reachable by the user
#include <stdint.h>
#define BMP180_SLA_ADDRESS 0xEF>>1//standard 7 bit address modular for all platforms
typedef enum {
	oss_0 = 0, oss_1 = 1, oss_2 = 2, oss_3 = 3

} BMP180_oss_mode_t; // oversampling_setting mode (oss)

typedef struct {

	uint8_t I2C_Buffer[25]; //data Buffer
	//the number inside the buffer array relates to the number of registers that can be hold in this buffer
	// it was put to be 25 as the max number of registers that could be hold in one time is 22 registers off the
	// EEPROM as it has 11 output calibrations each output take 2 register bytes (2*11=22)
	//it was put 25 for comfort
// the calculation constants not made as local variable at BMP_180.c to keep their values not changed as
//they are constants forever or it can be made as static BMP_180.c
//by putting it in struct that can be used as an argument in any function make them global and their value doesnt depend
//on the scope
	BMP180_oss_mode_t oss_mode; //oss mode
	int16_t AC1;  //calibration coefficient 0xAA|0xAB
	int16_t AC2; //calibration coefficient  0xAC|0xAD
	int16_t AC3; //calibration coefficient  0xAE|0xAF
	uint16_t AC4; //calibration coefficient 0xB0|0xB1
	uint16_t AC5; //calibration coefficient 0xB2|0xB3
	uint16_t AC6; //calibration coefficient 0xB4|0xB5
	int16_t B1; //calibration coefficient   0xB6|0xB7
	int16_t B2; //calibration coefficient   0xB8|0xB9
	int16_t MB; //calibration coefficient   0xBA|0xBB
	int16_t MC; //calibration coefficient   0xBC|0xBD
	int16_t MD; //calibration coefficient   0xBE|0xBF

	int32_t UT;	//un_compensated temperature coefficient
	int32_t UP;	//un_compensated pressure  coefficient

	int32_t p;	//final pressure
	int32_t T;	//final temperature

	int32_t B5_T_P; //its a shared variable between temperature and pressure calculations
	// i put this variable only here is to be global for all functions
	struct {
		/// @fn uint8_t (*BMP180_write_parmeters)
		/// @brief //function pointer points to function that performs write in your micro_controller library
		/// @param sla //salve address
		/// @param Data //data passed to the Buffer
		/// @param length //length of data in the Buffer

		uint8_t (*BMP180_write_parmeters)(uint8_t sla, uint8_t *Data,
				uint8_t length);

		/// @fn uint8_t (*BMP180_read_parmeters)
		/// @brief //function pointer points to function that performs read in your micro_controller library
		/// @param sla //salve address
		/// @param Data //data passed to the Buffer
		/// @param length //length of data in the Buffer
		uint8_t (*BMP180_read_parmeters)(uint8_t sla, uint8_t *Data,
				uint8_t length);

		/// @fn uint8_t (*BMP180_delay)(uint8_t)
		/// @brief //function pointer points to function that performs Delay in your micro_controller library
		/// @param delay
		/// @return
		uint8_t (*BMP180_delay)(uint8_t delay);

	} BMP180_AGhardware_interface_t; //Agnostic function pointer struct

} bmp_180_conf_t; // configuration struct

typedef enum {
	BMP180_failed, //to check if the bus failed

	BMP180_success, //to check if the bus success

} BMP180_checkstatus_t; //checking

//user_inteface functions

BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *B); //intialization function decleration
BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t *B, float *altitude); //read altitude function decleration
