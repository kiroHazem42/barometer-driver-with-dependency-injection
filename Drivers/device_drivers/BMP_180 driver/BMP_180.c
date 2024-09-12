/*
 * BMP_180.c
 *
 *  Created on: Sep 7, 2024
 *      Author: kirollos hazem */
#include "BMP_180.h" //including header file
#include "math.h"

#define p0 101325.0f //  pressure at sea level p0 in pascal
//////////////////////////hardware interface "needs to pass about the micro controller" ///////////////////////////

/// @fn uint8_t BMP180_read_eeprom_parmeters(bmp_180_conf_t*)
/// @brief  Hardware interface function to read eeprom data(calibration constants)using i2c
/// @param B configurtion struct type variable
/// @return success if the flow completes successfully if not return BMP180_failed and don't complete flow
static uint8_t BMP180_read_eeprom_parmeters(bmp_180_conf_t *B) {

	B->I2C_Buffer[0] = 0xAA; // writting register address of eeprom
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 1)) {
		return BMP180_failed;
		//return (ok == HAL_OK) ? 1 : 0;this type of chcking example used in hardware interface
		//to return 1 or 0 that be used in BMP_180.c in the checking
	}
	//reading the 22 byte(8 bit) of data from register as every callibration constatnt takes two bytes as illustrated
	//in conf struct
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 22)) {
		return BMP180_failed;	// if writing failed return failed
	}
	uint16_t *Param = &B->AC1;//pointer to points on the first element in calibration constant in the conf. struct

	for (int x = 0; x < 11; x++) {//looping on the 11 calibration constant and putting 2 bytes in each constatnt
		//to make each constatnt 16 bit
		Param[x] = (B->I2C_Buffer[2 * x] << 8) | (B->I2C_Buffer[2 * x] + 1);

	}
	return BMP180_success; //return success if done
}
/// @fn uint8_t BMP180_read_uncompensated_measurments(bmp_180_conf_t*)
/// @brief read the measurements of temp. and pressure but still un_compansated
///
/// @param B configuration struct type variable
/// @return success if the flow completes successfully if not return BMP180_failed and don't complete flow

static uint8_t BMP180_read_uncompensated_measurments(bmp_180_conf_t *B) {
	////////////////////////////read un_compensated temperature//////////////////
	B->I2C_Buffer[0] = 0xF4; //register address
	B->I2C_Buffer[1] = 0x2E; //value to write in register to start reading temp.
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters( // write
			BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}

	B->BMP180_AGhardware_interface_t.BMP180_delay(5); //delay as data sheet

	//prepare to read from address F6 so you have to write
	//any interaction with any register must prepare before this interaction as to write this register address
	B->I2C_Buffer[0] = 0xF6;
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 1)) {
		return BMP180_failed;// if writing failed return failed don't continue
	}

	//read the temp result
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {//2 just to read from the register next to 0xf6 which is 0xf7
		return BMP180_failed;// if writing failed return failed don't continue
	}

	B->UT = B->I2C_Buffer[0] << 8 | B->I2C_Buffer[1]; //merge the two reading with each other for UT to be 16 bit

	////////////////////////////read uncompensated pressure//////////////////

	B->I2C_Buffer[0] = 0xF4; //register address
	B->I2C_Buffer[1] = 0x34 | (B->oss_mode << 6);   //

	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}

	///////by choosing any case of these this already intilizes the start of conversion as its from the given combined register values from data sheet
	/// as it gives a register value for both(start of conversion+mood)
	switch (B->oss_mode) {

	case oss_0:    // Handle ultra low power mode settings

		B->BMP180_AGhardware_interface_t.BMP180_delay(5);

		break;

	case oss_1:		// Handle standard mode settings

		B->BMP180_AGhardware_interface_t.BMP180_delay(8);
		break;

	case oss_2:

		B->BMP180_AGhardware_interface_t.BMP180_delay(14);
		break;

	case oss_3:

		B->BMP180_AGhardware_interface_t.BMP180_delay(23);
		break;

	default:
		//Invalid oversampling_setting
		// Handle invalid mode selection
		return BMP180_failed;

		break;
	}

	//prepare to read from address F6
	B->I2C_Buffer[0] = 0xF6;
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 1)) {
		return BMP180_failed;
	}

	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 3)) {//read three consecutive registers starting from 0xf6 register
		return BMP180_failed;
	}

	B->UP = (B->I2C_Buffer[0] << 16 | B->I2C_Buffer[1] << 8 | B->I2C_Buffer[2])	//merging MSB LSB XLSB
	>> (8 - B->oss_mode);		//calculating UP and return it to the
	//configuration struct agin to make it global to be used between functions
	return BMP180_success;
	//return (ok == HAL_OK) ? 1 : 0;this type of chcking example used in hardware interface
	//to return 1 or 0 that be used in BMP_180.c in the checking
}

///////////////////////////////user interface////////////////////

///all the sensor calculation will be here/////

/// @fn BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t*)
/// @brief initialization of sensor whre the reading of eeprom happen
/// @param B
/// @return success or failed
BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *B) {
	///its type is BMP180_checkstatus_t to give me the local check of success or failed
	///the mode configuration is done in the( read uncompansated function) as the data sheet of the sensor flow chart need
/// initialization (start of conversion) and write the configurations is done in the mode in the (read uncompansated function)

	//read the calibration parameters

	uint8_t state = 1;
	state &= BMP180_read_eeprom_parmeters(B);/*???? msh &B*///we made our check here of BMP180_checkstatus_t to make this user interface function doesn't include in it any hal checks
	//so we made BMP180_WriteConfig return its hal check and we compare it with our checking
	if (state == 1) {
		return BMP180_success;
	} else
		return BMP180_failed;

}

/// @fn uint32_t bmp_180_read_temp(bmp_180_conf_t*)
/// @brief read the final temp T as all calculation to change from
/// UT to T happens here
/// @param B
/// @return T in the config struct to be globally used
uint32_t bmp_180_read_temp(bmp_180_conf_t *B) {
	int32_t x1 = (B->UT - B->AC6) * (B->AC5 / 32768.f);
	int32_t x2 = (B->MC * 2048) / (x1 + B->MD);
	B->B5_T_P = (x1) + (x2);
	B->T = (B->B5_T_P + 5) / pow(2, 4);

	//return t and local check
	return B->T; //3amlo beyreturn fe configuration struct ye7amel fiha el data returned
}
/// @fn uint32_t bmp_180_read_press(bmp_180_conf_t*)
/// @brief  read the final temp P as all calculation to change from
/// UP to P happens here
///
/// @param B
/// @return P in the config struct to be globally used

uint32_t bmp_180_read_press(bmp_180_conf_t *B) {
	int32_t B6 = B->B5_T_P - 4000;
	int32_t x1 = ((B->B2) * (B6 * (B6 / pow(2, 12)))) / pow(2, 11);
	int32_t x2 = ((B->AC2) * B6) / pow(2, 11);
	int32_t x3 = x1 + x2;
	int32_t B3 = ((((((int32_t) B->AC1 * 4) + x3) << B->oss_mode) + 2) / 4);
	x1 = (B->AC3) * (B6 / pow(2, 13));
	x2 = ((B->B1) * (B6 * (B6 / pow(2, 12)))) / pow(2, 16);
	x3 = ((x1 + x2) + 2) / pow(2, 2);
	uint32_t B4 = B->AC4 * (((/*????*/uint32_t) (x3 + 32768)) / pow(2, 15));
	uint32_t B7 = ((uint32_t) B->UP - B3) * (50000 >> B->oss_mode);

	if (B7 < 0x80000000) {
		B->p = (B7 * 2) / B4;
	} else {
		B->p = (B7 / 4) * 2;
	}
	x1 = (B->p / pow(2, 8)) * (B->p / pow(2, 8));/*??????*/
	x1 = (x1 * 3038) / pow(2, 16);
	x2 = (-7357 * B->p) / pow(2, 16);
	B->p = B->p + (x1 + x2 + 3791) / pow(2, 4);
	return B->p;

}

//verify callibration data first
/// @fn BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t*, float*)
/// @brief read the altitude
///
/// @param B
/// @param altitude :pointer to return  the altitude or we can not put
/// it in the agumets if it was in the config struct and return data to it in
/// config struct as made in T P
/// @return

BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t *B, float *altitude) {
	BMP180_checkstatus_t state = BMP180_success;

	//get uncomp pressure and temp
	state = BMP180_read_uncompensated_measurments(B);
	// calculate temp and pressure
	bmp_180_read_temp(B);
	bmp_180_read_press(B);

	// calculate temp and pressure

	//calculate altitude

	*altitude = 44330.0f * (1.0f - powf((B->p / p0), (1.0f / 5.255)));
	return state;
}

