/*
 * BMP_180.c
 *
 *  Created on: Sep 7, 2024
 *      Author: kirollos hazem */
#include "BMP_180.h"
#include "math.h"

//#define sla_address_read 0xEF //1110(E) 1111(F)
//#define sla_address_write 0xEE //1110(E) 1110(E)
//#define BMP180_SLA_ADDRESS 0xef>>1 //=0x77 the given slave address is 7 bit address as take care that your library must handle the last bit
//when reading and writing after shifting 0111 0111 it wil be 0x77 after shifting right
#define p0 101325.f
//////////////////////////hardware interface "needs to pass about the micro controller" ///////////////////////////

/*
 uint8_t I2C_Write(uint8_t SLA,uint8_t *Data, uint8_t Length,bmp_180_conf_t *conf) {
 HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(clock->I2C_BUS, SLA<<1 ,//<< 1,
 Data, Length, 100);
 //return __HAL_I2C_GET_FLAG(clock->I2C_BUS ,HAL_I2C_ERROR_AF) ? 0 : 1;
 return (ok == HAL_OK) ? 1 : 0;}

 static uint8_t I2C_Read(uint8_t *Data, uint8_t Length,bmp_180_conf_t *conf) {
 HAL_StatusTypeDef ok = HAL_I2C_Master_Receive(clock->I2C_BUS, sla_address_read, //<< 1,
 Data, Length, 100);
 //return __HAL_I2C_GET_FLAG(clock->I2C_BUS ,HAL_I2C_ERROR_AF) ? 0 : 1;
 return (ok == HAL_OK) ? 1 : 0;
 }
 */

/*
 *there no need to this equation for write only due to flow chart of data sheet
 static uint8_t (?????)BMP180_WriteConfig(
 uint8_t *data,uint8_t length,bmp_180_conf_t *B) {
 ///start reading tempreture by loading the I2c buffer and transmitting this buffer in any case to send the temp buffer and case buffer
 /// in one time not to take multiple cycles
 B->I2C_Buffer[0] = 0xF4; //register address
 B->I2C_Buffer[1] = 0x2E; //value to start reading temp
 HAL_Delay(4.5);


 }
 */
//////////////////////////hardware interface/////////////
///
static uint8_t BMP180_read_eeprom_parmeters(bmp_180_conf_t *B) {

	B->I2C_Buffer[0] = 0xAA;
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 1)) {
		return BMP180_failed;
	}

	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 22)) {uint16_t *Param = &B->AC1;

	for (int x = 0; x < 11; x++) {
		Param[x] = (B->I2C_Buffer[2 * x] << 8) | (B->I2C_Buffer[2 * x] + 1);

	}
		//return BMP180_failed;
	}
	/*uint16_t *Param = &B->AC1;

	for (int x = 0; x < 11; x++) {
		Param[x] = (B->I2C_Buffer[2 * x] << 8) | (B->I2C_Buffer[2 * x] + 1);

	}*/
	return BMP180_success;
}

inline uint8_t/*(????)*/BMP180_read_uncompensated_measurments(bmp_180_conf_t *B) {
	////////////////////////////read uncompensated temperature//////////////////
	B->I2C_Buffer[0] = 0xF4; //register address
	B->I2C_Buffer[1] = 0x2E; //value to start reading temp
	/*	HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS, //this transmit to confirm which reg address i want to reach
	 BMP180_SLA_ADDRESS, B->I2C_Buffer, 2, 100);*/
	B->BMP180_AGhardware_interface_t.BMP180_delay(5); //delay as data sheet
	/*return (ok == HAL_OK) ? 1 : 0;*/

	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}

	B->I2C_Buffer[0] = 0xF6;
	B->I2C_Buffer[1] = 0xF7;
	/*ok = HAL_I2C_Master_Receive(B->I2C_BUS, //confirm reading
	 BMP180_SLA_ADDRESS, B->I2C_Buffer, 2, 100);*/
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}

	B->UT = B->I2C_Buffer[0] << 8 | B->I2C_Buffer[1]; //merge the two reading with each other for UT to be 16 bit
	//return (ok == HAL_OK) ? 1 : 0;

	////////////////////////////read uncompensated pressure//////////////////

	B->I2C_Buffer[0] = 0xF4; //register address
	B->I2C_Buffer[1] = 0x34 | (B->oss_mode << 6);   //
	//ok = HAL_I2C_Master_Transmit(B->I2C_BUS, //this transmit to confirm which reg address i want to reach
	//BMP180_SLA_ADDRESS, B->I2C_Buffer, 2, 100);
	//return (ok == HAL_OK) ? 1 : 0;
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_write_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}
	B->I2C_Buffer[0] = 0xF6;
	B->I2C_Buffer[1] = 0xF7;
	B->I2C_Buffer[2] = 0xF8;
	//ok = HAL_I2C_Master_Receive(B->I2C_BUS, //confirm reading
	//	BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
	if (1 != B->BMP180_AGhardware_interface_t.BMP180_read_parmeters(
	BMP180_SLA_ADDRESS, B->I2C_Buffer, 2)) {
		return BMP180_failed;
	}
	B->UP = (B->I2C_Buffer[0] << 16 | B->I2C_Buffer[1] << 8 | B->I2C_Buffer[2])
			>> (8 - B->oss_mode);

	//return (ok == HAL_OK) ? 1 : 0;

	///////by choosing any case of these this already intilizes the start of conversion as its from the given combined register values from data sheet
	/// as it gives a register value for both(start of conversion+mood)
	switch (B->oss_mode) {

	case oss_0:    // Handle ultra low power mode settings

		//B->I2C_Buffer[0] = 0xf4; //register address
		//B->I2C_Buffer[2] = 0x34; //
		//HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		//BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100); //3 as i passer 3 registers data in the buffer
		//the function HAL_I2C_Master_Transmit returns me if all good the HAL_OK so i will use it to compare with (ok that have type of HAL_StatusTypeDef)
		//HAL_Delay(4.5);
		B->BMP180_AGhardware_interface_t.BMP180_delay(5);
		//return (ok == HAL_OK) ? 1 : 0; //i will yse the values 1 0 in my local check in the user interface function by successful or failed

		break;

	case oss_1:		// Handle standard mode settings

		//B->I2C_Buffer[0] = 0xf4; //register address
		//	B->I2C_Buffer[2] = 0x74;
		//HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		//BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		//	HAL_Delay(7.5);
		B->BMP180_AGhardware_interface_t.BMP180_delay(8);

		//return (ok == HAL_OK) ? 1 : 0;

		break;

	case oss_2:
		//High Resolution
		// Handle high resolution mode settings
		//	B->I2C_Buffer[0] = 0xf4; //register address
		//B->I2C_Buffer[2] = 0xB4;
		//HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		//BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		//	HAL_Delay(13.5);
		B->BMP180_AGhardware_interface_t.BMP180_delay(14);

		//return (ok == HAL_OK) ? 1 : 0;
		break;

	case oss_3:
		//Ultra High Resolution\n
		// Handle ultra high resolution mode settings
		//	B->I2C_Buffer[0] = 0xF4; //register address
		//B->I2C_Buffer[2] = 0xF4;
		//HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		//BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		//HAL_Delay(22.5);
		B->BMP180_AGhardware_interface_t.BMP180_delay(23);

		//return (ok == HAL_OK) ? 1 : 0;
		break;

	default:
		//Invalid oversampling_setting
		// Handle invalid mode selection
		return -1; /*???????????*/

		break;
	}
}

///////////////////////////////user interface////////////////////
///all the sensor calculation will be here/////

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
//we will use in line fuction t put this function copy in read altitude function without taking cycles in
//calling the function
  uint32_t bmp_180_read_temp(bmp_180_conf_t *B) {
	/*(float)you cant assign the whole expression as float before assigning it*/


	int32_t x1=(B->UT - B->AC6) * (B->AC5 /32768.f);
	int32_t x2 = (B->MC * 2048) / (x1 + B->MD);
	B->B5_T_P = (x1) + ( x2);
	B->T = ( B->B5_T_P + 5) / pow(2, 4);

	//return t and local check
	return B->T; //3amlo beyreturn fe configuration struct ye7amel fiha el data returned
}

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
		B->p = (B7 *2) / B4;
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

BMP180_checkstatus_t bmp_180_read_altitude(bmp_180_conf_t *B, float *altitude) {
	BMP180_checkstatus_t state = BMP180_success;

	//get uncomp pressure and temp
	//state = BMP180_read_uncompensated_measurments(B);

	//put the up and ut manually
	B->UT= 27898,
		B->UP = 23843,
	/*uint32_t temp =ne3ml kda lw ana msh ba return fel function be T gowa el config struct */
	bmp_180_read_temp(B);
	bmp_180_read_press(B);

	// cslculste temp and pressure

	//calculate altitude

	*altitude = 44330.0f * (1.0f - (float) pow((B->p /p0), (1.0f / 5.255)));
	return state;
}

