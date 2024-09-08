/*
 * BMP_180.c
 *
 *  Created on: Sep 7, 2024
 *      Author: victus
 */
#include "BMP_180.h"
#include "math.h"

//#define sla_address_read 0xEF //1110(E) 1111(F)
//#define sla_address_write 0xEE //1110(E) 1110(E)
#define BMP180_SLA_ADDRESS 0xef>>1  //=0x77 the given slave address is 7 bit address as take care that your library must handle the last bit
//when reading and writing after shifting 0111 0111 it wil be 0x77 after shifting right

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

/// @fn uint8_t BMP180_WriteConfig(bmp_180_conf_t*)
/// @brief this function used to write  data using i2c
///
/// @param B its the configuration struct
/// @return
static uint8_t /*(?????)*/BMP180_WriteConfig(
		/*uint8_t *data,uint8_t length,*/bmp_180_conf_t *B) {
	///start reading tempreture by loading the I2c buffer and transmitting this buffer in any case to send the temp buffer and case buffer
	/// in one time not to take multiple cycles
	B->I2C_Buffer[0] = 0xF4; //register address
	B->I2C_Buffer[1] = 0x2E; //value to start reading temp
	HAL_Delay(4.5);

///////by choosing any case of these this already intilizes the start of conversion as its from the given combined register values from data sheet
/// as it gives a register value for both(start of conversion+mood)
	switch (B->BMP180_mode_t) {

	case BMP180_ultra_low_power_mode:    // Handle ultra low power mode settings

		//B->I2C_Buffer[0] = 0xf4; //register address
		B->I2C_Buffer[2] = 0x34; //
		HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100); //3 as i passer 3 registers data in the buffer
		//the function HAL_I2C_Master_Transmit returns me if all good the HAL_OK so i will use it to compare with (ok that have type of HAL_StatusTypeDef)
		HAL_Delay(4.5);
		return (ok == HAL_OK) ? 1 : 0; //i will yse the values 1 0 in my local check in the user interface function by successful or failed

		break;

	case BMP180_standard_mode:		// Handle standard mode settings

		//B->I2C_Buffer[0] = 0xf4; //register address
		B->I2C_Buffer[2] = 0x74;
		HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		HAL_Delay(7.5);
		return (ok == HAL_OK) ? 1 : 0;

		break;

	case BMP180_high_resolution_mode:
		//High Resolution
		// Handle high resolution mode settings
		//	B->I2C_Buffer[0] = 0xf4; //register address
		B->I2C_Buffer[2] = 0xB4;
		HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		HAL_Delay(13.5);
		return (ok == HAL_OK) ? 1 : 0;
		break;

	case BMP180_ultra_high_resolution_mode:
		//Ultra High Resolution\n
		// Handle ultra high resolution mode settings
		//	B->I2C_Buffer[0] = 0xF4; //register address
		B->I2C_Buffer[2] = 0xF4;
		HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(B->I2C_BUS,
		BMP180_SLA_ADDRESS, B->I2C_Buffer, 3, 100);
		HAL_Delay(22.5);
		return (ok == HAL_OK) ? 1 : 0;
		break;

	default:
		//Invalid oversampling_setting
		// Handle invalid mode selection
		return -1; /*???????????*/

		break;
	}

}
static uint16_t /*(?????)*/ BMP180_read_parmeters(bmp_180_conf_t *B) {
uint8_t i=0;
B->I2C_Buffer[0]=0xAA;
for (i=1;i<=21;i++){
	B->I2C_Buffer[i]=0xAA+1;
}
  HAL_StatusTypeDef ok =  HAL_I2C_Master_Receive(B->I2C_BUS, BMP180_SLA_ADDRESS, B->I2C_Buffer, 22, 100);
  return (ok== HAL_OK )? 1:0;

}

static uint32_t/*(????)*/ BMP180_read_measurments(bmp_180_conf_t *B) {



}

///////////////////////////////user interface////////////////////

/// @fn void bmp_180_init(bmp_180_conf_t*, I2C_HandleTypeDef*)
/// @brief to initialize the sensor
/// Initializes a new BMp180 sensor instance
/// @param conf pointer to the sensor configuration
/// @param I2C_BUS pointer to the I2C bus handle
BMP180_checkstatus_t bmp_180_init(bmp_180_conf_t *conf,
	/*	I2C_HandleTypeDef *I2C_BUS *//*(??????)*/) {
	//read the calibration parameters
// initialization (start of conversion) and write the configurations
	uint8_t state1 = 1;
	uint8_t state2 = 1;


	state1 &= BMP180_WriteConfig(&conf); //we make here (1)& (the return of BMP180_writeconfig as it returns 1 if writing was succesful )
	//we made our check here of BMP180_checkstatus_t to make this user interface function doesn't include in it any hal checks
	//so we made BMP180_WriteConfig return its hal check and we compare it with our checking
	if (state1 == 1) {
		return BMP180_success;
	} else
		return BMP180_failed;

   state2&= BMP180_read_parmeters(&conf);
   if (state2 == 1) {
   		return BMP180_success;
   	} else
   		return BMP180_failed;

}

BMP180_checkstatus_t bmp_180_read_press(bmp_180_conf_t *conf, I2C_HandleTypeDef);
//verify callibration data first

