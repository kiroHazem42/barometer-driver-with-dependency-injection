/*
 * BMP180 hardware interface.c
 *
 *  Created on: Sep 9, 2024
 *      Author: kirollous hazem
 */
#include "BMP180 hardware interface.h"
extern I2C_HandleTypeDef hi2c1;//i got the hi2c1 from the main.c
uint8_t BMP180_write_parmeters(uint8_t sla,uint8_t *Data,uint8_t length){


	HAL_StatusTypeDef ok = HAL_I2C_Master_Transmit(&hi2c1, //this transmit to confirm which reg address i want to reach
			sla << 1 /*we shifted by on to creat the last bit we removed to make it generic in the generic driver
			 then now we re return the last bit to make the transmit function abl to handle the last bit*/
			 ,Data, length, 100);
	return (ok == HAL_OK) ? 1 : 0;
}





uint8_t BMP180_read_parmeters(uint8_t sla,uint8_t *Data,uint8_t length){
	HAL_StatusTypeDef ok = HAL_I2C_Master_Receive(&hi2c1, //this transmit to confirm which reg address i want to reach
				sla<<1 ,Data, length, 100);
		return (ok == HAL_OK) ? 1 : 0;


}
uint8_t BMP180_delay(uint8_t delay){
	HAL_Delay(delay);
}

