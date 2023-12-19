/*
 * w25Qxx.c
 *
 *  Created on: Dec 18, 2023
 *      Author: ebenezer
 */

#include "main.h"
#include "w25Qxx.h"

extern SPI_HandleTypeDef	hspi2;
#define w25Q_SPI			hspi2

//#define csLOW()		HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
//#define csHIGH()	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

#define numBlock	32	// number of total blocks for 16Mb flash

void csLOW(void){
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}

void csHIGH(void){
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
}


void SPI_Read(uint8_t * data, uint16_t len){
	HAL_SPI_Receive(&w25Q_SPI, data, len, 5000);
}

void SPI_Write(uint8_t * data, uint16_t len){
	HAL_SPI_Transmit(&w25Q_SPI, data, len, 2000);
}

/*
 * Reset command at 8.2.37
 */
void W25Q_Reset(void){
	uint8_t tData[2];
	tData[0] = 0x66;	// enable reset
	tData[1] = 0x99;	// Reset
	csLOW();
	SPI_Write(tData, 2);
//	HAL_SPI_Transmit(&w25Q_SPI, tData, 2, 1000);
	csHIGH();
	HAL_Delay(100); // let device settle properly after reset
}


uint32_t W25Q_ReadID(void){
	uint8_t	tData = 0x9F;	// Read JEDEC ID
	uint8_t	rData[3];
	csLOW();
	SPI_Write(&tData, 1);
	SPI_Read(&rData[0], 3);
//	HAL_SPI_Transmit(&w25Q_SPI, &tData, 1, 1000);
//	HAL_SPI_Receive(&w25Q_SPI, &rData[0], 3, 3000);
	csHIGH();
	return ((rData[0]<<16) | (rData[1]<<8) | rData[2]); // a 24_bit JEDEC ID
}



void W25Q_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t * rData){
	uint8_t tData[5];
	uint32_t memAddr = (startPage * 256) + offset;

	if(numBlock < 512){
		tData[0] = 0x03; 	// enable Read
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 8) & 0xFF;
		tData[3] =  memAddr & 0xFF;			// LSB of the Mem Address
	} else {
		tData[0] = 0x03; 					// enable Read
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >> 8) & 0xFF;
		tData[4] =  memAddr & 0xFF;			// LSB of the Mem Address
	}

	csLOW();				// pull the CS Low
	if(numBlock < 512){
		SPI_Write(tData, 4);
	} else {
		SPI_Write(rData, 5);
	}
	SPI_Read(rData, size);	// Read the data
	csHIGH();				// pull the CS High
}

void W25Q_Fast_read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t * rData){
	uint8_t tData[6];
	uint32_t memAddr = (startPage * 256) + offset;

	if(numBlock < 512){
		tData[0] = 0x0B; 	// enable Fast Read
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 8) & 0xFF;
		tData[3] =  memAddr & 0xFF;			// LSB of the Mem Address
		tData[4] = 0;						// Dummy clock
	} else {
		tData[0] = 0x0B; 					// enable Fast Read
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >> 8) & 0xFF;
		tData[4] =  memAddr & 0xFF;			// LSB of the Mem Address
		tData[5] = 0; // Dummy clock
	}

	csLOW();				// pull the CS Low
	if(numBlock < 512){
		SPI_Write(tData, 5);
	} else {
		SPI_Write(rData, 6);
	}
	SPI_Read(rData, size);	// Read the data
	csHIGH();				// pull the CS High
}



