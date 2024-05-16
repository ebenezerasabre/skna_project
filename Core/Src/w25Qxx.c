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
//	HAL_SPI_Receive(&w25Q_SPI, data, len, 5000);
	HAL_SPI_Receive(&w25Q_SPI, data, len, 3000);
}

void SPI_Write(uint8_t * data, uint16_t len){
//	HAL_SPI_Transmit(&w25Q_SPI, data, len, 2000);
	HAL_SPI_Transmit(&w25Q_SPI, data, len, 1000);
}

/*
 * Reset command at 8.2.37
 */
void W25Q_Reset(void){
	uint8_t tData[2];
	tData[0] = 0x66;	// enable reset
	tData[1] = 0x99;	// Reset
	csLOW();
//	SPI_Write(tData, 2);
	HAL_SPI_Transmit(&w25Q_SPI, tData, 2, 1000);
	csHIGH();
	HAL_Delay(100); // let device settle properly after reset
}


uint32_t W25Q_ReadID(void){
	uint8_t	tData = 0x9F;	// Read JEDEC ID
	uint8_t	rData[3];
	csLOW();
	SPI_Write(&tData, 1);

//	HAL_SPI_Transmit(&w25Q_SPI, &tData, 1, 1000);
//	HAL_SPI_Receive(&w25Q_SPI, &rData[0], 3, 3000);
	SPI_Read(&rData[0], 3);
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
		SPI_Write(tData, 5);
	}
	SPI_Read(rData, size);	// Read the data
	csHIGH();				// pull the CS High
}


void W25Q_Fast_read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t * rData){
	uint8_t tData[6];
	uint32_t memAddr = (startPage * 256) + offset;

	if(numBlock < 512){
		tData[0] = 0x0B; 					// enable Fast Read
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
		tData[5] = 0; 						// Dummy clock
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


void write_enable(void){
	uint8_t tData = 0x06;	// enable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	HAL_Delay(5);	// 5ms delay
}


void write_disable(void){
	uint8_t tData = 0x04;	// disable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	HAL_Delay(5);								// 5ms delay
}


void W25Q_erase_sector(uint16_t numSector){
	uint8_t tData[5];
	uint32_t memAddr = numSector * 16 * 256; 	// Each sector contains 16 pages * 256 bytes

	write_enable();

	if(numBlock < 512){
		tData[0] = 0x20; 					// erase sector
		tData[1] = (memAddr >> 16) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 8) & 0xFF;
		tData[3] =  memAddr & 0xFF;			// LSB of the Mem Address

		csLOW();
		SPI_Write(tData, 4);
		csHIGH();
	} else {
		tData[0] = 0x20; 					// erase sector
		tData[1] = (memAddr >> 24) & 0xFF;	// MSB of Mem Address
		tData[2] = (memAddr >> 16) & 0xFF;
		tData[3] = (memAddr >> 8) & 0xFF;
		tData[4] =  memAddr & 0xFF;			// LSB of the Mem Address

		csLOW();
		SPI_Write(tData, 5);
		csHIGH();
	}

	HAL_Delay(450); // 450 ms for sector erase

	write_disable();

}

uint32_t bytestowrite(uint32_t size, uint16_t offset){
	if((size + offset) < 256) return size;
	else return (256 - offset);
}

uint32_t bytestomodify(uint32_t size, uint16_t offset){
	if((size + offset) < 4096) return size;
	else return (4096 - offset);
}

void W25Q_Write_Page(uint32_t page, uint16_t offset, uint32_t size, uint8_t * data){
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage = startPage + ((size + offset-1)/256);
	uint32_t numPages = endPage - startPage + 1;

	// before we write, we have to erase first
	uint16_t startSector = startPage / 16;
	uint16_t endSector = endPage / 16;
	uint16_t numSectors = endSector - startSector + 1;

	for(uint16_t i = 0; i < numSectors; i++){
		W25Q_erase_sector(startSector++);
	}

	uint32_t dataPosition = 0;

	// write the data
	for(uint32_t i=0; i < numPages; i++){
		uint32_t memAddr = (startPage * 256) + offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		int ndx = 0;

		write_enable();


		if(numBlock < 512){
			tData[0] = 0x02; 					// page program
			tData[1] = (memAddr >> 16) & 0xFF;	// MSB of Mem Address
			tData[2] = (memAddr >> 8) & 0xFF;
			tData[3] =  memAddr & 0xFF;			// LSB of the Mem Address

			ndx = 4;
		} else {
			tData[0] = 0x02; 					// erase sector
			tData[1] = (memAddr >> 24) & 0xFF;	// MSB of Mem Address
			tData[2] = (memAddr >> 16) & 0xFF;
			tData[3] = (memAddr >> 8) & 0xFF;
			tData[4] =  memAddr & 0xFF;			// LSB of the Mem Address
			ndx = 5;
		}
		uint16_t bytestosend = bytesremaining + ndx;

		for(uint16_t i=0; i<bytesremaining; i++){
			tData[ndx++] = data[i+dataPosition];
		}

		/*
		 * He faced issues when writing more than 250 bytes into the flash
		 */
		if(bytestosend > 250){
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData + 100, bytestosend-100);
			csHIGH();
		} else {
			csLOW();
			SPI_Write(tData, bytestosend);
			csHIGH();
		}

		startPage++;
		offset = 0;
		size = size - bytesremaining;
		dataPosition += bytesremaining;

		HAL_Delay(5);
		write_disable();
	}
}


void W25Q_update_Page(uint32_t page, uint16_t offset, uint32_t size, uint8_t * data){
	uint16_t startSector = page / 16;
	uint16_t endSector = (page + ((size + offset - 1) / 256)) / 16;
	uint16_t numSectors = endSector - startSector + 1;

	uint8_t previousData[4096];
	uint32_t sectorOffset = ((page % 16) * 256) + offset;
	uint32_t dataindex = 0;

	for(uint16_t i=0; i < numSectors; i++){
		uint32_t startPage = startSector * 16;
		W25Q_Fast_Read(startPage, 0, 4096, previousData);

		uint16_t bytesRemaining = bytestomodify(size, sectorOffset);
		for(uint16_t i=0; i < bytesRemaining; i++){
			previousData[i + sectorOffset] = data[i + dataindex];
		}

		W25Q_Write_Page(startPage, 0, 4096, previousData);

		startSector++;
		sectorOffset = 0;
		dataindex = dataindex + bytesRemaining;
		size = size - bytesRemaining;
	}
}
















