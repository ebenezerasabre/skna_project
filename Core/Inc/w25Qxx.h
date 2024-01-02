/*
 * w25Qxx.h
 *
 *  Created on: Dec 18, 2023
 *      Author: ebenezer
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

void W25Q_Reset(void);
uint32_t W25Q_ReadID(void);
void W25Q_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t * rData);
void W25Q_Fast_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t * rData);
void write_enable(void);
void write_disable(void);
void W25Q_erase_sector(uint16_t numSector);
void W25Q_Write_Page(uint32_t page, uint16_t offset, uint32_t size, uint8_t * data);



#endif /* INC_W25QXX_H_ */
