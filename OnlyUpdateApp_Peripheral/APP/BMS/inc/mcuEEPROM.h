#ifndef _MCU_EEPROM_H
#define _MCU_EEPROM_H

#include "includes.h"


#if(MCU_LIB_SELECT == 1)

#define FLASH_SECTOR_SIZE 512
//#define FLASH_BASE_ADDR 0

#elif(MCU_LIB_SELECT ==2)

#define FLASH_SECTOR_SIZE 256
#define FLASH_SECTOR_NUMS 64
#endif



void Flash_Write(uint32_t StartAddr, uint8_t *writeDataBuf, uint16_t dataLen);
void Flash_Read(uint32_t StartAddrr, uint8_t *readDataBuf, uint16_t dataLen);

#endif
