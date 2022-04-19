/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"


#if (MCU_LIB_SELECT == 1)

#elif (MCU_LIB_SELECT == 2)

//static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);

void Flash_Write(uint32_t StartAddr, uint8_t *writeDataBuf, uint16_t dataLen)
{
/*	uint8_t *pRbuf;
	uint16_t i;
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;
	uint32_t offaddr;

	pRbuf = CommonRam;

	// if (StartAddr > (FLASH_SECTOR_SIZE+FLASH_SECTOR_NUMS * FLASH_SECTOR_SIZE))
	// 	return; //

	offaddr = StartAddr - FLASH_BASE_ADDR;	//
	secpos = offaddr / FLASH_SECTOR_SIZE;	//
	secoff = (offaddr % FLASH_SECTOR_SIZE); //
	secremain = FLASH_SECTOR_SIZE - secoff; //
	if (dataLen <= secremain)
		secremain = dataLen; //

	
	while (1)
	{
		// if(StartAddr+FLASH_SECTOR_SIZE > (FLASH_BASE_ADDR +FLASH_SECTOR_NUMS * FLASH_SECTOR_SIZE))
		// {
		// 	Flash_Read(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR, pRbuf, secoff+dataLen);
		// }else
		// {
		// 	//跨边界读取数据时，会有数据出错
			
		// }

		Flash_Read(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR, pRbuf, FLASH_SECTOR_SIZE); //
		
        for (i = 0; i < secremain; i++)
        {
        if (*(pRbuf + secoff + i) != 0XFF)
            break;
        }
		if (i < secremain) //
		{

		    EEPROM_ERASE(secpos * FLASH_SECTOR_SIZE, secremain) ;
			for (i = 0; i < secremain; i++) //复制
			{
				*(pRbuf + secoff + i) = *(writeDataBuf + i);
			}
			Flash_Write_Nocheck(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR, pRbuf, FLASH_SECTOR_SIZE);
		}
		else
		 	Flash_Write_Nocheck(StartAddr, writeDataBuf, secremain);

		if (dataLen == secremain)
			break; //
		else
		{
			secpos++;
			secoff = 0;
			writeDataBuf += secremain;
			StartAddr += secremain;
			dataLen -= secremain;
			if (dataLen > (FLASH_SECTOR_SIZE))
				secremain = FLASH_SECTOR_SIZE;
			else
				secremain = dataLen;
		}
	}
*/

      Flash_Read(StartAddr,CommonRam,EEPROM_PAGE_SIZE);
      EEPROM_ERASE(StartAddr, EEPROM_PAGE_SIZE);

      tmos_memcpy(CommonRam,writeDataBuf,dataLen);

      /* 编程DataFlash */
      EEPROM_WRITE(StartAddr,CommonRam, EEPROM_PAGE_SIZE);


	
}

//MUsT 4byte align
void Flash_Read(uint32_t StartAddrr, uint8_t *readDataBuf, uint16_t dataLen)
{

	EEPROM_READ(StartAddrr,CommonRam2,dataLen);
	tmos_memcpy(readDataBuf,CommonRam2,dataLen);
}

//static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
//{
//    tmos_memcpy(CommonRam2,pBuffer,NumToWrite);
//    EEPROM_WRITE(WriteAddr, CommonRam2 ,NumToWrite);
//
//}

#endif
