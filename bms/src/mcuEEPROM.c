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

static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);

void Flash_Write(uint32_t StartAddr, uint8_t *writeDataBuf, uint16_t dataLen)
{
	uint8_t *pRbuf;
	uint16_t i;
	uint32_t secpos;
	uint16_t secoff;
	uint16_t secremain;
	uint32_t offaddr;

	pRbuf = CommonRam;

	// if (StartAddr > (256 * FLASH_SECTOR_SIZE))
	// 	return; //

	offaddr = StartAddr - FLASH_BASE_ADDR;	//
	secpos = offaddr / FLASH_SECTOR_SIZE;	//
	secoff = (offaddr % FLASH_SECTOR_SIZE); //
	secremain = FLASH_SECTOR_SIZE - secoff; //
	if (dataLen <= secremain)
		secremain = dataLen; //

	while (1)
	{
		//跨边界读取数据时，会有数据出错
		Flash_Read(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR, pRbuf, FLASH_SECTOR_SIZE); //
		for (i = 0; i < secremain; i++)
		{
			if (*(pRbuf + secoff + i) != 0XFF)
				break;
		}
		if (i < secremain) //
		{
			///< FLASH目标扇区擦除
			while (Ok != Flash_SectorErase(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR))
			{
				;
			}

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

		//
	}

	//check data is right
	// for (int k = 0; k < dataLen; k++)
	// {
	// 	if (*(writeDataBuf + k) != *(uint8_t *)(StartAddr + k))
	// 	{
	// 		//SEGGER_RTT_printf(0, "write flash failed at %x\n",StartAddr + k);
	// 	}
	// }
}

void Flash_Read(uint32_t StartAddrr, uint8_t *readDataBuf, uint16_t dataLen)
{
	uint16_t i;

	for (i = 0; i < dataLen; i++)
	{
		*(readDataBuf + i) = *(uint8_t *)(StartAddrr + i);
	}
}

static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
	uint16_t i;
	uint8_t readBackData;
	uint8_t count = 0;
	for (i = 0; i < NumToWrite; i++)
	{
		Flash_WriteByte(WriteAddr, *(uint8_t *)(pBuffer + i));
		//readback
		readBackData = *(uint8_t *)(WriteAddr);

		count = 0;
		while (readBackData != *(uint8_t *)(pBuffer + i))
		{
			Flash_WriteByte(WriteAddr, *(uint8_t *)(pBuffer + i));
			count++;

			if (count > 3)
			{
				//SEGGER_RTT_printf(0, "failed at %x\n",WriteAddr);
				break;
			}
		}

		WriteAddr++; //
	}
}
#elif (MCU_LIB_SELECT == 2)

static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite);

void Flash_Write(uint32_t StartAddr, uint8_t *writeDataBuf, uint16_t dataLen)
{
	uint8_t *pRbuf;
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

	FLASH_ClockInit();
	
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
		
		// for (i = 0; i < secremain; i++)
		// {
		// 	if (*(pRbuf + secoff + i) != 0XFF)
		// 		break;
		// }
		//if (i < secremain) //
		{
			FLASH_Unlock();
			FLASH_EraseOnePage(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR);
			FLASH_Lock();

			for (i = 0; i < secremain; i++) //复制
			{
				*(pRbuf + secoff + i) = *(writeDataBuf + i);
			}
			Flash_Write_Nocheck(secpos * FLASH_SECTOR_SIZE + FLASH_BASE_ADDR, pRbuf, FLASH_SECTOR_SIZE);
		}
		// else
		// 	Flash_Write_Nocheck(StartAddr, writeDataBuf, secremain);

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

	
}

void Flash_Read(uint32_t StartAddrr, uint8_t *readDataBuf, uint16_t dataLen)
{
	uint16_t i;
	for (i = 0; i < dataLen; i++)
	{
		*(readDataBuf + i) = *(uint8_t *)(StartAddrr + i);
	}
}

static void Flash_Write_Nocheck(uint32_t WriteAddr, uint8_t *pBuffer, uint16_t NumToWrite)
{
	uint16_t i;
	uint16_t nOfBytes = NumToWrite % 4;

	uint32_t lastWordData = 0xFFFFFFFF;
	FLASH_Unlock();

	for (i = 0; i < NumToWrite / 4; i++)
	{
		FLASH_ProgramWord(WriteAddr, *(uint32_t *)(pBuffer + 4 * i));
		WriteAddr += 4; //
	}

	if (NumToWrite % 4)
	{
		// if(nOfBytes == 1)
		// {
		// 	lastWordData |= *(uint32_t*)(pBuffer+NumToWrite/4);
		// }else
		// if(nOfBytes == 2)
		// {
		// 	lastWordData |= *(uint32_t*)(pBuffer+NumToWrite/4)<<16;
		// }else
		// if(nOfBytes == 3)
		// {
		// 	lastWordData |= *(uint32_t*)(pBuffer+NumToWrite/4)<<8;
		// }
		lastWordData |= *(uint32_t *)(pBuffer + NumToWrite / 4);
		FLASH_ProgramWord(WriteAddr, lastWordData);
	}
	FLASH_Lock();
}

#endif