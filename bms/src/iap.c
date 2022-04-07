/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

typedef volatile uint32_t vu32;

//跳转入口函数指针声明
iapfun jump2app;

uint32_t pIapbufWriteCount = 0;
AppBinHandle_t appBin;
uint8_t IsIAPBusy = 0;

void InvertUint8(uint8_t *dBuf, uint8_t *srcBuf)
{
	int i;
	unsigned char tmp[4];
	tmp[0] = 0;
	for (i = 0; i < 8; i++)
	{
		if (srcBuf[0] & (1 << i))
			tmp[0] |= 1 << (7 - i);
	}
	dBuf[0] = tmp[0];
}

void InvertUint16(uint16_t *dBuf, uint16_t *srcBuf)
{
	int i;
	uint16_t tmp[4];
	tmp[0] = 0;
	for (i = 0; i < 16; i++)
	{
		if (srcBuf[0] & (1 << i))
			tmp[0] |= 1 << (15 - i);
	}
	dBuf[0] = tmp[0];
}

uint16_t CRC16_MODBUS(uint8_t *puchMsg, uint32_t usDataLen, uint16_t wCRCin)
{
	//uint16_t wCRCin = 0xFFFF;
	uint16_t wCPoly = 0x8005;
	uint8_t wChar = 0;
	uint8_t i;
	//uint16_t crc = wCRCin;
	while (usDataLen--)
	{
		wChar = *(puchMsg++);
		//InvertUint8(&wChar, &wChar);
		wCRCin ^= ((uint16_t)(wChar) << 8);
		for (i = 0; i < 8; i++)
		{
			if (wCRCin & 0x8000)
				wCRCin = (wCRCin << 1) ^ wCPoly;
			else
				wCRCin = wCRCin << 1;
		}
	}
	//InvertUint16(&wCRCin, &wCRCin);
	return (wCRCin);
}

uint8_t CalaCheckSum(uint8_t *puchMsg, uint32_t usDataLen)
{
	uint16_t i;
	uint8_t sum = 0;

	for (i = 0; i < usDataLen; i++)
	{
		sum += *(puchMsg + i);
	}

	return sum;
}

//get one block app bin data ,and use this store in flash memery
void iap_temporaryStore_appbin(uint8_t *data, uint32_t offsetAddr, uint16_t len)
{
	if(offsetAddr > 0x1f800)
	{	
		//最后一块区域保留，不能写
		return;
	}
	Flash_Write(FLASH_START_ADDR_APP2 + offsetAddr, data, len);
	pIapbufWriteCount += len;
}

void save_iap_configration()
{
	appBin.flag = 0xaa;
	appBin.srcFlashAddr = FLASH_START_ADDR_APP2;
	Flash_Write(APP_CONFIG_AREA_ADDR, (uint8_t *)&appBin, sizeof(AppBinHandle_t));
}

void iap_start(uint32_t appSize, uint16_t appCrc)
{
	appBin.appBinByteSize = appSize;
	appBin.appBinCrc = appCrc;
	pIapbufWriteCount = 0;
	IsIAPBusy = 1;
}

uint8_t IsIapBusy(void)
{
	return IsIAPBusy;
}

Iap_ErrorCode iap_verify()
{
	//uint8_t *pIapbuf = CommonRam;

	//uint8_t checksum = 0;
	uint16_t crc = 0;
	//uint32_t readCount = 0;
	//uint32_t readAddr = 0;

	if (pIapbufWriteCount == appBin.appBinByteSize)
	{
		/*while (1)
		{
			readCount =
				(readAddr + 512) < appBin.appBinByteSize ? 512 : (appBin.appBinByteSize - readAddr);

			Flash_Read(FLASH_START_ADDR_APP2 + readAddr, pIapbuf, readCount);
			if (readAddr == 0)
			{
				crc = CRC16_MODBUS(pIapbuf, readCount, 0xFFFF);
			}
			else
			{
				crc = CRC16_MODBUS(pIapbuf, readCount, crc);
			}

			readAddr += readCount;
			if (readAddr >= appBin.appBinByteSize)
			{
				break;
			}
		}*/

		crc = CRC16_MODBUS((uint8_t *)FLASH_START_ADDR_APP2, appBin.appBinByteSize, 0xFFFF);

		//checksum = CalaCheckSum((uint8_t*)FLASH_START_ADDR_APP2, appBin.appBinByteSize);

		if (crc == (uint16_t)appBin.appBinCrc)
		{
			IsIAPBusy = 0;
			return BIN_FILE_DOWN_OK;
		}
		else
		{
		}
	}
	//else
	//{
	//SEGGER_RTT_printf(0,
	//"appBin.appBinByteSize :%d != pIapbufWriteCount :%d\n",
	//appBin.appBinByteSize, pIapbufWriteCount);
	//}
	IsIAPBusy = 0;
	return BIN_CRC_ERROR;
}

void iap_write_appbin(uint32_t appxaddr, uint32_t appbufaddr, uint32_t appsize)
{
	uint32_t readCount;
	uint32_t fwaddr = appxaddr;
	uint16_t newCrc;

	while(1)
	{
		readCount =
				(appbufaddr-FLASH_START_ADDR_APP2 + 512) < appsize ? 512 : (appsize - (appbufaddr-FLASH_START_ADDR_APP2));
		Flash_Write(fwaddr, (uint8_t*)appbufaddr, readCount);
		fwaddr += readCount;
		appbufaddr += readCount;

		if(appbufaddr-FLASH_START_ADDR_APP2 >= appsize)
		{
			break;
		}
	}

	//计算新的 crc code 到app1可读取的地方
	newCrc = CRC16_MODBUS((uint8_t*)appxaddr, appsize, 0xFFFF);
	//写入新的crc到最后两个字节
	Flash_Write(APP_CONFIG_CRC_ADDR,(uint8_t*)&newCrc,2);
}

uint8_t IapHanle(uint8_t cmd,uint8_t len,uint8_t *inBuf,uint8_t *outBuf)
{
	uint32_t appbinWriteAddr;
	uint16_t appbinWriteLen;
	uint32_t appbinSize;
	uint16_t appbinCrc;
	uint16_t retLen;

	switch(cmd)
	{
	case 0xF7: //iap start
		//指令 5a 07 04 xx xx xx xx(size) xx xx(crc16) 04
		appbinSize = ((uint32_t)*(inBuf + 6))<<24;
		appbinSize = appbinSize+((uint32_t)*(inBuf + 7))<<16;
		appbinSize = appbinSize+((uint32_t)*(inBuf + 8))<<8;
		appbinSize = appbinSize+ (uint32_t)*(inBuf + 9);

		appbinCrc = ((uint16_t)*(inBuf + 10))<<8;
		appbinCrc = appbinCrc+ (uint16_t)*(inBuf + 11);

		iap_start(appbinSize,appbinCrc);

		*outBuf = 0x68;
		*(outBuf +1) = 0x31;
		*(outBuf +2) = 0xCE;
		*(outBuf +3) = 0x68;
		*(outBuf +4) = 0xF7;//CMD
		*(outBuf +5) = 0x01;//L

		*(outBuf +6) = 0x01;//OK
		*(outBuf +7) = *outBuf + *(outBuf +1)+*(outBuf +2)+*(outBuf +3)+*(outBuf +4)+*(outBuf +5)+*(outBuf +6);//CS
		*(outBuf +8) = 0x16;//end
		retLen = 9;

		break;
	case 0xF8:
		appbinWriteAddr = ((uint32_t)*(inBuf + 6))<<24;
		appbinWriteAddr = appbinWriteAddr + ((uint32_t)*(inBuf + 7))<<16;
		appbinWriteAddr = appbinWriteAddr + ((uint32_t)*(inBuf + 8))<<8;
		appbinWriteAddr = appbinWriteAddr + ((uint32_t)*(inBuf + 9));//本次下发数据快的偏移地址
		appbinWriteLen = (uint16_t)*(inBuf + 10);//本次下发数据的长度，固定240字节							
		//appbinWriteLen = appbinCrc+(uint16_t)*(CommdRxBuf + 8);//
		//memcpy(CommonRam, CommdRxBuf + 8, appbinWriteLen);//iap write bin data
		iap_temporaryStore_appbin(inBuf + 8+3, appbinWriteAddr, appbinWriteLen);

		//回应命令
		*outBuf = 0x68;
		*(outBuf +1) = 0x31;
		*(outBuf +2) = 0xCE;
		*(outBuf +3) = 0x68;
		*(outBuf +4) = 0xF8;
		*(outBuf +5)  =0x05;
		*(outBuf +6)  = 0x01;
		//回应已烧录地址
		*(outBuf+7) = (uint8_t)(appbinWriteAddr>>24);
		*(outBuf+8) = (uint8_t)(appbinWriteAddr>>16);
		*(outBuf+9) = (uint8_t)(appbinWriteAddr>>8);
		*(outBuf+10) = (uint8_t)(appbinWriteAddr);

		*(outBuf +11) = *outBuf + *(outBuf +1)+*(outBuf +2)+*(outBuf +3)+*(outBuf +4)+*(outBuf +5)+*(outBuf +6)+\
					*(outBuf +7)+*(outBuf +8)+*(outBuf +9)+*(outBuf +10);//CS
		*(outBuf +12) = 0x16;//end

		retLen = 13;
		break;
	case 0xF9: //iap verify
		if (iap_verify() == BIN_FILE_DOWN_OK)
		{
			save_iap_configration();
			*(outBuf +6) = 0x01;//OK
		}
		else
		{
			*(outBuf +6) = 0x02;//NOK
		}

		*outBuf = 0x68;
		*(outBuf +1) = 0x31;
		*(outBuf +2) = 0xCE;
		*(outBuf +3) = 0x68;
		*(outBuf +4) = 0xF9;//CMD
		*(outBuf +5) = 0x01;//L
		
		*(outBuf +7) = *outBuf + *(outBuf +1)+*(outBuf +2)+*(outBuf +3)+*(outBuf +4)+*(outBuf +5)+*(outBuf +6);//CS
		*(outBuf +8) = 0x16;//end
		retLen = 9;

		break;
	}

	return retLen;
}


/*
 * IAR Compiler
 */
#if defined ( __ICCARM__ ) || defined ( __GNUC__ )

void MSR_MSP(uint32_t addr)
{
	__ASM("msr msp, r0"); // set Main Stack value 将主堆栈地址保存到MSP寄存器(R13)中
	__ASM("bx lr");		// 跳转到lr中存放的地址处。bx是强制跳转指令 lr是连接寄存器，是STM32单片机的R14
}
#elif defined ( __CC_ARM )

__ASM void MSR_MSP(uint32_t addr)
{
	 MSR MSP, r0 
	  BX r14
}

#endif // __
//默认APP编译存放的地址为0x1000开始，如果改变了APP开始地址，请修改asm(" LDR R0,  =0x00001000")到相应的地址
void iap_load_app(uint32_t appxaddr)
{
	if (((*(vu32 *)appxaddr) & 0x2FFE0000) == 0x20000000)
	{
		jump2app = (iapfun) * (vu32 *)(appxaddr + 4);
		//__disableInterrupts();
		//MSR_MSP(*(uint32_t *)appxaddr);
		__set_MSP(*(uint32_t *)appxaddr);
		jump2app();
	}
}
