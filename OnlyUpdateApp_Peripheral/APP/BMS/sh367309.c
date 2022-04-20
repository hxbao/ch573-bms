/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/

#include "includes.h"

#if (AFE_CHIP_SELECT == 1)

#define VPRO_CON_ON()                    \
	do                                   \
	{                                    \
	  GPIOB_SetBits(TN_VPRO_CON_PBPIN); \
	} while (0);

#define VPRO_CON_OFF()                     \
	do                                     \
	{                                      \
		GPIOB_ResetBits(TN_VPRO_CON_PBPIN); \
	} while (0);

#define RSENSE AFE_CUR_SAMPLE_RES
#define TRVAL 0x3f

//sh367309 ram 寄存器表
typedef struct
{
	uint8_t conf;
	uint8_t balanceH;
	uint8_t balanceL;
	uint8_t bstatus1;
	uint8_t bstatus2;
	uint8_t bstauts3;
	uint8_t temp1H;
	uint8_t temp1L;
	uint8_t temp2H;
	uint8_t temp2L;
	uint8_t temp3H;
	uint8_t temp3L;
	uint8_t curH;
	uint8_t curL;
	uint8_t cellx[16 * 2];
	uint8_t cadcdh;
	uint8_t cadcdl;
	uint8_t bflag1;
	uint8_t bflag2;
	uint8_t rststat;
} SH_Regs_t;

static void SH_iicWriteRam(uint8_t ramAddr, uint8_t ramRegVal);
static uint8_t SH_iicReadRam(uint8_t ramAddr, uint8_t readSize, uint8_t *pdataBuf);

SH_Regs_t ShRamRegs;
// [EEPROM Data]
// SCONF1 = 0x6D;                 //System Config1.
// SCONF2 = 0xB1;                 //System Config2.
// OV Delay = 1S;                 //Over Voltage Delay Time.
// Load Delay = 100ms;            //Load Detect Delay Time.
// OV Threshold = 4250;           //Over Voltage Threshold.
// OV Recovery = 4150;            //Over Voltage Recovery Threshold.
// UV Delay = 1S;                 //Under Voltage Delay Time.
// UV threshold = 2800;           //Under Voltage Voltage.
// UV Recovery = 3100;            //Under Voltage Recovery Voltage.
// BALV threshold = 4200;         //Balance Voltage Threshold.
// PREV threshold = 2400;         //Pre-Charge Voltage Threshold.
// L0V Threshold = 2000;          //Low-0 Voltage Threshold.
// OVPF Threshold = 4400;         //Over Voltage Protect Failure Threshold.
// OCD1 Threshold = 100;          //Over Current Discharge1 Threshold.
// OCD1 Delay = 1S;               //Over Current Discharge1 Delay Time.
// OCD2 Threshold = 200;          //Over Current Discharge2 Threshold.
// OCD2 Delay = 100ms;            //Over Current Discharge2 Delay Time.
// SC Threshold = 400;            //Short Current Threshold.
// SC Delay = 128us;              //Short Current Delay Time.
// OCC Threshold = 50;            //Over Current Charge Threshold.
// OCC Delay = 1S;                //Over Current Charge Delay Time.
// CHS Threshold = 500uV;        //State of charge and discharge detection voltage.
// MOSFET Delay = 128us;          //MOSFET TurnOn Delay Time.
// OCR Delay = 16S;               //Over Current Recovery Delay Time.
// OVPF Delay = 8S;               //Over Voltage Protect Failure Delay Time.
// OTC Threshold = 60;            //Over Temperature Charge Threshold.
// OTCR Threshold = 55;           //Over Temperature Charge Recovery Threshold.
// UTC Threshold = 0;             //Under Temperature Charge Threshold.
// UTCR Threshold = 5;            //Under Temperature Charge Recovery Threshold.
// OTD Threshold = 70;            //Over Temperature Discharge Threshold.
// OTDR Threshold = 65;           //Over Temperature Discharge Recovery Threshold.
// UTD Threshold = -10;           //Under Temperature Discharge Threshold.
// UTDR Threshold = -5;           //Under Temperature Discharge Recovery Threshold.

uint8_t EEP_RegMap[25] = {
	SH_DEFAULT_EECONFIG_1  ,
	SH_DEFAULT_EECONFIG_2  ,
	SH_DEFAULT_EECONFIG_3  ,
	SH_DEFAULT_EECONFIG_4  ,
	SH_DEFAULT_EECONFIG_5  ,
	SH_DEFAULT_EECONFIG_6  ,
	SH_DEFAULT_EECONFIG_7  ,
	SH_DEFAULT_EECONFIG_8  ,
	SH_DEFAULT_EECONFIG_9  ,
	SH_DEFAULT_EECONFIG_10 ,
	SH_DEFAULT_EECONFIG_11 ,
	SH_DEFAULT_EECONFIG_12 ,
	SH_DEFAULT_EECONFIG_13 ,
	SH_DEFAULT_EECONFIG_14 ,
	SH_DEFAULT_EECONFIG_15 ,
	SH_DEFAULT_EECONFIG_16 ,
	SH_DEFAULT_EECONFIG_17 ,
	SH_DEFAULT_EECONFIG_18 ,
	SH_DEFAULT_EECONFIG_19 ,
	SH_DEFAULT_EECONFIG_20 ,
	SH_DEFAULT_EECONFIG_21 ,
	SH_DEFAULT_EECONFIG_22 ,
	SH_DEFAULT_EECONFIG_23 ,
	SH_DEFAULT_EECONFIG_24 ,
	SH_DEFAULT_EECONFIG_25 
};

//sh367309 内部参考电阻值
uint16_t REF_resVal = 9700;
uint8_t EE_ReadBuf[26];
uint16_t  CurrK = 1000;
int16_t  CurrO = 0;
uint16_t VoltK = 1000;
int16_t VoltO = 0;

static int16_t CurrOffset = 0;
static uint8_t MosCtrlIt = 0x00;  // bit0-3 不做任何操作 01 关闭放电MOS 02 打开放电MOS 04 关闭充电MOS 08 打开充电

static void SH_iicWriteRam(uint8_t ramAddr, uint8_t ramRegVal)
{
	uint8_t ret;
	uint8_t count = 5;
	//
	if ((ramAddr >= 0x40) && (ramAddr <= 0x70))
	{
		ret = SH309Twi_Write(I2C_SLAVE_ADDRESS7, ramAddr, ramRegVal);
		while(ret)
		{
			bsp_DelayUS(1000);
			ret = SH309Twi_Write(I2C_SLAVE_ADDRESS7, ramAddr, ramRegVal);
			count--;
			if(count == 0)
			{
				//PRINTF(0,"sh309 write regs timerout\n");
				break;
			}
		}
	}
}

static uint8_t SH_iicReadRam(uint8_t ramAddr, uint8_t readSize, uint8_t *pdataBuf)
{
	uint8_t ret = 0;
	uint8_t i = 0;
	uint8_t count = 5;
	//
	if ((ramAddr >= 0x40) && (ramAddr <= 0x70))
	{
		//先读到公共ram
		ret = SH309Twi_Read(I2C_SLAVE_ADDRESS7, ramAddr, readSize, CommonRam);

//		while(ret)
//		{
//			bsp_DelayUS(1000);
//			ret = SH309Twi_Read(I2C_SLAVE_ADDRESS7, ramAddr, readSize, CommonRam);
//			count--;
//			if(count == 0)
//			{
//				PRINT("sh309 read regs timerout\n");
//				break;
//			}
//		}

		if(ret == 0)
		{
			//crc is ok
			for(i = 0;i<readSize;i++)
			{
				*(pdataBuf +i) = CommonRam[i];
			}
		}else
		if(ret == 2)
		{
			//crc wrong
		}
	}
	return ret;
}


static uint16_t SH_CalcuTemp(uint16_t getdata)
{
	int i;
	
	uint8_t ucTempeMiddle = 81;
	uint16_t val;
	int temperature;
	float tempcalcu;

	REF_resVal = (680 + 5 * EE_ReadBuf[25]) * 10;
	val = 32768 - getdata;
	tempcalcu = (float)getdata / 1000 * (float)REF_resVal / 1000;
	tempcalcu = 1000 * tempcalcu / val;

	if (tempcalcu >= NTC103AT[0]) //Determine whether the excess temperature resistance range
	{
		temperature = 2731 - 400;
	}
	else if (tempcalcu <= NTC103AT[150])
	{
		temperature = 2731 + 1100;
	}
	else
	{
		i = ucTempeMiddle;
		if (tempcalcu > NTC103AT[i])
		{
			for (i = ucTempeMiddle - 1; i >= 0; i--)
			{
				if (tempcalcu <= NTC103AT[i]) //NTC103AT[i+1]<resis<NTC103AT[i]
				{
					break;
				}
			}
		}
		else
		{
			for (i = ucTempeMiddle + 1; i < 151; i++)
			{
				if (tempcalcu > NTC103AT[i]) //NTC103AT[i-1]<resis<NTC103AT[i]
				{
					break;
				}
			}
			i--;
		}
		ucTempeMiddle = i;

		temperature = (uint16_t)((ucTempeMiddle - 40) * 10 + (NTC103AT[i] - tempcalcu) * 10 / (NTC103AT[i] - NTC103AT[i + 1]) + 2731);
	}
	return (uint16_t)temperature;
}

void SH_iicWriteEEPROM(uint8_t EEAddr, uint8_t eeRegVal)
{
	uint8_t ret;
	uint8_t count = 5;
	if (EEAddr <= 0x18)
	{
		VPRO_CON_ON();
		bsp_DelayMS(20);
		ret = SH309Twi_Write(I2C_SLAVE_ADDRESS7, EEAddr, eeRegVal);

		while(ret)
		{
			bsp_DelayMS(1);
			ret = SH309Twi_Write(I2C_SLAVE_ADDRESS7, EEAddr, eeRegVal);
			count--;
			if(count == 0)
			{
				//PRINT("sh309 write EEPROM timerout\n");
				break;
			}
		}
		//delay1ms(35);
		bsp_DelayMS(35);
		VPRO_CON_OFF();
	}
}

uint8_t SH_iicReadEEPROM(uint8_t EEAddr, uint8_t readSize, uint8_t *pdataBuf)
{
	uint8_t ret = 0;
	uint8_t count = 5;
	if (EEAddr <= 0x18)
	{
		ret = SH309Twi_Read(I2C_SLAVE_ADDRESS7, EEAddr, readSize, pdataBuf);
		while(ret)
		{
			ret = SH309Twi_Read(I2C_SLAVE_ADDRESS7, EEAddr, readSize, pdataBuf);
			count--;
			if(count == 0)
			{
				PRINT("sh309 read EEPROM timerout\n");
				break;
			}
		}
	}
	return ret;
}

//软件RESET
void Sh_SWReset(void)
{
	//SH309Twi_Write(I2C_SLAVE_ADDRESS7, 0xEA, 0xC0);
	SH_ENABLE_SHIPMODE();
	bsp_DelayMS(200);
	SH_DISABLE_SHIPMODE();
	bsp_DelayMS(1000);
	Sh_CheckEEData();
	Sh_EnableCADC();
}


//周期型的轮询afe数据
void Sh_Process(void)
{
	static uint8_t idx = 0; //0-电压采集

	SH_iicReadRam(0x40+idx, 1, (uint8_t *)&(ShRamRegs)+idx);
	if(idx++ == sizeof(ShRamRegs))
	{
	    idx = 0;
	}

//	switch(sta)
//	{
//	  case 0:
//	    //前12字节
//	    SH_iicReadRam(0x40, 14, (uint8_t *)&(ShRamRegs));
//	    break;
//	  case 1:
//	    //电压数据1
//	    SH_iicReadRam(0x40+14, 16, (uint8_t *)&(ShRamRegs)+14);
//	    break;
//	  case 2:
//	    //电压数据2
//	    SH_iicReadRam(0x40+14+16, 16, (uint8_t *)&(ShRamRegs)+14+16);
//	    break;
//	  case 3:
//	    //尾部状态字节
//	    SH_iicReadRam(0x40+14+32, sizeof(ShRamRegs)-46, (uint8_t *)&(ShRamRegs)+14+32);
//	    break;
//	}
//	if(sta++ == 3)
//	{
//	    sta = 0;
//	}

	if ((ShRamRegs.conf & 0x08) == 0)
	{
	    Sh_EnableCADC();
	}

}

uint8_t Sh_CheckEEData(void)
{
	uint8_t ret = 0;
	uint8_t i;
	//uint8_t readbuf[32];

	ret = SH_iicReadEEPROM(0x00, 26, EE_ReadBuf);
	if (ret == 0)
	{
		if(CELL_NUMS > 13)
		{
			//自动添加芯片配置到15串以上
			EEP_RegMap[0] |= 0x0F;
		}

		for (i = 0; i < 25; i++)
		{
			if (EE_ReadBuf[i] != EEP_RegMap[i])
			{
				//写入EEP_RegMap 映射值
				SH_iicWriteEEPROM(i, EEP_RegMap[i]);
				ret = 1;
			}
		}
		if (ret == 1)
		{
			//临时注释掉，调试
			//#if(MCU_LIB_SELECT == 1)
			Sh_SWReset();
			bsp_DelayUS(300000);
			SH_iicReadEEPROM(0x00, 26, EE_ReadBuf);
			//#endif
		}
		//reset sh309
	}
	return ret;
}

void Sh_UpdateEEMapValue(uint8_t regval, uint8_t index)
{
	uint8_t ret;
	if (index < 25)
	{
		EEP_RegMap[index] = regval;
		ret = Sh_CheckEEData();

		if(ret)
		{
			Flash_Write(SH_EEPROM_ADDR, EEP_RegMap, 25);
		}
	}
}


void Sh_UpdateEEMapAll(void)
{
	Sh_CheckEEData();
}

//初始化 EEPROM 默认值
void Sh_EEMapValueInit(void)
{
	uint8_t i;
	//mcu 中默认的sheeprom值已经被初始化过，加载eeprom
	if (*(uint8_t *)(SH_EEPROM_ADDR + EE_START_ADDR + 1) != 0x00)
	{
		//memcpy(EEP_RegMap, (uint8_t *)(SH_EEPROM_ADDR + EE_START_ADDR), 25);
		for(i = 0;i <25;i++)
		{
			EEP_RegMap[i] = *(uint8_t *)(SH_EEPROM_ADDR + EE_START_ADDR+i);
		}
	}
}

//得到三个温度值
void Sh_GetTemp(uint16_t *RTx)
{
	uint16_t temp1, temp2, temp3;
	temp1 = (uint16_t)(ShRamRegs.temp1H << 8) + ShRamRegs.temp1L;
	temp2 = (uint16_t)(ShRamRegs.temp2H << 8) + ShRamRegs.temp2L;
	temp3 = (uint16_t)(ShRamRegs.temp3H << 8) + ShRamRegs.temp3L;

	*RTx = SH_CalcuTemp(temp1);
	*(RTx + 1) = SH_CalcuTemp(temp2);
	;
	*(RTx + 2) = SH_CalcuTemp(temp3);
	;
}

void Sh_GetCellsVolt(uint16_t *volts)
{
	uint8_t i;
	uint32_t cellTemp;

	for (i = 0; i < CELL_NUMS; i++)
	{
		cellTemp = ((uint32_t)ShRamRegs.cellx[2 * i] << 8) + ShRamRegs.cellx[2 * i + 1];
		*(volts + i) = (uint16_t)((cellTemp * 5) >> 5);
		//校准电压
		*(volts + i) = *(volts + i)*(float)VoltK/1000 + VoltO/1000;
		
	}
}

//返回值1-放电 0-充电，读取vadc 电流
uint8_t Sh_GetVadcCurrent(uint16_t *current)
{
	uint8_t ret = 0;
	*current = (uint16_t)((((uint16_t)ShRamRegs.curH) << 8 + ShRamRegs.curL) * 200 / 26837.0 / RSENSE);

	if ((ShRamRegs.curH) & 0x80 == 0x80)
	{
		ret = 1;
	}
	return ret;
}

//返回值1-放电 0-充电 读取cadc 电流,高精度电流，250ms中断Alarm输出
//可以累计电量积分
uint8_t Sh_GetCadcCurrent(uint32_t *current)
{
	uint8_t ret = 0;
	//uint8_t cadc[2];
	uint16_t tempvalue;

	//SH_iicReadRam(0x6e,2,cadc);
	//ShRamRegs.cadcdh = cadc[0];
	//ShRamRegs.cadcdl = cadc[1];
	tempvalue = (uint16_t)(ShRamRegs.cadcdh << 8) + ShRamRegs.cadcdl;

	if ((tempvalue & 0x8000) == 0x8000)
	{
		tempvalue = 0x10000 - tempvalue;
		//*current = (uint16_t)((float)(tempvalue + CurrOffset) * 200 / 21470.0 / RSENSE);
		*current = (uint32_t)((float)(tempvalue) * 200 / 21470.0 / RSENSE);
	}
	else
	{
		//*current = (uint16_t)((float)(tempvalue - CurrOffset) * 200 / 21470.0 / RSENSE);
		*current = (uint32_t)((float)(tempvalue) * 200 / 21470.0 / RSENSE);
	}
	//校准
	*current = *current * (float)CurrK/1000 + (float)CurrO/1000;

	if ((ShRamRegs.cadcdh & 0x80) == 0x80)
	{
		ret = 1;
	}
	return ret;
}

/**
 * @brief 校准电流参数
 * 
 * @param  
 */
void Sh_SetCurrCarlibation(uint16_t Kcurr,uint16_t Ocurr)
{
	CurrK = Kcurr;
	CurrO = (int16_t)Ocurr;
}

/**
 * @brief 校准电压参数
 * 
 * @param  
 */
void Sh_SetVoltCarlibation(uint16_t Kvolt,uint16_t Ovolt)
{
	VoltK = Kvolt;
	VoltO = (int16_t)Ovolt;
}

/**
 * @brief set sh309 run mode
 * 
 * @param mode =0x02 sleep mode ,mode = 0x01 idle mode 
 */
void Sh_SetRunMode(uint8_t mode)
{
	//uint8_t tempval;
	//SH_iicReadRam(0x40,1,&tempval);
	//uint8_t tempvalue;
	//SH_iicReadRam(0x40,1,&tempvalue);
	if (mode == 0x00)
	{
		SH_iicWriteRam(0x40, (ShRamRegs.conf & 0xFC));
	}
	else
	{
		SH_iicWriteRam(0x40, ShRamRegs.conf | mode);
	}
}

void Sh_EnableCADC(void)
{
	//uint8_t tempvalue;
	//SH_iicReadRam(0x40,1,&tempvalue);
	//SH_iicWriteRam(0x40,(ShRamRegs.conf | 0x08));
	uint8_t data;
	SH_iicReadRam(0x40, 1,&data);
	SH_iicWriteRam(0x40, 0x78);
}

//关闭充电MOS
void Sh_OpenChgMos(void)
{
	uint8_t data;
	SH_iicReadRam(0x40, 1,&data);
	SH_iicWriteRam(0x40, ShRamRegs.conf | 0x10);
	ShRamRegs.conf |=(0x10);

}

void Sh_ShutDChgMos(void)
{

	uint8_t data;
	SH_iicReadRam(0x40, 1,&data);
	SH_iicWriteRam(0x40, ShRamRegs.conf & (~0x10));
	ShRamRegs.conf &=(~0x10);

}

//关闭放电MOS
void Sh_ShutDDsgMos(void)
{
	uint8_t data;
	SH_iicReadRam(0x40, 1,&data);
	//if (ShRamRegs.conf & 0x20 == 0)
	{
		SH_iicWriteRam(0x40, ShRamRegs.conf & (~0x20));
		ShRamRegs.conf &=(~0x20);
	}
}

void Sh_OpenDsgMos(void)
{
	uint8_t data;
	//if (ShRamRegs.conf & 0x20)
	//{
		SH_iicReadRam(0x40, 1,&data);
		SH_iicWriteRam(0x40, ShRamRegs.conf | (0x20));
		ShRamRegs.conf |=(0x20);
	//}
}

void Sh_ShutDDsgMosIt(void){
	MosCtrlIt |= 0x01;
}

void Sh_OpenDsgMosIt(void){
	MosCtrlIt |= 0x02;
}

void Sh_ShutDChgMosIt(void)
{
	MosCtrlIt |= 0x04;
}

void Sh_OpenChgMosIt(void)
{
	MosCtrlIt |= 0x08;
}

//00-dsg_fet 关闭 chg_fet 关闭， 0x01 - dsg_fet 导通 chg_fet 关闭 ，02- dsg_fet 关闭 chg_fet 导通 03 - dsg_fet 导通 chg_fet 导通
uint8_t Sh_GetMOSSta(void)
{
	if(ShRamRegs.bstauts3 & 0x03)
	{
		return (ShRamRegs.bstauts3 & 0x03);
	}else
	{
		return 0;
	}
}

//得到平衡开关状态
uint32_t Sh_GetBalanceSta(){
	uint32_t balsta = 0;
	return balsta;
}

//控制平衡回路,开启平衡的通道 ch - 1，16
void Sh_SetBalance(uint8_t ch)
{
	
	if(ch<9)
	{
		SH_iicWriteRam(0x41, 0x01<<ch);
	}else
	if(ch <17)
	{
		SH_iicWriteRam(0x42, 0x01<<(ch-8));
	}
	
}

void Sh_DisableBalance()
{
	SH_iicWriteRam(0x41, 0);
	SH_iicWriteRam(0x42, 0);
}

void Sh_DisableCADC(void)
{
	uint8_t tempvalue;
	SH_iicReadRam(0x40, 1, &tempvalue);
	SH_iicWriteRam(0x40, (tempvalue & (~0x08)));
}

void Sh_WatchdogEnable(void)
{
	uint8_t tempvalue;
	SH_iicReadRam(0x40, 1, &tempvalue);
	SH_iicWriteRam(0x40, tempvalue | 0x04);
}

void Sh_WatchdogDisable(void)
{
	uint8_t tempvalue;
	SH_iicReadRam(0x40, 1, &tempvalue);
	SH_iicWriteRam(0x40, tempvalue & (~0x04));
}


//bstatus1、bstatus2、bstatus3 反映实时状态信息，硬件自动清除
//7~0  WDT PF SC OCC OCD2 OCD1 UV OV
uint8_t Sh_GetBStatus1(void)
{
	//uint8_t tempvalue;
	//SH_iicReadRam(0x43,1,&tempvalue);
	//ShRamRegs.bstatus1 = tempvalue;
	return ShRamRegs.bstatus1;
}

//3~0  OTD(放电高温保护) UTD(放电低温) OTC(充电高温保护) UTC(充电低温保护)
uint8_t Sh_GetBStatus2(void)
{
	//uint8_t tempvalue;
	//SH_iicReadRam(0x44,1,&tempvalue);
	//ShRamRegs.bstatus2 = tempvalue;
	return ShRamRegs.bstatus2;
}

//系统状态，7~0 7（充电状态/非充电）  6（放电/非放电） 4（EE写操作错误/EE写操作正确） 3（低电压禁止充电/未发送低电压禁止充电）
// 2（预充电MOS开启状态/预充电MOS关闭状态） 1（充电MOS开启/充电MOS关闭） 0（放电MOS开启/放电MOS关闭）
uint8_t Sh_GetBStatus3(void)
{
	//uint8_t tempvalue;
	//SH_iicReadRam(0x45,1,&tempvalue);
	//ShRamRegs.bstauts3 = tempvalue;
	return ShRamRegs.bstauts3;
}

//反映历史标志，需要MCU 清除
//bit[15~8] WDT_FLG,PF_FLG,SC_FLG,OCC_FLG,LOAD_FLG,OCD_FLG,UV_FLG,OV_FLG
//bit[7~0] RST_FLG,WAKE_FLG,CADC_FLG,VADC_FLG,OTD_FLG,UTD_FLG,OTC_FLG,UTC_FLG
uint16_t Sh_GetBFlag(void)
{

	return ((uint16_t)ShRamRegs.bflag1 << 8 + ShRamRegs.bflag2);
}

//判定充放电状态 0-静置 1-放电 2-充电 3-
uint16_t Sh_JudgeState(void)
{
	uint8_t i;
	uint16_t batsta = 0;
	if ((ShRamRegs.bstauts3 & 0x80) == 0x80)
	{
		//充电状态
		batsta |= 0x0002;
	}

	if ((ShRamRegs.bstauts3 & 0x40) == 0x40)
	{
		//放电状态
		batsta |= 0x0001;
	}
	
	//过充保护
	if(ShRamRegs.bstatus1 & 0x01)
	{
		batsta |= 0x0004;
	}

	//过放保护
	if(ShRamRegs.bstatus1 & 0x02)
	{
		batsta |= 0x0008;
	}

	//充电过流保护
	if(ShRamRegs.bstatus1 & 0x10)
	{
		batsta |= 0x0010;
	}

	//放电过流保护
	if(ShRamRegs.bstatus1 & 0x04)
	{
		batsta |= 0x0020;
	}

	//电池过热保护
	if(ShRamRegs.bstatus2 & 0x0a)
	{
		batsta |= 0x0040;
	}

	//电池低温保护
	if(ShRamRegs.bstatus2 & 0x05)
	{
		batsta |= 0x0080;
	}
	//电池组短路保护
	if(ShRamRegs.bstatus1 & 0x20)
	{
		batsta |= 0x0200;
	}

	/* MOS 失效判断 结合电流情况，在外部NiuLogic中判断
	//MOS损坏
	if ((ShRamRegs.bstauts3 & 0xc0)  && (ShRamRegs.bstauts3 & 0x03) != 0x00)
	{
		batsta |= 0x0800;
	}

	//充电MOS坏
	if((ShRamRegs.bstauts3 & 0x80) && (ShRamRegs.bstauts3 & 0x02) != 0x00)
	{
		batsta |= 0x1000;
	}

	//充电MOS坏
	if((ShRamRegs.bstauts3 & 0x40) && (ShRamRegs.bstauts3 & 0x01) != 0x00)
	{
		batsta |= 0x2000;
	}*/

	Sh_GetCellsVolt((uint16_t*)CommonRam2);

	for(i = 0;i<CELL_NUMS;i++)
	{
		if(*(uint16_t*)(CommonRam2+2*i) >6500)
		{
			batsta |= 0x4000;
		}
	}

	return batsta;
}

uint8_t Sh_GetTempProtectState(void)
{
	uint8_t sta = 0;
	//电池充电高温保护
	if(ShRamRegs.bstatus2 & 0x02)
	{
		sta |= 0x02;
	}

	//电池放电高温保护
	if(ShRamRegs.bstatus2 & 0x08)
	{
		sta |= 0x08;
	}

	//电池充电低温保护
	if(ShRamRegs.bstatus2 & 0x01)
	{
		sta |= 0x01;
	}

	//电池放电低温保护
	if(ShRamRegs.bstatus2 & 0x04)
	{
		sta |= 0x04;
	}

	//断线保护,pF
	if(ShRamRegs.bstatus1 & 0x40)
	{
		sta |= 0x10;
	}
	return sta;
}

void Sh_ClrFlag(uint8_t FlagCh, uint8_t flag)
{
	if (FlagCh == 1)
	{
		SH_iicWriteRam(0x70, ShRamRegs.bflag1 & (~flag));
	}
	else if (FlagCh == 1)
	{
		SH_iicWriteRam(0x70, ShRamRegs.bflag2 & (~flag));
	}
}

uint16_t Sh_GetRRef(void)
{
	return REF_resVal;
}

//输入寄存器配置的阈值，输出温度阈值 单位K *10
uint16_t Sh_ConvertTempTh2Value(uint8_t TempValue,uint16_t isLow)
{
	float temp;
	if(isLow)
	{
		temp = (float)TempValue/512 +0.5;
	}else
	{
		temp = (float)TempValue/512;
	}
	

	return CalcuTemp1(temp*REF_resVal/(1-temp)/1000);
}

//输入寄存器配置的温度值*10 K，输出寄存器配置值
uint8_t Sh_ConvertTemp2RegTh(uint16_t TempValue,uint16_t isLow)
{
	float rntc = 0;
	//从NTC温度曲线中，用温度，反查到电阻值
	rntc = CalaRntcFromTemp(TempValue);

	if(isLow)
	{
		return (uint8_t)(512*(rntc/(rntc+REF_resVal)-0.5));
	}else
	{
		return (uint8_t)(512*(rntc/(rntc+REF_resVal)));
	}
	
}

#endif
