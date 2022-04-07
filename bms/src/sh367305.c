/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/

#include "includes.h"

#if (AFE_CHIP_SELECT == 3)

#define RSENSE AFE_CUR_SAMPLE_RES

#define AFE_BKEN_ON()                    \
	do                                   \
	{                                    \
	  MCU_GPIO_SetBit(TN_BK_EN_PORT, TN_BK_EN_PIN); \
	} while (0);

#define AFE_BKEN_OFF()                     \
	do                                     \
	{                                      \
		MCU_GPIO_ClrBit(TN_BK_EN_PORT, TN_BK_EN_PIN); \
	} while (0);

//sh367305 ram 寄存器表
typedef struct
{
	uint8_t Ts1H;
	uint8_t Ts1L;
	uint8_t Ts2H;
	uint8_t Ts2L;
	uint8_t Temp1H;
	uint8_t Temp1L;
	uint8_t Temp2H;
	uint8_t Temp2L;
	uint8_t currState;//1 放电 2充电
	uint32_t curr;
} SH305_Regs_t;

static void SH_iicWriteRam(uint8_t ramAddr, uint8_t ramRegVal);
static uint16_t SH_iicReadReg(uint8_t ramAddr);
static uint16_t SH305_CalcuTemp(uint16_t getdata);

SH305_Regs_t SH305_Regs;

uint16_t  CurrK = 1000;
int16_t  CurrO = 0;
uint16_t VoltK = 1000;
int16_t VoltO = 0;

static uint8_t SH305_iicWriteReg(uint8_t ramAddr, uint8_t ramRegVal)
{
	uint8_t ret;
	uint8_t count = 10;

	ret = SH305Twi_Write(SH305_I2C_SLAVE_ADDR, ramAddr, ramRegVal);
	while(ret)
	{
		bsp_DelayUS(2000);
		ret = SH305Twi_Write(SH305_I2C_SLAVE_ADDR, ramAddr, ramRegVal);
		count --;
		if(count == 0)
		{
			// AFE_BKEN_ON();
			// bsp_DelayUS(35000);
			// AFE_BKEN_OFF();
			// bsp_DelayUS(50000);	
			SEGGER_RTT_printf(0,"sh305 write twi failed\n");
			break;
		}
	}
	return ret;

}

static uint16_t SH_iicReadReg(uint8_t ramAddr)
{
	uint8_t ret;
	uint8_t i = 0;
	uint8_t count = 10;
	uint8_t pdataBuf[2];
	
	ret = SH305Twi_Read(SH305_I2C_SLAVE_ADDR, ramAddr, 2, pdataBuf);
	while(ret)
	{
		bsp_DelayUS(1000);
		ret = SH305Twi_Read(SH305_I2C_SLAVE_ADDR, ramAddr, 2, pdataBuf);
		count --;
		if(count == 0)
		{
			// AFE_BKEN_ON();
			// bsp_DelayUS(35000);
			// AFE_BKEN_OFF();
			// bsp_DelayUS(50000);	
			// SEGGER_RTT_printf(0,"sh305 write twi failed\n");
			break;
		}
	}

	if(ret == 0)
	{
		return ((uint16_t)pdataBuf[0]<<8) + pdataBuf[1];
	}else
	if(ret == 1)
	{
		SEGGER_RTT_printf(0,"sh305 read twi timeout\n");	
	}
	else
	if(ret == 2)
	{
		//crc wrong
		SEGGER_RTT_printf(0,"sh305 read twi crc error\n");		
	}
	return 0xFFFF;
	
	
}


static uint16_t SH305_CalcuTemp(uint16_t getdata)
{
	int16_t i;
	float tempcalcu, temperature;
	uint8_t ucTempeMiddle = 75;
	
	tempcalcu = (float)getdata * 10 / (4096 - getdata);

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

void Sh305_DeviceInit(void)
{
	uint8_t temval;
	uint16_t readback;
	Sh_SetRunMode(0);
	//负载检测，OV_EN,SC_EN,WDT_EN
	SH305_iicWriteReg(SH305_SCONF1_ADDR,0x1C);
	readback = SH_iicReadReg(SH305_SCONF1_ADDR);
	SEGGER_RTT_printf(0,"SH305_SCONF12_ADDR->%4X\n",readback);
	//VADC EN,VADC_C
	SH305_iicWriteReg(SH305_SCONF3_ADDR,0xF9);
	readback = SH_iicReadReg(SH305_SCONF3_ADDR);
	SEGGER_RTT_printf(0,"SH305_SCONF34_ADDR->%4X\n",readback);

	//SCD 保护电压-200mv,400A、保护延时-100uS，CADC 采集范围选择100mV
	SH305_iicWriteReg(SH305_SCONF6_ADDR,0x85);
	readback = SH_iicReadReg(SH305_SCONF6_ADDR);
	SEGGER_RTT_printf(0,"SH305_SCONF67_ADDR->%4X\n",readback);
	//OVT 保护延迟设置 充电状态检测
	SH305_iicWriteReg(SH305_SCONF7_ADDR,0x70);
	readback = SH_iicReadReg(SH305_SCONF7_ADDR);
	SEGGER_RTT_printf(0,"SH305_SCONF78_ADDR->%4X\n",readback);
	//OVD 过充保护电压阈 设置 
	SH305_iicWriteReg(SH305_SCONF8_ADDR,(uint8_t)((uint16_t)(OC_VLT_P/5.86)>>8));
	SH305_iicWriteReg(SH305_SCONF9_ADDR,(uint8_t)((uint16_t)(OC_VLT_P/5.86)));
	readback = SH_iicReadReg(SH305_SCONF8_ADDR);
	SEGGER_RTT_printf(0,"SH305_SCONF89_ADDR->%4X\n",readback);

	//SH305_iicWriteReg(SH305_INT_EN_ADDR,0x7E);
	//readback = SH_iicReadReg(SH305_INT_EN_ADDR);
	//SEGGER_RTT_printf(0,"SH305_INT_EN_ADDR->%04X\n",readback);

	Sh_ClrFlag();

}

//软件RESET
void Sh_SWReset(void)
{

}


//周期型的轮询afe数据
void Sh_Process(void)
{
	uint16_t flag12;

	flag12 = SH_iicReadReg(SH305_FLAG1_ADDR);
	if((flag12 & 0x04) != 0 )
	{
		if(flag12 != 0xffff)
		{
			SEGGER_RTT_printf(0,"Sh305_DeviceInited\n");
			Sh305_DeviceInit();
		}		
	}

}

//得到2个温度值
void Sh_GetTemp(uint16_t *RTx)
{
	uint16_t temp1, temp2;
	temp1 = (uint16_t)(SH305_Regs.Temp1H << 8) + SH305_Regs.Temp1L;
	temp2 = (uint16_t)(SH305_Regs.Temp2H << 8) + SH305_Regs.Temp2L;
	*RTx = SH305_CalcuTemp(temp1);
	*(RTx + 1) = SH305_CalcuTemp(temp2);	
	*(RTx + 2) = 2731+250;
}

void Sh_GetCellsVolt(uint16_t *volts)
{
	uint8_t i;
	uint16_t flag12 = 0;
	uint16_t cellTemp;
	uint16_t TempVal;
	uint16_t tempVolt = 0;
	uint8_t count=100;
	


	SH305_iicWriteReg(SH305_INT_EN_ADDR,0x04);
	SH305_iicWriteReg(SH305_SCONF3_ADDR,0x18);

	do{
		flag12 = SH_iicReadReg(SH305_FLAG1_ADDR);
		if(count-- == 0)
		{
			return;
		}
	}while((flag12 & 0x0001)==0);	

	for (i = 0; i < CELL_NUMS; i++)
	{
		cellTemp = SH_iicReadReg(SH305_CELLX_START_ADDR+2*i);
		if(cellTemp == 0xffff)
		{
			return;
		}
		tempVolt = (uint16_t)(((float)cellTemp * 6)/4.096);
		if(tempVolt < 5000)
		{
			*(volts + i) = tempVolt;
			//校准电压
			*(volts + i) = *(volts + i)*(float)VoltK/1000 + VoltO/1000;	
		}
			
	}	

	//温探数据
	TempVal = SH_iicReadReg(SH305_TS_START_ADDR);
	if(TempVal == 0xffff)
	{
		return;
	}
	SH305_Regs.Temp1H = (uint8_t)(TempVal>>8);
	SH305_Regs.Temp1L= (uint8_t)TempVal;

	TempVal = SH_iicReadReg(SH305_TS_START_ADDR+2);
	if(TempVal == 0xffff)
	{
		return;
	}
	SH305_Regs.Temp2H = (uint8_t)(TempVal>>8);
	SH305_Regs.Temp2L= (uint8_t)TempVal;

}

//返回值1-放电 0-充电 读取cadc 电流,高精度电流，250ms中断Alarm输出
//可以累计电量积分
uint8_t Sh_GetCadcCurrent(uint32_t *current)
{
	uint8_t ret = 0;
	if(SH305_Regs.currState ==1)
	{
		ret = 1;		
	}
	*current = SH305_Regs.curr;
	return ret;
}


//返回值1-放电 0-充电 读取cadc 电流,高精度电流，250ms中断Alarm输出
//可以累计电量积分
static uint8_t Sh_GetCadcCurrent1(uint32_t *current)
{
	uint8_t ret = 0;
	uint16_t flag12 = 0;
	//uint8_t cadc[2];
	uint16_t tempvalue;
	uint16_t readback;
	uint8_t count = 100;

	//SH305_iicWriteReg(SH305_SCONF6_ADDR,0x85);
	readback = SH_iicReadReg(SH305_SCONF6_ADDR);
	ret = SH305_iicWriteReg(SH305_SCONF6_ADDR,(readback>>8)|0x80);
	if(ret != 0)
	{
		return 2;
	}
	//SH305_iicWriteReg(SH305_SCONF6_ADDR,0x80);
	ret = SH305_iicWriteReg(SH305_SCONF3_ADDR,0xe0);

	if(ret != 0)
	{
		return 2;
	}
	//获取sh305寄存器数据
	do{
		flag12 = SH_iicReadReg(SH305_FLAG1_ADDR);
		if(count-- == 0)
		{
			ret = 2;
			return ret;
		}

	}while((flag12 & 0x0002)==0);

	tempvalue = SH_iicReadReg(SH305_CUR_START_ADDR);
	if(tempvalue == 0xffff)
	{
		return 2;
	}

	//bit12 为1 放电，为0充电
	if ((tempvalue & 0x1000) == 0x1000)
	{
		tempvalue = tempvalue | 0xe000;
		*current = (uint32_t)((0x10000-tempvalue) * (1000 / 32768.0 / RSENSE));
		ret = 1;
	}
	else
	{
		*current = (uint32_t)(tempvalue * (1000 / 32768.0 / RSENSE));
	}
	//校准
	*current = *current * (float)CurrK/1000 + (float)CurrO/1000;
	if(ret == 1)
	{
		
		SH305_Regs.currState = 1;
	}else
	{
		SH305_Regs.currState = 2;
	}
	if(*current <200)
	{
		SH305_Regs.currState = 0;
	}

	SH305_Regs.curr = *current;
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
	uint16_t tempval;
	uint8_t count = 5;
	
	if (mode == 0x00)
	{
		do{
			AFE_BKEN_ON();
			bsp_DelayUS(35000);
			AFE_BKEN_OFF();
			bsp_DelayUS(50000);			

			if(GET_AFE_ON_STAT() == 0)
			{
				//正常模式
				SEGGER_RTT_printf(0,"sh305 switch on\n");
				//tempval &= ~0x2000;		
				//SH305_iicWriteReg(SH305_SCONF1_ADDR, tempval);
				break;	
			}else
			{
				SEGGER_RTT_printf(0,"sh305 switch on failed\n");
			}
		}while(count--);
		
	}
	else
	{
		tempval = SH_iicReadReg(SH305_SCONF1_ADDR);
		//低功耗模式
	    SH305_iicWriteReg(SH305_SCONF10_ADDR,0x33);
		SH305_iicWriteReg(SH305_SCONF1_ADDR, (uint8_t)(tempval>>8)|0x20);
	}
}

void Sh_EnableCADC(void)
{
	// uint16_t tempval;

	// tempval = SH_iicReadReg(SH305_SCONF3_ADDR);

	// SH305_iicWriteReg(SH305_SCONF3_ADDR,(uint8_t)(tempval>>8)|0xF8);
}

//关闭充电MOS
void Sh_OpenChgMos(void)
{

	uint16_t tempval;

	tempval = SH_iicReadReg(SH305_SCONF2_ADDR);
	SH305_iicWriteReg(SH305_SCONF2_ADDR,(uint8_t)(tempval>>8)|0x01);
}

void Sh_ShutDChgMos(void)
{
	uint16_t tempval;

	tempval = SH_iicReadReg(SH305_SCONF2_ADDR);
	tempval &= ~0x0100;
	SH305_iicWriteReg(SH305_SCONF2_ADDR,(uint8_t)(tempval>>8));

}

//关闭放电MOS
void Sh_ShutDDsgMos(void)
{
	uint16_t tempval;

	tempval = SH_iicReadReg(SH305_SCONF2_ADDR);
	tempval &= ~0x0200;
	SH305_iicWriteReg(SH305_SCONF2_ADDR,(uint8_t)(tempval>>8));
}

void Sh_OpenDsgMos(void)
{
	uint16_t tempval;

	tempval = SH_iicReadReg(SH305_SCONF2_ADDR);
	SH305_iicWriteReg(SH305_SCONF2_ADDR,(uint8_t)(tempval>>8)|0x02);
}

//00-dsg_fet 关闭 chg_fet 关闭， 0x01 - dsg_fet 导通 chg_fet 关闭 ，02- dsg_fet 关闭 chg_fet 导通 03 - dsg_fet 导通 chg_fet 导通
uint8_t Sh_GetMOSSta(void)
{
	return 0;
}

//得到平衡开关状态
uint32_t Sh_GetBalanceSta(){
	uint32_t balsta = 0;
	return balsta;
}

//控制平衡回路,开启平衡的通道 ch - 0开始
void Sh_SetBalance(uint8_t ch)
{
	
	if(ch<6)
	{
		//SH_iicWriteRam(0x41, 0x01<<ch);
		SH305_iicWriteReg(SH305_SCONF5_ADDR,0x01<<ch);
	}else
	if(ch <11)
	{
		SH305_iicWriteReg(SH305_SCONF4_ADDR,0x01<<(ch-6));
	}
	
}

void Sh_DisableBalance(){
	SH305_iicWriteReg(SH305_SCONF5_ADDR,0);
	SH305_iicWriteReg(SH305_SCONF4_ADDR,0);
}


//判定充放电状态 0-静置 1-放电 2-充电 3-
uint16_t Sh_JudgeState(void)
{
	uint8_t i;
	uint8_t ret;
	uint32_t c;
	uint16_t batsta = 0;
	uint16_t flag12 = 0;

	ret = Sh_GetCadcCurrent1(&c);
	#if (TYPROJECT_ID != 3)
	if(c > 800)
	#else
	if(c > 200)
	#endif
	{
		if(ret == 1)
		{
			//放电状态
			batsta |= 0x0001;
		}else
		if(ret == 0)
		{
			//充电状态
			batsta |= 0x0002;
		}
	}

	/*flag12 = SH_iicReadReg(SH305_SCONF3_ADDR);

	if((flag12 & 0x8000) ==0)
	{
		SH305_iicWriteReg(SH305_SCONF3_ADDR,(flag12 | 0x8000));
		bsp_DelayUS(100);
	}
	//得到充放电标志
	flag12 = SH_iicReadReg(SH305_BSTATUS_ADDR);

	if (flag12 & 0x0400)
	{
		//充电状态
		batsta |= 0x0002;
	}

	if (flag12 & 0x0800)
	{
		//放电状态
		batsta |= 0x0001;
	}*/


	flag12 = SH_iicReadReg(SH305_FLAG1_ADDR);
	//过充保护
	if(flag12 & 0x0400)
	{
		batsta |= 0x0004;
	}

	//过放保护

	//充电过流保护

	//放电过流保护

	//电池过热保护


	//电池低温保护

	//电池组短路保护
	if(flag12 & 0x0800)
	{
		batsta |= 0x0200;
	}

	/*
	Sh_GetCellsVolt((uint16_t*)CommonRam2);

	for(i = 0;i<CELL_NUMS;i++)
	{
		if(*(uint16_t*)(CommonRam2+2*i) >6500)
		{
			batsta |= 0x4000;
		}
	}*/

	return batsta;
}

uint8_t Sh_GetTempProtectState(void)
{
	//没有温度保护标志
}

//清除SC/OV/TWI/WDT保护标志
void Sh_ClrFlag()
{
	uint16_t tempval;

	tempval = SH_iicReadReg(SH305_SCONF1_ADDR);
	SH305_iicWriteReg(SH305_SCONF1_ADDR,(tempval|0x8000)>>8);
	tempval &= ~0x8000;
	SH305_iicWriteReg(SH305_SCONF1_ADDR,tempval>>8);
}


//输入寄存器配置的阈值，输出温度阈值 单位K *10
uint16_t Sh_ConvertTempTh2Value(uint8_t TempValue,uint16_t isLow)
{

}

//输入寄存器配置的温度值*10 K，输出寄存器配置值
uint8_t Sh_ConvertTemp2RegTh(uint16_t TempValue,uint16_t isLow)
{
	
}

#endif
