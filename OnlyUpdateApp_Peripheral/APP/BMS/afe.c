/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"



#define BUFFER_SIZE ((uint8_t)8)

//static void DMA_Config(void);
static uint16_t FindVolt(uint16_t *volt, uint8_t maxormin, uint8_t *pos);
static uint32_t CalcSumVolt(uint16_t *volt);

uint16_t AdcBuffer[BUFFER_SIZE]; // = { 0, 0, 0, 0, 0, 0, 0, 0, 0,0 };
uint16_t ShTempNTC[3];
uint32_t CadcCurrent;
uint8_t  AfeInitSample = 1;
uint8_t  b_Balance = 0;

AFEInfo_t afeInfo;



void AFE_ADCInit(void)
{
//	ADC_PortInit();
//	ADC_Config();
}

void AFE_Init(void)
{
#if (USE_SIM_AFEDATA == 1)

#else
    #if (AFE_CHIP_SELECT == 1)
	if(SYS_GetLastResetSta() != RST_STATUS_GPWSM)
	 {
        //中颖sh309前端芯片初始化
	    SH_ENABLE_SHIPMODE();
	    bsp_DelayMS(200);
	    SH_DISABLE_SHIPMODE();
	    bsp_DelayMS(500);
	    Sh_CheckEEData();
	    Sh_EnableCADC();
	 }
    #elif(AFE_CHIP_SELECT == 2)
        //集澈dvc前端芯片初始化
        DVC10xx_DeviceInit();
    #elif (AFE_CHIP_SELECT == 3)
        Sh305_DeviceInit();

    #endif
#endif

	//ADC_PortInit();
	//ADC_Config();
	bsp_StartAutoTimer(TMR_AFE, 500);
}

void Afe_SetInitSample(void){
	AfeInitSample = 1;
}

void AFE_Process(void)
{
	uint8_t State_RT2;
	uint16_t State_RT;
//	static uint8_t sCount = 0;
	static uint8_t sCount2 = 0;
//	static uint8_t sCount3 = 0;

#if(USE_SIM_AFEDATA == 1)
	if(bsp_CheckTimer(TMR_AFE))
	{
	    //电池状态数据模拟
	    afeInfo.State_RT = 0x00;
	    //电压数据模拟
	    for (uint8_t i = 0; i < CELL_NUMS; i++)
	    {
	        //电压数据加随机跳动
	        afeInfo.CellVolt[i] = 3600;

	    }
	    afeInfo.ChgCurrent = 0;
	    afeInfo.DsgCurrent = 0;

	}

#elif(AFE_CHIP_SELECT ==1)
	//500 MS loop
	if (bsp_CheckTimer(TMR_AFE)|| (AfeInitSample == 1))
	{
		//初始化采集
		AfeInitSample = 0;
		//采集sh309 寄存器信息
		//Sh_Process();

		State_RT2 = Sh_GetTempProtectState();
		afeInfo.AFE_State_RT2 = State_RT2;
		//sh309 根据充放电状态 获取

		//硬件AFE状态
		State_RT = Sh_JudgeState();	
		afeInfo.AFE_State_RT = State_RT;

		//充放电状态完全由硬件前端状态决定
		afeInfo.State_RT &= ~0x03;
		afeInfo.State_RT |= (afeInfo.AFE_State_RT & 0x03);

		//获取硬件短路保护标志
		afeInfo.State_RT &= ~0x200;
		afeInfo.State_RT |= (afeInfo.AFE_State_RT & 0x200);

		if(afeInfo.AFE_State_RT & 0x4000)
		{
			sCount2++;
			if(sCount2>10)
			{
				//PRINT("event->sh309 reseted\n");
				//NVIC_SystemReset();
			}
			//有错误数据暂时返回
			return;			
		}else
		{
			sCount2 = 0;
		}

		if (afeInfo.State_RT & 0x0003) //充放电状态
		{
			if (Sh_GetCadcCurrent(&CadcCurrent))
			{
				afeInfo.DsgCurrent = CadcCurrent;
				afeInfo.ChgCurrent = 0;

			}
			else
			{
				afeInfo.ChgCurrent = CadcCurrent;
				afeInfo.DsgCurrent = 0;
			}
			
			if((afeInfo.AFE_State_RT & 0x3000) != 0)
			{
				afeInfo.State_RT |= 0x0800;
			}else
			{
				afeInfo.State_RT &= (~0x0800);
			}
		}
		else
		{
			afeInfo.ChgCurrent = 0;
			afeInfo.DsgCurrent = 0;
			//静置状态清除MOS检测
			afeInfo.State_RT &= (~0x1803);
		}

		//if(b_Balance == 0)
		Sh_GetCellsVolt(afeInfo.CellVolt);
		//get sh309 温度
		Sh_GetTemp(ShTempNTC);
		afeInfo.ShTemp[0] = ShTempNTC[0];
		afeInfo.ShTemp[1] = ShTempNTC[1];
		afeInfo.ShTemp[2] = ShTempNTC[2];

		//calc sum voltage and average voltage
		afeInfo.SumBatteryPackVolt = CalcSumVolt(afeInfo.CellVolt);
		//最高、最低、压差、最高温、最低温、电芯温差、位置信息获取
		afeInfo.CellVmax = FindVolt(afeInfo.CellVolt, 1, &afeInfo.CellVmaxPos);
		afeInfo.CellVmin = FindVolt(afeInfo.CellVolt, 0, &afeInfo.CellVminPos);
		afeInfo.CellVdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//压差过大状态判定
		if(afeInfo.CellVdiff > 300)
		{
			afeInfo.State_RT |= 0x100;
		}else
		{
			afeInfo.State_RT &= (~0x100);
		}

		if(afeInfo.ShTemp[0]>afeInfo.ShTemp[1])
		{
			afeInfo.CellTmax = afeInfo.ShTemp[0];
			afeInfo.CellTmin = afeInfo.ShTemp[1];
			afeInfo.CellTmaxPos = 0;
			afeInfo.CellTminPos = 1;
		}else
		{
			afeInfo.CellTmax = afeInfo.ShTemp[1];
			afeInfo.CellTmin = afeInfo.ShTemp[0];
			afeInfo.CellTmaxPos = 1;
			afeInfo.CellTminPos = 0;
		}
		
		afeInfo.CellTdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//get mcu 采集信息
		//ADC_GetMcuAdcInfo(AdcBuffer,&afeInfo.MosTemp,&afeInfo.BalanceTemp,&afeInfo.PreDsgCurrent);
#if ((PROJECT_ID == 3) || (PROJECT_ID == 4))
		//没有硬件检测
		afeInfo.PreDsgCurrent = 0;
#endif
	}
#elif(AFE_CHIP_SELECT == 2)
	Dvc10xx_AFE_Process();

	if (bsp_CheckTimer(TMR_AFE)|| (AfeInitSample == 1))
	{
		//初始化采集
		AfeInitSample = 0;
		
		afeInfo.State_RT2 = Dvc10xx_AFE_GetTempProtectState();
		//sh309 根据充放电状态 获取
		afeInfo.State_RT = Dvc10xx_AFE_JudgeState();
		if (afeInfo.State_RT)
		{
			if (Dvc10xx_AFE_GetCadcCurrent(&CadcCurrent))
			{
				afeInfo.DsgCurrent = CadcCurrent;
				afeInfo.ChgCurrent = 0;

				//放到状态，检测到实际的驱动信号为关状态
				if(GET_DMOS_FB() == 1)
				{
					//放到MOS失效
					afeInfo.State_RT |= 0x2000;
				}else
				{
					afeInfo.State_RT &= (~0x2000);
				}
			}
			else
			{
				afeInfo.ChgCurrent = CadcCurrent;
				afeInfo.DsgCurrent = 0;

				if(GET_CMOS_FB() == 1)
				{
					//放到MOS失效
					afeInfo.State_RT |= 0x1000;
				}else
				{
					afeInfo.State_RT &= (~0x1000);
				}

			}
			
			if((afeInfo.State_RT & 0x3000) != 0)
			{
				afeInfo.State_RT |= 0x0800;
			}else
			{
				afeInfo.State_RT &= (~0x0800);
			}
		}
		else
		{
			afeInfo.ChgCurrent = 0;
			afeInfo.DsgCurrent = 0;
		}

		//发生断线保护后，CADC被关闭，读数不正确，不读取电压，温度、电流等数据，等待人工复位
		if(afeInfo.State_RT2 & 0x10)
		{
			return ;
		}
		if(b_Balance == 0)
		    Dvc10xx_AFE_GetCellsVolt(afeInfo.CellVolt);
		//get sh309 温度
		Dvc10xx_AFE_GetTemp(ShTempNTC);
		afeInfo.ShTemp[0] = ShTempNTC[0];
		afeInfo.ShTemp[1] = ShTempNTC[1];
		afeInfo.ShTemp[2] = ShTempNTC[2];

		//calc sum voltage and average voltage
		afeInfo.SumBatteryPackVolt = CalcSumVolt(afeInfo.CellVolt);
		//最高、最低、压差、最高温、最低温、电芯温差、位置信息获取
		afeInfo.CellVmax = FindVolt(afeInfo.CellVolt, 1, &afeInfo.CellVmaxPos);
		afeInfo.CellVmin = FindVolt(afeInfo.CellVolt, 0, &afeInfo.CellVminPos);
		afeInfo.CellVdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//压差过大状态判定
		if(afeInfo.CellVdiff > 300)
		{
			afeInfo.State_RT |= 0x100;
		}else
		{
			afeInfo.State_RT &= (~0x100);
		}

		if(afeInfo.ShTemp[0]>afeInfo.ShTemp[1])
		{
			afeInfo.CellTmax = afeInfo.ShTemp[0];
			afeInfo.CellTmin = afeInfo.ShTemp[1];
			afeInfo.CellTmaxPos = 0;
			afeInfo.CellTminPos = 1;
		}else
		{
			afeInfo.CellTmax = afeInfo.ShTemp[1];
			afeInfo.CellTmin = afeInfo.ShTemp[0];
			afeInfo.CellTmaxPos = 1;
			afeInfo.CellTminPos = 0;
		}
		
		afeInfo.CellTdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//get mcu 采集信息
		ADC_GetMcuAdcInfo(AdcBuffer,&afeInfo.MosTemp,&afeInfo.BalanceTemp,&afeInfo.PreDsgCurrent);
	}
#elif(AFE_CHIP_SELECT == 3)
	//500 MS loop
	if (bsp_CheckTimer(TMR_AFE)|| (AfeInitSample == 1))
	{
		//初始化采集
		AfeInitSample = 0;

		Sh_Process();
		//State_RT2 = Sh_GetTempProtectState();
		//afeInfo.AFE_State_RT2 = State_RT2;
		//sh309 根据充放电状态 获取
		//硬件AFE状态
		State_RT = Sh_JudgeState();	
		afeInfo.AFE_State_RT = State_RT;

		//充放电状态完全由硬件前端状态决定
		afeInfo.State_RT &= ~0x03;
		afeInfo.State_RT |= (afeInfo.AFE_State_RT & 0x03);

		//获取硬件短路保护标志
		afeInfo.State_RT &= ~0x200;
		afeInfo.State_RT |= (afeInfo.AFE_State_RT & 0x200);

		if(afeInfo.AFE_State_RT & 0x4000)
		{
			sCount2++;
			if(sCount2>10)
			{
				PRINT("event->sh309 reseted\n");
				NVIC_SystemReset();
			}
			//有错误数据暂时返回
			return;			
		}else
		{
			sCount2 = 0;
		}

		if (afeInfo.State_RT & 0x0003) //充放电状态
		{
			if (Sh_GetCadcCurrent(&CadcCurrent))
			{
				afeInfo.DsgCurrent = CadcCurrent;
				afeInfo.ChgCurrent = 0;

				//放到状态，检测到实际的驱动信号为关状态
				if(GET_DMOS_FB() == 1)
				{
					//连续检测5次
					if(afeInfo.DsgCurrent>500)
					{	
						sCount++;
						if(sCount>10)
						{
							//放电MOS失效
							afeInfo.State_RT |= 0x2000;
							sCount = 0;
						}
						
					}					
				}else
				{
					afeInfo.State_RT &= (~0x2000);					
				}
			}
			else
			{
				afeInfo.ChgCurrent = CadcCurrent;
				afeInfo.DsgCurrent = 0;
				if(GET_CMOS_FB() == 1)
				{
					if(afeInfo.ChgCurrent>500)
					{
						sCount++;
						if(sCount>10)
						{
							//充电MOS失效
							afeInfo.State_RT |= 0x1000;
							sCount = 0;
						}						
					}					
				}else
				{
					afeInfo.State_RT &= (~0x1000);
				}
			}
			
			if((afeInfo.AFE_State_RT & 0x3000) != 0)
			{
				afeInfo.State_RT |= 0x0800;
			}else
			{
				afeInfo.State_RT &= (~0x0800);
			}
		}
		else
		{
			//滤波，连续5次判断到状态为0，才设置电流为0
			sCount3++;
			if(sCount3 > 5)
			{
				afeInfo.ChgCurrent = 0;
				afeInfo.DsgCurrent = 0;
				//静置状态清除MOS检测
				afeInfo.State_RT &= (~0x1803);
				sCount3 = 0;
			}

		}

		if(b_Balance == 0)
		    Sh_GetCellsVolt(afeInfo.CellVolt);
		//get sh309 温度
		Sh_GetTemp(ShTempNTC);
		afeInfo.ShTemp[0] = ShTempNTC[0];
		afeInfo.ShTemp[1] = ShTempNTC[1];
		afeInfo.ShTemp[2] = ShTempNTC[2];

		//calc sum voltage and average voltage
		afeInfo.SumBatteryPackVolt = CalcSumVolt(afeInfo.CellVolt);
		//最高、最低、压差、最高温、最低温、电芯温差、位置信息获取
		afeInfo.CellVmax = FindVolt(afeInfo.CellVolt, 1, &afeInfo.CellVmaxPos);
		afeInfo.CellVmin = FindVolt(afeInfo.CellVolt, 0, &afeInfo.CellVminPos);
		afeInfo.CellVdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//压差过大状态判定
		if(afeInfo.CellVdiff > 300)
		{
			afeInfo.State_RT |= 0x100;
		}else
		{
			afeInfo.State_RT &= (~0x100);
		}

		if(afeInfo.ShTemp[0]>afeInfo.ShTemp[1])
		{
			afeInfo.CellTmax = afeInfo.ShTemp[0];
			afeInfo.CellTmin = afeInfo.ShTemp[1];
			afeInfo.CellTmaxPos = 0;
			afeInfo.CellTminPos = 1;
		}else
		{
			afeInfo.CellTmax = afeInfo.ShTemp[1];
			afeInfo.CellTmin = afeInfo.ShTemp[0];
			afeInfo.CellTmaxPos = 1;
			afeInfo.CellTminPos = 0;
		}
		
		afeInfo.CellTdiff = afeInfo.CellVmax - afeInfo.CellVmin;

		//get mcu 采集信息
		ADC_GetMcuAdcInfo(AdcBuffer,&afeInfo.MosTemp,&afeInfo.BalanceTemp,&afeInfo.PreDsgCurrent);
		//没有硬件检测
		afeInfo.PreDsgCurrent = 0;
				
	}


#endif

}

void AFE_DisableVSample()
{
	b_Balance = 1;
}

void AFE_EnableVSample()
{
	b_Balance = 0;
}

static uint16_t FindVolt(uint16_t *volt, uint8_t maxormin, uint8_t *pos)
{
	uint8_t i;
	uint16_t temp = *(volt);
	*pos = 0;
	for (i = 0; i < CELL_NUMS; i++)
	{
		if (maxormin == 1)
		{
			if (*(volt + i) > temp)
			{
				temp = *(volt + i);
				*pos = i;
			}
		}
		else if (maxormin == 0)
		{
			if (*(volt + i) < temp)
			{
				temp = *(volt + i);
				*pos = i;
			}
		}
	}

	return temp;
}

static uint32_t CalcSumVolt(uint16_t *volt)
{
	uint8_t i;
	uint32_t sumv =0;

	for (i = 0; i < CELL_NUMS; i++)
	{
		sumv += *(volt + i);
	}
	return sumv;
}
