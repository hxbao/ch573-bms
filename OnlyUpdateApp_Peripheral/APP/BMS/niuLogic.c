/******************************************************************************/
/** \file niuLogic.c
 **
 ** niu F serial bms logic
 **
 **   - 2021-9 hxbao    First Version
 **
 ******************************************************************************/

#include "includes.h"
#if(PROJECT_ID == 1)

typedef struct
{
	uint16_t OV;   //过压保护阈值  mV 5mv的整数倍
	uint16_t OVR;  //过压保护恢复阈值 mV 5mv的整数倍
	uint16_t UV;   //低压保护阈值 mV 20mv的整数倍
	uint16_t UVR;  //低压保护恢复阈值 mV 20mv的整数倍
	uint16_t BAL;  //平衡开启点电压 mV 20mv的整数倍
	uint16_t OCD;  //过放电流保护A  20A - 140A ，步进10A
	uint16_t SCV;  //短路保护A 50-350 A ，步进30A
	uint16_t OCC;  //过充电流保护A    20A-140A，步进10A
	uint16_t OTC;  //充电高温保护 *10 K （温度摄氏度 +273.1）*10
	uint16_t OTCR; //充电高温恢复点 *10 K （温度摄氏度 +273.1）*10
	uint16_t UTC;  //充电低温保护   *10 K （温度摄氏度 +273.1）*10
	uint16_t UTCR; //充电低温恢复   *10 K （温度摄氏度 +273.1）*10）
	uint16_t OTD;  //放电高温保护   *10 K  （温度摄氏度 +273.1）*10）
	uint16_t OTDR; //放电高温保护恢复   *10 K （温度摄氏度 +273.1）*10
	uint16_t UTD;  //放电Low保护 *10 K  （温度摄氏度 +273.1）*10
	uint16_t UTDR; //放电Low R保护  *10 K （温度摄氏度 +273.1）*10
} CommdIf_t;

CommdIf_t CmmdConfigData; //参数配置数据结构

stc_niuCommdTable_t niuCommdTable;


extern uint8_t fBleConnedSta;
extern app_drv_fifo_t app_uart_rx_fifo;

uint8_t stateCheckCount = 0;
uint8_t flagStarted20s = 0;
uint8_t flagIntWake = 0;	  //中断唤醒类型 bit0-3 1、ACC 唤醒  2、充电唤醒  4、通信唤醒 8、RTC 唤醒 10 -ALARM 唤醒  20-load 唤醒
uint8_t flagIntEnterType = 0; //进入中断的类型   1、20s后普通休眠进入 2、深度休眠进入 ，低电压休眠，只有充电才能唤醒

static void GpioInterruptConfig(void);
static void Toggle_Led(void);
static void LoadFlashVar(void);
static void System_EnterLowPower(void);
static void System_ExitLowPower(void);
static void NiuLogic_PortInit(void);
static void NiuLogic_UpdateEEPROM(void);
static void NiuTableWriteBatSN(void);
static void NiuLogic_UpdatePTable(void);
static void NiuLogic_ReadShPTable(void);
static void Niulogic_UpdateNiuPtTab(void);
static void Niulogic_UpdateNiuRTab(void);
static void NiuTableMosCtrl(void);
static void Niu_TableSyncHandle(void);
static void NiuLogic_CommdTabInit(void);
//static void NiuLogic_ThreeFulseProtect(void);
static void NiuLogic_PreDsgMosWake(void);
static uint8_t NiuLogic_PreDsgMosHandle(void);



static void Toggle_Led(void)
{
    GPIOA_InverseBits(TN_LED_PAPIN);
}

static void NiuLogic_PortInit()
{

}

//通信选择判定 1-一线通
static uint8_t NiuLogic_CommTypeSel(void)
{

	if (GPIOB_ReadPortPin(TN_ACC_PBPIN))
		return 0;
	else
		if(niuCommdTable.commMode == 0)
		{
			return 1;
		}else
		{
			return 0;
		}
			
			

}

//flash 加载固化参数
static void LoadFlashVar(void)
{
	//设备序列号，通过flash编程自定义烧录，或者软件写入,16个byte
	Flash_Read(BAT_SN_ADDR_START, niuCommdTable.SN_ID, 16);
	//PRINT("SN=%s\n");
}

#if (MCU_LIB_SELECT == 1)
#elif (MCU_LIB_SELECT == 2)



__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void GPIOA_IRQHandler(void)
{
    //通信中断
    if((GPIOA_ReadITFlagBit(TN_SHINT_PAPIN)&TN_SHINT_PAPIN) == TN_SHINT_PAPIN)
    {
        GPIOA_ClearITFlagBit(TN_SHINT_PAPIN);
        flagIntWake |= 0x04;
        afeInfo.MosState_RT |= 0x02;
        //      //PRINT("Event->rx Det interrupt\n");
        bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
    }


}



__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void GPIOB_IRQHandler(void)
{
    //ACC 上拉中断
   if(GPIOB_ReadITFlagBit(TN_ACC_PBPIN)&TN_ACC_PBPIN == TN_ACC_PBPIN)
   {
       GPIOB_ClearITFlagBit(TN_ACC_PBPIN);
       flagIntWake |= 0x01;

   }
   //负载检测中断
   if((GPIOB_ReadITFlagBit(TN_LOAD_DET_PBPIN)&TN_LOAD_DET_PBPIN) == TN_LOAD_DET_PBPIN)
   {
       GPIOB_ClearITFlagBit(TN_LOAD_DET_PBPIN);
       flagIntWake |= 0x20;
   }

   //充电激活中断
   if((GPIOB_ReadITFlagBit(TN_CHG_DET_PBPIN)&TN_CHG_DET_PBPIN) == TN_CHG_DET_PBPIN)
   {
      GPIOB_ClearITFlagBit(TN_CHG_DET_PBPIN);
      flagIntWake |= 0x02;
   }
}

static void GpioInterruptConfig()
{
    GPIOB_ITModeCfg(TN_ACC_PBPIN, GPIO_ITMode_RiseEdge);
    GPIOB_ITModeCfg(TN_LOAD_DET_PBPIN, GPIO_ITMode_RiseEdge);
    GPIOB_ITModeCfg(TN_CHG_DET_PBPIN, GPIO_ITMode_RiseEdge);
    GPIOA_ITModeCfg(TN_SHINT_PAPIN, GPIO_ITMode_RiseEdge);

    PFIC_EnableIRQ(GPIO_A_IRQn);
    PFIC_DisableIRQ(GPIO_B_IRQn);

}

static void System_EnterLowPower(void)
{

	SWITCH_LED_OFF();
	//ADC_DeConfig();
#if (USE_485_IF == 1)
	SWITCH_485_POWER_OFF
#endif

	if (afeInfo.CellVmin < SHIP_MODE_CELLV_TH) //低电压保护后
	{
		//sh309 进入进入ship 模式
		//SH_ENABLE_SHIPMODE();
		AFE_EN_SHIPMODE();
		afeInfo.MosState_RT &= ~0x60;
		//关闭预放电MOS
		SWITCH_PRED_OFF();
		afeInfo.MosState_RT &= ~0x80;

		//禁止RTC 唤醒
		//PFIC_EnableIRQ(RTC_IRQn);

		//禁止TN_WAKE_UP，ACC，SH_INT,ALARM
//		NVIC_DisableIRQ(EXTI2_IRQn);   //ACC
		R16_PB_INT_EN &=~TN_ACC_PBPIN;
//		NVIC_DisableIRQ(EXTI15_10_IRQn); //通信中断
		R16_PA_INT_EN &=~TN_SHINT_PAPIN;

		//负载检测中断
		R16_PB_INT_EN &= ~TN_LOAD_DET_PBPIN;

		//充电检测
		R16_PB_INT_EN |= TN_CHG_DET_PBPIN;


#if (USE_485_IF == 1)
		//NVIC_DisableIRQ(); //485外部唤醒
#endif
		//只应许充电器接入唤醒
		flagIntEnterType = 0x02;
		//PRINT("Event->System goto ShipMode\n");
	}
	else
	{
		//前端芯片睡眠
		//Sh_SetRunMode(0x01);//ALARM 可以唤醒
		//AFE_SET_RUNMODE(0x01);

//		NVIC_EnableIRQ(RTC_IRQn);
//
//		//ACC，SH_INT,ALARM

	    //ACC
	    R16_PB_INT_EN |= TN_ACC_PBPIN;
	    //通信中断
	    R16_PA_INT_EN |=TN_SHINT_PAPIN;
	    //充电检测
	    R16_PB_INT_EN |= TN_CHG_DET_PBPIN;
	    //负载检测中断
	    R16_PB_INT_EN |= TN_LOAD_DET_PBPIN;


		flagIntEnterType = 0x01;
		PRINT("Event->System goto sleep\n");
	}

	//低功耗
	//CH57X_LowPower(5);

}

static void System_ExitLowPower(void)
{
	//唤醒之后，点亮LED
	SWITCH_LED_ON();
	
//	//开启systick中断
//	SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk ;
#if (USE_485_IF == 1)
	SWITCH_485_POWER_ON
#endif

	//ADC_PortInit();
	//ADC_Config();
	Niu_ModbufFifoClr();
	//根据进入中断的类型,处理唤醒之后的操作
	if (flagIntEnterType == 2)
	{
		PRINT("Event->System wakeup from shipmode\n");

		//深度休眠唤醒
		//SH_DISABLE_SHIPMODE();
		AFE_DIS_SHIPMODE();
		bsp_DelayMS(500);
	}
	else
	{
	    PRINT("Event->System wakeup from sleep\n");
		//Sh_SetRunMode(0x00);
	}
#if (USE_485_IF == 1)
	//Gpio_DisableIrq(TN_485_INT_PORT, TN_485_INT_PIN, GpioIrqFalling); //485外部唤醒
#endif
	
}

#endif


static void NiuLogic_PreDsgMosWake(void)
{
	//预放电关闭处理 --------------------------------------
	if (afeInfo.MosState_RT & 0x20)   //放电主MOS 已经开启后
	{
		SWITCH_PRED_OFF();
		afeInfo.MosState_RT &= ~0x80; //设置预放电MOS关闭
	}
	else
		//预放电逻辑判定
		if (afeInfo.State_RT & 0x0200) //短路状态
	{
		SWITCH_PRED_OFF();
		afeInfo.MosState_RT &= ~0x80; //设置预放电MOS关闭
	}
	else

		if (afeInfo.State_RT & 0x0008) //电池过放
	{
		SWITCH_PRED_OFF();
		afeInfo.MosState_RT &= ~0x80; //设置预放电MOS关闭
	}
}

static uint8_t NiuLogic_PreDsgMosHandle(void)
{
	//static DelayFlag = 0;  //0 - 延时没有开启
	static uint8_t preDecState = 0; //预放电检测状态 1-延时20定时器已经开启  2-延时5s定时器已经开启 3-延时30s已经开启 4-预放电锁定
	static uint8_t preOCCount = 0;

	if (preDecState == 4 && ((afeInfo.Pre_State & 0x01) == 0))
	{
		//系统接入，ACC接入，充电唤醒，可以解除预防电锁定
		preDecState = 0;
	}

	if (preDecState == 0)
	{
		if (afeInfo.PreDsgCurrent > 50 && afeInfo.PreDsgCurrent < 100) //预放电电流大于50mA 延迟超20，100mA电流，延迟超5s，连续3次锁定
		{
			preDecState = 1;
			bsp_StartTimer(TMR_PRE_DELAY, 10000); //10s 过流50ma
		}
		else if (afeInfo.PreDsgCurrent > 100) //5s 过流100ma
		{
			preDecState = 1;
			bsp_StartTimer(TMR_PRE_DELAY, 5000);
		}
		else
		{
			preOCCount = 0;
		}

		if (preOCCount >= 3)
		{
			//预放电锁定,预放电锁定需要ACC重新唤醒或者充电再次接入之后
			preDecState = 4;
			SWITCH_PRED_OFF();
			afeInfo.MosState_RT &= ~0x80; //设置预放电MOS关闭
			afeInfo.Pre_State |= 0x01;
			preOCCount = 0;
		}
	}
	else if (preDecState == 1) //大于50ma延时20s开始计时状态
	{						   //连续检测
		if (bsp_CheckTimer(TMR_PRE_DELAY))
		{
			if (afeInfo.PreDsgCurrent > 50)
			{
				SWITCH_PRED_OFF();
			    afeInfo.MosState_RT &= ~0x80; //设置预放电MOS关闭

				preDecState = 2;
				bsp_StartTimer(TMR_PRE_DELAY, 30000); //30恢复状态
			}
			else
			{
				//恢复到初始状态
				preDecState = 0;
				bsp_StopTimer(TMR_PRE_DELAY);
			}
		}
	}
	else if (preDecState == 2) //30s计时恢复状态
	{
		if (bsp_CheckTimer(TMR_PRE_DELAY))
		{
			SWITCH_PRED_ON();
			preOCCount++;
			afeInfo.MosState_RT |= 0x80; //设置预放电MOS打开
			preDecState = 0;
		}
	}

	return preDecState;

	//预放电关闭处理 --------------------------------------
}

static void NiuLogic_MosHanle(void)
{
	uint8_t ret;
	//--CHGDET状态标志检测
	if (GET_CHGDET() == 0)
	{
		afeInfo.MosState_RT &= ~0x04;
	}
	else
	{
	}

	//--ACC状态标志检测
	if (GET_ACC() == 1)
	{
		afeInfo.MosState_RT &= ~0x08;
	}
	else
	{
		//ACC 接入
		afeInfo.MosState_RT |= 0x08;		
	}

	//--wakeup状态标志检测
	

#if (PROJECT_ID == 2)
	if(bsp_CheckTimer(TMR_COMM_INT_DELAY))
	{
		//rx 被常上拉，需上位机强制下发休眠指令，才休眠
		afeInfo.MosState_RT &= ~0x02;
    }
#else
	//--通信中断超时，清除状态
	if(GET_COMM_UP() == 0)
	{
		if (bsp_CheckTimer(TMR_COMM_INT_DELAY))
		{
			afeInfo.MosState_RT &= ~0x02;							
		}		
	}
#endif				
	
	

	//充电活动中、ACC 上拉、有通信唤醒， 不进入休眠
	if ((afeInfo.State_RT & 0x03) || ((afeInfo.MosState_RT & 0x0a) != 0))
	{
		//不是故障状态,OV后，可以放电
		if(afeInfo.State_RT & 0x03 )
		{
			
			if ((afeInfo.MosState_RT & 0x60) != 0x60) //放电MOS没有打开
			{
				bsp_DelayMS(10);
				stateCheckCount++;
				if(stateCheckCount >100)
				{
					stateCheckCount = 0;
					//Sh_OpenDsgMos();
					AFE_OPEN_DMOS();
					afeInfo.MosState_RT |= 0x20;
					//必须打开充电MOS
					AFE_OPEN_CMOS();
					afeInfo.MosState_RT |= 0x40;
					bsp_DelayMS(10);
					//关闭预放电MOS
					SWITCH_PRED_OFF();
					afeInfo.MosState_RT &= ~0x80; //设置预放电MOS为关闭
				}
			}
		}else
		if((afeInfo.State_RT <= 0x0004))
		{
			stateCheckCount = 0;
			//ACC 上拉
			if(afeInfo.MosState_RT & 0x08)
			{
				if((afeInfo.MosState_RT & 0x20) != 0x20)
				{
					//Sh_OpenDsgMos();
					AFE_OPEN_DMOS();
					afeInfo.MosState_RT |= 0x20;
					//必须打开充电MOS
					AFE_OPEN_CMOS();
					afeInfo.MosState_RT |= 0x40;
					bsp_DelayMS(10);
					//关闭预放电MOS
					SWITCH_PRED_OFF();
					afeInfo.MosState_RT &= ~0x80; //设置预放电MOS为关闭
				}			
			}
		}
		
		//判定如果是放电状态，ACC是否是上拉的，否则要停止放电
		//考虑骑行中ACC断线可能，不能断电处理，不加此逻辑，只要有放电，都是打开主MOS
		/*if(afeInfo.State_RT == 0x01)
		{
			if((afeInfo.MosState_RT & 0x08) == 0)
			{
				//关闭预放电MOS
				SWITCH_PRED_OFF();
				afeInfo.MosState_RT &= ~0x80; //设置预放电MOS为关闭
				//关闭放电主MOS
				AFE_CLOSE_DMOS();
				afeInfo.MosState_RT &= ~0x20; //设置放电MOS关闭
			}
		}*/		

		if (afeInfo.AFE_State_RT & 0x4000)
		{
		}
		else
		{
			//清除预放电锁定状态，检测到负载移除
			if(afeInfo.Pre_State & 0x01)
			{
				afeInfo.Pre_State &= ~0x01;

			}
			return;
		}
	}

	//如果负载锁定状态，查看负载是否释放，负载释放解决预放电锁定
	if(afeInfo.Pre_State & 0x01)
	{
		if(GET_LOAD_DET() == 1)
		{
			afeInfo.Pre_State &= ~0x01;
		}
	}


	//无充电接入、无ACC上啦,无串口通信，处于静置状态
	if (((afeInfo.MosState_RT & 0x1E) == 0) && ((afeInfo.State_RT & 0x03) == 0x0))
	{
		if (flagStarted20s == 0)
		{
			//关闭放电主MOS
			//Sh_ShutDDsgMos();
			AFE_CLOSE_DMOS();
			afeInfo.MosState_RT &= ~0x20;

#if(PROJECT_ID != 2)
			if ((afeInfo.State_RT & 0x0200) == 0) //没有短路
			{
				if(afeInfo.Pre_State & 0x01)//预放电锁定
				{					
					SWITCH_PRED_OFF();
					afeInfo.MosState_RT &= ~0x80; //设置预放电MOS打开
				}else
				{
					//打开预放电
					SWITCH_PRED_ON();
					afeInfo.MosState_RT |= 0x80; //设置预放电MOS打开
				}
				
			}
#endif

			//进入休眠之前，充电MOS开关逻辑，无故障状态
			if (afeInfo.State_RT < 0x0004 && afeInfo.CellVmax < MAXCELL_FULL_SLEEP_TH)
			{
				//Sh_OpenChgMos();
				AFE_OPEN_CMOS();
				afeInfo.MosState_RT |= 0x40; //设置充电MOS打开
			}
			else
			{
				//Sh_ShutDChgMos();
				AFE_CLOSE_CMOS();
				afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
			}

			//启动20s定时
			bsp_StartTimer(TMR_NIULOGIC_20S, 20000);
			flagStarted20s = 1;
		}
	}
	else
	{
		if (flagStarted20s == 1)
		{
			//停止20s计时
			bsp_StopTimer(TMR_NIULOGIC_20S);
			flagStarted20s = 0;
		}
	}

	//保护状态，延时20s
	if ((afeInfo.State_RT > 0x0002) && (afeInfo.State_RT != 0x100))
	{
		if (flagStarted20s == 0)
		{
			//启动20s定时
			bsp_StartTimer(TMR_NIULOGIC_20S, 20000);
			flagStarted20s = 1;
		}
	}
#if(PROJECT_ID != 2)
	//预放电电流检测逻辑
	ret = NiuLogic_PreDsgMosHandle();
	if ((ret > 0) && (ret < 4))
	{
		//检测到预放电过流情况，返回再次检测
		return;
	}else
	{
		if(afeInfo.PreDsgCurrent>50)
		{
			return;
		}
	}
#endif

	if (bsp_CheckTimer(TMR_NIULOGIC_20S) || Rtc_GetWakeUpFlag() == 0x01) //唤醒20s 时间超时,RTC 唤醒之后，最多跑一个循环就可以
	{

		//进入休眠前的清除计时开启标志
		flagStarted20s = 0;

		System_EnterLowPower();
		//-----------------------唤醒分界线---------------------//
		System_ExitLowPower();
		//ACC、中断退出处理
		if (flagIntWake & 0x01)
		{
			flagIntWake &= ~0x21;
			if (afeInfo.State_RT < 0x0004) //无故障状态
			{
#if(PROJECT_ID ==2)
				//先打开预放电
				SWITCH_PRED_ON();
				bsp_DelayMS(1000);
				SWITCH_PRED_OFF();

#endif
				//打开放电MOS
				//Sh_OpenDsgMos();
				AFE_OPEN_DMOS();
				//Sh_OpenChgMos();//强制打开充电MOS
				AFE_OPEN_CMOS();

				afeInfo.MosState_RT |= 0x60;
			}
		}

		//充电唤醒中断
		if (flagIntWake & 0x02)
		{
			flagIntWake &= ~0x02;
			if((afeInfo.State_RT == 0) || (afeInfo.State_RT == 0x08))
			{
				//Sh_OpenChgMos();
				AFE_OPEN_DMOS();
				AFE_OPEN_CMOS();
				afeInfo.MosState_RT |= 0x60; //打开充放电MOS
				//充电器接入
				afeInfo.MosState_RT |= 0x04;
			}			
		}
		//串口通信中断
		if (flagIntWake & 0x04)
		{
			flagIntWake &= ~0x04;
		}
		//唤醒之后预放电处理
		NiuLogic_PreDsgMosWake();
		//唤醒之后，需要马上采样数据
		Afe_SetInitSample();
	}
}


#if (AFE_CHIP_SELECT == 1)
//更新EEPROM 默认参数
static void NiuLogic_UpdateEEPROM(void)
{
	//OVDELAY
	EEP_RegMap[2] &= ~(0xF0);
	switch(OC_VLT_DELAY_H)
	{
		case 1000:
			EEP_RegMap[2] |= 0x60;
			break;
		case 2000:
			EEP_RegMap[2] |= 0x70;
			break;
		case 3000:
			EEP_RegMap[2] |= 0x80;
			break;
		case 4000:
			EEP_RegMap[2] |= 0x90;
			break;
		case 6000:
			EEP_RegMap[2] |= 0xa0;
			break;
		case 8000:
			EEP_RegMap[2] |= 0xb0;
			break;
		case 10000:
			EEP_RegMap[2] |= 0xc0;
			break;
		case 20000:
			EEP_RegMap[2] |= 0xd0;
			break;
	}
	//UVDELAY
	EEP_RegMap[4] &= ~(0xF0);
	switch(DC_VLT_DELAY_H)
	{
		case 1000:
			EEP_RegMap[4] |= 0x60;
			break;
		case 2000:
			EEP_RegMap[4] |= 0x70;
			break;
		case 3000:
			EEP_RegMap[4] |= 0x80;
			break;
		case 4000:
			EEP_RegMap[4] |= 0x90;
			break;
		case 6000:
			EEP_RegMap[4] |= 0xa0;
			break;
		case 8000:
			EEP_RegMap[4] |= 0xb0;
			break;
		case 10000:
			EEP_RegMap[4] |= 0xc0;
			break;
		case 20000:
			EEP_RegMap[4] |= 0xd0;
			break;
	}

	//OC_DELAY 
	EEP_RegMap[15] &= ~(0x0F);
	switch(CC_DELAY_H)
	{
		case 1000:
			EEP_RegMap[15] |= 0x0a;
			break;
		case 2000:
			EEP_RegMap[15] |= 0x0b;
			break;
		case 4000:
			EEP_RegMap[15] |= 0x0c;
			break;
		case 8000:
			EEP_RegMap[15] |= 0x0d;
			break;
		case 10000:
			EEP_RegMap[15] |= 0x0e;
			break;
		case 20000:
			EEP_RegMap[15] |= 0x0f;
			break;
	}

	//DC_DELAY 
	EEP_RegMap[13] &= ~(0x0F);
	switch(DC_DELAY_H)
	{
		case 50:
			EEP_RegMap[13] |= 0x00;
			break;
		case 100:
			EEP_RegMap[13] |= 0x01;
			break;
		case 200:
			EEP_RegMap[13] |= 0x02;
			break;
		case 400:
			EEP_RegMap[13] |= 0x03;
			break;
		case 600:
			EEP_RegMap[13] |= 0x04;
			break;
		case 800:
			EEP_RegMap[13] |= 0x05;
			break;
		case 1000:
			EEP_RegMap[13] |= 0x06;
			break;
		case 2000:
			EEP_RegMap[13] |= 0x07;
			break;
		case 4000:
			EEP_RegMap[13] |= 0x08;
			break;
		case 6000:
			EEP_RegMap[13] |= 0x09;
			break;
		case 8000:
			EEP_RegMap[13] |= 0x0a;
			break;
	}

	//SCD_DELAY
	EEP_RegMap[14] &= ~(0x0F);
	switch(ISC_DELAY_H)
	{
		case 0:
			EEP_RegMap[14] |= 0x00;
			break;
		case 64:
			EEP_RegMap[14] |= 0x01;
			break;
		case 128:
			EEP_RegMap[14] |= 0x02;
			break;
		case 192:
			EEP_RegMap[14] |= 0x03;
			break;
		case 256:
			EEP_RegMap[14] |= 0x04;
			break;
		case 320:
			EEP_RegMap[14] |= 0x05;
			break;
		case 384:
			EEP_RegMap[14] |= 0x06;
			break;
		case 448:
			EEP_RegMap[14] |= 0x07;
			break;
		case 512:
			EEP_RegMap[14] |= 0x08;
			break;
		case 576:
			EEP_RegMap[14] |= 0x09;
			break;
		case 640:
			EEP_RegMap[14] |= 0x0a;
			break;
		case 704:
			EEP_RegMap[14] |= 0x0b;
			break;
		case 768:
			EEP_RegMap[14] |= 0x0c;
			break;
		case 832:
			EEP_RegMap[14] |= 0x0d;
			break;
		case 896:
			EEP_RegMap[14] |= 0x0e;
			break;
		case 960:
			EEP_RegMap[14] |= 0x0f;
			break;
	}

	//OV
	EEP_RegMap[2] &= ~(0x03);
	EEP_RegMap[2] |= (uint8_t)((CmmdConfigData.OV / 5) >> 8) & 0x03;
	EEP_RegMap[3] = (uint8_t)((CmmdConfigData.OV / 5));
	//OVR
	EEP_RegMap[4] &= ~(0x03);
	EEP_RegMap[4] |= (uint8_t)((CmmdConfigData.OVR / 5) >> 8) & 0x03;
	EEP_RegMap[5] = (uint8_t)((CmmdConfigData.OVR / 5));
	//UV
	EEP_RegMap[6] = (uint8_t)(CmmdConfigData.UV / 20);
	//UVR
	EEP_RegMap[7] = (uint8_t)(CmmdConfigData.UVR / 20);
	//BAL
	EEP_RegMap[8] = (uint8_t)(CmmdConfigData.BAL / 20);
	EEP_RegMap[12] &= 0x0F;
	EEP_RegMap[12] |= ((uint8_t)((CmmdConfigData.OCD - 20) / 10) & 0x0F) << 4;
	//SCV
	EEP_RegMap[14] &= 0x0F;
	EEP_RegMap[14] |= ((uint8_t)((CmmdConfigData.SCV - 50) / 30) & 0x0F) << 4;
	//充电过流保护
	EEP_RegMap[15] &= 0x0F;
	EEP_RegMap[15] |= ((uint8_t)((CmmdConfigData.OCC - 20) / 10) & 0x0F) << 4;
	//温度阈值寄存器设置
	EEP_RegMap[17] = Sh_ConvertTemp2RegTh(CmmdConfigData.OTC, 0);
	EEP_RegMap[18] = Sh_ConvertTemp2RegTh(CmmdConfigData.OTCR, 0);
	EEP_RegMap[19] = Sh_ConvertTemp2RegTh(CmmdConfigData.UTC, 1);
	EEP_RegMap[20] = Sh_ConvertTemp2RegTh(CmmdConfigData.UTCR, 1);
	EEP_RegMap[21] = Sh_ConvertTemp2RegTh(CmmdConfigData.OTD, 0);
	EEP_RegMap[22] = Sh_ConvertTemp2RegTh(CmmdConfigData.OTDR, 0);
	EEP_RegMap[23] = Sh_ConvertTemp2RegTh(CmmdConfigData.UTD, 1);
	EEP_RegMap[24] = Sh_ConvertTemp2RegTh(CmmdConfigData.UTDR, 1);
}
#endif
//更新保护参数表
static void NiuLogic_UpdatePTable()
{
	CmmdConfigData.OV = (uint16_t)OC_VLT_P+10;					//过压保护阈值  mV 5mv的整数倍,比软件动作值高10mv
	CmmdConfigData.OVR = (uint16_t)ROC_VLT_P;					//过压保护恢复阈值 mV 5mv的整数倍
	CmmdConfigData.UV = (uint16_t)DC_VLT_P -400;					//低压保护阈值 mV 20mv的整数倍，比软件值低400mv
	CmmdConfigData.UVR = (uint16_t)RDC_VLT_P;				//低压保护恢复阈值 mV 20mv的整数倍
	CmmdConfigData.BAL = (uint16_t)BALANCE_START_V;					//平衡开启点电压 mV 20mv的整数倍
	CmmdConfigData.OCD = (uint16_t)DC_CUR_P*1.5;					//过放电流保护A  20A - 140A ，步进10A，是软件值的*1.5，动作时间600ms
	CmmdConfigData.SCV = (uint16_t)ISC_P;							//短路保护A 50-350 A ，步进30A
	CmmdConfigData.OCC = (uint16_t)C_CUR_P+10;						//过充电流保护A    20A-140A，步进10A，比软件值多10A
	CmmdConfigData.OTC = (uint16_t)((int8_t)COTP_P + 273+20) * 10;	//充电高温保护 *10 K （温度摄氏度 +273.1）*10 比软件值高20°
	CmmdConfigData.OTCR = (uint16_t)((int8_t)RCOTP_P + 273) * 10; //充电高温恢复点 *10 K （温度摄氏度 +273.1）*10
	CmmdConfigData.UTC   = 2731 -100;//(uint16_t)(niuCommdTable.RCoTp_P+273)*10;;  //充电低温保护   *10 K （温度摄氏度 +273.1）*10
	CmmdConfigData.UTCR  = 2731 -50;//; //充电低温恢复   *10 K （温度摄氏度 +273.1）*10）
	CmmdConfigData.OTD = (uint16_t)((int8_t)DCOTP_P + 273+5) * 10;	 //放电高温保护   *10 K  （温度摄氏度 +273.1）*10），比软件值大5°
	CmmdConfigData.OTDR = (uint16_t)((int8_t)RDCOTP_P + 273) * 10; //放电高温保护恢复   *10 K （温度摄氏度 +273.1）*10
	CmmdConfigData.UTD = (uint16_t)((int8_t)DCLTP_P + 273-5) * 10;	 //放电Low保护 *10 K  （温度摄氏度 +273.1）*10，比软件值低5°
	CmmdConfigData.UTDR = (uint16_t)((int8_t)RDCLTP_P + 273) * 10; //放电Low R保护  *10 K （温度摄氏度 +273.1）*10
}
#if (AFE_CHIP_SELECT == 1)
//读取SH309 保护参数值到niuCommdTable
static void NiuLogic_ReadShPTable()
{
	CmmdConfigData.OV = 5 * (((uint16_t)(EE_ReadBuf[2] & 0x03) << 8) + EE_ReadBuf[3]); //过压保护阈值  mV
	CmmdConfigData.OVR = 5 * (((uint16_t)(EE_ReadBuf[4] & 0x03) << 8) + EE_ReadBuf[5]);
	;										 //过压保护恢复阈值 mV
	CmmdConfigData.UV = 20 * EE_ReadBuf[6];	 //低压保护阈值 mV
	CmmdConfigData.UVR = 20 * EE_ReadBuf[7]; //低压保护恢复阈值 mV
	CmmdConfigData.BAL = 20 * EE_ReadBuf[8]; //平衡开启点电压 mV

	if ((EE_ReadBuf[12] & 0xF0) <= 0xC0)
	{
		CmmdConfigData.OCD = 10 * ((EE_ReadBuf[12] & 0xF0) >> 4) + 20; //过放电流保护mA
	}
	else if ((EE_ReadBuf[12] & 0xF0) == 0xD0)
	{
		CmmdConfigData.OCD = 160;
	}
	else if ((EE_ReadBuf[12] & 0xF0) == 0xE0)
	{
		CmmdConfigData.OCD = 180;
	}
	else if ((EE_ReadBuf[12] & 0xF0) == 0xF0)
	{
		CmmdConfigData.OCD = 200;
	}

	//短路保护A
	if ((EE_ReadBuf[14] & 0xF0) <= 0xA0)
	{
		CmmdConfigData.SCV = 30 * ((EE_ReadBuf[14] & 0xF0) >> 4) + 50;
	}
	else if ((EE_ReadBuf[14] & 0xF0) == 0xB0)
	{
		CmmdConfigData.SCV = 400;
	}
	else if ((EE_ReadBuf[14] & 0xF0) == 0xC0)
	{
		CmmdConfigData.SCV = 500;
	}
	else if ((EE_ReadBuf[14] & 0xF0) == 0xD0)
	{
		CmmdConfigData.SCV = 600;
	}

	//充电过流保护
	if ((EE_ReadBuf[15] & 0xF0) <= 0xC0)
	{
		CmmdConfigData.OCC = 10 * ((EE_ReadBuf[15] & 0xF0) >> 4) + 20; //过放电流保护A
	}

	//OTC value
	CmmdConfigData.OTC = Sh_ConvertTempTh2Value(EE_ReadBuf[17], 0);	 //充电高温保护   *10 K
	CmmdConfigData.OTCR = Sh_ConvertTempTh2Value(EE_ReadBuf[18], 0); //充电高温恢复点   *10 K
	CmmdConfigData.UTC = Sh_ConvertTempTh2Value(EE_ReadBuf[19], 1);	 //充电低温保护   *10 K
	CmmdConfigData.UTCR = Sh_ConvertTempTh2Value(EE_ReadBuf[20], 1); //充电低温恢复   *10 K
	CmmdConfigData.OTD = Sh_ConvertTempTh2Value(EE_ReadBuf[21], 0);	 //放电高温保护  *10 K
	CmmdConfigData.OTDR = Sh_ConvertTempTh2Value(EE_ReadBuf[22], 0); //放电高温保护恢复 *10 K
	CmmdConfigData.UTD = Sh_ConvertTempTh2Value(EE_ReadBuf[23], 1);	 //放电Low保护    *10 K
	CmmdConfigData.UTDR = Sh_ConvertTempTh2Value(EE_ReadBuf[24], 1); //放电Low R保护  *10 K
}
#elif (AFE_CHIP_SELECT == 2)
static void NiuLogic_ReadDVCPTable()
{
	CmmdConfigData.OV = (uint16_t)(AFE_OVT); //(AFE_CFG1_DEF_VAL4+546)*5.86);//过压保护阈值  mV OV1T/OV2T/OV3T
	CmmdConfigData.OVR = (uint16_t)AFE_OVRT;
	;												//过压保护恢复阈值 mV
	CmmdConfigData.UV = (uint16_t)(AFE_UVT);		//((AFE_CFG1_DEF_VAL5+324)*5.86);	 //低压保护阈值 mV
	CmmdConfigData.UVR = (uint16_t)AFE_UVRT;		//低压保护恢复阈值 mV
	CmmdConfigData.BAL = (uint16_t)BALANCE_START_V; //平衡开启点电压 mV
	CmmdConfigData.OCD = (uint16_t)AFE_OCDT;
	CmmdConfigData.SCV = (uint16_t)AFE_SCDT;
	CmmdConfigData.OCC = (uint16_t)AFE_OCCT;

	//OTC value
	CmmdConfigData.OTC = (uint16_t)(AFE_OTC * 10 + 2731);	//充电高温保护   *10 K
	CmmdConfigData.OTCR = (uint16_t)(AFE_OTCR * 10 + 2731); //充电高温恢复点   *10 K
	CmmdConfigData.UTC = (uint16_t)(AFE_UTC * 10 + 2731);	//充电低温保护   *10 K
	CmmdConfigData.UTCR = (uint16_t)(AFE_UTCR * 10 + 2731); //充电低温恢复   *10 K
	CmmdConfigData.OTD = (uint16_t)(AFE_OTD * 10 + 2731);	//放电高温保护  *10 K
	CmmdConfigData.OTDR = (uint16_t)(AFE_OTDR * 10 + 2731); //放电高温保护恢复 *10 K
	CmmdConfigData.UTD = (uint16_t)(AFE_UTD * 10 + 2731);	//放电Low保护    *10 K
	CmmdConfigData.UTDR = (uint16_t)(AFE_UTDR * 10 + 2731); //放电Low R保护  *10 K
}

#endif

//读取前端保护参数到NiuTable
static void Niulogic_UpdateNiuPtTab()
{
	niuCommdTable.OC_Vlt_P = (uint8_t)(OC_VLT_P/ 100);//(uint8_t)(CmmdConfigData.OV / 100);		 //过压保护阈值  mV 5mv的整数倍
	niuCommdTable.ROC_Vlt = (uint8_t)(ROC_VLT_P/ 100);//(uint8_t)(CmmdConfigData.OVR / 100);		 //过压保护恢复阈值 mV 5mv的整数倍
	niuCommdTable.Dc_Vlt_P = (uint8_t)(DC_VLT_P/ 100);//(uint8_t)(CmmdConfigData.UV / 100);		 //低压保护阈值 mV 20mv的整数倍
	niuCommdTable.RDc_Vlt_P = (uint8_t)(RDC_VLT_P/100);//(uint8_t)(CmmdConfigData.UVR / 100);		 //低压保护恢复阈值 mV 20mv的整数倍
	niuCommdTable.C_B_Vlt = (uint8_t)(BALANCE_START_V/100);//(uint8_t)(CmmdConfigData.BAL / 100);		 //平衡开启点电压 mV 20mv的整数倍
	niuCommdTable.Dc_Cur_P = (uint8_t)DC_CUR_P;//(uint8_t)CmmdConfigData.OCD;				 //过放电流保护A  20A - 140A ，步进10A
	niuCommdTable.Isc_P = (uint8_t)ISC_P;//(uint8_t)CmmdConfigData.SCV;					 //短路保护A 50-350 A ，步进30A
	niuCommdTable.C_Cur_P = (uint8_t)C_CUR_P;//(uint8_t)CmmdConfigData.OCC;				 //过充电流保护A    20A-140A，步进10A
	niuCommdTable.COTp_P =(int8_t) COTP_P;//(int8_t)((CmmdConfigData.OTC - 2731) / 10);	 //充电高温保护 *10 K （温度摄氏度 +273.1）*10
	niuCommdTable.RCoTp_P =(int8_t)RCOTP_P; //(int8_t)((CmmdConfigData.OTCR - 2731) / 10); //充电高温恢复点 *10 K （温度摄氏度 +273.1）*10
	//CmmdConfigData.UTC   = //(uint16_t)(niuCommdTable.RCoTp_P+273)*10;;  //充电低温保护   *10 K （温度摄氏度 +273.1）*10
	//CmmdConfigData.UTCR  = //; //充电低温恢复   *10 K （温度摄氏度 +273.1）*10）
	niuCommdTable.DcOTp_P = (int8_t)DCOTP_P;//(int8_t)((CmmdConfigData.OTD - 2731) / 10);	  //放电高温保护   *10 K  （温度摄氏度 +273.1）*10）
	niuCommdTable.RDcOTp_P = (int8_t)RDCOTP_P;//(int8_t)((CmmdConfigData.OTDR - 2731) / 10); //放电高温保护恢复   *10 K （温度摄氏度 +273.1）*10
	niuCommdTable.DcLTp_P = (int8_t)DCLTP_P;//(int8_t)((CmmdConfigData.UTD - 2731) / 10);	  //放电Low保护 *10 K  （温度摄氏度 +273.1）*10
	niuCommdTable.RDcLTp_P = (int8_t)RDCLTP_P;//(int8_t)((CmmdConfigData.UTDR - 2731) / 10); //放电Low R保护  *10 K （温度摄氏度 +273.1）*10
	//niuCommdTable.commMode = 0;
}

//默认值初始化
static void NiuLogic_CommdTabInit()
{
	uint16_t tempK;
	uint16_t tempO;
	uint16_t tempKv;
	uint16_t tempKvO;
	
	niuCommdTable.BMS_ID = 0x31;
	niuCommdTable.User = 0x20;
	niuCommdTable.S_Ver = FW_VERSION;
	niuCommdTable.H_Ver = HW_VERSION;
	niuCommdTable.P_Pwrd[0] = 0x55;
	niuCommdTable.P_Pwrd[1] = 0x55;
	niuCommdTable.P_Pwrd[2] = 0x55;
	niuCommdTable.P_Pwrd[3] = 0x55;
	niuCommdTable.Bat_Tytp = BAT_TYPE;		  //电芯类型
	niuCommdTable.Rated_Vlt = BAT_RATED_VOLT; //电池额定电压
	niuCommdTable.commMode = 0;

	//校准参数默认初始化
	Flash_Read(EE_START_ADDR + CALI_CURR_PARA_VAL, (uint8_t*)&niuCommdTable.Ratio_KvH, 8);
	

	tempK = ((uint16_t)niuCommdTable.Ratio_KcH<<8) +niuCommdTable.Ratio_KcL;
	tempO = ((uint16_t)niuCommdTable.Ratio_OffsetcH<<8) +niuCommdTable.Ratio_OffsetcL;

	tempKv = ((uint16_t)niuCommdTable.Ratio_KvH<<8) +niuCommdTable.Ratio_KvL;;
	tempKvO = ((uint16_t)niuCommdTable.Ratio_OffsetvH<<8) +niuCommdTable.Ratio_OffsetvL;

	if((tempKv > 1100) || (tempKv <900))
	{
	    niuCommdTable.Ratio_KvH = (uint8_t)(DEFAULT_RATIO_V_K>>8);
	    niuCommdTable.Ratio_KvL = (uint8_t)(DEFAULT_RATIO_V_K);
	    niuCommdTable.Ratio_OffsetvH = (uint8_t)(DEFAULT_RATIO_V_O>>8);
	    niuCommdTable.Ratio_OffsetvL = (uint8_t)(DEFAULT_RATIO_V_O);
	    niuCommdTable.Ratio_KcH = (uint8_t)(DEFAULT_RATIO_C_K>>8);
	    niuCommdTable.Ratio_KcL = (uint8_t)(DEFAULT_RATIO_C_K);
	    niuCommdTable.Ratio_OffsetcH = (uint8_t)(DEFAULT_RATIO_C_O>>8);
	    niuCommdTable.Ratio_OffsetcL = (uint8_t)(DEFAULT_RATIO_C_O);
	}else {
	    Sh_SetCurrCarlibation(tempK,tempO);
	    Sh_SetVoltCarlibation(tempKv,tempKvO);

	}

}

//向NiuTable更新实时电压，电流，单体电压，温度等信息
static void Niulogic_UpdateNiuRTab()
{
	uint8_t i;

	//FCC
	niuCommdTable.Bat_To_Cap[0] = (uint8_t)((uint16_t)(1000 * algIntnel.capacity) >> 8); //可用容量
	niuCommdTable.Bat_To_Cap[1] = (uint8_t)((uint16_t)(1000 * algIntnel.capacity));		//可用容量
																							//已充电次数
	niuCommdTable.C_Cont[0] = (uint8_t)(algEnginer.cycCount >> 8);							//已充电次数
	niuCommdTable.C_Cont[1] = (uint8_t)algEnginer.cycCount;									//已充电次数

	//总电压
	niuCommdTable.To_Vlt_RT[0] = (uint8_t)((afeInfo.SumBatteryPackVolt / 100) >> 8);
	niuCommdTable.To_Vlt_RT[1] = (uint8_t)(afeInfo.SumBatteryPackVolt / 100);

	//充电电流
	niuCommdTable.C_Cur_RT[0] = (uint8_t)((afeInfo.ChgCurrent / 100) >> 8);
	niuCommdTable.C_Cur_RT[1] = (uint8_t)(afeInfo.ChgCurrent / 100);

	//放电电流
	niuCommdTable.Dc_Cur_RT[0] = (uint8_t)((afeInfo.DsgCurrent / 100) >> 8);
	niuCommdTable.Dc_Cur_RT[1] = (uint8_t)(afeInfo.DsgCurrent / 100);

	//SOC
	niuCommdTable.SOC_RT = (uint8_t)(algEnginer.soc_r / 10);

	//电池实时状态
	niuCommdTable.Bat_Sta_RT[0] = (uint8_t)(afeInfo.State_RT >> 8);
	niuCommdTable.Bat_Sta_RT[1] = (uint8_t)(afeInfo.State_RT);
	//充满电剩余时间
	niuCommdTable.DC_Fl_T_RT = (uint8_t)algEnginer.chgResTime;

	//温探
	niuCommdTable.Tp1_RT[0] = (int8_t)(((int16_t)afeInfo.ShTemp[0] - 2731) / 10); //前端温探1
	niuCommdTable.Tp1_RT[1] = (int8_t)(((int16_t)afeInfo.ShTemp[1] - 2731) / 10); //前端温探2
	niuCommdTable.Tp1_RT[2] = (int8_t)(((int16_t)afeInfo.ShTemp[2] - 2731) / 10); //前端温探3
	//
	niuCommdTable.Tp5_RT = (uint8_t)(((int16_t)afeInfo.MosTemp - 2731) / 10); //mos温探

	//单体电压
	for (i = 0; i < CELL_NUMS; i++)
	{
		niuCommdTable.G_Vlt_RT[2 * i] = (uint8_t)(afeInfo.CellVolt[i] >> 8);
		niuCommdTable.G_Vlt_RT[2 * i + 1] = (uint8_t)(afeInfo.CellVolt[i]);
	}

	//软件版本和硬件版本号
	for (uint8_t i = 0; i < 8; i++)
	{
		if (*((uint8_t *)NIU_FW_VER + i) != 0xFF)
		{
			niuCommdTable.S_Ver_N[i] = *((uint8_t *)NIU_FW_VER + i);
		}

		if (*((uint8_t *)NIU_HW_VER + i) != 0xFF)
		{
			niuCommdTable.H_Ver_N[i] = *((uint8_t *)NIU_HW_VER + i);
		}

		//天能内部软件版本号
		if (*((uint8_t *)TN_FW_VER + i) != 0xFF)
		{
			niuCommdTable.TN_S_Ver_N[i] = *((uint8_t *)TN_FW_VER + i);
		}
	}
	//均衡回路温度
	niuCommdTable.Blance_T[0] = (uint8_t)(((int16_t)(afeInfo.BalanceTemp - 2731)) >> 8); //均衡温探
	niuCommdTable.Blance_T[1] = (uint8_t)((int16_t)(afeInfo.BalanceTemp - 2731));		  //均衡温探
	//电池串数
	niuCommdTable.Cell_Num = (uint8_t)CELL_NUMS;

	//最高单体电压
	niuCommdTable.Cell_Vmax[0] = (uint8_t)((afeInfo.CellVmax) >> 8);
	niuCommdTable.Cell_Vmax[1] = (uint8_t)(afeInfo.CellVmax);
	//最低单体电压
	niuCommdTable.Cell_Vmin[0] = (uint8_t)((afeInfo.CellVmin) >> 8);
	niuCommdTable.Cell_Vmin[1] = (uint8_t)(afeInfo.CellVmin);

	//单体最高电压位置
	niuCommdTable.Cell_Vmax_Num = (uint8_t)(afeInfo.CellVmaxPos);

	//单体最低电压位置
	niuCommdTable.Cell_Vmin_Num = (uint8_t)(afeInfo.CellVminPos);

	//压差
	niuCommdTable.Cell_Vdiff[0] = (uint8_t)((afeInfo.CellVdiff) >> 8);
	niuCommdTable.Cell_Vdiff[1] = (uint8_t)(afeInfo.CellVdiff);

	//最高单体温度
	niuCommdTable.Cell_Tmax[0] = (uint8_t)(((int16_t)(afeInfo.CellTmax-2731)) >> 8);
	niuCommdTable.Cell_Tmax[1] = (uint8_t)((int16_t)(afeInfo.CellTmax-2731));
	//最低单体温度
	niuCommdTable.Cell_Tmin[0] = (uint8_t)(((int16_t)(afeInfo.CellTmin-2731)) >> 8);
	niuCommdTable.Cell_Tmin[1] = (uint8_t)((int16_t)(afeInfo.CellTmax-2731));

	//单体最高温度位置
	niuCommdTable.Cell_Tmax_Num = (uint8_t)(afeInfo.CellTmaxPos);

	//单体最低温度位置
	niuCommdTable.Cell_Tmin_Num = (uint8_t)(afeInfo.CellTminPos);

	//温度差
	niuCommdTable.Cell_Tdiff[0] = (uint8_t)((afeInfo.CellTdiff/10) >> 8);
	niuCommdTable.Cell_Tdiff[1] = (uint8_t)(afeInfo.CellTdiff/10);

	//心跳信号
	int32_t heart = 0;//bsp_GetRunTime() / 1000;
	niuCommdTable.LifeValue = (uint8_t)heart;

	//SOC High
	niuCommdTable.SOC_High_Precision = 10 * algEnginer.soc_r/40;
	niuCommdTable.SOH_RT = algEnginer.soh_r;

	//充电需求电压
	niuCommdTable.ChargDemandVolt[0] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_VOLT >> 8);
	niuCommdTable.ChargDemandVolt[1] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_VOLT);
	//充电需求电流
	niuCommdTable.ChargDemandCurrent[0] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_CURR >> 8);
	niuCommdTable.ChargDemandCurrent[1] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_CURR);

	//当前放电最大功率
	niuCommdTable.DisCharg_Max_P[0] = (uint8_t)(afeInfo.SumBatteryPackVolt * afeInfo.DsgCurrent / 1000000 >> 8);
	niuCommdTable.DisCharg_Max_P[1] = (uint8_t)(afeInfo.SumBatteryPackVolt * afeInfo.DsgCurrent / 1000000);
	//当前充电最大功率
	niuCommdTable.Charg_Max_P[0] = (uint8_t)(afeInfo.SumBatteryPackVolt * afeInfo.ChgCurrent / 1000000 >> 8);
	niuCommdTable.Charg_Max_P[1] = (uint8_t)(afeInfo.SumBatteryPackVolt * afeInfo.ChgCurrent / 1000000);

	//平衡开关状态
	niuCommdTable.Blance_Status[0] = (uint8_t)afeInfo.balState >> 24;
	niuCommdTable.Blance_Status[1] = (uint8_t)(afeInfo.balState >> 16);
	niuCommdTable.Blance_Status[2] = (uint8_t)(afeInfo.balState >> 8);
	niuCommdTable.Blance_Status[3] = (uint8_t)afeInfo.balState;

	//MOS 开关状态
	niuCommdTable.MOS_Status = afeInfo.MosState_RT;
	MyMemcpy((uint8_t*)&niuCommdTable.algInfo,(uint8_t*)&algIntnel,sizeof(algIntnel));
	MyMemcpy((uint8_t*)&niuCommdTable.algengInfo,(uint8_t*)&algEnginer,sizeof(algEnginer));
}

void NiuTableWriteCalibration()
{
	uint16_t tempK;
	uint16_t tempO;
	uint16_t tempKv;
	uint16_t tempKvO;
	//保存电流校准参数
	Flash_Write(EE_START_ADDR + CALI_CURR_PARA_VAL, (uint8_t*)&niuCommdTable.Ratio_KvH, 8);
	tempK = ((uint16_t)niuCommdTable.Ratio_KcH<<8) +niuCommdTable.Ratio_KcL;
	tempO = ((uint16_t)niuCommdTable.Ratio_OffsetcH<<8) +niuCommdTable.Ratio_OffsetcL;
	tempKv = ((uint16_t)niuCommdTable.Ratio_KvH<<8) +niuCommdTable.Ratio_KvL;;
	tempKvO = ((uint16_t)niuCommdTable.Ratio_OffsetvH<<8) +niuCommdTable.Ratio_OffsetvL;
	Sh_SetCurrCarlibation(tempK,tempO);
	Sh_SetVoltCarlibation(tempKv,tempKvO);
}

static void NiuTableMosCtrl()
{
	if (niuCommdTable.MOS_Ctrol & 0x80)
	{
		//预防的MOS强制开启
		//清楚标志
	}
	else if (niuCommdTable.MOS_Ctrol & 0x40)
	{
		//充电MOS强制开启
		//清楚标志
	}
	else if (niuCommdTable.MOS_Ctrol & 0x20)
	{
		//放电MOS强制开启
	}
}

static void NiuTableWriteBatSN(void)
{
	Flash_Write(EE_START_ADDR + BAT_SN_ADDR_START, niuCommdTable.SN_ID, 16);
}

/**
 * function:Niu_TableSyncHandle 
 * @para : 
 * @    :
 * @    :
 * @    :
 * @return :无
 */
static void Niu_TableSyncHandle(void)
{
	if(NiuMdSycWriteFlg & 0x04)
	{
		NiuTableWriteCalibration();
		NiuMdSycWriteFlg = 0;
	}
	else if (NiuMdSycWriteFlg & 0x02)
	{
		//其他参数需要写入更新
		NiuTableMosCtrl();
		//写电池ID
		NiuTableWriteBatSN();
		NiuMdSycWriteFlg = 0;
	}
	else
	{
#if (AFE_CHIP_SELECT == 1)
		NiuLogic_ReadShPTable();
#elif (AFE_CHIP_SELECT == 2)
		//集澈芯片读参数保护表

#endif
		//读取保护参数
		
		//读取实时信息
		Niulogic_UpdateNiuRTab();
	}
}

void NiuLogicInit(void)
{

	LoadFlashVar();
	NiuLogic_PortInit();
	GpioInterruptConfig();
	AFE_Init();

	Uart3Init(NIU_ModbusRecvHandle);
	NiuLogic_CommdTabInit();
	Niulogic_UpdateNiuPtTab();
	NiuLogic_UpdatePTable();

#if (AFE_CHIP_SELECT == 1)
		//配置到EEPROM_Map表中，并更新到Sh309EEPROM中
		//CmmdConfigData - >EEP_RegMap
		NiuLogic_UpdateEEPROM();
		//EEP_RegMap ->SH309 内部寄存器
		Sh_UpdateEEMapAll();
#elif (AFE_CHIP_SELECT == 2)

#endif

	AlgEngineInit();
	Niu_OneBusInit();
	bsp_StartAutoTimer(TMR_MAIN, 1000);
	bsp_StartAutoTimer(TMR_PROTECT_DELAY,1000);
}

//模拟蓝牙下发指令


void NiuLogicRun(void)
{
	static uint8_t swCount = 0;
	uint8_t cmd[10] = {0x68,0x31,0xce,0x68,0x02,0x02,0x33,0xd3,0xd9,0x16};

	swCount++;
	if(swCount>10)
	{
	    swCount = 0;
	}

	if (bsp_CheckTimer(TMR_MAIN))
	{
		Toggle_Led();

		//判断蓝牙是否已经连接
		if(fBleConnedSta == 1)
		{
		    Niu_ModbusCfg(1,&app_uart_rx_fifo);
		    for(uint8_t i = 0;i <10;i++)
		    {
		    	NIU_ModbusRecvHandle(cmd[i]);
		    }
		}
	}

	switch(swCount)
	{
	  case 1:
	    AFE_Process();
	    break;
	  case 2:
	    AlgEngineProcess();
	    break;
	  case 3:
	    Niu_TableSyncHandle();
	    break;
	  case 4:
		//一线通逻辑切换
	      	if (NiuLogic_CommTypeSel())
	      	{
	      #if (USE_485_IF != 1)
	      		//Disable Uart 接收中断
	      		//DISABLE_UART_RXINT();
	      	      UART3_Reset();
	      //一线通发送数据
	      #if (ONEBUS_TYPE == 1) //小牛
	      		//Niu_OneBusProcess();
	      #elif (ONEBUS_TYPE == 2) //爱玛 天能
	      		TN_OneBusProcess();
	      #elif (ONEBUS_TYPE == 3) //雅迪一线通50字节
	      		Yd_OneBusProcess();
	      #elif (ONEBUS_TYPE == 4) //新日一线通12字节
	      		Xr_OneBusProcess();
	      #elif (ONEBUS_TYPE == 7) //钻豹一线通数据发送
	      		Zb_OneBusProcess();
	      #endif
	      		//Enable Uart 接收中断
	      		Uart3Init(NIU_ModbusRecvHandle);

	      #endif
	      	}
	      	else
	      	{

	      	}
	    break;

	  case 5:
	    NiuModbusPoll();
	    break;

	  case 6:
	    NiuLogic_MosHanle();
	    break;
	  default:
	    break;

	}
}

#endif
