/******************************************************************************/
/** \file niuLogic.c
 **
 ** niu F serial bms logic
 **
 **   - 2021-9 hxbao    First Version
 **
 ******************************************************************************/

#include "includes.h"
#if(PROJECT_ID == 2)

#define TY_GETSWITCHSTA() MCU_GPIO_GetBit(GpioPortA,GpioPin11)
#define SWITCH_LED2_ON() MCU_GPIO_ClrBit(GpioPortA,GpioPin5)
#define SWITCH_LED2_OFF() MCU_GPIO_SetBit(GpioPortA,GpioPin5)

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

uint16_t switch_count = 0;
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
static void NiuLogic_ThreeFulseProtect(void);
static void NiuLogic_PreDsgMosWake(void);
static uint8_t NiuLogic_PreDsgMosHandle(void);
static void Niu_1WireUartHandle(void);


static void Toggle_Led(void)
{
	static uint8_t s = 0;
	if (s)
	{
		s = 0;
		MCU_GPIO_ClrBit(TN_LED_PORT, TN_LED_PIN);
#if (PROJECT_ID == 2)
		if(afeInfo.State_RT & 0x00C0)//过温、低温保护
		{
			if((afeInfo.MosState_RT & 0x20) == 0)
			{
				MCU_GPIO_ClrBit(TN_LED2_PORT, TN_LED2_PIN);
			}			
		}else
		{
			MCU_GPIO_SetBit(TN_LED2_PORT, TN_LED2_PIN);
		}
		
#endif
	}
	else
	{
		s = 1;
		MCU_GPIO_SetBit(TN_LED_PORT, TN_LED_PIN);
#if (PROJECT_ID == 2)
		if(afeInfo.State_RT & 0x00C0)//过温、低温保护
		{
			if((afeInfo.MosState_RT & 0x20) == 0)
			{
				MCU_GPIO_SetBit(TN_LED2_PORT, TN_LED2_PIN);
			}
		}		
#endif
	}
}

static void NiuLogic_PortInit()
{
    stc_gpio_cfg_t stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

	Gpio_Init(GpioPortA,GpioPin11 , &stcGpioCfg);

	//LED2 故障灯初始化
	stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

	Gpio_Init(GpioPortA,GpioPin5 , &stcGpioCfg);
	Gpio_SetIO(GpioPortA,GpioPin5);//LED2 灭
   
}

//通信选择判定 1-一线通
static uint8_t NiuLogic_CommTypeSel(void)
{
#if (ONEBUS_TYPE != 7)
	if (MCU_GPIO_GetBit(TN_ACC_PORT, TN_ACC_PIN))
		return 0;
	else
		return 1;
#else
	if (MCU_GPIO_GetBit(TN_ONE_RX_PORT, TN_ONE_RX_PIN))
		return 1;
	else
		return 0;
#endif
}

//flash 加载固化参数
static void LoadFlashVar(void)
{
	//设备序列号，通过flash编程自定义烧录，或者软件写入,16个byte
	Flash_Read(EE_START_ADDR + BAT_SN_ADDR_START, niuCommdTable.SN_ID, 16);
}

#if (MCU_LIB_SELECT == 1)
void PortA_IRQHandler(void)
{
	//中断退出继续跑
	SCB->SCR &= ~0x02;

	if (TRUE == Gpio_GetIrqStatus(TN_CHG_DET_PORT, TN_CHG_DET_PIN))
	{
		Gpio_ClearIrq(TN_CHG_DET_PORT, TN_CHG_DET_PIN);
		SEGGER_RTT_printf(0,"Event->CHG Pullup\n");
		flagIntWake |= 0x02;
	}

	if (TRUE == Gpio_GetIrqStatus(TN_ACC_PORT, TN_ACC_PIN))
	{
		Gpio_ClearIrq(TN_ACC_PORT, TN_ACC_PIN);
		SEGGER_RTT_printf(0,"Event->ACC Pullup\n");
		flagIntWake |= 0x01;
	}

	//天瑞项目，sw开关on
	if (TRUE == Gpio_GetIrqStatus(TN_LOAD_DET_PORT, TN_LOAD_DET_PIN))
	{
		Gpio_ClearIrq(TN_LOAD_DET_PORT, TN_LOAD_DET_PIN);
		bsp_StartAutoTimer(TMR_PRE_DELAY,60000);//1分钟计时
		switch_count = 0;
		SEGGER_RTT_printf(0,"Event->SWitch on\n");
		flagIntWake |= 0x20;
	}

	if (TRUE == Gpio_GetIrqStatus(TN_SHINT_PORT, TN_SHINT_PIN))
	{
		Gpio_ClearIrq(TN_SHINT_PORT, TN_SHINT_PIN);

		flagIntWake |= 0x04;
		afeInfo.MosState_RT |= 0x02;
		bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
	}
#if (USE_485_IF == 1)
	if (TRUE == Gpio_GetIrqStatus(TN_485_INT_PORT, TN_485_INT_PIN))
	{
		Gpio_ClearIrq(TN_485_INT_PORT, TN_485_INT_PIN);

		flagIntWake |= 0x01;
	}
#endif
}

static void GpioInterruptConfig()
{
	EnableNvic(RTC_IRQn, IrqLevel3, TRUE);
	Gpio_EnableIrq(TN_SHINT_PORT, TN_SHINT_PIN, GpioIrqRising); //通信中断
	//Gpio_EnableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling);	//sh309 中断
	Gpio_EnableIrq(TN_ACC_PORT, TN_ACC_PIN, GpioIrqFalling);		//ACC
	Gpio_EnableIrq(TN_CHG_DET_PORT, TN_CHG_DET_PIN, GpioIrqRising); //充电器接入
	Gpio_EnableIrq(GpioPortA,GpioPin11,GpioIrqRising); //Swich 开关接入
	bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
}

//唤醒中断
void PortB_IRQHandler(void)
{
	//中断退出继续跑
	SCB->SCR &= ~0x02;

	if (TRUE == Gpio_GetIrqStatus(TN_ALARM_PORT, TN_ALARM_PIN))
	{
		Gpio_ClearIrq(TN_ALARM_PORT, TN_ALARM_PIN);
		flagIntWake |= 0x10;
		SEGGER_RTT_printf(0,"Event->Sh309 ALARM interrupt\n");
	}

	if (TRUE == Gpio_GetIrqStatus(TN_SHINT_PORT, TN_SHINT_PIN))
	{
		Gpio_ClearIrq(TN_SHINT_PORT, TN_SHINT_PIN);
		//通信唤醒
		flagIntWake |= 0x04;
		bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
	}
}

static void System_EnterLowPower(void)
{
	SWITCH_LED_OFF();
	//休眠ADC处理
	ADC_DeConfig();

#if (USE_485_IF == 1)
	SWITCH_485_POWER_OFF
#endif
	//----------------------------------------------------------------

	
	if (afeInfo.CellVmin < SHIP_MODE_CELLV_TH) //低电压保护后
	{
		//sh309 进入进入ship 模式
		AFE_EN_SHIPMODE();
		afeInfo.MosState_RT &= ~0x60;
		//关闭预放电MOS
		SWITCH_PRED_OFF();
		afeInfo.MosState_RT &= ~0x80;

		//清iwdt
		EnableNvic(RTC_IRQn, IrqLevel3, TRUE);
		//EnableNvic(RTC_IRQn, IrqLevel3,FALSE);

		Gpio_DisableIrq(TN_ACC_PORT, TN_ACC_PIN, GpioIrqRising);	  //ACC
		Gpio_DisableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling); //sh309 中断
		Gpio_DisableIrq(TN_SHINT_PORT, TN_SHINT_PIN, GpioIrqRising);  //通信中断
		Gpio_EnableIrq(TN_LOAD_DET_PORT, TN_LOAD_DET_PIN, GpioIrqRising);
		//Gpio_DisableIrq(GpioPortA, GpioPin11, GpioIrqRising);  //switch 开关

#if (USE_485_IF == 1)
		Gpio_DisableIrq(TN_485_INT_PORT, TN_485_INT_PIN, GpioIrqFalling); //485外部唤醒
#endif

		//只应许充电器接入唤醒
		Gpio_EnableIrq(TN_CHG_DET_PORT, TN_CHG_DET_PIN, GpioIrqRising); //充电器接入
		EnableNvic(PORTA_IRQn, IrqLevel2, TRUE);
		EnableNvic(PORTB_IRQn, IrqLevel3, FALSE);
		flagIntEnterType = 0x02;
		SEGGER_RTT_printf(0,"Event->System goto ShipMode\n");
	}
	else
	{
		//前端芯片睡眠
		//AFE_SET_RUNMODE(0x01);

		EnableNvic(RTC_IRQn, IrqLevel3, TRUE);
		Gpio_EnableIrq(TN_SHINT_PORT, TN_SHINT_PIN, GpioIrqRising); //通信中断
#if (AFE_CHIP_SELECT != 3)
		Gpio_EnableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling);	//sh309 中断
#else
		Gpio_DisableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling);	//sh309 中断
#endif
		Gpio_EnableIrq(TN_CHG_DET_PORT, TN_CHG_DET_PIN, GpioIrqRising); //充电器接入
#if (PROJECT_ID == 2)
		if(TY_GETSWITCHSTA() == 1)
		{
			Gpio_EnableIrq(TN_ACC_PORT, TN_ACC_PIN, GpioIrqFalling);		//ACC
		}else
		{
			Gpio_DisableIrq(TN_ACC_PORT, TN_ACC_PIN, GpioIrqFalling);       //ACC 不能唤醒
		}
		Gpio_EnableIrq(TN_LOAD_DET_PORT, TN_LOAD_DET_PIN, GpioIrqRising);//SW on 接入
#endif

#if (USE_485_IF == 1)
		Gpio_EnableIrq(TN_485_INT_PORT, TN_485_INT_PIN, GpioIrqFalling); //485外部唤醒
#endif

		EnableNvic(PORTB_IRQn, IrqLevel3, TRUE);
		EnableNvic(PORTA_IRQn, IrqLevel2, TRUE);
		flagIntEnterType = 0x01;
		SEGGER_RTT_printf(0,"Event->System goto sleep\n");
	}
	Lpm_GotoDeepSleep(TRUE);
}

static void System_ExitLowPower(void)
{
	//Mcu_PeripherWakeFromSleep();
	//唤醒之后，点亮LED
	SWITCH_LED_ON();

#if (USE_485_IF == 1)
	SWITCH_485_POWER_ON
#endif
	ADC_PortInit();
	ADC_Config();
	//根据进入中断的类型,处理唤醒之后的操作
	if (flagIntEnterType == 2)
	{
		SEGGER_RTT_printf(0,"Event->System wakeup from shipmode\n");
		//深度休眠唤醒
		//SH_DISABLE_SHIPMODE();
		AFE_DIS_SHIPMODE();
		bsp_DelayMS(500);
	}
	else
	{
		SEGGER_RTT_printf(0,"Event->System wakeup from sleep\n");
		//Sh_SetRunMode(0x00);
		//AFE_SET_RUNMODE(0x00);
	}
	//Gpio_DisableIrq(TN_SHINT_PORT, TN_SHINT_PIN, GpioIrqRising);	//通信中断
	Gpio_DisableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling); //ALARM中断
#if (USE_485_IF == 1)
	Gpio_DisableIrq(TN_485_INT_PORT, TN_485_INT_PIN, GpioIrqFalling); //485外部唤醒
#endif
}

#elif (MCU_LIB_SELECT == 2)
//外部中断0，充电激活唤醒
void EXTI0_IRQHandler(void)
{
	if (RESET != EXTI_GetITStatus(EXTI_LINE0))
	{

		flagIntWake |= 0x02;
		EXTI_ClrITPendBit(EXTI_LINE0);
		SEGGER_RTT_printf(0,"Event->CHG Pullup\n");
	}
}

//TN_WAKE_UP -PA1 系统接入中断
void EXTI1_IRQHandler(void)
{
	if (RESET != EXTI_GetITStatus(EXTI_LINE1))
	{
		afeInfo.MosState_RT |= 0x80; //设置预放电MOS打开
		//系统接入
		afeInfo.MosState_RT |= 0x10;

		EXTI_ClrITPendBit(EXTI_LINE1);
	}
}

//ACC 唤醒中断
void EXTI2_IRQHandler(void)
{
	if (RESET != EXTI_GetITStatus(EXTI_LINE2))
	{

		flagIntWake |= 0x01;

		EXTI_ClrITPendBit(EXTI_LINE2);
		SEGGER_RTT_printf(0,"Event->ACC Pullup\n");
	}
}

//通信中断唤醒 PA10
void EXTI9_5_IRQHandler(void)
{
	if (RESET != EXTI_GetITStatus(EXTI_LINE6))
	{
		EXTI_ClrITPendBit(EXTI_LINE6);
		flagIntWake |= 0x04;
		afeInfo.MosState_RT |= 0x02;
		bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
	}
}

//alarm 中断 SHINT PA10
void EXTI15_10_IRQHandler(void)
{
	if (RESET != EXTI_GetITStatus(EXTI_LINE12))
	{
		flagIntWake |= 0x10;
		EXTI_ClrITPendBit(EXTI_LINE12);
		SEGGER_RTT_printf(0,"Event->Sh309 ALARM interrupt\n");
	}

	if (RESET != EXTI_GetITStatus(EXTI_LINE11)) //load det
	{
		flagIntWake |= 0x20;
		EXTI_ClrITPendBit(EXTI_LINE11);
		SEGGER_RTT_printf(0,"Event->Load Det interrupt\n");
	}

	if (RESET != EXTI_GetITStatus(EXTI_LINE10))//shint pa10
	{
		EXTI_ClrITPendBit(EXTI_LINE10);
		flagIntWake |= 0x04;
		afeInfo.MosState_RT |= 0x02;
		//SEGGER_RTT_printf(0,"Event->rx Det interrupt\n");
		bsp_StartTimer(TMR_COMM_INT_DELAY, 5000);
	}
}

static void GpioInterruptConfig()
{
	GPIO_InitType GPIO_InitStructure;
	EXTI_InitType EXTI_InitStructure;
	NVIC_InitType NVIC_InitStructure;

	//TN_WAKE_UP - PA01，TN_ACC-PA02，TN_CHG_DET-PA0，TN_SHINT-PB06

	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

	//初始化管脚为浮动输入模式
	GPIO_InitStruct(&GPIO_InitStructure);
	GPIO_InitStructure.Pin = TN_WAKE_UP_PIN | TN_ACC_PIN |TN_LOAD_DET_PIN|TN_SHINT_PIN|TN_CHG_DET_PIN;
	GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input; //
	GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = TN_ALARM_PIN;
	GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

	// GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
	// GPIO_InitStructure.Pin = TN_SHINT_PIN|TN_CHG_DET_PIN;
	// GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

	//配置中断线
	GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE0);  //chg-det
	GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE1);  //TN_WAKE_UP
	GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE2);  //PA02，TN_ACC
	GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE10);  //PB06 SHINT
	//GPIO_ConfigEXTILine(GPIOB_PORT_SOURCE, GPIO_PIN_SOURCE12); //PB12 ALARM
	GPIO_ConfigEXTILine(GPIOA_PORT_SOURCE, GPIO_PIN_SOURCE11); //PA11 LOAD DET
	
	EXTI_InitStructure.EXTI_Line = EXTI_LINE0; //chg-det
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE1; //TN_WAKE_UP
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE2; //TN_ACC
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE10; //SHINT
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	// EXTI_InitStructure.EXTI_Line = EXTI_LINE12; //ALARM
	// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
	// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	// EXTI_InitPeripheral(&EXTI_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_LINE11; //LOAD DET
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitPeripheral(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

static void System_EnterLowPower(void)
{

	SWITCH_LED_OFF();
	ADC_DeConfig();
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
		NVIC_EnableIRQ(RTC_IRQn);

		//禁止TN_WAKE_UP，ACC，SH_INT,ALARM
		NVIC_DisableIRQ(EXTI1_IRQn);   //禁止TN_WAKE_UP
		NVIC_DisableIRQ(EXTI2_IRQn);   //ACC
		NVIC_DisableIRQ(EXTI15_10_IRQn); //通信中断
#if (USE_485_IF == 1)
		//NVIC_DisableIRQ(); //485外部唤醒
#endif
		//只应许充电器接入唤醒
		NVIC_EnableIRQ(EXTI0_IRQn);
		flagIntEnterType = 0x02;
		SEGGER_RTT_printf(0,"Event->System goto ShipMode\n");
	}
	else
	{
		//前端芯片睡眠
		//Sh_SetRunMode(0x01);//ALARM 可以唤醒
		//AFE_SET_RUNMODE(0x01);

		NVIC_EnableIRQ(RTC_IRQn);

		//禁止TN_WAKE_UP，ACC，SH_INT,ALARM
		NVIC_EnableIRQ(EXTI1_IRQn);		//TN_WAKE_UP
		NVIC_EnableIRQ(EXTI2_IRQn);		//ACC
		//NVIC_EnableIRQ(EXTI9_5_IRQn);	//通信中断
#if (AFE_CHIP_SELECT != 3)		
		NVIC_EnableIRQ(EXTI15_10_IRQn); //ALARM 中断
#else
		NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif
		//只应许充电器接入唤醒
		NVIC_EnableIRQ(EXTI0_IRQn);
		flagIntEnterType = 0x01;
		SEGGER_RTT_printf(0,"Event->System goto sleep\n");
	}
	//禁止systick中断
	SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk ; 
	//MCU ,进入Stop2 模式
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);

	PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI, PWR_CTRL3_RAM1RET);
}

static void System_ExitLowPower(void)
{
	//唤醒之后，点亮LED
	SWITCH_LED_ON();
	
	//开启systick中断
	SysTick->CTRL  |= SysTick_CTRL_TICKINT_Msk ; 
#if (USE_485_IF == 1)
	SWITCH_485_POWER_ON
#endif

	ADC_PortInit();
	ADC_Config();
	Niu_ModbufFifoClr();
	//根据进入中断的类型,处理唤醒之后的操作
	if (flagIntEnterType == 2)
	{
		SEGGER_RTT_printf(0,"Event->System wakeup from shipmode\n");

		//深度休眠唤醒
		//SH_DISABLE_SHIPMODE();
		AFE_DIS_SHIPMODE();
		bsp_DelayMS(500);
	}
	else
	{
		SEGGER_RTT_printf(0,"Event->System wakeup from sleep\n");
		//Sh_SetRunMode(0x00);
	}
#if (USE_485_IF == 1)
	//Gpio_DisableIrq(TN_485_INT_PORT, TN_485_INT_PIN, GpioIrqFalling); //485外部唤醒
#endif
	
}

#endif

static void NiuLogic_Sleep(void)
{
}

static void NiuLogic_ThreeFulseProtect(void)
{
	//mosfet 失效
	if (afeInfo.State_RT & 0x0800)
	{
		//继续高压充电
		if (afeInfo.CellVmax > CELL_VMAX_OVER_TH)
		{

			//必须开放电MOS，才能烧毁
			//Sh_OpenDsgMos();
			AFE_OPEN_DMOS();
			/*MCU_GpioKillMeInit();
			SWITCH_KILLME_ON();*/
		}
	}
}

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
	
	static uint8_t flagsw = 0;
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
	if (GET_WAKEUP() == 1)
	{
		afeInfo.MosState_RT &= ~0x10;
	}

#if (TYPROJECT_ID != 3)
	//--DMOS 状态检测
	if(GET_DMOS_FB() == 1)
	{
		afeInfo.MosState_RT &= ~0x20;
	}else
	{
		afeInfo.MosState_RT |= 0x20;
	}

	//--CMOS状态
	if(GET_CMOS_FB() == 1)
	{
		afeInfo.MosState_RT &= ~0x40;
	}else
	{
		afeInfo.MosState_RT |= 0x40;
	}
#else
	afeInfo.State_RT &= (~0x3000);

#endif	



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
	//switch on
	if(TY_GETSWITCHSTA() == 1)
	{
		// //SEGGER_RTT_printf(0,"SWitch state is on\n");
		if(bsp_CheckTimer(TMR_PRE_DELAY))
		{
			switch_count++;
			if(switch_count>60)//测试时用5分钟，5，量产化后60分钟
			{
				//一个小时后清除，可以休眠
				flagsw = 0;
				//switch_count = 0;
				//bsp_stop_timer(TMR_PRE_DELAY);
			}else
			{
				flagsw = 1;
			}
		}else
		if(switch_count < 60 )
		{
			flagsw = 1;
		}
	}else
	{
		flagsw = 0;
		switch_count = 0;
		//强制acc disable
		afeInfo.MosState_RT &= ~0x08;
		//强制关闭通信唤醒
		afeInfo.MosState_RT &= ~0x02;
		//强制关闭充放电状态
		afeInfo.State_RT &= ~0x03;

	}			
	
	//充电活动中、ACC 上拉、有通信唤醒， 不进入休眠
	if ((afeInfo.State_RT & 0x03) || (afeInfo.MosState_RT & 0x0a) != 0 || (flagsw == 1))
	{
		//不是故障状态,OV后，可以放电
		if(afeInfo.State_RT & 0x03 )
		{
			if ((afeInfo.MosState_RT & 0x60) != 0x60) //放电MOS没有打开
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
		}else
		if((afeInfo.MosState_RT & 0x08) || (flagsw == 1))		
		{
			//ACC 上拉
			if((afeInfo.State_RT == 0)||(afeInfo.State_RT == 4))
			{
				//确保放电MOS打开
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
			if(Rtc_GetWakeUpFlag())
			{
				Rtc_ClrWakeUpFlag();
			}
			return;
		}
	}
	

	if(flagsw == 1)
	{
		if(Rtc_GetWakeUpFlag())
		{
			Rtc_ClrWakeUpFlag();
		}
		return;
	}
	

	//如果负载锁定状态，查看负载是否释放，负载释放解决预放电锁定
	// if(afeInfo.Pre_State & 0x01)
	// {
	// 	if(GET_LOAD_DET() == 1)
	// 	{
	// 		afeInfo.Pre_State &= ~0x01;
	// 	}
	// }


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
			if (afeInfo.State_RT < 0x0004)
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
			bsp_StartTimer(TMR_NIULOGIC_20S, 300000);//测试版本5分钟
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
	//if (Rtc_GetWakeUpFlag() == 0x01) //去除20s延时休眠功能
	if ((flagsw==0) || bsp_CheckTimer(TMR_NIULOGIC_20S) || Rtc_GetWakeUpFlag() == 0x01) //唤醒20s 时间超时,RTC 唤醒之后，最多跑一个循环就可以
	{

		RTC_ConfigInit();
		if (Rtc_GetWakeUpFlag())
		{ //清楚RTC中断唤醒标志
			Rtc_ClrWakeUpFlag();
			//20s rtc 定时中断
		}else{
			SEGGER_RTT_printf(0,"Event->20s Timeout\n");
		}

		//进入休眠前的清除计时开启标志
		flagStarted20s = 0;

		System_EnterLowPower();
		//-----------------------唤醒分界线---------------------//
		System_ExitLowPower();
		SEGGER_RTT_printf(0,"Event Wake->flagIntWake %02x\n",flagIntWake);
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
				afeInfo.MosState_RT &= ~0x80;

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
			if((afeInfo.State_RT == 0)|| (afeInfo.State_RT & 0x08))
			{
				//Sh_OpenChgMos();
				AFE_OPEN_DMOS();
				AFE_OPEN_CMOS();
				afeInfo.MosState_RT |= 0x60; //打开充放电MOS
				//充电器接入
				afeInfo.MosState_RT |= 0x04;
				SEGGER_RTT_printf(0,"Event->charger start charge\n");
			}			
		}
		//串口通信中断
		if (flagIntWake & 0x04)
		{
			flagIntWake &= ~0x04;
		}
		//唤醒之后预放电处理
		//NiuLogic_PreDsgMosWake();
		//唤醒之后，需要马上采样数据
		Afe_SetInitSample();
	}
}





static void Niu_1WireUartHandle(void)
{
	//选择单线串口协议需要处理的逻辑
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

	//校准参数默认初始化
	Flash_Read(EE_START_ADDR + CALI_CURR_PARA_VAL, (uint8_t*)&niuCommdTable.Ratio_KvH, 8);
	if(niuCommdTable.Ratio_KvH == 0xFF || \
		niuCommdTable.Ratio_KvL == 0xFF
	
	)
	{
		niuCommdTable.Ratio_KvH = (uint8_t)(DEFAULT_RATIO_V_K>>8);
		niuCommdTable.Ratio_KvL = (uint8_t)(DEFAULT_RATIO_V_K);
		niuCommdTable.Ratio_OffsetvH = (uint8_t)(DEFAULT_RATIO_V_O>>8);
		niuCommdTable.Ratio_OffsetvL = (uint8_t)(DEFAULT_RATIO_V_O);
		niuCommdTable.Ratio_KcH = (uint8_t)(DEFAULT_RATIO_C_K>>8);
		niuCommdTable.Ratio_KcL = (uint8_t)(DEFAULT_RATIO_C_K);
		niuCommdTable.Ratio_OffsetcH = (uint8_t)(DEFAULT_RATIO_C_O>>8);
		niuCommdTable.Ratio_OffsetcL = (uint8_t)(DEFAULT_RATIO_C_O);
	}

	tempK = ((uint16_t)niuCommdTable.Ratio_KcH<<8) +niuCommdTable.Ratio_KcL;
	tempO = ((uint16_t)niuCommdTable.Ratio_OffsetcH<<8) +niuCommdTable.Ratio_OffsetcL;

	tempKv = ((uint16_t)niuCommdTable.Ratio_KvH<<8) +niuCommdTable.Ratio_KvL;;
	tempKvO = ((uint16_t)niuCommdTable.Ratio_OffsetvH<<8) +niuCommdTable.Ratio_OffsetvL;
	Sh_SetCurrCarlibation(tempK,tempO);
	Sh_SetVoltCarlibation(tempKv,tempKvO);

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

		if (*((uint8_t *)NIU_FW_VER + i) != 0xFF)
		{
			niuCommdTable.H_Ver_N[i] = *((uint8_t *)NIU_HW_VER + i);
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
	int32_t heart = bsp_GetRunTime() / 1000;
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
#if(PROJECT_ID == 2)
	Uart0Init4TY(TY_ModbusRecvHandle);
#else
	Uart0Init(NIU_ModbusRecvHandle);
#endif
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
#if (ONEBUS_TYPE == 1) //小牛
	Niu_OneBusInit();
#elif (ONEBUS_TYPE == 2)
	Tn_OneBusInit();
#elif (ONEBUS_TYPE == 3)
	Yd_OneBusInit();
#elif (ONEBUS_TYPE == 4)
	Xr_OneBusInit();
#elif (ONEBUS_TYPE == 7)
	Zb_OneBusInit();
#elif (ONEBUS_TYPE == 0)
#endif

	bsp_StartAutoTimer(TMR_MAIN, 1000);
	bsp_StartAutoTimer(TMR_PROTECT_DELAY,1000);
	bsp_StartAutoTimer(TMR_PRE_DELAY,60000);//switch on 逻辑
}

void NiuLogicRun(void)
{
	if (bsp_CheckTimer(TMR_MAIN))
	{
		Toggle_Led();
	}
	//前端信息采集和mcu采集数据
	AFE_Process();
	//soc，soh等算法实现
	AlgEngineProcess();
	//同步数据
	Niu_TableSyncHandle();
#if(PROJECT_ID == 2)	
	Ty_TableUpdate();
#endif
	//一线通逻辑切换
	if (NiuLogic_CommTypeSel())
	{
#if (USE_485_IF != 1)
		//Disable Uart 接收中断
		DISABLE_UART_RXINT();
//一线通发送数据
#if (ONEBUS_TYPE == 1) //小牛
		Niu_OneBusProcess();
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
		ENABLE_UART_RXINT();
#endif
	}
	else
	{
	}
	//均衡控制处理
	NiuLogic_BalanceHanle();
	//MOS失效之后，三端保险紧急保护措施
	NiuLogic_ThreeFulseProtect();

	//软件保护控制
	NiuLogic_Protect();
	//充放电MOS、和预放电MOS 和休眠进入逻辑处理
	NiuLogic_MosHanle();
}

#endif
