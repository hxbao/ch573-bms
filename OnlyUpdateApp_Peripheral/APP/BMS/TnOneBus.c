/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2020/10/16		 Preliminary
********************************************************************************/

#include "includes.h"
#if(ONEBUS_TYPE == 2)

#define ONEBUS_SEND_MODE     1          //发送模式  0-阻塞   1-定时触发
#if(ONEBUS_SEND_MODE == 0)
static void Tn_OneBusSendSyncStart(void);
static void Tn_OneBusSendStop(void);
static void Tn_OneBusSendBit1(void);
static void Tn_OneBusSendBit0(void);
static void Tn_OneBusSendByte(uint8_t data);
#else
static void Tn_OneBusTimerInit();
static void Tn_OneBusTimerStart();
static void Tn_OneBusTimerStop();
static void Tn_OneBusSetDelay(uint16_t dly);

#endif

static void Tn_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);



typedef struct TnOneBus
{

    uint8_t AIMA_id;
  
    uint8_t status;
    uint8_t soc;
    uint8_t cycleCountL;
    uint8_t cycleCountH;
    int8_t temperature;

    uint8_t CellhighVoltL; //最高电芯电压mv 低字节
    uint8_t CellHighVoltH; //最高电芯电压mv 高字节
    uint8_t CellLowVoltL;  //最低电芯电压mv 低字节
    uint8_t CellLowVoltH;  //最低电芯电压mv 高字节
    uint8_t MaxAllowMotoChargeCurrent;//最大应许回馈电流 A
    uint8_t MaxAllowChargeCurrent;//最大应许充电电流 A
    uint8_t MaxAllowChargeVoltL;//最大应许充电电压
    uint8_t MaxAllowChargeVoltH;//最大应许充电电压
    uint8_t sum;
} OneBusDataStruct_t;


typedef struct OneBusBitSendCfg
{
    uint16_t syncLDelayTime;   //同步头阶段低电平需要的延时us
    uint16_t syncHDelayTime;   //同步头阶段高电平需要的延时us
    uint16_t Data1LDelayTime;    //数据低电平需要延时的us数
    uint16_t Data1HDelayTime;    //数据高电平需要延时的us数
    uint16_t Data0LDelayTime;    //数据低电平需要延时的us数
    uint16_t Data0HDelayTime;    //数据高电平需要延时的us数
    uint16_t StopLDelayTime;    //Stop低电平需要延时的us数
}OneBusBitSendCfg_t;

typedef struct OneBusBitSendSta
{
    uint8_t *buf;
    uint8_t bitSize;
    uint8_t bitPhase;        //发送bit的当前阶段，1-同步头低电平阶段    2-同步头高电平阶段       3 数据低电平阶段   4 数据高电平  5 -Stop低阶段   6-stop高电平
    uint16_t bitIndex;       //bit f发送的索引

}OneBusBitSendSta_t;

#if(ONEBUS_SEND_MODE == 1)
OneBusBitSendCfg_t OneBusSendCfg;
OneBusBitSendSta_t OneBusSta;
uint8_t byteIndex;
uint8_t bitIndex;
uint8_t curData;
#endif


OneBusDataStruct_t OneBusData;

/**
 * @brief 一线通IO初始化
 *
 */
void Tn_OneBusInit(void)
{
#if (ONEBUS_SEND_MODE == 1)
    Tn_OneBusTimerInit();
#endif
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,1000);
}


void Tn_OneBusProcess(void)
{
    uint16_t tempval = 0;
    //uint8_t sh_sta;
    uint8_t onebus_sta = 0;
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {

	//充电MOS状态
	onebus_sta |= (afeInfo.MosState_RT &0x40)<<1;
	//放电MOS状态
	onebus_sta |= (afeInfo.MosState_RT &0x20)<<1;
	//预放电MOS状态
	onebus_sta |= (afeInfo.MosState_RT &0x80)>>2;
	//充电允许
	onebus_sta |= 0x01;

        OneBusData.status = onebus_sta;// 0xc0;//onebus_sta;
        OneBusData.soc = (uint8_t) (algEnginer.soc_r/10);//0x5b;//(uint8_t) (batinfo.soc/10);
        OneBusData.cycleCountL = (uint8_t)(algEnginer.cycCount);//0x0100;//batinfo.cycles;
        OneBusData.cycleCountH = (uint8_t)(algEnginer.cycCount>>8);
        OneBusData.temperature = (int8_t)(afeInfo.ShTemp[0] - 2731);//0x0c;//(batinfo.Temp-2731);
        OneBusData.sum = 0;//0x28;//0;

        OneBusData.AIMA_id = 0x3A;

        OneBusData.CellHighVoltH = (uint8_t)(afeInfo.CellVmax>>8); //最高电芯电压低字节
        OneBusData.CellhighVoltL = (uint8_t)(afeInfo.CellVmax); //最高电芯电压高字节

        OneBusData.CellLowVoltL = (uint8_t)(afeInfo.CellVmin);  //最低电芯电压低字节
        OneBusData.CellLowVoltH = (uint8_t)(afeInfo.CellVmin>>8);  //最低电芯电压高字节
        OneBusData.MaxAllowMotoChargeCurrent = DEFAULT_BACKCHG_CURR; //最大允许回馈电流
        OneBusData.MaxAllowChargeCurrent = DEFAULT_CHARG_DEMAND_CURR;     //最大允许充电电流
        OneBusData.MaxAllowChargeVoltL = (uint8_t)DEFAULT_CHARG_DEMAND_VOLT;       //最大允许充电电压
        OneBusData.MaxAllowChargeVoltH = (uint8_t)(DEFAULT_CHARG_DEMAND_VOLT>>8);

        for (int i = 0; i < sizeof(OneBusDataStruct_t) - 1; i++)
        {
            OneBusData.sum += *((uint8_t *)(&OneBusData) + i);
        }

        Tn_OneBusSendData((uint8_t *)(&OneBusData), sizeof(OneBusDataStruct_t));
    }
}

/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
static void Tn_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{

#if(ONEBUS_SEND_MODE == 0)
    uint16_t i;
    Tn_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Tn_OneBusSendByte(*(sndBuf+i));
    }

    Tn_OneBusSendStop();
#elif(ONEBUS_SEND_MODE == 1)

    OneBusSendCfg.syncLDelayTime = 40000;   //同步头阶段低电平需要的延时us
    OneBusSendCfg.syncHDelayTime = 2000;   //同步头阶段高电平需要的延时us
    OneBusSendCfg.Data1LDelayTime = 2000;    //数据低电平需要延时的us数
    OneBusSendCfg.Data1HDelayTime = 4000;    //数据高电平需要延时的us数
    OneBusSendCfg.Data0LDelayTime = 4000;    //数据低电平需要延时的us数
    OneBusSendCfg.Data0HDelayTime = 2000;    //数据高电平需要延时的us数
    OneBusSendCfg.StopLDelayTime = 10000;    //Stop低电平需要延时的us数

    OneBusSta.bitPhase = 1;
    OneBusSta.bitIndex = 0;
    OneBusSta.bitSize = dataLen*8;
    OneBusSta.buf = sndBuf;
    Tn_OneBusTimerStart();

#endif

}



#if(ONEBUS_SEND_MODE == 0)
static void Tn_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(40000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Tn_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(10000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);

}

static void Tn_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
}

static void Tn_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Tn_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x80)
        {
            Tn_OneBusSendBit1();
        }else
        {
            Tn_OneBusSendBit0();
        }
        data <<=1;
    }
}


#else

static void Tn_OneBusTimerInit()
{
    //Fsys = 60Mhz,
    //端口重新配置成IO
    GPIOA_ModeCfg(TN_ONE_TX_PAPIN, GPIO_ModeOut_PP_20mA);
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
    PFIC_EnableIRQ(TMR0_IRQn);
    //TMR0_CountOverflowCfg(cyc);
}

static void Tn_OneBusTimerStart()
{
    //1us 后马上开始
    TMR0_TimerInit(FREQ_SYS / 1000000);//us
    TMR0_Enable();
}

static void Tn_OneBusTimerStop()
{
    TMR0_Disable();
}

static void Tn_OneBusSetDelay(uint16_t dly)
{
    TMR0_TimerInit(dly*FREQ_SYS / 1000000);//us
}

/*********************************************************************
 * @fn      TMR0_IRQHandler
 *
 * @brief   TMR0中断函数
 *
 * @return  none
 */
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR0_IRQHandler(void) // TMR0 定时中断
{

    if(TMR0_GetITFlag(TMR0_3_IT_CYC_END))
    {
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志
        //GPIOA_InverseBits(TN_LED_PAPIN);

        //GPIOB_InverseBits(GPIO_Pin_15);
        switch(OneBusSta.bitPhase)
        {
            //1-同步头低电平阶段    2-同步头高电平阶段       3 数据低电平阶段   4 数据高电平  5 -Stop低阶段   6-stop高电平
            case 1://
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Tn_OneBusSetDelay(OneBusSendCfg.syncLDelayTime);

                OneBusSta.bitPhase = 2;
                break;
            case 2:
                GPIOA_SetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Tn_OneBusSetDelay(OneBusSendCfg.syncHDelayTime);
                OneBusSta.bitPhase = 3;
                break;
            case 3:
                byteIndex = OneBusSta.bitIndex/8;
                bitIndex = 7-OneBusSta.bitIndex%8;
                curData = *(OneBusSta.buf+byteIndex);
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                if(curData & ((uint8_t)0x01 << bitIndex))
                {
                    //设置延迟
                    Tn_OneBusSetDelay(OneBusSendCfg.Data1LDelayTime);
                }else {
                    //设置延迟
                    Tn_OneBusSetDelay(OneBusSendCfg.Data0LDelayTime);
                }
                OneBusSta.bitPhase = 4;

                break;
            case 4:

//                if(OneBusSta.bitIndex++ == OneBusSta.bitSize)
//                {
//                    OneBusSta.bitPhase = 5;
//                }else {
//                    GPIOA_SetBits(TN_ONE_TX_PAPIN);
//                    if(curData & ((uint8_t)0x01 << bitIndex))
//                    {
//                        Tn_OneBusSetDelay(OneBusSendCfg.Data1HDelayTime); //设置延迟
//                        //设置延迟
//                    }else {
//                        Tn_OneBusSetDelay(OneBusSendCfg.Data0HDelayTime); //设置延迟
//                    }
//                    OneBusSta.bitPhase = 3;
//                }

              GPIOA_SetBits(TN_ONE_TX_PAPIN);
	      if(curData & ((uint8_t)0x01 << bitIndex))
	      {
		  Tn_OneBusSetDelay(OneBusSendCfg.Data1HDelayTime); //设置延迟
		  //设置延迟
	      }else {
		  Tn_OneBusSetDelay(OneBusSendCfg.Data0HDelayTime); //设置延迟
	      }

	      if((OneBusSta.bitIndex++) == OneBusSta.bitSize)
	      {
		  OneBusSta.bitPhase = 5;
	      }else {
		  OneBusSta.bitPhase = 3;
	      }

                break;
            case 5:
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Tn_OneBusSetDelay(OneBusSendCfg.StopLDelayTime);
                OneBusSta.bitPhase = 6;
                break;
            case 6:
                GPIOA_SetBits(TN_ONE_TX_PAPIN);
                OneBusSta.bitPhase = 0;
                Tn_OneBusTimerStop();
                break;
        }
    }
}

#endif
#endif
