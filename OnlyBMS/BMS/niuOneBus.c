/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"
#if(ONEBUS_TYPE == 1)


#define ONEBUS_SEND_MODE     1          //发送模式  0-阻塞   1-定时触发

#if(ONEBUS_SEND_MODE == 0)
static void Niu_OneBusSendSyncStart(void);
static void Niu_OneBusSendStop(void);
static void Niu_OneBusSendBit1(void);
static void Niu_OneBusSendBit0(void);
static void Niu_OneBusSendByte(uint8_t data);
#else
static void Niu_OneBusTimerInit();
static void Niu_OneBusTimerStart();
static void Niu_OneBusTimerStop();
static void Niu_OneBusSetDelay(uint16_t dly);
#endif

typedef struct NiuOneBus1
{
    uint8_t pid;
    uint8_t soc;
    uint8_t tvoltH;
    uint8_t tvoltL;
    uint8_t mosSta;
    uint8_t sum;
} OneBusDataStruct1_t;

typedef struct NiuOneBus2
{
    uint8_t pid;
    uint8_t iCurrH;
    uint8_t iCurrL;
    uint8_t celltemp;
    uint8_t mostemp;
    uint8_t sum;
} OneBusDataStruct2_t;

typedef struct NiuOneBus3
{
    uint8_t  pid;
    uint8_t  errStaH;
    uint8_t  errStaL;
    uint8_t  cycleCountH;
    uint8_t  cycleCountL;
    uint8_t sum;
} OneBusDataStruct3_t;

typedef struct NiuOneBus4
{
    uint8_t pid;
    uint8_t heartbeat;
    uint8_t chgReqireCurrL;
    uint8_t chgReqireVoltH;
    uint8_t chgReqireVoltL;
    uint8_t sum;
} OneBusDataStruct4_t;


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


OneBusDataStruct1_t OneBusData1;
OneBusDataStruct2_t OneBusData2;
OneBusDataStruct3_t OneBusData3;
OneBusDataStruct4_t OneBusData4;

#if(ONEBUS_SEND_MODE == 1)
OneBusBitSendCfg_t OneBusSendCfg;
OneBusBitSendSta_t OneBusSta;
#endif

/**
 * @brief 一线通IO初始化
 * 
 */
void Niu_OneBusInit(void)
{
    Niu_OneBusTimerInit();
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,500);
}


void Niu_OneBusProcess(void)
{
    uint8_t select[4] = {3,2,1,0}; 
    static uint8_t buseIndex = 0;
    static uint8_t heartbeat = 0;
    uint8_t mosSta = 0;
    uint16_t errCode = 0;
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        switch( select[buseIndex++])
        {
            case 0:
                if(niuCommdTable.MOS_Status & 0x20)
                {
                    mosSta |= 0x01; //D-MOS 导通
                }

                if(niuCommdTable.MOS_Status & 0x40)
                {
                    mosSta |= 0x02; //C-MOS 导通
                }

                if(niuCommdTable.MOS_Status & 0x10)
                {
                    mosSta |= 0x04; //系统 接入
                }

                if(niuCommdTable.MOS_Status & 0x08)
                {
                    mosSta |= 0x08; //ACC 接入
                }

                if(niuCommdTable.MOS_Status & 0x04)
                {
                    mosSta |= 0x10; //充电器接入
                }

                OneBusData1.pid = 0;
                OneBusData1.soc = (uint8_t)(algEnginer.soc_display/10);
                OneBusData1.tvoltH = (uint8_t)((uint16_t)(afeInfo.SumBatteryPackVolt/100)>>8);
                OneBusData1.tvoltL = (uint8_t)((uint16_t)(afeInfo.SumBatteryPackVolt/100));
                OneBusData1.mosSta = mosSta;
                OneBusData1.sum = OneBusData1.pid + OneBusData1.soc+OneBusData1.tvoltH+OneBusData1.tvoltL+OneBusData1.mosSta;

                Niu_OneBusSendData((uint8_t *)(&OneBusData1), sizeof(OneBusDataStruct1_t));    
            break;

            case 1:
                OneBusData2.pid = 1;
                OneBusData2.celltemp = (uint8_t)((afeInfo.ShTemp[0] - 2731)/10+40);//正偏移40°
                if(afeInfo.State_RT & 0x0001)//放电
                {
                    OneBusData2.iCurrH = (uint8_t)(int16_t)(((0-afeInfo.DsgCurrent)/100) >>8);
                    OneBusData2.iCurrL = (uint8_t)(int16_t)((0-afeInfo.DsgCurrent)/100);
                }else
                if(afeInfo.State_RT & 0x0002)//充电
                {
                    OneBusData2.iCurrH = (uint8_t)(afeInfo.ChgCurrent/100 >>8);
                    OneBusData2.iCurrL = (uint8_t)(afeInfo.ChgCurrent/100);
                }else
                {
                    OneBusData2.iCurrH = 0;
                    OneBusData2.iCurrL = 0;
                }
                
                OneBusData2.mostemp = (uint8_t)((afeInfo.MosTemp - 2731)/10+40);//正偏移40°

                OneBusData2.sum = OneBusData2.pid+OneBusData2.celltemp+OneBusData2.iCurrH+OneBusData2.iCurrL+OneBusData2.mostemp ;
                Niu_OneBusSendData((uint8_t *)(&OneBusData2), sizeof(OneBusDataStruct2_t));
            break;

            case 2:
                //C MOS故障
                if(afeInfo.State_RT & 0x1000)
                {
                    errCode |= 0x02;
                }

                //D MOS 故障
                if(afeInfo.State_RT & 0x2000)
                {
                    errCode |= 0x01;
                }
                //MOS 高温报警
                if((afeInfo.MosTemp) > (MOSOTP_W *10+2731))
                {
                    errCode |= 0x04;
                }

                //放电高温报警
                if(afeInfo.State_RT2 & 0x0008 )
                {
                    errCode |= 0x08;
                }

                //放电低温报警
                if(afeInfo.State_RT2 & 0x0004)
                {
                    errCode |= 0x10;
                }

                //充电高温报警
                if(afeInfo.State_RT2 & 0x0002 )
                {
                    errCode |= 0x20;
                }
                //充电低温报警
                if(afeInfo.State_RT2 & 0x0001)
                {
                    errCode |= 0x40;
                }
                //总压过压
                if(afeInfo.SumBatteryPackVolt > CELL_NUMS*niuCommdTable.OC_Vlt_P*100)
                {
                    errCode |= 0x80;
                }
                //总压低压
                if(afeInfo.SumBatteryPackVolt < CELL_NUMS*niuCommdTable.Dc_Vlt_P*100)
                {
                    errCode |= 0x100;
                }
                //充电过流
                if(afeInfo.State_RT & 0x0010)
                {
                    errCode |= 0x200;
                }
                //放电过流
                if(afeInfo.State_RT & 0x0020)
                {
                    errCode |= 0x400;
                }

                //不均衡
                if(afeInfo.CellVdiff > VDIFF_WARN)
                {
                    errCode |= 0x800;
                }

                //短路保护
                if(afeInfo.AFE_State_RT & 0x0200)
                {
                    errCode |= 0x1000;
                }

                OneBusData3.pid = 2;
                OneBusData3.errStaH     = (uint8_t)(errCode>>8);
                OneBusData3.errStaL     = (uint8_t)errCode;
                OneBusData3.cycleCountH = (uint8_t)(algEnginer.cycCount>>8);
                OneBusData3.cycleCountL = (uint8_t)(algEnginer.cycCount);
                OneBusData3.sum         = OneBusData3.pid + OneBusData3.errStaH+OneBusData3.errStaL+OneBusData3.cycleCountH+OneBusData3.cycleCountL ;
                Niu_OneBusSendData((uint8_t *)(&OneBusData3), sizeof(OneBusDataStruct3_t));
            break;

            case 3:
            OneBusData4.pid = 3;
            OneBusData4.heartbeat = heartbeat;
            if((niuCommdTable.ChargDemandCurrent[0])/10 & 0x01)
            {
                OneBusData4.heartbeat |= 0x80;
            }
            OneBusData4.chgReqireCurrL = niuCommdTable.ChargDemandCurrent[1];
            OneBusData4.chgReqireVoltL = (uint8_t)(((uint16_t)(niuCommdTable.ChargDemandCurrent[0]<<8) + niuCommdTable.ChargDemandCurrent[1])/10 >>8);
            OneBusData4.chgReqireVoltH = (uint8_t)(((uint16_t)niuCommdTable.ChargDemandCurrent[0]<<8) + niuCommdTable.ChargDemandCurrent[1])/10;
            OneBusData4.sum            = OneBusData4.pid + OneBusData4.heartbeat+OneBusData4.chgReqireCurrL+OneBusData4.chgReqireVoltL+OneBusData4.chgReqireVoltH ;
            Niu_OneBusSendData((uint8_t *)(&OneBusData4), sizeof(OneBusDataStruct4_t));
            break;
        }

        if(buseIndex == 4)
        {
            buseIndex = 0;
        }

        if(heartbeat == 100)
        {
            heartbeat = 0;
        }
    }
}

/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
void Niu_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{
#if(ONEBUS_SEND_MODE == 0)
    uint16_t i;
    Niu_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Niu_OneBusSendByte(*(sndBuf+i));
    }

    Niu_OneBusSendStop();
#elif(ONEBUS_SEND_MODE == 1)

    OneBusSendCfg.syncLDelayTime = 62000;   //同步头阶段低电平需要的延时us
    OneBusSendCfg.syncHDelayTime = 2000;   //同步头阶段高电平需要的延时us
    OneBusSendCfg.Data1LDelayTime = 2000;    //数据低电平需要延时的us数
    OneBusSendCfg.Data1HDelayTime = 4000;    //数据高电平需要延时的us数
    OneBusSendCfg.Data0LDelayTime = 2000;    //数据低电平需要延时的us数
    OneBusSendCfg.Data0HDelayTime = 4000;    //数据高电平需要延时的us数
    OneBusSendCfg.StopLDelayTime = 1000;    //Stop低电平需要延时的us数

    OneBusSta.bitPhase = 1;
    OneBusSta.bitIndex = 0;
    OneBusSta.bitSize = dataLen*8;
    Niu_OneBusTimerStart();

#endif

}

#if(ONEBUS_SEND_MODE == 0)
static void Niu_OneBusSendSyncStart(void)
{
    //MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    GPIOA_ResetBits(TN_ONE_TX_PAPIN);
    bsp_DelayUS(62000);
    GPIOA_SetBits(TN_ONE_TX_PAPIN);
    //MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Niu_OneBusSendStop(void)
{
    //MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    GPIOA_ResetBits(TN_ONE_TX_PAPIN);
    bsp_DelayUS(1000);
    //MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    GPIOA_SetBits(TN_ONE_TX_PAPIN);
    

}

static void Niu_OneBusSendBit1(void)
{
    //MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    GPIOA_ResetBits(TN_ONE_TX_PAPIN);
    bsp_DelayUS(2000);
    GPIOA_SetBits(TN_ONE_TX_PAPIN);
    //MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
 
}

static void Niu_OneBusSendBit0(void)
{
    //MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    GPIOA_ResetBits(TN_ONE_TX_PAPIN);
    bsp_DelayUS(4000);
    GPIOA_SetBits(TN_ONE_TX_PAPIN);
    //MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Niu_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x80)
        {
            Niu_OneBusSendBit1();
        }else
        {
            Niu_OneBusSendBit0();
        }
        data <<=1;
    }
}
#endif

static void Niu_OneBusTimerInit()
{
    //Fsys = 60Mhz,
    TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
    PFIC_EnableIRQ(TMR0_IRQn);
    //TMR0_CountOverflowCfg(cyc);
}

static void Niu_OneBusTimerStart()
{
    //1us 后马上开始
    TMR0_TimerInit(FREQ_SYS / 1000000);//us
    TMR0_Enable();
}

static void Niu_OneBusTimerStop()
{
    TMR0_Disable();
}

static void Niu_OneBusSetDelay(uint16_t dly)
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
    uint8_t byteIndex;
    uint8_t bitIndex;
    uint8_t curData;
    if(TMR0_GetITFlag(TMR0_3_IT_CYC_END))
    {
        TMR0_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志

        //GPIOB_InverseBits(GPIO_Pin_15);
        switch(OneBusSta.bitPhase)
        {
            //1-同步头低电平阶段    2-同步头高电平阶段       3 数据低电平阶段   4 数据高电平  5 -Stop低阶段   6-stop高电平
            case 1://
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Niu_OneBusSetDelay(OneBusSendCfg.syncLDelayTime);

                OneBusSta.bitPhase = 2;
                break;
            case 2:
                GPIOA_SetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Niu_OneBusSetDelay(OneBusSendCfg.syncHDelayTime);
                OneBusSta.bitPhase = 3;
                break;
            case 3:
                byteIndex = OneBusSta.bitIndex/8;
                bitIndex = OneBusSta.bitIndex%8;
                curData = *(OneBusSta.buf+byteIndex);
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                if(curData & bitIndex == (0x01 << bitIndex))//bit == 1
                {
                    //设置延迟
                    Niu_OneBusSetDelay(OneBusSendCfg.Data1LDelayTime);
                }else {
                    //设置延迟
                    Niu_OneBusSetDelay(OneBusSendCfg.Data0LDelayTime);
                }
                OneBusSta.bitPhase = 4;

                break;
            case 4:

                if(OneBusSta.bitIndex++ == OneBusSta.bitSize)
                {
                    OneBusSta.bitPhase = 5;
                }else {
                    GPIOA_SetBits(TN_ONE_TX_PAPIN);
                    if((curData & bitIndex) == (0x01 << bitIndex))
                    {
                        Niu_OneBusSetDelay(OneBusSendCfg.Data1HDelayTime); //设置延迟
                        //设置延迟
                    }else {
                        Niu_OneBusSetDelay(OneBusSendCfg.Data0HDelayTime); //设置延迟
                    }
                    OneBusSta.bitPhase = 3;
                }
                break;
            case 5:
                GPIOA_ResetBits(TN_ONE_TX_PAPIN);
                //设置延迟
                Niu_OneBusSetDelay(OneBusSendCfg.StopLDelayTime);
                OneBusSta.bitPhase = 6;
                break;
            case 6:
                GPIOA_SetBits(TN_ONE_TX_PAPIN);
                OneBusSta.bitPhase = 0;
                Niu_OneBusTimerStop();
                break;
        }
    }
}
#endif
