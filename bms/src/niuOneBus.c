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


static void Niu_OneBusSendSyncStart(void);
static void Niu_OneBusSendStop(void);
static void Niu_OneBusSendBit1(void);
static void Niu_OneBusSendBit0(void);
static void Niu_OneBusSendByte(uint8_t data);

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

OneBusDataStruct1_t OneBusData1;
OneBusDataStruct2_t OneBusData2;
OneBusDataStruct3_t OneBusData3;
OneBusDataStruct4_t OneBusData4;


/**
 * @brief 一线通IO初始化
 * 
 */
void Niu_OneBusInit(void)
{
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

    uint16_t i;
    Niu_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Niu_OneBusSendByte(*(sndBuf+i));
    }

    Niu_OneBusSendStop();

}


static void Niu_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(62000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Niu_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    

}

static void Niu_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
 
}

static void Niu_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
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
