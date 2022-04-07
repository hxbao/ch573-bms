/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2020/10/16		 Preliminary
********************************************************************************/
#if(ONEBUS_TYPE == 2)
#include "includes.h"


static void Tn_OneBusSendSyncStart(void);
static void Tn_OneBusSendStop(void);
static void Tn_OneBusSendBit1(void);
static void Tn_OneBusSendBit0(void);
static void Tn_OneBusSendByte(uint8_t data);
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

OneBusDataStruct_t OneBusData;

void Tn_OneBusInit(void)
{
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,150);
}

void TN_OneBusProcess(void)
{
    uint16_t tempval = 0;
    uint8_t sh_sta;
    uint8_t onebus_sta = 0;
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        sh_sta = Sh_GetBStatus3();
        //放电mos状态
        if ((sh_sta & 0x04) == 0x01)
        {
            onebus_sta |= 0x40;
        }
        else
        {
            onebus_sta &= (~0x40);
        }

        //充电mos状态
        if ((sh_sta & 0x08) == 0x02)
        {
            onebus_sta |= 0x80;
        }
        else
        {
            onebus_sta &= (~0x80);
        }

        //预充电
        if ((sh_sta & 0x04) == 0x04)
        {
            onebus_sta |= 0x20;
        }
        else
        {
            onebus_sta &= (~0x20);
        }

        //充电器连接状态
        if ((sh_sta & 0x80) == 0x80)
        {
            onebus_sta |= 0x08;
        }
        else
        {
            onebus_sta &= (~0x08);
        }

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
        OneBusData.MaxAllowMotoChargeCurrent = 30; //最大允许回馈电流
        OneBusData.MaxAllowChargeCurrent = 10;     //最大允许充电电流
        OneBusData.MaxAllowChargeVoltL = 55;       //最大允许充电电压
        OneBusData.MaxAllowChargeVoltH = 0x00;

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

    uint16_t i;
    Tn_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Tn_OneBusSendByte(*(sndBuf+i));
    }

    Tn_OneBusSendStop();

}

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
#endif