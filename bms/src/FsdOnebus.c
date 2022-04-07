/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2021/10/16
History:
	V0.0		2021/10/16		 Preliminary
********************************************************************************/

#include "includes.h"
#if(ONEBUS_TYPE == 9)
static void Fsd_OneBusSendSyncStart(void);
static void Fsd_OneBusSendStop(void);
static void Fsd_OneBusSendBit1(void);
static void Fsd_OneBusSendBit0(void);
static void Fsd_OneBusSendByte(uint8_t data);
static void Fsd_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);

//电池状态和充电器状态
static uint8_t Fsd_GetBatState();

//雅迪50字节 协议数据帧
typedef struct FsdOneBus
{
    uint8_t pid;//报文ID 固定0x00
    uint8_t soc;//
    uint8_t byte[3];
    uint8_t sum;//校验和
} FsdOneBusDataStruct_t;

FsdOneBusDataStruct_t FsdOneBusData;

/**
 * @brief 一线通IO初始化
 * 
 */
void FsdOneBusInit(void)
{
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,1000);
}

void FsdOneBusProcess(void)
{
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        FsdOneBusData.pid = 0x00;
        FsdOneBusData.soc = (uint8_t)(algEnginer.soc_r/10);
        FsdOneBusData.byte[0] = 0x00;
        FsdOneBusData.byte[1] = 0x00;
        FsdOneBusData.byte[2] = 0x00;

        for (int i = 0; i < sizeof(FsdOneBusDataStruct_t) - 1; i++)
        {
            FsdOneBusData.sum += *((uint8_t *)(&FsdOneBusData) + i);
        }

        Fsd_OneBusSendData((uint8_t *)(&FsdOneBusData), sizeof(FsdOneBusDataStruct_t));
    }
    
}


/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
static void Fsd_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{

    uint16_t i;
    Fsd_OneBusSendStop();
    Fsd_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Fsd_OneBusSendByte(*(sndBuf+i));
    }
    Fsd_OneBusSendStop();
}

static void Fsd_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(62000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Fsd_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(20000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
}

static void Fsd_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
}

static void Fsd_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Fsd_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x01)
        {
            Fsd_OneBusSendBit1();
        }else
        {
            Fsd_OneBusSendBit0();
        }
        data >>=1;
    }
}
#endif