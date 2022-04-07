/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2021/10/16
History:
	V0.0		2021/10/16		 Preliminary
********************************************************************************/

#include "includes.h"
#if(ONEBUS_TYPE == 7)
static void Zb_OneBusSendSyncStart(void);
static void Zb_OneBusSendStop(void);
static void Zb_OneBusSendBit1(void);
static void Zb_OneBusSendBit0(void);
static void Zb_OneBusSendByte(uint8_t data);
static void Zb_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);

//电池状态和充电器状态
static uint8_t Zb_GetBatState();

//雅迪50字节 协议数据帧
typedef struct ZbOneBus
{
    uint8_t pid;//报文ID 固定0x19
    uint8_t len;//总的数据长度
    uint8_t state;//内部状态
                  //bit0 保留
                  //bit1 保留
                  //bit2 保留
                  //bit3 充电器连接
                  //bit4 保留
                  //bit5 预放电
                  //bit6 放电mos
                  //bit7 充电
    uint8_t soc; //0-100
    uint8_t cs;



    uint8_t sum;//校验和
} ZbOneBusDataStruct_t;

ZbOneBusDataStruct_t ZbOneBusData;

/**
 * @brief 一线通IO初始化
 * 
 */
void Zb_OneBusInit(void)
{
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,50);
}

void Zb_OneBusProcess(void)
{
    uint16_t tempval = 0;
    uint8_t sh_sta;
    uint8_t onebus_sta = 0;
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        ZbOneBusData.pid = 0x29;
        ZbOneBusData.len = 5;
        ZbOneBusData.state = Zb_GetBatState();
        ZbOneBusData.soc = algEnginer.soc_r;

        for (int i = 0; i < sizeof(ZbOneBusDataStruct_t) - 1; i++)
        {
            ZbOneBusData.sum += *((uint8_t *)(&ZbOneBusData) + i);
        }

        Zb_OneBusSendData((uint8_t *)(&ZbOneBusData), sizeof(ZbOneBusDataStruct_t));
    }
    
}

static uint8_t Zb_GetBatState()
{
    uint8_t r;
    //电池工作状态，0xFF，0x00 -放电 0x01 -充电  0x02- 单独馈电 0x03 保留
    ZbOneBusData.state &= ~0x0F;
    if(afeInfo.MosState_RT & 0x02)
    {
        r |= 0x08;
    }            
    
    if(afeInfo.MosState_RT & 0x80)
    {
        r |= 0x20;
    }

    if(afeInfo.MosState_RT & 0x20)
    {
        r |= 0x40;
    }

    if(afeInfo.MosState_RT & 0x40)
    {
        r |= 0x80;
    }
 
    return r;  

}


/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
static void Zb_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{

    uint16_t i;
    Zb_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Zb_OneBusSendByte(*(sndBuf+i));
    }
    Zb_OneBusSendStop();
}

static void Zb_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(40000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Zb_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(10000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
}

static void Zb_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
}

static void Zb_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(4000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(2000);
}

static void Zb_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x01)
        {
            Zb_OneBusSendBit1();
        }else
        {
            Zb_OneBusSendBit0();
        }
        data >>=1;
    }
}
#endif