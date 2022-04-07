/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2021/10/17
History:
	V0.0		2021/10/17		 Preliminary
********************************************************************************/

#include "includes.h"
#if (ONEBUS_TYPE == 4)

//新日12字节协议
typedef struct YdOneBus
{
    uint8_t devid;//设备ID 0x03
    uint8_t cmd;  //配置指令 0x01
    uint8_t errSta;//故障状态
    uint8_t cellVmax;//最高电芯电压
    uint8_t soc;    //0-100
    uint8_t soh;    //0-100
    uint8_t cellTmax; //最高电芯温度 
    uint8_t cellTmin;//最低电池温度
    uint8_t cycleCountH;//循环周期高
    uint8_t cycleCountL;//循环周期L
    uint8_t cellVmin;//最低电芯电压
    uint8_t sum;
} xrOneBusDataStruct_t;

xrOneBusDataStruct_t xrOneBusData;


static uint8_t xr_GetErrSta(void);
static void Xr_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);
static void Xr_OneBusSendSyncStart(void);
static void Xr_OneBusSendBit1(void);
static void Xr_OneBusSendBit0(void);
static void Xr_OneBusSendByte(uint8_t data);

/**
 * @brief 一线通IO初始化
 * 
 */
void Xr_OneBusInit(void)
{
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,50);
}


void Xr_OneBusProcess(void)
{
    uint8_t i;
    uint8_t sum;
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        xrOneBusData.devid = 0x03;
        xrOneBusData.cmd   = 0x01;
        xrOneBusData.errSta = xr_GetErrSta();
        xrOneBusData.cellVmax = afeInfo.CellVmax/10 -185;//10mv ,正偏移185 4.4-1.85V
        xrOneBusData.soc  = algEnginer.soc_r/10;
        xrOneBusData.soh  = algEnginer.soh_r/10;
        xrOneBusData.cellTmax = (int8_t)((uint8_t)(afeInfo.CellTmax -2731)/10);
        xrOneBusData.cellTmin = (int8_t)((uint8_t)(afeInfo.CellTmin -2731)/10);
        xrOneBusData.cycleCountH = (uint8_t)(algEnginer.cycCount>>8);
        xrOneBusData.cycleCountL = (uint8_t)algEnginer.cycCount;
        xrOneBusData.cellVmin = afeInfo.CellVmin/10 -185;//10mv ,正偏移185 4.4-1.85V
        
        //11个字节异或
        sum = xrOneBusData.devid;
        for(i =0;i<11;i++)
        {
            sum = sum^ *((uint8_t*)&xrOneBusData + i);
        }
        xrOneBusData.sum = sum;
        Xr_OneBusSendData((uint8_t *)(&xrOneBusData), sizeof(xrOneBusDataStruct_t));
    }
}

static uint8_t xr_GetErrSta(void)
{
    return 0x00;
}

/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
static void Xr_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{

    uint16_t i;
    Xr_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Xr_OneBusSendByte(*(sndBuf+i));
    }
}

static void Xr_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(16000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(500);
}

static void Xr_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(5000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
}

static void Xr_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(500);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1000);
}

static void Xr_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(500);
}

static void Xr_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x80)
        {
            Xr_OneBusSendBit1();
        }else
        {
            Xr_OneBusSendBit0();
        }
        data <<=1;
    }
}

#endif