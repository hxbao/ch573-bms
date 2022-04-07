/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2021/10/16
History:
	V0.0		2021/10/16		 Preliminary
********************************************************************************/
#if(ONEBUS_TYPE == 3)
#include "includes.h"

static void Yd_OneBusSendSyncStart(void);
static void Yd_OneBusSendStop(void);
static void Yd_OneBusSendBit1(void);
static void Yd_OneBusSendBit0(void);
static void Yd_OneBusSendByte(uint8_t data);
static void Yd_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);


static uint8_t Yd_GetChgState(void);
static uint8_t Yd_GetAfeErrState();
static uint8_t Yd_GetDMOSErrState();
//bit4-5 预放电MOS故障
static uint8_t Yd_GetPreMOSErrState();
//bit6-7 充电MOS故障
static uint8_t Yd_GetCMOSErrState();
static uint8_t Yd_GetOVErrState();
static uint8_t Yd_GetCellTempErrState();
static uint8_t Yd_GetMOSTempErrState();
static uint8_t Yd_GetEnvTempErrState();
static uint8_t Yd_GetUVErrState();
static uint8_t Yd_GetCellOVErrState();
static uint8_t Yd_GetCellUVErrState();
static uint8_t Yd_GetCellVDiffErrState();
static uint8_t Yd_GetDsgOcErrState();
static uint8_t Yd_GetCellTDiffErrState();
//bit2-3 回馈过流 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetRchgOcErrState();
//bit4-5 充电过流 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgOcErrState();
//bit6-7 SOC低 0-正常 1-一般过低 2-严重过低（控制器保护） 3-void
static uint8_t Yd_GetSocErrState();

//bit2-3 充电高温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgOTErrState();
//bit2-3 充电低温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgUTErrState();
//bit4-5 放电高温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetDsgOTErrState();
//bit6-7 放电低温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetDsgUTErrState();
//0-未充电 1-握手阶段 2-配置阶段 3-恒流充电 4-恒压充电 5涓流充电 6-充电完成 7-充电保护 -x0F -void
static uint8_t Yd_GetChgPhase();

//0-未充电 1-握手阶段 2-配置阶段 3-恒流充电 4-恒压充电 5涓流充电 6-充电完成 7-充电保护 -x0F -void
static uint8_t Yd_GetChgState();
//获取电芯类型
static uint8_t Yd_GetLiOntype();
//电池状态和充电器状态
static uint8_t Yd_GetBatState();

//雅迪50字节 协议数据帧
typedef struct YdOneBus
{
    uint8_t pid;//报文ID 固定0x19
    uint8_t ver;//协议版本
    uint8_t bitBuf[47];//47字节
    uint8_t sum;//校验和
} YdOneBusDataStruct_t;

YdOneBusDataStruct_t YdOneBusData;

/**
 * @brief 一线通IO初始化
 * 
 */
void Yd_OneBusInit(void)
{
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,50);
}

void Yd_OneBusProcess(void)
{
    uint16_t tempval = 0;
    uint8_t sh_sta;
    uint8_t onebus_sta = 0;
    
    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))
    {
        YdOneBusData.pid = 0x19;
        YdOneBusData.ver = 0x16;     //协议版本 0-3 次协议版本 4-7 主协议版本 V1.06

        //电池状态、充电状态
        YdOneBusData.bitBuf[0] = Yd_GetBatState();
        //Soc
        YdOneBusData.bitBuf[1] = (uint8_t)(algEnginer.soc_r/10);
        //电池最大允许充电电压 16bit
        YdOneBusData.bitBuf[2] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_VOLT/10));
        YdOneBusData.bitBuf[3] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_VOLT/10)>>8);

        //电池最大允许充电电流 16bit
        YdOneBusData.bitBuf[4] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_CURR/10));
        YdOneBusData.bitBuf[5] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_CURR/10)>>8);

        //电池最大允许回馈电流 16bit
        YdOneBusData.bitBuf[6] = (uint8_t)((uint16_t)(DEFAULT_BACKCHG_CURR/10));
        YdOneBusData.bitBuf[7] = (uint8_t)((uint16_t)(DEFAULT_BACKCHG_CURR/10))>>8;

        //电池最大允许放电电流 16bit
        YdOneBusData.bitBuf[8] = (uint8_t)((uint16_t)(DEFAULT_DSG_CURR/10));
        YdOneBusData.bitBuf[9] = (uint8_t)((uint16_t)(DEFAULT_DSG_CURR/10)>>8);

        //电池剩余电流充满时间 16bit
        YdOneBusData.bitBuf[10] = (uint8_t)algEnginer.chgResTime;
        YdOneBusData.bitBuf[11] = (uint8_t)(algEnginer.chgResTime>>8);

        //电池总压 16bit 0.1V
        YdOneBusData.bitBuf[12] = (uint8_t)((afeInfo.SumBatteryPackVolt/100));
        YdOneBusData.bitBuf[13] = (uint8_t)((afeInfo.SumBatteryPackVolt/100)>>8);

        //电池电流 16bit 0.1A 偏移-500 ,充电为正，放电为负
        if(afeInfo.State_RT & 0x0001)
        {
            YdOneBusData.bitBuf[12] = (uint8_t)(afeInfo.DsgCurrent/100 -500);
            YdOneBusData.bitBuf[13] = (uint8_t)((afeInfo.DsgCurrent/100 -500)>>8);
        }else
        if(afeInfo.State_RT & 0x0002)
        {
            YdOneBusData.bitBuf[14] = (uint8_t)(afeInfo.ChgCurrent/100);
            YdOneBusData.bitBuf[15] = (uint8_t)((afeInfo.ChgCurrent/100)>>8);
        }else
        {
            YdOneBusData.bitBuf[14] = 0xFF;
            YdOneBusData.bitBuf[15] = 0xFF;
        }

        //MOS 状态 bit0-1 放电MOS状态 0-OFF 1-ON 2-保留 3 void 默认3
        //        bit2-3 充电MOS状态 0-OFF 1-ON 2-保留 3 void 默认3
        //        bit4-5 预放电MOS状态 0-OFF 1-ON 2-保留 3 void 默认3
        YdOneBusData.bitBuf[16] = 0x00;
        if(afeInfo.MosState_RT & 0x0010)
        {
            YdOneBusData.bitBuf[16] |= 0x01;
        }
        if(afeInfo.MosState_RT & 0x0020)
        {
            YdOneBusData.bitBuf[16] |= 0x04;
        }
        if(afeInfo.MosState_RT & 0x0040)
        {
            YdOneBusData.bitBuf[16] |= 0x10;
        }

        //电池剩余能量SOE 16bit
        YdOneBusData.bitBuf[17] = (uint8_t)(algEnginer.resCapAH_r*afeInfo.SumBatteryPackVolt/1000);
        YdOneBusData.bitBuf[18] = (uint8_t)((algEnginer.resCapAH_r*afeInfo.SumBatteryPackVolt/1000)>>8);

        //循环次数
        YdOneBusData.bitBuf[19] = (uint8_t)(algEnginer.cycCount);
        YdOneBusData.bitBuf[20] = (uint8_t)(algEnginer.cycCount>>8);

        //SOH_RT
        YdOneBusData.bitBuf[21] = (algEnginer.soh_r);
        //电池电芯温度最高,-40 -150,偏移-40
        YdOneBusData.bitBuf[22] = (uint8_t)(afeInfo.CellTmax-2731 +400)/10;
        //电池电芯温度最低,-40 -150,偏移-40
        YdOneBusData.bitBuf[23] = (uint8_t)(afeInfo.CellTmin-2731 +400)/10;
        //电池环境温度,-40 -150,偏移-40 用平衡温探代替
        YdOneBusData.bitBuf[24] = (uint8_t)(afeInfo.BalanceTemp-2731 +400)/10;
        //电池单体最小电压
        YdOneBusData.bitBuf[25] = (uint8_t)(afeInfo.CellVmin);
        YdOneBusData.bitBuf[26] = (uint8_t)(afeInfo.CellVmin>>8);
        //电池单体最高电压
        YdOneBusData.bitBuf[27] = (uint8_t)(afeInfo.CellVmax);
        YdOneBusData.bitBuf[28] = (uint8_t)(afeInfo.CellVmax>>8);
        //电池故障码
        YdOneBusData.bitBuf[29] = 0xFF;
            //bit0-1 前端afe故障 0-正常 1-采集均衡线开路 2-采集均衡通信错误 3-void
            YdOneBusData.bitBuf[29] |= Yd_GetAfeErrState();
            //bit2-3 放电MOS故障
            YdOneBusData.bitBuf[29] |= Yd_GetDMOSErrState();
            //bit4-5 预放电MOS故障
            YdOneBusData.bitBuf[29] |= Yd_GetPreMOSErrState();
            //bit6-7 充电MOS故障
            YdOneBusData.bitBuf[29] |= Yd_GetCMOSErrState();
        //温度传感器故障
        YdOneBusData.bitBuf[30] = 0xFF;
            //bit0-1 环境温度 0-正常 1-故障 2-保留 3-void
            YdOneBusData.bitBuf[30] |= Yd_GetEnvTempErrState();
            //bit2-3 MOS温度 0-正常 1-故障 2-保留 3-void
            YdOneBusData.bitBuf[30] |= Yd_GetMOSTempErrState();
            //bit4-5 电芯温度 0-正常 1-故障 2-保留 3-void
            YdOneBusData.bitBuf[30] |= Yd_GetCellTempErrState();
            //bit6-7 电压过压 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[30] |= Yd_GetOVErrState();
        //欠压、单体过压、单体欠压、单体压差大
        YdOneBusData.bitBuf[31] = 0xFF;
            //bit0-1 电压欠压 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[31] |= Yd_GetUVErrState();
            //bit2-3 单体过压 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[31] |= Yd_GetCellOVErrState();
            //bit4-5 电芯单体欠压 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[31] |= Yd_GetCellUVErrState();
            //bit6-7 单体压差大 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[31] |= Yd_GetCellVDiffErrState();        
         //放电过流、回馈过流、充电过流、SOC低
        YdOneBusData.bitBuf[32] = 0xFF;
            //bit0-1 放电过流 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[32] |= Yd_GetDsgOcErrState();
            //bit2-3 回馈过流 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[32] |= Yd_GetRchgOcErrState();
            //bit4-5 充电过流 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[32] |= Yd_GetChgOcErrState();
            //bit6-7 SOC低 0-正常 1-一般过低 2-严重过低（控制器保护） 3-void
            YdOneBusData.bitBuf[32] |= Yd_GetSocErrState();
        //充电高温、充电低温、放电高温、放电低温
        YdOneBusData.bitBuf[33] = 0xFF;
            //bit0-1 充电高温 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[33] |= Yd_GetChgOTErrState();
            //bit2-3 充电低温 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[33] |= Yd_GetChgUTErrState();
            //bit4-5 放电高温 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[33] |= Yd_GetDsgOTErrState();
            //bit6-7 放电低温 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[33] |= Yd_GetDsgUTErrState();
        //单体温差大
        YdOneBusData.bitBuf[34] = 0xFF;
            //bit0-1 单体温差过大 0-正常 1-警告 2-保护 3-void
            YdOneBusData.bitBuf[34] |= Yd_GetCellTDiffErrState();
        
        //MOS温度 偏移-40°
        YdOneBusData.bitBuf[35] = (uint8_t)(afeInfo.MosTemp-2731+400)/10;
        //保留
        YdOneBusData.bitBuf[36] = 0xFF;
        YdOneBusData.bitBuf[37] = 0xFF;

        //BMS硬件版本号
        YdOneBusData.bitBuf[38] = HW_VERSION;
        //软件次版本号
        YdOneBusData.bitBuf[39] = ((uint8_t)FW_VERSION & 0x0F);
        //软件主版本号
        YdOneBusData.bitBuf[40] = ((uint8_t)FW_VERSION & 0xF0)>>4;

        YdOneBusData.bitBuf[41] = 0x00;
            //充电阶段状态，0-未充电 1-握手阶段 2-配置阶段 3-恒流充电 4-恒压充电 5涓流充电 6-充电完成 7-充电保护 -x0F -void
            YdOneBusData.bitBuf[41] |= Yd_GetChgPhase();
            //雅迪get电芯类型
            YdOneBusData.bitBuf[41] |= Yd_GetLiOntype();
        //电芯串数    
        YdOneBusData.bitBuf[42] = (uint8_t)CELL_NUMS;
        //电池厂家代号bit0-3  
        YdOneBusData.bitBuf[43] |= 0x0F;
        //电池产品规格
        YdOneBusData.bitBuf[43] |= 0xF0;
        //电池生产年份bit1-7
        YdOneBusData.bitBuf[44] = 0x00;
        //电池生产月份bit0-3
        YdOneBusData.bitBuf[45] = 0x00;

        //电池生产流水码D45.4-7 D46.0-7 D47.0-7
        YdOneBusData.bitBuf[45] = 0x00;
        YdOneBusData.bitBuf[46] = 0x00;
        YdOneBusData.bitBuf[47] = 0x00;


        YdOneBusData.sum = 0;//0x28;//0;

        for (int i = 0; i < sizeof(YdOneBusDataStruct_t) - 1; i++)
        {
            YdOneBusData.sum += *((uint8_t *)(&YdOneBusData) + i);
        }

        Yd_OneBusSendData((uint8_t *)(&YdOneBusData), sizeof(YdOneBusDataStruct_t));
    }
    
}

static uint8_t Yd_GetBatState()
{
    uint8_t r;
    //电池工作状态，0xFF，0x00 -放电 0x01 -充电  0x02- 单独馈电 0x03 保留
    YdOneBusData.bitBuf[0] &= ~0x0F;
    if(afeInfo.State_RT & 0x0001)
    {
        r |= 0x00;
    }            
    else
    if(afeInfo.State_RT & 0x0002)
    {
        r |= 0x01;
    }else
    {
        r |= 0x0F;
    }

    //电池充电状态 0x00 -充电器未连接 0x01 -充电器连接  0x02- 充电器连接充电中 0x03 充电器连接完成充电 0x04 -保留 0x07 void
    YdOneBusData.bitBuf[0] &= ~0xF0;
    if(Yd_GetChgState() == 0)
    {
        r |= 0x00;
    }            
    else
    if(Yd_GetChgState() == 1)
    {
        r |= 0x10;
    }else
    if(Yd_GetChgState() == 2)
    {
        r |= 0x20;
    }else
    if(Yd_GetChgState() == 3)
    {
        r |= 0x30;
    }else
    if(Yd_GetChgState() == 7)
    {
        r |= 0x70;
    }
    return r;  

}

//bit4-5 预放电MOS故障
static uint8_t Yd_GetPreMOSErrState()
{
    uint8_t r = 0;
    return r<<4;
}
//bit6-7 充电MOS故障 0-正常 1-开路 2-短路 3-void
static uint8_t Yd_GetCMOSErrState()
{
    uint8_t r = 0;
    if(afeInfo.State_RT & 0x1000)
    {
        //充电MOS短路
        r = 0x2;
    }
    return r<<6;
}

//获取电芯类型 bit4-7
static uint8_t Yd_GetLiOntype()
{
    uint8_t r = 0;
    r = (uint8_t)LITHIUM_TYPE;
    return r<<4;
}

//0-未充电 1-握手阶段 2-配置阶段 3-恒流充电 4-恒压充电 5涓流充电 6-充电完成 7-充电保护 -x0F -void
static uint8_t Yd_GetChgPhase()
{
    uint8_t r = 0;

    return r;
}

//bit2-3 充电高温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgOTErrState()
{
    uint8_t r = 0;
    //充电中...
    if(afeInfo.State_RT & 0x0002)
    {
        //充电最高温度高于55°,设置为警告
        if(afeInfo.CellTmax > (2731+550))
        {
            r = 1;
        }        
    }
    if(afeInfo.State_RT2 & 0x02)//充电高温保护状态
    {
        r = 2;
    }
    return r;
}
//bit2-3 充电低温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgUTErrState()
{
    uint8_t r = 0;
    //充电中...
    if(afeInfo.State_RT & 0x0002)
    {
        //充电最高温度低于0°,设置为警告
        if(afeInfo.CellTmax < (2731))
        {
            r = 1;
        }        
    }

    if(afeInfo.State_RT2 & 0x01)//充电低温保护状态
    {
        r = 2;
    }
    return r<<2;
}
//bit4-5 放电高温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetDsgOTErrState()
{
    uint8_t r = 0;
     //放电中...
    if(afeInfo.State_RT & 0x0001)
    {
        //充电最高温度高于60°,设置为警告
        if(afeInfo.CellTmax > (2731+600))
        {
            r = 1;
        }        
    }    

    if(afeInfo.State_RT2 & 0x08)//放电高温保护状态
    {
        r = 2;
    }

    return r<<4;
}
//bit6-7 放电低温 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetDsgUTErrState()
{
    uint8_t r = 0;
     //放电中...
    if(afeInfo.State_RT & 0x0001)
    {
        //充电最低温度低于0°,设置为警告
        if(afeInfo.CellTmin < (2731))
        {
            r = 1;
        }        
    }    

    if(afeInfo.State_RT2 & 0x04)//放电低温保护状态
    {
        r = 2;
    }


    return r<<6;
}


//bit0-1 放电过流 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetDsgOcErrState()
{
    uint8_t r = 0;
    if(afeInfo.State_RT & 0x0020)
    {
        //放电过流保护
        r = 2;
    }

    return r;
}
//bit2-3 回馈过流 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetRchgOcErrState()
{
    uint8_t r = 0;

    //回馈过流的判断方法：当在放电过程中，短时的进入充电状态
    return r<<2;
}
//bit4-5 充电过流 0-正常 1-警告 2-保护 3-void
static uint8_t Yd_GetChgOcErrState()
{
    uint8_t r = 0;
    if(afeInfo.State_RT & 0x0010)
    {
        r = 2;
    }
    return r<<4;
}
//bit6-7 SOC低 0-正常 1-一般过低 2-严重过低（控制器保护） 3-void
static uint8_t Yd_GetSocErrState()
{
    uint8_t r = 0;
    //低于15%，认为一般过低
    if(algEnginer.soc_r <150)
    {
        r = 1;
    }else
    if(algEnginer.soc_r <50)
    {
        r = 2;
    }

    return r<<6;
}

/**
 * @brief 判断欠压故障
 * 
 * @return bit0-1 电压欠压 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetUVErrState()
{
    uint8_t r = 0;
    //总电压欠压
    //if()

    return r;
}

/**
 * @brief 判断单体过压故障
 * 
 * @return  bit2-3 单体过压 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetCellOVErrState()
{
    uint8_t r = 0;

    return r<<2;
}

/**
 * @brief 温差大故障
 * 
 * @return  bit0-1 温度差 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetCellTDiffErrState()
{
    uint8_t r = 0;

    return r;
}

/**
 * @brief 判断单体欠压故障
 * 
 * @return  bit4-5 电芯单体欠压 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetCellUVErrState()
{
     uint8_t r = 0;

    return r<<4;
}

/**
 * @brief 判断压差故障
 * 
 * @return  bit6-7 单体压差大 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetCellVDiffErrState()
{
     uint8_t r = 0;

    return r<<6;
}

/**
 * @brief 判断过压故障
 * 
 * @return  bit6-7  电压过压 0-正常 1-警告 2-保护 3-void
 * 
 */
static uint8_t Yd_GetOVErrState()
{

}


/**
 * @brief 判断MOS温度故障
 * 
 * @return  电芯温度 0-正常 1-故障 2-保留 3-void
 * 
 */
static uint8_t Yd_GetCellTempErrState()
{

}

/**
 * @brief 判断MOS温度故障
 * 
 * @return  MOS温度 0-正常 1-故障 2-保留 3-void
 * 
 */
static uint8_t Yd_GetMOSTempErrState()
{

}

/**
 * @brief 判断环境故障
 * 
 * @return  bit0-1  0-正常 1-开路 2-短路 3-void 
 * 
 */
static uint8_t Yd_GetEnvTempErrState()
{

}

/**
 * @brief 判断放电MOS故障
 * 
 * @return  bit2-3  0-正常 1-开路 2-短路 3-void 
 * 
 */
static uint8_t Yd_GetDMOSErrState()
{

}


/**
 * @brief 判断前端采集故障
 * 
 * @return  bit0-1 前端afe故障 0-正常 1-采集均衡线开路 2-采集均衡通信错误 3-void 
 * 
 */
static uint8_t Yd_GetAfeErrState()
{
    return 0;
}

/**
 * @brief 充电器状态的判断
 * 
 * @return  0x00 -充电器未连接 0x01 -充电器连接  0x02- 充电器连接充电中 0x03 充电器连接完成充电 0x04 -保留 0x07 void  
 * 
 */
static uint8_t Yd_GetChgState()
{
    return 0;
}

/**
 * @brief 一线通发送数据
 * 
 * @param sndBuf 
 * @param dataLen 需要发送数据帧的字节数据的个数
 */
static void Yd_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen)
{

    uint16_t i;
    Yd_OneBusSendSyncStart();
    for(i = 0;i<dataLen;i++)
    {
        Yd_OneBusSendByte(*(sndBuf+i));
    }
}

static void Yd_OneBusSendSyncStart(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(10000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1000);
}

static void Yd_OneBusSendStop(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(5000);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
}

static void Yd_OneBusSendBit1(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(500);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1500);
}

static void Yd_OneBusSendBit0(void)
{
    MCU_GPIO_ClrBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(1500);
    MCU_GPIO_SetBit(TN_ONE_TX_PORT, TN_ONE_TX_PIN);
    bsp_DelayUS(500);
}

static void Yd_OneBusSendByte(uint8_t data)
{
    uint8_t i;
    for(i =0;i<8;i++)
    {
        if(data&0x01)
        {
            Yd_OneBusSendBit1();
        }else
        {
            Yd_OneBusSendBit0();
        }
        data >>=1;
    }
}
#endif