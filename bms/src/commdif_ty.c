/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/11/29
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

#if(PROJECT_ID == 2)

//天瑞定义的数据表总字节数
#define TY_COMMD_TAB_SIZE (sizeof(stc_TYCommdTable_t))

#define TY_MODBUS_DEV_A0 0x01

//命令码,主机发送的命令码
#define TY_MODBUS_CMD_READ           0x03
#define TY_MODBUS_CMD_WRITE          0x10


TY_MdStructTable_t TyTable;


uint8_t TYMdOrgInBuf[256];
uint8_t TYMdAckBuf[256];



//请求命令帧格式---------------------
// 说明     代码     长度（字节）
// 地址域   A0        1
// 命令码   CMD       1
// 指令地址 ADDRH     1
// 指令地址 ADDRL     1
// 数据长度 LEN      1
// TY_CRC   TY_CRCH        1      
// TY_CRC   TY_CRCL        1         
//命令帧格式---------------------

//响应命令帧格式---------------------
// 说明     代码     长度（字节）
// 地址域   A0        1
// 命令码   CMD       1
// 响应内容长度 LEN     1
// 数据长度 DATA1      1
// 数据长度 DATAn      1
// TY_CRC   TY_CRCH        1      
// TY_CRC   TY_CRCL        1         
//命令帧格式---------------------


static uint16_t Ty_CalcTY_CRC(uint8_t *pDesBuf,uint8_t len)
{
    uint16_t TY_CRC = 0xFFFF;
    uint16_t i;
    uint16_t count;

    for(count = 0; count < len; count++)
    {
        int i;
        TY_CRC = TY_CRC^*(pDesBuf + count);
        for(i = 0; i < 8; i++)
        {
            if(TY_CRC&1)
            {
                TY_CRC>>=1;
                TY_CRC^=0xA001;
            }
            else{
                TY_CRC>>=1;
            }
        }
    }
    return TY_CRC;
}

/**
 * function:小牛回应帧打包函数
 * @pDesBuf:外部输入被打包后的数据存放的缓冲
 * @cmd    :回应的命令码
 * @data   :根据对应的命令，做相应填充
 * @length :data数据的长度
 */ 
static uint8_t CommdPackAckFrame(uint8_t *pDesBuf,uint8_t *data,uint8_t length)
{
    uint8_t i;
    uint16_t TY_CRC = 0;
    *pDesBuf = 0x01;
    *(pDesBuf+1) = 0x03;
    *(pDesBuf+2) = length;
 
    for(i = 0;i < length; i++)
    {
        *(pDesBuf+3+i) = *(data+i);
    }

    TY_CRC = Ty_CalcTY_CRC(pDesBuf,length+3);

    *(pDesBuf+3+length) = (uint8_t)TY_CRC;
    *(pDesBuf+4+length) = (uint8_t)(TY_CRC>>8);
    return 5+length;
    
}

// bit0 单节过压保护(报警)
// bit1 单节欠压保护(报警)
// bit2 总压过压保护(报警)
// bit3 总压欠压保护(报警)
// bit4 充电过流保护(报警)
// bit5 放电过流保护(报警)
// bit6 充电过温保护(报警)
// bit7 放电过温保护(报警)
// bit8 充电低温保护(报警)
// bit9 放电低温保护(报警)
// bit10 压差过大保护(报警)
// bit11 预留
// bit12 SOC过低保护(报警)
// bit13 散热片过温保护(报警)
// bit14 预留
// bit15 预留

//#define TH_WARN_OV   4200
//#define TH_WARN_UV   3100
//#define TH_WARN_OCC  25
//#define TH_WARN_OCD  100
//#define TH_WARN_OCT  50
//#define TH_WARN_ODT  65
//#define TH_WARN_UCT  3
//#define TH_WARN_UDT  -10
//#define TH_WARN_VDIFF 400
//#define TH_WARN_LOW_SOC 10
//#define TH_WARN_MOS_OT  80
static uint16_t Ty_GetProtect1(void)
{
    uint16_t protectState = 0;

    static uint16_t CountOV = 0; //过压计数
    static uint16_t CountUV = 0; //欠压计数
    static uint16_t VdiffFlag = 0;
    static uint16_t CountOCC = 0;//充电过流计数
    static uint16_t CountOCD = 0;//放电过流计数
    static uint16_t CountOTC = 0;//充电高温计数
    static uint16_t CountOTD = 0;//放电高温计数
    static uint16_t CountUTC = 0;//低温充电计数
    static uint16_t CountUTD = 0;//低温放电计数
    static uint16_t CountMOSOT = 0;//MOS 过温保护
    static uint16_t CountSoc100 = 0;//100%满电关断
    static uint16_t CountVdiff = 0;//压差大延迟计数
    //单体过压
    if(afeInfo.CellVmax > TH_WARN_OV)
    {
        if(CountOV++ >10)
        {
            protectState |= 0x01;
        }
    }else
    if(afeInfo.CellVmax < (TH_WARN_OV-50))
    {
        protectState &= ~0x01;
        CountOV = 0;
    }

    //单体欠压
    if(afeInfo.CellVmin < TH_WARN_UV)
    {
        if(CountUV++ >10)
        {
            protectState |= 0x02;
        }
    }else
    if(afeInfo.CellVmin > (TH_WARN_UV+50))
    {
        protectState &= ~0x02;
        CountUV = 0;
    }

    //总压过压
    if(afeInfo.SumBatteryPackVolt > CELL_NUMS*TH_WARN_OV)
    {
        if(CountOV++ >10)
        {
            protectState |= 0x04;
        }
    }else
    if(afeInfo.SumBatteryPackVolt < CELL_NUMS*(TH_WARN_OV-50))
    {
        protectState &= ~0x04;
        CountOV = 0;
    }

    //总压欠压
    if(afeInfo.SumBatteryPackVolt < CELL_NUMS*TH_WARN_UV)
    {
        if(CountUV++ >10)
        {
            protectState |= 0x08;
        }
    }else
    if(afeInfo.SumBatteryPackVolt > CELL_NUMS*(TH_WARN_UV+50))
    {
        protectState &= ~0x08;
        CountUV = 0;
    }

    //充电过流保护(报警)
    if(afeInfo.ChgCurrent >1000*TH_WARN_OCC)
    {
        if(CountOCC++ >10)
        {
            protectState |= 0x10;
        }
    }else
    if(afeInfo.ChgCurrent < 1000*(TH_WARN_OCC-5))
    {
        protectState &= ~0x10;
        CountOCC = 0;
    }

    //放电过流保护(报警)
    if(afeInfo.DsgCurrent >1000*TH_WARN_OCD)
    {
        if(CountOCD++ >10)
        {
            protectState |= 0x20;
        }
    }else
    if(afeInfo.ChgCurrent < 1000*(TH_WARN_OCD-5))
    {
        protectState &= ~0x20;
        CountOCD = 0;
    }

    //充电过温保护
    if(afeInfo.State_RT & 0x02)
    {
         if(afeInfo.CellTmax >(TH_WARN_OCT*10 + 2731))
        {
            if(CountOTC++ >10)
            {
                protectState |= 0x40;
            }
        }else
        if(afeInfo.CellTmax < (10*(TH_WARN_OCT-5)+2731))
        {
            protectState &= ~0x40;
            CountOTC = 0;
        }
    }
   

    //放电过温保护(报警)
    if(afeInfo.State_RT & 0x01)
    {
        if(afeInfo.CellTmax >(TH_WARN_ODT*10 + 2731))
        {
            if(CountOTD++ >10)
            {
                protectState |= 0x80;
            }
        }else
        if(afeInfo.CellTmax < (10*(TH_WARN_ODT-5)+2731))
        {
            protectState &= ~0x80;
            CountOTD = 0;
        }
    }

    //充电低温报警
    if(afeInfo.State_RT & 0x02)
    {
         if(afeInfo.CellTmin < (TH_WARN_UCT*10 + 2731))
        {
            if(CountUTC++ >10)
            {
                protectState |= 0x100;
            }
        }else
        if(afeInfo.CellTmin > (10*(TH_WARN_UCT+3)+2731))
        {
            protectState &= ~0x100;
            CountUTC = 0;
        }
    }

     //放电电低温报警
    if(afeInfo.State_RT & 0x01)
    {
         if(afeInfo.CellTmin < (TH_WARN_UDT*10 + 2731))
        {
            if(CountUTD++ >10)
            {
                protectState |= 0x200;
            }
        }else
        if(afeInfo.CellTmin > (10*(TH_WARN_UDT+3)+2731))
        {
            protectState &= ~0x200;
            CountUTD = 0;
        }
    }

    //压差大报警
    if(afeInfo.CellVdiff > TH_WARN_VDIFF)
    {
        if(CountVdiff++ >10)
        {
            protectState |= 0x400;
        }
    }else
    if(afeInfo.CellVdiff < TH_WARN_VDIFF-100)
    {
        protectState &= ~0x400;
        CountVdiff = 0;
    }

    //Soc 低报警
    if(algEnginer.soc_r <TH_WARN_LOW_SOC*10)
    {
        protectState |= 0x1000;
    }else
    {
        protectState &= ~0x1000;
    }

    //mos 过温
    if(afeInfo.CellTmax >(TH_WARN_MOS_OT*10 + 2731))
    {
        if(CountMOSOT++ >10)
        {
            protectState |= 0x2000;
        }
    }else
    if(afeInfo.CellTmax < (10*(TH_WARN_MOS_OT-5)+2731))
    {
        protectState &= ~0x2000;
        CountMOSOT = 0;
    }

}

static uint16_t Ty_GetProtect2(void)
{

}

static uint16_t Ty_GetProtect3(void)
{

}

void Ty_TableUpdate(void)
{
    uint8_t i;

    for(i = 0;i<32;i++)
    {
        if(i <CELL_NUMS)
        {
            TyTable.Cellx[2*i] = (uint8_t)(afeInfo.CellVolt[i]>>8);
            TyTable.Cellx[2*i+1] = (uint8_t)(afeInfo.CellVolt[i]);
        }else
        {
            TyTable.Cellx[2*i] = (uint8_t)(((uint16_t)61001)>>8);
            TyTable.Cellx[2*i+1] = (uint8_t)(61001);
        }
        
    }

    TyTable.CellVMax[0] = (uint8_t)(afeInfo.CellVmax>>8);
    TyTable.CellVMax[1] = (uint8_t)afeInfo.CellVmax;
    TyTable.CellVMin[0] = (uint8_t)(afeInfo.CellVmin>>8);
    TyTable.CellVMin[1] = (uint8_t)afeInfo.CellVmin;

    TyTable.CellVMaxPos[0] = 0;
    TyTable.CellVMaxPos[1] = (afeInfo.CellVmaxPos);

    TyTable.CellVminPos[0] = 0;
    TyTable.CellVminPos[1] = afeInfo.CellVminPos;

    TyTable.CellVSum[0] = (uint8_t)((afeInfo.SumBatteryPackVolt/10)>>8);
    TyTable.CellVSum[1] = (uint8_t)(afeInfo.SumBatteryPackVolt/10);

    TyTable.CellVdiff[0] = (uint8_t)(afeInfo.CellVdiff>>8);
    TyTable.CellVdiff[1] = (uint8_t)afeInfo.CellVdiff;

    TyTable.TempX[0] = (uint8_t)((afeInfo.ShTemp[0] -2731+400)>>8);
    TyTable.TempX[1] = (uint8_t)(afeInfo.ShTemp[0] -2731+400);


    TyTable.TempX[2] = (uint8_t)((afeInfo.ShTemp[1] -2731+400)>>8);
    TyTable.TempX[3] = (uint8_t)(afeInfo.ShTemp[1] -2731+400);

    TyTable.TempX[4] = (uint8_t)((afeInfo.ShTemp[2] -2731+400)>>8);
    TyTable.TempX[5] = (uint8_t)(afeInfo.ShTemp[2] -2731+400);

    TyTable.TempX[6] = (uint8_t)((uint16_t)1200>>8);
    TyTable.TempX[7] = (uint8_t)1200;

    TyTable.TempX[8] = (uint8_t)((uint16_t)1200>>8);
    TyTable.TempX[9] = (uint8_t)1200;

    TyTable.TempX[10] = (uint8_t)((uint16_t)1200>>8);
    TyTable.TempX[11] = (uint8_t)1200;


    TyTable.EnvTempx[0] = (uint8_t)((uint16_t)1200>>8);
    TyTable.EnvTempx[1] = (uint8_t)1200;

    TyTable.EnvTempx[1] = (uint8_t)((uint16_t)1200>>8);
    TyTable.EnvTempx[2] = (uint8_t)1200;

    TyTable.EnvTempx[3] = (uint8_t)((uint16_t)1200>>8);
    TyTable.EnvTempx[4] = (uint8_t)1200;

    TyTable.MOsTemp[0] = (uint8_t)((afeInfo.MosTemp -2731 +400)>>8);
    TyTable.MOsTemp[1] = (uint8_t)(afeInfo.MosTemp -2731 +400);


    TyTable.TempHigh[0] = (uint8_t)((afeInfo.CellTmax -2731+400)>>8);
    TyTable.TempHigh[1] = (uint8_t)(afeInfo.CellTmax -2731+400);


    TyTable.TempLow[0] = (uint8_t)((afeInfo.CellTmin -2731+400)>>8);
    TyTable.TempLow[1] = (uint8_t)(afeInfo.CellTmin -2731+400);

    TyTable.ChgCurrent[0] = (uint8_t)(((uint16_t)afeInfo.ChgCurrent/100)>>8);
    TyTable.ChgCurrent[1] = (uint8_t)((uint16_t)afeInfo.ChgCurrent/100);

    TyTable.DsgCurrent[0] = (uint8_t)((uint16_t)(afeInfo.DsgCurrent/100)>>8);
    TyTable.DsgCurrent[1] = (uint8_t)(afeInfo.DsgCurrent/100);

    TyTable.Soc[0] = (uint8_t)(((uint16_t)algEnginer.soc_r/10)>>8);
    TyTable.Soc[1] = (uint8_t)((uint16_t)algEnginer.soc_r/10);

    TyTable.Soh[0] = (uint8_t)(algEnginer.soh_r>>8);
    TyTable.Soh[1] = (uint8_t)(algEnginer.soh_r);

    TyTable.ResAh[0] = (uint8_t)((uint16_t)(algEnginer.resCapAH_r*100)>>8);
    TyTable.ResAh[1] = (uint8_t)(algEnginer.resCapAH_r*100);


    TyTable.Fcc[0] = (uint8_t)((uint16_t)(algIntnel.capacity*100)>>8);
    TyTable.Fcc[1] = (uint8_t)(algIntnel.capacity*100);

    TyTable.EFcc[0] = (uint8_t)((uint16_t)(LITHIUM_FULL_AH*100)>>8);
    TyTable.EFcc[1] = (uint8_t)((LITHIUM_FULL_AH*100));


    TyTable.cycCount[0] = (uint8_t)(algEnginer.cycCount>>8);
    TyTable.cycCount[1] = (uint8_t)algEnginer.cycCount;

    TyTable.ProtectInfo1[0] = (uint8_t)(Ty_GetProtect1()>>8);
    TyTable.ProtectInfo1[1] = (uint8_t)(Ty_GetProtect1());


    TyTable.ProtectInfo2[0] = (uint8_t)(Ty_GetProtect2()>>8);
    TyTable.ProtectInfo2[1] = (uint8_t)(Ty_GetProtect2());


    TyTable.ProtectInfo3[0] = (uint8_t)(Ty_GetProtect3()>>8);
    TyTable.ProtectInfo3[1] = (uint8_t)(Ty_GetProtect3());


    TyTable.BalanceState1[0] = (uint8_t)((uint16_t)afeInfo.balState>>8);
    TyTable.BalanceState1[1] = (uint8_t)afeInfo.balState;


    TyTable.BalanceState2[0] = (uint8_t)(afeInfo.balState>>24);
    TyTable.BalanceState2[1] = (uint8_t)(afeInfo.balState>>16);

    TyTable.ChgResTime[0] = (uint8_t)((uint16_t)(6.0*algEnginer.chgResTime2)>>8);
    TyTable.ChgResTime[1] = (uint8_t)(6*algEnginer.chgResTime2);


    TyTable.DsgResTime[0] = (uint8_t)((uint16_t)(60*1000*algEnginer.resCapAH_r/afeInfo.DsgCurrent)>>8);
    TyTable.DsgResTime[1] = (uint8_t)(60*1000*algEnginer.resCapAH_r/afeInfo.DsgCurrent);
    
}

static void TYModbusParse(uint8_t *pMdInput,uint8_t len)
{
    uint8_t i;
    uint8_t byCount;
    uint8_t Addr;   //内部寄存器读写地址
    uint16_t TY_CRCO;
    uint16_t TY_CRC;
    uint16_t length2;

    //TyModbusTY_CRCCheck
    TY_CRCO = pMdInput[len-2] + (((uint16_t)pMdInput[len-1])<<8);
    TY_CRC = Ty_CalcTY_CRC(pMdInput,len-2);

    if(TY_CRCO == TY_CRC)
    {
        Addr = ((uint16_t)pMdInput[2]<<8) +pMdInput[3];
        byCount = ((uint16_t)pMdInput[4]<<8) +pMdInput[5];

        Addr = Addr - TY_MDTABLE_BASEADDR;

        length2 = CommdPackAckFrame(TYMdAckBuf,(uint8_t*)&TyTable+Addr*2,2*byCount);        
        //返回需要读的数据
        Uart0SendData(TYMdAckBuf,length2);
        SEGGER_RTT_printf(0,"ty data send:");
        for(int i = 0;i<length2;i++)
        {
            SEGGER_RTT_printf(0,"%02X ",TYMdAckBuf[i]);
        }
        SEGGER_RTT_printf(0,"\n");
    }else
    {
        SEGGER_RTT_printf(0,"recv crc error\n");
    }
}

void TY_ModbusRecvHandle(uint8_t rdata)
{
    //连续2次
    static uint8_t rxIndex = 0;

    if(rdata == TY_MODBUS_DEV_A0 && rxIndex == 0)
    {
        TYMdOrgInBuf[rxIndex++] = rdata;
        SEGGER_RTT_printf(0,"recv:%02X ",rdata);

        goto here;
    }else
    if(rxIndex > 0)
    {           
        TYMdOrgInBuf[rxIndex++] = rdata;
        if(rxIndex > 254)
        {
            rxIndex = 0;
            goto here;
        }
        SEGGER_RTT_printf(0,"%02X ",rdata);
        //请求读数据帧
        if(rxIndex >=8)
        {   
            //请求读数据帧         
            if(TY_MODBUS_CMD_READ == TYMdOrgInBuf[1] && rxIndex == 8)
            {
                SEGGER_RTT_printf(0,"\n");
                TYModbusParse(TYMdOrgInBuf,rxIndex);
                rxIndex = 0;
            }else{
                rxIndex = 0;
            }
            //请求写数据帧,暂时不用实现
            if(TY_MODBUS_CMD_WRITE == TYMdOrgInBuf[1] && rxIndex == (9+TYMdOrgInBuf[5]))
            {
            
                rxIndex = 0;
            }

        }   
    }
    
here:
    //再次兼容自家协议,自己家协议上位机需要修改串口配置的极性
    NIU_ModbusRecvHandle(rdata);
}

#endif

