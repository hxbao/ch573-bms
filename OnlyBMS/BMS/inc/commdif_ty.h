#ifndef _COMMD_TY_H
#define _COMMD_TY_H

#include "includes.h"
#if(PROJECT_ID ==2)
#define TY_MDTABLE_BASEADDR 0xD000

#define TH_WARN_OV   4200
#define TH_WARN_UV   3100
#define TH_WARN_OCC  25
#define TH_WARN_OCD  100
#define TH_WARN_OCT  50
#define TH_WARN_ODT  65
#define TH_WARN_UCT  3
#define TH_WARN_UDT  -10
#define TH_WARN_VDIFF 400
#define TH_WARN_LOW_SOC 10
#define TH_WARN_MOS_OT  80


typedef struct
{
    uint8_t Cellx[32*2];              //mv 高字节在前
    uint8_t CellVMax[2];               //mv 高字节在前
    uint8_t CellVMin[2];               //mv 高字节在前
    uint8_t CellVMaxPos[2]; 
    uint8_t CellVminPos[2];
    uint8_t CellVdiff[2];              //mv 高字节在前
    uint8_t CellVSum[2];               //V *100
    uint8_t TempX[6*2];               //摄氏度*10+400,8° = 10*8+400 = 480
    uint8_t EnvTempx[3*2];            //摄氏度*10+400,8° = 10*8+400 = 480
    uint8_t MOsTemp[2];                //摄氏度*10+400,8° = 10*8+400 = 480
    uint8_t TempHigh[2];               //摄氏度*10+400,8° = 10*8+400 = 480
    uint8_t TempLow[2];                //摄氏度*10+400,8° = 10*8+400 = 480
    uint8_t ChgCurrent[2];             //A*10
    uint8_t DsgCurrent[2];             //A*10
    uint8_t Soc[2];                    //0-100
    uint8_t Soh[2];                    //0-100
    uint8_t ResAh[2];                  //剩余容量 Ah*100
    uint8_t Fcc[2];                    //当前满电容量 Ah*100
    uint8_t EFcc[2];                   //额定容量 Ah*100
    uint8_t cycCount[2];               //cycle 循环次数   
    uint8_t ProtectInfo1[2];           //一级保护信息
    uint8_t ProtectInfo2[2];           //二级级保护信息
    uint8_t ProtectInfo3[2];           //三级保护信息
    uint8_t BalanceState1[2];          //均衡状态 1-16串
    uint8_t BalanceState2[2];          //17-32 串均衡状态
    uint8_t ChgResTime[2];             //充电剩余时间，分钟
    uint8_t DsgResTime[2];             //放电剩余时间, 分钟
}TY_MdStructTable_t;



void Ty_TableUpdate(void);
void TY_ModbusRecvHandle(uint8_t rdata);

#endif

#endif