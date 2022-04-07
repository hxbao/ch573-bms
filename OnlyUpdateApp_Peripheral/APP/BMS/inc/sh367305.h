#ifndef INCLUDE_SH305_H_
#define INCLUDE_SH305_H_

#include "includes.h"

#define SH305_FLAG1_ADDR 0
#define SH305_FLAG2_ADDR 1
#define SH305_BSTATUS_ADDR 2
#define SH305_INT_EN_ADDR 3
#define SH305_SCONF1_ADDR 4
#define SH305_SCONF2_ADDR 5
#define SH305_SCONF3_ADDR 6
#define SH305_SCONF4_ADDR 7
#define SH305_SCONF5_ADDR 8
#define SH305_SCONF6_ADDR 9
#define SH305_SCONF7_ADDR 10
#define SH305_SCONF8_ADDR 11
#define SH305_SCONF9_ADDR 12
#define SH305_SCONF10_ADDR 13
#define SH305_CELLX_START_ADDR 18
#define SH305_TS_START_ADDR   34
#define SH305_TEMP_START_ADDR 38
#define SH305_CUR_START_ADDR  42





void Sh305_DeviceInit(void);
void Sh_GetTemp(uint16_t *RTx);
void Sh_GetCellsVolt(uint16_t *volts);

void Sh_Process();
//返回值1-放电 0-充电 读取cadc 电流,高精度电流，250ms中断Alarm输出
//可以累计电量积分
uint8_t Sh_GetCadcCurrent(uint32_t *current);
void Sh_SetRunMode(uint8_t mode);

void Sh_ShutDChgMos(void);
void Sh_OpenChgMos(void);

void Sh_OpenDsgMos(void);
void Sh_ShutDDsgMos(void);

//判定充放电状态 0-静置 1-放电 2-充电
uint16_t Sh_JudgeState(void);

//获取温度保护类型
uint8_t Sh_GetTempProtectState(void);

void Sh_ClrFlag(void);

//控制平衡回路,开启平衡的通道 ch - 1，16
void Sh_SetBalance(uint8_t ch);

void Sh_DisableBalance(void);

//设定电流校准参数
void Sh_SetCurrCarlibation(uint16_t Kcurr,uint16_t Ocurr);

//设定电压校准参数
void Sh_SetVoltCarlibation(uint16_t Kvolt,uint16_t Ovolt);



#endif