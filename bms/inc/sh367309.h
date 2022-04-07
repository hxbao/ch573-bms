#ifndef INCLUDE_SH309_H_
#define INCLUDE_SH309_H_

#include "includes.h"

#define SH_DEFAULT_EECONFIG_1  0x7D
#define SH_DEFAULT_EECONFIG_2  0x12
#define SH_DEFAULT_EECONFIG_3  0x93
#define SH_DEFAULT_EECONFIG_4  0x54
#define SH_DEFAULT_EECONFIG_5  0xa3
#define SH_DEFAULT_EECONFIG_6  0x2a
#define SH_DEFAULT_EECONFIG_7  0x82
#define SH_DEFAULT_EECONFIG_8  0xa5
#define SH_DEFAULT_EECONFIG_9  0xbe
#define SH_DEFAULT_EECONFIG_10 0x78
#define SH_DEFAULT_EECONFIG_11 0x4b
#define SH_DEFAULT_EECONFIG_12 0xdc
#define SH_DEFAULT_EECONFIG_13 0x39
#define SH_DEFAULT_EECONFIG_14 0x54
#define SH_DEFAULT_EECONFIG_15 0x64
#define SH_DEFAULT_EECONFIG_16 0x0d
#define SH_DEFAULT_EECONFIG_17 0x16
#define SH_DEFAULT_EECONFIG_18 0x56
#define SH_DEFAULT_EECONFIG_19 0x9d
#define SH_DEFAULT_EECONFIG_20 0xa2
#define SH_DEFAULT_EECONFIG_21 0x8f
#define SH_DEFAULT_EECONFIG_22 0x56
#define SH_DEFAULT_EECONFIG_23 0x7c
#define SH_DEFAULT_EECONFIG_24 0xd0
#define SH_DEFAULT_EECONFIG_25 0xb6

typedef enum
{
    WATCHDOG_OVER = 0x80, 
    PF = 0x40, //二次过充电保护
    SC = 0x20, //短路保护
    OCC = 0x10,//充电过流保护
    OCD2 = 0x08,//放电过流2保护
    OCD1 = 0x04,//放电过流1保护
    UV = 0x02,//低电压保护
    OV = 0x01//过压保护
}BStatusFlagDef;

extern uint8_t EE_ReadBuf[26];
extern uint8_t EEP_RegMap[25];

void Sh_Process(void);
uint8_t Sh_CheckEEData(void);
void Sh_UpdateEEMapValue(uint8_t regval,uint8_t index);
void Sh_EEMapValueInit(void);

void SH_iicWriteEEPROM(uint8_t EEAddr,uint8_t eeRegVal);
uint8_t SH_iicReadEEPROM(uint8_t EEAddr,uint8_t readSize,uint8_t* pdataBuf);

void Sh_GetTemp(uint16_t *RTx);
void Sh_GetCellsVolt(uint16_t *volts);


#define SH_DISABLE_SHIPMODE() do  \
                            {                                    \
                                 MCU_GPIO_SetBit(TN_SHSHIP_PORT, TN_SHSHIP_PIN); \
                            } while (0);

#define SH_ENABLE_SHIPMODE() do  \
                            {                                    \
                                 MCU_GPIO_ClrBit(TN_SHSHIP_PORT, TN_SHSHIP_PIN); \
                            } while (0);
//返回值1-放电 0-充电，读取vadc 电流
uint8_t Sh_GetVadcCurrent(uint16_t *current);

//返回值1-放电 0-充电 读取cadc 电流,高精度电流，250ms中断Alarm输出
//可以累计电量积分
uint8_t Sh_GetCadcCurrent(uint32_t *current);
void Sh_SetRunMode(uint8_t mode);
void Sh_WatchdogEnable(void);
void Sh_WatchdogDisable(void);
void Sh_EnableCADC(void);
void Sh_DisableCADC(void);
void Sh_ShutDChgMos(void);
void Sh_OpenChgMos(void);

void Sh_OpenDsgMos(void);
void Sh_ShutDDsgMos(void);

void Sh_ShutDDsgMosIt(void);
void Sh_OpenDsgMosIt(void);
void Sh_ShutDChgMosIt(void);
void Sh_OpenChgMosIt(void);

void Sh_SWReset(void);
//00-dsg_fet 关闭 chg_fet 关闭， 0x01 - dsg_fet 导通 chg_fet 关闭 ，02- dsg_fet 关闭 chg_fet 导通 03 - dsg_fet 导通 chg_fet 导通
uint8_t Sh_GetMOSSta(void);

//7~0  WDT PF SC OCC OCD2 OCD1 UV OV
uint8_t Sh_GetBStatus1(void);

//3~0  OTD(放电高温保护) UTD(放电低温) OTC(充电高温保护) UTC(充电低温保护)
uint8_t Sh_GetBStatus2(void);

//系统状态，7~0 7（充电状态/非充电）  6（放电/非放电） 4（EE写操作错误/EE写操作正确） 3（低电压禁止充电/未发送低电压禁止充电） 
// 2（预充电MOS开启状态/预充电MOS关闭状态） 1（充电MOS开启/充电MOS关闭） 0（放电MOS开启/放电MOS关闭） 
uint8_t Sh_GetBStatus3(void);

//bit[15~8] WDT_FLG,PF_FLG,SC_FLG,OCC_FLG,LOAD_FLG,OCD_FLG,UV_FLG,OV_FLG
//bit[7~0] RST_FLG,WAKE_FLG,CADC_FLG,VADC_FLG,OTD_FLG,UTD_FLG,OTC_FLG,UTC_FLG
uint16_t Sh_GetBFlag(void);

void Sh_UpdateEEMapAll(void);

//判定充放电状态 0-静置 1-放电 2-充电
uint16_t Sh_JudgeState(void);

//获取温度保护类型
uint8_t Sh_GetTempProtectState(void);

void Sh_ClrFlag(uint8_t FlagCh,uint8_t flag);

//转化寄存器阈值到温度值
uint16_t Sh_ConvertTempTh2Value(uint8_t TempValue,uint16_t isLow);

//输入寄存器配置的温度值*10 K，输出寄存器配置值
uint8_t Sh_ConvertTemp2RegTh(uint16_t TempValue,uint16_t isLow);

//控制平衡回路,开启平衡的通道 ch - 1，16
void Sh_SetBalance(uint8_t ch);

void Sh_DisableBalance();

//设定电流校准参数
void Sh_SetCurrCarlibation(uint16_t Kcurr,uint16_t Ocurr);

//设定电压校准参数
void Sh_SetVoltCarlibation(uint16_t Kvolt,uint16_t Ovolt);



#endif