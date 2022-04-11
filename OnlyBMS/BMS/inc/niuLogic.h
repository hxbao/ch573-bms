/******************************************************************************/
/** \file niuLogic.c
 **
 ** niu F serial bms logic
 **
 **   - 2021-9 hxbao    First Version
 **
 ******************************************************************************/

#ifndef INCLUDE_NIULOGIC_H_
#define INCLUDE_NIULOGIC_H_

#include "includes.h"

#define SWITCH_LED_ON() (GPIOA_ResetBits(TN_LED_PAPIN))
#define SWITCH_LED_OFF() (GPIOA_SetBits(TN_LED_PAPIN))

#define SWITCH_PRED_ON() (GPIOB_SetBits(TN_PREEN_PBPIN))
#define SWITCH_PRED_OFF() (GPIOB_ResetBits(TN_PREEN_PBPIN))



#define GET_ACC()  ( GPIOB_ReadPortPin(TN_ACC_PBPIN) == TN_ACC_PBPIN)
#define GET_CHGDET()  ( GPIOB_ReadPortPin(TN_CHG_DET_PBPIN) == TN_CHG_DET_PBPIN)
#define GET_COMM_UP()  ( GPIOA_ReadPortPin(TN_SHINT_PAPIN) == TN_SHINT_PAPIN)
#define GET_LOAD_DET()  ( GPIOB_ReadPortPin(TN_LOAD_DET_PBPIN) == TN_LOAD_DET_PBPIN)

#if(AFE_CHIP_SELECT == 3)
#define GET_AFE_ON_STAT()  ( MCU_GPIO_GetBit(TN_AFE_ON_PORT,TN_AFE_ON_PIN))
#endif


#if(AFE_CHIP_SELECT ==1)
    #define AFE_OPEN_DMOS()  Sh_OpenDsgMos()
    #define AFE_CLOSE_DMOS() Sh_ShutDDsgMos()
    #define AFE_OPEN_CMOS()  Sh_OpenChgMos()
    #define AFE_CLOSE_CMOS() Sh_ShutDChgMos()
    #define AFE_EN_SHIPMODE() SH_ENABLE_SHIPMODE()
    #define AFE_DIS_SHIPMODE() SH_DISABLE_SHIPMODE()
    #define AFE_SET_RUNMODE(mode) Sh_SetRunMode(mode)
    #define AFE_SET_BALANCE(cell) Sh_SetBalance(cell)
    #define AFE_DISABLE_BALANCE() Sh_DisableBalance()
#elif(AFE_CHIP_SELECT ==2)
    #define AFE_OPEN_DMOS()  Dvc10xx_AFE_OpenDsgMos()
    #define AFE_CLOSE_DMOS() Dvc10xx_AFE_CloseDsgMos()
    #define AFE_OPEN_CMOS()  Dvc10xx_AFE_OpenDsgMos_CloseChg()
    #define AFE_CLOSE_CMOS() Dvc10xx_AFE_OpenDsgMos_CloseChg()
    #define AFE_EN_SHIPMODE() 
    #define AFE_DIS_SHIPMODE() 
    #define AFE_SET_RUNMODE(mode) //Dvc10xx_AFE_SetRunMode(mode)
    #define AFE_SET_BALANCE(cell) //Sh_SetBalance(cell)
#elif(AFE_CHIP_SELECT ==3)
    #define AFE_OPEN_DMOS()   Sh_OpenDsgMos()
    #define AFE_CLOSE_DMOS()  Sh_ShutDDsgMos()
    #define AFE_OPEN_CMOS()   Sh_OpenChgMos()
    #define AFE_CLOSE_CMOS()  Sh_ShutDChgMos()
    #define AFE_EN_SHIPMODE() Sh_SetRunMode(1)
    #define AFE_DIS_SHIPMODE() Sh_SetRunMode(0)
    #define AFE_SET_RUNMODE(mode) Sh_SetRunMode(mode)
    #define AFE_SET_BALANCE(cell) Sh_SetBalance(cell)
    #define AFE_DISABLE_BALANCE()     Sh_DisableBalance()
#endif


//小牛uart通信命令数据结构表
typedef struct stc_niuCommdTable {
    uint8_t User;                   //用户权限
    uint8_t BMS_ID;                 //设备ID,BCD
    uint8_t S_Ver;                  //软件版本,BCD
    uint8_t H_Ver;                  //硬件版本,BCD
    uint8_t P_Pwrd[4];              //参数保护密码
    uint8_t Bat_Tytp;               //电芯型号代码,BCD
    uint8_t SN_ID[16];              //设备序列号
    uint8_t OC_Vlt_P;               //过充保护电压 0.1V ,最大25.5V
    uint8_t ROC_Vlt;                //过充恢复电压 0.1V ,最大25.5V
    uint8_t Dc_Vlt_P;               //过放保护电压 0.1V ,最大25.5V
    uint8_t RDc_Vlt_P;              //过放恢复电压 0.1V，最大25.5V
    uint8_t C_B_Vlt;                //充电均衡起始电压 0.1V
    uint8_t C_Cur_P;               //充电保护电流 1A
    uint8_t Dc_Cur_P;               //放电保护电流 1A
    uint8_t Isc_P;                  //短路保护电流 A，最大255A
    uint8_t DcOTp_P;                //放电高温保护 °C ，最大255℃
    uint8_t RDcOTp_P;               //放电恢复高温 °C ，最大255℃
    uint8_t COTp_P;                 //充电高温保护 °C ，最大255℃
    uint8_t RCoTp_P;                //充电恢复高温保护 °C ，最大255℃
    uint8_t DcLTp_P;                //放电低温保护 °C ，最大255℃
    uint8_t RDcLTp_P;               //放电恢复低温保护 °C ，最大255℃
    uint8_t NCom_DC_En;             //无通信放电使能  0x02 关闭 0x01打开
    uint8_t Rated_Vlt;              //额定电压 1V
    uint8_t Bat_To_Cap[2];          //电池可用容量FCC 十进制， mAh 0mAh～65000mAh
    //uint8_t Bat_To_CapL;
    uint8_t C_Cont[2];              //已充电次数
    uint8_t To_Vlt_RT[2];           //总电压
    //uint8_t To_Vlt_RTL;           //总电压
    uint8_t C_Cur_RT[2];            //充电电流H
    //uint8_t C_Cur_RTL;            //充电电流L
    uint8_t Dc_Cur_RT[2];           //放电电流H
    //uint8_t Dc_Cur_RTL;           //放电电流L
    uint8_t SOC_RT;                 //SOC 估算 1-100%
    uint8_t Bat_Sta_RT[2];          //实时电池状态
    uint8_t DC_Fl_T_RT;             //充满电剩余时间   0.1h 0-120, 0xFF 关闭
    uint8_t Tp1_RT[4];              //当前温度 1,2,3,4 十进制， ℃
    uint8_t Tp5_RT;                 //MOS 温度
    uint8_t G_Vlt_RT[48];           //24 串电芯电压 2个字节，高位为前个字节 0.001V 65.535V
    uint8_t S_Ver_N[8];             //新软件版本
    uint8_t H_Ver_N[8];             //新硬件版本
    uint8_t Blance_T[2];            //0.1℃ 6553.5℃
    uint8_t Cell_Num;               //电池串数
    uint8_t Cell_Vmax[2];           //最大电池电压 0.001V
    uint8_t Cell_Vmin[2];           //最低电池电压 0.001V
    uint8_t Cell_Vmax_Num;          //最高单体电压位置
    uint8_t Cell_Vmin_Num;          //最低单体电压位置
    uint8_t Cell_Vdiff[2];          //单体电压差 0.001V
    uint8_t Cell_Tmax[2];           //电芯最高温度值 0.1℃
    uint8_t Cell_Tmin[2];           //电芯最低温度值 0.1℃
    uint8_t Cell_Tmax_Num;          //最高温度位置
    uint8_t Cell_Tmin_Num;          //最低温度位置
    uint8_t Cell_Tdiff[2];          //电芯温度差值 0.1℃
    uint8_t LifeValue;              //心跳信号 1s
    uint8_t SOC_High_Precision;     //高精度SOC 0.40% 0-100%
    uint8_t SOH_RT;                 //电池健康状态 1-100%
    uint8_t ChargDemandVolt[2];     //充电需求电压 0.01V 655.35V
    uint8_t ChargDemandCurrent[2];  //充电需求电流 0.01A 655.35A
    uint8_t DisCharg_Max_P[2];      //当前最大允许放电功率
    uint8_t Charg_Max_P[2];
    uint8_t Blance_Status[4];       //各通道均衡开启状态
                                                        //bit7： V1 均衡开启状态
                                                        //bit6： V2 均衡开启状态
                                                        //. . .
                                                        //bit1:V23 均衡开启状态
                                                        //bit0:V24 均衡开启状态
    uint8_t MOS_Ctrol;              //MOS 控制   0：开路 1：闭合
                                                //bit7：预放电 MOS 闭合
                                                //bit6： 充电 MOS 闭合/继电器吸合
                                                //bit5： 放电 MOS 闭合/继电器吸合（预留）
    uint8_t MOS_Status;             //MOS 状态   0：开路 1：闭合
                                                // bit7： 预放电 MOS 闭合/继电器吸合（继电器版本）
                                                // bit6： 充电 MOS 闭合/继电器吸合（预留）
                                                // bit5： 放电 MOS 闭合
                                                // bit4： 系统接入状态
                                                // bit3： ACC 接入状态
                                                // bit2：充电器接入状态
    uint8_t Reserve9;             //预留字段 160
    uint8_t Reserve32[32];           //预留32字节160+32 = 192

    uint8_t Ratio_KvH;               //电压校准系数Kv高字节，address 160+32 = 192 = C0
    uint8_t Ratio_KvL;               //电压校准系数Kv低字节

    uint8_t Ratio_OffsetvH;          //电压校准系数offset高字节
    uint8_t Ratio_OffsetvL;          //电压校准系数offset低字节

    uint8_t Ratio_KcH;               //电流校准系数Kc高字节
    uint8_t Ratio_KcL;               //电流校准系数Kc低字节

    uint8_t Ratio_OffsetcH;          //电流校准系数offset高字节
    uint8_t Ratio_OffsetcL;          //电流校准系数offset低字节 192+8 = 200

    uint8_t TN_S_Ver_N[8];           //天能内部软件版本号
    uint8_t commMode;                //串口通信的模式，默认为0，当ACC上拉后，自动切换到一线通发送模式，如果设置为1，一直为串口模式
    //内部调试信息
    AlgEnginerIntnel_t algInfo;
    AlgEnginer_t    algengInfo;


}stc_niuCommdTable_t;

extern stc_niuCommdTable_t niuCommdTable;
extern uint8_t flagIntEnterType;

void NiuLogicInit(void);
void NiuLogicRun(void);



#endif
