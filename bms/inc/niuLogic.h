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

#define SWITCH_LED_ON() (MCU_GPIO_ClrBit(TN_LED_PORT, TN_LED_PIN))
#define SWITCH_LED_OFF() (MCU_GPIO_SetBit(TN_LED_PORT, TN_LED_PIN))

#define SWITCH_PRED_ON() (MCU_GPIO_SetBit(TN_PREEN_PORT, TN_PREEN_PIN))
#define SWITCH_PRED_OFF() (MCU_GPIO_ClrBit(TN_PREEN_PORT, TN_PREEN_PIN))

#define SWITCH_KILLME_ON() do {   \
									MCU_GPIO_SetBit(TN_KILLME_EN_PORT, TN_KILLME_EN_PIN);  \
									MCU_GPIO_ClrBit(TN_KILLME_PORT, TN_KILLME_PIN);    \
							  }while(0)

#define SWITCH_KILLME_OFF() do {   \
									MCU_GPIO_SetBit(TN_KILLME_PORT, TN_KILLME_PIN);  \
									MCU_GPIO_ClrBit(TN_KILLME_EN_PORT , TN_KILLME_EN_PIN );    \
							  }while(0)

#define GET_DMOS_FB()  ( MCU_GPIO_GetBit(TN_DMOSFB_PORT,TN_DMOSFB_PIN))
#define GET_CMOS_FB()  ( MCU_GPIO_GetBit(TN_CMOSFB_PORT,TN_CMOSFB_PIN))
#define GET_ACC()  ( MCU_GPIO_GetBit(TN_ACC_PORT,TN_ACC_PIN))
#define GET_CHGDET()  ( MCU_GPIO_GetBit(TN_CHG_DET_PORT,TN_CHG_DET_PIN))
#define GET_WAKEUP()  ( MCU_GPIO_GetBit(TN_WAKE_UP_PORT,TN_WAKE_UP_PIN))
#define GET_COMM_UP()  ( MCU_GPIO_GetBit(TN_SHINT_PORT,TN_SHINT_PIN))
#define GET_LOAD_DET()  ( MCU_GPIO_GetBit(TN_LOAD_DET_PORT,TN_LOAD_DET_PIN))

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


//??????uart???????????????????????????
typedef struct stc_niuCommdTable {
    uint8_t User;                   //????????????
    uint8_t BMS_ID;                 //??????ID,BCD
    uint8_t S_Ver;                  //????????????,BCD
    uint8_t H_Ver;                  //????????????,BCD
    uint8_t P_Pwrd[4];              //??????????????????
    uint8_t Bat_Tytp;               //??????????????????,BCD
    uint8_t SN_ID[16];              //???????????????
    uint8_t OC_Vlt_P;               //?????????????????? 0.1V ,??????25.5V
    uint8_t ROC_Vlt;                //?????????????????? 0.1V ,??????25.5V
    uint8_t Dc_Vlt_P;               //?????????????????? 0.1V ,??????25.5V
    uint8_t RDc_Vlt_P;              //?????????????????? 0.1V?????????25.5V
    uint8_t C_B_Vlt;                //???????????????????????? 0.1V
    uint8_t C_Cur_P;               //?????????????????? 1A
    uint8_t Dc_Cur_P;               //?????????????????? 1A
    uint8_t Isc_P;                  //?????????????????? A?????????255A
    uint8_t DcOTp_P;                //?????????????????? ??C ?????????255???
    uint8_t RDcOTp_P;               //?????????????????? ??C ?????????255???
    uint8_t COTp_P;                 //?????????????????? ??C ?????????255???
    uint8_t RCoTp_P;                //???????????????????????? ??C ?????????255???
    uint8_t DcLTp_P;                //?????????????????? ??C ?????????255???
    uint8_t RDcLTp_P;               //???????????????????????? ??C ?????????255???
    uint8_t NCom_DC_En;             //?????????????????????  0x02 ?????? 0x01??????
    uint8_t Rated_Vlt;              //???????????? 1V
    uint8_t Bat_To_Cap[2];          //??????????????????FCC ???????????? mAh 0mAh???65000mAh
    //uint8_t Bat_To_CapL;
    uint8_t C_Cont[2];              //???????????????
    uint8_t To_Vlt_RT[2];           //?????????
    //uint8_t To_Vlt_RTL;           //?????????
    uint8_t C_Cur_RT[2];            //????????????H
    //uint8_t C_Cur_RTL;            //????????????L
    uint8_t Dc_Cur_RT[2];           //????????????H
    //uint8_t Dc_Cur_RTL;           //????????????L
    uint8_t SOC_RT;                 //SOC ?????? 1-100%
    uint8_t Bat_Sta_RT[2];          //??????????????????
    uint8_t DC_Fl_T_RT;             //?????????????????????   0.1h 0-120, 0xFF ??????
    uint8_t Tp1_RT[4];              //???????????? 1,2,3,4 ???????????? ???
    uint8_t Tp5_RT;                 //MOS ??????
    uint8_t G_Vlt_RT[48];           //24 ??????????????? 2????????????????????????????????? 0.001V 65.535V
    uint8_t S_Ver_N[8];             //???????????????
    uint8_t H_Ver_N[8];             //???????????????
    uint8_t Blance_T[2];            //0.1??? 6553.5???
    uint8_t Cell_Num;               //????????????
    uint8_t Cell_Vmax[2];           //?????????????????? 0.001V
    uint8_t Cell_Vmin[2];           //?????????????????? 0.001V
    uint8_t Cell_Vmax_Num;          //????????????????????????
    uint8_t Cell_Vmin_Num;          //????????????????????????
    uint8_t Cell_Vdiff[2];          //??????????????? 0.001V
    uint8_t Cell_Tmax[2];           //????????????????????? 0.1???
    uint8_t Cell_Tmin[2];           //????????????????????? 0.1???
    uint8_t Cell_Tmax_Num;          //??????????????????
    uint8_t Cell_Tmin_Num;          //??????????????????
    uint8_t Cell_Tdiff[2];          //?????????????????? 0.1???
    uint8_t LifeValue;              //???????????? 1s
    uint8_t SOC_High_Precision;     //?????????SOC 0.40% 0-100%
    uint8_t SOH_RT;                 //?????????????????? 1-100%
    uint8_t ChargDemandVolt[2];     //?????????????????? 0.01V 655.35V
    uint8_t ChargDemandCurrent[2];  //?????????????????? 0.01A 655.35A
    uint8_t DisCharg_Max_P[2];      //??????????????????????????????
    uint8_t Charg_Max_P[2];
    uint8_t Blance_Status[4];       //???????????????????????????  
                                                        //bit7??? V1 ??????????????????
                                                        //bit6??? V2 ??????????????????
                                                        //. . .
                                                        //bit1:V23 ??????????????????
                                                        //bit0:V24 ??????????????????
    uint8_t MOS_Ctrol;              //MOS ??????   0????????? 1????????? 
                                                //bit7???????????? MOS ??????
                                                //bit6??? ?????? MOS ??????/???????????????
                                                //bit5??? ?????? MOS ??????/???????????????????????????
    uint8_t MOS_Status;             //MOS ??????   0????????? 1????????? 
                                                // bit7??? ????????? MOS ??????/????????????????????????????????????
                                                // bit6??? ?????? MOS ??????/???????????????????????????
                                                // bit5??? ?????? MOS ??????
                                                // bit4??? ??????????????????
                                                // bit3??? ACC ????????????
                                                // bit2????????????????????????
    uint8_t Reserve9;             //???????????? 160 
    uint8_t Reserve32[32];           //??????32??????160+32 = 192

    uint8_t Ratio_KvH;               //??????????????????Kv????????????address 160+32 = 192 = C0
    uint8_t Ratio_KvL;               //??????????????????Kv?????????

    uint8_t Ratio_OffsetvH;          //??????????????????offset?????????
    uint8_t Ratio_OffsetvL;          //??????????????????offset?????????

    uint8_t Ratio_KcH;               //??????????????????Kc?????????
    uint8_t Ratio_KcL;               //??????????????????Kc?????????

    uint8_t Ratio_OffsetcH;          //??????????????????offset?????????
    uint8_t Ratio_OffsetcL;          //??????????????????offset????????? 192+8 = 200 

    uint8_t TN_S_Ver_N[8];           //???????????????????????????
    uint8_t commMode;                //?????????????????????????????????0??????ACC??????????????????????????????????????????????????????????????????1????????????????????????
    //??????????????????
    AlgEnginerIntnel_t algInfo;
    AlgEnginer_t    algengInfo;


}stc_niuCommdTable_t;

extern stc_niuCommdTable_t niuCommdTable;
extern uint8_t flagIntEnterType;

void NiuLogicInit(void);
void NiuLogicRun(void);



#endif