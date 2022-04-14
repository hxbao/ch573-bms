/*
 * GPIO.c
 *
 *  Created on: Apr 4, 2022
 *      Author: hxbao
 */
#include "GPIO.h"
#include "CH57x_common.h"

void Hal_GpioInit()
{

    // ‰≥ˆ
    GPIOA_SetBits(TN_LED_PAPIN);
    GPIOA_ModeCfg(TN_LED_PAPIN, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(TN_PREEN_PBPIN, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(TN_SHSHIP_PAPIN, GPIO_ModeOut_PP_5mA);

    GPIOA_ResetBits(TN_NTC0_POWER_PAPIN);
    GPIOA_ModeCfg(TN_NTC0_POWER_PAPIN, GPIO_ModeOut_PP_5mA);
    GPIOA_ModeCfg(TN_ONE_TX_PAPIN, GPIO_ModeOut_PP_20mA);
    GPIOB_ModeCfg(TN_PREEN_PBPIN, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(TN_VPRO_CON_PBPIN, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(TN_I2C1_SCL_PBPIN, GPIO_ModeOut_PP_5mA);
    GPIOB_ModeCfg(TN_I2C1_SDA_PBPIN, GPIO_ModeOut_PP_5mA);

    // ‰»Î
    GPIOB_ModeCfg(TN_ACC_PBPIN, GPIO_ModeIN_Floating);
    GPIOA_ModeCfg(TN_SHINT_PAPIN, GPIO_ModeIN_Floating);

    //uart3
    GPIOA_SetBits(bTXD3);
    GPIOA_ModeCfg(bTXD3, GPIO_ModeOut_PP_20mA);

    //uart rx io
    GPIOA_SetBits(bRXD3);
    GPIOA_ModeCfg(bRXD3, GPIO_ModeIN_PU);
//    UART3_DefInit();
//    //enable interupt
//    UART3_INTCfg(ENABLE, RB_IER_RECV_RDY | RB_IER_LINE_STAT);
//    PFIC_EnableIRQ(UART3_IRQn);

}

