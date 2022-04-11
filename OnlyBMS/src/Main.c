/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2021/03/09
 * Description        : adc采样示例，包括温度检测、单通道检测、差分通道检测、TouchKey检测、中断方式采样。
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "CH57x_common.h"
#include "GPIO.h"
#include "includes.h"

uint16_t abcBuff[40];

volatile uint8_t adclen;
volatile uint8_t DMA_end = 0;

/*********************************************************************
 * @fn      DebugInit
 *
 * @brief   调试初始化
 *
 * @return  none
 */
void DebugInit(void)
{
    GPIOA_SetBits(GPIO_Pin_9);
    GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
    GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
    UART1_DefInit();

    GPIOA_SetBits(bRXD1);
    GPIOA_ModeCfg(bRXD1, GPIO_ModeIN_PU);
    UART1_DefInit();


    //enable interupt
    UART1_INTCfg(ENABLE, RB_IER_RECV_RDY|RB_IER_LINE_STAT);
    PFIC_EnableIRQ(UART1_IRQn);
}


__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART1_IRQHandler(void)
{
    uint8_t data;
    switch(UART1_GetITFlag())
    {
        case UART_II_LINE_STAT:
            UART1_GetLinSTA();
            break;

        case UART_II_RECV_RDY:
        case UART_II_RECV_TOUT:

            for(uint8_t i = 0; i < R8_UART1_RFC; i++)
            {
                data = R8_UART1_RBR;
                DebugHandleRecvData(data);
                NIU_ModbusRecvHandle(data);
//                DebugHandleRecvData(R8_UART1_RBR);
//                NIU_ModbusRecvHandle(R8_UART1_RBR);
            }
            break;

        case UART_II_THR_EMPTY:
            break;
        case UART_II_MODEM_CHG:
            break;
        default:
            break;
    }
}




/*********************************************************************
 * @fn      main
 *
 * @brief   主函数
 *
 * @return  none
 */
int main()
{
    uint32_t sysclk;



    SetSysClock(CLK_SOURCE_PLL_60MHz);
    sysclk = GetSysClock();
    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("sysclk = %d\n",sysclk);

    Hal_GpioInit();

    bsp_InitTimer();
    UnitTestInit();
    NiuLogicInit();
    while(1)
    {
        NiuLogicRun();
        UnitTestProcess();
    }
}

