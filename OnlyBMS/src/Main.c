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
    uint32_t count = 0;


    SetSysClock(CLK_SOURCE_PLL_60MHz);
    sysclk = GetSysClock();
    /* 配置串口调试 */
    DebugInit();
    PRINT("Start @ChipID=%02X\n", R8_CHIP_ID);
    PRINT("sysclk = %d\n",sysclk);

    Hal_GpioInit();

    bsp_InitTimer();
    bsp_StartAutoTimer(0, 1000);
    NiuLogicInit();
    while(1)
    {
        //NiuLogicRun();
        if(bsp_CheckTimer(0))
        {
            PRINT("count =%d\n",count++);
        }
    }
}

