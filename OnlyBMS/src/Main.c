/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.0
 * Date               : 2021/03/09
 * Description        : adc����ʾ���������¶ȼ�⡢��ͨ����⡢���ͨ����⡢TouchKey��⡢�жϷ�ʽ������
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
 * @brief   ���Գ�ʼ��
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
 * @brief   ������
 *
 * @return  none
 */
int main()
{
    uint32_t sysclk;
    uint32_t count = 0;


    SetSysClock(CLK_SOURCE_PLL_60MHz);
    sysclk = GetSysClock();
    /* ���ô��ڵ��� */
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

