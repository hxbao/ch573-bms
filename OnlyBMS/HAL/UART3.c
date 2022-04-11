/*
 * UART3.c
 *
 *  Created on: Apr 7, 2022
 *      Author: hxbao
 */
#include "CH57x_common.h"
#include "UART3.h"

pf_RxCallback RxCallback;


static void HandleRecvData(uint8_t data)
{
    RxCallback(data);
}


void Uart3Init(pf_RxCallback callback)
{
    //小牛版需要偶校验
    UART3_DefInit();
    UART3_BaudRateCfg(9600);
    UART3_INTCfg(ENABLE, RB_IER_RECV_RDY);
    PFIC_EnableIRQ(UART3_IRQn);

    //接收回调函数
    RxCallback = callback;
}

void Uart3SendData(uint8_t *pdata,uint16_t len)
{
    UART3_INTCfg(DISABLE,RB_IER_RECV_RDY);
    UART3_SendString(pdata, len);
    mDelayuS(100);
    UART3_CLR_RXFIFO();
    UART3_INTCfg(ENABLE,RB_IER_RECV_RDY);


}

__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void UART3_IRQHandler(void)
{
    uint8_t data;
     switch(UART3_GetITFlag())
     {
         case UART_II_LINE_STAT:
             //UART3_GetLinSTA();
             break;

         case UART_II_RECV_RDY:
         case UART_II_RECV_TOUT:

             for(uint8_t i = 0; i < R8_UART3_RFC; i++)
             {
                 data = R8_UART3_RBR;
                 HandleRecvData(data);
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
