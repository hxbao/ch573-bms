#ifndef  __UART0_H
#define  __UART0_H

#include "includes.h"


#define ENABLE_485_TX_PIN() MCU_GPIO_SetBit(TN_485_TXEN_PORT,TN_485_TXEN_PIN)
#define DISABLE_485_TX_PIN() MCU_GPIO_ClrBit(TN_485_TXEN_PORT,TN_485_TXEN_PIN)

typedef void (*pf_RxCallback)(uint8_t rData);


extern uint8_t Uart0_RxBuf[256];
extern uint8_t FlagUart0Inited;
extern uint16_t Uart0RxCount;

#if(PROJECT_ID == 2)
void Uart0Init4TY(pf_RxCallback callback);
#elif(PROJECT_ID == 4)
void Uart0Init4TY(pf_RxCallback callback);
#endif

void Uart0Init(pf_RxCallback callback);
void Uart0DeInit(void);
void Uart0SendData(uint8_t *pData, uint16_t len);
uint16_t Uart0ReceiveData(uint8_t *pData);

#if(MCU_LIB_SELECT == 1)
#define DISABLE_UART_RXINT() Uart_DisableIrq(M0P_UART0,UartRxIrq)
#define ENABLE_UART_RXINT()  Uart_EnableIrq(M0P_UART0,UartRxIrq)
#elif(MCU_LIB_SELECT == 2)
#define DISABLE_UART_RXINT() USART_ConfigInt(USART1,USART_INT_RXDNE,DISABLE)
#define ENABLE_UART_RXINT()  USART_ConfigInt(USART1,USART_INT_RXDNE,ENABLE)
#endif

#endif
