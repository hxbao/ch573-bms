#ifndef  __Uart3_H
#define  __Uart3_H

#include "includes.h"


#define ENABLE_485_TX_PIN() MCU_GPIO_SetBit(TN_485_TXEN_PORT,TN_485_TXEN_PIN)
#define DISABLE_485_TX_PIN() MCU_GPIO_ClrBit(TN_485_TXEN_PORT,TN_485_TXEN_PIN)

typedef void (*pf_RxCallback)(uint8_t rData);


extern uint8_t Uart3_RxBuf[256];
extern uint8_t FlagUart3Inited;
extern uint16_t Uart3RxCount;

#if(PROJECT_ID == 2)
void Uart3Init4TY(pf_RxCallback callback);
#elif(PROJECT_ID == 4)
void Uart3Init4TY(pf_RxCallback callback);
#endif

void Uart3Init(pf_RxCallback callback);
void Uart3DeInit(void);
void Uart3SendData(uint8_t *pData, uint16_t len);
uint16_t Uart3ReceiveData(uint8_t *pData);

#if(MCU_LIB_SELECT == 1)

#elif(MCU_LIB_SELECT == 2)
#define DISABLE_UART_RXINT()   UART3_INTCfg(DISABLE, RB_IER_RECV_RDY);         //USART_ConfigInt(USART1,USART_INT_RXDNE,DISABLE)
#define ENABLE_UART_RXINT()  UART3_INTCfg(ENABLE, RB_IER_RECV_RDY); //USART_ConfigInt(USART1,USART_INT_RXDNE,ENABLE)
#endif

#endif
