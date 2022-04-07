#ifndef	_Niu_ONEBUS_H
#define	_Niu_ONEBUS_H
#include "includes.h"

#if(ONEBUS_TYPE == 1)
#define SWITCH_TX_HIGH() MCU_GPIO_SetBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN)
#define SWITCH_TX_LOW()  MCU_GPIO_ClrBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN)


void Niu_OneBusInit(void);
uint8_t Niu_OneBusGetBusType(void);
void Niu_OneBusSendData(uint8_t *sndBuf, uint16_t dataLen);
void Niu_OneBusProcess(void);
void HAL_TIM_PeriodElapsedCallback(void);
#endif

#endif
