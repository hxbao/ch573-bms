#ifndef __TIMER_H
#define __TIMER_H

#include "includes.h"

#define TMR_COUNT	15		

#define TMR_SOC_ENGINE 0
#define TMR_MAIN       1
#define TMR_AFE        2
#define TMR_ONEBUS_CHECK 3
#define TMR_NIULOGIC_20S  4
#define TMR_UART_SEND    5
//预放电延时控制
#define TMR_PRE_DELAY    6

#define TMR_COMM_INT_DELAY 7
#define TMR_DVCAFE_POLL 8

#define TMR_PROTECT_DELAY 9
#define TMR_UINT_TEST     10
#define TMR_BLANCE_PWM    11

typedef enum
{
	TMR_ONCE_MODE = 0,		
	TMR_AUTO_MODE = 1		
}TMR_MODE_E;

typedef void (*pFun)(void);


typedef struct
{
	volatile uint8_t Mode;		
	volatile uint8_t Flag;		
	volatile uint32_t Count;	
	volatile uint32_t PreLoad;	
	pFun Callback;
}SOFT_TMR;


void bsp_InitTimer(void);
void bsp_DelayMS(uint32_t n);
void bsp_DelayUS(uint32_t n);
void bsp_StartTimer(uint8_t _id, uint32_t _period);
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period);
void bsp_StartCallBackTimer(uint8_t _id,pFun _callFun, uint32_t _period);
void bsp_StopTimer(uint8_t _id);
uint8_t bsp_CheckTimer(uint8_t _id);
uint64_t bsp_GetRunTime(void);
void Timer_UpdateCallBack(void);
void bsp_StartFreeRunTime(void);
uint32_t bsp_GetFreeRunTime(void);


#endif
