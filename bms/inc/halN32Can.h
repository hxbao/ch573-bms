
#ifndef _HAL_N32_CAN_H
#define _HAL_N32_CAN_H

#include "includes.h"

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

typedef void (*pf_CANRxCallback)(CanRxMessage RxMessage);

#define  CAN_BAUDRATE_1M            1
#define  CAN_BAUDRATE_500K          2
#define  CAN_BAUDRATE_250K          3
#define  CAN_BAUDRATE_125K          4
#define  CAN_BAUDRATE               CAN_BAUDRATE_250K

#if(CAN_BAUDRATE==CAN_BAUDRATE_1M)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_5tq
#define   CAN_BIT_BS2               CAN_TBS2_2tq
#define   CAN_BAUDRATEPRESCALER     2 
#elif(CAN_BAUDRATE==CAN_BAUDRATE_500K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_10tq
#define   CAN_BIT_BS2               CAN_TBS2_5tq
#define   CAN_BAUDRATEPRESCALER     2
#elif(CAN_BAUDRATE==CAN_BAUDRATE_250K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_10tq
#define   CAN_BIT_BS2               CAN_TBS2_5tq
#define   CAN_BAUDRATEPRESCALER     4 
#elif(CAN_BAUDRATE==CAN_BAUDRATE_125K)
#define   CAN_BIT_RSJW              CAN_RSJW_1tq
#define   CAN_BIT_BS1               CAN_TBS1_10tq
#define   CAN_BIT_BS2               CAN_TBS2_5tq
#define   CAN_BAUDRATEPRESCALER     8
#endif

#define CAN_TXDLC_8    ((uint8_t)8)
#define CAN_FILTERNUM0 ((uint8_t)0)
#define CAN_FILTERNUM1 ((uint8_t)1)

void BxCanPortInit(void);
void CAN_Config(uint32_t CAN_BaudRate);
void BxCanConfig(pf_CANRxCallback callback,uint16_t *idListTab,uint8_t idListNum);
uint8_t CANTxMessage(CAN_Module* CANx,CanTxMessage* TxMessage);
void RYCAN_RxProcess(CanRxMessage rxm);
#endif 