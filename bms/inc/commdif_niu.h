#ifndef _COMMD_H
#define _COMMD_H

#include "includes.h"
//数据表需同步处理标志 0-无须处理   1-参数需要同步写入
extern uint8_t NiuMdSycWriteFlg;

#if (PROJECT_ID != 3)
#define NIu_ModbusSendAck(a,b) Uart0SendData(a,b)
#elif(PROJECT_ID ==3)
#define NIu_ModbusSendAck(a,b) RYCAN_SendData(a,b)
#endif

void Niu_ModbufFifoClr(void);
void NIU_ModbusRecvHandle(uint8_t rdata);
void NiuModbusPoll(void);


#endif