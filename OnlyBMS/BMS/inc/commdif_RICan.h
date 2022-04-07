#ifndef _COMMD_RYCAN_H
#define _COMMD_RYCAN_H
#include "includes.h"
#if(PROJECT_ID == 3)
#define TH_WARN_OV1   3700
#define TH_WARN_OV2   3600
#define TH_WARN_OV3   3500

#define TH_WARN_UV1   2300
#define TH_WARN_UV2   2500
#define TH_WARN_UV3   2600

#define TH_WARN_OCC1  20
#define TH_WARN_OCC2  18
#define TH_WARN_OCC3  10

#define TH_WARN_OCD1  100
#define TH_WARN_OCD2  95
#define TH_WARN_OCD3  90

#define TH_WARN_OCT1  53
#define TH_WARN_OCT2  50
#define TH_WARN_OCT3  45

#define TH_WARN_ODT1  68
#define TH_WARN_ODT2  65
#define TH_WARN_ODT3  60

#define TH_WARN_UCT1  -5
#define TH_WARN_UCT2  0
#define TH_WARN_UCT3  5

#define TH_WARN_UDT1  -20
#define TH_WARN_UDT2  -15
#define TH_WARN_UDT3  -10

#define TH_WARN_VDIFF1 1000
#define TH_WARN_VDIFF2 600
#define TH_WARN_VDIFF3 400

#define TH_WARN_LOW_SOC1 5
#define TH_WARN_LOW_SOC2 10
#define TH_WARN_LOW_SOC3 20

#define TH_WARN_MOS_OT1  82
#define TH_WARN_MOS_OT2  80
#define TH_WARN_MOS_OT3  75

void RYCAN_Init(void);
void RYCAN_SendData(uint8_t *buf,uint16_t len);
void RYCAN_TxProcess(void);

#endif

#endif