#ifndef _MCU_ADAPTER_H
#define _MCU_ADAPTER_

#include "includes.h"
#if(MCU_LIB_SELECT == 1)

#define TWI_I2C  M0P_I2C1

#elif(MCU_LIB_SELECT == 2)



#endif //MCU_LIB_SELECT

#if(USE_485_IF == 1)

#define SWITCH_485_POWER_ON   do{MCU_GPIO_SetBit(TN_485_COMM_ON_PORT,TN_485_COMM_ON_PIN);}while(0);
#define SWITCH_485_POWER_OFF  do{MCU_GPIO_ClrBit(TN_485_COMM_ON_PORT,TN_485_COMM_ON_PIN);}while(0);


#endif //USE_485_IF

#define NTC0_ADC_CH CH_EXTIN_1
#define NTC1_ADC_CH CH_EXTIN_2
#define MOS_ADC_CH  CH_EXTIN_3
#define BAL_ADC_CH  CH_EXTIN_4


void RTC_ConfigInit(void);
uint8_t Rtc_GetWakeUpFlag(void);
uint16_t Rtc_GetCount();
void Rtc_SetCount(uint16_t val);
void Rtc_ClrWakeUpFlag(void);
void RTC_AlarmITEnable(void);
void RTC_AlarmITDisable(void);
//void ADC_PortInit(void);
//void ADC_Config(void);

void Mcu_PeripherGoSleep(void);
void Mcu_PeripherWakeFromSleep(void);
void ADC_GetMcuAdcInfo(uint16_t *buf,uint16_t *mosTemp,uint16_t *balTemp,uint16_t *preDsgCur);
void TWI_Init(void);
uint8_t I2C_MasterReadData(uint8_t SlaveID,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
uint8_t I2C_MasterWriteData(uint8_t SlaveID,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);

#endif
