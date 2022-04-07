#ifndef _MCU_ADAPTER_H
#define _MCU_ADAPTER_

#include "includes.h"
#if(MCU_LIB_SELECT == 1)
#define MCU_GPIO_SetBit Gpio_SetIO
#define MCU_GPIO_ClrBit Gpio_ClrIO
#define MCU_GPIO_GetBit Gpio_GetInputIO
#define MCU_GPIO_ReadOutPutBit Gpio_ReadOutputIO
#define MCU_Gpio_EnableIrq Gpio_EnableIrq

#define TWI_I2C  M0P_I2C1

#elif(MCU_LIB_SELECT == 2)

#define MCU_GPIO_SetBit GPIO_SetBits
#define MCU_GPIO_ClrBit GPIO_ResetBits
#define MCU_GPIO_GetBit GPIO_ReadInputDataBit
#define MCU_GPIO_ReadOutPutBit GPIO_ReadOutputDataBit


enum
{
    SYSCLK_PLLSRC_HSI,
    SYSCLK_PLLSRC_HSIDIV2,
    SYSCLK_PLLSRC_HSI_PLLDIV2,
    SYSCLK_PLLSRC_HSIDIV2_PLLDIV2,
    SYSCLK_PLLSRC_HSE,
    SYSCLK_PLLSRC_HSEDIV2,
    SYSCLK_PLLSRC_HSE_PLLDIV2,
    SYSCLK_PLLSRC_HSEDIV2_PLLDIV2,
};

#define TWI_I2C  I2C2

#endif //MCU_LIB_SELECT

#if(USE_485_IF == 1)

#define SWITCH_485_POWER_ON   do{MCU_GPIO_SetBit(TN_485_COMM_ON_PORT,TN_485_COMM_ON_PIN);}while(0);
#define SWITCH_485_POWER_OFF  do{MCU_GPIO_ClrBit(TN_485_COMM_ON_PORT,TN_485_COMM_ON_PIN);}while(0);


#endif //USE_485_IF


void MCU_SetNVOffset(void);
void GPIOInit(void);
void MCU_GpioKillMeInit(void);
void Clk_Config(void);
void Iwdt_Config(void);
void Iwdt_Start(void);
void Iwdt_Feed(void);
void RTC_ConfigInit(void);
uint8_t Rtc_GetWakeUpFlag(void);
uint16_t Rtc_GetCount();
void Rtc_SetCount(uint16_t val);
void Rtc_ClrWakeUpFlag(void);
void RTC_AlarmITEnable(void);
void RTC_AlarmITDisable(void);
void ADC_PortInit(void);
void ADC_Config(void);
void ADC_DeConfig(void);
void Mcu_PeripherGoSleep(void);
void Mcu_PeripherWakeFromSleep(void);
void ADC_GetMcuAdcInfo(uint16_t *buf,uint16_t *mosTemp,uint16_t *balTemp,uint16_t *preDsgCur);
void TWI_Init(void);
uint8_t I2C_MasterReadData(uint8_t SlaveID,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);
uint8_t I2C_MasterWriteData(uint8_t SlaveID,uint8_t u8Addr,uint8_t *pu8Data,uint32_t u32Len);

#endif