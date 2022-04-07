#ifndef _HAL_SPI_H_
#define _HAL_SPI_H_
#include "includes.h"


#if (MCU_LIB_SELECT == 1)
    #define SPI_PRESCALER_256 SpiClkMskDiv128
    #define SPI_PRESCALER_4   SpiClkMskDiv4
#elif (MCU_LIB_SELECT == 2)
    #define SPI_PRESCALER_256 SPI_BR_PRESCALER_256
    #define SPI_PRESCALER_4   SPI_BR_PRESCALER_4
#endif

void SPI2_GpioInit(void);
void SPI2_Init(uint16_t prescaler);
uint8_t SPI2_WriteReadByte(uint8_t TxData, uint8_t *RdData);


#endif