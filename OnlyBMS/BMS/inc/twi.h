/*
 * twi.h
 *
 *  Created on: 2020Äê12ÔÂ22ÈÕ
 *      Author: hxbao
 */

#ifndef INCLUDE_TWI_H_
#define INCLUDE_TWI_H_
#include "includes.h"



#define I2C_SPEED              50000
#define I2C_SLAVE_ADDRESS7     0x34
#define SH305_I2C_SLAVE_ADDR   0x36

 
/* Defintions for the state of the DMA transfer */   
#define TWI_STATE_READY         0
#define TWI_STATE_BUSY          1
   
/* Maximum timeout value for counting before exiting waiting loop on DMA 
   Trasnfer Complete. This value depends directly on the maximum page size and
   the sytem clock frequency. */
#define TWI_TIMEOUT_MAX         0x005000


uint8_t SH309Twi_Write(uint8_t SlaveID, uint16_t WrAddr, uint8_t WrData);

uint8_t SH309Twi_Read(uint8_t SlaveID, uint16_t RdAddr, uint8_t Length,
		uint8_t *RdBuf);

uint8_t SH305Twi_Write(uint8_t SlaveID, uint16_t WrAddr, uint8_t WrData);
uint8_t SH305Twi_Read(uint8_t SlaveID, uint16_t RdAddr, uint8_t Length,
					  uint8_t *RdBuf);

#endif /* INCLUDE_TWI_H_ */