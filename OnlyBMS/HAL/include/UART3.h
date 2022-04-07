/*
 * UART3.h
 *
 *  Created on: Apr 7, 2022
 *      Author: hxbao
 */

#ifndef HAL_INCLUDE_UART3_H_
#define HAL_INCLUDE_UART3_H_

#include "CH57x_common.h"



typedef void (*pf_RxCallback)(uint8_t rData);




void Uart3Init(pf_RxCallback callback);



#endif /* HAL_INCLUDE_UART3_H_ */
