/*
 * HLOnebus.h
 *
 *  Created on: Apr 29, 2022
 *      Author: hxbao
 */

#ifndef BMS_INC_HLONEBUS_H_
#define BMS_INC_HLONEBUS_H_
#include "includes.h"

#if(ONEBUS_TYPE == 3)
void HL_OneBusInit(void);
void HL_OneBusProcess(void);
#endif


#endif /* BMS_INC_HLONEBUS_H_ */
