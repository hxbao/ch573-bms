#ifndef _INCLUDES_H
#define _INCLUDES_H

#include "config.h"
#include "config-F0-n32.h"
#include "string.h"
#ifdef UNIT_TEST_EN
void UnitTestProcess(void);
#endif



#if(MCU_LIB_SELECT ==2)


#include "afe.h"
#if(AFE_CHIP_SELECT == 1)
#include "sh367309.h"
#elif(AFE_CHIP_SELECT == 3)

#endif
#include "commdif_ty.h"
#include "haltimer.h"
#include "AlgEngine.h"
#include "mcuEEPROM.h"
#include "niuLogic.h"
#include "commdif_niu.h"
#include "haluart0.h"
#include "TnOneBus.h"
#include "niuOneBus.h"
#include "HAL.h"
#include "GPIO.h"
#include "ADC.h"
#include "common.h"
#include "twi.h"
#include "mcuAdapter.h"
#include "balance.h"
#include "protect.h"
#include "UART3.h"



#endif

#endif
