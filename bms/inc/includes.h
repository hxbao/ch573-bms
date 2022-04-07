#ifndef _INCLUDES_H
#define _INCLUDES_H

//#include "config-F0.h" //Huada
//#include "config-TY.h" //天瑞华大
//#include "config-F0-n32.h"
#include "config.h"
#include "string.h"
#ifdef UNIT_TEST_EN
void UnitTestProcess(void);
#endif

#if(MCU_LIB_SELECT ==1)
#include "ddl.h"
#include "afe.h"
#include "wdt.h"
#if(AFE_CHIP_SELECT == 1)
#include "sh367309.h"
#elif(AFE_CHIP_SELECT == 3)
#include "sh367305.h"
#endif
#include "commdif_ty.h"
#include "commdif_share485.h"
#include "haltimer.h"
#include "AlgEngine.h"
#include "common.h"
#include "sysctrl.h"
#include "uart.h"
#include "gpio.h"
#include "rtc.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "wdt.h"
#include "lpm.h"
#include "bgr.h"
#include "trim.h"
#include "flash.h"
#include "string.h"
#include "mcuEEPROM.h"
#include "iap.h"
#include "niuLogic.h"
#include "commdif_niu.h"
#include "haluart0.h"
#include "halspi.h"
#include "rchTrim.h"
#include "TnOneBus.h"
#include "ydOnebus.h"
#include "xrOnebus.h"
#include "zbOnebus.h"
#include "niuOneBus.h"
#include "twi.h"
#include "mcuadapter.h"
#include "dvc10xx.h"
#include "dvc10xx_afe.h"
#include "SEGGER_RTT.h"
#include "balance.h"
#include "protect.h"



#elif(MCU_LIB_SELECT ==2)

#define USE_STDPERIPH_DRIVER

#include "n32l40x.h"
#include "afe.h"
#if(AFE_CHIP_SELECT == 1)
#include "sh367309.h"
#elif(AFE_CHIP_SELECT == 3)
#include "sh367305.h"
#endif
#include "commdif_ty.h"
#include "haltimer.h"
#include "AlgEngine.h"
#include "common.h"
#include "mcuEEPROM.h"
#include "iap.h"
#include "SEGGER_RTT.h"
#include "niuLogic.h"
#include "commdif_niu.h"
#include "haluart0.h"
#include "Rchtrim.h"
#include "TnOneBus.h"
#include "ydOnebus.h"
#include "zbOnebus.h"
#include "xrOnebus.h"
#include "niuOneBus.h"
#include "fsdOnebus.h"
#include "twi.h"
#include "mcuAdapter.h"
#include "halspi.h"
#include "dvc10xx.h"
#include "dvc10xx_afe.h"
#include "halN32Rtc.h"
#include "halN32Can.h"
#include "commdif_RICan.h"
#include "balance.h"
#include "protect.h"


#endif

#endif