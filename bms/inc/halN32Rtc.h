#ifndef  __N32_RTC_H
#define  __N32_RTC_H

#include "includes.h"

#if(MCU_LIB_SELECT == 2)

void RTC_PrescalerConfig(void);
void WakeUpClockSelect(uint8_t WKUPClkSrcSel);
void RTC_CLKSourceConfig(uint8_t ClkSrc, uint8_t FirstLastCfg, uint8_t RstBKP);
void EXTI20_RTCWKUP_Configuration(FunctionalState Cmd);
ErrorStatus RTC_DateRegulate(void);
void RTC_DateAndTimeDefaultVale(void);
ErrorStatus RTC_TimeRegulate(void);

#endif

#endif