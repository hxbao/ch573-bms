/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

static volatile uint32_t s_uiDelayCount = 0;
static volatile uint8_t s_ucTimeOutFlag = 0;

/*  */
static SOFT_TMR s_tTmr[BSP_TMR_COUNT];

/*
 *
*/
__IO uint64_t g_iRunTime = 0;
uint32_t s_RunTime = 0;

static void bsp_SoftTimerDec(SOFT_TMR *_tmr);

/*
*********************************************************************************************************
*	
*	
*********************************************************************************************************
*/

void bsp_InitTimer(void)
{
  uint8_t i;

  /**/
  for (i = 0; i < BSP_TMR_COUNT; i++)
  {
    s_tTmr[i].Count = 0;
    s_tTmr[i].PreLoad = 0;
    s_tTmr[i].Flag = 0;
    s_tTmr[i].Mode = TMR_ONCE_MODE; /**/
  }

  //1us 后马上开始
  TMR1_ITCfg(ENABLE, TMR0_3_IT_CYC_END); // 开启中断
  PFIC_EnableIRQ(TMR1_IRQn);
  TMR1_TimerInit(FREQ_SYS / 1000);//ms
  TMR1_Enable();
}

/*
*********************************************************************************************************
*****************
*/
void bsp_InitHardTimer(void)
{
#if(MCU_LIB_SELECT ==1)

#elif(MCU_LIB_SELECT == 2)

#endif
}

/*
*********************************************************************************************************
*	SysTick_ISR
*		
*	
*********************************************************************************************************
*/
__attribute__((interrupt("WCH-Interrupt-fast")))
__attribute__((section(".highcode")))
void TMR1_IRQHandler(void) // TMR0 定时中断
{
    uint8_t i;
    if(TMR1_GetITFlag(TMR0_3_IT_CYC_END))
    {
        TMR1_ClearITFlag(TMR0_3_IT_CYC_END); // 清除中断标志

        if (s_uiDelayCount > 0)
        {
            if (--s_uiDelayCount == 0)
            {
              s_ucTimeOutFlag = 1;
            }
        }

        for (i = 0; i < BSP_TMR_COUNT; i++)
        {
        bsp_SoftTimerDec(&s_tTmr[i]);
        }

        g_iRunTime++;
        s_RunTime++;
        if (g_iRunTime == 0x7FFFFFFF) /*  int32_t  0x7FFFFFFF */
        {
        g_iRunTime = 0;
        }

    }
}

/*
*********************************************************************************************************
*	bsp_SoftTimerDec
*********************************************************************************************************
*/
static void bsp_SoftTimerDec(SOFT_TMR *_tmr)
{
  if (_tmr->Count > 0)
  {
    if (--_tmr->Count == 0)
    {
      _tmr->Flag = 1;
      if (_tmr->Mode == TMR_AUTO_MODE)
      {
        _tmr->Count = _tmr->PreLoad;
      }

      if (_tmr->Callback != 0)
      {
        _tmr->Callback();
      }
    }
  }
}

/*
*********************************************************************************************************
*bsp_StartTimer
**_period :
*********************************************************************************************************
*/
void bsp_StartTimer(uint8_t _id, uint32_t _period)
{
  if (_id >= BSP_TMR_COUNT)
  {
    while (1)
      ;
  }

  s_tTmr[_id].Count = (uint16_t)(_period);   /**/
  s_tTmr[_id].PreLoad = (uint16_t)(_period); /**/
  s_tTmr[_id].Flag = 0;                      /**/
  s_tTmr[_id].Mode = TMR_ONCE_MODE;          /**/
}

/*
 ************************************************************************************************
 ********************************
 */
void bsp_StartFreeRunTime(void)
{
  s_RunTime = 0;
}

/*
*********************************************************************************************************
*bsp_StartAutoTimer
*
*********************************************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
  if (_id >= BSP_TMR_COUNT)
  {
    while (1)
      ; /**/
  }

  s_tTmr[_id].Count = (uint16_t)(_period);   /**/
  s_tTmr[_id].PreLoad = (uint16_t)(_period); /**/
  s_tTmr[_id].Flag = 0;                      /**/
  s_tTmr[_id].Mode = TMR_AUTO_MODE;          /**/
}

void bsp_StartCallBackTimer(uint8_t _id, pFun _callFun, uint32_t _period)
{
  if (_id >= BSP_TMR_COUNT)
  {
    return;
  }
  s_tTmr[_id].Count = (uint16_t)(_period);
  s_tTmr[_id].PreLoad = (uint16_t)(_period);
  s_tTmr[_id].Flag = 0;
  s_tTmr[_id].Mode = TMR_AUTO_MODE;

  s_tTmr[_id].Callback = _callFun;
}

/*
*********************************************************************************************************
*	bsp_StopTimer
*	*********************************************************************************************************
*/
void bsp_StopTimer(uint8_t _id)
{
  if (_id >= BSP_TMR_COUNT)
  {
    return;
  }
  s_tTmr[_id].Count = 0;
  s_tTmr[_id].Flag = 0;
  s_tTmr[_id].Mode = TMR_ONCE_MODE;
}

/*
*********************************************************************************************************
*	bsp_CheckTimer
*	
*	_period :
*	
*********************************************************************************************************
*/
uint8_t bsp_CheckTimer(uint8_t _id)
{
  if (_id >= BSP_TMR_COUNT)
  {
    return 0;
  }

  if (s_tTmr[_id].Flag == 1)
  {
    s_tTmr[_id].Flag = 0;
    return 1;
  }
  else
  {
    return 0;
  }
}


uint32_t bsp_GetFreeRunTime(void)
{
  return s_RunTime;
}

/*********************************************************************************************************
*	函 数 名: bsp_DelayMS
*	功能说明: ms级延迟，延迟精度为正负1ms
*	形    参:  n : 延迟长度，单位1 ms。 n 应大于2
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DelayMS(uint32_t n)
{
  mDelaymS(n);
}


/*
*********************************************************************************************************
*    函 数 名: bsp_DelayUS
*    功能说明: us级延迟。 必须在systick定时器启动后才能调用此函数。
*    形    参:  n : 延迟长度，单位1 us
*    返 回 值: 无
*********************************************************************************************************
*/
void bsp_DelayUS(uint32_t n)
{
  mDelayuS(n);
} 

uint64_t bsp_GetRunTime(void)
{
  return g_iRunTime;
}





