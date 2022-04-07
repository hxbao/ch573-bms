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
static SOFT_TMR s_tTmr[TMR_COUNT];

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
  for (i = 0; i < TMR_COUNT; i++)
  {
    s_tTmr[i].Count = 0;
    s_tTmr[i].PreLoad = 0;
    s_tTmr[i].Flag = 0;
    s_tTmr[i].Mode = TMR_ONCE_MODE; /**/
  }
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
#if(MCU_LIB_SELECT == 2)
void SysTick_Handler(void)
#elif(MCU_LIB_SELECT == 1)
void SysTick_IRQHandler(void)
#endif
{
  uint8_t i;

  if (s_uiDelayCount > 0)
  {
    if (--s_uiDelayCount == 0)
    {
      s_ucTimeOutFlag = 1;
    }
  }

  for (i = 0; i < TMR_COUNT; i++)
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
  if (_id >= TMR_COUNT)
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

uint32_t bsp_GetFreeRunTime(void)
{
  return s_RunTime;
}

/*
*********************************************************************************************************
*bsp_StartAutoTimer
*
*********************************************************************************************************
*/
void bsp_StartAutoTimer(uint8_t _id, uint32_t _period)
{
  if (_id >= TMR_COUNT)
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
  if (_id >= TMR_COUNT)
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
  if (_id >= TMR_COUNT)
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
  if (_id >= TMR_COUNT)
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

/*********************************************************************************************************
*	函 数 名: bsp_DelayMS
*	功能说明: ms级延迟，延迟精度为正负1ms
*	形    参:  n : 延迟长度，单位1 ms。 n 应大于2
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_DelayMS(uint32_t n)
{
  if (n == 0)
  {
     return;
  }

  s_uiDelayCount = n;
  s_ucTimeOutFlag = 0;


  while (1)
  {
   /*
       等待延迟时间到
       注意：编译器认为 s_ucTimeOutFlag = 0，所以可能优化错误，因此 s_ucTimeOutFlag 变量必须申明为 volatile
    */
    if (s_ucTimeOutFlag )
    {
            break;
    }
  }
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
    uint32_t ticks;
    uint32_t told;
    uint32_t tnow;
    uint32_t tcnt = 0;
    uint32_t reload;
       
    reload = SysTick->LOAD;                
    ticks = n * (SystemCoreClock / 1000000);	 /* 需要的节拍数 */  
    
    tcnt = 0;
    told = SysTick->VAL;             /* 刚进入时的计数器值 */

    while (1)
    {
        tnow = SysTick->VAL;    
        if (tnow != told)
        {    
            /* SYSTICK是一个递减的计数器 */    
            if (tnow < told)
            {
                tcnt += told - tnow;    
            }
            /* 重新装载递减 */
            else
            {
                tcnt += reload - tnow + told;    
            }        
            told = tnow;

            /* 时间超过/等于要延迟的时间,则退出 */
            if (tcnt >= ticks)
            {
            	break;
            }
        }  
    }
} 

uint64_t bsp_GetRunTime(void)
{
  return g_iRunTime;
}


