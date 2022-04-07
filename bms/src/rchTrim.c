/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"


#if(MCU_LIB_SELECT ==1)
static void RCH_TrimCalInit(void)
{
    stc_trim_cfg_t stcCfg;

    //打开TRIM外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTrim, TRUE);
     
    //TRIM校准流程
    stcCfg.enMON     = TrimMonDisable;
    stcCfg.enREFCLK  = TrimRefMskExtClk;
    stcCfg.enCALCLK  = TrimCalMskRCH;
    stcCfg.u32RefCon = 240000u;                     //10ms校准时间（增加该时间可提高TRIM精度）外部24M时钟输入，10ms
    stcCfg.u32CalCon = 0xFFFFFFFFu;                 //配置为默认值
    Trim_Init(&stcCfg);
}

//外部时钟初始化，判定有外部时钟输入，且稳定，做校准

uint8_t RCH_Calbration(void)
{
    uint8_t u8TrimTestFlag = 0;
    uint8_t calval = 0;
    uint32_t flashTrimVal = 0;

    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    //bsp_DelayMS(200);
    //外部CLK 已经输入，且稳定,进行校准
    if((1 != M0P_SYSCTRL->XTH_CR_f.STABLE))
    {
       return 0xFF;
    }else
    {        
        //加载已经校准过的值
        Flash_Read(EE_START_ADDR + CALI_RCH_TRIM_VAL_ADDR, (uint8_t*)&flashTrimVal, 4);
        if(flashTrimVal != 0xFFFFFFFF)
        {
            M0P_SYSCTRL->RCH_CR_f.TRIM  = flashTrimVal;
            return 0xFE;
        }
    }

    //内部初始24M校准的值
    calval = M0P_SYSCTRL->RCH_CR_f.TRIM;
    M0P_SYSCTRL->RCH_CR_f.TRIM = calval-100;

    RCH_TrimCalInit();
    Trim_Run();
    volatile uint32_t u32CalCnt;
    while(1)
    {
        if(Trim_GetIntFlag(TrimStop))
        {
            u32CalCnt = Trim_CalCntGet();
            ///<参考计数值计数完成（10ms)时，查看待校准计数值是否也为（10ms）计数值,或是否在允许误差范围内，此处为24000/100 = 328(±0.3%)
            ///<可根据实际修改该比较范围，提高TRIM校准精度。
            if ((u32CalCnt <= (240000u + 1u)) &&
                (u32CalCnt >= (240000u - 1u)))
            {
                Trim_Stop();
                ///< 校准结束,此时的TRIM值即为最佳频率值
                ///保存到flash中
                Flash_Write(EE_START_ADDR + CALI_RCH_TRIM_VAL_ADDR, (uint8_t*)&u32CalCnt, 4);
                u8TrimTestFlag = 0x01u;
                break;
            }
            else
            {
                Trim_Stop();
                ///< 为达到目标精度，TRIM值增加1，继续校准
                M0P_SYSCTRL->RCH_CR_f.TRIM += 1;       
                Trim_Run();           
            }
            
        }
        
        if(Trim_GetIntFlag(TrimCalCntOf))  //参考校准时间设置过长，导致待校准计数器溢出，此时需要重新配置参考校准时间及校准精度
        {
            //设为默认值
            M0P_SYSCTRL->RCH_CR_f.TRIM  = calval;
            u8TrimTestFlag = 0;
            break;
        }
    }

    return u8TrimTestFlag;
    
}
#elif(MCU_LIB_SELECT ==2)
uint8_t RCH_Calbration(void)
{
    
}

#endif