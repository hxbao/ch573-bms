
#include "includes.h"

void NiuLogic_BalanceHanle(void)
{
#if (AFE_CHIP_SELECT == 3)//需要测试
	/*
	static uint8_t ucBalanceStep = 0;
	static uint16_t balCount = 0;
	if ((afeInfo.State_RT & 0x0002) || (afeInfo.State_RT == 0x0000))
	{
		//开启
		if (afeInfo.CellVmin > BALANCE_START_V &&
			afeInfo.CellVdiff > BALANCE_START_S &&
			afeInfo.CellVdiff < BALANCE_START_E &&
			afeInfo.BalanceTemp < (2731 + 800))
		{
			//高于平均电压的电芯需要平衡，采用奇偶平衡的办法
			afeInfo.balState |= 1 << afeInfo.CellVmaxPos;
		}

		//开启平衡，一次平衡阶段避让电压采集
		if(afeInfo.balState != 0)
		{
			switch(ucBalanceStep)
			{
				case 0:    
						                 
					ucBalanceStep = 1;
					AFE_DisableVSample(); 
					AFE_SET_BALANCE(afeInfo.CellVmaxPos);  //开启平衡

					bsp_StartTimer(TMR_BLANCE_PWM,100);
					break;
				case 1:
					if(bsp_CheckTimer(TMR_BLANCE_PWM))
					{
						AFE_DISABLE_BALANCE();//关闭平衡
						AFE_EnableVSample();

						ucBalanceStep = 2;
						bsp_StartTimer(TMR_BLANCE_PWM,100);
						balCount++;
						//平衡30s
						if(balCount>600)
						{
							balCount = 0;
							ucBalanceStep = 3;
						}
					}
					//pwm 平衡
					break;
				case 2:
					if(bsp_CheckTimer(TMR_BLANCE_PWM))
					{
						ucBalanceStep = 0;
						bsp_StartTimer(TMR_BLANCE_PWM,100);
					}
					
					break;
				case 3://平衡结束
					afeInfo.balState = 0;
					break;
			}
		}
	}*/
#elif(AFE_CHIP_SELECT == 1)
	if (afeInfo.State_RT & 0x0002)
	{
		//开启
		if (afeInfo.CellVmin > BALANCE_START_V &&
			afeInfo.CellVdiff > BALANCE_START_S &&
			afeInfo.CellVdiff < BALANCE_START_E &&
			afeInfo.BalanceTemp < (2731 + 800))
		{
			//高于平均电压的电芯需要平衡，采用奇偶平衡的办法
			afeInfo.balState |= 1 << afeInfo.CellVmaxPos;
			AFE_SET_BALANCE(afeInfo.CellVmaxPos);
		}else
		{
			afeInfo.balState = 0;
			AFE_DISABLE_BALANCE();//关闭平衡			
		}
	}else
	{
		afeInfo.balState = 0;
		AFE_DISABLE_BALANCE();//关闭平衡	
	}
#else
	//使用智能平衡控制算法
	Alg_BalanceHandle();
#endif
	//不满足条件，延迟一分钟sh367309自动停止
}