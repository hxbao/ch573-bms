/******************************************************************************

                  版权所有 (C), 2021-2031, 南京集澈电子科技有限公司

 ******************************************************************************
  文 件 名   : AFE.c
  版 本 号   : V2.0
  作    者   : 武俊杰
  生成日期   : 2021年7月22日
  最近修改   :
  功能描述   : dvc1024/18/12/06芯片
  函数列表   :
  修改历史   :
  1.日    期   : 2021年4月25日
    作    者   : 武俊杰
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "includes.h"
#include "dvc10xx.h"
#include "dvc10xx_afe.h"
#if (AFE_CHIP_SELECT ==2)

/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
 

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/


/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
st_AfeGet g_stAfeGet = {0};			//AFE采集数据暂存结构体全局变量
st_AfeState g_stAfeState = {0};		//AFE保护等状态暂存结构体全局变量
st_AfeControl g_stAfeControl = {0}; //AFE控制数据暂存结构体全局变量

//短接电池电压 序号映射表 				 数组序号为未短接需要。数组中存储值为短接后的新序号，用于电压采集和均衡使用
uint8_t g_ucShortConnectMap[DVC_BATT_STRINGS_NUM_MAX] = {0};	//序号值从1开始，依次增加。短接序号值为0。

 /*----------------------------------------------*
  * 模块级变量 								  *
  *----------------------------------------------*/
 
 /*----------------------------------------------*
  * 常量定义									 *
  *----------------------------------------------*/
 
 /*----------------------------------------------*
  * 宏定义										*
  *----------------------------------------------*/


 
/*****************************************************************************
 函 数 名  : Afe_DataStruct_Init
 功能描述  : AFE 相关数据初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void Afe_DataStruct_Init(void)
{
	dvc10xx_DataStructInit();

	MemSet((uint8_t *)&g_stAfeGet, 0x0, sizeof(st_AfeGet));
	MemSet((uint8_t *)&g_stAfeState, 0x0, sizeof(st_AfeState));
	MemSet((uint8_t *)&g_stAfeControl, 0x0, sizeof(st_AfeControl));
	MemSet(g_ucShortConnectMap, 0x0, DVC_BATT_STRINGS_NUM_MAX);
}

/*****************************************************************************
 函 数 名  : afe_SingleBattVolMonitor
 功能描述  : AFE 单串电池电压值监测
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void afe_SingleBattVolMonitor(void)
{
	uint8_t DataBuf[32] = {0};
	uint8_t ret, i;

	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG1, DataBuf, DVC_MAX_DATA_BYTE);	 //STRG1中只包含单节电压信息 1，2，3  7，8，9  13，14，15 19，20，21
	if(ret == DVC_SUCCESS)
	{
		dvc10xx_OneBattVol_allGet(DataBuf, DVC_MAX_DATA_BYTE, DVC_CMD_RDSTRG1);	 //更新单节电池电压	
	}
	else
	{
		return;
	}

 
	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG2, DataBuf, DVC_MAX_DATA_BYTE);	 //STRG2中只包含单节电压信息	4，5，6 10，11，12 16，17，18 22，23，24
	if(ret == DVC_SUCCESS)
	{
		dvc10xx_OneBattVol_allGet(DataBuf, DVC_MAX_DATA_BYTE, DVC_CMD_RDSTRG2);	 //更新单节电池电压
	}
	else
	{
		return;
	}
 

	dvc10xx_CalculateMaxBattVol(); 	 //计算最高电压
	dvc10xx_CalculateMinBattVol(); 	 //计算最低电压
}


/*****************************************************************************
 函 数 名  : afe_TemperatureMonitor
 功能描述  : AFE 温度监测
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void afe_TemperatureMonitor(void)
{
	uint8_t DataBuf[32] = {0};
	uint8_t ret;
	 
	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG3, DataBuf, DVC_MAX_DATA_BYTE);	 //STRG1中只包含温度电压信息
	if(ret == DVC_SUCCESS)
	{
		dvc10xx_TcAndTs_allGet(DataBuf, DVC_MAX_DATA_BYTE);	 //更新温度

		if((g_stAfeGet.TemperMax <= AFE_TEMPER_THRESHOLD_UP) && (g_stAfeGet.TemperMin >= AFE_TEMPER_THRESHOLD_DOWN))	//在温度阈值范围内
		{
			g_stAfeControl.TemperCount = 0;		//清零过温阈值统计
			g_stAfeState.OverTpFlag = 0;		//过温标志清零
		}
		else
		if(g_stAfeGet.TemperMax > AFE_TEMPER_THRESHOLD_UP)//在温度阈值范围外
		{
			//三次都是超过温度阈值
			if(g_stAfeControl.TemperCount < 3)
			{
				g_stAfeControl.TemperCount++;	//增加过温阈值统计计数
			}
			else if(g_stAfeControl.TemperCount == 3)
			{
				g_stAfeState.OverTpFlag = 1;		//置位过温标志
			}		
		}else
		if(g_stAfeGet.TemperMin > AFE_TEMPER_THRESHOLD_DOWN)
		{
			//三次都是超过温度阈值
			if(g_stAfeControl.TemperCount < 3)
			{
				g_stAfeControl.TemperCount++;	//增加过温阈值统计计数
			}
			else if(g_stAfeControl.TemperCount == 3)
			{
				g_stAfeState.OverTpFlag = 2;		//置位低温保护
			}
		}
	}
}

/*****************************************************************************
 函 数 名  : dvc10xx_Cur_BPack_AltStMonitor
 功能描述  : AFE芯片 电流 组电压 和警报信息监测
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_Cur_BPack_AltStMonitor(void)
{
	dvc10xx_StatePackVolCur_Get();
}

/*****************************************************************************
 函 数 名  : afe_SleepManage
 功能描述  : AFE休眠管理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void afe_SleepManage(void)
{
	if(g_stDvcOpt.sLeepFlag == E_AFE_SleepOp)		//休眠标志
	{
		//睡眠会自动关闭充放电
		if(dvc10xx_cmd_sleep_check() != DVC_SUCCESS)		//向芯片内写入休眠指令，并检查。
		{
			
		}

		g_stDvcOpt.sLeepFlag = E_AFE_SleepOpLast;		//置位休眠标志位，已经休眠操作过了。
	}

	if(g_stDvcOpt.sLeepFlag == E_AFE_SleepWaUp)		//唤醒标志
	{
		dvc10xx_SleepWakeup();						//唤醒操作，WAUP管脚DVC芯片
		
		//重写入配置			需要区分是写入默认配置，还是上次配置，本处写入默认配置。
		dvc10xx_init_cfg();
		g_stDvcOpt.sLeepFlag = E_AFE_SleepNoOp;		//无休眠操作标志

		//是否打开充放电MOS，在充放电管理中打开
	}

	//………………………………………………………………测试使用………………………………………………………………………………
	// if(g_stDvcOpt.sLeepFlag == E_AFE_SleepOpLast)
	// {
	// 	delay_ms(20000);	//20s后重新唤醒
	// 	g_stDvcOpt.sLeepFlag = E_AFE_SleepWaUp;	//5S后唤醒，实际唤醒由其他处置位
	// }
	//………………………………………………………………测试使用………………………………………………………………………………
}

/*****************************************************************************
 函 数 名  : afe_DsgChgManage
 功能描述  : 充放电管理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void afe_DsgChgManage(void)
{
	uint8_t openChDsFlag = 0;
	uint8_t closeChDsFlag = 0;
 
	/********************************************  开启充放电管        ***************************************************/
	if(g_stAfeState.ProtectFlag == 0)	//芯片没有警报
	{
		if(g_stAfeControl.ChgDsgInt != 0)		//有外部检测充电器和负载中断
		{
			openChDsFlag = 1;
			g_stAfeControl.ChgDsgInt = 0; 		//清零中断位
		}

		if((g_stAfeState.LdAltPdFlag & 0x1) != 0)	//使用了AFE芯片的负载检测
		{
			openChDsFlag = 1;
			dvc10xx_CmdClearManage();		//清除警报
		}
	}
	else  //芯片有警报
	{		
		//允许充电   且 有欠压保护标志 且当前是电流方向是充电
		if(g_stAfeControl.AllowChg == 1 && ((g_stAfeState.OvUvFlag & 0xf) != 0) && g_stDvcOpt.currentDir == E_AFE_CurDirChg)
		{
			openChDsFlag = 1;
			if(g_stAfeControl.MaskUvFlag == 0)	//没有屏蔽欠压警报
			{
				//屏蔽欠压警报
				dvc10xx_FuncFlushCfg(DVC_Func_MASK_UV, 0, 1);
				
				g_stAfeControl.MaskUvFlag = 1;
			}
		}

		//允许放电  且 有过压保护标志 且当前是电流方向是放电
		if(g_stAfeControl.AllowDsg == 1 && ((g_stAfeState.OvUvFlag & 0xf0) != 0) && g_stDvcOpt.currentDir == E_AFE_CurDirDsg)
		{
			openChDsFlag = 1;
			if(g_stAfeControl.MaskUvFlag == 0)	//没有屏蔽过压警报
			{
				//屏蔽过压警报
				dvc10xx_FuncFlushCfg(DVC_Func_MASK_OV, 0, 1);
				g_stAfeControl.MaskUvFlag = 1;
			}
		}
	}

	if(openChDsFlag == 1)		//真正开启充放电MOS
	{
		//打开充放电MOS
		if(g_stAfeState.ChDshFlag != 0x3)
		{
			dvc10xx_MOS_QuikerOpen();
			return;
		}
	}
	/*********************************************  关闭充放电管        ***************************************************/
	if(g_stAfeControl.AllowDsg == 0 && g_stAfeControl.AllowChg == 0)	//不允许充电，也不允许放电
	{
		closeChDsFlag = 1;
	}

	//允许充电   不允许放电，且此时为放电
	if(g_stAfeControl.AllowDsg == 0 && g_stAfeControl.AllowChg == 1 && g_stDvcOpt.currentDir == E_AFE_CurDirDsg)
	{
		closeChDsFlag = 1;
	}

	//允许放电不允许充电，且此时为充电
	if(g_stAfeControl.AllowDsg == 1 && g_stAfeControl.AllowChg == 0 && g_stDvcOpt.currentDir == E_AFE_CurDirChg)
	{
		closeChDsFlag = 1;
	}

	//关闭充放电MOS
	if(closeChDsFlag == 1)
	{
		if(g_stAfeState.ChDshFlag != 0x0)
		{
			dvc10xx_MOS_QuikerClose();
		}
	}
}

/*****************************************************************************
 函 数 名  : afe_AlertResumeManage
 功能描述  : AFE芯片保护警报后处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void afe_AlertResumeManage(void)
{	//芯片恢复管理
	uint8_t flag = 0;
	
	if(g_stAfeState.ProtectFlag != 0 || g_stAfeState.ProtectExitFlag != 0)		//芯片有警报
	{
		if(g_stAfeState.PassageFlag != 0)	//通道故障		
		{
			//需要重启芯片，多次还是通道故障，需要更换芯片
			//通道故障没法通过清除命令清除状态位。
		}

		if(g_stAfeState.OvUvFlag != 0)		//过压欠压故障
		{
			//过压，   因充满而过压。允许放电，不允许充电
			if((g_stAfeState.OvUvFlag & 0xf0) != 0 && (g_stAfeState.OvUvFlag & 0x0f) == 0)		
			{
				g_stAfeControl.AllowChg = 0;	//不允许充电
				g_stAfeControl.AllowDsg = 1;	//允许放电
			}

			//欠压，因放电而欠压。过低，不允许充电和放电，视为电池报废需要维修。非过低则继续进行充电操作，不允许放电。
			if((g_stAfeState.OvUvFlag & 0x0f) != 0 && (g_stAfeState.OvUvFlag & 0xf0) == 0)
			{
				if(g_stAfeGet.BattVolMin < AFE_VOL_THRESHOLD_MIN)		//电压极低，不允许充电
				{
					g_stAfeControl.AllowDsg = 0;	//不允许放电
					g_stAfeControl.AllowChg = 0;	//不允许充电
				}
				else		//在允许充电范围内，只许充电，不许放电
				{
					g_stAfeControl.AllowDsg = 0;	//不允许放电
					g_stAfeControl.AllowChg = 1;	//允许充电
				}
			}

			//既有过压也有欠压，视为电池故障。不允许充放电
			if((g_stAfeState.OvUvFlag & 0xf0) != 0 && (g_stAfeState.OvUvFlag & 0x0f) != 0)
			{
				g_stAfeControl.AllowDsg = 0;	//不允许放电
				g_stAfeControl.AllowChg = 0;	//不允许充电
			}

			//过压、欠压恢复	 		电压都在正常范围内，有电压
			if((g_stAfeGet.BattVolMax <= AFE_VOL_THRESHOLD_UP) && (g_stAfeGet.BattVolMin >= AFE_VOL_THRESHOLD_DOWN))		//在电压阈值范围内
			{
				g_stAfeControl.AllowChg = 1;	//允许充电
				g_stAfeControl.AllowDsg = 1;	//允许放电

				//清除过欠压屏蔽警报位
				if(g_stAfeControl.MaskOvFlag != 0)
				{
					dvc10xx_FuncFlushCfg(DVC_Func_MASK_OV, 0, 0);
					g_stAfeControl.MaskOvFlag = 0;
				}
				if(g_stAfeControl.MaskUvFlag != 0)
				{
					dvc10xx_FuncFlushCfg(DVC_Func_MASK_UV, 0, 0);
					g_stAfeControl.MaskUvFlag = 0;
				}

				flag++;		//清除警报
			}
		}
	
		if(g_stAfeState.CurFlag != 0)		//电流过限故障
		{
			//电流过限，配合芯片保护功能后，充放电MOS会关闭。
			//在关闭充放电MOS情况下，测到的电流约为0。不能作为恢复依据，需要清除警报后再试
			//延时2s后，清除警报。重复3次仍然有警报，交互。
			if(g_stAfeControl.ChgDsgAltCount < 3)
			{
				bsp_DelayMS(2000); //延时2S.
				flag++;		//清除警报
				g_stAfeControl.ChgDsgAltCount++;
			}
			else	//3次都是电流警报，进行交互
			{
				//完全关闭充放电
				g_stAfeControl.AllowDsg = 0;	//不允许放电
				g_stAfeControl.AllowChg = 0;	//不允许充电
				//交互，重新使用用电器和负载，且没有电流警报可恢复
			}
		}
		else
		{
			if(g_stAfeControl.ChgDsgAltCount != 0)	//清0计数
			{
				g_stAfeControl.ChgDsgAltCount = 0;
			}
		}

		if(g_stAfeState.LdAltPdFlag != 0)	//负载检测（不是故障，不在这里处理）和警报下拉故障（不能够同时进行警报中断）
		{
			//下拉管脚置高后清除警报即可恢复正常。
			//*******此处拉高警报下拉管脚。
			flag++;
		}

		if(g_stAfeState.OverTpFlag != 0)	//超过了温度正常阈值
		{
			//温度没有状态位在监测。不需要做清除操作。
			g_stAfeControl.AllowChg = 0;
			g_stAfeControl.AllowDsg = 0;
		}

		if(flag != 0)		//需要清除警报
		{
			//清除故障并检查
			dvc10xx_CmdClearManage();
			g_stAfeState.ProtectFlag = 0;	//清除状态位，下次扫描采集数据后，或ALT中断置位
			g_stAfeState.ProtectExitFlag = 0;
		}
	}
}

void afe_CalibrationManage(void)
{
	/*
		软件校准管理，用于一些因所在电路电阻精度等问题造成的采样数据偏移。
		校准需要结合外部校准，需要在上位机或者MCU完成校准参数。
			建议在上位机校准，记录数据直接输入到上位机更方便。
			
		将校准后的参数在每次采集时都代入实时计算。

		-------------后续版本实现
	*/
}

/*****************************************************************************
 函 数 名  : AFE_TimerInt_callback
 功能描述  : 定时轮询控制采集等
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void AFE_TimerInt_callback(void)
{
	g_stDvcOpt.timerCountMs++;

	if(g_stDvcOpt.timerCountMs % 64 == 0)
	{
		g_stDvcOpt.timerFlag = E_AFE_Tim64MS;	 //置位64ms标志
	}

	if(g_stDvcOpt.timerCountMs % 256 == 0)  
	{
		g_stDvcOpt.timerFlag = E_AFE_Tim256MS; 	 //置位256ms标志
	}

	if(g_stDvcOpt.timerCountMs == 1000)
	{
		g_stDvcOpt.timerFlag = E_AFE_Tim1S;		 //置位1s标志
		g_stDvcOpt.timerCountMs = 0;

		//Sec计数用于休眠操作
		g_stDvcOpt.timerCountSec++;
		if(g_stDvcOpt.timerCountSec == 40)		 //40s内无充放电后进入休眠
		{
			g_stDvcOpt.timerCountSec = 0;
			if(g_stDvcOpt.sLeepFlag != E_AFE_SleepOpLast)
			{
				g_stDvcOpt.sLeepFlag = E_AFE_SleepOp;
			}
		}

		if(g_stDvcOpt.currentDir != E_AFE_CurDirNo)	//有充放电会清除秒定时计数
		{
			g_stDvcOpt.timerCountSec = 0;
		}
	}
}

void Dvc10xx_AFE_Init(void)
{
	Afe_DataStruct_Init();
	bsp_StartCallBackTimer(TMR_DVCAFE_POLL,AFE_TimerInt_callback,1);
}

/*****************************************************************************
 函 数 名  : AFE_MAIN_Scanf
 功能描述  : main主循环中根据状态位轮询处理AFE数据
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年7月27日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void Dvc10xx_AFE_Process(void)
{
	//根据定时标志，监管采集数据。
	if(g_stDvcOpt.sLeepFlag != E_AFE_SleepOpLast)
	{
		if(g_stDvcOpt.timerFlag == E_AFE_Tim64MS)
		{	 
			g_stDvcOpt.timerFlag = E_AFE_TimNoOp;
		}
		 
		if(g_stDvcOpt.timerFlag == E_AFE_Tim256MS)
		{
			dvc10xx_Current_QuikerGet();		 //快速更新电流数据
			g_stDvcOpt.timerFlag = E_AFE_TimNoOp;
		}
		 
		if(g_stDvcOpt.timerFlag == E_AFE_Tim1S)
		{
			afe_TemperatureMonitor();		//监测温度
			afe_SingleBattVolMonitor();	    //监测单节电压
			dvc10xx_Cur_BPack_AltStMonitor();    //检测电流，电池组电压，警报状态等。
			g_stDvcOpt.timerFlag = E_AFE_TimNoOp;
		}
	}
 
	afe_SleepManage();			//休眠管理
	afe_DsgChgManage();			//充放电管理
	afe_AlertResumeManage();	//警报恢复管理
}

uint8_t Dvc10xx_AFE_GetTempProtectState()
{

}

uint16_t Dvc10xx_AFE_JudgeState(void)
{
	uint16_t batsta = 0;
	if ((g_stAfeState.ChDshFlag & 0x01))
	{
		//充电状态
		batsta |= 0x0002;
	}

	if ((g_stAfeState.ChDshFlag & 0x02))
	{
		//放电状态
		batsta |= 0x0001;
	}
	
	//过充保护 -bit4-7 OV1/2/3/4
	if(g_stAfeState.OvUvFlag & 0xF0)
	{
		batsta |= 0x0004;
	}
	//过放保护
	if(g_stAfeState.OvUvFlag & 0x0F)
	{
		batsta |= 0x0008;
	}
	//充电过流保护bit0/1/2/3 OCC/OCD/SCD
	if(g_stAfeState.CurFlag & 0x01)
	{
		batsta |= 0x0010;
	}
	//放电过流保护
	if(g_stAfeState.CurFlag & 0x02)
	{
		batsta |= 0x0020;
	}
	//电池过热保护
	if(g_stAfeState.OverTpFlag == 1)
	{
		batsta |= 0x0040;
	}
	//电池低温保护
	if(g_stAfeState.OverTpFlag == 2)
	{
	 	batsta |= 0x0080;
	}
	//电池组短路保护
	if(g_stAfeState.CurFlag & 0x04)
	{
		batsta |= 0x0200;
	}
	return batsta;
}

uint8_t Dvc10xx_AFE_GetCadcCurrent(uint16_t *curr)
{
	*curr = (uint16_t)(g_stAfeGet.CurAverage*1000);
}

void Dvc10xx_AFE_GetCellsVolt(uint16_t *CellVolt)
{
	uint8_t i;
	for(i = 0;i<CELL_NUMS;i++)
	{
		*(CellVolt+i) = (uint16_t)(g_stAfeGet.BattVol[i]*1000);
	}
}

void Dvc10xx_AFE_GetTemp(uint16_t *TempNTC)
{
	*(TempNTC +0) = g_stAfeGet.TC[0]*10+2731;
	*(TempNTC +1) = g_stAfeGet.TC[1]*10+2731;
	*(TempNTC +2) = g_stAfeGet.TC[2]*10+2731;
}

//open
void Dvc10xx_AFE_OpenDsgMos()
{
	dvc10xx_MOS_QuikerOpen();
}

//close
void Dvc10xx_AFE_CloseDsgMos()
{
   dvc10xx_MOS_QuikerClose();	//快速打开充放电MOS
}

//openchg
void Dvc10xx_AFE_OpenChgMos_CLoseDsg()
{
	dvc10xx_DMOS_Close_CMOS_Open();
}

//closechg
void Dvc10xx_AFE_OpenDsgMos_CloseChg()
{
	dvc10xx_DMOS_Open_CMOS_Close();
}

#endif
