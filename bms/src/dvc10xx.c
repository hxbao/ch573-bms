/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2021/10/16
History:
	V0.0		2021/10/16		 Preliminary
********************************************************************************/

#include "includes.h"
#include "dvc10xx.h"
#include "dvc10xx_afe.h"


#if (AFE_CHIP_SELECT == 2)

/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

//短接电池电压 序号映射表 				 数组序号为未短接需要。数组中存储值为短接后的新序号，用于电压采集和均衡使用
//extern uint8_t g_ucShortConnectMap;	//序号值从1开始，依次增加。短接序号值为0。
/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
 

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
static void dvc_PEC_CRC_Attend(uint8_t *dataPtr, uint8_t dataLen);
static E_DvcCmdRet dvc_PEC_CRC_Check(uint8_t *bufPtr, uint8_t byte, uint8_t *ErrorPos);
static uint8_t dvc10xx_FrameWrite(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen);
static uint8_t dvc10xx_FrameRead(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen);
static E_DvcCmdRet dvc10xx_FrameWrite_Check(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen);



/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/

//dvc10xx命令字，dvc1024/18/12/06命令字均相同，  直接使用PEC校验值，避免频繁计算校验值耗费时间
const uint8_t dvc_cmd[10][4] = {{0x01, 0x00, 0x3e, 0x10}, 
						   {0x01, 0x01, 0xb5, 0x22},
						   
						   {0x02, 0x00, 0x2d, 0xf6},
						   {0x02, 0x01, 0xa6, 0xc4},
						   {0x02, 0x02, 0xb0, 0xa0},
						   {0x02, 0x03, 0x3b, 0x92},
						   
						   {0x03, 0x00, 0xa5, 0xba},
						   {0x03, 0x01, 0x2e, 0x88},
						   
						   {0x04, 0x00, 0x0a, 0x3a},
						   {0x04, 0x01, 0x81, 0x08}
						   };

//-------dvcxx配置寄存器，初始化配置数据。
//-------2,表示配置寄存器0，1
//-------4，对应1组寄存器中24字节数据，分成4组，对应4组电池（每组6节电池）的控制。4 x 6 = 24
//---------------实际对应与DVC芯片中内部4组小芯片（1个小芯片控制一组6节电池）。
//-------6，1组电池控制需要6字节数据
//-----此数据为DVC芯片默认配置。即掉电后上电的配置。需要根据业务需求修改。
//-----------数据中的第3列 0，1，2，3均为测试写入数据，正常使用不参考。
const uint8_t dvc_cfg[2][4][6] = {
								{	//cfg0
									{AFE_CFG0_DEF_VAL0, AFE_CFG0_DEF_VAL1, AFE_CFG0_DEF_VAL2, AFE_CFG0_DEF_VAL3, AFE_CFG0_DEF_VAL4, AFE_CFG0_DEF_VAL5}, 	//第一组电池 1006必须有
						 			{AFE_CFG0_DEF_VAL6, AFE_CFG0_DEF_VAL7, AFE_CFG0_DEF_VAL8, AFE_CFG0_DEF_VAL9, AFE_CFG0_DEF_VAL10, AFE_CFG0_DEF_VAL11},	//第二组电池 1012必须有，1006可以裁剪掉之后的数据
									{AFE_CFG0_DEF_VAL12, AFE_CFG0_DEF_VAL13, AFE_CFG0_DEF_VAL14, AFE_CFG0_DEF_VAL15, AFE_CFG0_DEF_VAL16, AFE_CFG0_DEF_VAL17}, 	//依次类推
						 			{AFE_CFG0_DEF_VAL18, AFE_CFG0_DEF_VAL19, AFE_CFG0_DEF_VAL20, AFE_CFG0_DEF_VAL21, AFE_CFG0_DEF_VAL22, AFE_CFG0_DEF_VAL23},
								},
								{	//cfg1    cfg1 BYTE3 bit5 置1可减小电流ADC采集死区
									{AFE_CFG1_DEF_VAL0, AFE_CFG1_DEF_VAL1, AFE_CFG1_DEF_VAL2, AFE_CFG1_DEF_VAL3, AFE_CFG1_DEF_VAL4, AFE_CFG1_DEF_VAL5}, 	//第一组电池 1006必须有
						 			{AFE_CFG1_DEF_VAL6, AFE_CFG1_DEF_VAL7, AFE_CFG1_DEF_VAL8, AFE_CFG1_DEF_VAL9, AFE_CFG1_DEF_VAL10, AFE_CFG1_DEF_VAL11},	//第二组电池 1012必须有，1006可以裁剪掉之后的数据
									{AFE_CFG1_DEF_VAL12, AFE_CFG1_DEF_VAL13, AFE_CFG1_DEF_VAL14, AFE_CFG1_DEF_VAL15, AFE_CFG1_DEF_VAL16, AFE_CFG1_DEF_VAL17}, 	//依次类推
						 			{AFE_CFG1_DEF_VAL18, AFE_CFG1_DEF_VAL19, AFE_CFG1_DEF_VAL20, AFE_CFG1_DEF_VAL21, AFE_CFG1_DEF_VAL22, AFE_CFG1_DEF_VAL23},
								}
							};

//暂存配置寄存器0， 1.用于实时刷新
uint8_t g_ucDvc_cfg[2][4][6] = {0};	

//定义dvc设备操作结构体。并初始化为0. 
//----此结构体主要用于SPI操作时序。暂存命令字节，中断执行前后的读写数据等。
st_DvcOpt g_stDvcOpt = {0};

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

/*****************************************************************************
 函 数 名  : dvc_PEC_CRC_Attend
 功能描述  : PEC添加校验值函数，会在传参长度之后2个字-
                 节内附加FEC校验值
 输入参数  :     uint8_t *dataPtr 		    注意：：：传入参数必须保证buf长度大于len 2个字节 
             uint8_t dataLen				如果是命令校验，传入2。如果是数据校验，传入6     
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月28日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
static void dvc_PEC_CRC_Attend(uint8_t *dataPtr, uint8_t dataLen)
{
	uint8_t chChar, i, j;
	uint16_t CRCtmp = PEC_SAW_VEL;

	for(i = 0; i < dataLen; i++)
	{
		chChar = *(dataPtr + i);
    	CRCtmp ^= (uint16_t)(chChar << 8);
		for (j = 0; j < 8; j++)
		{
		    if((CRCtmp & 0x8000) != 0)
				CRCtmp = (uint16_t)((CRCtmp << 1) ^ PEC_QUANTIC_VEL);
			else
				CRCtmp <<= 1;
		}
	}
	
	*(dataPtr + dataLen) = (uint8_t)((CRCtmp & 0xFF00) >> 8);  //写回到命令字后相邻第一个PEC字节
	*(dataPtr + dataLen + 1) = (uint8_t)(CRCtmp & 0x00FF);         //写回到命令字后相邻第二个PEC字节
}

/*****************************************************************************
 函 数 名  : dvc_PEC_CRC_Check
 功能描述  : 读数据命令后，校验读取值
 输入参数  : uint8_t *bufPtr 	    输入：待校验的数据指针，注意：：：支持1组中有6个数据字节，2个校验字节
 		    uint8_t byte   		   输入：待校验的真实长度
 		    uint8_t *ErrorPos      输出：0bit为1表示第一组出错，以此类推
            	
 输出参数  : 无
 返 回 值  : E_DvcCmdRet  
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月28日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
static E_DvcCmdRet dvc_PEC_CRC_Check(uint8_t *bufPtr, uint8_t byte, uint8_t *ErrorPos) //byte为真实byte数，不能减一
{
	uint8_t arrTmp[8];		//将传入buf中数据拷入，添加PEC后，与buf中对应PEC比较
	uint8_t i, j;
	uint8_t tmp = 0;			//tmp用于记录PEC错误，可用于查看第几个错误, 如0x0c表示第三，第四组数据校验出错
	
	for(j = 0; j < (byte / 8); j++)
	{
		for(i = 0; i < 6; i++)
		{
			arrTmp[i] = *bufPtr;
			bufPtr++;
		}	
		dvc_PEC_CRC_Attend(arrTmp, 6);  	//	填充PEC,用于校验
		
		if((arrTmp[6] != *bufPtr) || (arrTmp[7] != *(bufPtr + 1)))
			tmp |= 1 << j;
		
		bufPtr += 2; //原始指针加2 跳过PEC段
	}	

	if(tmp != 0)
	{
		*ErrorPos = tmp;  //传回数据错误组位置
		return DVC_PECWrong;
	}		
	else
		return DVC_SUCCESS;		
}

/*****************************************************************************
 函 数 名  : MemSet
 功能描述  : 存储空间初始化函数
 输入参数  : uint8_t   *p    
            uint8_t val     
            uint8_t length  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月4日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void MemSet(uint8_t   *p, uint8_t val, uint8_t length)
{
	uint8_t i;
	
	for(i = 0; i < length; i++)
	{
		*p = val;
		p++;
	}
}

/*****************************************************************************
 函 数 名  : ADC1_Calculate
 功能描述  : ADC计算函数	基于183.1uV
 输入参数  : char a  
            char b  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年5月7日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
double ADC1_Calculate(char a, char b)
{
	return ((a << 8) + b) * ADC_MAGIC_1;
}

/*****************************************************************************
 函 数 名  : ADC2_Calculate
 功能描述  : ADC计算函数	基于1.465mV
 输入参数  : char a  
            char b  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年5月7日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
double ADC2_Calculate(char a, char b)	
{
	return ((a << 8) + b) * ADC_MAGIC_2;
}


/*****************************************************************************
 函 数 名  : dvc10xx_BattNumMapInit
 功能描述  : 实际电池串序号对应芯片内部电池串序号，初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_BattNumMapInit(void)
{
	uint8_t i;
	uint8_t lastNum = 0;
	uint8_t num = 0;

	for(i = 0; i < DVC_BATT_STRINGS_NUM_MAX; i++)
	{
		if((DVC_BATT_MASK_FLAG & (1 << i)) == 0)		//
		{
			num++;
		}
		
		if(lastNum != num)
		{
			g_ucShortConnectMap[i] = num;			//数组中num 从1开始
		}

		lastNum = num;
	}
}


/*****************************************************************************
 函 数 名  : dvc10xx_BattNum_ToChipInnerNum
 功能描述  : 实际电池串序号转换成芯片内部电池串序号
 输入参数  : uint8_t battNum  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_BattNum_ToChipInnerNum(uint8_t battNum)
{
	uint8_t i;
	
	for(i = 0; i < DVC_BATT_STRINGS_NUM_MAX; i++)
	{
		if(g_ucShortConnectMap[i] == battNum)
		{
			return i;
		}
	}

	return 0xff;
}

/*****************************************************************************
 函 数 名  : dvc10xx_AlertState_Get
 功能描述  : 解析获取警报状态数据
 输入参数  : uint8_t *Pbuf  
             uint8_t len    
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_AlertState_Get(uint8_t *Pbuf, uint8_t len)
{
	uint8_t i, tmp, tmp1;
	
	g_stAfeState.ChDshFlag = (*Pbuf) & 0x3;		//填充充放电标志，此状态位不触发警报低电平中断上报

	g_stAfeState.OvUvFlag = 0;
	g_stAfeState.PassageFlag = 0;
	g_stAfeState.LdAltPdFlag = 0;
	g_stAfeState.CurFlag = 0;
	
	for(i = 0; i < len / 32; i++)				//填充电压保护标志，电压通道标志
	{
		tmp = (*(Pbuf + i * 8));	//取出首字节
		tmp1 =  (tmp & 0x10) >> 4;	//取出当前UV
		g_stAfeState.OvUvFlag |=  tmp1 << i;		//置位欠压UV位
		tmp1 =  (tmp & 0x20) >> 5;	//取出当前OV
		g_stAfeState.OvUvFlag |=  tmp1 << (i + 4);	//置位过压OV位

		tmp = (*(Pbuf + i * 8 + 1));	//取出1号字节
		tmp1 = (tmp & 0x2) >> 1;	//取出当前电压通道故障位
		g_stAfeState.PassageFlag |= tmp1 << (i + 4);	//置位电压通道故障位
	}

	g_stAfeState.PassageFlag |= ((*(Pbuf + 1)) & 0x1);	//置位电流通道故障位

	g_stAfeState.CurFlag |= ((*Pbuf) & 0x40) >> 6;	//置位OCC
	g_stAfeState.CurFlag |= ((*Pbuf) & 0x0c) >> 1;	//置位OCD, SCD
	
	g_stAfeState.LdAltPdFlag |= (((*(Pbuf + 1)) & 0x40) >> 6);		//负载和警报下拉位，不触发警报低电平中断上报
	g_stAfeState.LdAltPdFlag |= (((*Pbuf) & 0x80) >> 6);

	tmp = g_stAfeState.CurFlag | g_stAfeState.OvUvFlag | g_stAfeState.PassageFlag | g_stAfeState.LdAltPdFlag;
	if(tmp != 0)
	{
		g_stAfeState.ProtectFlag = 1;	//仅置位，由其他地方使用清除
	}
}


/*****************************************************************************
 函 数 名  : dvc10xx_TcAndTs_allGet
 功能描述  : 解析计算温度数据
 输入参数  : uint8_t *Pbuf   
             uint8_t bufLen  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_TcAndTs_allGet(uint8_t *Pbuf, uint8_t bufLen)
{
	uint8_t i, tmp, Tnum;
	double TcTstmp;

	Tnum = bufLen / 8;
	for(i = 0; i < Tnum; i++)
	{
		//计算出本次   TS 电压值
		tmp = i * 8;
		TcTstmp = ADC1_Calculate(*(Pbuf + tmp), *(Pbuf + tmp + 1));
		g_stAfeGet.TS[i] = TcTstmp;		//储存温感测量温度数据-----------此处仅是电压值，需要转换成对应的温度值


		//计算出本次   TC 电压值 ************  计算芯片内部温度
		tmp = i * 8 + 2;
		TcTstmp = ADC1_Calculate(*(Pbuf + tmp), *(Pbuf + tmp + 1));
		TcTstmp = TcTstmp / 1000000.0;  
		TcTstmp = -249.1366 * TcTstmp + 571.5225;
		g_stAfeGet.TC[i] = TcTstmp;		//储存芯片内部温度数据
		
	}

	if(Tnum > 1)
	{
		g_stAfeGet.TemperMax = g_stAfeGet.TS[0]; 
		g_stAfeGet.TemperMin = g_stAfeGet.TS[0]; 
		for(i = 1; i < Tnum; i++)
		{
			if(g_stAfeGet.TemperMax < g_stAfeGet.TS[i])
			{
				g_stAfeGet.TemperMax = g_stAfeGet.TS[i];
			}

			if(g_stAfeGet.TemperMin > g_stAfeGet.TS[i])
			{
				g_stAfeGet.TemperMin = g_stAfeGet.TS[i];
			}
		}
	}
	else
	{
		g_stAfeGet.TemperMax = g_stAfeGet.TS[0];
		g_stAfeGet.TemperMin = g_stAfeGet.TS[0];
	}
}



/*****************************************************************************
 函 数 名  : dvc10xx_OneBattVol_allGet
 功能描述  : 解析计算单串电池电压
 输入参数  : uint8_t *Pbuf   
             uint8_t bufLen  
             uint8_t flag    
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_OneBattVol_allGet(uint8_t *Pbuf, uint8_t bufLen, uint8_t flag)
{
	uint8_t i, j, tmp, num;
	double volTmp;

	for(i = 0; i < bufLen / 8; i++)	 //组电压,1,2,3,4
	{
		for(j = 0; j < 3; j++) 	 
		{			 
#ifdef	DVC_BATT_SHORT_CONNECT		//有短接情况
			if(DVC_CMD_RDSTRG1 == flag)
			{
				num = g_ucShortConnectMap[j + i * 6];
			}
			else
			{
				num = g_ucShortConnectMap[j + i * 6 + 3];
			}
			if(num != 0)
			{
				num--;

				tmp = i * 8 + j * 2;	 //计算出当前处理在数据中下标	 
				volTmp = ADC1_Calculate(*(Pbuf + tmp), *(Pbuf + tmp + 1)) / 1000000.0; 	 //uV值转化为V。

				g_stAfeGet.BattVol[num] = (float)volTmp;	 
			}
#else
			tmp = i * 8 + j * 2;	 //计算出当前处理在数据中下标	 
			volTmp = ADC1_Calculate(*(Pbuf + tmp), *(Pbuf + tmp + 1)) / 1000000.0; 	 //uV值转化为V。

			if(DVC_CMD_RDSTRG1 == flag)
			{
				num = j + i * 6 - 1;
			}
			else
			{
				num = j + i * 6 + 3 - 1;
			}

			g_stAfeGet.BattVol[num] = (float)volTmp;
#endif
		}
	}
}

/*****************************************************************************
 函 数 名  : dvc10xx_BattPackVol_allGet
 功能描述  : 解析计算电池组电压
 输入参数  : uint8_t *Pbuf   
             uint8_t bufLen  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_BattPackVol_allGet(uint8_t *Pbuf, uint8_t bufLen)	 //本函数不做读操作，仅供0组状态读取完调用
{
	uint8_t i, tmp;
	double VolTmp, VolTotal = 0;

	for(i = 0; i < bufLen / 8; i++)
	{
		 //计算出本电池组电压
		 tmp = i * 8 + 2;
		 VolTmp = ADC2_Calculate(*(Pbuf + tmp), *(Pbuf + tmp + 1)) / 1000.0;		 //1000.0.mV值转化为V。
		 g_stAfeGet.BattPackVol[i] = VolTmp;
		 VolTotal += VolTmp;
	}

	g_stAfeGet.BattTotalVol = VolTotal;
 }

 /*****************************************************************************
 函 数 名  : dvc10xx_Current_Get
 功能描述  : 解析计算电流值并判断充放电
 输入参数  : uint8_t *Pbuf   
             uint8_t bufLen  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_Current_Get(uint8_t *Pbuf, uint8_t bufLen)
{
	double current;
	uint16_t Vol;

	Vol = (*(Pbuf + 4) << 8) + *(Pbuf + 5);
	current = ((short int)Vol) * ADC_MAGIC_CURRENT / 1000;	  //单位mV。
	//此处电流值不是真正值， 算出来的是电压差值。需要结合测量电阻阻值计算。 电压值除以电阻值就是电流值。
	//电流压差测量范围-200~200mv。

	//此处电流方向用压差mV判断，亦可改成用电流A判断
	if(current > AFE_CUR_ZERO_THRESHOLD) 	  //如果电流有误差，使用范围判断
	{
		g_stDvcOpt.currentDir = E_AFE_CurDirDsg;	  //放电状态
	}
	else if(current < -(AFE_CUR_ZERO_THRESHOLD))	  //小于-阈值，判断为充电
	{
		g_stDvcOpt.currentDir = E_AFE_CurDirChg;	  //充电状态
	}
	else
	{
		g_stDvcOpt.currentDir = E_AFE_CurDirNo;	  //未充电
	}

	g_stAfeGet.Cur = 1000*current / AFE_CUR_SAMPLE_RES;	// V / Ω = A
	  
}


/*****************************************************************************
 函 数 名  : dvc10xx_CalculateMaxBattVol
 功能描述  : 计算单串电池最高电压
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_CalculateMaxBattVol(void)
{
	uint8_t i;

	g_stAfeGet.BattVolMax = g_stAfeGet.BattVol[0];
	for(i = 1; i < DVC_BATT_STRINGS_NUM; i++)
	{
		if(g_stAfeGet.BattVol[i] >= g_stAfeGet.BattVolMax)
		{
			g_stAfeGet.BattVolMax = g_stAfeGet.BattVol[i];
		}
	}
}

/*****************************************************************************
 函 数 名  : dvc10xx_CalculateMinBattVol
 功能描述  : 计算单串电池最低电压
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_CalculateMinBattVol(void)
{
	uint8_t i;

	g_stAfeGet.BattVolMin = g_stAfeGet.BattVol[0];
	for(i = 1; i < DVC_BATT_STRINGS_NUM; i++)
	{
		if(g_stAfeGet.BattVolMin >= g_stAfeGet.BattVol[i])
		{
			g_stAfeGet.BattVolMin = g_stAfeGet.BattVol[i];
		}
	}
}

/*****************************************************************************
 函 数 名  : dvc10xx_STCFG_TempDataInit
 功能描述  : 初始化临时配置全局变量
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_STCFG_TempDataInit(void)
{
	uint8_t i, len;
	uint8_t *p1, *p2;

	len = 2 * 4 * 6;
	
	p1 = &g_ucDvc_cfg[0][0][0];
	p2 = (uint8_t *)&dvc_cfg[0][0][0];
	for(i = 0; i < len; i++)
	{
		*p1 = *p2;
		p1++;
		p2++;
	}	
}

/*****************************************************************************
 函 数 名  : dvc10xx_DataStructInit
 功能描述  : 全局数据结构初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_DataStructInit(void)
{
	dvc10xx_BattNumMapInit();	//电池标号对应，用于不足串数电压值对齐。
	MemSet((uint8_t *)&g_stDvcOpt, 0x0, sizeof(st_DvcOpt));
	dvc10xx_STCFG_TempDataInit();	
}



/*****************************************************************************
 函 数 名  : dvc10xx_FrameRead
 功能描述  : 同步SPI帧数据读取
 输入参数  : uint8_t dvcSpiCmdNum  
             uint8_t *bufPtr       
             uint8_t bufLen        
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
static uint8_t dvc10xx_FrameRead(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen)
{
	uint8_t i, readTmp, delay;
	uint8_t bufTmp[32];
#ifdef DVC_DEBUG
	uint8_t j;
#endif

	if(bufLen == 8)		//读取长度为8字节，用1M速率。
	{
		delay = 1;
	}
	else
	{
		delay = 63;		//延时63*2 = 125us。
	}
	
	//拉低片选
	CSB_S_L;
	bsp_DelayUS(delay);
	
	//发送命令字节
	for(i = 0; i < 4; i++)
	{
		if(SPI2_WriteReadByte(dvc_cmd[dvcSpiCmdNum][i], &readTmp) != 0)
		{
			CSB_S_H;
			bsp_DelayUS(2 * delay);
			return 0xff;
		}
		bsp_DelayUS(1);
	}

	//读取数据字节
	for(i = 0; i < bufLen; i++)
	{
		bsp_DelayUS(1);
		if(SPI2_WriteReadByte(0xff, &bufTmp[i]) != 0)
		{
			CSB_S_H;
			bsp_DelayUS(2 * delay);
			return 0xff;
		}
	}

	//拉高片选
	bsp_DelayUS(delay);
	CSB_S_H;
	bsp_DelayUS(2 * delay);

	//pec校验
	if(dvc_PEC_CRC_Check(bufTmp, bufLen, &readTmp) != DVC_SUCCESS)
	{
	
		return 0xff;
	}

	for(i = 0; i < bufLen; i++)
	{
		*bufPtr = bufTmp[i];
		bufPtr++;
	}

	return 0;
}

/*****************************************************************************
 函 数 名  : dvc10xx_FrameWrite
 功能描述  : 同步SPI帧数据写入
 输入参数  : uint8_t dvcSpiCmdNum  
             uint8_t *bufPtr       
             uint8_t bufLen        
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
static uint8_t dvc10xx_FrameWrite(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen)
{
	uint8_t i, j, num, readTmp, delay;
	uint8_t writeBuf[DVC_MAX_DATA_BYTE] = {0};

	if(bufLen == 8)		//读取长度为8字节，用1M速率。
	{
		delay = 1;
	}
	else
	{
		delay = 63;		//延时63*2 = 125us。
	}

	//添加PEC校验
	//往发送buff中填充数据,先将高电池组的数据取出来，最先发出
	for(i = 0; i < bufLen / 8; i++)
	{
	    for(j = 0; j < 6; j++)
	    {
	    	num = (bufLen / 8 - i - 1) * 8 + j;
			writeBuf[num] = *bufPtr;
			bufPtr++;
		}

		dvc_PEC_CRC_Attend(&writeBuf[num - 5], 6); 	//	填充PEC	
	}
		
	//拉低片选
	CSB_S_L;
	bsp_DelayUS(delay);
	
	//发送命令字节
	for(i = 0; i < 4; i++)
	{
		if(SPI2_WriteReadByte(dvc_cmd[dvcSpiCmdNum][i], &readTmp) != 0)
		{
			CSB_S_H;
			bsp_DelayUS(2 * delay);
			return 0xff;
		}
		bsp_DelayUS(1);
	}

	//发送数据字节
	for(i = 0; i < bufLen; i++)
	{
		if(SPI2_WriteReadByte(writeBuf[i], &readTmp) != 0)
		{
			CSB_S_H;
			bsp_DelayUS(2 * delay);
			return 0xff;
		}
		bsp_DelayUS(1);
	}

	//拉高片选
	bsp_DelayUS(delay);
	CSB_S_H;
	bsp_DelayUS(2 * delay);
	
	return 0;
}

/*****************************************************************************
 函 数 名  : dvc10xx_FrameRead_Check
 功能描述  : 帧数据读取，并校验读取数据。并可重试。
 输入参数  : uint8_t dvcSpiCmdNum  
             uint8_t *bufPtr       
             uint8_t bufLen        
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
E_DvcCmdRet dvc10xx_FrameRead_Check(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen)
{
	uint8_t j, ret;
	
	for(j = 0; j < DVC_CMD_RESOLUTION_REPEAT; j++)
	{
		ret = dvc10xx_FrameRead(dvcSpiCmdNum, bufPtr, bufLen);

		if(ret != 0)
		{
			continue;
		}

		if(ret == 0)
		{
			return DVC_SUCCESS;
		}
	}

	return DVC_RepeatErr;
}

/*****************************************************************************
 函 数 名  : dvc10xx_FrameWrite_Check
 功能描述  : 帧数据写入，并检验写入前后是否一致。并可重试
 输入参数  : uint8_t dvcSpiCmdNum  
             uint8_t *bufPtr       
             uint8_t bufLen        
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
static E_DvcCmdRet dvc10xx_FrameWrite_Check(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen)		//用于写入数据并读取检验 *bufPtr不带校验值
{
	uint8_t i, j, k, ret;
	uint8_t SpiCmdNum;

	uint8_t bufPecTmp[32] = {0};

	for(i = 0; i < DVC_CMD_RESOLUTION_REPEAT; i++)
	{
		ret = dvc10xx_FrameWrite(dvcSpiCmdNum, bufPtr, bufLen);
		if(ret != 0)
		{
			continue;
		}

		if(DVC_CMD_WRCFRG0 == dvcSpiCmdNum)		//写命令转换成读
		{
			SpiCmdNum = DVC_CMD_RDCFRG0;
		}

		if(DVC_CMD_WRCFRG1 == dvcSpiCmdNum)		//写命令转换成读
		{
			SpiCmdNum = DVC_CMD_RDCFRG1;
		}
			
		ret = dvc10xx_FrameRead_Check(SpiCmdNum, bufPecTmp, bufLen);	//保证读取数据成功
		if(ret == DVC_SUCCESS)
		{
			for(j = 0; j < bufLen / 8; j++)		//判断读写是否一致
			{
				for(k = 0; k < 6; k++)
				if(*(bufPtr + j * 6 + k) != (bufPecTmp[j * 8 + k]))
				{
					goto BREAK_THIS;	//跳出本次循环
				}
			}

			return DVC_SUCCESS;
		}
		BREAK_THIS:
		;
	}

	return DVC_RepeatErr;
}

/*****************************************************************************
 函 数 名  : dvc10xx_init_cfg
 功能描述  : dvc配置初始化函数
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年5月8日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_init_cfg(void)
{
	uint8_t ret, i;
	uint8_t *p;
	
	for(i = 0; i < DVC_CMD_RESOLUTION_REPEAT; i++)			//主控端突然复位，从设备第一次操作可能出错
	{
		p = (uint8_t *)&dvc_cfg[0][0][0];	//初始化配置寄存器0
		ret = dvc10xx_FrameWrite_Check(DVC_CMD_WRCFRG0, p, DVC_MAX_DATA_BYTE);
		if(0 != ret)
			continue;	//下次重试
		
		p = (uint8_t *)&dvc_cfg[1][0][0];	//初始化配置寄存器1
		ret = dvc10xx_FrameWrite_Check(DVC_CMD_WRCFRG1, p, DVC_MAX_DATA_BYTE);
		if(0 == ret)
		{
			return;
		}
	}
}


/*****************************************************************************
 函 数 名  : dvc10xx_GpioExit_Init
 功能描述  : 芯片控制GPIO和外部中断初始化
 输入参数  : 无
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_GpioExit_Init()
{
	// GPIO_InitTypeDef gpio_init_struct;
	// EXTI_InitTypeDef EXTI_InitStructure;
	// NVIC_InitTypeDef NVIC_InitStructure;

	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//PORTC时钟使能
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);		//使能外部中断时钟
	
	// //WAUP  Gpio初始化
	// gpio_init_struct.GPIO_Pin = GPIO_DVC_WAUP_PIN;
	// gpio_init_struct.GPIO_Speed =GPIO_Speed_50MHz;
	// gpio_init_struct.GPIO_Mode =GPIO_Mode_Out_PP;
	// GPIO_Init(GPIO_DVC_WAUP_PORT, &gpio_init_struct);
	// GPIO_SetBits(GPIO_DVC_WAUP_PORT, GPIO_DVC_WAUP_PIN);

	// //ALERT Gpio初始化
	// gpio_init_struct.GPIO_Pin = GPIO_DVC_ALERT_PIN;
	// gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;
	// GPIO_Init(GPIO_DVC_ALERT_PORT, &gpio_init_struct);

	// //外部中断初始化
	// //ALERT 中断线及中断初始化
  	// GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);

  	// EXTI_InitStructure.EXTI_Line=EXTI_Line7;
  	// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  	// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	// EXTI_Init(&EXTI_InitStructure);

	// NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;				//使能外部中断通道
  	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2
  	// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;			//子优先级0
  	// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;					//使能外部中断通道
  	// NVIC_Init(&NVIC_InitStructure); 
}

/*****************************************************************************
 函 数 名  : DVC10xx_DeviceInit
 功能描述  : dvc设备初始化函数
 		   1、注册芯片类型：：：（用户注册）
 		   2、代码中根据芯片类型，
 		      自动注册速率spiSpeedType
 		      初始化定时器TIM6_Int_Init，

 		   3、初始化模拟GPIO管脚GPIO_IMITATE_SPI_Init
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月29日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void DVC10xx_DeviceInit(void)
{
	SPI2_GpioInit();
	dvc10xx_GpioExit_Init();

/* BEGIN: Modified by 武俊杰, 2021/8/24 */
#ifndef DVC1006		//1024/1018/1012初始化频率为8K.
/* END:   Modified by 武俊杰, 2021/8/24   PN: */
	SPI2_Init(SPI_PRESCALER_256);	//SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.007812
#else				//1006初始化频率为1M.
	SPI2_Init(SPI_PRESCALER_4);		//SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.5  配置成1M有问题
#endif

	dvc10xx_BattNumMapInit();		//初始化电池序号映射表。
	/* BEGIN: Added by 武俊杰, 2021/8/6 */
	dvc10xx_SleepWakeup();			//首次启动先唤醒DVC芯片
	/* END:   Added by 武俊杰, 2021/8/6   PN: */
	dvc10xx_init_cfg();				//写入配置数据到芯片
}

/*****************************************************************************
 函 数 名  : dvc10xx_globle_cfg0_update
 功能描述  : 更新全局配置寄存器数据
 输入参数  : E_DVC_FUNC_FLAG funcFlag  
             uint8_t backNum  	转化为从0开始。比如1~24节电池，传入前-1              
             uint8_t status                 
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年5月8日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_globle_cfg0_update(E_DVC_FUNC_FLAG funcFlag, uint8_t battNum, uint8_t status)
{
	uint8_t i, j;

	switch(funcFlag)
	{
		case DVC_Func_PassEqual:	//更新电池被动均衡
		{
			if(battNum == 0xff)		//更新全部节电池被动均衡
			{
				if(status == 0)		//关闭全部被动均衡
				{
					for(i = 0; i < DVC_PACK_NUMBER; i++)
					{
						g_ucDvc_cfg[0][i][1] &= ~(0x3f);
					}
				}
				else				//打开全部被动均衡
				{
					for(i = 0; i < DVC_PACK_NUMBER; i++)
					{
						g_ucDvc_cfg[0][i][1] |= 0x3f;
					}
				}
			}
			else
			{	//根据backNum处理
				i = battNum / 6;	//取出对应组标号
				j = battNum % 6;	//取出对应组对应位
			
				if(status == 0)		//关闭backNum节被动均衡
				{
					g_ucDvc_cfg[0][i][1] &= ~(0x1 << j); 
				}
				else				//打开backNum节被动均衡
				{
					g_ucDvc_cfg[0][i][1] |= (0x1 << j); 
				}
			}

			break;
		}
		case DVC_Func_CHGDSG_EN:	//更新充放电驱动开关状态
		{
			if(status == 0)		//关闭充放电驱动
			{
				g_ucDvc_cfg[0][0][0] &= ~(0x3); 
			}
			else if(status == 1)	//打开充放电驱动
			{
				g_ucDvc_cfg[0][0][0] |= 0x3; 
			}else if(status == 2)   //打开充电驱动 ，关闭放电MOS
			{
				g_ucDvc_cfg[0][0][0] &= ~(0x3); 
				g_ucDvc_cfg[0][0][0] |= 0x1; 
			}
			else if(status == 3)   //打开放电驱动，关闭充电MOS
			{
				g_ucDvc_cfg[0][0][0] &= ~(0x3); 
				g_ucDvc_cfg[0][0][0] |= 0x2; 
			}

			break;
		}
		case DVC_Func_MASK_OV:		//更新屏蔽过充电压位
		{
			if(status == 0)		//关闭屏蔽充电过压
			{
				g_ucDvc_cfg[0][0][5] &= ~(0x8); 
			}
			else			//打开屏蔽充电过压
			{
				g_ucDvc_cfg[0][0][5] |= 0x8; 

			}

			break;
		}
		case DVC_Func_MASK_UV:		//更新屏蔽放电欠压位
		{
			if(status == 0)		//关闭屏蔽放电欠压
			{
				g_ucDvc_cfg[0][0][4] &= ~(0x4); 
			}
			else			//打开屏蔽放电欠压
			{
				g_ucDvc_cfg[0][0][4] |= 0x4; 
			}

			break;
		}
		default:
			;
	}
}

void dvc10xx_globle_cfg1_update(void)
{
	//检测倍数，保护延时，保护阈值等数据。一般在初始化阶段配置好就可以，不需要实时配置。
}



/*****************************************************************************
 函 数 名  : dvc10xx_CalculateMaxWriteLen
 功能描述  : 计算写入配置数据长度
 输入参数  : uint8_t num  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_CalculateMaxWriteLen(uint8_t num)
{
	uint8_t wrLen;
	
	if(num < 6)
	{
		wrLen = 8;
	}
	else if(num < 12)
	{
		wrLen = 16;
	}
	else if(num < 18)
	{
		wrLen = 24;
	}
	else if(num < 24)
	{
		wrLen = 32;
	}
	else if(num == 0xff)
	{
		wrLen = 32;
	}

	return wrLen;
}

/*****************************************************************************
 函 数 名  : dvc10xx_FuncFlushCfg
 功能描述  : 根据功能刷新配置数据
 输入参数  : E_DVC_FUNC_FLAG funcFlag 		功能索引值 
             uint8_t battNum        	   参数序号        
             uint8_t status                配置状态
 输出参数  : 无
 返 回 值  : uint8_t 写入操作结果
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_FuncFlushCfg(E_DVC_FUNC_FLAG funcFlag, uint8_t battNum, uint8_t status)
{
	uint8_t ret, num, wrLen;
	
	switch(funcFlag)
	{
		case DVC_Func_PassEqual:		//更新电池被动均衡
		{
			if(0xff != battNum)
			{
				num = dvc10xx_BattNum_ToChipInnerNum(battNum);		//从电池序号映射到芯片内部序号。			
			}
			else
			{
				num = 0xff;
			}
			wrLen = dvc10xx_CalculateMaxWriteLen(num);		//计算写入字节长度
			dvc10xx_globle_cfg0_update(DVC_Func_PassEqual, num, status);		//battNum = 0xff，打开全部均衡。=某一序号，打开该序号均衡。

			if(num == 8)
			{
				goto WRITE_8_BYTES_CFG0;
			}
			
			break;
		}
		case DVC_Func_CHGDSG_EN:		//更新充电驱动开关状态
		{
			dvc10xx_globle_cfg0_update(DVC_Func_CHGDSG_EN, 0, status);
			goto WRITE_8_BYTES_CFG0;
		}
		case DVC_Func_MASK_OV:		//更新屏蔽过充电压位
		{
			dvc10xx_globle_cfg0_update(DVC_Func_MASK_OV, 0, status);
			goto WRITE_8_BYTES_CFG0;
		}
		case DVC_Func_MASK_UV:		//更新屏蔽放电欠压位
		{
			dvc10xx_globle_cfg0_update(DVC_Func_MASK_UV, 0, status);
			goto WRITE_8_BYTES_CFG0;
		}
		default:
			;
	}

	ret = dvc10xx_FrameWrite_Check(DVC_CMD_WRCFRG0, &g_ucDvc_cfg[0][0][0], wrLen);	
	return ret;

	WRITE_8_BYTES_CFG0:
		SPI2_Init(SPI_PRESCALER_4);	  //SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.5 配置成1M有问题
		ret = dvc10xx_FrameWrite_Check(DVC_CMD_WRCFRG0, &g_ucDvc_cfg[0][0][0], 8U);
		SPI2_Init(SPI_PRESCALER_256);   //SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.007812
		return ret;	
}

/*****************************************************************************
 函 数 名  : dvc10xx_cmd_sleep_check
 功能描述  : 休眠并检查。可重试保证休眠成功
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_cmd_sleep_check(void)
{
	uint8_t tmp[8] = {0}, i;
	uint8_t byteTmp = 0xff;

	for(i = 0; i < DVC_CMD_RESOLUTION_REPEAT; i++)
	{
		dvc10xx_FrameWrite(DVC_CMD_SLEEP, &byteTmp, 1);	//DVC1012/18/24执行周期为5ms左右,DVC1006 US级

		//检测状态寄存器前8字节数据，PEC校验是否正确。
		if(dvc10xx_FrameRead_Check(DVC_CMD_RDCFRG0, tmp, 8) != DVC_SUCCESS)
		{
			return DVC_SUCCESS;
		}
	}

	return DVC_RepeatErr;
}

/*****************************************************************************
 函 数 名  : dvc10xx_SleepWakeup
 功能描述  : 休眠唤醒信号操作
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_SleepWakeup(void)
{
	//WKUP  AFE芯片要求先拉低1us, 再拉高。需要上升沿唤醒。
	WAUP_S_H;
	bsp_DelayUS(1);
	WAUP_S_L;
	bsp_DelayUS(1);
	WAUP_S_H;
	/* BEGIN: Added by 武俊杰, 2021/8/6 */
	bsp_DelayMS(32);		//延时32ms。
	/* END:   Added by 武俊杰, 2021/8/6   PN: */
}

/*****************************************************************************
 函 数 名  : dvc10xx_MOS_QuikerOpen
 功能描述  : 快速打开充放电MOS
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_MOS_QuikerOpen(void)
{
	return dvc10xx_FuncFlushCfg(DVC_Func_CHGDSG_EN, 0, 1);
}

/*****************************************************************************
 函 数 名  : dvc10xx_MOS_QuikerClose
 功能描述  : 快速关闭充放电MOS
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_MOS_QuikerClose(void)
{
	return dvc10xx_FuncFlushCfg(DVC_Func_CHGDSG_EN, 0, 0);
}

/*****************************************************************************
 函 数 名  : dvc10xx_DMOS_Close_CMOS_Open
 功能描述  : 快速关闭放电MOS,保留充电MOS状态
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_DMOS_Close_CMOS_Open(void)
{
	return dvc10xx_FuncFlushCfg(DVC_Func_CHGDSG_EN, 0, 2);
}

/*****************************************************************************
 函 数 名  : dvc10xx_DMOS_Close_CMOS_Open
 功能描述  : 快速关闭放电MOS,保留充电MOS状态
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_DMOS_Open_CMOS_Close(void)
{
	return dvc10xx_FuncFlushCfg(DVC_Func_CHGDSG_EN, 0, 3);
}





/*****************************************************************************
 函 数 名  : dvc10xx_Current_QuikerGet
 功能描述  : 快速读取电流数据
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_Current_QuikerGet(void)
{
	uint8_t BufTmp[8];
	uint8_t ret;

	//此处SPI控制器有频率切换
	SPI2_Init(SPI_PRESCALER_4);	  //SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.5 配置成1M有问题
	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG0, BufTmp, 8U);
	SPI2_Init(SPI_PRESCALER_256);   //SYSCLK = 64, PCLK1 = 2,  SPICLK = 0.007812
	if(ret == 0)
	{
		dvc10xx_Current_Get(BufTmp, 8U);	  
	}   
}

/*****************************************************************************
 函 数 名  : dvc10xx_cmd_clear
 功能描述  : 清除指令操作
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_cmd_clear(void)		   //需要判断执行后是否真正清除
{
	uint8_t byteTmp = 0xff;

	dvc10xx_FrameWrite(DVC_CMD_CLEAR, &byteTmp, 1); //DVC1012/18/24执行周期为5ms左右,DVC1006 US级
}

/*****************************************************************************
 函 数 名  : dvc10xx_cmd_clear_RegCheck
 功能描述  : 清除指令后检查
 输入参数  : uint8_t *Pbuf  
             uint8_t len    
 输出参数  : 无
 返 回 值  : uint8_t 是否完全清除
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_cmd_clear_RegCheck(uint8_t *Pbuf, uint8_t len)	   //需要判断执行后是否真正清除
{   //检查状态0组 0字节 [7:2]。6/12/18字节 [5:4]。
	uint8_t tmp, i;

	for(i = 0; i < DVC_MAX_DATA_BYTE/8; i++)
	{
		if(i == 0)
		{
			tmp = 0xfc;	   //检查状态寄存器0，0字节
		}
		else
		{
			tmp = 0x30;	   //检查状态寄存器0，6字节（第二组电池首字节），12字节，18字节 
		}

		if(*(Pbuf + i * 8) & tmp)
		{
			return ERROR;
		}	   
	}

	return SUCCESS;
}

/*****************************************************************************
 函 数 名  : dvc10xx_CmdClearManage
 功能描述  : 清除警报指令管理（发送指令，读取状态并检查）
 输入参数  : void  
 输出参数  : 无
 返 回 值  : uint8_t 是否完成完全清除
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t dvc10xx_CmdClearManage(void)
{
	uint8_t ret;
	uint8_t rdBuf[DVC_MAX_DATA_BYTE] = {0};
	
	dvc10xx_cmd_clear();	//发送清除命令

	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG0, rdBuf, DVC_MAX_DATA_BYTE);	//读取状态寄存器
	if(ret == DVC_SUCCESS)
	{
		ret = dvc10xx_cmd_clear_RegCheck(rdBuf, DVC_MAX_DATA_BYTE);		//检查数据
		if(ret == DVC_SUCCESS)
		{
			return DVC_SUCCESS;
		}
	}

	return DVC_ERROR;
}



/*****************************************************************************
 函 数 名  : dvc10xx_StatePackVolCur_Get
 功能描述  : 读取并解析芯片状态，组电压数据，电流数据
                 
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void dvc10xx_StatePackVolCur_Get(void)
{
	uint8_t DataBuf[32] = {0};
	uint8_t ret;

	//STRG0中包含：小组电池电压，电流压差
	//OCC   SCD  OCD	  ALT_PD  DSG_ST  CHG_ST  LD  CS_ST 	  可通过1M频率读取此状态	  
	//OV1(/2/3/4)    UV1(/2/3/4)	  VS1(/2/3/4)_ST			  OV2以上需要8K频率读取此状态位
	ret = dvc10xx_FrameRead_Check(DVC_CMD_RDSTRG0, DataBuf, DVC_MAX_DATA_BYTE); 
	if(ret == 0)
	{
		dvc10xx_BattPackVol_allGet(DataBuf, DVC_MAX_DATA_BYTE);	  //更新6节电池总电压
		dvc10xx_Current_Get(DataBuf, 8U);   //更新电流
		dvc10xx_AlertState_Get(DataBuf, DVC_MAX_DATA_BYTE);
	}
}


#endif



