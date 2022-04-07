#ifndef __DVC10xx_AFE_H
#define __DVC10xx_AFE_H
#include "includes.h"

#if(AFE_CHIP_SELECT == 2)

#ifdef __cplusplus
extern "C" {
#endif

#include "dvc10xx.h"


#define ADC_MAGIC_1 	    183.1		//单位为uV电压		计算TC，Ts.单个电池电压
#define ADC_MAGIC_2 	    1.465		//单位为mV电压		计算电池组电压
#define ADC_MAGIC_CURRENT 	9.155		//单位为uV.  SRP和SRN之间的压差


//记录采集数据
typedef struct
{
	float BattVol[DVC_BATT_STRINGS_NUM];		//单串电池电压值
	float BattVolMax;		//单串电池电压最高值
	float BattVolMin;		//单串电池电压最低值
	float BattPackVol[DVC_PACK_NUMBER];		//1~6串为一组，4组电池pack电压
	float BattTotalVol;		//总电压
	float TS[DVC_PACK_NUMBER];			//外部温感检测温度
	float TC[DVC_PACK_NUMBER];			//芯片内部温度
	float TemperMax;		//外部温感检测最大温度
	float TemperMin;		//外部温感检测最小温度
	float Cur;				//当前采集电流
	float CurAverage;		//采集的平均电流，可用于电量计算
}st_AfeGet;

//记录AFE芯片状态
typedef struct
{
	uint8_t ProtectFlag;		//芯片有警报,状态位查询的警报。由于只能外部中断只能下降沿触发一次
	uint8_t ProtectExitFlag; //外部中断上报的警报
	
	//--------------------------------下面是详细分类
	uint8_t OvUvFlag;		//过压欠压警报标志 bit0/1/2/3---Uv1/2/3/4           bit4/5/6/7---Ov1/2/3/4
	uint8_t CurFlag;			//电流警报标志		   bit0/1/2/3---OCC/OCD/SCD
	uint8_t PassageFlag;		//通道警报标志           bit0---电流通道故障          bit4/5/6/7---电压通道故障1/2/3/4 
	uint8_t ChDshFlag;       //充放电状态标志位 bit0---充电               bit1---放电
	uint8_t LdAltPdFlag;     //bit0---LD负载检测       bit1---ALT下拉标志位
	uint8_t OverTpFlag;		//温度超温标志		
}st_AfeState;

//AFE控制结构
typedef struct
{
	uint8_t TemperOpFlag;	//温度操作标志 
	uint8_t TemperCount;		//超过阈值温度计数，多次超过阈值采取关闭充放电
	uint8_t BattVolOpFlag;	//电池电压操作标志 
	uint8_t BattVolCount;	//超过阈值电压计数，多次达到阈值范围内，可恢复充放电
	uint8_t ChgDsgAltCount;	//充放电警报计数
	uint8_t AllowChg;		//允许充电标志
	uint8_t AllowDsg;		//允许放电标志
	uint8_t ChgDsgInt;      	//充放电中断标志，用于有外部电路检测充电器，负载。
	uint8_t MaskOvFlag;		//放电屏蔽标志，屏蔽充放电过压
	uint8_t MaskUvFlag;		//充电屏蔽标志，屏蔽充放电欠压
}st_AfeControl;

//定时器定时标志
typedef enum 
{
	E_AFE_TimNoOp = 0, 	//未操作
	E_AFE_Tim64MS,		//64ms
	E_AFE_Tim256MS,		//256ms
	E_AFE_Tim1S,		//1s
	E_AFE_Tim10S,		//10s
	E_AFE_TimErr		//未定义错误
} E_AFE_TimerFlag;

//电流方向标志
typedef enum 
{
	E_AFE_CurDirNo = 0, 	//未操作
	E_AFE_CurDirChg,		//充电
	E_AFE_CurDirDsg,		//放电
	E_AFE_CurDirErr			//未定义错误
} E_AFE_CurDirFlag;

//休眠标志
typedef enum 
{
	E_AFE_SleepNoOp = 0, 	//未操作
	E_AFE_SleepOp,			//休眠
	E_AFE_SleepOpLast,		//休眠过了
	E_AFE_SleepWaUp,		//休眠后唤醒
	E_AFE_SleepErr			//未定义错误
} E_AFE_SleepFlag;

extern uint8_t g_ucShortConnectMap[DVC_BATT_STRINGS_NUM_MAX];	//实际电池串和芯片内部电池串映射表
extern st_AfeGet g_stAfeGet;								//AFE采集数据结构
extern st_AfeState g_stAfeState;							//AFE状态数据结构


/*****************************************************************************
 函 数 名  : AFE_TimerInt_callback
 功能描述  : 中断处理SPI流程，回调函数，配合中断次数调整取出放入数据和读写时序
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年4月28日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void Dvc10xx_AFE_Init(void);

/*****************************************************************************
 函 数 名  : AFE_MAIN_Scanf
 功能描述  : dvc芯片在main主函数中，扫描执行操作
 			用于spi操作结束后，业务处理过程
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年5月7日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void Dvc10xx_AFE_Process(void);

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

void Afe_DataStruct_Init(void);


uint8_t Dvc10xx_AFE_GetTempProtectState();

uint16_t Dvc10xx_AFE_JudgeState(void);

uint8_t Dvc10xx_AFE_GetCadcCurrent(uint16_t *curr);

void Dvc10xx_AFE_GetCellsVolt(uint16_t *CellVolt);

void Dvc10xx_AFE_GetTemp(uint16_t *TempNTC);

//全开
void Dvc10xx_AFE_OpenDsgMos(void);
//全close
void Dvc10xx_AFE_CloseDsgMos(void);
//只open chgMOS
void Dvc10xx_AFE_OpenChgMos_CLoseDsg(void);
//只open DsgMOS
void Dvc10xx_AFE_OpenDsgMos_CloseChg(void);


#ifdef __cplusplus
}
#endif

#endif /*AFE_CHIP*/

#endif /* __AFE_H */