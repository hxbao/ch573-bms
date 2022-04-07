#ifndef INCLUDE_DVC10XX_H_
#define INCLUDE_DVC10XX_H_

#include "includes.h"



#if (AFE_CHIP_SELECT == 2)

#define DVC1018 1 //本例程使用DVC1024。打开芯片宏

//软件控制NSS
#define GPIO_SPI_CSB_PIN	               TN_SPI_CS_PIN  //GPIO_Pin_12
#define GPIO_SPI_CSB_PORT                  TN_SPI_CS_PORT //GPIOB

#define CSB_S_H    MCU_GPIO_SetBit(GPIO_SPI_CSB_PORT, GPIO_SPI_CSB_PIN)  	
#define CSB_S_L    MCU_GPIO_ClrBit(GPIO_SPI_CSB_PORT, GPIO_SPI_CSB_PIN)

//休眠唤醒WAUP
#define GPIO_DVC_WAUP_PIN	               TN_DVC_WAUP_PIN//GPIO_Pin_6
#define GPIO_DVC_WAUP_PORT                 TN_DVC_WAUP_PORT//GPIOC

#define WAUP_S_H    MCU_GPIO_SetBit(GPIO_DVC_WAUP_PORT, GPIO_DVC_WAUP_PIN)  	
#define WAUP_S_L    MCU_GPIO_ClrBit(GPIO_DVC_WAUP_PORT, GPIO_DVC_WAUP_PIN)

//警报中断ALEART
#define GPIO_DVC_ALERT_PIN	               TN_DVC_ALERT_PIN   //GPIO_Pin_7
#define GPIO_DVC_ALERT_PORT                TN_DVC_ALERT_PORT  //GPIOC
#define ALERT_PIN                          MCU_GPIO_GetBit(GPIO_DVC_ALERT_PORT, GPIO_DVC_ALERT_PIN)   //GPIO_ReadInputDataBit(GPIO_DVC_ALERT_PORT, GPIO_DVC_ALERT_PIN)

//为保证命令执行能够每次都成功，设置失败会再重复次数
#define DVC_CMD_RESOLUTION_REPEAT 3U		


//PEC种子值，多项式均逻辑左移一位
#define PEC_SAW_VEL 		0x0020		//PEC种子值
#define PEC_QUANTIC_VEL		0x8B32		//PEC多项式


//配置一帧数据的最大字节数
#ifdef DVC1024
#define DVC_MAX_DATA_BYTE 32U		//最大数据字节数
#define DVC_PACK_NUMBER    4		//数据组数
#elif DVC1018
#define DVC_MAX_DATA_BYTE 24U
#define DVC_PACK_NUMBER    3
#elif DVC1012
#define DVC_MAX_DATA_BYTE 16U
#define DVC_PACK_NUMBER    2
#elif DVC1006
#define DVC_MAX_DATA_BYTE 8U
#define DVC_PACK_NUMBER    1

#else
//后续型号
#endif

//电池串数		短接方式（不足串数）的电压采集可通过此宏判断操作
#define DVC_BATT_STRINGS_NUM  24							//真实电池串数    
#define DVC_BATT_STRINGS_NUM_MAX  24						//最大可接电池串数
#define DVC_BATT_MASK_FLAG  	0	//每一位为1表示有短接，为0表示没有短接，不需要跳过，对应高串到低串
#define DVC_BATT_SHORT_CONNECT  0	//0没有短接，1有短接


//DVC设备操作结构
typedef struct 
{
	/***********************控制字段*****************************/
	uint8_t chipStatOkFlag;	//芯片状态
	uint8_t updateVolflag;	//电压温度更新标志
	uint8_t currentDir;		//电流方向
	uint8_t sLeepFlag;		//休眠标志
	/************************四字节*****************************/
	uint16_t timerCountMs;     //定时计数ms
	uint8_t	timerCountSec;	   //定时计数sec
	uint8_t timerFlag;  	   //定时器标志
	/************************四字节*****************************/
	uint8_t timerSleepCount;   //无充电放电计数
	/************************四字节*****************************/
}st_DvcOpt;


//DVC操作返回类型
typedef enum 
{
	DVC_SUCCESS = 0, 	//操作成功
	DVC_ERROR,
	DVC_LastNoCom,		//需要等待中断操作
	DVC_PECWrong,		//PEC检测失败
	DVC_ParaWrong,		//参数错误
	DVC_RepeatErr,		//经DVC_CMD_RESOLUTION_REPEAT重试次数仍然失败
	DVC_ErrorNoOp		//未定义错误
} E_DvcCmdRet;

//DVC功能类型，可用于更新全局配置寄存器数据
typedef enum 
{
	DVC_Func_PassEqual = 0, //更新电池被动均衡开关状态
	DVC_Func_CHGDSG_EN,		//更新充放电驱动开关状态
	DVC_Func_MASK_OV,		//更新屏蔽过压位
	DVC_Func_MASK_UV,		//更新屏蔽欠压位
	DVC_Func_End
} E_DVC_FUNC_FLAG;

//DVC 命令索引
enum DVCSpiCmd
{
	DVC_CMD_WRCFRG0 = 0,	//写入配置寄存器0组
	DVC_CMD_WRCFRG1,		//写入配置寄存器1组
	DVC_CMD_RDSTRG0,		//读取状态寄存器0组
	DVC_CMD_RDSTRG1,		//读取状态寄存器1组
	DVC_CMD_RDSTRG2,		//读取状态寄存器2组
	DVC_CMD_RDSTRG3,		//读取状态寄存器3组
	DVC_CMD_RDCFRG0,		//读取状态寄存器0组
	DVC_CMD_RDCFRG1,		//读取状态寄存器1组
	DVC_CMD_CLEAR = 8,		//清除指令
	DVC_CMD_SLEEP			//睡眠指令
};

extern st_DvcOpt g_stDvcOpt;		//dvc芯片控制结构体全局变量

void MemSet(uint8_t   *p, uint8_t val, uint8_t length);	//初始化存储空间函数

void dvc10xx_GpioExit_Init();
void DVC10xx_DeviceInit(void);				//dvc设备初始化函数
void dvc10xx_DataStructInit(void);			//初始化芯片涉及数据结构
void dvc10xx_init_cfg(void);				//初始化芯片配置

//带有重试和PEC校验的帧数据读取函数
E_DvcCmdRet dvc10xx_FrameRead_Check(uint8_t dvcSpiCmdNum, uint8_t *bufPtr, uint8_t bufLen);

void dvc10xx_CalculateMaxBattVol(void);	//计算单串电池电压最高值
void dvc10xx_CalculateMinBattVol(void);	//计算单串电池电压最低值
void dvc10xx_OneBattVol_allGet(uint8_t *Pbuf, uint8_t bufLen, uint8_t flag);		//解析单串电压值
void dvc10xx_TcAndTs_allGet(uint8_t *Pbuf, uint8_t bufLen);		                    //解析温度

void dvc10xx_StatePackVolCur_Get(void); //获取解析芯片状态，组电压，电流
void dvc10xx_Current_QuikerGet(void);	//快速采集电流

void dvc10xx_SleepWakeup(void);			//休眠唤醒
uint8_t dvc10xx_cmd_sleep_check(void);	//休眠检查

uint8_t dvc10xx_CmdClearManage(void);	//清除指令并检查

uint8_t dvc10xx_MOS_QuikerOpen(void);	//快速关闭充放电MOS
uint8_t dvc10xx_MOS_QuikerClose(void);	//快速打开充放电MOS
uint8_t dvc10xx_DMOS_Close_CMOS_Open(void);
uint8_t dvc10xx_DMOS_Open_CMOS_Close(void);

//根据功能动态刷新配置数据
uint8_t dvc10xx_FuncFlushCfg(E_DVC_FUNC_FLAG funcFlag, uint8_t backNum, uint8_t status);

#endif

#endif