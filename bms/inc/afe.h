#ifndef __AFE_H
#define __AFE_H
#include "includes.h"

typedef struct
{
	//unit K, *10 ,AFE 温度
	uint16_t BalanceTemp;	//均衡温度
	//mos 管温度
	uint16_t MosTemp;
	//dscharger current unit mA
	uint32_t DsgCurrent;
	//charger current unit mA
	uint32_t ChgCurrent;
	//预防电电流
	uint16_t PreDsgCurrent;
	//every cell voltage mv
	uint16_t CellVolt[20];
	//pack volt mv
	uint32_t SumBatteryPackVolt;
	//ShTemp AFE前端温度
	uint16_t ShTemp[3];
	uint16_t CellVmax;
	uint16_t CellVmin;
	uint16_t CellVdiff;
	uint8_t  CellVmaxPos;  //0 开始
	uint8_t  CellVminPos;  //0 开始
	uint16_t CellTmax;
	uint16_t CellTmin;
	uint16_t CellTdiff;
	uint8_t  CellTmaxPos;  //0 开始
	uint8_t  CellTminPos;  //0 开始
	uint16_t AFE_State_RT;  //前端芯片故障
							//0x0000 空载
                            // 0x0001 放电
                            // 0x0002 充电
                            // 0x0004 过充保护警告
                            // 0x0008 过放保护警告
                            // 0x0010 充电过流保护警告
                            // 0x0020 放电过流保护警告
                            // 0x0040 电池过热保护警告
                            // 0x0080 电池过冷保护
                            // 0x0200 电池短路保护警告
							// 0x4000 AFE 前端采集数据故障


	uint16_t State_RT;		//电池实时状态,软件
							//0x0000 空载
                            // 0x0001 放电
                            // 0x0002 充电
                            // 0x0004 过充保护警告
                            // 0x0008 过放保护警告
                            // 0x0010 充电过流保护警告
                            // 0x0020 放电过流保护警告
                            // 0x0040 电池过热保护警告
                            // 0x0080 电池过冷保护
                            // 0x0100
                            // 电池其他异常警告（电池组开路/电池
                            // 组压差大于 0.3V）
                            // 0x0200 电池短路保护警告
                            // 0x0400 电池包进水
                            // 0x0800 BMS MOS 管损坏
							// 0x1000 BMS C MOS 损坏
							// 0x2000 BMS D MOS 损坏
							// 0x4000 AFE 前端采集数据故障
							// 0x8000 MOS 过温
							
	uint8_t MosState_RT;   
							// bit7： 预放电 MOS 闭合/继电器吸合（继电器版本）
							// bit6： 充电 MOS 闭合/继电器吸合（预留）
							// bit5： 放电 MOS 闭合
							// bit4： 系统接入状态
							// bit3： ACC 接入状态
							// bit2： 充电器接入状态
							// bit1:  通信被上拉
	uint8_t State_RT2;
							//bit 0 1-发生充电低温保护
							//bit 1 1-发生充电高温保护
							//bit 2 1-发生放电低温保护
							//bit 3 1-发生放电高温保护
							//bit 4 1-发生PF故障保护，断线保护
	uint8_t AFE_State_RT2;  //硬件温度保护状态标志
							//bit 0 1-发生充电低温保护
							//bit 1 1-发生充电高温保护
							//bit 2 1-发生放电低温保护
							//bit 3 1-发生放电高温保护
							//bit 4 1-发生PF故障保护，断线保护
	uint8_t Pre_State;
							// bit 0 0x0001 预放电锁定 
	uint32_t balState;     
							// bit23： 24号
							// bit22： 23号
							// ...： 
							// bit2： 3号
							// bit1： 2号
							// bit0： 1号电芯正在平衡
		
	
} AFEInfo_t;

extern AFEInfo_t afeInfo;



void AFE_Init(void);
void Afe_SetInitSample(void);
//peridic call this function
void AFE_Process(void);

void AFE_DisableVSample();
void AFE_EnableVSample();



#endif
