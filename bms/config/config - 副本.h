#ifndef CONFIG_H
#define CONFIG_H

//定义中断向量地址偏移
#define __VTOR_PRESENT (1)
#define VECT_OFFSET  (0x1000)

#define FW_VERSION 0x01
#define HW_VERSION 0x01
//电芯代码 1-三元 2-磷酸 3锰酸
#define BAT_TYPE   0x01

//小牛定义的软件版本
#define NIU_FW_VER "F0D10V01"
//小牛定义的硬件版本
#define NIU_HW_VER "V1"

#define COMPLIE_TIME "2021-9-26"

#ifndef MCU_LIB_SELECT
#define MCU_LIB_SELECT    (1)  //处理器标准库选择 1、华大L170固件   2、国民技术 N32L406CBL7
#endif
#define ONEBUS_TYPE   (1)     //1小牛 2爱玛一线通  3雅迪一线通 4新日  5铁塔 6雅迪50bit字节协议 7钻豹一线通

#define UNIT_TEST_EN  (1)     //使能单元测试模块
#define CELL_NUMS     (13)
#define BAT_RATED_VOLT  (48)
#define LITHIUM_TYPE  (1)       //1-三元锂电  2-磷酸铁锂  3-锰酸锂
#define LITHIUM_CYCLE_E (1000)  //额定循环次数
#define LITHIUM_FULL_AH (16)
#define SOCSUMCALC_INTVAL (1000) //电量累计计算周期

#define DEFAULT_CHARG_DEMAND_VOLT   (5460) //充电需求电压 0.01V
#define DEFAULT_CHARG_DEMAND_CURR   (300)   //充电需求电流 0.01A

#define CHG_FULL_OCV_TH       (4100)     //满电OCV校准阈值

#define MAXCELL_FULL_SLEEP_TH (4100)     //静置态禁止高电压充电
#define SHIP_MODE_CELLV_TH    (2500)
#define CELL_VMAX_OVER_TH     (4300)
#define MOS_HIGH_WARN         (1000+2731)

#define VDIFF_WARN			  (300)
 
 //平衡开启电压
#define BALANCE_START_V       (3800) 
//平衡开启压差最小
#define BALANCE_START_S (50) 
//平衡开启压差最大
#define BALANCE_START_E (500)


//flash 参数配置地址
#define EE_START_ADDR 0x1FF00
#define BAT_SN_ADDR_START 0       //序列号地址 + 校准时间戳 0x1000
#define SH_EEPROM_ADDR (BAT_SN_ADDR_START+32+8) //SH309 EEPORM 映射 //40
#define SHG_STATE_ADDR (SH_EEPROM_ADDR+25)  //充电控制状态地址 40+25 = 65
#define SOC_ENGINEE_ADDR (SHG_STATE_ADDR+1)  //保留SOC 记录 2个字节 = 66
#define BATCYCLE_COUNT_ADDR (SOC_ENGINEE_ADDR+2) //68
#define SOC_INIT_CAPACITY_ADDR (BATCYCLE_COUNT_ADDR+2) //*70
#define CALI_RCH_TRIM_VAL_ADDR (SOC_INIT_CAPACITY_ADDR+2)//4个字节

//SH 默认参数配置表值
	// 0x5D, //0-prechg , enmos 1,ocpm 0,bal 1 by sh309 control,CN3-CN0 1101
	// 0x02, //0x23//E0VB -0,allow low volt chg,UV_OP -0 DIS_PF 1,CTRL -00,OCRA -1 EUVR - 0，//允许电流保护定时恢复，过放电保护状态释放与负载无关
	// 0x63, //OVT[3-0]=[0110]=1S OV9,OV8 =[1,1]
	// 0x52, //OV7-OV0 = [0x52]
	// 0xA3, //UVT[3-0]=[1010]=6S  OVR9,OVR8 = [1,1]
	// 0x34, //OVR7-OVR0 = [0x34]  //4100
	// 0x8c, //UV[7-0]=0x8c  2900/20 = 140 = 0x91
	// 0x96, //UVR[7-0]=0x91 3300/20 = 150 = 0xA5
	// 0xd2, //BAV[7-0]= 4200/20 = 200 = 0xD2,balance start voltage
	// 0x78, //PREV[7-0] = 2400/20 =120 = 0x78,precharge voltage
	// 0x4B, //L0V[7-0] = 1500/20 = 75 = 0x4B,low voltage forbid charge
	// 0xdc, //OVPF[7-0] = 4400/20 = 220 = 0xdc,second charge

	// // OCD1 Threshold = 50;         //Over Current Discharge1 Threshold.
	// // OCD1 Delay =2S;              //Over Current Discharge1 Delay Time.
	// // OCD2 Threshold = 80;         //Over Current Discharge2 Threshold.
	// // OCD2 Delay = 400MS;          //Over Current Discharge2 Delay Time.
	// 0x39, //OCD1  
	// 0x57, //OCD2

	// // SC Threshold = 230;          //Short Current Threshold.
	// // SC Delay = 256us;             //Short Current Delay Time.
	// 0x64, //SCV

	// // OCC Threshold = 20mv;        //Over Current Charge Threshold.if rsample = 1mR mean 20A
	// // OCC Delay = 20S;             //Over Current Charge Delay Time.20S
	// 0x1f,

	// //CHS Threshold = 200uV;        //State of charge and discharge detection voltage.
	// // MOSFET Delay = 128us;        //MOSFET TurnOn Delay Time.
	// // OCR Delay = 32S;             //Over Current Recovery Delay Time.
	// // OVPF Delay = 8S;             //Over Voltage Protect Failure Delay Time.
	// 0x16,

	// // OTC Threshold = 70;            //Over Temperature Charge Threshold.
	// 0x60,
	// // OTCR Threshold = 65;           //Over Temperature Charge Recovery Threshold.
	// 0x6c,
	// // UTC Threshold = -15;            //Under Temperature Charge Threshold.
	// 0xb4,
	// // UTCR Threshold = -10;            //Under Temperature Charge Recovery Threshold.
	// 0xa3,
	// // OTD Threshold = 65;            //Over Temperature Discharge Threshold.
	// 0x6c,
	// // OTDR Threshold = 60;           //Over Temperature Discharge Recovery Threshold.
	// 0x79,
	// // UTD Threshold = -10;           //Under Temperature Discharge Threshold.
	// 0xcf,
	// // UTDR Threshold = -5;           //Under Temperature Discharge Recovery Threshold.
	// 0xc2

#define SH_DEFAULT_EECONFIG_1  0x7D //1、启动充放电控制位，当充电过温或者过流保护后，当检测到放电，则开启充电MOS 2、充放电过流关闭充放电MOS，3、平衡开启MCU决定，
#define SH_DEFAULT_EECONFIG_2  0x12 //应许过流保护定时恢复，过流保护释放与负载无关
#define SH_DEFAULT_EECONFIG_3  0x73 //OVT = 2S ，OV = 4250
#define SH_DEFAULT_EECONFIG_4  0x52
#define SH_DEFAULT_EECONFIG_5  0xA3 //UVT = 6S
#define SH_DEFAULT_EECONFIG_6  0x34 //OVR = 4100
#define SH_DEFAULT_EECONFIG_7  0x91 //UV  = 2900
#define SH_DEFAULT_EECONFIG_8  0xA5 //UVR =  3300
#define SH_DEFAULT_EECONFIG_9  0xd2
#define SH_DEFAULT_EECONFIG_10 0x78
#define SH_DEFAULT_EECONFIG_11 0x4B //低压禁止充电1.5V
#define SH_DEFAULT_EECONFIG_12 0xdc
#define SH_DEFAULT_EECONFIG_13 0x39 //OCD1 50A，5S
#define SH_DEFAULT_EECONFIG_14 0x57 //OCD2 80A，400MS
#define SH_DEFAULT_EECONFIG_15 0x64 //SCD 230A，256us
#define SH_DEFAULT_EECONFIG_16 0x1f //充电过流20A，20S
#define SH_DEFAULT_EECONFIG_17 0x56 //充电过流恢复 CHS1 0 200uv 32S
#define SH_DEFAULT_EECONFIG_18 0x60
#define SH_DEFAULT_EECONFIG_19 0x6c
#define SH_DEFAULT_EECONFIG_20 0xb4
#define SH_DEFAULT_EECONFIG_21 0xa3
#define SH_DEFAULT_EECONFIG_22 0x6c
#define SH_DEFAULT_EECONFIG_23 0x79
#define SH_DEFAULT_EECONFIG_24 0xcf
#define SH_DEFAULT_EECONFIG_25 0xc2

//OCV 校准曲线值，小牛2100 25°
#define OCV_100  4167
#define OCV_95   4097
#define OCV_90   4065
#define OCV_85   4048
#define OCV_80   4037
#define OCV_75   4022
#define OCV_70   3994
#define OCV_65   3963
#define OCV_60   3933
#define OCV_55   3897
#define OCV_50   3869
#define OCV_45   3847
#define OCV_40   3823
#define OCV_35   3788
#define OCV_30   3734
#define OCV_25   3676
#define OCV_20   3624
#define OCV_15   3577
#define OCV_10   3523
#define OCV_5    3488
#define OCV_0    3327

//磷酸铁锂参数曲线 IRF18650-2000
// #define OCV_100  3525
// #define OCV_95   3331
// #define OCV_90   3329
// #define OCV_85   3329
// #define OCV_80   3328
// #define OCV_75   3328
// #define OCV_70   3327
// #define OCV_65   3326
// #define OCV_60   3325
// #define OCV_55   3304
// #define OCV_50   3290
// #define OCV_45   3288
// #define OCV_40   3287
// #define OCV_35   3286
// #define OCV_30   3285
// #define OCV_25   3278
// #define OCV_20   3256
// #define OCV_15   3222
// #define OCV_10   3203
// #define OCV_5    3170
// #define OCV_0    2822

#define OCV_VAL_NUM 21

/*
{
    "FW_VERSION":"01", //软件版本号
    "HW_VERSION":"01", //硬件版本号
    "BAT_TYPE":1,      //电芯代码 可选 1-三元 2-磷酸 3锰酸
    "NIU_FW_VER":"F0D10V01", //小牛定义的软件版本号
    "NIU_HW_VER":"V1",       //小牛定义的硬件版本号
    "COMPLIE_TIME":"2021-9-26",//编译的时间
    "MCU_SELECT":"HC32L170",       //MCU 选择，可以选择"HC32L170"-华大170 "N32L406CBL7"
    "ONEBUS_TYPE":1,           //一线通协议选择 1小牛 2爱玛一线通  3雅迪一线通 4新日  5铁塔
    "CELL_NUMS":13,            //电芯串数 
    "BAT_RATED_VOLT":48,       //电池标称电压 ，可以选择48，60，72

    "LITHIUM_CYCLE_E":1000,    //电芯额定循环次数
    "LITHIUM_FULL_AH":20,      //电池包额定容量
    "CHG_FULL_OCV_TH":4100,    //满电OCV阈值 4100mv
    "MAXCELL_FULL_SLEEP_TH":4100,
    "SHIP_MODE_CELLV_TH":2500, //仓运模式电芯低电压阈值
    "CELL_VMAX_OVER_TH":4300,  //电芯过电压三端自毁阈值
    "MOS_HIGH_WARN":100,      //MOS温度报警阈值 100度，实际要换算成2731+100*10
    "VDIFF_WARN":300,         //压差报警阈值300mv
    "BALANCE_START_V":3800,   //平衡开启电压
    "BALANCE_START_VDIFF_S":50, //平衡开启电压最小压差
    "BALANCE_START_VDIFF_E":500,//平衡开启电压最大压差
    "AFE_CONFIG":{              //前端配置
        "AFEIC":"SH309",        //前端芯片选择
		                        //前端芯片EEPROM 参数配置
        "EE":"0x7D,0x82,0x73,0x52,0xA3,0x34,0x91,0xA5,0xd2,0x78,0x4B,0xdc,0x39,0x57,0x64,0x1f,,0x16,0x60,0x6c,0xb4,0xa3,0x6c,0x79,0xcf,0xc2"
    },
    "OCV":{                    //OCV曲线配置
        "OCV_VAL_NUM":21,      //曲线点数
							   //OCV 电压值
        "OCVVAL":"4167,4097,4065,4048,4037,4022,3994,3963,3933,3897,3869,3847,3823,3788,3734,3676,3624,3577,3523,3488,3327"
    }
}*/


#endif