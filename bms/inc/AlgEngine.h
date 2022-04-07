#ifndef ALG_ENGINE
#define ALG_ENGINE

#include "includes.h"

#if (BAT_TYPE == 1)

    #if(BAT_TYPE2 == 2600)
    #define OCV_100  4180
    #define OCV_95   4100
    #define OCV_90   4040
    #define OCV_85   3985
    #define OCV_80   3935
    #define OCV_75   3887
    #define OCV_70   3842
    #define OCV_65   3800
    #define OCV_60   3763
    #define OCV_55   3729
    #define OCV_50   3692
    #define OCV_45   3659
    #define OCV_40   3639
    #define OCV_35   3623
    #define OCV_30   3610
    #define OCV_25   3597
    #define OCV_20   3582
    #define OCV_15   3561
    #define OCV_10   3520
    #define OCV_5    3468
    #define OCV_0    3436
    #elif (BAT_TYPE2 == 2000)
    #define OCV_100  4115
    #define OCV_95   4050
    #define OCV_90   4028
    #define OCV_85   3990
    #define OCV_80   3957
    #define OCV_75   3944
    #define OCV_70   3936
    #define OCV_65   3926
    #define OCV_60   3894
    #define OCV_55   3861
    #define OCV_50   3826
    #define OCV_45   3770
    #define OCV_40   3710
    #define OCV_35   3669
    #define OCV_30   3630
    #define OCV_25   3613
    #define OCV_20   3563
    #define OCV_15   3522
    #define OCV_10   3466
    #define OCV_5    3395
    #define OCV_0    3366
    #elif (BAT_TYPE2 == 2100)
    //2100p 
    #define OCV_100  4149
    #define OCV_95   4087
    #define OCV_90   4060
    #define OCV_85   4045
    #define OCV_80   4033
    #define OCV_75   4019
    #define OCV_70   3993
    #define OCV_65   3962
    #define OCV_60   3925
    #define OCV_55   3893
    #define OCV_50   3869
    #define OCV_45   3847
    #define OCV_40   3825
    #define OCV_35   3795
    #define OCV_30   3748
    #define OCV_25   3690
    #define OCV_20   3639
    #define OCV_15   3591
    #define OCV_10   3538
    #define OCV_5    3488
    #define OCV_0    3325
    #endif
#elif (BAT_TYPE == 2)

//IRF18650-2000
#define OCV_100  3525
#define OCV_95   3331
#define OCV_90   3329
#define OCV_85   3329
#define OCV_80   3328
#define OCV_75   3328
#define OCV_70   3327
#define OCV_65   3326
#define OCV_60   3325
#define OCV_55   3304
#define OCV_50   3290
#define OCV_45   3288
#define OCV_40   3287
#define OCV_35   3286
#define OCV_30   3285
#define OCV_25   3278
#define OCV_20   3256
#define OCV_15   3222
#define OCV_10   3203
#define OCV_5    3170
#define OCV_0    2822
#elif (BAT_TYPE == 4)

#define OCV_100  4180
#define OCV_95   4100
#define OCV_90   4040
#define OCV_85   3985
#define OCV_80   3935
#define OCV_75   3887
#define OCV_70   3842
#define OCV_65   3800
#define OCV_60   3763
#define OCV_55   3729
#define OCV_50   3692
#define OCV_45   3659
#define OCV_40   3639
#define OCV_35   3623
#define OCV_30   3610
#define OCV_25   3597
#define OCV_20   3582
#define OCV_15   3561
#define OCV_10   3520
#define OCV_5    3468
#define OCV_0    3436
#endif

#define OCV_VAL_NUM 21

typedef struct 
{
    uint16_t soc_r;       //soc实时
    uint16_t soc_display;       //显示soc
    uint16_t soh_r;       //soh实    
    uint16_t cycCount;    //cycle 循环次数
    uint16_t chgResTime;  //充电剩余时间估算
    uint16_t DsgResTime;
    float resCapAH_r;  //剩余容量实时
    float chgResTime2; //充电剩余时间2float数形式
    
}AlgEnginer_t;

typedef struct 
{
    float DsgCapacityAH;//已放电容量
    //一次充电周期内
    //累计充电容量
    float ChgCapacityAH;//已充电容量
    float DsgCapacity90AH;//放电90%实际容量
    float capacity;   //电池满电容量（FCC） AH  
    float preCapacityF10;  //预测前10%容量
    float preCapacityB10;  //预测后10%容量
    float ocvCapdiff;      //ocv校准容量差
    float ocvCapdiffdivbyM;//容量差/每秒
    //静止时间累加器
    uint32_t State0Count;//状态0计数
    uint8_t SocState; //电池当前工作状态
    //静止状态的上一个状态
    uint8_t State1Last ;      //0 - 放电状态  1-充电状态
    uint8_t RecodEEPromFlag; //额定容量写flash标志 bit 0，循环次数写flash标志 bit 1
    uint8_t flagState;//状态flagstate标志
    

}AlgEnginerIntnel_t;

typedef struct
{
    uint8_t SocState; //电池当前工作状态
    float capacity;   //电池满电容量 AH
} SocEngineRate_t;    //SOC 工作引擎系数

//extern SocEngineRate_t SocEngine;
extern AlgEnginer_t algEnginer;
extern AlgEnginerIntnel_t algIntnel;

void AlgEngineInit(void);
void AlgEngineProcess(void);
void AlgReConfigInitCapacity(uint8_t initc);
void SocEngineGetPrintInfo(void);
void Alg_BalanceHandle(void);
#endif
