/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"

//SocEngineRate_t SocEngine;
AlgEnginer_t algEnginer;
AlgEnginerIntnel_t algIntnel;

uint16_t DispSOC = 0;
uint16_t CalcSOC = 0;          //计算SOC，x10
uint16_t OcvCalibratedSoc = 0; //经过ocv校准的soc值

#if (ONEBUS_TYPE == 3)//雅迪专用
float topAddAh = 0;
#endif

//计算总放电容量
float timeCC = 0;
float calcDsgCapacityAH = 0;
uint8_t AlgEnginePowerUp = 0;

//from 100% -0%  step 5%

const uint16_t OCV_Line[OCV_VAL_NUM] = {
    OCV_100, OCV_95, OCV_90, OCV_85, OCV_80, OCV_75, OCV_70, OCV_65, OCV_60, OCV_55,
    OCV_50, OCV_45, OCV_40, OCV_35, OCV_30, OCV_25, OCV_20, OCV_15, OCV_10, OCV_5, OCV_0};

static void SocTimerCallback(void);
static void SocOCVCalibrate(uint16_t avgCellVolt);
static void Alg_ReadKInfo(void);


/**
 * @brief 
 * 
 * 
 */
void AlgEngineInit(void)
{
    uint8_t tmpbuf[2];
    //读取EEPROM
    //1、初始化变量，某些变量需要从eeprom中加载读取
    Flash_Read(EE_START_ADDR+SOC_INIT_CAPACITY_ADDR,tmpbuf,2);

    if (*(uint16_t *)(tmpbuf) == 0xFFFF)
    {
        algIntnel.capacity = (float)LITHIUM_FULL_AH;
    }
    else
    {
        //algIntnel.capacity = (float)*(uint8_t *)(EE_START_ADDR + SOC_INIT_CAPACITY_ADDR); //加载flash容量数据
    }

    Flash_Read(EE_START_ADDR+BATCYCLE_COUNT_ADDR,tmpbuf,2);
    //2、循环次数，加载
    if (*(uint16_t *)(tmpbuf) == 0xFFFF)
    {
        algEnginer.cycCount = 0;
    }
    else
    {
        algEnginer.cycCount = *(uint16_t *)(tmpbuf);//*(uint16_t *)(EE_START_ADDR + BATCYCLE_COUNT_ADDR); //加载flash中循环次数
    }

    algIntnel.ocvCapdiff = 0;

    //2、定义一个自动定时器
    bsp_StartCallBackTimer(TMR_SOC_ENGINE, SocTimerCallback, SOCSUMCALC_INTVAL);
    //bsp_StartAutoTimer(TMR_SOC_ENGINE,SOCSUMCALC_INTVAL);
}

/**
 * @brief 在静止状态下，校准soc值
 * 
 * @param avgCellVolt 
 */
static void SocOCVCalibrate(uint16_t avgCellVolt)
{
    uint8_t i;

    if (avgCellVolt >= OCV_Line[0])
    {
        OcvCalibratedSoc = 1000;
        if(AlgEnginePowerUp == 0)
        {
            algEnginer.resCapAH_r = algIntnel.capacity;
        }else
        {
            algIntnel.ocvCapdiff = algEnginer.resCapAH_r - algIntnel.capacity ;
            //PRINT("OCV Calibrated,ocvCapdiff*1000 = %d\n",algIntnel.ocvCapdiff*1000);
        }
        
    }
    else if (avgCellVolt <= OCV_Line[20])
    {
        OcvCalibratedSoc = 0;
        if(AlgEnginePowerUp == 0)
        {
            algEnginer.resCapAH_r = 0;
        }else
        {
            //计算容量差，后备下次开始充放电时，容量渐变
            algEnginer.resCapAH_r = 0;
            algIntnel.ocvCapdiff = algEnginer.resCapAH_r;
            //PRINT("OCV Calibrated,ocvCapdiff*1000 = %d\n",algIntnel.ocvCapdiff*1000);
        }
        
    }
    else
    {
        for (i = 1; i < OCV_VAL_NUM; i++)
        {
            if (avgCellVolt > OCV_Line[i])
            {
                break;
            }
        }
        OcvCalibratedSoc = 1000 - 50 * i + 50 * (avgCellVolt - OCV_Line[i]) / (OCV_Line[i - 1] - OCV_Line[i]);
       
        if(AlgEnginePowerUp == 0)
        {
            algEnginer.resCapAH_r = (algIntnel.capacity * OcvCalibratedSoc / 1000);
        }else
        {
             //计算容量差，后备下次开始充放电时，容量渐变
            algIntnel.ocvCapdiff = algEnginer.resCapAH_r - (algIntnel.capacity * OcvCalibratedSoc / 1000);
        }
        
    }
}

//soh 估算
static void Alg_SOHCalc(void)
{
    //计算soh 循环次数 、压差
    algEnginer.soh_r = (uint16_t)(algIntnel.capacity / LITHIUM_FULL_AH) * 100;
    if(algEnginer.soh_r >100)
    {
        algEnginer.soh_r = 100;
    }
}

//存储引擎关键信息
void Alg_SaveKInfo(void)
{
    uint16_t saveFlag = 0xaaaa;
    MyMemcpy(CommonRam2,(uint8_t*)&saveFlag,2);
    MyMemcpy(CommonRam2+2,(uint8_t*)&algEnginer,sizeof(algEnginer));
    MyMemcpy(CommonRam2+2+sizeof(algEnginer),(uint8_t*)&algIntnel,sizeof(algIntnel));

    Flash_Write(EE_START_ADDR + ALG_PARAM_START,CommonRam2, 2+sizeof(algEnginer)+sizeof(algIntnel));
    PRINT("save alginfo\n");

}

void Alg_ClrKInfo(void)
{

  EEPROM_ERASE(EE_START_ADDR + ALG_PARAM_START, EEPROM_PAGE_SIZE);
//  EEPROM_ERASE(EE_START_ADDR + ALG_PARAM_START, EEPROM_PAGE_SIZE);
//  EEPROM_ERASE(EE_START_ADDR + ALG_PARAM_START, EEPROM_PAGE_SIZE);
//  EEPROM_ERASE(EE_START_ADDR + ALG_PARAM_START, EEPROM_PAGE_SIZE);
}

static void Alg_ReadKInfo(void)
{
    Flash_Read(EE_START_ADDR + ALG_PARAM_START+2,(uint8_t*)&algEnginer, sizeof(algEnginer));    
    Flash_Read(EE_START_ADDR + ALG_PARAM_START+2+ sizeof(algEnginer),(uint8_t*)&algIntnel, sizeof(algIntnel));

    PRINT("read algInfo\n");
    PRINT("soc_r-> %d\n",algEnginer.soc_r);
    PRINT("chgResTime -> %d\n",(uint16_t)(algEnginer.chgResTime));
    PRINT("cycCount -> %d\n",algEnginer.cycCount);
    PRINT("soh_r -> %d\n",algEnginer.soh_r);
    PRINT("resCapAH_r*1000 -> %d\n",(uint16_t)(algEnginer.resCapAH_r*1000));
    PRINT("ChgCapacity mAH -> %d\n",(uint16_t)(algIntnel.ChgCapacityAH*1000));
    PRINT("DsgCapacity mAH -> %d\n",(uint16_t)(algIntnel.DsgCapacityAH*1000));
    PRINT("RecodEEPromFlag -> %2x\n",algIntnel.RecodEEPromFlag);
    PRINT("algIntnel.State0Count-> %d\n",algIntnel.State0Count);
    PRINT("algIntnel.capacity -> %d\n",(uint16_t)(algIntnel.capacity*1000));
    PRINT("algIntnel.SocState -> %d\n",algIntnel.SocState);
    PRINT("algIntnel.preCapacityF10 -> %d\n",(uint16_t)(algIntnel.preCapacityF10*1000));
    PRINT("algIntnel.preCapacityB10 -> %d\n",(uint16_t)(algIntnel.preCapacityB10*1000));
    PRINT("algIntnel.calcCapacity90 -> %d\n",(uint16_t)(algIntnel.DsgCapacity90AH*1000));
    PRINT("algIntnel.flagState -> %d\n",algIntnel.flagState);
    if(algIntnel.ocvCapdiff>0)
    {
	PRINT("algIntnel.ocvCapdiff -> %d\n",algIntnel.ocvCapdiff*1000);
    }else
    {
	PRINT("algIntnel.ocvCapdiff -> -%d\n",0-algIntnel.ocvCapdiff*1000);
    }
}

/**
 * @brief 电池状态机推测，周期性调用
 * 
 */
void BatStateMachineLoop(void)
{
 
    //即没有充电电流也没有放电电流情况下
    if ((afeInfo.State_RT & 0x0003) == 0)
    {
        algIntnel.SocState = 0;

        //State0Count 必须是每秒累加1，要区分是休眠态和非休眠态,非休眠态不ocv校准
        algIntnel.State0Count = Rtc_GetCount();
        //休眠唤醒时间为10s
        if (afeInfo.MosState_RT & 0x1f)
        {
            // if (algIntnel.State0Count > 1800) //静止时间超过30分钟，由于是静置态，只能是休眠唤醒，休眠是10s唤醒
            // {
            //     algIntnel.SocState = 3; //状态迁移到ocv校准状态
            //     algIntnel.State0Count = 0;
            // }
        }
        else
        {
            PRINT("algIntnel.State0Count->%d\n",algIntnel.State0Count);
            if (algIntnel.State0Count > 180) //休眠是10s唤醒
            {
                algIntnel.SocState = 3; //状态迁移到ocv校准状态
                Rtc_SetCount(0);
                //algIntnel.State0Count = 0;
            }
        }
    }
    else if ((afeInfo.State_RT & 0x0001)== 1) //放电状态优先
    {
        //SocEngine.state = 1;//状态迁移到放电状态
        algIntnel.SocState = 1;
        Rtc_SetCount(0);
        //algIntnel.State0Count = 0;
    }
    else if ((afeInfo.State_RT &0x0002) == 2)
    {
        algIntnel.SocState = 2; //状态迁移到充电状态
        Rtc_SetCount(0);
        //algIntnel.State0Count = 0;
    }
}

/**
 * @brief 
 * 
 * 
 */
void AlgEngineProcess(void)
{
    uint8_t tmpbuf[2];
    uint16_t temp;
    float cald;
    static uint16_t ErecordCount = 0;

    if (AlgEnginePowerUp == 0)
    {
        if (afeInfo.CellVmin > 2600)
        {
            Flash_Read(EE_START_ADDR + ALG_PARAM_START, tmpbuf, 2);
//            //保存数据到flash
            if (*(uint16_t *)(tmpbuf) != 0xaaaa)
            {
                //上电初始化校准OCV一次
                SocOCVCalibrate(afeInfo.CellVmin);
            }else
            {
                Alg_ReadKInfo();
                //容量的再次比对判定,比额定容量超过30%，或者小于而定容量的70%
                if((algIntnel.capacity >(float)LITHIUM_FULL_AH*1.3) || algIntnel.capacity <(float)LITHIUM_FULL_AH*0.75)
                {
                    algIntnel.capacity = (float)LITHIUM_FULL_AH;
                }
            }

            AlgEnginePowerUp = 1;
        }
    }

    //1s 定时器计时到
    if (bsp_CheckTimer(TMR_SOC_ENGINE)|| (Rtc_GetWakeUpFlag() == 0x01))
    {
        BatStateMachineLoop();
        Alg_SOHCalc();
        //计算soc
        CalcSOC = (uint16_t)(algEnginer.resCapAH_r * 1000 / algIntnel.capacity);
        if(CalcSOC >=999)
        {
            CalcSOC = 1000;
            if(algEnginer.resCapAH_r > algIntnel.capacity)
            {
                algEnginer.resCapAH_r = algIntnel.capacity;
            }
        }
        algEnginer.soc_r = CalcSOC;
        if(algEnginer.soc_r > 995)
        {
            algEnginer.soc_display = 1000;
        }else
        {
            algEnginer.soc_display = CalcSOC;
        }

        //预放电小电流积分
        if(afeInfo.MosState_RT & 0x80)//
        {
            if (afeInfo.PreDsgCurrent >10)//大于10ma电流
            {
                //10s 钟积分
                temp = (float)(10)*afeInfo.PreDsgCurrent / 1000 / 3600;
                algIntnel.DsgCapacityAH += temp;
                algEnginer.resCapAH_r -= temp;
            }
        }

        //OV 保护后，soc 满电修正
        if(afeInfo.State_RT & 0x0004)
        {
            if(CalcSOC >= 990)
            {
                //满电校准
                algEnginer.resCapAH_r = algIntnel.capacity;
            }else
            {
                //快速累加 10倍快速累加
                //如果还没有达到99%，需要渐变到快速渐变到99%，然后跳到100%
                //float temp = 0.000027;//(float)(SOCSUMCALC_INTVAL)*10000.0 / 1000000 / 3600;
                //固定时间3分钟
                cald = (algIntnel.capacity - algEnginer.resCapAH_r) /180;
                algEnginer.resCapAH_r += cald;
                if(algEnginer.resCapAH_r > algIntnel.capacity)
                {
                    algEnginer.resCapAH_r = algIntnel.capacity;
                }
            }
            //algEnginer.resCapAH_r = algIntnel.capacity;
        }

        //安时积分
        //SocLoopCalc();
        if (algIntnel.SocState == 3)
        {
#if (BAT_TYPE == 1)
            //返回静置状态
            algIntnel.SocState = 0;
             //静置状态
            algIntnel.RecodEEPromFlag = 0x00; //清除

            //当已经是满电状态，不进一步校准
            if(algEnginer.soc_r > 995)
            {
                OcvCalibratedSoc = 1000;
            }else
            {   //调用ocv校准
                SocOCVCalibrate(afeInfo.CellVmin);
            }
            
            if (OcvCalibratedSoc >= 900)
            {
                //初始化剩余容量为满电状态
                //algEnginer.resCapAH_r = (1.02 * SocEngine.capacity); //加上0.02%容量,100->99%，的变化需要放0.6AH的容量
                algIntnel.preCapacityF10 = algIntnel.capacity * (1-(float)OcvCalibratedSoc/1000); 
                algIntnel.flagState = 1;                  
            }

            if (OcvCalibratedSoc <= 150)
            {
                if(algIntnel.flagState == 1)
                {
                    algIntnel.flagState = 2;
                    algIntnel.preCapacityB10 = algIntnel.capacity * (float)OcvCalibratedSoc/1000; 
                    //DsgCapacityAH 必须是全部采集到90% -10%全部放电容量才计算推测容量，中间浅充浅放不推测计算
                    calcDsgCapacityAH = algIntnel.DsgCapacity90AH + algIntnel.preCapacityF10 + algIntnel.preCapacityB10;
                    //在正负4AH范围内校准
                    if((calcDsgCapacityAH < algIntnel.capacity*1.2) && (calcDsgCapacityAH > algIntnel.capacity*0.75))
                    {
                        if ((algIntnel.RecodEEPromFlag & 0x01) == 0x00)
                        {
                            //循环次数限制在800次以内，按正常校准
                            //if(algEnginer.cycCount < LITHIUM_LOW_POWER_E)
                            //{
                                //PRINT("Info->SocEngine.capacity is updated,algIntnel.capacity x1000->%d\n",(uint16_t)(calcDsgCapacityAH*1000));
                                algIntnel.capacity = calcDsgCapacityAH;
                                Flash_Write(EE_START_ADDR + SOC_INIT_CAPACITY_ADDR, (uint8_t *)&algIntnel.capacity, 2);
                                algIntnel.RecodEEPromFlag |= 0x01; //充电周期循环次数已经记录
                            //}else
                            //超出使用次数，soc降容使用
                            //{
                            //    algIntnel.capacity = calcDsgCapacityAH - 0.8*LITHIUM_FULL_AH*(1-(float)algEnginer.cycCount/LITHIUM_CYCLE_E);
                            //    Flash_Write(EE_START_ADDR + SOC_INIT_CAPACITY_ADDR, (uint8_t *)&algIntnel.capacity, 2);
                            //    algIntnel.RecodEEPromFlag |= 0x01; //充电周期循环次数已经记录
                            //}
                            
                        }
                    }          
                }       
                //algEnginer.chgResTime = 0xFFFF;
            }
#elif(BAT_TYPE == 2)//IRF
            algIntnel.SocState = 0;
            //已经处于100%SOC时不进行校准
            if (CalcSOC != 1000)
            {
                if(afeInfo.CellVmin > OCV_Line[1] )//95%以上才校准 5%以下
                {
                    SocOCVCalibrate(afeInfo.CellVmin);
                }else
                if(afeInfo.CellVmax < OCV_Line[19])
                {
                    SocOCVCalibrate(afeInfo.CellVmax);
                }
                //充电之后的状态
                if (OcvCalibratedSoc >= 950)
                {
                   
                    algIntnel.preCapacityF10 = algIntnel.capacity * (1-(float)OcvCalibratedSoc/1000); 
                    algIntnel.flagState = 1;                
                }

                //容量学习要点：
                //1、当累计放出的电量大于额定容量时，取累计放电容量为额定容量
                //2、当累计放出的电量还小于额定容量时，但是ocv校准之后soc已为0，或者已经被欠压保护了，额定容量取当时的累计放出电量
                //全部放完电,容量学习
                if (OcvCalibratedSoc <= 50)
                {
                    if(algIntnel.flagState == 1)
                    {
                        algIntnel.flagState = 2;
                        algIntnel.preCapacityB10 = algIntnel.capacity * (float)OcvCalibratedSoc/1000; 
                        //DsgCapacityAH 必须是全部采集到90% -10%全部放电容量才计算推测容量，中间浅充浅放不推测计算
                        calcDsgCapacityAH = algIntnel.DsgCapacity90AH + algIntnel.preCapacityF10 + algIntnel.preCapacityB10;
                        //在正负4AH范围内校准
                        if((calcDsgCapacityAH < algIntnel.capacity*1.2) && (calcDsgCapacityAH > algIntnel.capacity*0.75))
                        {
                            if ((algIntnel.RecodEEPromFlag & 0x01) == 0x00)
                            {
                                //循环次数限制在800次以内，按正常校准
                                //if(algEnginer.cycCount < LITHIUM_LOW_POWER_E)
                                //{
                                    PRINT("Info->SocEngine.capacity is updated,algIntnel.capacity x1000->%d\n",(uint16_t)(calcDsgCapacityAH*1000));
                                    algIntnel.capacity = calcDsgCapacityAH;
                                    Flash_Write(EE_START_ADDR + SOC_INIT_CAPACITY_ADDR, (uint8_t *)&algIntnel.capacity, 2);
                                    algIntnel.RecodEEPromFlag |= 0x01; //充电周期循环次数已经记录
                                //}else
                                //超出使用次数，soc降容使用
                                //{
                                //    algIntnel.capacity = calcDsgCapacityAH - 0.8*LITHIUM_FULL_AH*(1-(float)algEnginer.cycCount/LITHIUM_CYCLE_E);
                                //    Flash_Write(EE_START_ADDR + SOC_INIT_CAPACITY_ADDR, (uint8_t *)&algIntnel.capacity, 2);
                                //    algIntnel.RecodEEPromFlag |= 0x01; //充电周期循环次数已经记录
                                //}
                                
                            }
                        }          
                    }
                }
                //algEnginer.chgResTime = 0xFFFF;
            }
#endif      
        }
        else if (algIntnel.SocState == 1) //放电状态
        {
#if (ONEBUS_TYPE == 3)//雅迪专用
            //100%->99%,多放2%容量逻辑
            if(CalcSOC == 1000)
            {
                topAddAh = 0;
            }
            else
            if(CalcSOC >= 990)
            {
                float temp = (float)(SOCSUMCALC_INTVAL)*0.5*afeInfo.DsgCurrent / 1000000 / 3600;
                topAddAh += temp;
                if(topAddAh < 0.02* SocEngine.capacity)
                {
                    algEnginer.resCapAH_r += temp;
                    
                }else
                {
                    topAddAh = 0.02 * SocEngine.capacity;
                }                
            }else
            if(CalcSOC >= 980)//99->98 容量渐变减掉0.005
            {
                float temp = (float)(SOCSUMCALC_INTVAL)*0.5*afeInfo.DsgCurrent / 1000000 / 3600;
                topAddAh -= temp;

                if(topAddAh > 0.015* SocEngine.capacity)
                {
                    algEnginer.resCapAH_r -= temp;
                    
                }else
                {
                    topAddAh = 0.015 * SocEngine.capacity;
                }
            }else
            if(CalcSOC >= 970)
            {
                float temp = (float)(SOCSUMCALC_INTVAL)*0.5*afeInfo.DsgCurrent / 1000000 / 3600;
                topAddAh -= temp;

                if(topAddAh > 0.01* SocEngine.capacity)
                {
                    algEnginer.resCapAH_r -= temp;
                    
                }else
                {
                    topAddAh = 0.01 * SocEngine.capacity;
                }
            }else
            if(CalcSOC >= 960)
            {
                float temp = (float)(SOCSUMCALC_INTVAL)*0.5*afeInfo.DsgCurrent / 1000000 / 3600;
                topAddAh -= temp;

                if(topAddAh > 0.005* SocEngine.capacity)
                {
                    algEnginer.resCapAH_r -= temp;
                    
                }else
                {
                    topAddAh = 0.005 * SocEngine.capacity;
                }
            }else
            if(CalcSOC >= 950)
            {
                float temp = (float)(SOCSUMCALC_INTVAL)*0.5*afeInfo.DsgCurrent / 1000000 / 3600;
                topAddAh -= temp;

                if(topAddAh > 0.001* SocEngine.capacity)
                {
                    algEnginer.resCapAH_r -= temp;
                    
                }else
                {
                    topAddAh = 0;
                }
            }
#endif
            //每隔3分钟信息保持
            ErecordCount++;

            if(ErecordCount >= 180)
            {
                ErecordCount = 0;
                Alg_SaveKInfo();
            }

            //当前放电剩余时间
            algEnginer.DsgResTime = (uint16_t)(60*algEnginer.resCapAH_r/((float)afeInfo.DsgCurrent/1000)); //
                    
        }
        else if (algIntnel.SocState == 2) //充电状态
        {
            if (afeInfo.SumBatteryPackVolt > CHG_FULL_OCV_TH * CELL_NUMS) //满充电修正100%策略
            {
                if (afeInfo.ChgCurrent <= 0.05 * algIntnel.capacity*1000)
                {
                    if(CalcSOC >= 990)
                    {
                        //满电校准
                        algEnginer.resCapAH_r = algIntnel.capacity;
                        Alg_SaveKInfo();
                    }else
                    {
                        //快速累加 10倍快速累加
                        //如果还没有达到99%，需要渐变到快速渐变到99%，然后跳到100%
                        float temp = (float)(SOCSUMCALC_INTVAL)*10*afeInfo.ChgCurrent / 1000000 / 3600;
                        algEnginer.resCapAH_r += temp;
                    }
                     
                }else
                if((CalcSOC >= 990) && (CalcSOC <= 999))
                {
                    //已经是99%，等待真正充满
                    algEnginer.resCapAH_r = algIntnel.capacity*0.99;
                }

            }
            //累计循环次数判定,以累计充电容量大于90%额定容量
            if (algIntnel.ChgCapacityAH >= 0.9 * algIntnel.capacity)
            {
                if ((algIntnel.RecodEEPromFlag & 0x02) == 0)
                {
                    algEnginer.cycCount++;
                    //PRINT("Info->SocEngine.cycCount is updated,SocEngine.cycCount ->%d\n",algEnginer.cycCount);
                    Flash_Write(EE_START_ADDR + BATCYCLE_COUNT_ADDR, (uint8_t *)&algEnginer.cycCount, 2);
                    algIntnel.RecodEEPromFlag |= 0x02; //充电周期循环次数已经记录
                    algIntnel.ChgCapacityAH = 0;
                }
            }

            //充电剩余时间估算,大于1A计算
            if (afeInfo.ChgCurrent > 1000)
            {
                //当前剩余容量，当前充电电流，满电容量
                //1\计算到恒压阶段的时间，90% +恒压0.5小时
                if(algEnginer.resCapAH_r < 0.9*algIntnel.capacity)
                {
                    algEnginer.chgResTime = (uint16_t)(10 * 0.9*(algIntnel.capacity - algEnginer.resCapAH_r) / ((float)afeInfo.ChgCurrent / 1000))+5; //计算到0.1H ,加恒压半个小时
                    algEnginer.chgResTime2 = (10 * 0.9*(algIntnel.capacity - algEnginer.resCapAH_r) / ((float)afeInfo.ChgCurrent / 1000))+5.0;
                    timeCC = algEnginer.chgResTime2;
                }else
                {
                    if(timeCC == 0)
                    {
                        timeCC = 5;
                    }
                    algEnginer.chgResTime = (uint16_t)(timeCC*10*(1.0-algEnginer.resCapAH_r/algIntnel.capacity));//随容量百分比均分
                    algEnginer.chgResTime2 = timeCC*10*(1.0-algEnginer.resCapAH_r/algIntnel.capacity);
                }
                
            }

            ErecordCount++;
            //充电记录时间10分钟
            if(ErecordCount >= 600)
            {
                ErecordCount = 0;
                Alg_SaveKInfo();
            }

            algIntnel.DsgCapacityAH = 0;
            algIntnel.flagState = 0;
            algIntnel.DsgCapacity90AH = 0;
        }
    }
}

//智能平衡控制算法
void Alg_BalanceHandle(void)
{

}

void SocReConfigInitCapacity(uint8_t initc)
{
    algIntnel.capacity = initc;
}

float SocGetInitCapacity(void)
{
    return algIntnel.capacity;
}

uint16_t SocGetSocValue(void)
{
    if (CalcSOC >= 1000)
    {
        DispSOC = 1000;
    }
    else
    {
        DispSOC = CalcSOC;
    }
    return DispSOC;
}

void SocEngineGetPrintInfo(void)
{
    /*algIntnel.ChgCapacityAH = ChgCapacityAH;
    algIntnel.DsgCapacityAH = DsgCapacityAH;
    algIntnel.DsgCapacity90AH = calcCapacity90;
    algIntnel.RecodEEPromFlag = RecodEEPromFlag;
    algIntnel.State0Count = State0Count;
    algIntnel.State1Last = State1Last;
    algIntnel.flagState  = flagCalcState;*/
}

static void SocTimerCallback(void)
{
    float temp;
    float ddf;


    //1、电量积分计算
    if (algIntnel.SocState == 1)
    {
        //放电容量积分
        temp = (float)(SOCSUMCALC_INTVAL)*afeInfo.DsgCurrent / 1000000 / 3600;
        
        algIntnel.DsgCapacityAH += temp;
        algEnginer.resCapAH_r -= temp;

        //剩余容量差 = 计算剩余容量 - 校准后的剩余容量
        //在放电模式，容量差大于1%额定容量
        //算法均应分摊到放电到soc为0的周期
        if(algIntnel.ocvCapdiff>0.01*algIntnel.capacity)
        {
            algIntnel.ocvCapdiffdivbyM = 1.4*algIntnel.ocvCapdiff/(60*algEnginer.DsgResTime);
            algIntnel.ocvCapdiff -= algIntnel.ocvCapdiffdivbyM;
            algEnginer.resCapAH_r -= algIntnel.ocvCapdiffdivbyM;
        }else
        if(algIntnel.ocvCapdiff<-0.01*algIntnel.capacity)
        {
            ddf = 0.3*temp+algIntnel.ocvCapdiff/(60*algEnginer.DsgResTime);
            if(ddf>0)
            {
                algIntnel.ocvCapdiffdivbyM = 1.4*algIntnel.ocvCapdiff/(60*algEnginer.DsgResTime);
                algIntnel.ocvCapdiff += algIntnel.ocvCapdiffdivbyM;
                algEnginer.resCapAH_r -= algIntnel.ocvCapdiffdivbyM ;
            }
            
        }

        if(algEnginer.resCapAH_r<0)
        {
            algEnginer.resCapAH_r = 0;
        }
        if(algIntnel.flagState ==1)
        {
            algIntnel.DsgCapacity90AH += temp;
        }
    }
    else if (algIntnel.SocState == 2)
    {
        temp = (float)(SOCSUMCALC_INTVAL)*afeInfo.ChgCurrent / 1000000 / 3600;

        if((algEnginer.resCapAH_r + temp) < 0.999*algIntnel.capacity)
        {
            algEnginer.resCapAH_r += temp;
            //算法均应分摊到充电电到soc为100的周期
            if(algIntnel.ocvCapdiff>0.01*algIntnel.capacity)
            {
                ddf = 0.3*temp+algIntnel.ocvCapdiff/(6.0*algEnginer.chgResTime2);
                if(ddf>0)
                {
                    algIntnel.ocvCapdiff += algIntnel.ocvCapdiff/(6.0*algEnginer.chgResTime2);
                    algEnginer.resCapAH_r -= algIntnel.ocvCapdiff/(6.0*algEnginer.chgResTime2);
                }                
            }else
            if(algIntnel.ocvCapdiff<-0.01*algIntnel.capacity)
            {
                algIntnel.ocvCapdiffdivbyM = 1.4*algIntnel.ocvCapdiff/(6.0*algEnginer.chgResTime2);
                algIntnel.ocvCapdiff -= algIntnel.ocvCapdiffdivbyM;
                algEnginer.resCapAH_r -= algIntnel.ocvCapdiff;
            }
        }
        algIntnel.ChgCapacityAH += temp;

    }
}
