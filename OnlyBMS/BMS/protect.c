#include "includes.h"

uint16_t DelayCountOV = 0; //过压计数
uint16_t DelayCountUV = 0; //欠压计数
uint16_t VdiffFlag = 0;
uint16_t DelayCountOCC = 0;//充电过流计数
uint16_t DelayCountOCD = 0;//放电过流计数
uint16_t DelayCountOTC = 0;//充电高温计数
uint16_t DelayCountOTD = 0;//放电高温计数
uint16_t DelayCountUTC = 0;//低温充电计数
uint16_t DelayCountUTD = 0;//低温放电计数
uint16_t DelayCountMOSOT = 0;//MOS 过温保护
uint16_t DelayCountSoc100 = 0;//100%满电关断
uint16_t DelayCountVdiff = 0;//压差大延迟计数

uint16_t timestampe = 0;

static void ProtectOV()
{
    if(afeInfo.CellVmax >= niuCommdTable.OC_Vlt_P*100)
    {
        if(afeInfo.MosState_RT & 0x40)
        {
            DelayCountOV++;
            if(DelayCountOV>(OC_VLT_DELAY_S/1000))//延迟2s
            {
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
                afeInfo.State_RT |= 0x0004;   //过压保护告警
                DelayCountOV = 0;
                PRINT("[%d],Event->ov protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }

    }else
    if(afeInfo.CellVmax <= niuCommdTable.ROC_Vlt*100)//过压恢复
    {
        if(afeInfo.State_RT & 0x0004)
        {
#if(AFE_CHIP_SELECT == 3)
            Sh_ClrFlag();
#endif
            AFE_OPEN_CMOS();//关充电MOS
            afeInfo.MosState_RT |= 0x40; //设置充电MOS关闭
            afeInfo.State_RT &= ~0x0004;
            DelayCountOV = 0;
            PRINT("[%d],Event->ov recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectUV()
{
    if(afeInfo.CellVmin <= niuCommdTable.Dc_Vlt_P*100)//一级温度保护
    {
        if(afeInfo.MosState_RT & 0x20)
        {
            if(afeInfo.CellTmin >= 2730)//温度判断
            {
                DelayCountUV++;
                if(DelayCountUV>(DC_VLT_DELAY_S/1000))//延迟4s
                {
                    SWITCH_PRED_OFF(); //预放电关
                    afeInfo.MosState_RT &= ~0x80; //预放电电MOS关闭
                    AFE_CLOSE_DMOS();//关充电MOS
                    afeInfo.MosState_RT &= ~0x20; //放电电MOS关闭
                    afeInfo.State_RT |= 0x0008;   //过放电标志
                    
                    DelayCountUV = 0;
                    PRINT("[%d],Event->uv protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
                }
            }else //<0°
            {
                if(afeInfo.CellVmin  <= (niuCommdTable.Dc_Vlt_P*100))//二级阈值点保护3000-400 = 2600mv
                {
                    DelayCountUV++;
                    if(DelayCountUV>(DC_VLT_DELAY_S/1000))//延迟4s
                    {
                        SWITCH_PRED_OFF(); //预放电关
                        afeInfo.MosState_RT &= ~0x80; //预放电电MOS关闭
                        AFE_CLOSE_DMOS();//关充电MOS
                        afeInfo.MosState_RT &= ~0x20; //设置充电MOS关闭
                        afeInfo.State_RT |= 0x0008;   //过压保护告警
                        DelayCountUV = 0;
                        PRINT("[%d],Event->uv protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
                    }
                }
            }
        }		
        
    }else
    if(afeInfo.CellVmin >= niuCommdTable.RDc_Vlt_P*100)//欠压压恢复
    {
        if(afeInfo.State_RT & 0x0008)
        {
            AFE_OPEN_DMOS();//关充电MOS
            afeInfo.MosState_RT |= 0x20; //设置充电MOS关闭
            afeInfo.State_RT &= ~0x0008;   //过压保护告警
            DelayCountUV = 0;
            PRINT("[%d],Event->uv recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }

    }
}

static void ProtectVdiff()
{
    if(afeInfo.CellVdiff >= 500)//压差保护阈值
    {
        DelayCountVdiff++;
        if(DelayCountVdiff>10)
        {
            if(afeInfo.State_RT & 0x0002)//充电保护
            {
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
                VdiffFlag = 1;
            }else
            if(afeInfo.State_RT & 0x0001)//放电保护
            {
                SWITCH_PRED_OFF(); //预放电关
                afeInfo.MosState_RT &= ~0x80; //预放电电MOS关闭
                AFE_CLOSE_DMOS();//关放电MOS
                afeInfo.MosState_RT &= ~0x20; //设置充电MOS关闭
                VdiffFlag = 2;
            }

            PRINT("[%d],Event->vdiff protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            afeInfo.State_RT |= 0x0100;  //压差大保护
            DelayCountVdiff = 0;
        }

    }else
    if(afeInfo.CellVdiff < 300) //压差保护恢复
    {
        if(afeInfo.State_RT &0x0100)
        {
            afeInfo.State_RT &= ~0x0100;
            if(VdiffFlag == 1)
            {
                AFE_OPEN_CMOS();
                afeInfo.MosState_RT |= 0x40; 	
            }else
            if(VdiffFlag == 2)
            {
                AFE_OPEN_DMOS();//开充电MOS
                afeInfo.MosState_RT |= 0x20; 
            }

            PRINT("[%d],Event->vdiff recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
        DelayCountVdiff = 0;
    }
}

static void ProtectOTC()
{
    if(afeInfo.CellTmax > ((int8_t)niuCommdTable.COTp_P*10+2731))//55°
    {
        DelayCountOTC++;
        if(DelayCountOTC >(COT_DELAY_S/1000))//延时4S
        {
            if(afeInfo.State_RT & 0x2)//充电状态
            {
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
                afeInfo.State_RT |= 0x0040;   //过温保护
                afeInfo.State_RT2 |= 0x02;    //充电高温保护
                DelayCountOTC = 0;

                PRINT("[%d],Event->otc protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }

        }
    }else
    if(afeInfo.CellTmax < ((int8_t)niuCommdTable.RCoTp_P*10+ 2731)) //恢复
    {
        if(afeInfo.State_RT2 & 0x02) //充电高温保护
        {
            AFE_OPEN_CMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x40;   //设置充电MOS开启
            afeInfo.State_RT &= ~0x0040;
            afeInfo.State_RT2 &= ~0x02;    //充电高温保护
            DelayCountOTC = 0;
            PRINT("[%d],Event->otc recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectOTD()
{
    if(afeInfo.CellTmax > ((int8_t)niuCommdTable.DcOTp_P*10+2731))//70°
    {
        DelayCountOTD++;
        if(DelayCountOTD >(DCOT_DELAY_S/1000))//延时4S
        {
            if(afeInfo.State_RT & 0x0001)//放电电状态
            {
                AFE_CLOSE_DMOS();//关放电MOS
                afeInfo.MosState_RT &= ~0x20; //设置放电MOS关闭
                afeInfo.State_RT |= 0x0040;   //过温保护
                afeInfo.State_RT2 |= 0x08;    //放电保护
                DelayCountOTD = 0;
                PRINT("[%d],Event->otd protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }

        }
    }else
    if(afeInfo.CellTmax < ((int8_t)niuCommdTable.RDcOTp_P*10+2731)) //放电高温恢复
    {
        if(afeInfo.State_RT2 & 0x08)//过温保护
        {
            AFE_OPEN_DMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x20; //设置充电MOS开启
            afeInfo.State_RT &= ~0x0040;
            afeInfo.State_RT2 &= ~0x08;    //清除放电高温保护
            DelayCountOTD = 0;
            PRINT("[%d],Event->otd recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectUTC()
{
    if(afeInfo.CellTmin < CCLTP_P*10+2731)//0°
    {
        if(afeInfo.State_RT & 0x0002)//充电状态
        {
            DelayCountUTC++;
            if(DelayCountUTC >(CCLT_DELAY_S/1000))//延时4S
            {
                
                    AFE_CLOSE_CMOS();//关充电MOS
                    afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
                    afeInfo.State_RT |= 0x0080;   //低温保护
                    afeInfo.State_RT2 |= 0x01;    //充电低温保护
                    DelayCountUTC = 0;
                    PRINT("[%d],Event->utc protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }
    }else
    if(afeInfo.CellTmin > (RCCLTP_P*10+2731)) //恢复4摄氏度
    {
        if(afeInfo.State_RT2 & 0x01)
        {
            AFE_OPEN_CMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x40; //设置充电MOS开启
            afeInfo.State_RT &= ~0x0080;
            afeInfo.State_RT2 &= ~0x01;    //充电低温保护
            DelayCountUTC = 0;
            PRINT("[%d],Event->utc recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectUTD()
{
    if(afeInfo.CellTmin < ((int8_t)niuCommdTable.DcLTp_P*10)+2731)//零下20度
    {
        if(afeInfo.State_RT & 0x0001)//放电电状态
        {
            DelayCountUTD++;
            if(DelayCountUTD >(DCLT_DELAY_S/1000))//延时4S
            {
                
                    AFE_CLOSE_DMOS();//关充电MOS
                    afeInfo.MosState_RT &= ~0x20; //设置充电MOS关闭
                    afeInfo.State_RT |= 0x0080;   //低温保护
                    afeInfo.State_RT2 |= 0x04;    //低温放电
                    DelayCountUTD = 0;
                    PRINT("[%d],Event->utd protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }
    }else
    if(afeInfo.CellTmin > ((int8_t)niuCommdTable.RDcLTp_P*10+2731)) //恢复零下15度
    {
        if(afeInfo.State_RT2 & 0x04)
        {
            AFE_OPEN_CMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x40; //设置充电MOS开启
            afeInfo.State_RT &= ~0x0080;
            afeInfo.State_RT2 &= ~0x04;    //充电低温保护
            DelayCountUTD = 0;
            PRINT("[%d],Event->otd recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectOCD()
{
    if(afeInfo.DsgCurrent > niuCommdTable.Dc_Cur_P*1000)//
    {
        if(afeInfo.State_RT & 0x0001)//放电电状态
        {
            DelayCountOCD++;
            if(DelayCountOCD >(DC_DELAY_S/1000))//延时5S
            {
                
                    AFE_CLOSE_DMOS();//关充电MOS
                    afeInfo.MosState_RT &= ~0x20; //设置放电MOS关闭
                    afeInfo.State_RT |= 0x0020;   //放电过流标志
                    DelayCountOCD = 0;
                    PRINT("[%d],Event->ocd protect,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }
    }else
    if(afeInfo.State_RT & 0x0020) 
    {
        DelayCountOCD++;
        if(DelayCountOCD > (RDC_DELAY_S/1000))//延迟30s 恢复
        {
            AFE_OPEN_DMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x20; //设置放电MOS开启
            afeInfo.State_RT &= ~0x0020;
            DelayCountOCD = 0;
            PRINT("[%d],Event->ocd recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectOCC()
{
    if(afeInfo.ChgCurrent > niuCommdTable.C_Cur_P*1000)
    {
        if(afeInfo.State_RT & 0x0002)//充电电状态
        {
            DelayCountOCC++;
            if(DelayCountOCC >(CC_DELAY_S/1000))//延时15S
            {
                
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置放电MOS关闭
                afeInfo.State_RT |= 0x0010;   //放电过流标志
                DelayCountOCC = 0;
                PRINT("[%d],Event->occ protect,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }

        }
    }else
    if(afeInfo.State_RT & 0x0010) 
    {
        DelayCountOCC++;
        if(DelayCountOCC > (RCC_DELAY_S/1000))//延迟30s
        {
            AFE_OPEN_CMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x40; //设置放电MOS开启
            afeInfo.State_RT &= ~0x0010;
            DelayCountOCC = 0;
            PRINT("[%d],Event->occ recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
}

static void ProtectMOSOT()
{
    if(afeInfo.MosTemp > (MOSOTP_P*10+2731))
    {
        DelayCountMOSOT++;
        if(DelayCountMOSOT >(MOSOT_DELAY_S/1000))//延时15S
        {
            if(afeInfo.State_RT & 0x0002)//充电电状态
            {
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置放电MOS关闭
                afeInfo.State_RT |= 0x8000;   //放电过流标志
            }else
            if(afeInfo.State_RT & 0x0001)//放电状态
            {
                AFE_CLOSE_DMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x20; //设置放电MOS关闭
                afeInfo.State_RT |= 0x8000;   //放电过流标志
            }
            DelayCountMOSOT = 0;
            PRINT("[%d],Event->mos ot protect,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
        }
    }
    else
    if((afeInfo.State_RT & 0x8000) && (afeInfo.MosTemp < RMOSOTP_P*10+2731)) 
    {
        if((afeInfo.State_RT & 0x20) == 0)
        {
            AFE_OPEN_DMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x20; //设置放电MOS关闭
        }else
        if((afeInfo.State_RT & 0x40) == 0)
        {
            AFE_OPEN_CMOS();//开充电MOS
            afeInfo.MosState_RT |= 0x40; //设置放电MOS关闭
        }
        DelayCountMOSOT = 0;
        afeInfo.State_RT &= ~0x8000;
        PRINT("[%d],Event->mos recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
    }
}

static void ProtectSocFull()
{
    if(afeInfo.State_RT & 0x0002)
    {
        //soc 已经满电情况
        if((algEnginer.soc_r == 1000) && (afeInfo.MosState_RT & 0x40))
        {
            //加延迟判断
            DelayCountSoc100++;
            if(DelayCountSoc100 >10)
            {
                DelayCountSoc100 = 0;
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //关充电MOS
                PRINT("[%d],Event->soc 100% protect,afeInfo.MosState_RT,State_RT->%2x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }				
        }else
        {
            DelayCountSoc100 = 0;
        }
    }else
    if(afeInfo.State_RT & 0x0001)//放电
    {
        AFE_OPEN_CMOS();//强制打开充电MOS
        afeInfo.MosState_RT |= 0x40;
    }
}

//超出使用期限，降容使用
static void ProtectOutofDate()
{
    uint16_t cycle = 0;

    if(algEnginer.cycCount > LITHIUM_LOW_POWER_E)
    {
        //最小电压已经超过保护电压
        cycle = algEnginer.cycCount - LITHIUM_LOW_POWER_E;
        if(afeInfo.CellVmax >= niuCommdTable.OC_Vlt_P*100 - cycle*2)//超过循环次数后，每个循环周期降低3mv过压保护
        {
            DelayCountOV++;
            if(DelayCountOV>(OC_VLT_DELAY_S/1000))//延迟2s
            {
                AFE_CLOSE_CMOS();//关充电MOS
                afeInfo.MosState_RT &= ~0x40; //设置充电MOS关闭
                afeInfo.State_RT |= 0x0004;   //过压保护告警
                DelayCountOV = 0;
                PRINT("[%d],Event->out date ov protected,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }else
        if(afeInfo.CellVmax <= niuCommdTable.ROC_Vlt*100 - cycle*3)//过压恢复
        {
            if(afeInfo.State_RT & 0x0004)
            {
    #if(AFE_CHIP_SELECT == 3)
                Sh_ClrFlag();
    #endif
                AFE_OPEN_CMOS();//开充电MOS
                afeInfo.MosState_RT |= 0x40; //设置充电MOS打开
                afeInfo.State_RT &= ~0x0004;
                DelayCountOV = 0;
                PRINT("[%d],Event->out date ov recover,afeInfo.MosState_RT,State_RT->%x,%4x\n",timestampe,afeInfo.MosState_RT,afeInfo.State_RT);
            }
        }
    }
}

static void ProtectSCR()
{
    if(afeInfo.AFE_State_RT & 0x200)
    {
        //短路保护
        if(GET_LOAD_DET() == 1)
        {
            #if(AFE_CHIP_SELECT == 3)
            Sh_ClrFlag();
            #endif
        }
    }
}

void NiuLogic_Protect(void)
{

	if(bsp_CheckTimer(TMR_PROTECT_DELAY))//保护计时周期100ms
	{
		timestampe = bsp_GetRunTime();
#if (EN_OUTDATA_PROTECT == 1)
        if(algEnginer.cycCount >= LITHIUM_LOW_POWER_E)
        {
            ProtectOutofDate();
        }
        else
        {
            ProtectOV();
        }
#else
        //过压保护控制
		ProtectOV();
#endif
		
		//欠压保护控制
		ProtectUV();
		//压差保护控制
		ProtectVdiff();
		//过温充电控制
		ProtectOTC();
		//过温放电控制
		ProtectOTD();
		//低温充电控制
		ProtectUTC();
		//低温放电控制
		ProtectUTD();
		//放电过流控制
		ProtectOCD();
		//充电过流控制
		ProtectOCC();
		//MOS 过温保护
		//ProtectMOSOT();
		//soc 充满电关充电MOS
#if(PROJECT_ID != 2)
		ProtectSocFull();
#endif
		//sc 短路保护解除
		ProtectSCR();
	}
}
