#include "includes.h"

#ifdef UNIT_TEST_EN



//返回的数据
//68 31 CE 68 82 A0 53 64 34 34 88 88 88 88 34 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 32 33 33 33 33 33 33 33 33 22 22 22 22 22 22 33 63 33 33 33 33 34 ED 33 33 33 33 33 34 33 32 37 37 4B 33 38 41 F1 40 59 3F 24 40 3A 40 CE 40 3F 40 55 41 B0 40 36 3F 2F 3F 1E 40 44 3F FC 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 33 79 63 77 64 63 89 63 64 89 64 33 33 33 3A 41 3C 34 79 40 41 F1 3F FC 33 3F 34 28 3E 0A 3E 09 33 34 34 28 52 33 97 48 87 48 87 33 33 33 33 33 33 33 33 33 33 1D 16 


// function
//     local s = ...
//     local str = s:toHex()
  
//     local t = {}
//     local m = {}

//     t.Msg = "niuprotocol"
//     t.originData = str
//     m.niuPayload = t
//     return json.encode(m)

// end



// function
//     local s = ...
//     local str = s:toHex()
//     local addr = str:sub(1,10)
//     local strlen = str:len()/2 -2
//     local cs = 0
//     local t = {}
//     local m = {}

//     --地址命令匹配判断
//     if(addr ~= "6831ce") then 
//         t.errMsg = "Invalid addr"
//         t.originData = str
//         m.err = t
//         return json.encode(m)
//     end
//      --校验数据完整性
//     for i =1,strlen do
//        cs = cs + tonumber(str:sub(2*(i-1)+1,2*(i-1)+2),16)
//     end
//     cs = cs%0x100
//     if(cs == tonumber(str:sub(-4,-3),16)) then       
//         --解码数据包
//         local cmd = tonumber(str:sub(9,10),16)
//         local datlen = tonumber(str:sub(11,12),16)
//         --payload 数据需要减0x33，才能解码到原始数据，payload 数据就是表地址0开始到最后的数据
//         local payload = str:sub(13,-5) 
//         --读表数据回应
       
//           t.fun = 0x82
//           t.datlen = 0x6
//           t.payload = payload        
//           m.niuprotocol = t
//           return json.encode(m)

//     else
//         t.errMsg = "checksum error"
//         t.originData = str
//         m.err = t
//         return json.encode(m)
//     end
// end



uint8_t inited = 0;
uint8_t abDataOut[512];
uint64_t timeTick = 0;
//RTT接收字符数据
uint8_t rttKeyInArray[11]={0x68,0x31,0xce,0x68,0x02,0x02,0x33,0xd3,0xd9,0x16};//read whole table
uint8_t rttRxIndex = 0;

static void PrintOutNiuCommdTable(void);
static void PrintOutConfigMacro(void);
static void PrintOutAlgEnginerInfo(void);
static void  RttLogRecord(void);

void UnitTestProcess(void)
{
    //stc_gpio_cfg_t pstcGpioCfg;
    int rttKeyinChar;

    //rttKeyinChar = SEGGER_RTT_GetKey();

    if(inited == 0)
    {
//        inited = 1;
//        SEGGER_RTT_ConfigUpBuffer(1, "DataOut", &abDataOut[0], 512,
//        SEGGER_RTT_MODE_NO_BLOCK_SKIP);
//        bsp_StartAutoTimer(TMR_UINT_TEST,1000);
    }

    if(bsp_CheckTimer(TMR_UINT_TEST))
    {
        //data record
        //RttLogRecord();
    }
    
    if(rttKeyinChar>0)
    {

           
        if((char)rttKeyinChar=='1')
        {
            
           
//            PRINT("DelayMS ONETX 500ms Test...\n");
//            MCU_GPIO_SetBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
//            bsp_DelayMS(500);
//            MCU_GPIO_ClrBit(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
           
             //使能时钟输出，测量
        }else
        if((char)rttKeyinChar=='2')
        {
           /*   ///< 端口方向配置->输出
            pstcGpioCfg.enDir = GpioDirOut;
            ///< 端口驱动能力配置->高驱动能力
            pstcGpioCfg.enDrv = GpioDrvH;
            ///< 端口上下拉配置->无上下拉
            pstcGpioCfg.enPu = GpioPuDisable;
            pstcGpioCfg.enPd = GpioPdDisable;
            ///< 端口开漏输出配置->开漏输出关闭
            pstcGpioCfg.enOD = GpioOdDisable;    
            ///< GPIO IO PB00初始化
            //Gpio_Init(GpioPortA, GpioPin14, &pstcGpioCfg);
            Gpio_Init(GpioPortA, GpioPin1, &pstcGpioCfg);

            Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
            Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);*/
        }else
        if((char)rttKeyinChar=='3')//afeInfo view
        {
            //三个前端温度
            PRINT( "TimeS->%d\n",bsp_GetRunTime());
            PRINT("ShTemp1,2,3 ->%d,%d,%d\n",afeInfo.ShTemp[0]-2731,afeInfo.ShTemp[1]-2731,afeInfo.ShTemp[2]-2731);
            PRINT("BalenceTemp,MOSTemp ->%d,%d\n",afeInfo.BalanceTemp-2731,afeInfo.MosTemp-2731);
            PRINT("Vmax,Vmin,VmaxPos,VminPos ->%d,%d,%d,%d\n",afeInfo.CellVmax,afeInfo.CellVmin,afeInfo.CellVmaxPos+1,afeInfo.CellVminPos+1);
            PRINT( "sumv->%d\n",afeInfo.SumBatteryPackVolt);
            PRINT( "State_RT->%4X\n",afeInfo.State_RT);
            PRINT( "MosState_RT->%4X\n",afeInfo.MosState_RT); 
            PRINT( "ChgCurrent mA->%d\n",afeInfo.ChgCurrent);   
            PRINT( "DsgCurrent mA->%d\n",afeInfo.DsgCurrent);         
            PRINT( "PreDCurr->%d\n",afeInfo.PreDsgCurrent);
            PRINT( "Pre_State->%2X\n",afeInfo.Pre_State);
            
            for (int i = 0; i < CELL_NUMS; i++)
            {
                PRINT( "vcell->(%d:%d) \n", i + 1, afeInfo.CellVolt[i]);
            }
            PRINT( "\n");

        }else
        if((char)rttKeyinChar=='4')//开关充放电MOS，采集MOS反馈信号
        {
            //Sh_ShutDDsgMos();
            //Sh_ShutDChgMos();
            AFE_CLOSE_DMOS();
            afeInfo.MosState_RT &= ~0x20;
           

            bsp_DelayMS(1000);
            if(GET_DMOS_FB() == 1)
            {
                PRINT("DMOS FB is OK\n");
            }else
            {
                PRINT("DMOS FB is NOK\n");
            }
            
           

            AFE_OPEN_DMOS();
            afeInfo.MosState_RT |= 0x20;
           
            if(GET_DMOS_FB() == 0)
            {
                PRINT("DMOS FB is OK\n");
            }else
            {
                PRINT("DMOS FB is NOK\n");
            }
            
            
            
            //Sh_OpenDsgMos();
            //Sh_OpenChgMos();
        }else
        if((char)rttKeyinChar=='5')//预放电开关信号测试
        {
            AFE_CLOSE_CMOS();
            afeInfo.MosState_RT &= ~0x40;
            bsp_DelayMS(500);
            if(GET_CMOS_FB() == 1)
            {
                PRINT("CMOS FB is OK\n");
            }else
            {
                PRINT("CMOS FB is NOK\n");
            }


            AFE_OPEN_CMOS();
            bsp_DelayMS(500);
            afeInfo.MosState_RT |= 0x40;
            if(GET_CMOS_FB() == 0)
            {
                PRINT("CMOS FB is OK\n");
            }else
            {
                PRINT("CMOS FB is NOK\n");
            }


            // SWITCH_PRED_ON();
            // bsp_DelayMS(500);
            // SWITCH_PRED_OFF();
            // PRINT("PREDsg Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='6')//串口一线通电路测试
        {

            //SWITCH_TX_LOW();
            NIU_ModbusRecvHandle(rttKeyInArray[rttRxIndex++]);
            
            if(rttRxIndex == 10)
            {
                rttRxIndex = 0;
            }
            //Sh_ShutDChgMos();



            //MCU_GPIO_SetBit(TN_SHINT_PORT,TN_SHINT_PIN);


            // SWITCH_PRED_ON();
            // bsp_DelayMS(500);
            // SWITCH_PRED_OFF();
            // PRINT("PREDsg Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='7')//充电唤醒电路测试
        {
            // SWITCH_PRED_ON();
            // bsp_DelayMS(500);
            // SWITCH_PRED_OFF();
            // PRINT("PREDsg Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='8')//系统唤醒和ACC唤醒电路测试
        {
            // SWITCH_PRED_ON();
            // bsp_DelayMS(500);
            // SWITCH_PRED_OFF();
            // PRINT("PREDsg Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='9')//三端保险丝自毁电路驱动测试
        {
            //关断DMOS和CMOS，防止烧毁保险丝
            //Sh_ShutDDsgMos();
            //Sh_ShutDChgMos();

            //MCU_GpioKillMeInit();
            //SWITCH_KILLME_ON();
            //bsp_DelayMS(500);
            //SWITCH_KILLME_OFF();
            PRINT("KillMe Driver test,Please Check Oscilloscope Signal\n");
        }else
        if((char)rttKeyinChar=='a')//低功耗测试
        {
            PRINT("shipmode power test,Please Check power consumption\n");
            //关断DMOS和CMOS，防止烧毁保险丝
            //Sh_ShutDDsgMos();
            //打开预放
            SWITCH_PRED_ON();
            
            // Sh_ShutDChgMos();

            // //芯片进入SHIP mode
            // SH_ENABLE_SHIPMODE();

            // EnableNvic(RTC_IRQn, IrqLevel3, FALSE);
		    // //禁止通信唤醒
            // Gpio_DisableIrq(TN_WAKE_UP_PORT, TN_WAKE_UP_PIN, GpioIrqRising); //系统接入
            // Gpio_DisableIrq(TN_ACC_PORT, TN_ACC_PIN, GpioIrqRising);			//ACC
            // Gpio_DisableIrq(TN_ALARM_PORT, TN_ALARM_PIN, GpioIrqFalling);	//sh309 中断
            // Gpio_DisableIrq(TN_SHINT_PORT, TN_SHINT_PIN, GpioIrqRising);		//通信中断
            // //
            // ADC_DeConfig();
            // Lpm_GotoSleep(FALSE);            
        }else
        if((char)rttKeyinChar=='b')//获取时钟频率
        {
           //uint32_t fval;
           // fval = Sysctrl_GetHClkFreq();
           // PRINT("HclkFreq->%d\n",fval);
        }
        else
        if((char)rttKeyinChar=='c')//打印niu modbus 数据表
        {
                          
            PrintOutNiuCommdTable();
        }else
        if((char)rttKeyinChar=='d')//打印flash write 测试
        {
            for(int i = 0; i<16;i++){
                niuCommdTable.SN_ID[i] = (uint8_t)i+0x30;
            }
                         
            Flash_Write(EE_START_ADDR + BAT_SN_ADDR_START, niuCommdTable.SN_ID, 16);

            for(int i = 0;i<16;i++){
                PRINT("flash read data->%02x \n",*(uint8_t*)(EE_START_ADDR + BAT_SN_ADDR_START+i));
            }
            PRINT("\n");
        }else
        if((char)rttKeyinChar=='f')//打印config文件
        {
            PrintOutConfigMacro();
        }else
        if((char)rttKeyinChar=='s')
        {
            //打印soc 引擎信息
            PrintOutAlgEnginerInfo();
        }
    }
}

static void  RttLogRecord(void)
{

}

static void PrintOutNiuCommdTable(void)
{
    PRINT("addr %d,User_ID %02x\n",0,niuCommdTable.User); 
    PRINT("addr %d,BMS_ID %02x\n",1,niuCommdTable.BMS_ID);                 
    PRINT("addr %d,S_Ver %02x\n",2,niuCommdTable.S_Ver);                  
    PRINT("addr %d,H_Ver %02x\n",3,niuCommdTable.H_Ver);                  
    PRINT("addr %d,P_Pwrd[0] %02x\n" ,4,niuCommdTable.P_Pwrd[0]);  
    PRINT("addr %d,P_Pwrd[1] %02x\n" ,5,niuCommdTable.P_Pwrd[1]); 
    PRINT("addr %d,P_Pwrd[2] %02x\n" ,6,niuCommdTable.P_Pwrd[2]); 
    PRINT("addr %d,P_Pwrd[3] %02x\n" ,7,niuCommdTable.P_Pwrd[3]);            
    PRINT("addr %d,Bat_Tytp %02x\n" ,8,niuCommdTable.Bat_Tytp);
    for(int i = 0;i<16;i++) 
    {
        PRINT("addr %d,SN_ID[%d] %2x\n", 9+i,i,niuCommdTable.SN_ID[i]);  
    }              
                    
    PRINT("addr %d,OC_Vlt_P %02x,x10,%d\n" ,25,niuCommdTable.OC_Vlt_P,niuCommdTable.OC_Vlt_P);               
    PRINT("addr %d,ROC_Vlt %02x,x10,%d\n" ,26,niuCommdTable.ROC_Vlt,niuCommdTable.ROC_Vlt);                
    PRINT("addr %d,Dc_Vlt_P %02x,x10,%d\n" ,27,niuCommdTable.Dc_Vlt_P,niuCommdTable.Dc_Vlt_P);               
    PRINT("addr %d,RDc_Vlt_P %02x,x10,%d\n" ,28,niuCommdTable.RDc_Vlt_P,niuCommdTable.RDc_Vlt_P);              
    PRINT("addr %d,C_B_Vlt %02x ,x10,%d\n" ,29,niuCommdTable.C_B_Vlt,niuCommdTable.C_B_Vlt);                
    PRINT("addr %d,C_Cur_P %02x ,x10,%d\n" ,30,niuCommdTable.C_Cur_P,niuCommdTable.C_Cur_P);               
    PRINT("addr %d,Dc_Cur_P %02x ,x10,%d\n" ,31,niuCommdTable.Dc_Cur_P,niuCommdTable.Dc_Cur_P);               
    PRINT("addr %d,Isc_P %02x,x10,%d\n" ,32,niuCommdTable.Isc_P,niuCommdTable.Isc_P);                  
    PRINT("addr %d,DcOTp_P %02x,x10,%d\n" ,33,niuCommdTable.DcOTp_P,niuCommdTable.DcOTp_P);                
    PRINT("addr %d,RDcOTp_P %02x,x10,%d\n" ,34,niuCommdTable.RDcOTp_P,niuCommdTable.RDcOTp_P);               
    PRINT("addr %d,COTp_P %02x,x10,%d\n" ,35,niuCommdTable.COTp_P,niuCommdTable.COTp_P);                 
    PRINT("addr %d,RCoTp_P %02x,x10,%d\n" ,36,niuCommdTable.RCoTp_P,niuCommdTable.RCoTp_P);                
    PRINT("addr %d,DcLTp_P %02x,x10,%d\n" ,37,niuCommdTable.DcLTp_P,niuCommdTable.DcLTp_P);                
    PRINT("addr %d,RDcLTp_P %02x,x10,%d\n" ,38,niuCommdTable.RDcLTp_P,niuCommdTable.RDcLTp_P);               
    PRINT("addr %d,NCom_DC_En %02x\n" ,39,niuCommdTable.NCom_DC_En);             
    PRINT("addr %d,Rated_Vlt %02x\n" ,40,niuCommdTable.Rated_Vlt);              
    PRINT("addr %d,Bat_To_Cap[0] %02x\n" ,41,niuCommdTable.Bat_To_Cap[0]);
    PRINT("addr %d,Bat_To_Cap[1] %02x\n" ,42,niuCommdTable.Bat_To_Cap[1]);            
    PRINT("addr %d,C_Cont[0] %02x\n" ,43,niuCommdTable.C_Cont[0]); 
    PRINT("addr %d,C_Cont[1] %02x\n" ,44,niuCommdTable.C_Cont[1]);              
    PRINT("addr %d,To_Vlt_RT[0]) %02x\n" ,45,niuCommdTable.To_Vlt_RT[0]);           
    PRINT("addr %d,To_Vlt_RT[1] %02x\n" ,46,niuCommdTable.To_Vlt_RT[1]);           
    PRINT("addr %d,C_Cur_RT[0] %02x\n" ,47,niuCommdTable.C_Cur_RT[0]);            
    PRINT("addr %d,C_Cur_RT[1] %02x\n" ,48,niuCommdTable.C_Cur_RT[1]);            
    PRINT("addr %d,Dc_Cur_RT[0] %02x\n" ,49,niuCommdTable.Dc_Cur_RT[0]);           
    PRINT("addr %d,Dc_Cur_RT[1] %02x\n" ,50,niuCommdTable.Dc_Cur_RT[1]);           
    PRINT("addr %d,SOC_RT %02x\n" ,51,niuCommdTable.SOC_RT);                 
    PRINT("addr %d,Bat_Sta_RT[0] %02x\n" ,52,niuCommdTable.Bat_Sta_RT[0]); 
    PRINT("addr %d,Bat_Sta_RT[1] %02x\n" ,53,niuCommdTable.Bat_Sta_RT[1]);          
    PRINT("addr %d,DC_Fl_T_RT %02x\n" ,54,niuCommdTable.DC_Fl_T_RT);              
    PRINT("addr %d,Tp1_RT[0] %02x\n" ,55,niuCommdTable.Tp1_RT[0]); 
    PRINT("addr %d,Tp1_RT[1] %02x\n" ,56,niuCommdTable.Tp1_RT[1]); 
    PRINT("addr %d,Tp1_RT[2] %02x\n" ,57,niuCommdTable.Tp1_RT[2]);
    PRINT("addr %d,Tp1_RT[3] %02x\n" ,58,niuCommdTable.Tp1_RT[3]);             
    PRINT("addr %d,Tp5_RT %02x\n" ,59,niuCommdTable.Tp5_RT); 

    for(int i = 0;i<24;i++)    
    {
        PRINT("addr %d,G_Vlt_RT[%d],%04x,%d\n",60+2*i,i,((uint16_t)niuCommdTable.G_Vlt_RT[2*i]<<8)+niuCommdTable.G_Vlt_RT[2*i+1],((uint16_t)niuCommdTable.G_Vlt_RT[2*i]<<8)+niuCommdTable.G_Vlt_RT[2*i+1]);  
    }            

    for(int i = 0;i<8;i++)
    {
        PRINT("addr %d,S_Ver_N[%d],%02x\n" ,108+i,i,niuCommdTable.S_Ver_N[i]);             
    
    } 
    for(int i = 0;i<8;i++)
    {
        PRINT("addr %d,H_Ver_N[%d],%02x\n" ,116+i,i,niuCommdTable.H_Ver_N[i]);
    }
                    
                
    PRINT("addr %d,Blance_T[0],%02x\n" ,124,niuCommdTable.Blance_T[0]); 
    PRINT("addr %d,Blance_T[1],%02x\n" ,125,niuCommdTable.Blance_T[1]);  

            
    PRINT("addr %d,Cell_Num,%d\n" ,126,niuCommdTable.Cell_Num);               
    PRINT("addr %d,Cell_Vmax[0],%02x\n" ,127,niuCommdTable.Cell_Vmax[0]); 
    PRINT("addr %d,Cell_Vmax[1],%02x\n" ,128,niuCommdTable.Cell_Vmax[1]); 

    PRINT("addr %d,Cell_Vmin[0],%02x\n" ,129,niuCommdTable.Cell_Vmin[0]); 
    PRINT("addr %d,Cell_Vmin[1],%02x\n" ,130,niuCommdTable.Cell_Vmin[1]); 
    
    PRINT("addr %d,Cell_Vmax_Num,%d\n" ,131,niuCommdTable.Cell_Vmax_Num); 
    PRINT("addr %d,Cell_Vmin_Num,%d\n" ,132,niuCommdTable.Cell_Vmin_Num); 
    
    PRINT("addr %d,Cell_Vdiff[0],%02x\n" ,133,niuCommdTable.Cell_Vdiff[0]);
    PRINT("addr %d,Cell_Vdiff[1],%02x\n" ,134,niuCommdTable.Cell_Vdiff[1]);  
    
    PRINT("addr %d,Cell_Tmax[0],%02x\n" ,135,niuCommdTable.Cell_Tmax[0]);
    PRINT("addr %d,Cell_Tmax[1],%02x\n" ,136,niuCommdTable.Cell_Tmax[1]);                   

    PRINT("addr %d,Cell_Tmin[0],%02x\n" ,137,niuCommdTable.Cell_Tmin[0]);
    PRINT("addr %d,Cell_Tmin[1],%02x\n" ,138,niuCommdTable.Cell_Tmin[1]);                   


    PRINT("addr %d,Cell_Tmax_Num,%d\n" ,139,niuCommdTable.Cell_Tmax_Num);
    PRINT("addr %d,Cell_Tmin_Num,%d\n" ,140,niuCommdTable.Cell_Tmin_Num);
            

    PRINT("addr %d,Cell_Tdiff[0],%02x\n" ,141,niuCommdTable.Cell_Tdiff[0]);
    PRINT("addr %d,Cell_Tdiff[1],%02x\n" ,142,niuCommdTable.Cell_Tdiff[1]);
    PRINT("addr %d,LifeValue,%02x\n" ,143,niuCommdTable.LifeValue);
    PRINT("addr %d,SOC_High_Precision,%d\n" ,144,niuCommdTable.SOC_High_Precision);
    PRINT("addr %d,SOH_RT,%d\n" ,145,niuCommdTable.SOH_RT);

    PRINT("addr %d,ChargDemandVolt[0],%02x\n" ,146,niuCommdTable.ChargDemandVolt[0]);
    PRINT("addr %d,ChargDemandVolt[1],%02x\n" ,147,niuCommdTable.ChargDemandVolt[1]);
    
    PRINT("addr %d,ChargDemandCurrent[0],%02x\n" ,148,niuCommdTable.ChargDemandCurrent[0]);
    PRINT("addr %d,ChargDemandCurrent[1],%02x\n" ,149,niuCommdTable.ChargDemandCurrent[1]);                                   

    PRINT("addr %d,DisCharg_Max_P[0],%02x\n" ,150,niuCommdTable.DisCharg_Max_P[0]);
    PRINT("addr %d,DisCharg_Max_P[1],%02x\n" ,151,niuCommdTable.DisCharg_Max_P[1]); 

    PRINT("addr %d,Charg_Max_P[0],%02x\n" ,152,niuCommdTable.Charg_Max_P[0]);
    PRINT("addr %d,Charg_Max_P[1],%02x\n" ,153,niuCommdTable.Charg_Max_P[1]);

    PRINT("addr %d,Blance_Status[0],%02x\n" ,154,niuCommdTable.Blance_Status[0]);  
    PRINT("addr %d,Blance_Status[1],%02x\n" ,155,niuCommdTable.Blance_Status[1]);
    PRINT("addr %d,Blance_Status[2],%02x\n" ,156,niuCommdTable.Blance_Status[2]);
    PRINT("addr %d,Blance_Status[3],%02x\n" ,157,niuCommdTable.Blance_Status[3]);    

    PRINT("addr %d,MOS_Ctrol,%02x\n" ,158,niuCommdTable.MOS_Ctrol);  
    
    PRINT("addr %d,MOS_Status,%02x\n" ,159,niuCommdTable.MOS_Status);
}

static void PrintOutConfigMacro(void)
{
    PRINT("FW_VERSION = %d\n",FW_VERSION);
    PRINT("HW_VERSION = %d\n",FW_VERSION);

    PRINT("BAT_TYPE = %d\n",BAT_TYPE);
    PRINT("NIU_FW_VER = %s\n",NIU_FW_VER);
    PRINT("NIU_HW_VER = %s\n",NIU_HW_VER);
    PRINT("COMPLIE_TIME = %s\n",COMPLIE_TIME);
    PRINT("MCU_LIB_SELECT = %d\n",MCU_LIB_SELECT);
    PRINT("CELL_NUMS = %d\n",CELL_NUMS);
    PRINT("BAT_RATED_VOLT = %d\n",BAT_RATED_VOLT);
    PRINT("LITHIUM_TYPE = %d\n",LITHIUM_TYPE);
    PRINT("LITHIUM_CYCLE_E = %d\n",LITHIUM_CYCLE_E);
    PRINT("LITHIUM_FULL_AH = %d\n",LITHIUM_FULL_AH);
    PRINT("SOCSUMCALC_INTVAL = %d\n",SOCSUMCALC_INTVAL);
    PRINT("DEFAULT_CHARG_DEMAND_VOLT = %d\n",DEFAULT_CHARG_DEMAND_VOLT);
    PRINT("DEFAULT_CHARG_DEMAND_CURR = %d\n",DEFAULT_CHARG_DEMAND_CURR);
    PRINT("CHG_FULL_OCV_TH = %d\n",CHG_FULL_OCV_TH);
    PRINT("MAXCELL_FULL_SLEEP_TH = %d\n",MAXCELL_FULL_SLEEP_TH);
    PRINT("SHIP_MODE_CELLV_TH = %d\n",SHIP_MODE_CELLV_TH);
    PRINT("CELL_VMAX_OVER_TH = %d\n",CELL_VMAX_OVER_TH);
    PRINT("MOS_HIGH_WARN = %d\n",MOSOTP_W);
    PRINT("BALANCE_START_V = %d\n",BALANCE_START_V);
    PRINT("BALANCE_START_S = %d\n",BALANCE_START_S);
    PRINT("BALANCE_START_E = %d\n",BALANCE_START_E);
    PRINT("EE_START_ADDR = %X\n",EE_START_ADDR);
    
#if(AFE_CHIP_SELECT == 1)

    PRINT("SH_DEFAULT_EECONFIG_1 = %X\n",SH_DEFAULT_EECONFIG_1);
    PRINT("SH_DEFAULT_EECONFIG_2 = %X\n",SH_DEFAULT_EECONFIG_2);
    PRINT("SH_DEFAULT_EECONFIG_3 = %X\n",SH_DEFAULT_EECONFIG_3);
    PRINT("SH_DEFAULT_EECONFIG_4 = %X\n",SH_DEFAULT_EECONFIG_4);
    PRINT("SH_DEFAULT_EECONFIG_5 = %X\n",SH_DEFAULT_EECONFIG_5);
    PRINT("SH_DEFAULT_EECONFIG_6 = %X\n",SH_DEFAULT_EECONFIG_6);
    PRINT("SH_DEFAULT_EECONFIG_7 = %X\n",SH_DEFAULT_EECONFIG_7);
    PRINT("SH_DEFAULT_EECONFIG_8 = %X\n",SH_DEFAULT_EECONFIG_8);
    PRINT("SH_DEFAULT_EECONFIG_9 = %X\n",SH_DEFAULT_EECONFIG_9);
    PRINT("SH_DEFAULT_EECONFIG_10 = %X\n",SH_DEFAULT_EECONFIG_10);
    PRINT("SH_DEFAULT_EECONFIG_11 = %X\n",SH_DEFAULT_EECONFIG_11);
    PRINT("SH_DEFAULT_EECONFIG_12 = %X\n",SH_DEFAULT_EECONFIG_12);
    PRINT("SH_DEFAULT_EECONFIG_13 = %X\n",SH_DEFAULT_EECONFIG_13);
    PRINT("SH_DEFAULT_EECONFIG_14 = %X\n",SH_DEFAULT_EECONFIG_14);
    PRINT("SH_DEFAULT_EECONFIG_15 = %X\n",SH_DEFAULT_EECONFIG_15);
    PRINT("SH_DEFAULT_EECONFIG_16 = %X\n",SH_DEFAULT_EECONFIG_16);
    PRINT("SH_DEFAULT_EECONFIG_17 = %X\n",SH_DEFAULT_EECONFIG_17);
    PRINT("SH_DEFAULT_EECONFIG_18 = %X\n",SH_DEFAULT_EECONFIG_18);
    PRINT("SH_DEFAULT_EECONFIG_19 = %X\n",SH_DEFAULT_EECONFIG_19);
    PRINT("SH_DEFAULT_EECONFIG_20 = %X\n",SH_DEFAULT_EECONFIG_20);
    PRINT("SH_DEFAULT_EECONFIG_21 = %X\n",SH_DEFAULT_EECONFIG_21);
    PRINT("SH_DEFAULT_EECONFIG_22 = %X\n",SH_DEFAULT_EECONFIG_22);
    PRINT("SH_DEFAULT_EECONFIG_23 = %X\n",SH_DEFAULT_EECONFIG_23);
    PRINT("SH_DEFAULT_EECONFIG_24 = %X\n",SH_DEFAULT_EECONFIG_24);
    PRINT("SH_DEFAULT_EECONFIG_25 = %X\n",SH_DEFAULT_EECONFIG_25);

#endif

    PRINT("OCV_100 = %d\n",OCV_100);
    PRINT("OCV_95 = %d\n",OCV_95);
    PRINT("OCV_90 = %d\n",OCV_90);
    PRINT("OCV_85 = %d\n",OCV_85);
    PRINT("OCV_80 = %d\n",OCV_80);
    PRINT("OCV_75 = %d\n",OCV_75);
    PRINT("OCV_70 = %d\n",OCV_70);
    PRINT("OCV_65 = %d\n",OCV_65);
    PRINT("OCV_60 = %d\n",OCV_60);
    PRINT("OCV_55 = %d\n",OCV_55);
    PRINT("OCV_50 = %d\n",OCV_50);
    PRINT("OCV_45 = %d\n",OCV_45);
    PRINT("OCV_40 = %d\n",OCV_40);
    PRINT("OCV_35 = %d\n",OCV_35);
    PRINT("OCV_30 = %d\n",OCV_30);
    PRINT("OCV_25 = %d\n",OCV_25);
    PRINT("OCV_20 = %d\n",OCV_20);
    PRINT("OCV_15 = %d\n",OCV_15);
    PRINT("OCV_10 = %d\n",OCV_10);
    PRINT("OCV_5 = %d\n",OCV_5);
    PRINT("OCV_0 = %d\n",OCV_0);

}

static void PrintOutAlgEnginerInfo(void)
{
    SocEngineGetPrintInfo();
    PRINT( "TimeS->%d\n",bsp_GetRunTime());
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

#endif
