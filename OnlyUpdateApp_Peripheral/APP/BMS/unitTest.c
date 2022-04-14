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




//RTT接收字符数据
uint8_t rttKeyInArray[20];//read whole table
uint8_t rttRxIndex = 0;
uint8_t fEnTestSh309 = 0;


void RunUnitTest();
static void PrintOutNiuCommdTable(void);
static void PrintOutAlgEnginerInfo(void);
static void PrintOutConfigMacro(void);

void UnitTestInit(void)
{
    bsp_StartAutoTimer(TMR_UINT_TEST, 1000);
}

void DebugHandleRecvData(uint8_t data)
{
    if(rttRxIndex<20)
    {
        rttKeyInArray[rttRxIndex++] = data;
        if(data == '\n')
        {
           RunUnitTest();
           rttRxIndex = 0;
        }
    }
}

void RunUnitTest()
{
    //afe test
    if(strstr(rttKeyInArray,"tafe")>0)
    {

        SH_ENABLE_SHIPMODE();
        bsp_DelayMS(200);
        SH_DISABLE_SHIPMODE();
        bsp_DelayMS(500);
        fEnTestSh309 = 1;
        PRINT("run tafe\n");

//        Sh_CheckEEData();
//        Sh_EnableCADC();
    }else
    if(strstr(rttKeyInArray,"tonebus")>0){
        //一线通时序测试
        fEnTestSh309 = 2;
        UART3_Reset();
        //Niu_OneBusInit();
        PRINT("run tonebus\n");

    }else
    if(strstr(rttKeyInArray,"tmodbus")>0){
        //串口modbus测试
        PRINT("run tmodbus\n");
        fEnTestSh309 = 3;
        Uart3Init(NIU_ModbusRecvHandle);

    }else
    if(strstr(rttKeyInArray,"teepromw")>0)
    {
        //写eeprom数据测试
        Flash_Write(BAT_SN_ADDR_START, "1234567812345678", 16);
    }
    else
    if(strstr(rttKeyInArray,"teepromr")>0)
    {
            //写eeprom数据测试
        Flash_Read(BAT_SN_ADDR_START, CommonRam, 16);
//        niuCommdTable.SN_ID[0] = '1';
//        niuCommdTable.SN_ID[1] = '2';
//        niuCommdTable.SN_ID[2] = '3';
//        niuCommdTable.SN_ID[3] = '4';
//        niuCommdTable.SN_ID[4] = '5';
//        niuCommdTable.SN_ID[5] = '6';
        PRINT("niuCommdTable.SN_ID->%s\n",CommonRam);
    }else
    if(strstr(rttKeyInArray,"trcom")>0)
    {
        PrintOutNiuCommdTable();
    }
    else
    if(strstr(rttKeyInArray,"trconf")>0)
    {
        PrintOutConfigMacro();
    }
    else
    if(strstr(rttKeyInArray,"talg")>0)
    {
        PrintOutAlgEnginerInfo();
    }else
    if(strstr(rttKeyInArray,"tgafe")>0){
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
            PRINT("vcell->(%d:%d) \n", i + 1, afeInfo.CellVolt[i]);
        }
        PRINT( "\n");
    }

}

void UnitTestProcess(void)
{
    //static uint8_t count = 0;
    uint8_t ret;
    uint8_t buf[2];
    //test afe data out
    if(bsp_CheckTimer(TMR_UINT_TEST))
    {
        if(fEnTestSh309 == 1)
        {
            //PRINT("sh_process count%d\n",count++);
            ret = SH309Twi_Read(I2C_SLAVE_ADDRESS7, 0x40, 2, buf);
            if(ret != 0)
            {

                PRINT("sh309 read ret =%d\n",ret);
            }else {
                PRINT("sh309 read buf =%x,%x\n",buf[0],buf[1]);
            }
        }


    }
    if(fEnTestSh309 == 2)
    {
        UART3_INTCfg(DISABLE, RB_IER_RECV_RDY);
        //一线通时序测试
        Tn_OneBusProcess();
        UART3_CLR_RXFIFO();
        UART3_INTCfg(ENABLE,RB_IER_RECV_RDY);
    }
    if(fEnTestSh309 == 3)
    {
        NiuModbusPoll();
    }
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
