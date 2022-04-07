#include "includes.h"

#if (PROJECT_ID == 3)




#define  CAN_FILTER_STDID(STDID)    ((STDID&0x7FF)<<5)
#define  CAN_FILTER_EXTID_H(EXTID)  ((uint16_t)(((EXTID>>18)<<5)|((EXTID&0x2FFFF)>>13)))
#define  CAN_FILTER_EXTID_L(EXTID)  ((uint16_t)((EXTID&0x2FFFF)<<3))

#define CAN_EE_FLASH_START   EE_END_ADDR


CanTxMessage TxMessage;

//100ms 发送周期
static uint8_t RYCAN_Send0x200(uint16_t sv,uint16_t sc,uint8_t soc,uint8_t life)
{
    //static uint8_t life = 0;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x200;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = (uint8_t)(sv>>8);
    TxMessage.Data[1] = (uint8_t)(sv);

    TxMessage.Data[2] = (uint8_t)(sc>>8);
    TxMessage.Data[3] = (uint8_t)(sc);

    TxMessage.Data[4] = soc;
    TxMessage.Data[5] = 0x00;
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = life;

    return CANTxMessage(CAN,&TxMessage);
}

//100ms 发送周期
static uint8_t RYCAN_Send0x201(uint16_t hv,uint16_t hvp,uint16_t lv,uint16_t lvp,uint8_t bsta,uint8_t life)
{
    //static uint8_t life = 0;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x201;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = (uint8_t)(hv>>8);
    TxMessage.Data[1] = (uint8_t)(hv);

    TxMessage.Data[2] = hvp;

    TxMessage.Data[3] = (uint8_t)(lv>>8);
    TxMessage.Data[4] = (uint8_t)lv;

    TxMessage.Data[5] = lvp;
    TxMessage.Data[6] = bsta;//bit0 1=放电 0=不放电  bit1 1=充电 0=不充电  bit2 1=加热  0=不加热
    TxMessage.Data[7] = life;

    return CANTxMessage(CAN,&TxMessage);
}


//100ms 发送周期
static uint8_t RYCAN_Send0x202(uint8_t ht,uint8_t htp,uint8_t lt,uint8_t ltp,uint8_t rsta,uint8_t life)
{
    //static uint8_t life = 0;
 
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x202;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = (uint8_t)(ht);
    TxMessage.Data[1] = (uint8_t)(htp);

    TxMessage.Data[2] = lt;
    TxMessage.Data[3] = (uint8_t)(ltp);

    TxMessage.Data[4] = (uint8_t)rsta;//bit0 0-断开  1-闭合

    TxMessage.Data[5] = 0x00;
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = life;

    return CANTxMessage(CAN,&TxMessage);

}

//1000ms 发送周期
static uint8_t RYCAN_Send0x203(uint16_t batcap,uint8_t usetA,uint8_t usetB,uint16_t usetC,uint8_t life)
{
    //static uint8_t life = 0;
    
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x203;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = (uint8_t)(batcap>>8);
    TxMessage.Data[1] = (uint8_t)(batcap);

    TxMessage.Data[2] = usetA;
    TxMessage.Data[3] = usetB;

    TxMessage.Data[4] = (uint8_t)(usetC>>8);
    TxMessage.Data[5] = (uint8_t)usetC;
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = life;

    return CANTxMessage(CAN,&TxMessage);
}

//发送周期100ms
static uint8_t RYCAN_Send0x190(uint8_t err1,uint8_t err2,uint8_t err3,uint8_t err4,uint8_t err5,uint8_t err6,uint8_t bmsreq,uint8_t life)
{
    //static uint8_t life = 0;
    
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x190;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = err1;
    TxMessage.Data[1] = err2;

    TxMessage.Data[2] = err3;
    TxMessage.Data[3] = err4;

    TxMessage.Data[4] = err5;
    TxMessage.Data[5] = err6;
    TxMessage.Data[6] = bmsreq;
    TxMessage.Data[7] = life;

    return CANTxMessage(CAN,&TxMessage);
}

static uint8_t RYCAN_Send0x5F2(uint8_t cmd,uint8_t gid,uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4)
{
    //static uint8_t life = 0;
    
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x5f2;
    TxMessage.DLC = CAN_TXDLC_8;

    TxMessage.Data[0] = 0x80;
    TxMessage.Data[1] = 0x00;

    TxMessage.Data[2] = cmd;
    TxMessage.Data[3] = gid;

    TxMessage.Data[4] = data1;
    TxMessage.Data[5] = data2;
    TxMessage.Data[6] = data3;
    TxMessage.Data[7] = data4;

    return CANTxMessage(CAN,&TxMessage);
}

static uint8_t RYCAN_Send0x45b(uint8_t *buf,uint8_t len)
{
    uint8_t i;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x45b;
    TxMessage.DLC = len;

    for(i = 0;i< len;i++)
    {
        TxMessage.Data[i] = *(buf + i);
    }
    return CANTxMessage(CAN,&TxMessage);
}

static uint8_t RYCAN_GetSta()
{
    uint8_t sta = 0;
    //bit0 充电是否允许 0=允许,1=不允许  
    //bit1 充电是否完成 0=未完成，1=完成 
    //bit2 温度状态 0=正常,1=过温
    //bit3 过流状态 0=正常,1=过流
    //bit4 过压状态 0=正常,1=过压

    //不允许充电情况，过压保护，充电过流，充电过温
    if(afeInfo.State_RT & 0x54)
    {
        sta |= 0x01;
    }

    //充电完成
    if(algEnginer.soc_r ==1000)
    {
        sta |= 0x02;
    }

    //过温
    if(afeInfo.State_RT & 0x40)
    {
        sta |= 0x04;
    }

    //过流
    if(afeInfo.State_RT & 0x10)
    {
        sta |= 0x08;
    }

    //过压
    if(afeInfo.State_RT & 0x04)
    {
        sta |= 0x10;
    }


}

static uint8_t RYCAN_Send0x246()
{
    uint8_t i;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTRQ_DATA;
    TxMessage.StdId = 0x246;
    TxMessage.DLC = 8;

    TxMessage.Data[0] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_VOLT/10));
    TxMessage.Data[1] = (uint8_t)((uint16_t)(DEFAULT_CHARG_DEMAND_VOLT/10) >> 8);
    TxMessage.Data[2] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_CURR);
    TxMessage.Data[3] = (uint8_t)((uint16_t)DEFAULT_CHARG_DEMAND_CURR >> 8);
    TxMessage.Data[4] = (uint8_t)(algEnginer.soc_r/10);
    TxMessage.Data[5] = RYCAN_GetSta();
    TxMessage.Data[6] = 0x00;
    TxMessage.Data[7] = 0x00;

    return CANTxMessage(CAN,&TxMessage);
}


//写ROM处理及回应
static uint8_t RYCAN_WriteRomHandle(CanRxMessage rmsg)
{
    uint8_t gid;
    uint16_t crc;
    if(rmsg.StdId == 0x6f2 && rmsg.Data[0] == 0x22)
    {
        crc = rmsg.Data[0] + rmsg.Data[1]+rmsg.Data[2]+rmsg.Data[3]+rmsg.Data[4]+rmsg.Data[5];
        //if((rmsg.Data[6]+(uint16_t)rmsg.Data[7]<<8) == crc)
        if((rmsg.Data[6] == (uint8_t)crc) && (rmsg.Data[7] == (uint8_t)(crc>>8)))

        {
            gid = rmsg.Data[1]-1;
            //调用写报文函数
            Flash_Write(CAN_EE_FLASH_START+gid*4,(uint8_t*)&rmsg.Data[2],4);
            //回应数据
            return RYCAN_Send0x5F2(0x22,gid,1,0,0,0);
        }else
        {
            //发送错误报文
            return RYCAN_Send0x5F2(0x22,gid,0,0,0,0);
        }
    }
}


//读ROM处理及回应
static uint8_t RYCAN_ReadRomHandle(CanRxMessage rmsg)
{
    uint8_t gid;
    uint8_t romdata[4];
    if(rmsg.StdId == 0x6f2 && rmsg.Data[2] == 0x42)
    {
        gid = rmsg.Data[3] - 1;
        Flash_Read(CAN_EE_FLASH_START+gid*4,romdata,4);
        return RYCAN_Send0x5F2(0x42,gid,romdata[0],romdata[1],romdata[2],romdata[3]);
    }
}

//定义iaphanle协议，包装iaphanle数据接收协议
static void CanIapHandle(CanRxMessage rmsg)
{
    uint8_t i;
    uint8_t dlen = rmsg.DLC;
    //stdid - 0x45a 升级写数据id 
    //stdid - 0x45b 升级回写数据id

    if(rmsg.StdId == 0x45a)
    {
        for(i = 0;i<dlen;i++)
        {
            NIU_ModbusRecvHandle(rmsg.Data[i]);
        }
    }
}

static uint8_t RYCAN_GetErrCode(uint8_t *err1,uint8_t *err2,uint8_t *err3,uint8_t *err4,uint8_t *err5,uint8_t *err6,uint8_t *bmsreq)
{
    static uint16_t CountOV = 0; //过压计数
    static uint16_t CountUV = 0; //欠压计数
    static uint16_t VdiffFlag = 0;
    static uint16_t CountOCC = 0;//充电过流计数
    static uint16_t CountOCD = 0;//放电过流计数
    static uint16_t CountOTC = 0;//充电高温计数
    static uint16_t CountOTD = 0;//放电高温计数
    static uint16_t CountUTC = 0;//低温充电计数
    static uint16_t CountUTD = 0;//低温放电计数
    static uint16_t CountMOSOT = 0;//MOS 过温保护
    static uint16_t CountSoc100 = 0;//100%满电关断
    static uint16_t CountVdiff = 0;//压差大延迟计数
    //单体过压
    if(afeInfo.CellVmax > TH_WARN_OV3)
    {
        if(CountOV++ >10)
        {
            *err2 |= (0x03 <<2);
        }

        //二级报警判别
        if(afeInfo.CellVmax > TH_WARN_OV2)
        {
            *err2 &= ~(0x03 <<2);
            *err2 |= (0x02 <<2);
        }

        //三级报警判别
        if(afeInfo.CellVmax > TH_WARN_OV1)
        {
            *err2 &= ~(0x03 <<2);
            *err2 |= (0x01 <<2);
        }
    }else
    if(afeInfo.CellVmax < (TH_WARN_OV3-50))
    {
        *err2 &= ~(0x03 <<2);
        CountOV = 0;
    }

    //单体欠压
    if(afeInfo.CellVmin < TH_WARN_UV3)
    {
        //三级
        if(CountUV++ >10)
        {
           *err2 |= (0x03);
        }

        //二级报警判别
        if(afeInfo.CellVmin < TH_WARN_UV2)
        {
            *err2 &= ~(0x03);
            *err2 |= (0x02);
            *bmsreq &= ~0x03;
            *bmsreq |= 0x02;
        }

        //一级报警判别
        if(afeInfo.CellVmin < TH_WARN_UV1)
        {
            *err2 &= ~(0x03);
            *err2 |= (0x01);
            *bmsreq &= ~0x03;
            *bmsreq |= 0x01;
        }
    }else
    if(afeInfo.CellVmin > (TH_WARN_UV3+50))
    {
        *err2 &= ~(0x03);
       
        CountUV = 0;
    }

    //充电过流保护(报警)
    if(afeInfo.ChgCurrent >1000*TH_WARN_OCC3)
    {
        //三级
        if(CountOCC++ >10)
        {
            *err3 |= (0x03 <<2);
        }

        //二级报警
        if(afeInfo.ChgCurrent >1000*TH_WARN_OCC2)
        {
            *err3 &= ~(0x03 <<2);
            *err3 |= (0x02 <<2);
        }

        //一级报警
        if(afeInfo.ChgCurrent >1000*TH_WARN_OCC1)
        {
            *err3 &= ~(0x03 <<2);
            *err3 |= (0x03 <<2);
        }


    }else
    if(afeInfo.ChgCurrent < 1000*(TH_WARN_OCC3-5))
    {
        *err3 &= ~(0x03 <<2);
        CountOCC = 0;
    }

    //放电过流保护(报警)
    if(afeInfo.DsgCurrent >1000*TH_WARN_OCD3)
    {
        if(CountOCD++ >10)
        {
            *err3 |= (0x03 <<6);
        }

        if(afeInfo.DsgCurrent >1000*TH_WARN_OCD2)
        {
            *err3 &= ~(0x03 <<6);
            *err3 |= (0x02<<6);
        }

        if(afeInfo.DsgCurrent >1000*TH_WARN_OCD1)
        {
            *err3 &= ~(0x03 <<6);
            *err3 |= (0x01<<6);
        }

    }else
    if(afeInfo.ChgCurrent < 1000*(TH_WARN_OCD3-5))
    {
        *err3 &= ~(0x03 <<6);
        CountOCD = 0;
    }

    //充电过温保护
    if(afeInfo.State_RT & 0x02)
    {
        if(afeInfo.CellTmax >(TH_WARN_OCT3*10 + 2731))
        {
            if(CountOTC++ >10)
            {
                *err1 |= (0x03 <<2);
            }

            if(afeInfo.CellTmax >(TH_WARN_OCT2*10 + 2731))
            {
                *err1 &= ~(0x03 <<2);
                *err1 |= (0x02 <<2);
            }

            if(afeInfo.CellTmax >(TH_WARN_OCT1*10 + 2731))
            {
                *err1 &= ~(0x03 <<2);
                *err1 |= (0x01 <<2);
            }

        }else
        if(afeInfo.CellTmax < (10*(TH_WARN_OCT3-5)+2731))
        {
            *err1 &= ~(0x03 <<2);
            CountOTC = 0;
        }
    }
   

    //放电过温保护(报警)
    if(afeInfo.State_RT & 0x01)
    {
        if(afeInfo.CellTmax >(TH_WARN_ODT3*10 + 2731))
        {
            if(CountOTD++ >10)
            {
                *err1 |= (0x03 <<6);
            }

            if(afeInfo.CellTmax >(TH_WARN_ODT2*10 + 2731))
            {
                *err1 &= ~(0x03 <<6);
                *err1 |= (0x02 <<6);
                *bmsreq &= ~0x03;
                *bmsreq |= 0x02;
            }

            if(afeInfo.CellTmax >(TH_WARN_ODT1*10 + 2731))
            {
                *err1 &= ~(0x03 <<6);
                *err1 |= (0x01 <<6);
                *bmsreq &= ~0x03;
                *bmsreq |= 0x01;
            }

        }else
        if(afeInfo.CellTmax < (10*(TH_WARN_ODT3-5)+2731))
        {
            *err1 &= ~(0x03 <<6);
          
            CountOTD = 0;
        }
    }

    //充电低温报警
    if(afeInfo.State_RT & 0x02)
    {
        if(afeInfo.CellTmin < (TH_WARN_UCT3*10 + 2731))
        {
            if(CountUTC++ >10)
            {
                *err1 |= (0x03);
            }

            if(afeInfo.CellTmin < (TH_WARN_UCT2*10 + 2731))
            {
                *err1 &= ~(0x03);
                *err1 |= (0x02);
            }

            if(afeInfo.CellTmin < (TH_WARN_UCT1*10 + 2731))
            {
                *err1 &= ~(0x03);
                *err1 |= (0x01);
            }

        }else
        if(afeInfo.CellTmin > (10*(TH_WARN_UCT3+3)+2731))
        {
            *err1 &= ~(0x03);
            CountUTC = 0;
        }
    }

     //放电电低温报警
    if(afeInfo.State_RT & 0x01)
    {
        if(afeInfo.CellTmin < (TH_WARN_UDT3*10 + 2731))
        {
            if(CountUTD++ >10)
            {
                *err1 |= (0x03<<4);
            }

            if(afeInfo.CellTmin < (TH_WARN_UDT2*10 + 2731))
            {
                *err1 &= ~(0x03<<4);
                *err1 |= (0x02<<4);
            }

            if(afeInfo.CellTmin < (TH_WARN_UDT1*10 + 2731))
            {
                *err1 &= ~(0x03<<4);
                *err1 |= (0x01<<4);
            }

        }else
        if(afeInfo.CellTmin > (10*(TH_WARN_UDT3+3)+2731))
        {
            *err1 &= ~(0x03<<4);
            CountUTD = 0;
        }
    }

    //压差大报警
    if(afeInfo.CellVdiff > TH_WARN_VDIFF3)
    {
        if(CountVdiff++ >10)
        {
            *err2 |= (0x03<<4);
        }

        if(afeInfo.CellVdiff > TH_WARN_VDIFF2)
        {
            *err2 &= ~(0x03<<4);
            *err2 |= (0x02<<4);
        }

        if(afeInfo.CellVdiff > TH_WARN_VDIFF2)
        {
            *err2 &= ~(0x03<<4);
            *err2 |= (0x01<<4);
        }

    }else
    if(afeInfo.CellVdiff < TH_WARN_VDIFF3-100)
    {
       *err2 &= ~(0x03<<4);
        CountVdiff = 0;
    }

    //Soc 低报警
    if(algEnginer.soc_r <TH_WARN_LOW_SOC3*10)
    {
        *err3 |= (0x03);
        if(algEnginer.soc_r <TH_WARN_LOW_SOC2*10)
        {
            *err3 &= ~(0x03);
            *err3 |= (0x02);
            *bmsreq &= ~0x03;
            *bmsreq |= 0x02;

        }

        if(algEnginer.soc_r <TH_WARN_LOW_SOC1*10)
        {
            *err3 &= ~(0x03);
            *err3 |= (0x01);
            *bmsreq &= ~0x03;
            *bmsreq |= 0x01;
        }

    }else
    {
        *err3 &= ~(0x03);
    }

     //总压过压
    if(afeInfo.SumBatteryPackVolt > CELL_NUMS*TH_WARN_OV3)
    {
        //if(CountOV++ >10)
        {
            *err4 |= (0x03<<6);
        }

        if(afeInfo.SumBatteryPackVolt > CELL_NUMS*TH_WARN_OV2)
        {
            *err4 &= ~(0x03<<6);
            *err4 |= (0x02<<6);
        }

        if(afeInfo.SumBatteryPackVolt > CELL_NUMS*TH_WARN_OV1)
        {
            *err4 &= ~(0x03<<6);
            *err4 |= (0x01<<6);
        }

    }else
    if(afeInfo.SumBatteryPackVolt < CELL_NUMS*(TH_WARN_OV3-50))
    {
        *err3 &= ~(0x03<<6);
        CountOV = 0;
    }

    //总压欠压
    if(afeInfo.SumBatteryPackVolt < CELL_NUMS*TH_WARN_UV3)
    {
        //if(CountUV++ >10)
        {
            *err4 |= (0x03<<4);
        }

        if(afeInfo.SumBatteryPackVolt < CELL_NUMS*TH_WARN_UV2)
        {
            *err4 &= ~(0x03<<4);
            *err4 |= (0x02<<4);
        }

         if(afeInfo.SumBatteryPackVolt < CELL_NUMS*TH_WARN_UV2)
        {
            *err4 &= ~(0x03<<4);
            *err4 |= (0x01<<4);
        }

    }else
    if(afeInfo.SumBatteryPackVolt > CELL_NUMS*(TH_WARN_UV3+50))
    {
        *err3 &= ~(0x03<<4);
        //CountUV = 0;
    } 

    //清楚故障码
    if((*err1 == 0) && (*err2 == 0) && (*err3 == 0)&&(*err4 == 0)&&(*err5 == 0)&&(*err6 == 0))
    {
        if(*bmsreq&0x03)
        {
            *bmsreq &= ~0x03;
        }  
    }
}

void RYCAN_Init(void)
{
    uint16_t idList[2] = {0x06f2,0x045a};

    idList[0] = CAN_FILTER_STDID(idList[0]);
    idList[1] = CAN_FILTER_STDID(idList[1]);
    BxCanPortInit();
    BxCanConfig(RYCAN_RxProcess,idList,2);
    bsp_StartAutoTimer(TMR_ONEBUS_CHECK,100);
}

void RYCAN_SendData(uint8_t *buf,uint16_t len)
{
    uint16_t i = 0;
    uint16_t dl = len;
    uint8_t ret;
    //发送stdid - 0x45b 
    //分块，分片传送
    do{
        if(dl >8)
        {

            //ret = RYCAN_Send0x45b(buf+i,8);
            do{                
                ret = RYCAN_Send0x45b(buf+i,8);
                if(ret != CAN_TxSTS_NoMailBox)
                {
                    break;
                }
                bsp_DelayUS(1000);
            }while(1);
            
            i += 8;
            dl -=8;
        }else
        {            
            do{                
                ret = RYCAN_Send0x45b(buf+i,dl);
                if(ret != CAN_TxSTS_NoMailBox)
                {
                    break;
                }
                bsp_DelayUS(1000);
            }while(1);
            break;
        }        
    }while(1);   
}

void RYCAN_TxProcess(void)
{
    static uint8_t tCount = 0;
    static uint8_t life = 0;
    uint8_t rsta,bsta,hvp,htp,ltp,lvp;
    uint8_t err1 =0,err2 =0,err3 =0,err4 =0,err5 =0,err6 =0,bmsreq=0;
    uint16_t sv,sc,hv,lv,ht,lt,soc;
    uint16_t batcap,usetA,usetB,usetC;
    uint8_t ret;


    if(bsp_CheckTimer(TMR_ONEBUS_CHECK))//100ms
    {
        sv = afeInfo.SumBatteryPackVolt/100;
        if(afeInfo.State_RT == 0x01)
        {
            sc = afeInfo.DsgCurrent/100;
        }else
        if(afeInfo.State_RT == 0x02)
        {
            sc = afeInfo.ChgCurrent/100;
        }else
        {
            sc = 0;
        }

        hv = afeInfo.CellVmax;
        hvp = afeInfo.CellVmaxPos;

        lv = afeInfo.CellVmin;
        lvp = afeInfo.CellVminPos;

        ht = (afeInfo.CellTmax -2731+500) /10;
        htp = afeInfo.CellTmaxPos;

        lt = (afeInfo.CellTmin - 2731+500)/10;
        ltp = afeInfo.CellTminPos;

        if(afeInfo.State_RT == 0x01)
        {
            bsta = 0x01;
        }else
        if(afeInfo.State_RT == 0x02)
        {
            bsta = 0x02;
        }

        if(afeInfo.MosState_RT & 0x60)
        {
            rsta = 0x01;
        }
        
        soc = (uint8_t)(algEnginer.soc_r/10);
        batcap = (uint16_t)LITHIUM_FULL_AH;

        //使用时间 0.1h/bit
        usetA = 0;
        //使用时间 1h/bit
        usetB = 0;
        //使用时间 100h/bit
        usetC = 0;

        RYCAN_GetErrCode(&err1,&err2,&err3,&err4,&err5,&err6,&bmsreq);
        if(life++%2){
            ret = RYCAN_Send0x200(sv,sc,soc,life);
            ret = RYCAN_Send0x201(hv,hvp,lv,lvp,bsta,life);
            ret = RYCAN_Send0x202(ht,htp,lt,ltp,rsta,life);
        }else
        {
            ret = RYCAN_Send0x190(err1,err2,err3,err4,err5,err6,bmsreq,life);
            ret = RYCAN_Send0x246();
            if(tCount++ == 10)
            {
                ret = RYCAN_Send0x203(batcap,usetA,usetB,usetC,life);
                tCount = 0;
            }
        }    

        if(ret == CAN_TxSTS_NoMailBox)
        {
           // SEGGER_RTT_printf(0,"CAN SendData overlay\n");
        }
    }
}

void RYCAN_RxProcess(CanRxMessage rxm)
{
    if(rxm.StdId == 0x6f2)
    {
        RYCAN_WriteRomHandle(rxm);
        RYCAN_ReadRomHandle(rxm);
    }else
    {
        CanIapHandle(rxm);
    }
}

#endif




