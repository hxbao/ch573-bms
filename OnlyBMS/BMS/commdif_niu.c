/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"
//#include "app_drv_fifo.h"


//小牛定义的数据表总字节数
#define NIU_COMMD_TAB_SIZE (sizeof(stc_niuCommdTable_t))
#define NIU_MODBUS_START_ID 0x68
#define NIU_MODBUS_END_ID   0x16
//小牛BMS设备地址
#define NIU_MODBUS_DEV_A0 0x31
#define NIU_MODBUS_DEV_A1 0xCE

//命令码,主机发送的命令码
#define NIU_MODBUS_CMD_READ_SINGLE    0x01
#define NIU_MODBUS_CMD_READ           0x02
#define NIU_MODBUS_CMD_WRITE_SINGLE   0x03
#define NIU_MODBUS_CMD_WRITE          0x04
#define NIU_MODBUS_CMD_CMPARE         0x05
#define NIU_MODBUS_CMD_READ_HIS       0x06

#define TN_MODBUS_SWRESET             0xF6
#define TN_MODBUS_CMD_IAP_START       0xF7
#define TN_MODBUS_CMD_IAP_WRITE       0xF8
#define TN_MODBUS_CMD_IAP_VERIFY      0xF9

//从机回应的命令码
#define NIU_MODBUS_CMD_READ_ACK       0x82  //无后续帧
#define NIU_MODBUS_CMD_READ_ACK_ERR   0xC2  //读异常帧回应
#define NIU_MODBUS_CMD_WRITE_ACK      0x84  //写回应正常帧
#define NIU_MODBUS_CMD_WRITE_ERR      0xC4  //写回应异常帧

//回应命令错误码
typedef enum {
    CS_ERR = 0x80,
    ADDR_ERR = 0x40,
    UNKONWN_CMD_ERR = 0x20,
    NO_DATA_ERR = 0x10,
    PWD_ERR = 0x11
}ACK_ERR_en;

typedef struct{
    uint8_t mdPhyType; //物理层接入协议 0-串口   1-蓝牙
    //app_drv_fifo_t *fifo;     //蓝牙外发fifo


}NiuMdCfg_t;

uint8_t rxIndex = 0;
uint8_t NiuMdOrgInBuf[256];
uint8_t NiuMdInBuf[256];
uint8_t NiuMdAckBuf[256];
uint8_t NiuMdAckTempBuf[256];

//数据表需同步处理标志 0-无须处理   1-参数需要同步写入
uint8_t NiuMdSycWriteFlg = 0;
uint8_t NiuMdPollFlg = 0;

//蓝牙发送标志
uint8_t bleTxFlag = 0;

NiuMdCfg_t mdcfg ;//= {1,app_uart_rx_fifo};

//命令帧格式---------------------
// 说明     代码     长度（字节）
// 帧起始符 68H       1
// 地址域   A0        1
//          A1       1
// 帧起始符 68H       1
// 命令码   CMD       1
// 数据长度 LENTH     1
// 数据域   DATA    LENTH      LENTH 为数据域的字节数。读写数据时 L≤200， L=0 表示无数据域。
// 校验码   CS       1         数据域可包括数据标识、密码、数据、帧序号等，其结构随命令码的功能而改变。 传输时发送方按字节进行加 33H 处理，接收方按字节进行减 33H 处理。
// 结束符   16H      1         从第一个帧起始符开始到校验码之前的所有各字节的模 256 的和，即各字节二进制算术和，不计超过 256 的溢出值。
//命令帧格式---------------------


//命令码格式----------------------
//D7 D6 D5
//D7 传送方向  0：主站发出的命令帧  1：从站发出的应答帧
//D6 从站应答标志 0：从站正确应答
//D5 1：有后续数据帧 0 后续帧标志

//（D4 D3 D2 D1 D0）
// 00000 ：保留
// 00001 ：读数据
// 00010 ：连续读数据
// 00011 ：写数据
// 00100 ：连续写数据
// 00101 ：对比数据
// 00110 ： BMS 履历读取
//命令码格式----------------------

/**
 * function:小牛回应帧打包函数
 * @pDesBuf:外部输入被打包后的数据存放的缓冲
 * @cmd    :回应的命令码
 * @data   :根据对应的命令，做相应填充
 * @length :data数据的长度
 */ 
static uint8_t CommdPackAckFrame(uint8_t *pDesBuf,uint8_t cmd,uint8_t *data,uint8_t length)
{
    uint8_t i;
    uint8_t cs = 0;
    *pDesBuf = NIU_MODBUS_START_ID;
    *(pDesBuf+1) = NIU_MODBUS_DEV_A0;
    *(pDesBuf+2) = NIU_MODBUS_DEV_A1;
    *(pDesBuf+3) = NIU_MODBUS_START_ID;
    *(pDesBuf+4) = cmd;
    *(pDesBuf+5) = length;

    for(i = 0;i < length; i++)
    {
        *(pDesBuf+6+i) = *(data+i);
    }

    for(i = 0;i < length+6; i++)
    {
        cs += *(pDesBuf+i);
    }
    *(pDesBuf+6+length) = cs;
    *(pDesBuf+7+length) = NIU_MODBUS_END_ID;
    return 8+length;
    
}

/**
 * function:CommdModbusCheck 
 * @pSrcD  :输入的原始数据帧
 * @cmd    :输出 主机发送的cmd码
 * @len    :输出 主机发送的长度
 * @data   :输出 主机发送的数据字段
 * @return :检测的结果码
 */ 

static uint8_t CommdModbusCheck(uint8_t *pSrcD,uint8_t *cmd,uint8_t *len,uint8_t *data)
{
    uint8_t i;
    uint8_t l;
    uint8_t cs = 0;
    uint8_t err;
    uint8_t c;

    l = *(pSrcD+5);
    c = *(pSrcD+4);
    for(i = 0;i <l+6;i++)
    {
        cs += *(pSrcD + i);
    }

    if(*(pSrcD+l+6) != cs)
    {
        //cs 校验错误
        err = CS_ERR;
        *cmd = 0;
        *len = 0;
        *data = 0;
        return err;
    }else         
        if( c != NIU_MODBUS_CMD_READ_SINGLE   && \
            c != NIU_MODBUS_CMD_READ          && \
            c != NIU_MODBUS_CMD_WRITE_SINGLE  && \
            c != NIU_MODBUS_CMD_WRITE         && \
            c != NIU_MODBUS_CMD_CMPARE        && \
            c != NIU_MODBUS_CMD_READ_HIS      && \
            c != TN_MODBUS_SWRESET            && \
            c != TN_MODBUS_CMD_IAP_START      && \
            c != TN_MODBUS_CMD_IAP_WRITE      && \
            c != TN_MODBUS_CMD_IAP_VERIFY     
            
            )
        {
            err = UNKONWN_CMD_ERR;
            *cmd = 0;
            *len = 0;
            *data = 0;
            return err;
        }else
        {
            err = 0x00;
            *cmd = *(pSrcD+4);
            *len = *(pSrcD+5);

            for(i = 0;i <*len;i++)
            {
                *(data+i)= *(pSrcD+6+i);
            }        
            
        }
    return err;
}

static void NIu_ModbusSendAck(uint8_t *pbuf,uint8_t len)
{
   // uint16_t len1 = len;
    //一线通串口接收
    if(mdcfg.mdPhyType == 0)
    {
        //Uart3SendData(pbuf,len);
        UART3_SendString(pbuf, len);
    }else
    if(mdcfg.mdPhyType == 1){
        //ble蓝牙接收
        //app_drv_fifo_write(mdcfg.fifo, pbuf, &len1);

        //发送ble发送事件
        bleTxFlag = 1;
        //tmos_start_task(Peripheral_TaskID, UART_TO_BLE_SEND_EVT, 2);
    }

}

static void NiuModbusParse(uint8_t *pMdInput)
{
    uint8_t i;
    uint8_t cmd;
    uint8_t len;
    uint8_t Addr;   //内部寄存器读写地址
    uint8_t length; //内部寄存器读写的数据长度
    uint8_t length2;//返回数据的发送长度
    uint8_t WPwd[4];
    uint8_t ret;

    ret = CommdModbusCheck(pMdInput,&cmd,&len,NiuMdInBuf);

    switch(cmd)
    {
        case NIU_MODBUS_CMD_READ_SINGLE:
            break;
        case NIU_MODBUS_CMD_READ : 
            if((ACK_ERR_en)ret == CS_ERR || (ACK_ERR_en)ret == UNKONWN_CMD_ERR)
            {
                length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_READ_ACK_ERR,&ret,1);
                NIu_ModbusSendAck(NiuMdAckBuf,length2);
            }else
            {
                Addr = NiuMdInBuf[0] - 0x33;
                length = NiuMdInBuf[1] - 0x33;
                if(Addr + length >=NIU_COMMD_TAB_SIZE)
                {
                    ret = (ACK_ERR_en)ADDR_ERR;
                    length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_READ_ACK_ERR,&ret,1);
                    NIu_ModbusSendAck(NiuMdAckBuf,length2);
                }else
                {
                    //恢复数据+0x33
                    for(i = 0;i<length;i++)
                    {
                        NiuMdAckTempBuf[i] = *((uint8_t*)&niuCommdTable+Addr+i)+0x33;
                    }
                
                    length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_READ_ACK,NiuMdAckTempBuf,length);
                    NIu_ModbusSendAck(NiuMdAckBuf,length2);
                }
            }
            break;    
        case NIU_MODBUS_CMD_WRITE_SINGLE:
            break;
        case NIU_MODBUS_CMD_WRITE :
            if((ACK_ERR_en)ret == CS_ERR || (ACK_ERR_en)ret == UNKONWN_CMD_ERR)
            {
                length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_WRITE_ERR,&ret,1);
                NIu_ModbusSendAck(NiuMdAckBuf,length2);
            }else
            {
                Addr = NiuMdInBuf[0]- 0x33;
                length = NiuMdInBuf[1] -0x33;
                WPwd[0] = NiuMdInBuf[2] -0x33;
                WPwd[1] = NiuMdInBuf[3]-0x33;
                WPwd[2] = NiuMdInBuf[4]-0x33;
                WPwd[3] = NiuMdInBuf[5]-0x33;
                if(Addr + length >NIU_COMMD_TAB_SIZE)
                {
                    ret = (uint8_t)ADDR_ERR;
                    length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_WRITE_ERR,&ret,1);
                    NIu_ModbusSendAck(NiuMdAckBuf,length2);
                }else if(WPwd[0] != niuCommdTable.P_Pwrd[0] && \
                         WPwd[1] != niuCommdTable.P_Pwrd[1] && \
                         WPwd[2] != niuCommdTable.P_Pwrd[2] && \
                         WPwd[3] != niuCommdTable.P_Pwrd[3]
                         )
                {

                    //密码错误
                    length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_WRITE_ERR,(uint8_t*)&niuCommdTable+Addr,length);
                    NIu_ModbusSendAck(NiuMdAckBuf,length2);
                }
                else
                {
                    //更新table
                    for(i = 0;i < length;i++)
                    {
                        *((uint8_t*)&niuCommdTable+Addr+i) = (uint8_t)(NiuMdInBuf[6+i] -0x33);
                        if(Addr+i >=25 && Addr+i <=38)
                        {
                            NiuMdSycWriteFlg |= 0x01; //保护参数表更新
                        }else
                        if(Addr+i >=193 && Addr+i <=201)//校准参数
                        {
                            //校准指令 电压k =1000 o=0，电流k=990 o=0
                            //68 31 ce 68 04  0e(L) c1(addr)+0x33 08+0x33 55+0x33 55+0x33 55+0x33 55+0x33  03+0x33  E8+0x33  00+0x33  00+0x33  03+0x33  DE+0x33  00+0x33  00+0x33  BE 16
                            //68 31 ce 68 04 0e f4 3b 88 88 88 88 03 E8 00 00 03 DE 00 00 FC 16
                            NiuMdSycWriteFlg |= 0x04; //校准参数更新
                        }
                        else
                        if(Addr+i == 209)
                        {
                            //NiuMdSycWriteFlg |= 0x08; //通信模式设定
                            //写通信模式指令
                            //68 31 ce 68 04  0e(L) d2(addr)+0x33 08+0x33 55+0x33 55+0x33 55+0x33 55+0x33  03+0x33  E8+0x33  00+0x33  00+0x33  03+0x33  DE+0x33  00+0x33  00+0x33  BE 16
                            //68 31 ce 68 04 07 04 34 88 88 88 88 01(模式控制位 01 为强制uart通信 00 自主选择一线通模式) 33 16
                            //PRINT("commMode = %d\n",niuCommdTable.commMode);
                        }
                        else
                        {
                            NiuMdSycWriteFlg |= 0x02; //其他参数需要更新
                        }
                    }
                    
                    length2 = CommdPackAckFrame(NiuMdAckBuf,NIU_MODBUS_CMD_WRITE_ACK,0,0);
                    NIu_ModbusSendAck(NiuMdAckBuf,length2);
                }
            }
            break;  

            break;        
        case NIU_MODBUS_CMD_CMPARE :  
            break;      
        case NIU_MODBUS_CMD_READ_HIS:
            break;
        case TN_MODBUS_CMD_IAP_START:
        case TN_MODBUS_CMD_IAP_WRITE:
        case TN_MODBUS_CMD_IAP_VERIFY:
 //           length2 = IapHanle(cmd,len,pMdInput,NiuMdAckBuf);
 //           NIu_ModbusSendAck(NiuMdAckBuf,length2);

//            PRINT("iap ack:\n");
//            for(int i = 0;i<length2;i++){
//                    PRINT("%02X ",NiuMdAckBuf[i]);
//            }
//            PRINT("\n");

//            if((NiuMdAckBuf[4] == TN_MODBUS_CMD_IAP_VERIFY) && NiuMdAckBuf[6] == 0x01)
//            {
//                //
//                bsp_DelayMS(1000);
//                NVIC_SystemReset();
//            }
            break;

        case TN_MODBUS_SWRESET:   //软件复位
            CommdPackAckFrame(NiuMdAckBuf,TN_MODBUS_SWRESET,0,0);
            NIu_ModbusSendAck(NiuMdAckBuf,2);
			//NVIC_SystemReset();
            break;
    }
}

void Niu_ModbufFifoClr(void)
{
    rxIndex = 0;
}

//void Niu_ModbusCfg(uint8_t mtype,app_drv_fifo_t *ackFifo)
//{
//    if(mtype == 1)
//    {
//        mdcfg.mdPhyType = mtype;
//        //mdcfg.fifo = ackFifo;
//    }else {
//        mdcfg.mdPhyType = 0;
//    }
//
//}

void NIU_ModbusRecvHandle(uint8_t rdata)
{
    if(rdata == NIU_MODBUS_START_ID && rxIndex == 0)
    {
        NiuMdOrgInBuf[rxIndex++] = rdata;
        return;
    }else
    if(rxIndex > 0)
    {           
        NiuMdOrgInBuf[rxIndex++] = rdata;
        if(rxIndex > 254)
        {
            rxIndex = 0;
            return;
        }
        if(rxIndex >=3)
        {
            if(NiuMdOrgInBuf[1] != NIU_MODBUS_DEV_A0 || NiuMdOrgInBuf[2] != NIU_MODBUS_DEV_A1)
            {
                //地址不匹配，重新计数
                rxIndex = 0;
                return;
            }
        }
        
        if(rdata == NIU_MODBUS_END_ID)
        {
            if(rxIndex == (NiuMdOrgInBuf[5]+8) && rdata == NIU_MODBUS_END_ID)
            {
                //接收完完整一帧
                //68 31 ce 68 02 02 60 6a 9d 16
                //PRINT("iap recv:\n"); 
                // for(int i = 0;i<rxIndex;i++){
                //     PRINT("%02X ",NiuMdOrgInBuf[i]);
                // }   
                // PRINT("\n"); 
                NiuMdPollFlg = 1;          
                //NiuModbusParse(NiuMdOrgInBuf);
                rxIndex = 0;
            }
        }        
    }
}

void NiuModbusPoll(void)
{
    if(NiuMdPollFlg)
    {
        NiuModbusParse(NiuMdOrgInBuf);
        NiuMdPollFlg = 0;
    }    
}

