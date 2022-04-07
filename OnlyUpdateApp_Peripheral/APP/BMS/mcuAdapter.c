#include "includes.h"
#include "HAL.h"


#if (MCU_LIB_SELECT == 1)


#elif (MCU_LIB_SELECT == 2)

#define VREF_VAL (2048)
float ADC_Ratio = (3300.0) / 4095;


void ADC_GetMcuAdcInfo(uint16_t *buf, uint16_t *mosTemp, uint16_t *balTemp, uint16_t *preDsgCur)
{

//    //开启温探power
//    GPIOA_SetBits(TN_NTC0_POWER_PAPIN);
//    bsp_DelayMS(10);
//
//    *buf = Hal_GetAdcSampleValue(NTC0_ADC_CH);             //PMATCH,PB12 AdcExInputCH19
//    *(buf + 1) = Hal_GetAdcSampleValue(NTC1_ADC_CH);       //balance temp ,PA07 ，NTC1
//    *(buf + 2) = Hal_GetAdcSampleValue(MOS_ADC_CH);       //mos temp ,PB00 ，NTC
//    //*(buf + 3) = ADC_GetChannleVal(ADC_CH_VREFINT); //2.048V
//    *(buf + 4) = Hal_GetAdcSampleValue(BAL_ADC_CH);      //PB1 ,pre_Isen_p
//    *(buf + 5) = 0;                                 // ADC_GetChannleVal(ADC_CH_4);   //PB1 ,pre_Isen_n
//
//    //use vrefint calibrate the VDD as vref
//    //ADC_Ratio = (float)VREF_VAL / *(buf + 3);
//    *mosTemp = CalcuTemp((uint16_t)(*(buf + 2) * ADC_Ratio));
//    *balTemp = CalcuTemp((uint16_t)(*(buf + 1) * ADC_Ratio));
//
//    *preDsgCur = (uint16_t)(*(buf + 4) * ADC_Ratio);
//    //关闭 temperature power
//    GPIOA_ResetBits(TN_NTC0_POWER_PAPIN);
}

/**
 * @brief  Initializes peripherals
 * @param  None
 * @retval None
 */
void TWI_Init(void)
{
/*    I2C_InitType i2c1_master;
    GPIO_InitType i2c1_gpio;
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C2, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    I2C_DeInit(TWI_I2C);
    i2c1_master.BusMode = I2C_BUSMODE_I2C;
    i2c1_master.FmDutyCycle = I2C_FMDUTYCYCLE_2;
    i2c1_master.OwnAddr1 = 0x55;
    i2c1_master.AckEnable = I2C_ACKEN;
    i2c1_master.AddrMode = I2C_ADDR_MODE_7BIT;
    i2c1_master.ClkSpeed = I2C_SPEED; // 100K

    I2C_Init(TWI_I2C, &i2c1_master);
    I2C_Enable(TWI_I2C, ENABLE);
    */
   //use io simulate
}

//------------------------------------------------------------------
//模块内部函数开始
//------------------------------------------------------------------
#define SCK_H() GPIOB_SetBits(TN_I2C1_SCL_PBPIN)
#define SCK_L() GPIOB_ResetBits( TN_I2C1_SCL_PBPIN)

#define SDA_H() GPIOB_SetBits(TN_I2C1_SDA_PBPIN)
#define SDA_L() GPIOB_ResetBits( TN_I2C1_SDA_PBPIN)


static uint8_t READ_SDA(void)
{
    uint8_t bit = 0;
    GPIOB_ModeCfg(TN_I2C1_SDA_PBPIN, GPIO_ModeIN_PU);
    bsp_DelayUS(10);
    bit = GPIOB_ReadPortPin(TN_I2C1_SDA_PBPIN);
    return bit;
}

static void TWI_IIC_Start(void)
{
    
    SDA_H();
    SCK_H();    
    bsp_DelayUS(100);
    SDA_L();
    bsp_DelayUS(100);
    SCK_L();
    // bsp_DelayUS(10);
}

static void TWI_IIC_Stop(void)
{
    SDA_L();
    SCK_H();
    bsp_DelayUS(10);
    SDA_H();
}

static uint8_t TWI_IIC_Ack(void)
{
    uint8_t ack = 0;
    //check ack
    //SWITCH_SDA_MODE(1);
    SCK_L();
    bsp_DelayUS(10);
    SCK_H();    
    ack = READ_SDA();
    //SWITCH_SDA_MODE(0);
    bsp_DelayUS(5);
    SCK_L();
    bsp_DelayUS(10);
    return ack;
}

static void TWI_IIC_SendAck(uint8_t isAck)
{
    if(isAck)
    {
        SDA_L();
    }else
    {
        SDA_H();
    }
    bsp_DelayUS(10);  
    SCK_H();     
    bsp_DelayUS(10);
    SCK_L();
    bsp_DelayUS(1);
    SDA_H();
}

static void TWI_IIC_WriteByte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if ((byte & 0x80) == 0x80)
        {
            SDA_H();
        }
        else
        {
            SDA_L();
        }
        bsp_DelayUS(10);
        SCK_H();
        bsp_DelayUS(10);
        SCK_L();
        bsp_DelayUS(10);
        byte <<= 1;
    }
    SDA_H();
    bsp_DelayUS(5);
}

static uint8_t TWI_IIC_ReadByte()
{
    uint8_t i;
    uint8_t byte;

    for (i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCK_L();
        bsp_DelayUS(10);
        SCK_H();
        if (READ_SDA() == SET)
        {
            byte |= 0x01;
        }
        bsp_DelayUS(10);
    }
    SCK_L();
    SDA_H();
    return byte;
}

//static uint8_t TWI_WriteRegByte(uint8_t addr, uint8_t byte)
//{
//    uint8_t retCode = 0;
//    TWI_IIC_Start();
//    TWI_IIC_WriteByte(addr);
//    if (TWI_IIC_Ack())
//    {
//        retCode = 1;
//    }
//    TWI_IIC_WriteByte(byte);
//    if (TWI_IIC_Ack())
//    {
//        retCode = 1;
//    }
//    TWI_IIC_Stop();
//    return retCode;
//}

/**
 ******************************************************************************
 ** \brief  主机接收函数
 **
 ** \param u8Addr从机内存地址，pu8Data读数据存放缓存，u32Len读数据长度
 **
 ** \retval 读数据是否成功
 **
 ******************************************************************************/
uint8_t I2C_MasterReadData(uint8_t SlaveID, uint8_t u8Addr, uint8_t *pu8Data, uint32_t u32Len)
{
    uint8_t retCode = 0;
    uint8_t i;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(SlaveID);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //PRINT("ACK ERROR1\n");
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //PRINT("ACK ERROR2\n");
    }
#if(AFE_CHIP_SELECT == 3)
#else
    TWI_IIC_WriteByte((uint8_t)u32Len);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //PRINT("ACK ERROR3\n");
    }
#endif
    TWI_IIC_Start();
    //read
    TWI_IIC_WriteByte((SlaveID|0x01));//(SlaveID|0x01)
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //PRINT("ACK ERROR4\n");
    }

    for(i = 0;i<u32Len;i++)
    {
        *(pu8Data+i) = TWI_IIC_ReadByte();
        //PRINT("%02x ",*(pu8Data+i));
        //send ack
        TWI_IIC_SendAck(1);

    }
    //PRINT("\n");
    //read crc
    *(pu8Data+i) = TWI_IIC_ReadByte();
    //send nack
    TWI_IIC_SendAck(0);

    TWI_IIC_Stop();
    return retCode;
}
/**
 ******************************************************************************
 ** \brief  主机发送函数
 **
 ** \param u8Addr从机内存地址，pu8Data写数据，u32Len写数据长度
 **
 ** \retval 写数据是否成功
 **
 ******************************************************************************/
uint8_t I2C_MasterWriteData(uint8_t SlaveID, uint8_t u8Addr, uint8_t *pu8Data, uint32_t u32Len)
{
    uint8_t retCode = 0;
    uint8_t i;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(SlaveID);
    if (TWI_IIC_Ack())
    {
        //PRINT("I2C_MasterWriteData ACK ERROR1\n");
        retCode = 1;
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        //PRINT("I2C_MasterWriteData ACK ERROR2\n");
        retCode = 1;
    }

    for(i = 0;i<u32Len;i++)
    {
        TWI_IIC_WriteByte(*(pu8Data+i));
        if (TWI_IIC_Ack())
        {
            //PRINT("I2C_MasterWriteData ACK ERROR3\n");
            retCode = 1;
            break;
        }
    }
    TWI_IIC_Stop();
    return retCode;
}

#endif
