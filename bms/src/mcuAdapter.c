#include "includes.h"


static uint8_t FlagRtcWakeUp = 0;
#if (MCU_LIB_SELECT == 1)

#define VREF_VAL (1200)
float ADC_Ratio = (3300.0) / 4095;
void MCU_SetNVOffset()
{
    if (VECT_OFFSET != 0 && __VTOR_PRESENT == 1)
    { //需要设置中断地址的偏移
        SCB->VTOR = VECT_OFFSET;
    }
}

void GPIOInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;
    //所有管脚初始化驱动为输入L，下拉
    for (uint8_t i = 0; i < 15; i++)
    {

        Gpio_Init(GpioPortA, (en_gpio_pin_t)i, &stcGpioCfg);
        Gpio_Init(GpioPortB, (en_gpio_pin_t)i, &stcGpioCfg);
    }
    Gpio_Init(GpioPortC, (en_gpio_pin_t)13, &stcGpioCfg);
    Gpio_Init(GpioPortC, (en_gpio_pin_t)14, &stcGpioCfg);
    Gpio_Init(GpioPortC, (en_gpio_pin_t)15, &stcGpioCfg);
    Gpio_Init(GpioPortF, (en_gpio_pin_t)1, &stcGpioCfg);
    Gpio_Init(GpioPortF, (en_gpio_pin_t)0, &stcGpioCfg);

    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

    Gpio_Init(TN_SHINT_PORT, TN_SHINT_PIN, &stcGpioCfg);
    Gpio_Init(TN_ALARM_PORT, TN_ALARM_PIN, &stcGpioCfg);
    Gpio_Init(TN_ACC_PORT, TN_ACC_PIN, &stcGpioCfg);
    Gpio_Init(TN_WAKE_UP_PORT, TN_WAKE_UP_PIN, &stcGpioCfg);
    Gpio_Init(TN_CHG_DET_PORT, TN_CHG_DET_PIN, &stcGpioCfg);
    Gpio_Init(TN_DMOSFB_PORT, TN_DMOSFB_PIN, &stcGpioCfg);
    Gpio_Init(TN_CMOSFB_PORT, TN_CMOSFB_PIN, &stcGpioCfg);


    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;//GpioPdEnable
    stcGpioCfg.enOD = GpioOdDisable;

#if(PROJECT_ID == 2)    
    /// BKEN
    Gpio_Init(TN_BK_EN_PORT, TN_BK_EN_PIN, &stcGpioCfg);
    Gpio_Init(TN_LED2_PORT, TN_LED2_PIN, &stcGpioCfg);

#endif

    /// PREEN
    Gpio_Init(TN_PREEN_PORT, TN_PREEN_PIN, &stcGpioCfg);
    /// VPRO_CON
    Gpio_Init(TN_VPRO_CON_PORT, TN_VPRO_CON_PIN, &stcGpioCfg);
    //SHIP
    Gpio_Init(TN_SHSHIP_PORT, TN_SHSHIP_PIN, &stcGpioCfg);
    /// NTC0_Power
    Gpio_Init(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN, &stcGpioCfg);
    /// NTC1_Power
    Gpio_Init(TN_NTC1_POWER_PORT, TN_NTC1_POWER_PIN, &stcGpioCfg);
    //LED
    Gpio_Init(TN_LED_PORT, TN_LED_PIN, &stcGpioCfg);

    Gpio_ClrIO(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    Gpio_ClrIO(TN_NTC1_POWER_PORT, TN_NTC1_POWER_PIN);
    Gpio_ClrIO(TN_PREEN_PORT, TN_PREEN_PIN);
    Gpio_ClrIO(TN_VPRO_CON_PORT, TN_VPRO_CON_PIN);


    //PF1 CLKout
    //Gpio_Init(GpioPortF, GpioPin1, &stcGpioCfg);

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdEnable;
    //IIC SDA,SCL
    Gpio_Init(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN, &stcGpioCfg); ///<
    Gpio_Init(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN, &stcGpioCfg); ///<
   

    Gpio_SetAfMode(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN, GpioAf2); ///<
    Gpio_SetAfMode(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN, GpioAf2); ///<

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvH;
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

    Gpio_Init(TN_ONE_TX_PORT, TN_ONE_TX_PIN, &stcGpioCfg);
    Gpio_SetIO(TN_ONE_TX_PORT, TN_ONE_TX_PIN);

#if(USE_485_IF == 1)

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    stcGpioCfg.enOD = GpioOdDisable;
    //485 TX PIN
    Gpio_Init(TN_485_TXEN_PORT, TN_485_TXEN_PIN, &stcGpioCfg);
    //电源使能
    Gpio_Init(TN_485_COMM_ON_PORT, TN_485_COMM_ON_PIN, &stcGpioCfg);
    //对外输出使能电源
    Gpio_Init(TN_COMM_ON2_PORT, TN_COMM_ON2_PIN, &stcGpioCfg);

    Gpio_ClrIO(TN_485_TXEN_PORT,TN_485_TXEN_PIN);
    Gpio_SetIO(TN_485_COMM_ON_PORT, TN_485_COMM_ON_PIN);

    stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

    //485 中断
    Gpio_Init(TN_485_INT_PORT, TN_485_INT_PIN, &stcGpioCfg);


#endif
}

void Clk_Config(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);
    Flash_WaitCycle(FlashWaitCycle0);
    //Sysctrl_SetXTHFreq(SysctrlXthFreq24_32MHz);
    Sysctrl_SetRCHTrim(SysctrlRchFreq24MHz);
    stcCfg.enClkSrc = SysctrlClkRCH;
    stcCfg.enHClkDiv = SysctrlHclkDiv1;
    stcCfg.enPClkDiv = SysctrlPclkDiv1;
    //初始化为内部高速时钟24MHZ
    Sysctrl_ClkInit(&stcCfg);

    //内部低速时钟
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
}

void MCU_GpioKillMeInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdDisable;

        ///< KILLME
    Gpio_Init(TN_KILLME_PORT, TN_KILLME_PIN, &stcGpioCfg);
    Gpio_Init(TN_KILLME_EN_PORT, TN_KILLME_EN_PIN, &stcGpioCfg);

}

void Iwdt_Config(void)
{
    ///< 开启WDT外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralWdt, TRUE);
    ///< WDT 初始化
    Wdt_Init(WdtResetEn, WdtT52s4);
}

void Iwdt_Start(void)
{
    Wdt_Start();
}

void Iwdt_Feed(void)
{
    Wdt_Feed();
}

//-----------------------------RTC----------------------------------------------
void RTC_ConfigInit(void)
{
    stc_rtc_initstruct_t RtcInitStruct;

    Sysctrl_SetPeripheralGate(SysctrlPeripheralRtc, TRUE); //
    RtcInitStruct.rtcAmpm = RtcPm;                         //
    RtcInitStruct.rtcClksrc = RtcClkRcl;                   //
    RtcInitStruct.rtcPrdsel.rtcPrdsel = RtcPrdx;           //
    RtcInitStruct.rtcPrdsel.rtcPrdx = 20;                  //10秒，步进0.5s
    RtcInitStruct.rtcTime.u8Second = 0x55;
    RtcInitStruct.rtcTime.u8Minute = 0x01;
    RtcInitStruct.rtcTime.u8Hour = 0x10;
    RtcInitStruct.rtcTime.u8Day = 0x01;
    RtcInitStruct.rtcTime.u8DayOfWeek = 0x04;
    RtcInitStruct.rtcTime.u8Month = 0x04;
    RtcInitStruct.rtcTime.u8Year = 0x21;
    RtcInitStruct.rtcCompen = RtcCompenEnable; //实在补偿
    RtcInitStruct.rtcCompValue = 0;            //补偿值
    Rtc_Init(&RtcInitStruct);
    Rtc_AlmIeCmd(TRUE);                    //
    EnableNvic(RTC_IRQn, IrqLevel3, TRUE); //
    Rtc_Cmd(TRUE);                         //
    Rtc_StartWait();                       //
}


uint16_t rtcCount = 0;
void Rtc_IRQHandler(void)
{
    
    if (Rtc_GetPridItStatus() == TRUE)
    {
        Rtc_ClearPrdfItStatus(); //
        //RTC 唤醒之后，需要根据进入中断的类型
        if(flagIntEnterType == 0x02)
        {
            //深度休眠
            //只清楚WDT，然后继续休眠
            Iwdt_Feed();
        }else
        if(flagIntEnterType == 0x01)
        {
            //唤醒之后退出中断，继续跑
            SCB->SCR &= ~0x02;
            
        }
        rtcCount ++;
        FlagRtcWakeUp = 0x01;
    }
}

uint16_t Rtc_GetCount(){
    return rtcCount;
}

void Rtc_SetCount(uint16_t val)
{
    rtcCount = 0;
}

uint8_t Rtc_GetWakeUpFlag(void)
{
    return FlagRtcWakeUp;
}

void Rtc_ClrWakeUpFlag(void)
{
    FlagRtcWakeUp = 0;
}

void RTC_AlarmITEnable(void)
{
    Rtc_AlmIeCmd(TRUE);
}

void RTC_AlarmITDisable(void)
{
    Rtc_AlmIeCmd(FALSE);
}

//-----------------------------END RTC----------------------------------------------

///< ADC 采样端口初始化
void ADC_PortInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    ///< 开启 GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);


    Gpio_SetAnalogMode(TN_NTC1_ADIN_PORT, TN_NTC1_ADIN_PIN);
    Gpio_SetAnalogMode(TN_NTC0_ADIN_PORT, TN_NTC0_ADIN_PIN);
    Gpio_SetAnalogMode(TN_PRE_ISEN_P_PORT, TN_PRE_ISEN_P_PIN);
    //Gpio_SetAnalogMode(TN_PRE_ISEN_N_PORT, TN_PRE_ISEN_N_PIN);
}

void ADC_DeConfig(void)
{
    stc_adc_cfg_t stcAdcCfg;
    DDL_ZERO_STRUCT(stcAdcCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    Bgr_BgrEnable(); ///< 开启BGR

    ///< ADC 初始化配置
    stcAdcCfg.enAdcMode = AdcSglMode;                   ///<采样模式-单次
    stcAdcCfg.enAdcClkDiv = AdcMskClkDiv1;              ///<采样分频-1
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle12Clk; ///<采样周期数-12
    stcAdcCfg.enAdcRefVolSel = AdcMskRefVolSelInBgr2p5; ///<参考电压选择-内部2.5V
    stcAdcCfg.enAdcOpBuf = AdcMskBufDisable;            ///<OP BUF配置-关
    stcAdcCfg.enInRef = AdcMskInRefEnable;             ///<内部参考电压使能-开
    stcAdcCfg.enAdcAlign = AdcAlignRight;               ///<转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);
    Adc_Disable();
    Bgr_BgrDisable();
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, FALSE);
}

uint16_t ADC_GetChannleVal(en_adc_samp_ch_sel_t ch)
{
    uint8_t i;
    uint16_t adcVal = 0;
    ///< ADC 采样通道配置
    Adc_CfgSglChannel(ch);

    ///< 启动单次转换采样
    for (i = 0; i < 4; i++)
    {
        Adc_SGL_Start();
        //wait finish convert
        while (Adc_GetIrqStatus(AdcMskIrqSgl) == FALSE)
            ;
        adcVal += Adc_GetSglResult();
        Adc_SGL_Stop();
    }
    return (adcVal >> 2);
}

void ADC_Config(void)
{
    stc_adc_cfg_t stcAdcCfg;

    DDL_ZERO_STRUCT(stcAdcCfg);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE);
    Bgr_BgrEnable(); ///< 开启BGR

    ///< ADC 初始化配置 
    stcAdcCfg.enAdcMode = AdcSglMode;                   ///<采样模式-单次
    stcAdcCfg.enAdcClkDiv = AdcMskClkDiv1;              ///<采样分频-1
    stcAdcCfg.enAdcSampCycleSel = AdcMskSampCycle4Clk; ///<采样周期数-12
    stcAdcCfg.enAdcRefVolSel = AdcMskRefVolSelAVDD;     ///<参考电压选择-AVDD
    stcAdcCfg.enAdcOpBuf = AdcMskBufEnable;             ///<OP BUF配置-关
    stcAdcCfg.enInRef = AdcMskInRefEnable;              ///<内部参考电压使能-开
    stcAdcCfg.enAdcAlign = AdcAlignRight;               ///<转换结果对齐方式-右
    Adc_Init(&stcAdcCfg);
}

void ADC_GetMcuAdcInfo(uint16_t *buf, uint16_t *mosTemp, uint16_t *balTemp, uint16_t *preDsgCur)
{
    //开启温探power
    Gpio_SetIO(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    bsp_DelayMS(10);
    //*buf = ADC_GetChannleVal(AdcExInputCH19);        //PMATCH,PB12 AdcExInputCH19
    
    *(buf + 3) = ADC_GetChannleVal(AdcVref1_2Input); //1.2V
    *(buf + 4) = ADC_GetChannleVal(AdcExInputCH9);   //PB1 ,pre_Isen_p
    //*(buf + 5) = ADC_GetChannleVal(AdcExInputCH16);  //PB2 ,pre_Isen_n
    *(buf + 1) = ADC_GetChannleVal(AdcExInputCH3);   // ,PA03 ，balance temp 
    *(buf + 2) = ADC_GetChannleVal(AdcExInputCH4);   // ,PA04 ，mos temp

    //use vrefint calibrate the VDD as vref
    //ADC_Ratio = (float)VREF_VAL / *(buf + 3);
    *mosTemp = CalcuTemp((uint16_t)(*(buf + 2) * ADC_Ratio));
    *balTemp = CalcuTemp((uint16_t)(*(buf + 1) * ADC_Ratio));
    *preDsgCur = (uint16_t)(*(buf + 4) * ADC_Ratio); //mA
    //关闭 temperature power
    Gpio_ClrIO(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
}

/**
 * @brief  Initializes peripherals
 * @param  None
 * @retval None
 */
void TWI_Init(void)
{
    stc_i2c_cfg_t stcI2cCfg;

    DDL_ZERO_STRUCT(stcI2cCfg); ///< 初始化结构体变量的值为0
    Sysctrl_SetPeripheralGate(SysctrlPeripheralI2c1, TRUE); ///< 开启I2C1时钟门控

    stcI2cCfg.u32Pclk = Sysctrl_GetPClkFreq(); ///< 获取PCLK时钟
    stcI2cCfg.u32Baud = 50000;                 ///< 波特率50kHz
    stcI2cCfg.enMode = I2cMasterMode;          ///< I2C主机模式
    stcI2cCfg.u8SlaveAddr = 0x55;              ///< 从地址，主模式无效
    stcI2cCfg.bGc = FALSE;                     ///< 广播地址应答使能关闭，主模式无效
    I2C_Init(TWI_I2C, &stcI2cCfg);             ///< 模块初始化
}

// void Mcu_PeripherGoSleep(void)
// {
//     stc_gpio_cfg_t stcGpioCfg;
//     Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

//     stcGpioCfg.enDir = GpioDirIn;
//     stcGpioCfg.enDrv = GpioDrvL;
//     stcGpioCfg.enPu = GpioPuDisable;
//     stcGpioCfg.enPd = GpioPdDisable;
//     stcGpioCfg.enOD = GpioOdDisable;
//     //所有管脚初始化驱动为输入L，下拉
//     for (uint8_t i = 0; i < 15; i++)
//     {

//         Gpio_Init(GpioPortA, (en_gpio_pin_t)i, &stcGpioCfg);
//         Gpio_Init(GpioPortB, (en_gpio_pin_t)i, &stcGpioCfg);
//     }
//     Gpio_Init(GpioPortC, (en_gpio_pin_t)13, &stcGpioCfg);
//     Gpio_Init(GpioPortC, (en_gpio_pin_t)14, &stcGpioCfg);
//     Gpio_Init(GpioPortC, (en_gpio_pin_t)15, &stcGpioCfg);
//     Gpio_Init(GpioPortF, (en_gpio_pin_t)1, &stcGpioCfg);
//     Gpio_Init(GpioPortF, (en_gpio_pin_t)0, &stcGpioCfg);


//     stcGpioCfg.enDir = GpioDirIn;
//     stcGpioCfg.enDrv = GpioDrvL;
//     stcGpioCfg.enPu = GpioPuDisable;
//     stcGpioCfg.enPd = GpioPdDisable;
//     stcGpioCfg.enOD = GpioOdDisable;

//     Gpio_Init(TN_SHINT_PORT, TN_SHINT_PIN, &stcGpioCfg);
//     Gpio_Init(TN_ALARM_PORT, TN_ALARM_PIN, &stcGpioCfg);
//     Gpio_Init(TN_ACC_PORT, TN_ACC_PIN, &stcGpioCfg);
//     Gpio_Init(TN_WAKE_UP_PORT, TN_WAKE_UP_PIN, &stcGpioCfg);
//     Gpio_Init(TN_CHG_DET_PORT, TN_CHG_DET_PIN, &stcGpioCfg);

//     //除了使能shipmode管脚和预放电控制管脚
//     stcGpioCfg.enDir = GpioDirOut;
//     stcGpioCfg.enDrv = GpioDrvL;
//     stcGpioCfg.enPu = GpioPuDisable;
//     stcGpioCfg.enPd = GpioPdDisable;
//     stcGpioCfg.enOD = GpioOdDisable;

//     Gpio_Init(TN_PREEN_PORT, TN_PREEN_PIN,&stcGpioCfg);
//     Gpio_Init(TN_SHSHIP_PORT, TN_SHSHIP_PIN,&stcGpioCfg);


// }

// void Mcu_PeripherWakeFromSleep(void)
// {

//     GPIOInit();
// }

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
    uint8_t enRet = 0;
    uint8_t u8i = 0, u8State;
    uint8_t IsSendLen = 0;
    uint8_t count = 10;

    I2C_SetFunc(TWI_I2C, I2cStart_En);
    while (1)
    {
        count = 200;
        while (0 == I2C_GetIrq(TWI_I2C))
        {
            bsp_DelayUS(50);
            if(count -- == 0)
            {
                enRet = 1;
                return enRet;
            }
        }
        u8State = I2C_GetState(TWI_I2C);
        switch (u8State)
        {
        case 0x08: ///< 已发送起始条件，将发送SLA+W
            I2C_ClearFunc(TWI_I2C, I2cStart_En);
            I2C_WriteByte(TWI_I2C, SlaveID);
            //IsSendLen = 0;
            count--;
            break;
        case 0x18:                          ///< 已发送SLA+W,并接收到ACK
            I2C_WriteByte(TWI_I2C, u8Addr); ///< 发送从机内存地址
            break;
        case 0x28:
#if (AFE_CHIP_SELECT == 3)
            I2C_SetFunc(TWI_I2C, I2cStart_En); ///< 发送重复起始条件

#else
            ///< 已发送数据，接收到ACK, 此处是已发送从机内存地址u8Addr并接收到ACK
            if (IsSendLen == 0)
            {
                I2C_WriteByte(TWI_I2C, u32Len);
                IsSendLen = 1;
            }
            else
            {
                I2C_SetFunc(TWI_I2C, I2cStart_En); ///< 发送重复起始条件
                IsSendLen = 0;
            }
#endif

            break;
        case 0x10: ///< 已发送重复起始条件
            I2C_ClearFunc(TWI_I2C, I2cStart_En);
            I2C_WriteByte(TWI_I2C, SlaveID | 0x01); ///< 发送SLA+R，开始从从机读取数据
            break;
        case 0x40: ///< 已发送SLA+R，并接收到ACK
            if (u32Len > 1)
            {
                I2C_SetFunc(TWI_I2C, I2cAck_En); ///< 使能主机应答功能
            }
            break;
        case 0x50: ///< 已接收数据字节，并已返回ACK信号
            pu8Data[u8i++] = I2C_ReadByte(TWI_I2C);
            //if (u8i == u32Len - 1)
            if (u8i == u32Len)
            {
                I2C_ClearFunc(TWI_I2C, I2cAck_En); ///< 已接收到倒数第二个字节，关闭ACK应答功能
            }
            break;
        case 0x58: ///< 已接收到最后一个数据，NACK已返回
            pu8Data[u8i++] = I2C_ReadByte(TWI_I2C);
            I2C_SetFunc(TWI_I2C, I2cStop_En); ///< 发送停止条件
            break;
        case 0x38:                             ///< 在发送地址或数据时，仲裁丢失
            I2C_SetFunc(TWI_I2C, I2cStart_En); ///< 当总线空闲时发起起始条件
            break;
        case 0x48:                             ///< 发送SLA+R后，收到一个NACK
            I2C_SetFunc(TWI_I2C, I2cStop_En);  ///< 发送停止条件
            I2C_SetFunc(TWI_I2C, I2cStart_En); ///< 发送起始条件
            break;
        default:
            I2C_ClearIrq(TWI_I2C); ///< 清除中断状态标志位
            enRet = 1;
            return enRet;
        }
        I2C_ClearIrq(TWI_I2C); ///< 清除中断状态标志位
        if (u8i == u32Len+1)     ///< 数据全部读取完成，跳出while循环
        {
            //crc 校验


            break;
        }
    }
    enRet = Ok;
    return enRet;
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
    en_result_t enRet = Error;
    uint8_t u8i = 0, u8State;  
    uint8_t count;  
    
    I2C_SetFunc(TWI_I2C, I2cStart_En);
    while (1)
    {
        count = 200;
        while (0 == I2C_GetIrq(TWI_I2C))
        {
            bsp_DelayUS(50);
            if(count -- == 0)
            {
                enRet = 1;
                return enRet;
            }
        }        
        u8State = I2C_GetState(TWI_I2C);
        switch (u8State)
        {
        case 0x08:
            I2C_ClearFunc(TWI_I2C, I2cStart_En);///< 已发送起始条件
            I2C_WriteByte(TWI_I2C, SlaveID); ///< 从设备地址发送
            break;
        case 0x18:                          ///< 已发送SLA+W，并接收到ACK
            I2C_WriteByte(TWI_I2C, u8Addr); ///< 从设备内存地址发送
            break;
        case 0x28:                                  ///< 上一次发送数据后接收到ACK
            I2C_WriteByte(TWI_I2C, pu8Data[u8i++]); ///< 继续发送数据
            break;
        case 0x20:                             ///< 上一次发送SLA+W后，收到NACK
        case 0x38:                             ///< 上一次在SLA+读或写时丢失仲裁
            I2C_SetFunc(TWI_I2C, I2cStart_En); ///< 当I2C总线空闲时发送起始条件
            break;
        case 0x30:                            ///< 已发送I2Cx_DATA中的数据，收到NACK，将传输一个STOP条件
            I2C_SetFunc(TWI_I2C, I2cStop_En); ///< 发送停止条件
            break;
        case 0x00:                            //总线错误
            I2C_ClearIrq(TWI_I2C);
            enRet = 1;
            return enRet;
        default:
            I2C_ClearIrq(TWI_I2C);
            enRet = 1;
            return enRet;
        }
        if (u8i > u32Len)
        {
            I2C_SetFunc(TWI_I2C, I2cStop_En); ///< 此顺序不能调换，出停止条件
            I2C_ClearIrq(TWI_I2C);
            break;
        }
        I2C_ClearIrq(TWI_I2C); ///< 清除中断状态标志位
    }
    enRet = Ok;
    return enRet;
}

#elif (MCU_LIB_SELECT == 2)

#define VREF_VAL (2048)
float ADC_Ratio = (3300.0) / 4095;

void MCU_SetNVOffset()
{
    if (VECT_OFFSET != 0 && __VTOR_PRESENT == 1)
    { //需要设置中断地址的偏移
        SCB->VTOR = 0x8000000 | VECT_OFFSET;
    }
}

void GPIOInit(void)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB | RCC_APB2_PERIPH_AFIO, ENABLE);

    //所有管脚都初始化为浮动输入
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input;
    GPIO_InitStructure.Pin = TN_DMOSFB_PIN;
    GPIO_InitPeripheral(TN_DMOSFB_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_CMOSFB_PIN;
    GPIO_InitPeripheral(TN_CMOSFB_PORT, &GPIO_InitStructure);




    //GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);


    // GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    // GPIO_InitStructure.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    // GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);
    // GPIO_InitStructure.Pin = GPIO_PIN_14|GPIO_PIN_15;
    // GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);
    //输出管脚配置
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

#if(PROJECT_ID == 2)    
    /// BKEN
    GPIO_InitStructure.Pin = TN_BK_EN_PIN;
    GPIO_InitPeripheral(TN_BK_EN_PORT, &GPIO_InitStructure);
  
    GPIO_InitStructure.Pin = TN_LED2_PIN;
    GPIO_InitPeripheral(TN_LED2_PORT, &GPIO_InitStructure);

#endif


    GPIO_InitStructure.Pin = TN_PREEN_PIN;
    GPIO_InitPeripheral(TN_PREEN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_VPRO_CON_PIN;
    GPIO_InitPeripheral(TN_VPRO_CON_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_485_TXEN_PIN;
    GPIO_InitPeripheral(TN_485_TXEN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_SHSHIP_PIN;
    GPIO_InitPeripheral(TN_SHSHIP_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_NTC0_POWER_PIN;
    GPIO_InitPeripheral(TN_NTC0_POWER_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_NTC1_POWER_PIN;
    GPIO_InitPeripheral(TN_NTC1_POWER_PORT, &GPIO_InitStructure);

    

    GPIO_InitStructure.Pin = TN_ONE_TX_PIN;
    GPIO_InitPeripheral(TN_ONE_TX_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_KILLME_PIN;
    GPIO_InitPeripheral(TN_KILLME_PORT, &GPIO_InitStructure);   

    GPIO_InitStructure.Pin = TN_KILLME_EN_PIN;
    GPIO_InitPeripheral(TN_KILLME_EN_PORT, &GPIO_InitStructure);
    //关闭使能
    GPIO_ResetBits(TN_KILLME_EN_PORT, TN_KILLME_EN_PIN);
     //下拉KILL
    GPIO_ResetBits(TN_KILLME_PORT,TN_KILLME_PIN);


    GPIO_ResetBits(TN_PREEN_PORT, TN_PREEN_PIN);
    GPIO_ResetBits(TN_VPRO_CON_PORT, TN_VPRO_CON_PIN);
    GPIO_ResetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    GPIO_ResetBits(TN_NTC1_POWER_PORT, TN_NTC1_POWER_PIN);
    GPIO_SetBits(TN_ONE_TX_PORT,TN_ONE_TX_PIN);
    


    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.Pin = TN_LED_PIN;
    GPIO_InitPeripheral(TN_LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(TN_LED_PORT,TN_LED_PIN);
  

    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    //GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_I2C2;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_Slew_Rate_High;

    GPIO_InitStructure.Pin = TN_I2C1_SCL_PIN|TN_I2C1_SDA_PIN;
    GPIO_InitPeripheral(TN_I2C1_SCL_PORT, &GPIO_InitStructure);

    //GPIO_InitStructure.Pin = TN_I2C1_SDA_PIN;
    //GPIO_InitPeripheral(TN_I2C1_SDA_PORT, &GPIO_InitStructure);

#if(USE_485_IF == 1)

    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

    GPIO_InitStructure.Pin = TN_485_TXEN_PIN;
    GPIO_InitPeripheral(TN_485_TXEN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_485_COMM_ON_PIN;
    GPIO_InitPeripheral(TN_485_COMM_ON_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_COMM_ON2_PIN;
    GPIO_InitPeripheral(TN_COMM_ON2_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(TN_485_TXEN_PORT, TN_485_TXEN_PIN);
    GPIO_SetBits(TN_485_COMM_ON_PORT, TN_485_COMM_ON_PIN);
    
    //485 中断
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input;
    GPIO_InitStructure.Pin = TN_485_INT_PIN;
    GPIO_InitPeripheral(TN_485_INT_PORT, &GPIO_InitStructure);



#endif

}

void MCU_GpioKillMeInit(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Current = GPIO_DC_4mA;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;

        ///< KILLME
    GPIO_InitStructure.Pin = TN_KILLME_PIN;
    GPIO_InitPeripheral(TN_KILLME_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = TN_KILLME_EN_PIN;
    GPIO_InitPeripheral(TN_KILLME_EN_PORT, &GPIO_InitStructure);

}

void SetSysClockToPLL(uint32_t freq, uint8_t src)
{
    uint32_t pllsrcclk;
    uint32_t pllsrc;
    uint32_t pllmul;
    uint32_t plldiv = RCC_PLLDIVCLK_DISABLE;
    uint32_t latency;
    uint32_t pclk1div, pclk2div;
    uint32_t msi_ready_flag = RESET;
    RCC_ClocksType RCC_ClockFreq;
    ErrorStatus HSEStartUpStatus;
    ErrorStatus HSIStartUpStatus;

    if (HSE_VALUE != 8000000)
    {
        /* HSE_VALUE == 8000000 is needed in this project! */
        while (1);
    }

    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration
     * -----------------------------*/

    if ((src == SYSCLK_PLLSRC_HSI)         || (src == SYSCLK_PLLSRC_HSIDIV2) 
     || (src == SYSCLK_PLLSRC_HSI_PLLDIV2) || (src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2))
    {
        /* Enable HSI */
        RCC_ConfigHsi(RCC_HSI_ENABLE);

        /* Wait till HSI is ready */
        HSIStartUpStatus = RCC_WaitHsiStable();

        if (HSIStartUpStatus != SUCCESS)
        {
            /* If HSI fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            while (1);
        }

        if ((src == SYSCLK_PLLSRC_HSIDIV2) || (src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2))
        {
            pllsrc = RCC_PLL_HSI_PRE_DIV2;
            pllsrcclk = HSI_VALUE/2;

            if(src == SYSCLK_PLLSRC_HSIDIV2_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSI_VALUE/4;
            }
        } else if ((src == SYSCLK_PLLSRC_HSI) || (src == SYSCLK_PLLSRC_HSI_PLLDIV2))
        {
            pllsrc = RCC_PLL_HSI_PRE_DIV1;
            pllsrcclk = HSI_VALUE;

            if(src == SYSCLK_PLLSRC_HSI_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSI_VALUE/2;
            }
        }

    } else if ((src == SYSCLK_PLLSRC_HSE)         || (src == SYSCLK_PLLSRC_HSEDIV2) 
            || (src == SYSCLK_PLLSRC_HSE_PLLDIV2) || (src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2))
    {
        /* Enable HSE */
        RCC_ConfigHse(RCC_HSE_ENABLE);

        /* Wait till HSE is ready */
        HSEStartUpStatus = RCC_WaitHseStable();

        if (HSEStartUpStatus != SUCCESS)
        {
            /* If HSE fails to start-up, the application will have wrong clock
               configuration. User can add here some code to deal with this
               error */

            /* Go to infinite loop */
            return;
        }

        if ((src == SYSCLK_PLLSRC_HSEDIV2) || (src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2))
        {
            pllsrc = RCC_PLL_SRC_HSE_DIV2;
            pllsrcclk = HSE_VALUE/2;

            if(src == SYSCLK_PLLSRC_HSEDIV2_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSE_VALUE/4;
            }
        } else if ((src == SYSCLK_PLLSRC_HSE) || (src == SYSCLK_PLLSRC_HSE_PLLDIV2))
        {
            pllsrc = RCC_PLL_SRC_HSE_DIV1;
            pllsrcclk = HSE_VALUE;

            if(src == SYSCLK_PLLSRC_HSE_PLLDIV2)
            {
                plldiv = RCC_PLLDIVCLK_ENABLE;
                pllsrcclk = HSE_VALUE/2;
            }
        }
    }

    latency = (freq/32000000);
    
    if(freq > 54000000)
    {
        pclk1div = RCC_HCLK_DIV4;
        pclk2div = RCC_HCLK_DIV2;
    }
    else
    {
        if(freq > 27000000)
        {
            pclk1div = RCC_HCLK_DIV2;
            pclk2div = RCC_HCLK_DIV1;
        }
        else
        {
            pclk1div = RCC_HCLK_DIV1;
            pclk2div = RCC_HCLK_DIV1;
        }
    }
    
    if(((freq % pllsrcclk) == 0) && ((freq / pllsrcclk) >= 2) && ((freq / pllsrcclk) <= 32))
    {
        pllmul = (freq / pllsrcclk);
        if(pllmul <= 16)
        {
            pllmul = ((pllmul - 2) << 18);
        }
        else
        {
            pllmul = (((pllmul - 17) << 18) | (1 << 27));
        }
    }
    else
    {
        while(1);
    }

    /* Cheak if MSI is Ready */
    if(RESET == RCC_GetFlagStatus(RCC_CTRLSTS_FLAG_MSIRD))
    {
        /* Enable MSI and Config Clock */
        RCC_ConfigMsi(RCC_MSI_ENABLE, RCC_MSI_RANGE_4M);
        /* Waits for MSI start-up */
        while(SUCCESS != RCC_WaitMsiStable());

        msi_ready_flag = SET;
    }

    /* Select MSI as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_MSI);

    FLASH_SetLatency(latency);

    /* HCLK = SYSCLK */
    RCC_ConfigHclk(RCC_SYSCLK_DIV1);

    /* PCLK2 = HCLK */
    RCC_ConfigPclk2(pclk2div);

    /* PCLK1 = HCLK */
    RCC_ConfigPclk1(pclk1div);

    /* Disable PLL */
    RCC_EnablePll(DISABLE);

    RCC_ConfigPll(pllsrc, pllmul, plldiv);

    /* Enable PLL */
    RCC_EnablePll(ENABLE);

    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_CTRL_FLAG_PLLRDF) == RESET);

    /* Select PLL as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_GetSysclkSrc() != 0x0C);

    if(msi_ready_flag == SET)
    {
        /* MSI oscillator OFF */
        RCC_ConfigMsi(RCC_MSI_DISABLE, RCC_MSI_RANGE_4M);
    }
}

void Clk_Config(void)
{
    //n32l406x 时钟配置
    SetSysClockToPLL(64000000, SYSCLK_PLLSRC_HSE);
}

void Iwdt_Config(void)
{
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV256); //IWDG_PRESCALER_DIV256

    /* Set counter reload value to obtain 250ms IWDG TimeOut.
       Counter Reload Value = 1000
                            f = 40k/256 = 156.25
                            period = 3000*1/156.25 = 4096*0.0064 = 19.2s                            
     */
    IWDG_CntReload(3000); //LSI = 40K
    /* Reload IWDG counter */
    IWDG_ReloadKey();

    /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
}

void Iwdt_Start(void)
{
    IWDG_Enable();
}

void Iwdt_Feed(void)
{
    IWDG_ReloadKey();
}

//-----------------------------RTC----------------------------------------------
void RTC_ConfigInit(void)
{
    RTC_DateAndTimeDefaultVale();
    RTC_CLKSourceConfig(3, 0, 1);
    RTC_PrescalerConfig();

    RTC_DateRegulate();
    RTC_TimeRegulate();

    WakeUpClockSelect(1);
    /* wake up timer value */
    //RTC_SetWakeUpCounter(25000);

    EXTI20_RTCWKUP_Configuration(ENABLE);
    /* Enable the RTC Wakeup Interrupt */
    RTC_ConfigInt(RTC_INT_WUT, ENABLE);
    RTC_EnableWakeUp(ENABLE);
    RTC_SetWakeUpCounter(25000);
}

uint16_t rtcCount = 0;
void RTC_WKUP_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_INT_WUT) != RESET)
    {
        RTC_ClrIntPendingBit(RTC_INT_WUT);
        EXTI_ClrITPendBit(EXTI_LINE20);
        
        //RTC 唤醒之后，需要根据进入中断的类型
        if(flagIntEnterType == 0x02)
        {
            //深度休眠
            //只清楚WDT，然后继续休眠
            Iwdt_Feed();
            SCB->SCR |= 0x02;
        }else
        if(flagIntEnterType == 0x01)
        {
            //唤醒之后退出中断，继续跑
            //SCB->SCR &= ~0x02;
        }

        FlagRtcWakeUp = 0x01;
        rtcCount++;
    }
}

uint8_t Rtc_GetWakeUpFlag(void)
{
    return FlagRtcWakeUp;
}

void Rtc_ClrWakeUpFlag(void)
{
    FlagRtcWakeUp = 0;
}

uint16_t Rtc_GetCount(){
    return rtcCount;
}

void Rtc_SetCount(uint16_t val)
{
    rtcCount = 0;
}

void RTC_AlarmITEnable(void)
{
    RTC_ConfigInt(RTC_INT_WUT,ENABLE);
}

void RTC_AlarmITDisable(void)
{
    RTC_ConfigInt(RTC_INT_WUT,DISABLE);
}

//-----------------------------END RTC----------------------------------------------



///< ADC 采样端口初始化
void ADC_PortInit(void)
{
}

void ADC_DeConfig(void)
{
    ADC_DeInit(ADC);
    ADC_Enable(ADC, DISABLE);
}

uint16_t ADC_GetChannleVal(uint8_t ADC_Channel)
{
    uint16_t dat;

    ADC_ConfigRegularChannel(ADC, ADC_Channel, 1, ADC_SAMP_TIME_55CYCLES5);
    /* Start ADC Software Conversion */
    ADC_EnableSoftwareStartConv(ADC, ENABLE);
    while (ADC_GetFlagStatus(ADC, ADC_FLAG_ENDC) == 0)
    {
    }
    ADC_ClearFlag(ADC, ADC_FLAG_ENDC);
    ADC_ClearFlag(ADC, ADC_FLAG_STR);
    dat = ADC_GetDat(ADC);
    return dat;
}

void ADC_Config(void)
{
    ADC_InitType ADC_InitStructure;
    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    //RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8); //selsect HSE as RCC ADC1M CLK Source

    /* ADC configuration ------------------------------------------------------*/
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_Init(ADC, &ADC_InitStructure);
    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);
    /* Check ADC Ready */
    while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC);
    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC))
        ;
}

void ADC_GetMcuAdcInfo(uint16_t *buf, uint16_t *mosTemp, uint16_t *balTemp, uint16_t *preDsgCur)
{

    //开启温探power
    GPIO_SetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
    bsp_DelayMS(10);

    *buf = ADC_GetChannleVal(ADC_CH_0);             //PMATCH,PB12 AdcExInputCH19
    *(buf + 1) = ADC_GetChannleVal(ADC_CH_4);       //balance temp ,PA07 ，NTC1
    *(buf + 2) = ADC_GetChannleVal(ADC_CH_5);       //mos temp ,PB00 ，NTC
    //*(buf + 3) = ADC_GetChannleVal(ADC_CH_VREFINT); //2.048V
    *(buf + 4) = ADC_GetChannleVal(ADC_CH_10);      //PB1 ,pre_Isen_p
    *(buf + 5) = 0;                                 // ADC_GetChannleVal(ADC_CH_4);   //PB1 ,pre_Isen_n

    //use vrefint calibrate the VDD as vref
    //ADC_Ratio = (float)VREF_VAL / *(buf + 3);
    *mosTemp = CalcuTemp((uint16_t)(*(buf + 2) * ADC_Ratio));
    *balTemp = CalcuTemp((uint16_t)(*(buf + 1) * ADC_Ratio));
    
    *preDsgCur = (uint16_t)(*(buf + 4) * ADC_Ratio); 
    //关闭 temperature power
    GPIO_ResetBits(TN_NTC0_POWER_PORT, TN_NTC0_POWER_PIN);
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
#define SCK_H() MCU_GPIO_SetBit(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN)
#define SCK_L() MCU_GPIO_ClrBit(TN_I2C1_SCL_PORT, TN_I2C1_SCL_PIN)

#define SDA_H() MCU_GPIO_SetBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN)
#define SDA_L() MCU_GPIO_ClrBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN)


static uint8_t READ_SDA(void)
{
    uint8_t bit = 0;
    
    //gpio_init(GPIOE,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    //Hal_DelayUS(2);
    
    //bsp_DelayUS(5);
    bit = MCU_GPIO_GetBit(TN_I2C1_SDA_PORT, TN_I2C1_SDA_PIN);
    //gpio_init(GPIOE, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,GPIO_PIN_5);
    return bit;
}

static void TWI_IIC_Start(void)
{
    
    // SCK_H();
    // SDA_H();
    // bsp_DelayUS(30);
    // SDA_L();
    // bsp_DelayUS(30);
    // SCK_L();
    // SCK_H();
    // SDA_L();
    // bsp_DelayUS(10);
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
    bsp_DelayUS(10);
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
        bsp_DelayUS(10);
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

static uint8_t TWI_WriteRegByte(uint8_t addr, uint8_t byte)
{
    uint8_t retCode = 0;
    TWI_IIC_Start();
    TWI_IIC_WriteByte(addr);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
    }
    TWI_IIC_WriteByte(byte);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
    }
    TWI_IIC_Stop();
    return retCode;
}

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
        //SEGGER_RTT_printf(0,"ACK ERROR1\n");
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR2\n");
    }
#if(AFE_CHIP_SELECT == 3)
#else
    TWI_IIC_WriteByte((uint8_t)u32Len);
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR3\n");
    }
#endif
    TWI_IIC_Start();
    //read
    TWI_IIC_WriteByte((SlaveID|0x01));//(SlaveID|0x01)
    if (TWI_IIC_Ack())
    {
        retCode = 1;
        //SEGGER_RTT_printf(0,"ACK ERROR4\n");
    }

    for(i = 0;i<u32Len;i++)
    {
        *(pu8Data+i) = TWI_IIC_ReadByte();
        //SEGGER_RTT_printf(0,"%02x ",*(pu8Data+i));
        //send ack
        TWI_IIC_SendAck(1);

    }
    //SEGGER_RTT_printf(0,"\n");
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
        //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR1\n");
        retCode = 1;
    }
    TWI_IIC_WriteByte(u8Addr);
    if (TWI_IIC_Ack())
    {
        //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR2\n");
        retCode = 1;
    }

    for(i = 0;i<u32Len;i++)
    {
        TWI_IIC_WriteByte(*(pu8Data+i));
        if (TWI_IIC_Ack())
        {
            //SEGGER_RTT_printf(0,"I2C_MasterWriteData ACK ERROR3\n");
            retCode = 1;
            break;
        }
    }
    TWI_IIC_Stop();
    return retCode;
}

#endif