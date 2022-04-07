/********************************************************************************
Copyright (C), TianNeng Power. Ltd.
Author: 	hxbao
Version: 	V0.0
Date: 		2020/10/16
History:
	V0.0		2021/9/16		 Preliminary
********************************************************************************/
#include "includes.h"


#if(MCU_LIB_SELECT == 1)
void SPI2_GpioInit(void)
{
	stc_gpio_cfg_t stcGpioCfg;

	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enDrv = GpioDrvH;

    
	Gpio_Init(TN_SPI_CS_PORT, TN_SPI_CS_PIN, &stcGpioCfg);
	Gpio_Init(TN_SPI_CLK_PORT, TN_SPI_CLK_PIN, &stcGpioCfg);
	Gpio_Init(TN_SPI_MOSI_PORT, TN_SPI_MOSI_PIN, &stcGpioCfg);
	Gpio_SetAfMode(TN_SPI_CLK_PORT,TN_SPI_CLK_PIN,GpioAf1);
	Gpio_SetAfMode(TN_SPI_MOSI_PORT,TN_SPI_MOSI_PIN,GpioAf1);

	stcGpioCfg.enDir = GpioDirIn;
    stcGpioCfg.enDrv = GpioDrvL;
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    stcGpioCfg.enOD = GpioOdEnable;
	Gpio_Init(TN_SPI_MISO_PORT, TN_SPI_MISO_PIN, &stcGpioCfg);
	Gpio_SetAfMode(TN_SPI_MISO_PORT,TN_SPI_MISO_PIN,GpioAf1);
}

void SPI2_Init(uint16_t prescaler)		//prescaler = 4	 Or  256
{
    stc_spi_cfg_t spi2_init_struct;

	spi2_init_struct.enSpiMode = SpiMskMaster;						  						    
	spi2_init_struct.enCPOL = SpiMskcpolhigh;     					 	
	spi2_init_struct.enCPHA = SpiMskCphasecond;   										
	spi2_init_struct.enPclkDiv = prescaler;					
	 						
	Spi_Init(M0P_SPI1, &spi2_init_struct);
}

uint8_t SPI2_WriteReadByte(uint8_t TxData, uint8_t *RdData)
{      
	uint32_t retry = 0;  
	while (Spi_GetStatus(M0P_SPI1, SpiTxe) == FALSE) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry > 1000)
			return 0xff;
	} 
	
	Spi_SendData(M0P_SPI1, TxData); //通过外设SPIx发送一个数据
		
	while (Spi_GetStatus(M0P_SPI1, SpiRxne) == FALSE) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry > 1000)
			return 0xff;
	}
	
	*RdData =  Spi_ReceiveData(M0P_SPI1); //返回通过SPIx最近接收的数据
	return 0;
}

#elif(MCU_LIB_SELECT == 2) 

/*****************************************************************************
 函 数 名  : SPI2_GpioInit
 功能描述  : SPI GPIO初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void SPI2_GpioInit(void)
{
	GPIO_InitType GPIO_InitStructure;

	//时钟初始化
    RCC_EnableAPB1PeriphClk(RCC_APB2_PERIPH_GPIOB,ENABLE);
    
    GPIO_InitStruct(&GPIO_InitStructure);

    GPIO_InitStructure.Pin            = TN_SPI_CLK_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI2;
    GPIO_InitPeripheral(TN_SPI_CLK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = TN_SPI_MOSI_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI2;
    GPIO_InitPeripheral(TN_SPI_MOSI_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = TN_SPI_MISO_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI2;
    GPIO_InitPeripheral(TN_SPI_MISO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = TN_SPI_CS_PIN;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_SPI2;
    GPIO_InitPeripheral(TN_SPI_CS_PORT, &GPIO_InitStructure);
}

/*****************************************************************************
 函 数 名  : SPI2_Init
 功能描述  : SPI控制器初始化
 输入参数  : uint16_t prescaler  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
void SPI2_Init(uint16_t prescaler)		//prescaler = 4	 Or  256
{
    SPI_InitType spi2_init_struct;

	spi2_init_struct.DataDirection = SPI_DIR_DOUBLELINE_FULLDUPLEX;  	
	spi2_init_struct.SpiMode = SPI_MODE_MASTER;						  	
	spi2_init_struct.DataLen = SPI_DATA_SIZE_8BITS; 					    
	spi2_init_struct.CLKPOL = SPI_CLKPOL_HIGH;     					 	
	spi2_init_struct.CLKPHA = SPI_CLKPHA_SECOND_EDGE;   							
	spi2_init_struct.NSS = SPI_NSS_SOFT; 				
	spi2_init_struct.BaudRatePres = prescaler;					
	spi2_init_struct.FirstBit = SPI_FB_MSB;     						
	spi2_init_struct.CRCPoly  = 7;
	SPI_Enable(SPI2, DISABLE); //去使能SPI外设
	SPI_Init(SPI2, &spi2_init_struct);
	SPI_Enable(SPI2, ENABLE); //使能SPI外设
}

/*****************************************************************************
 函 数 名  : SPI2_WriteReadByte
 功能描述  : SPI单字节访问
 输入参数  : uint8_t TxData   
             uint8_t *RdData  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2021年8月2日
    作    者   : 武俊杰
    修改内容   : 新生成函数

*****************************************************************************/
uint8_t SPI2_WriteReadByte(uint8_t TxData, uint8_t *RdData)
{      
	uint32_t retry = 0;  
	while (SPI_I2S_GetStatus(SPI2, SPI_I2S_TE_FLAG) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
		retry++;
		if(retry > 1000)
			return 0xff;
	} 
	
	SPI_I2S_TransmitData(SPI2, TxData); //通过外设SPIx发送一个数据
		
	while (SPI_I2S_GetStatus(SPI2, SPI_I2S_RNE_FLAG) == RESET) //检查指定的SPI标志位设置与否:接受缓存非空标志位
	{
		retry++;
		if(retry > 1000)
			return 0xff;
	}
	
	*RdData =  SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据
	return 0;
}

#endif