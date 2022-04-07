#include "includes.h"
#if (MCU_LIB_SELECT == 2)

#if (PROJECT_ID == 3)


CanRxMessage CAN_RxMessage;

pf_CANRxCallback canRxCallback;

void BxCanPortInit()
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configures CAN IOs */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO | RCC_APB2_PERIPH_GPIOA|RCC_APB2_PERIPH_GPIOB, ENABLE);

    //BKEN
    GPIO_InitStructure.Pin       = GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_No_Pull;
    //GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_CAN;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    //commOn2
    GPIO_InitStructure.Pin       = GPIO_PIN_12;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_PIN_15);
    GPIO_SetBits(GPIOA,GPIO_PIN_12);


    /* Configure CAN RX pin */
    GPIO_InitStructure.Pin       = GPIO_PIN_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Input;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_CAN;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    /* Configure CAN TX pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief  CAN Interrupt Configures .
 */
void CAN_NVIC_Config(void)
{
    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures CAN.
 * @param CAN_BaudRate 10Kbit/s ~ 1Mbit/s
 */
void BxCanConfig(pf_CANRxCallback callback,uint16_t *idListTab,uint8_t idListNum)
{
    CAN_InitType CAN_InitStructure;
    CAN_FilterInitType CAN_FilterInitStructure;

    /* Configure CAN */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN, ENABLE);

    /* CAN register init */
    CAN_DeInit(CAN);

    /* Struct init*/
    CAN_InitStruct(&CAN_InitStructure);

    /* CAN cell init */
    CAN_InitStructure.TTCM          = DISABLE;
    CAN_InitStructure.ABOM          = ENABLE;
    CAN_InitStructure.AWKUM         = DISABLE;
    CAN_InitStructure.NART          = DISABLE;
    CAN_InitStructure.RFLM          = DISABLE;
    CAN_InitStructure.TXFP          = ENABLE;
    CAN_InitStructure.OperatingMode = CAN_Normal_Mode;
    CAN_InitStructure.RSJW          = CAN_BIT_RSJW;
    CAN_InitStructure.TBS1          = CAN_BIT_BS1;
    CAN_InitStructure.TBS2          = CAN_BIT_BS2;
    CAN_InitStructure.BaudRatePrescaler = CAN_BAUDRATEPRESCALER;

    /*Initializes the CAN */
    CAN_Init(CAN, &CAN_InitStructure);

    if(idListNum<3)
    {
        /* CAN filter init */
        CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
        CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdListMode;
        CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_16bitScale;
        CAN_FilterInitStructure.Filter_HighId         = *idListTab;//0x06f2;//0x
        CAN_FilterInitStructure.Filter_LowId          = *(idListTab+1);//0x045a;//升级id
        CAN_FilterInitStructure.FilterMask_HighId     = 0x0000;
        CAN_FilterInitStructure.FilterMask_LowId      = 0x0000;
        CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
        CAN_FilterInitStructure.Filter_Act            = ENABLE;
        CAN_InitFilter(&CAN_FilterInitStructure);

    }else
    if (idListNum<5)
    {
        /* CAN filter init */
        CAN_FilterInitStructure.Filter_Num            = CAN_FILTERNUM0;
        CAN_FilterInitStructure.Filter_Mode           = CAN_Filter_IdListMode;
        CAN_FilterInitStructure.Filter_Scale          = CAN_Filter_16bitScale;
        CAN_FilterInitStructure.Filter_HighId         = *(idListTab+0);//0x06f2;//0x
        CAN_FilterInitStructure.Filter_LowId          = *(idListTab+1);//0x045a;//升级id
        CAN_FilterInitStructure.FilterMask_HighId     = *(idListTab+2);
        CAN_FilterInitStructure.FilterMask_LowId      = *(idListTab+3);
        CAN_FilterInitStructure.Filter_FIFOAssignment = CAN_FIFO0;
        CAN_FilterInitStructure.Filter_Act            = ENABLE;
        CAN_InitFilter(&CAN_FilterInitStructure);
    }
    
    
    CAN_INTConfig(CAN, CAN_INT_FMP0, ENABLE);

    CAN_NVIC_Config();

    canRxCallback = callback;
}

/**
 * @brief  CAN Transmit Message.
 * @param  CAN
 * @param  TxMessage CAN_TxMessage
 * @param  StdId
 * @param  ExtId
 * @param  IDE
 * @param  RTR
 * @param  DLC
 * @param  Data0~7
 * @return The number of the mailbox that is used for transmission or CAN_TxSTS_NoMailBox if there is no empty mailbox.
 */
uint8_t CANTxMessage(CAN_Module* CANx,
                     CanTxMessage* TxMessage)
{
    return CAN_TransmitMessage(CANx, TxMessage);
    /* ******** */
}

/**
 * @brief  This function handles CAN RX0 Handler.
 */
void CAN_RX0_IRQHandler(void)
{
    CanRxMessage canRxMsg;

    CAN_ReceiveMessage(CAN, 0, &canRxMsg);
    canRxCallback(canRxMsg);
    
}



#endif


#endif