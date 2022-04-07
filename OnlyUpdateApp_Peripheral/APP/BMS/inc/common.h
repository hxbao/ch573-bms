#ifndef COMMON_H
#define COMMON_H

#include "includes.h"


#if(MCU_LIB_SELECT ==1)
///< TN GPIO DEFINE
//LED
#define TN_LED_PORT       GpioPortB
#define TN_LED_PIN        GpioPin10



///< I2C SH309
#define TN_I2C1_SCL_PORT    GpioPortB
#define TN_I2C1_SCL_PIN     GpioPin13
#define TN_I2C1_SDA_PORT    GpioPortB
#define TN_I2C1_SDA_PIN     GpioPin14

///< CHG_DET                
#define TN_CHG_DET_PORT      GpioPortA
#define TN_CHG_DET_PIN       GpioPin0

///< WEAK_UP
#define TN_WAKE_UP_PORT      GpioPortA
#define TN_WAKE_UP_PIN       GpioPin1

#define TN_LOAD_DET_PORT      GpioPortA
#define TN_LOAD_DET_PIN       GpioPin11

#if(PROJECT_ID == 2)
#define TN_AFE_ON_PORT      GpioPortA
#define TN_AFE_ON_PIN       GpioPin1

#define TN_LED2_PORT      GpioPortA
#define TN_LED2_PIN       GpioPin5

#define TN_BK_EN_PIN   GpioPin15
#define TN_BK_EN_PORT  GpioPortB

#define TN_CTRL_AFE_PIN GpioPin12
#define TN_CTRL_AFE_PORT GpioPortA

#endif

///< ACC
#define TN_ACC_PORT      GpioPortA
#define TN_ACC_PIN       GpioPin2

///<SH_INT,RX 中断
#define TN_SHINT_PORT      GpioPortA
#define TN_SHINT_PIN       GpioPin10


///< TXD0
#define TN_UART0_TXD_PORT      GpioPortA
#define TN_UART0_TXD_PIN       GpioPin9

///< RXD0
#define TN_UART0_RXD_PORT      GpioPortA
#define TN_UART0_RXD_PIN      GpioPin10

///< VPRO_CON
#define TN_VPRO_CON_PORT      GpioPortB
#define TN_VPRO_CON_PIN       GpioPin5

///< SH_SHIP
#define TN_SHSHIP_PORT       GpioPortA
#define TN_SHSHIP_PIN       GpioPin6

///<NTC0 POWER
#define TN_NTC0_POWER_PORT      GpioPortB
#define TN_NTC0_POWER_PIN       GpioPin7

///<NTC1 POWER
#define TN_NTC1_POWER_PORT      GpioPortB
#define TN_NTC1_POWER_PIN       GpioPin7

///NTC1 ADC Input
#define TN_NTC1_ADIN_PORT    GpioPortA
#define TN_NTC1_ADIN_PIN     GpioPin3
///NTC0 ADC Input
#define TN_NTC0_ADIN_PORT    GpioPortA
#define TN_NTC0_ADIN_PIN     GpioPin4


///<ONE_TX
#define TN_ONE_TX_PORT      GpioPortA
#define TN_ONE_TX_PIN       GpioPin9

///<ONE_RX
#define TN_ONE_RX_PORT      GpioPortA
#define TN_ONE_RX_PIN       GpioPin10

///<AlARM
#define TN_ALARM_PORT      GpioPortB
#define TN_ALARM_PIN       GpioPin12


///PRE_ISEN_P
#define TN_PRE_ISEN_P_PORT      GpioPortB
#define TN_PRE_ISEN_P_PIN       GpioPin1

///PRE_ISEN_N
#define TN_PRE_ISEN_N_PORT      GpioPortB
#define TN_PRE_ISEN_N_PIN       GpioPin2

#if(PROJECT_ID == 2)

#define TN_PREEN_PORT          GpioPortA
#define TN_PREEN_PIN           GpioPin8
#else
///PREEN
#define TN_PREEN_PORT          GpioPortA
#define TN_PREEN_PIN       GpioPin5
#endif

///<KILLME
#define TN_KILLME_PORT      GpioPortA
#define TN_KILLME_PIN       GpioPin15

///<KILLME_EN
#define TN_KILLME_EN_PORT      GpioPortB
#define TN_KILLME_EN_PIN       GpioPin4

///<DMOS_FB
#define TN_DMOSFB_PORT     GpioPortA
#define TN_DMOSFB_PIN      GpioPin7

///<CMOS_FB
#define TN_CMOSFB_PORT      GpioPortB
#define TN_CMOSFB_PIN       GpioPin0

#define TN_PF1_PORT      GpioPortF                                       
#define TN_PF1_PIN       GpioPin1

#define TN_SPI_CS_PIN   GpioPin12
#define TN_SPI_CS_PORT  GpioPortB

#define TN_SPI_CLK_PIN   GpioPin13
#define TN_SPI_CLK_PORT  GpioPortB

#define TN_SPI_MISO_PIN   GpioPin14
#define TN_SPI_MISO_PORT  GpioPortB

#define TN_SPI_MOSI_PIN   GpioPin15
#define TN_SPI_MOSI_PORT  GpioPortB

#define TN_DVC_ALERT_PIN GpioPin12
#define TN_DVC_ALERT_PORT GpioPortA

#define TN_DVC_ALERT_PIN GpioPin12
#define TN_DVC_ALERT_PORT GpioPortA

#define TN_DVC_ALERT_PIN GpioPin12
#define TN_DVC_ALERT_PORT GpioPortA

#define TN_DVC_WAUP_PIN GpioPin12
#define TN_DVC_WAUP_PORT GpioPortA

#if(USE_485_IF == 1)

#define TN_485_COMM_ON_PORT GpioPortA
#define TN_485_COMM_ON_PIN GpioPin11

#define TN_COMM_ON2_PORT GpioPortA
#define TN_COMM_ON2_PIN GpioPin12

#define TN_485_TXEN_PORT  GpioPortB
#define TN_485_TXEN_PIN   GpioPin3

#define TN_485_INT_PORT GpioPortB
#define TN_485_INT_PIN GpioPin6

#endif


#elif(MCU_LIB_SELECT == 2)

#define TN_LED_PORT       GPIOB
#define TN_LED_PIN        GPIO_PIN_10



#define TN_PF1_PORT      GPIOF                                        
#define TN_PF1_PIN       GPIO_PIN_1



///< I2C SH309

#define TN_I2C1_SCL_PORT    GPIOB
#define TN_I2C1_SCL_PIN     GPIO_PIN_13



#define TN_I2C1_SDA_PORT    GPIOB
#define TN_I2C1_SDA_PIN     GPIO_PIN_14



///< CHG_DET                
#define TN_CHG_DET_PORT      GPIOA
#define TN_CHG_DET_PIN       GPIO_PIN_0



///< WEAK_UP
#define TN_WAKE_UP_PORT      GPIOA
#define TN_WAKE_UP_PIN       GPIO_PIN_1


///< ACC
#define TN_ACC_PORT      GPIOA
#define TN_ACC_PIN       GPIO_PIN_2


#if(PROJECT_ID == 2)
#define TN_AFE_ON_PORT      GPIOA
#define TN_AFE_ON_PIN       GPIO_PIN_1

#define TN_LED2_PORT      GPIOA
#define TN_LED2_PIN       GPIO_PIN_5

#define TN_BK_EN_PIN   GPIO_PIN_15
#define TN_BK_EN_PORT  GPIOB

#define TN_CTRL_AFE_PIN GPIO_PIN_12
#define TN_CTRL_AFE_PORT GPIOA

#endif

///<SH_INT
#define TN_SHINT_PORT      GPIOA
#define TN_SHINT_PIN       GPIO_PIN_10



///< TXD0
#define TN_UART0_TXD_PORT      GPIOA
#define TN_UART0_TXD_PIN       GPIO_PIN_9

///< RXD0
#define TN_UART0_RXD_PORT      GPIOA
#define TN_UART0_RXD_PIN      GPIO_PIN_10

///< VPRO_CON
#define TN_VPRO_CON_PORT      GPIOB
#define TN_VPRO_CON_PIN       GPIO_PIN_5

///< SH_SHIP
#define TN_SHSHIP_PORT       GPIOA
#define TN_SHSHIP_PIN       GPIO_PIN_6

///<NTC0 POWER
#define TN_NTC0_POWER_PORT      GPIOB
#define TN_NTC0_POWER_PIN       GPIO_PIN_7

///<NTC1 POWER
#define TN_NTC1_POWER_PORT      GPIOB
#define TN_NTC1_POWER_PIN       GPIO_PIN_7

///NTC1 ADC Input
#define TN_NTC1_ADIN_PORT    GPIOA
#define TN_NTC1_ADIN_PIN     GPIO_PIN_3
///NTC0 ADC Input
#define TN_NTC0_ADIN_PORT    GPIOA
#define TN_NTC0_ADIN_PIN     GPIO_PIN_4


///<ONE_TX
#define TN_ONE_TX_PORT      GPIOA
#define TN_ONE_TX_PIN       GPIO_PIN_9

///<ONE_RX
#define TN_ONE_RX_PORT      GPIOA
#define TN_ONE_RX_PIN       GPIO_PIN_10

///<AlARM
#define TN_ALARM_PORT      GPIOB
#define TN_ALARM_PIN       GPIO_PIN_12


///PRE_ISEN_P
#define TN_PRE_ISEN_P_PORT      GPIOB
#define TN_PRE_ISEN_P_PIN       GPIO_PIN_1

///PRE_ISEN_N
#define TN_PRE_ISEN_N_PORT      GPIOB
#define TN_PRE_ISEN_N_PIN       GPIO_PIN_2


#if(PROJECT_ID == 2)

#define TN_PREEN_PORT          GPIOA
#define TN_PREEN_PIN           GPIO_PIN_8
#elif(PROJECT_ID == 3)

#define TN_PREEN_PORT          GPIOA
#define TN_PREEN_PIN           GPIO_PIN_8

#define TN_LOAD_DET_PORT      GPIOA
#define TN_LOAD_DET_PIN       GPIO_PIN_11

#else
///PREEN
#define TN_PREEN_PORT          GPIOA
#define TN_PREEN_PIN       GPIO_PIN_5


#define TN_LOAD_DET_PORT      GPIOA
#define TN_LOAD_DET_PIN       GPIO_PIN_11

#endif

///<KILLME
#define TN_KILLME_PORT      GPIOA
#define TN_KILLME_PIN       GPIO_PIN_15

///<KILLME_EN
#define TN_KILLME_EN_PORT      GPIOB
#define TN_KILLME_EN_PIN       GPIO_PIN_4

///<DMOS_FB
#define TN_DMOSFB_PORT     GPIOA
#define TN_DMOSFB_PIN      GPIO_PIN_7

///<CMOS_FB
#define TN_CMOSFB_PORT      GPIOB
#define TN_CMOSFB_PIN       GPIO_PIN_0

#define TN_485_TXEN_PORT  GPIOB
#define TN_485_TXEN_PIN   GPIO_PIN_3

#define TN_SPI_CS_PIN   GPIO_PIN_12
#define TN_SPI_CS_PORT  GPIOB

#define TN_SPI_CLK_PIN   GPIO_PIN_13
#define TN_SPI_CLK_PORT  GPIOB

#define TN_SPI_MISO_PIN   GPIO_PIN_14
#define TN_SPI_MISO_PORT  GPIOB

#define TN_SPI_MOSI_PIN   GPIO_PIN_15
#define TN_SPI_MOSI_PORT  GPIOB

#define TN_DVC_ALERT_PIN GPIO_PIN_12
#define TN_DVC_ALERT_PORT GPIOA

#define TN_DVC_ALERT_PIN GPIO_PIN_12
#define TN_DVC_ALERT_PORT GPIOA

#define TN_DVC_ALERT_PIN GPIO_PIN_12
#define TN_DVC_ALERT_PORT GPIOA

#define TN_DVC_WAUP_PIN GPIO_PIN_11
#define TN_DVC_WAUP_PORT GPIOA

#if(USE_485_IF == 1)

#define TN_485_COMM_ON_PORT GPIOA
#define TN_485_COMM_ON_PIN GPIO_PIN_11

#define TN_COMM_ON2_PORT GPIOA
#define TN_COMM_ON2_PIN GPIO_PIN_12

#define TN_485_TXEN_PORT  GPIOB
#define TN_485_TXEN_PIN   GPIO_PIN_3

#define TN_485_INT_PORT GPIOB
#define TN_485_INT_PIN GPIO_PIN_6

#endif



#endif

#if(MCU_LIB_SELECT ==1)
extern uint8_t CommonRam[512];
extern uint8_t CommonRam2[256];
#elif(MCU_LIB_SELECT ==2)
extern uint8_t CommonRam[256];
extern uint8_t CommonRam2[256];
#endif


extern const float NTC103AT[151];


uint16_t CalcuTemp(uint16_t Vntc);
uint16_t CalcuTemp1(float Rntc);
float CalaRntcFromTemp(uint16_t temp);
void MyMemcpy(uint8_t *dst,uint8_t *src,uint16_t len);
#endif
