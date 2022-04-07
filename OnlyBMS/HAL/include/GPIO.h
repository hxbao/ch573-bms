/*
 * GPIO.h
 *
 *  Created on: Apr 4, 2022
 *      Author: hxbao
 */

#ifndef HAL_INCLUDE_GPIO_H_
#define HAL_INCLUDE_GPIO_H_

#define TN_LED_PAPIN                      GPIO_Pin_12
#define TN_ACC_PBPIN                      GPIO_Pin_15
#define TN_SHINT_PAPIN                    GPIO_Pin_4
#define TN_LOAD_DET_PBPIN                 GPIO_Pin_7
#define TN_CHG_DET_PBPIN                  GPIO_Pin_11
#define TN_PREEN_PBPIN                    GPIO_Pin_12
#define TN_SHSHIP_PAPIN                   GPIO_Pin_10
#define TN_VPRO_CON_PBPIN                 GPIO_Pin_10
#define TN_NTC0_POWER_PAPIN               GPIO_Pin_10
#define TN_I2C1_SCL_PBPIN                 GPIO_Pin_23
#define TN_I2C1_SDA_PBPIN                 GPIO_Pin_22
#define TN_ONE_TX_PAPIN                   GPIO_Pin_5

void Hal_GpioInit();

#endif /* HAL_INCLUDE_GPIO_H_ */
