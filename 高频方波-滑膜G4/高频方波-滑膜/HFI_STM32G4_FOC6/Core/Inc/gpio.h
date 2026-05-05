/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* LED端口定义 */ 
	/* 引脚 定义 */


#define LED0(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

#define LED1(x)   do{ x ? \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = GREEN */

/* LED取反定义 */
#define LED0_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_Port, LED0_Pin); }while(0)       /* LED0 = !LED0 */
#define LED1_TOGGLE()    do{ HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin); }while(0)       /* LED1 = !LED1 */

/*按键*/



#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_Port,  KEY0_Pin)     /* 读取KEY0引脚 */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_Port,  KEY1_Pin)     /* 读取KEY1引脚 */
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_Port,  KEY2_Pin)     /* 读取KEY2引脚 */

#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */
#define KEY2_PRES    3              /* KEY2按下 */

/*蜂鸣器*/

#define BEEP(x)   do{ x ? \
                      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET); \
                  }while(0)      

/*电机启动停止控制 ---- MOTOR1_SD  MOTOR2_SD*/
									
#define MOTOR1_SD(x)   do{ x ? \
                      HAL_GPIO_WritePin(MOTOR1_SD_GPIO_Port, MOTOR1_SD_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MOTOR1_SD_GPIO_Port, MOTOR1_SD_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED0 = RED */

#define MOTOR2_SD(x)   do{ x ? \
                      HAL_GPIO_WritePin(MOTOR2_SD_GPIO_Port, MOTOR2_SD_Pin, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(MOTOR2_SD_GPIO_Port, MOTOR2_SD_Pin, GPIO_PIN_RESET); \
                  }while(0)       /* LED1 = GREEN */									
									
#define MOTOR1_SD_TOGGLE()    do{ HAL_GPIO_TogglePin(MOTOR1_SD_GPIO_Port,MOTOR1_SD_Pin); }while(0)       /* LED0 = !LED0 */
#define MOTOR2_SD_TOGGLE()    do{ HAL_GPIO_TogglePin(MOTOR2_SD_GPIO_Port,MOTOR2_SD_Pin); }while(0)       /* LED1 = !LED1 */
								
									
/* 由于GPIO_PIN_10 ~ GPIO_PIN_15是共用同一个中断线以及中断服务函数的，所以这里只需定义一个即可 */
#define KEY_INT_IRQHandler              EXTI15_10_IRQHandler

extern unsigned char key0_flag;
extern unsigned char key1_flag;
extern unsigned char key2_flag;							
									
									
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t key_scan(uint8_t mode);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

