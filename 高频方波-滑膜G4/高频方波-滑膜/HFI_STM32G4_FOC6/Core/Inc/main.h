/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/******************/
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/******************/
#include "stm32g474xx.h"
#include "stdio.h"
#include "sys.h"
#include "delay.h"
#include  "my_usart.h"
/******************/
#include "IIC_OLED.h"
#include "OLED_I2C.h"	
/******************/
#include "vofa.h"	
#include "arm_math.h"
/******************/
#include "low_tesk.h"
#include "speed_pid.h"	

#include "foc_algorithm.h"
#include "Foc_Control.h"
#include "IIR_LPF_wrapper.h"
#include "hfi_square_wave.h"
#include "SMO_PLL.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SDA_Pin GPIO_PIN_2
#define OLED_SDA_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_3
#define OLED_SCL_GPIO_Port GPIOE
#define MOTOR1_SD_Pin GPIO_PIN_13
#define MOTOR1_SD_GPIO_Port GPIOC
#define VTEMP_Pin GPIO_PIN_0
#define VTEMP_GPIO_Port GPIOC
#define AMPU1_Pin GPIO_PIN_1
#define AMPU1_GPIO_Port GPIOC
#define AMPV1_Pin GPIO_PIN_2
#define AMPV1_GPIO_Port GPIOC
#define AMPW1_Pin GPIO_PIN_3
#define AMPW1_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_3
#define VBUS_GPIO_Port GPIOA
#define BEMFW_Pin GPIO_PIN_4
#define BEMFW_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_12
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI15_10_IRQn
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOE
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR2_SD_Pin GPIO_PIN_11
#define MOTOR2_SD_GPIO_Port GPIOD
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
