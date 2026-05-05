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

#define BUTTON1_PIN    GPIO_PIN_10
#define BUTTON1_PORT   GPIOC
#define BUTTON2_PIN    GPIO_PIN_11
#define BUTTON2_PORT   GPIOC
#define BUTTON3_PIN    GPIO_PIN_13
#define BUTTON3_PORT   GPIOC

// 按键状态读取宏定义
#define BUTTON1        HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN)
#define BUTTON2        HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN)
#define BUTTON3        HAL_GPIO_ReadPin(BUTTON3_PORT, BUTTON3_PIN)

// 按键返回值定义
#define BUTTON1_PRES   1
#define BUTTON2_PRES   2
#define BUTTON3_PRES   3

uint8_t key_scan(uint8_t mode);


/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

