/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : Button3_Pin Button1_Pin Button2_Pin */
  GPIO_InitStruct.Pin = Button3_Pin|Button1_Pin|Button2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;
    uint8_t keyval = 0;
    
    if (mode) key_up = 1;
    
    if (key_up && (BUTTON1 == 0 || BUTTON2 == 0 || BUTTON3 == 0))
    {
        HAL_Delay(10);
        key_up = 0;
        
        if (BUTTON3 == 0)       keyval = BUTTON3_PRES;
        else if (BUTTON2 == 0)  keyval = BUTTON2_PRES;
        else if (BUTTON1 == 0)  keyval = BUTTON1_PRES;
    }
    else if (BUTTON1 == 1 && BUTTON2 == 1 && BUTTON3 == 1)
    {
        key_up = 1;
    }
    
    return keyval;
}


// 外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch(GPIO_Pin)
    {
        case GPIO_PIN_10:       // PC10 - Button1
            // 按键1处理
            break;
            
        case GPIO_PIN_11:       // PC11 - Button2  
            // 按键2处理
            break;
            
        case GPIO_PIN_13:       // PC13 - Button3
            // 按键3处理
            break;
            
        default:
            break;
    }
}


/* USER CODE END 2 */
