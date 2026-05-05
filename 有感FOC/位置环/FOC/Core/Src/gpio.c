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

  /*Configure GPIO pin : Encoder_Z_Pin */
  GPIO_InitStruct.Pin = Encoder_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Encoder_Z_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

// uint8_t key_scan(uint8_t mode)
// {
//     static uint8_t key_up = 1;
//     uint8_t keyval = 0;
    
//     if (mode) key_up = 1;
    
//     if (key_up && (BUTTON1 == 0 || BUTTON2 == 0 || BUTTON3 == 0))
//     {
//         HAL_Delay(10);
//         key_up = 0;
        
//         if (BUTTON3 == 0)       keyval = BUTTON3_PRES;
//         else if (BUTTON2 == 0)  keyval = BUTTON2_PRES;
//         else if (BUTTON1 == 0)  keyval = BUTTON1_PRES;
//     }
//     else if (BUTTON1 == 1 && BUTTON2 == 1 && BUTTON3 == 1)
//     {
//         key_up = 1;
//     }
    
//     return keyval;
// }


// 外部中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 简单的消抖处理（100ms内的重复触发忽略）
    if(current_time - last_press_time < 100)
        return;
    last_press_time = current_time;
    
    switch(GPIO_Pin)
    {
        case GPIO_PIN_8:        // PB8 - Encoder_Z
            // 编码器Z信号处理
            if(Encoder.Z == 0)
                Encoder.Z = 1;
            else if(Encoder.Z == 1)
                Encoder.Z = 0;
            break;
            
        case GPIO_PIN_10:       // PC10 - Button1 (Motor Start/Stop)
            if(rtX.Speed_ref != 0)
            {
                rtX.Speed_ref = 0.0f;  // 停止电机
                rtX.Angle_ref = 0.0f;  // 重置角度目标
            }
            else
            {
                rtX.Angle_ref = 0.0f;  // 设置初始角度目标
            }
            break;
            
        case GPIO_PIN_11:       // PC11 - Button2 (正向旋转PI/4)
            rtX.Angle_ref += PI/4;  // 增加45度
            if(rtX.Angle_ref >= 2*PI)
                rtX.Angle_ref -= 2*PI;
            
            break;
            
        case GPIO_PIN_13:       // PC13 - Button3 (反向旋转PI/4)
            rtX.Angle_ref -= PI/4;  // 减少45度
            if(rtX.Angle_ref < 0)
                rtX.Angle_ref += 2*PI;   
            break;
            
        default:
            break;
    }
}

/* USER CODE END 2 */
