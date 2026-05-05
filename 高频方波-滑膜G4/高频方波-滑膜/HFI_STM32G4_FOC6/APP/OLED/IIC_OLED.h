#ifndef __IIC_OLED_H
#define __IIC_OLED_H
#include  "main.h"
 	   		   
/* IO操作 */
#define OLED_IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define OLED_IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */

#define IIC_READ_SDA     HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin)        /* 读取SDA */

//IIC所有操作函数
void OLED_IIC_Init(void);                //初始化IIC的IO口				 
void OLED_IIC_Start(void);				//发送IIC开始信号
void OLED_IIC_Stop(void);	  			//发送IIC停止信号
void OLED_IIC_Send_Byte(uint8_t txd);			//IIC发送一个字节
uint8_t OLED_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
uint8_t OLED_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void OLED_IIC_Ack(void);					//IIC发送ACK信号
void OLED_IIC_NAck(void);				//IIC不发送ACK信号

 void OLED_WriteOneByte(uint8_t addr,uint8_t data);
#endif
















