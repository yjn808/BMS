#ifndef __OLED_I2C_H
#define	__OLED_I2C_H

#include "main.h"

#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78
/*开设一个能够缓存一个屏幕一帧所需要的数据内存大小*/
extern uint8_t GRAM[8][128];



void I2C_Configuration(void);
void I2C_WriteByte(uint8_t addr,uint8_t data);
void WriteCmd(unsigned char I2C_Command);
void WriteDat(unsigned char I2C_Data);
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

void OLED_Frame(uint8_t(*date)[128]);


void GRAM_Fill(uint8_t date,uint8_t (*buf)[128]);

void OLED_Point(uint8_t x,uint8_t y,uint8_t date);


void OLED_Drawline( int x1, int y1, int x2, int y2);

void headline(void);
void OLED_ShowNum(unsigned char x, unsigned char y, short num);
void FOC_data_oled(uint8_t buff1[] , uint8_t buff2[] ,uint8_t buff3[], uint8_t buff4[]);
void OLED_Fill_block(unsigned char fill_Data,uint8_t start_y,uint8_t end_y);//部分页填充

#endif
