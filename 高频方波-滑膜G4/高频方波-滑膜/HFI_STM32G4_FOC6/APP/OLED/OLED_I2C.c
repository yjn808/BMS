/************************************************************************************
*  Copyright (c), 2014, HelTec Automatic Technology co.,LTD.
*            All rights reserved.
*
* Http:    www.heltec.cn
* Email:   cn.heltec@gmail.com
* WebShop: heltec.taobao.com
*
* File name: OLED_I2C.c
* Project  : HelTec.uvprij
* Processor: STM32F103C8T6
* Compiler : MDK fo ARM
* 
* Author : 小林
* Version: 1.00
* Date   : 2014.4.8
* Email  : hello14blog@gmail.com
* Modification: none
* 
* Description:128*64点阵的OLED显示屏驱动文件，仅适用于惠特自动化(heltec.taobao.com)的SD1306驱动IIC通信方式显示屏
*
* Others: none;
*
* Function List:
*	1. void I2C_Configuration(void) -- 配置CPU的硬件I2C
* 2. void I2C_WriteByte(uint8_t addr,uint8_t data) -- 向寄存器地址写一个byte的数据
* 3. void WriteCmd(unsigned char I2C_Command) -- 写命令
* 4. void WriteDat(unsigned char I2C_Data) -- 写数据
* 5. void OLED_Init(void) -- OLED屏初始化
* 6. void OLED_SetPos(unsigned char x, unsigned char y) -- 设置起始点坐标
* 7. void OLED_Fill(unsigned char fill_Data) -- 全屏填充
* 8. void OLED_CLS(void) -- 清屏
* 9. void OLED_ON(void) -- 唤醒
* 10. void OLED_OFF(void) -- 睡眠
* 11. void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize) -- 显示字符串(字体大小有6*8和8*16两种)
* 12. void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N) -- 显示中文(中文需要先取模，然后放到codetab.h中)
* 13. void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]) -- BMP图片
*
* History: none;
*
*************************************************************************************/

#include "OLED_I2C.h"
#include "codetab.h"

uint8_t GRAM[8][128];
 
 
 
 
void I2C_Configuration(void)
{
    //I2C接口初始化
			OLED_IIC_Init();
}

void I2C_WriteByte(uint8_t addr,uint8_t data)
{
//  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
//	
//	I2C_GenerateSTART(I2C1, ENABLE);//开启I2C1
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));/*EV5,主模式*/

//	I2C_Send7bitAddress(I2C1, OLED_ADDRESS, I2C_Direction_Transmitter);//器件地址 -- 默认0x78
//	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//	I2C_SendData(I2C1, addr);//寄存器地址
//	
//	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//	I2C_SendData(I2C1, data);//发送数据
//	
//	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	
//	I2C_GenerateSTOP(I2C1, ENABLE);//关闭I2C1总线
	
 OLED_WriteOneByte(addr,data);
	
}

void WriteCmd(unsigned char I2C_Command)//写命令
{
	I2C_WriteByte(0x00, I2C_Command);
}

void WriteDat(unsigned char I2C_Data)//写数据
{
	I2C_WriteByte(0x40, I2C_Data);
}

void OLED_Init(void)
{
	delay_ms(100); //这里的延时很重要
	
	WriteCmd(0xAE); //display off
	WriteCmd(0x20);	//Set Memory Addressing Mode	
	WriteCmd(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	WriteCmd(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	WriteCmd(0xc8);	//Set COM Output Scan Direction
	WriteCmd(0x00); //---set low column address
	WriteCmd(0x10); //---set high column address
	WriteCmd(0x40); //--set start line address
	WriteCmd(0x81); //--set contrast control register
	WriteCmd(0xff); //亮度调节 0x00~0xff
	WriteCmd(0xa1); //--set segment re-map 0 to 127
	WriteCmd(0xa6); //--set normal display
	WriteCmd(0xa8); //--set multiplex ratio(1 to 64)
	WriteCmd(0x3F); //
	WriteCmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	WriteCmd(0xd3); //-set display offset
	WriteCmd(0x00); //-not offset
	WriteCmd(0xd5); //--set display clock divide ratio/oscillator frequency
	WriteCmd(0xf0); //--set divide ratio
	WriteCmd(0xd9); //--set pre-charge period
	WriteCmd(0x22); //
	WriteCmd(0xda); //--set com pins hardware configuration
	WriteCmd(0x12);
	WriteCmd(0xdb); //--set vcomh
	WriteCmd(0x20); //0x20,0.77xVcc
	WriteCmd(0x8d); //--set DC-DC enable
	WriteCmd(0x14); //
	WriteCmd(0xaf); //--turn on oled panel
}

void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{ 
	WriteCmd(0xb0+y);
	WriteCmd(((x&0xf0)>>4)|0x10);//高四位列地址
	WriteCmd((x&0x0f)|0x01);//低四位列地址
}

void OLED_Fill(unsigned char fill_Data)//全屏填充
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}


void OLED_Fill_block(unsigned char fill_Data,uint8_t start_y,uint8_t end_y)//部分页填充
{
	unsigned char m,n;
	if(start_y>end_y) start_y = end_y - 1;
	for(m=start_y;m<end_y;m++)
	{
		WriteCmd(0xb0+m);		//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
			{
				WriteDat(fill_Data);
			}
	}
}



void OLED_CLS(void)//清屏
{
	OLED_Fill(0x00);
}

//--------------------------------------------------------------
// Prototype      : void OLED_ON(void)
// Calls          : 
// Parameters     : none
// Description    : 将OLED从休眠中唤醒
//--------------------------------------------------------------
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED唤醒
}

//--------------------------------------------------------------
// Prototype      : void OLED_OFF(void)
// Calls          : 
// Parameters     : none
// Description    : 让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
//--------------------------------------------------------------
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED休眠
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowChar(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); ch[] -- 要显示的字符串; TextSize -- 字符大小(1:6*8 ; 2:8*16)
// Description    : 显示codetab.h中的ASCII字符,有6*8和8*16可选择
//--------------------------------------------------------------
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize)
{
	unsigned char c = 0,i = 0,j = 0;
	switch(TextSize)
	{
		case 1:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 126)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<6;i++)
					WriteDat(F6x8[c][i]);
				x += 6;
				j++;
			}
		}break;
		case 2:
		{
			while(ch[j] != '\0')
			{
				c = ch[j] - 32;
				if(x > 120)
				{
					x = 0;
					y++;
				}
				OLED_SetPos(x,y);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i]);
				OLED_SetPos(x,y+1);
				for(i=0;i<8;i++)
					WriteDat(F8X16[c*16+i+8]);
				x += 8;
				j++;
			}
		}break;
	}
}

//--------------------------------------------------------------
// Prototype      : void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); N:汉字在codetab.h中的索引
// Description    : 显示codetab.h中的汉字,16*16点阵
//--------------------------------------------------------------
void OLED_ShowCN(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=32*N;
	OLED_SetPos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
	OLED_SetPos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		WriteDat(F16x16[adder]);
		adder += 1;
	}
}
/*
void OLED_ShowCN40(unsigned char x, unsigned char y, unsigned char N)
{

	unsigned char i=0;
    unsigned char wm=0;
	unsigned int  adder=200*N;
	
	for(i=0;i<5;i++)
	{
		OLED_SetPos(x , y+i);
		for(wm = 0;wm < 40;wm++)
		{
			WriteDat(F40x40[adder]);
			adder += 1;
		}
   }

}

*/


/*void OLED_ShowCN24(unsigned char x, unsigned char y, unsigned char N)
{

	unsigned char i=0;
    unsigned char wm=0;
	unsigned int  adder=72*N;
	
	for(i=0;i<3;i++)
	{
		OLED_SetPos(x , y+i);
		for(wm = 0;wm < 24;wm++)
		{
			WriteDat(F24x24[adder]);
			adder += 1;
		}
   }

}
*/
/*
void OLED_ShowCN32(unsigned char x, unsigned char y, unsigned char N)
{

	unsigned char i=0;
    unsigned char wm=0;
	unsigned int  adder=128*N;
	
	for(i=0;i<4;i++)
	{
		OLED_SetPos(x , y+i);
		for(wm = 0;wm < 32;wm++)
		{
			WriteDat(F32x32[adder]);
			adder += 1;
		}
   }

}
*/



//--------------------------------------------------------------
// Prototype      : void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- 起始点坐标(x0:0~127, y0:0~7); x1,y1 -- 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
// Description    : 显示BMP位图
//--------------------------------------------------------------
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[])
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		OLED_SetPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			WriteDat(BMP[j++]);
		}
	}
}




/* 
*函数功能：一帧显示函数
*作者    ：何荣涛
*   当其它OLED函数将显示数据传入到了GRAM数组里面后
    调用此函数将数据一次更新到OLED屏幕的FLASH里面
    这样能够尽量的提高屏幕的刷新率

*/
void OLED_Frame(uint8_t(*date)[128])
{
	uint8_t page = 0;
	uint8_t line = 0;
	for(page = 0;page<8;page++)
	{
		WriteCmd(0xb0+page);//设置页地址
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(line = 0; line <128;line ++)
		{
				I2C_WriteByte(0x40, *(*(date+page)+line));
		}
	}
}

/* 
*函数功能：GRAM数组数据填充传入
*作者    ：何荣涛
*函数参数：data 需要显示的数据传入缓存数组
*
*/
/*开设一个能够缓存一个屏幕一帧所需要的数据内存大小*/

void GRAM_Fill(uint8_t date,uint8_t (*buf)[128])
{
	uint8_t page = 0;
	uint8_t line = 0;
	for(page = 0;page<8;page++)
	{
		for(line =0; line <128;line++)
		{
			*(*(buf+page)+line) = date;
		}	
	}
}
/* 
*函数功能：oled打点函数
*作者    ：何荣涛
*函数参数： X坐标范围 0-127   Y轴坐标范围 0-63   data 需要显示的数据缓存数组
* GRAM幅值时不会覆盖掉数组里面其他位置的数据，只会覆盖指定位置的数据
*/
void OLED_Point(uint8_t x,uint8_t y,uint8_t date)
{
	 uint8_t page = 0;
	 uint8_t line = 0;
	 uint8_t bx   = 0;
	page = 7-(y/8);//第几页
	line = y%8;
	bx = 1<<(7-line);
	if(date) GRAM[page][x] |= bx;
	else GRAM[page][x] &= ~bx;	
	
}
/* 
*函数功能：画线函数
*作者    ：何荣涛
*函数参数： X坐标范围 0-127   Y轴坐标范围 0-63  
*
*/
void OLED_Drawline( int  x1, int y1, int x2, int y2)
{
			short int slope = 0;
	    short int line = 0;
	    short int hang = 0;
	     short int X,Y;
		//绝对值函数
				X = x2 - x1;
				if(X<0) X=-X;
				Y =y2 - y1;
	      if(Y<0) Y=-Y;
				slope = (y2 - y1)/(x2 - x1);    
		//直线部分
	    if((X==0)||(Y==0))
			{
					if(X==0) 
					{
						for(line = 0;line <Y+1;line++)
						{
							OLED_Point(x1,line,1);
						
						}
					}
						if(Y==0) 
						{
							for(hang = 0;hang <X+1;hang++)
							{
								
								OLED_Point(hang,y1,1);
							}
						}		    
			}
			//斜线部分  y = kx +b
			else
			{
				
			    for(hang = 0;hang<X;hang++)
					 {						 
					     OLED_Point(x1+hang,(y1+(slope*hang)),1);
					 }
			}
}


void headline(void)
{
	 
	 OLED_ShowCN(31 , 4, 6);//欢
	 OLED_ShowCN(47 , 4, 7);//迎
	 OLED_ShowCN(63 , 4, 8);//使
	 OLED_ShowCN(79 , 4, 9);//用
			
}

void OLED_ShowNum(unsigned char x, unsigned char y, short num)
{
	unsigned char a= 0,b = 0,c = 0,d=0,i=0;
	if(num<0)
	{
		num=-num;
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
			WriteDat(F6x8[13][i]);
		x+=6;
	}
	a=num/1000;//索引号
	b=num/100%10;
	c=num/10%10;
	d=num%10;
	
	if(x > 126)//
	{
		x = 0;
		y++;//
	}

		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
			WriteDat(F6x8[a+16][i]);
		x += 6;//一个字符占6列，打完1个字符起始点+6
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
			WriteDat(F6x8[b+16][i]);	
		x += 6;
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
			WriteDat(F6x8[c+16][i]);	
		x += 6;
		OLED_SetPos(x,y);
		for(i=0;i<6;i++)
			WriteDat(F6x8[d+16][i]);	
		x += 6;

}


/*************************************************************
作者 *   					何荣涛
函数功能*					传入数据缓存数组OLED显示
传入参数*					要显示的数据缓存数组
时间*						2022年12月26日
**************************************************************/
void FOC_data_oled(uint8_t buff1[] , uint8_t buff2[] ,uint8_t buff3[], uint8_t buff4[])
{
	  //AD采集通道1
  	//OLED_ShowCN16(0,0,0);//通
		//OLED_ShowCN16(16,0,1);//道
		OLED_ShowStr(0,0,"Uq",2);//1
		OLED_ShowStr(16,0,":",2);// ：	
		OLED_ShowStr(24,0,buff1,2);//数据显示
	
	  //AD采集通道2
//		OLED_ShowCN16(0,2,0);
//		OLED_ShowCN16(16,2,1);
		OLED_ShowStr(0,2,"Arg",2);
		OLED_ShowStr(24,2,":",2);
		OLED_ShowStr(32,2,buff2,2);
		
	 //AD采集通道4
//		OLED_ShowCN16(0,4,0);
//		OLED_ShowCN16(16,4,1);
//		OLED_ShowStr(0,4,"NO",2);
//		OLED_ShowStr(16,4,":",2);
//		OLED_ShowStr(24,4,buff3,2);
		
	//AD采集通道8
//		OLED_ShowCN16(0,6,0);
//		OLED_ShowCN16(16,6,1);
//		OLED_ShowStr(0,6,"NO",2);
//		OLED_ShowStr(16,6,":",2);
//	  OLED_ShowStr(24,6,buff4,2);
}




