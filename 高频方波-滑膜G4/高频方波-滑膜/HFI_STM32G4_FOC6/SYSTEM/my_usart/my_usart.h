#ifndef _my_usart_H
#define _my_usart_H
#include "main.h"

/*
       ////---------注意---------////
函数使用字符传入注意：小数点后面第一位最好是偶数，不要奇数，硬件计算原因
                     例：456.234  解析出来为456.234
										     456.123  解析出来可能为 456.129999
			////---------注意---------////
*/
/*
			数据解析函数接口
			注意数据格式必须为 
			例  -------整形数据
			1:234344
			2:78945
			3：-234234
			5：-1343455
			例  -------浮点数据，小数点后最多6位
			6:0.2345
			7:123.089354
			8：-0.456
			9：-124.564958


*/

       /*---------对外接口调用--------*/

//整形数据解析---字符串转为数字
int my_usart_date_int(uint8_t  *date,uint16_t signed_number,uint16_t BUFF_RX_MAX_DATE,uint8_t *text_buf);
//浮点数据解析---字符串转为数字
float my_usart_date_float(uint8_t  *date,uint16_t signed_number,uint16_t BUFF_RX_MAX_DATE,uint8_t *text_buf);
//---------------数据转为字符串
void date_number_text_float(float date,uint8_t buff[]);
//---------------获取数据标号
uint16_t head_number_date(uint8_t  *date);



//-----------------内部函数-----------//
uint16_t  my_strlen(uint8_t  *date);
void date_clear(uint8_t  *date,uint16_t number);
int Date_number_int(uint16_t head_number ,uint16_t number,uint8_t *date,uint8_t *text_buf);
float Date_number_float(uint16_t head_number ,uint16_t number,uint8_t *date,uint8_t *text_buf);
int text_number(uint8_t  * text);



//--------------------------函数使用说明-------使用例子-----------------//
/*
       例1：解析串口发送来的数据-幅值给kp。kp,ki为浮点类型数据
            串口接收数组为 USART_RX_BUF[len]   len = 100; 
						串口接收数据为  1:12.245
						串口接收数据为  2:2.445
						设定kp接收标号1的数据
						设定ki接收标号2的数据
						
        
				      static float kp,ki;
				      uint16_t  head_number = 0;
							//获取标号
              head_number =  head_number_date(uint8_t  *date);
							//最后一个形参为15，因为我知道接收到的字符串最大个数为15个，所以幅值为15
							//函数里面也就只清除数组里面接收到的15个数据，如果不确定字符接收个数，直接填len
							//注意串口每次只能发送一个数据，不能一次发送两个
							
							//串口数据接收到数据标志判断	
								if((USART2_RX_STA&0x8000) == 0x8000)
								{			 
									//数据标号判断
									head_number= head_number_date(USART2_RX_BUF);
									if(head_number == 1)
									{			  
											//kp 大小设置
											kp= my_usart_date_float(USART2_RX_BUF,1,15,OLED_BUFF);
											//限幅
											if(kp >6.0f ||kp <0)kp = 0;
											//OLED显示
												OLED_Fill_block(0x00,0,2);//部分页填充																				
												OLED_ShowStr(0,0,"kp",2);//1
												OLED_ShowStr(16,0,":",2);//  
												OLED_ShowStr(32,0,OLED_BUFF,2);
											//printf("接受到的数据kp：%f\r\n",kp);	
									}
										
									if(head_number == 2)
									{	
										 //串口接收数据处理
										ki = my_usart_date_float(USART2_RX_BUF,2,15,OLED_BUFF);			
										//限幅							
									if(ki >1.5f || ki <0)ki = 0;	
										//OLED显示
												OLED_Fill_block(0x00,3,4);//部分页填充					
												OLED_ShowStr(0,2,"ki",2);
												OLED_ShowStr(24,2,":",2);
												OLED_ShowStr(32,2,OLED_BUFF,2);	
										//printf("接受到的数据arg_amplitude：%f\r\n",arg_amplitude);	
									}			
									//清除接收标志---准备下一次接收
									USART2_RX_STA = 0;			
								}
								
								
								
								
								
								

*/




#endif




