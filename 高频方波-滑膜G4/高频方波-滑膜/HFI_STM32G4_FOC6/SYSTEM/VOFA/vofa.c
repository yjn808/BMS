#include "vofa.h"

//小端浮点数组形式，纯十六进制，节省带宽
//适用于通道数量多---发送频率高
//将浮点数据转为4个单字节
////帧尾
unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};



/*
float 和 unsigned long 具有相同的数据结构长度
union据类型里面的数据存放在相同的物理空间
*/
typedef union
{

  float fdate;
	unsigned long Ldate;
}Floatlongtype;
/*
将浮点数据转换为单四个字节的数据，用于传输

*/
void float_to_byte(float date,unsigned char buf[4])
{
	Floatlongtype f1;
	f1.fdate = date;
   buf[0] = (unsigned char)f1.Ldate;
	 buf[1] = (unsigned char)(f1.Ldate>>8);
   buf[2] = (unsigned char)(f1.Ldate>>16);
   buf[3] = (unsigned char)(f1.Ldate>>24);
}

/*
date：需要上传显示波形的数据
title：为1上传结束帧尾，表示数据上传完毕，其他值表示后面需要继续上传数据
*/

void justfloat_update(float date,unsigned char title)
{
    unsigned char buf[4];
	 // unsigned char DMA_BUF[8];
	  unsigned char i  =0;
    float_to_byte(date,buf);

/**********不使用DMA传输*************/	
	for(i = 0;i<4;i++)
	{
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET){}; //循环发送,直到发送完毕   
	      //HAL_UART_Transmit(&huart1, &buf[i], 1, 1);
			 USART1->TDR =  buf[i]; 
	}
	
	
	//发送帧尾
	if(title ==1)
	{
	  for(i = 0;i<4;i++)
		{
	  while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET){}; //循环发送,直到发送完毕   
			//HAL_UART_Transmit(&huart1, &tail[i], 1, 1); 
			 USART1->TDR =  tail[i]; 
		}		
	}
	/**********使用DMA传输*************/
//  if(title == 1)
//	{
//		DMA_BUF[0] = buf[0];
//		DMA_BUF[1] = buf[1];
//		DMA_BUF[2] = buf[2];
//		DMA_BUF[3] = buf[3];
//		
//		DMA_BUF[4] = tail[0];
//		DMA_BUF[5] = tail[1];
//		DMA_BUF[6] = tail[2];
//		DMA_BUF[7] = tail[3];
//    //避免前面的数据没有传输完毕		
//		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET){}; 
//		HAL_UART_Transmit_DMA(&huart1,DMA_BUF,8);
//	}		
//  else
//	{
//		//避免前面的数据没有传输完毕		
//		while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) != SET){};
//	  HAL_UART_Transmit_DMA(&huart1,buf,4);
//	}

}

