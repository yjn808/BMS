#include "IIC_OLED.h"


//初始化IIC
void OLED_IIC_Init(void)
{			
  OLED_IIC_SCL(1);	
	OLED_IIC_SDA(1);
}
void OLED_IIC_Start(void)
{
	
	OLED_IIC_SDA(1);  	  
	 OLED_IIC_SCL(1);
//	delay_us(1);
 	OLED_IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
//	delay_us(2);
	OLED_IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void OLED_IIC_Stop(void)
{
	
	OLED_IIC_SCL(0);
	OLED_IIC_SDA(0);//STOP:when CLK is high DATA change form low to high
// 	delay_us(1);
	 OLED_IIC_SCL(1); 
//	delay_us(2);
	OLED_IIC_SDA(1); //发送I2C总线结束信号
	//delay_us(2);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t OLED_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;

	OLED_IIC_SDA(1);
//	delay_us(1);	   
	 OLED_IIC_SCL(1);
//	delay_us(1);	 
	while(IIC_READ_SDA )
	{
		ucErrTime++;
		if(ucErrTime>10)
		{
			OLED_IIC_Stop();
			return 1;
		}
	}
	OLED_IIC_SCL(0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void OLED_IIC_Ack(void)
{
	OLED_IIC_SCL(0);
	OLED_IIC_SDA(0);
//	delay_us(1);
	 OLED_IIC_SCL(1);
//	delay_us(2);
	OLED_IIC_SCL(0);
	//delay_us(2);
}
//不产生ACK应答		    
void OLED_IIC_NAck(void)
{
	OLED_IIC_SCL(0);
	OLED_IIC_SDA(1);
	delay_us(1);
	 OLED_IIC_SCL(1);
//	delay_us(2);
	OLED_IIC_SCL(0);
	//delay_us(2);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void OLED_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t  t;   	    
    OLED_IIC_SCL(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
    OLED_IIC_SDA((txd&0x80)>>7);
    txd<<=1; 	  
	  delay_us(1);   //对TEA5767这三个延时都是必须的
		 OLED_IIC_SCL(1);
//		delay_us(2); 
		OLED_IIC_SCL(0);	
//		delay_us(2);
    }
		 OLED_IIC_SCL(1);
//		delay_us(2); 
		OLED_IIC_SCL(0);
		
} 	    


void OLED_WriteOneByte(uint8_t addr,uint8_t data)
{				   	  	    																 
		OLED_IIC_Start();  
    OLED_IIC_Send_Byte(OLED_ADDRESS);	    //器件地址
	  //OLED_IIC_Wait_Ack(); 
		OLED_IIC_Send_Byte(addr);	    //寄存器地址
		//OLED_IIC_Wait_Ack();	 										  		   
		OLED_IIC_Send_Byte(data);     //发送字节							       	   
    OLED_IIC_Stop();//产生一个停止条件 
	 
}



















