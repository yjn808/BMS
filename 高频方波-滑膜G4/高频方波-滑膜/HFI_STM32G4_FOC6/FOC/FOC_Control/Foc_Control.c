#include "Foc_Control.h"

uint32_t A_offset,B_offset,C_offset;

double Ia,Ib,Ic;
float Ia_test,Ib_test,Ic_test;
float Vbus;


float arg_amplitude = 0.0f;
float arg_step = 0.0f;

uint8_t get_offset_flag = 0;
uint8_t get_offset_sample_cnt = 0;

uint8_t OLED_BUFF[15];
uint16_t funcation_up_date = 0;

float Iq_temp1 = 0.0f;
float Iq_temp2 = 0.0f;

uint8_t Iq_temp2_flag= 0;
uint8_t Iq_temp_flag= 0;
uint8_t Iq_temp3_flag= 0;

uint16_t speed_low_flag = 0;
uint16_t speed_err_flag = 0;

//修改兼容正点原子母线电压采集

#define ADC_REF_V                   (float)(3.3f)
#define VBUS_UP_RES                 (float)(24.0f) 
#define VBUS_DOWN_RES               (float)(1.0f) 
//#define VBUS_CONVERSION_FACTOR      (float)(ADC_REF_V*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/4095.0f)
#define VBUS_CONVERSION_FACTOR      (float)(0.02014f)
//电流检测--偏置电压1.25对应ADC值1551    
#define SAMPLE_RES                  (double)(0.02f)
#define AMP_GAIN                    (double)(6.0f)
//#define SAMPLE_CURR_CON_FACTOR      (double)(ADC_REF_V/4095.0f/AMP_GAIN/SAMPLE_RES)
#define SAMPLE_CURR_CON_FACTOR      (double)(0.0067138671875)
void get_offset(uint32_t *a_offset,uint32_t *b_offset,uint32_t *c_offset)
{
  if(get_offset_sample_cnt<128)
  {
    *a_offset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    *b_offset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		*c_offset += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    get_offset_sample_cnt++;
  }
  else
  {
    *a_offset >>= 7;
    *b_offset >>= 7;
		*c_offset >>= 7;
    get_offset_sample_cnt=0;
		HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    get_offset_flag = 2;
		
  }
}
void usart_down_date(void)
{

	//串口数据接收	
	if((g_usart_rx_sta&0x8000) == 0x8000)
	{			 
		uint16_t head_number = 0;
		//数据标号判断
		head_number= head_number_date(g_usart_rx_buf);
		if(head_number == 1)
		{			  
				//Uq 大小设置
				FOC_Input.Iq_ref = my_usart_date_float(g_usart_rx_buf,1,15,OLED_BUFF);
				Iq_temp2 = FOC_Input.Iq_ref;
			  Iq_temp_flag = 1;
			  Iq_temp3_flag = 1;
				//限幅
				if(FOC_Input.Iq_ref >3.0f ||FOC_Input.Iq_ref<-3.0)FOC_Input.Iq_ref = 0.0f;
			  //OLED显示
					OLED_Fill_block(0x00,0,2);//部分页填充								
					OLED_ShowStr(0,0,"Iq",2);//1
				  OLED_ShowStr(16,0,":",2);//  
			    OLED_ShowStr(32,0,OLED_BUFF,2);
			  //printf("接受到的数据rtU.Uq：%f\r\n",rtU.Uq);	
		}
			 //角度增加幅值设置	
		if(head_number == 2)
		{	
			 //串口接收数据处理
			arg_amplitude = my_usart_date_float(g_usart_rx_buf,2,15,OLED_BUFF);			
			//限幅							
		if(arg_amplitude >1.5f || arg_amplitude <0)arg_amplitude = 0;	
			//OLED显示
					OLED_Fill_block(0x00,3,4);//部分页填充
					OLED_ShowStr(0,2,"Arg",2);
				  OLED_ShowStr(24,2,":",2);
					OLED_ShowStr(32,2,OLED_BUFF,2);	
			//printf("接受到的数据arg_amplitude：%f\r\n",arg_amplitude);	
		}
		 //设置速度环参考的大小
		if(head_number == 3)
		{
				Speed_Ref = my_usart_date_float(g_usart_rx_buf,3,15,OLED_BUFF);
		}
		//设置电流环KI的大小
		if(head_number == 4)
		{
//		    MY_PID.Q_KI = my_usart_date_float(USART2_RX_BUF,4,15,OLED_BUFF);
		}
			
		//数据上传到上位机显示--判断显示那个数据
		if(head_number == 5)
		{
		    funcation_up_date=my_usart_date_int(g_usart_rx_buf,5,15,OLED_BUFF);		  
		}
		//清除接收标志---准备下一次接收
		//g_usart_rx_sta = 0;
    for(g_usart_rx_sta = 0;g_usart_rx_sta<15;g_usart_rx_sta++)
		{
      OLED_BUFF[g_usart_rx_sta] = 0; 
		}	
    g_usart_rx_sta = 0;		
	}
}

void motor_run(void)
{
		float vbus_temp;
		double ia_temp,ib_temp,ic_temp;	
	  HAL_GPIO_WritePin(MOTOR2_SD_GPIO_Port, MOTOR2_SD_Pin, GPIO_PIN_SET) ;
		// 获取注入通道的转换结果 
		ia_temp = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)-(int16_t)A_offset);//得到A相电流 adc转换值
		ib_temp = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)-(int16_t)B_offset);//得到B相电流 adc转换值
		ic_temp = ((uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3)-(int16_t)C_offset);//得到C相电流 adc转换值
		vbus_temp = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);//得到母线电压 adc转换值
	
		Vbus = vbus_temp*VBUS_CONVERSION_FACTOR;                        //通过电压转换因子（通过分压电阻得到）把adc转换值 转化为 真实电压
		Ia = ia_temp*SAMPLE_CURR_CON_FACTOR;                            //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值
		Ib = ib_temp*SAMPLE_CURR_CON_FACTOR;                            //通过电流转换因子（通过采样电阻和运算放大倍数得到）把adc采样值转化为真实电流值 
		Ic = ic_temp*SAMPLE_CURR_CON_FACTOR;
		
		Ia_test = Ia;
		Ib_test = Ib;
		Ic_test = Ic;
	
	//缓冲启动
    if(Iq_temp_flag == 1)
		{
			if(Iq_temp3_flag ==1)
			{
				 if(Iq_temp1<FOC_Input.Iq_ref)
				 {
						Iq_temp2_flag= 1;
				 }
				 else
				 {
						Iq_temp2_flag= 2;
				 }
				 Iq_temp3_flag = 0;
		  }
       if(Iq_temp2_flag==1)
			 {
			    Iq_temp1+=0.001f;
				   if(Iq_temp1>Iq_temp2)
					 {
					
					    Iq_temp_flag = 0;
						  Iq_temp2_flag = 0;
						  Iq_temp1 = Iq_temp2;
					 }
			 }
			 
			  if(Iq_temp2_flag==2)
			 {
			    Iq_temp1-=0.001f;
				   if(Iq_temp1<Iq_temp2)
					 {
					    Iq_temp_flag = 0;
						 Iq_temp2_flag = 0;
						 Iq_temp1 = Iq_temp2;
					 }
			 }	 
		}
		
#ifdef  SENSORLESS_FOC_SELECT            //通过条件编译选择无感FOC运行

/************
直接零速闭环----低速高频方波--中高速滑膜
**************/
	//高频方波注入
  /****1----首先低速目标给定*****/
	  /*极性辨识完成---直接切入速度闭环*/
		/*
		参数说明：NS_flag = 0 表示可以进入极性识别
		          NS_flag = 1 表示极性识别完成
							NS_flag = 2 表示进入中高速，切入滑膜
		*/
		switch(NS_flag )
		{		
			case 0:
					//极性识别没有完成之前，Iq的值为0			
					FOC_Input.Iq_ref= Iq_temp1;
					Speed_Pid.I_Sum = Iq_temp1;
			break;
			
			case 1 :			
				 FOC_Input.Iq_ref = Speed_Pid_Out;			
				 Speed_Fdk = hfi_square_PLL_def.we;			
          Speed_Pid.speed_last_lpf  = Speed_Pid_Out;	
				/*********如果速度达到中高速阈值--切入滑膜********/
				if(hfi_square_PLL_def.we >50||hfi_square_PLL_def.we<-50)
				{
					speed_low_flag++;
					Speed_Pid_Out = Speed_Pid_Out*0.7f +Speed_Pid.speed_last_lpf*0.3f;					
				  FOC_Input.Iq_ref = Speed_Pid_Out;
					
					 if(speed_low_flag>200)
					{
						NS_flag = 2;
						Uin_inject_start_off = 1;//关闭高频注入 
						Speed_Pid.I_Sum = 0.0f;
	          speed_low_flag = 0;
						
						Speed_Pid.speed_last_lpf =Speed_Pid_Out;
							
						Speed_Pid_Out = Speed_Pid_Out*0.7f +Speed_Pid.speed_last_lpf*0.3f;					
						FOC_Input.Iq_ref = Speed_Pid_Out;
										          					
//						if(hfi_square_PLL_def.we>0.0f)
//						{
//						  PLL_def.we= hfi_square_PLL_def.we+20.0f;	
//						}	
//             else
//						 {
//						    PLL_def.we= hfi_square_PLL_def.we-20.0f;	
//						 }							 
					}						
				}
			break;
				
			case 2:
							
            if(speed_low_flag<50)
						{
						  speed_low_flag++;
							Speed_Pid_Out = Speed_Pid_Out*0.7f +Speed_Pid.speed_last_lpf*0.3f;
							Speed_Pid.speed_last_lpf=Speed_Pid_Out;
						}		

						FOC_Input.Iq_ref = Speed_Pid_Out;
						
						FOC_Input.theta = PLL_def.theta;//使用滑膜角度
						Speed_Fdk = PLL_def.we;//使用滑膜we
			     
			   	//给定正转--实际SMO反转情况1
					if((Speed_Ref>1.0f) && (PLL_def.we<-400.0f)) 
					{			
						speed_err_flag++;
						FOC_Input.Iq_ref -= 0.01f*speed_err_flag;			 
					}
					//给定反转--实际SMO正转情况2
					else if((Speed_Ref<-1.0f) && (PLL_def.we>400.0f))
					{
						speed_err_flag++;
						FOC_Input.Iq_ref += 0.001f*speed_err_flag;				
					}	
					//不在可控范围内
					if((PLL_def.we>500)||(PLL_def.we<-500))
					{
							SMO_PLL_Init(&SMO_Struct_def,&PLL_def);		
							speed_err_flag = 0;
					}
			 /**********低速使用高频注入***********/
			  if((PLL_def.we < 30)&&(PLL_def.we>-30))
				{
					speed_low_flag++;
					if(speed_low_flag>12000)
					{
						NS_flag = 1;
					  Uin_inject_start_off = 0;//开启高频注入
						Speed_Pid.I_Sum = Speed_Pid_Out;	
										
						hfi_square_PLL_def.err_sum = 0.0f;					
						speed_low_flag = 0;
					}					
				}			
			break;
	}
#endif 

  FOC_Input.Tpwm = PWM_TIM_PULSE_TPWM;         //FOC运行函数需要用到的输入信息
  FOC_Input.Udc = Vbus;
  FOC_Input.ia = Ia_test;
  FOC_Input.ib = Ib_test;
  FOC_Input.ic = Ic_test; 
	FOC_Input.Id_ref = 0.0f;
	//计算好后赋值到PWM_CCRX比较寄存器通道
   foc_algorithm_step();
	 //通过SVPWM得到的占空比赋值给定时器的寄存器
	TIM1->CCR1 = (uint16_t)(FOC_Output.Tcmp1);     
	TIM1->CCR2 = (uint16_t)(FOC_Output.Tcmp2);
	TIM1->CCR3 = (uint16_t)(FOC_Output.Tcmp3);
	HAL_GPIO_WritePin(MOTOR2_SD_GPIO_Port, MOTOR2_SD_Pin, GPIO_PIN_RESET);
}



