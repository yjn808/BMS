#include "low_tesk.h"

//unsigned char key0_flag =0;
//unsigned char key1_flag =0;
//unsigned char key2_flag =0;

//void low_tesk(void)
//{
//	//按键0按下-----
//   if(key0_flag == 1 ||key0_flag == 3)
//	 {
//	    if(key0_flag == 1)
//			{
//			   key0_flag = 2;
//			}
//			else if(key0_flag == 3)
//			{
//			   key0_flag = 0;
//			}
//	 }
//	//按键1按下-----
//   if(key1_flag == 1 ||key1_flag == 3)
//	 {
//	    if(key1_flag == 1)
//			{
//			  key1_flag = 2;
//			}
//			else if(key1_flag == 3)
//			{
//			   key1_flag = 0;
//			}
//	 }
//	 	//按键2按下-----做SD_CONTROL开启关闭电机
//   if(key2_flag == 1 ||key2_flag == 3)
//	 {
//	    if(key2_flag == 1)
//			{				
//				TIM1->CCR1 = 0;
//				TIM1->CCR2 = 0;
//				TIM1->CCR3 = 0;
//				MOTOR1_SD(1);
//				FOC_Input.Iq_ref = 0.0f;
//				foc_algorithm_initialize();//EKF数据初始化，不初始化EKF运算无效
//					
//				OLED_ShowStr(0,6,"Start",2); 
//				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//				key2_flag = 2;
//			}
//			else if(key2_flag == 3)
//			{
//				
//				//get_offset_flag=1;
//				MOTOR1_SD(0);
//				TIM1->CCR1 = 0;
//				TIM1->CCR2 = 0;
//				TIM1->CCR3 = 0;
//				FOC_Input.Iq_ref = 0.0f;
//			
//				OLED_Fill(0x00);//部分页填充	
//        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);				
//			  key2_flag = 0;
//			}
//	 }	 

//}


