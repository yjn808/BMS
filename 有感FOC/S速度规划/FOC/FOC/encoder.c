#include "encoder.h"

Encoder_Typedef Encoder;


extern Speed_PID_DEF Speed_PID;
extern ExtX rtX; 



void Encoder_Init(void)
{
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_2);
	Encoder.Z = 0;
	
	Encoder.flag = 0;
	Encoder_ZeroAlignment();
	Encoder.flag = 1;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t count,dir_last;
    
    if(htim->Instance != TIM4) return; 
    
    if(TIM4->SR & TIM_FLAG_UPDATE)
    {
        TIM4->SR &= ~TIM_FLAG_UPDATE;
        count++;
        Encoder.count = count;
    }
    
    if(TIM4->CR1 & TIM_CR1_DIR)
        Encoder.dir = 1;
    else
        Encoder.dir = 0;
        
    if(Encoder.dir ^ dir_last)
        Speed_PID.I_Sum = 0;
    
    dir_last = Encoder.dir;
}


void Encoder_SpeedFilter(void)
{
	static uint8_t i;
	static float speed_mean,speed[10];

	
	/* 速度滤波 */
	speed[i++] = Encoder.speed;		// 0 1 2 3 4 5 6 7 8 9
	if(i == 10)
	{
		i = 0;
		speed_mean = 0;
		BubbleSort(&speed[0],10);
		for(uint8_t j=2; j<8; j++)
			speed_mean += speed[i];
		speed_mean /= 6.0f;
	}
	Encoder.speed_filter = 0.5f*speed_mean+0.5f*Encoder.speed;	
}


void Encoder_SpeedCalc(void)
{
	static float cnt,cnt_last,count,count_last,speed_last;
	cnt = (float)TIM4->CNT;
	count = (float)Encoder.count;
	
	Encoder.speed = (float)(cnt-cnt_last+(count-count_last)*4000)*2*PI;
	
	if(Encoder.speed > 500 || Encoder.speed <-500)
		Encoder.speed = speed_last;
	
	cnt_last = cnt;
	count_last = count;
	speed_last = Encoder.speed;
}

void Encoder_SpeedRamp(void)
{
	/* 速度环 */
	if(Encoder.flag == 1)
	{
		// S曲线处理
		Motor_SCurve_Process();  // 处理S曲线规划
		
		// 使用S曲线输出作为最终速度指令
		rtX.Speed_ref = Motor_SCurve_GetOutput();
		
		// 注释掉位置环，现在用S曲线进行速度规划
		// Angle_PID_Calc(rtX.Angle_ref,Encoder.angle_mach,&rtX.Speed_ref,&Angle_PID);
		
		// 保留速度死区处理
		if(rtX.Speed_ref > 0 && rtX.Speed_ref < 5)
			rtX.Speed_ref = 5;
		else if(rtX.Speed_ref < 0 && rtX.Speed_ref > -5)
			rtX.Speed_ref = -5;
			
		// 速度环PI控制
		Speed_PID_Calc(rtX.Speed_ref,Encoder.speed_filter,&rtX.Iq_ref,&Speed_PID);
	}
}





void BubbleSort(float* speed, uint8_t length) 
{
    for (int i = 0; i < length - 1; i++) 
        for (int j = 0; j < length - 1 - i; j++) 
		{
            if (speed[j] > speed[j + 1])
			{
                float temp = speed[j];
                speed[j] = speed[j + 1];
                speed[j + 1] = temp;
            }
        }
}

void Encoder_ZeroAlignment(void)
{
	rtX.Udc = 24;
	rtX.Tpwm = 8400*2;
	Voltage_DQ.Vd = 1.5f;
	Voltage_DQ.Vq = 0.0f;
	
	for(uint32_t i=0; i<10000; i++)
	{
		Angle_To_Cos_Sin(0.0f, &Transf_Cos_Sin);								/* 计算三角函数 */
		Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta); 		/* Revpark变换 */ 
		SVPWM_Calc(Voltage_Alpha_Beta, rtX.Udc, rtX.Tpwm);      				/* SVPWM */
	}
	TIM4->CNT = 0;
}





