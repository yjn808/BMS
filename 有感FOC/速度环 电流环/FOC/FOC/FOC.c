#include "foc.h"


ExtX rtX;                                    // 系统输入参数结构体
ExtY rtY;                                    // 系统输出参数结构体
Transf_Cos_Sin_DEF Transf_Cos_Sin;           // 当前转子角度对应的正弦余弦值
Voltage_DQ_DEF Voltage_DQ;                   // dq坐标系下的期望电压指令
Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;   // αβ坐标系下的电压输出

Current_ABC_DEF Current_Iabc;                // 三相静止坐标系电流（UVW/ABC）
Current_Alpha_Beta_DEF Current_Ialpha_beta;  // αβ静止坐标系电流（Clarke变换结果）
Current_DQ_DEF Current_Idq;                  // dq旋转坐标系电流（Park变换结果）


float I_uvw[3];    // 电机三相电流
float power;       // 母线电压
float cnt;


uint16_t ADC_DMABuff[1]; // 母线电压
extern Current_PID_DEF Current_D_PID; // d轴电流PI控制器
extern Current_PID_DEF Current_Q_PID; // q轴电流PI控制器




// 坐标变换
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin)
{
    cos_sin->Cos = cosf(angle);
    cos_sin->Sin = sinf(angle);
}




//反Park变换
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta)
{
    v_alpha_beta->Valpha = cos_sin.Cos * v_dq.Vd - cos_sin.Sin * v_dq.Vq;
    v_alpha_beta->Vbeta  = cos_sin.Sin * v_dq.Vd + cos_sin.Cos * v_dq.Vq;
}




//Clarke变换（ABC → αβ）
void Clarke_Transf(Current_ABC_DEF Current_abc, Current_Alpha_Beta_DEF* Current_alpha_beta)
{
    Current_alpha_beta->Ialpha = Current_abc.Ia;
    Current_alpha_beta->Ibeta = (Current_abc.Ia + Current_abc.Ib * 2.0F) * 0.577350269F;  // 0.577350269 = 1/√3
}





//Park变换（αβ → dq）
void Park_Transf(Current_Alpha_Beta_DEF Current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* current_dq)
{
    current_dq->Id = Current_alpha_beta.Ialpha * cos_sin.Cos + Current_alpha_beta.Ibeta * cos_sin.Sin;  // d轴电流（励磁分量）
    current_dq->Iq = -Current_alpha_beta.Ialpha * cos_sin.Sin + Current_alpha_beta.Ibeta * cos_sin.Cos; // q轴电流（转矩分量）
}




// 空间矢量脉宽调制
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm)
{
    int sector;
    float Tx, Ty, T, Ta, Tb, Tc;
    
    // 扇区判断：N = 4C + 2B + A
    sector = 0;
    if (v_alpha_beta.Vbeta > 0.0F)                                                    
        sector = 1;
    if ((1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F)      
        sector += 2;
    if ((-1.73205078F * v_alpha_beta.Valpha - v_alpha_beta.Vbeta) / 2.0F > 0.0F)     
        sector += 4;
    
    // 基本矢量作用时间计算
    switch (sector) 
    {
        case 1: // 扇区I
            Tx = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            Ty = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            break;

        case 2: // 扇区II
            Tx = (1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            Ty = -(1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc);
            break;

        case 3: // 扇区III
            Tx = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            Ty = 1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc;
            break;

        case 4: // 扇区IV
            Tx = -(1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc);
            Ty = (-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc);
            break;

        case 5: // 扇区V
            Tx = 1.73205078F * v_alpha_beta.Vbeta * Tpwm / Udc;
            Ty = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            break;

        default: // 扇区VI
            Tx = -((1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            Ty = -((-1.5F * v_alpha_beta.Valpha + 0.866025388F * v_alpha_beta.Vbeta) * (Tpwm / Udc));
            break;
    }

    // 过调制处理
    T = Tx + Ty;
    if (T > Tpwm)
    {
        Tx = Tx * Tpwm / T;   
        Ty = Ty * Tpwm / T;
    }

    // 计算三相切换时间点
    Ta = (Tpwm - (Tx + Ty)) / 4.0F;  
    Tb = Tx / 2.0F + Ta;             
    Tc = Ty / 2.0F + Tb;             
    
    // 根据扇区分配三相PWM比较值
    switch (sector)
    {
        case 1:
            rtY.Tcmp1 = Tb;  // U相
            rtY.Tcmp2 = Ta;  // V相
            rtY.Tcmp3 = Tc;  // W相
            break;
            
        case 2:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Tb;
            break;
            
        case 3:
            rtY.Tcmp1 = Ta;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Tc;
            break;
            
        case 4:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Tb;
            rtY.Tcmp3 = Ta;
            break;
            
        case 5:
            rtY.Tcmp1 = Tc;
            rtY.Tcmp2 = Ta;
            rtY.Tcmp3 = Tb;
            break;
            
        case 6:
            rtY.Tcmp1 = Tb;
            rtY.Tcmp2 = Tc;
            rtY.Tcmp3 = Ta;
            break;
    }
}


// 电机初始化
void Motor_Init(void)
{  
    // 启动运放
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
    
    // ADC校准
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

    // 启动三相PWM
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
    
    // 启动ADC触发定时器
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);

    // 修改ADC启动方式 - 参考代码的做法
    HAL_ADCEx_InjectedStart_IT(&hadc1);  // 只启动ADC1中断
    HAL_ADCEx_InjectedStart(&hadc2);     // ADC2不启动中断
    HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC_DMABuff,1); 

    HAL_Delay(100);
	
    Encoder_Init();


	PID_Init();

    FOC_Init();

    rtX.Id_ref = 0.0f;
	rtX.Iq_ref = 1.5f;
	rtX.Speed_ref = 100;


}





void Motor_Run(void)
{
    static float theta;
    static uint8_t cnt = 0;
    static uint16_t IA_Offset = 0, IB_Offset = 0, IC_Offset = 0;
    static uint8_t ADC_offset = 0;
    
    /* 电机转向 */
    if(rtX.Dir == CW)
        theta += 0.005f;
    else if(rtX.Dir == CCW)
        theta -= 0.005f;

    if(theta > 2*PI)
        theta -= 2*PI;
    else if(theta < 0)
        theta += 2*PI;
    rtX.theta = theta;
    
    // 偏置校准
    if(ADC_offset == 0)
    {
        // 偏置校准阶段：前10次采样求平均
        cnt++;
        uint16_t adc_a = ADC1->JDR1;  // A相
        uint16_t adc_b = ADC2->JDR1;  // B相
        uint16_t adc_c = ADC1->JDR2;  // C相
        
        IA_Offset += adc_a;
        IB_Offset += adc_b;
        IC_Offset += adc_c;
        
        if(cnt >= 10)
        {
            ADC_offset = 1;
            IA_Offset = IA_Offset / 10;
            IB_Offset = IB_Offset / 10;
            IC_Offset = IC_Offset / 10;
        }
        // 校准期间不执行FOC，直接返回
        return;
    }
    else
    {
        // 正常工作阶段 - 减去动态偏置值
        uint16_t adc_a = ADC1->JDR1;
        uint16_t adc_b = ADC2->JDR1;
        uint16_t adc_c = ADC1->JDR2;
        
        I_uvw[0] = ((float)(adc_a - IA_Offset)) * SAMPLE_CURR_CON_FACTOR;  // A相
        I_uvw[1] = ((float)(adc_b - IB_Offset)) * SAMPLE_CURR_CON_FACTOR;  // B相  
        I_uvw[2] = ((float)(adc_c - IC_Offset)) * SAMPLE_CURR_CON_FACTOR;  // C相
    }


    Encoder.angle = (float)(TIM4->CNT%1000)/1000*2*PI;
	rtX.theta = Encoder.angle;

    /* 调用FOC */
    FOC_Run();
    
    // /* 设置PWM占空比 */
    TIM1->CCR1 = rtY.Tcmp1;
    TIM1->CCR2 = rtY.Tcmp2;
    TIM1->CCR3 = rtY.Tcmp3;

    power = ADC_DMABuff[0] * VBUS_CONVERSION_FACTOR;
}



void FOC_Init(void)
{
    rtX.Udc = 24;           // 直流母线电压 24V
    rtX.Tpwm = 8500 * 2;    // PWM周期计数值
    rtX.Dir = CW;           // 电机旋转方向：顺时针
    
    Voltage_DQ.Vd = rtX.Id_ref;   // d轴电压指令（励磁分量，通常为0）
    Voltage_DQ.Vq = rtX.Iq_ref;   // q轴电压指令（转矩分量）
}





// FOC主控制循环
void FOC_Run(void)
{
    // 获取三相电流采样值
    Current_Iabc.Ia = I_uvw[0];  // A相电流
    Current_Iabc.Ib = I_uvw[1];  // B相电流  
    Current_Iabc.Ic = I_uvw[2];  // C相电流
    
    // Clarke变换 (ABC → αβ)
    Clarke_Transf(Current_Iabc, &Current_Ialpha_beta);
    
    // 计算当前转子角度的三角函数值
    Angle_To_Cos_Sin(rtX.theta, &Transf_Cos_Sin);
    
    // Park变换 (αβ → dq)
    Park_Transf(Current_Ialpha_beta, Transf_Cos_Sin, &Current_Idq);
    

    Current_PID_Calc(rtX.Id_ref,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);
	Current_PID_Calc(rtX.Iq_ref,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID);



    // 反Park变换 (dq → αβ)
    Rev_Park_Transf(Voltage_DQ, Transf_Cos_Sin, &Voltage_Alpha_Beta);
    
    // 空间矢量PWM调制
    SVPWM_Calc(Voltage_Alpha_Beta, rtX.Udc, rtX.Tpwm);
}



void SysTick_ISR(void)
{	
	// 1ms间隔
	Encoder_SpeedCalc();

	Encoder_SpeedFilter();	
	
	Encoder_SpeedRamp();
}

































