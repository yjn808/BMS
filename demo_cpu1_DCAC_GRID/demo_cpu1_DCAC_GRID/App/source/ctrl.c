/*
 * ctrl.c
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */


#include "ctrl.h"
#include "epwm.h"
#include "parameter.h"
#include <string.h>
#include "timer.h"
#include "sogi_spll.h"

PI_CTRL PI_Out_Loop;
REPEAT_CTRL Repeat_Ctrl;
SAMPLE Sample;

void Init_CTRL(CTRL *p)
{
    memset( (void *) (p) , 0 , sizeof( CTRL) );

    p->Ctrl_Loop = CLOSE_IN_LOOP;//默认使用单电流内环模式（CLOSE_IN_LOOP）
    p->Dead_Time_Comp_Enable = false;
    p->Feed_Ford_Enable = false;

    p->In_Loop_Kp = 3.0f;

    p->Out_Loop_Kp = 0.0f;
    p->Out_Loop_Ki = 0.0f;

}

void Init_PI(void)
{


}


void Init_Repeat_Ctrl(REPEAT_CTRL *p)
{
    memset( (void *) (p) , 0 , sizeof( REPEAT_CTRL ) );

    p->K_Step = 15;//超前步数，对应超前角（用于相位补偿）
    p->Gain = 0.95f;//收敛因子，小于1保证稳定性

}



/*       565ns=355+210;
 *            0             1
 *      A   A4(V_AC)    A1(I_INV_G)
 *
 *      B   B1(I_DC)    B2(I_INV_L)
 *
 *      C   C3(V_DC)     VREFLOC
 *
 *      D      *            *
 */

#pragma CODE_SECTION(AdcSam,".TI.ramfunc");



void AdcSam(SAMPLE *v, SAMP_COEFF *p)
{
    v->V_AC      = AdcaResultRegs.ADCRESULT0 * p->V_AC_Gain      +  p->V_AC_Offset;
    v->I_DC      = AdcbResultRegs.ADCRESULT0 * p->I_DC_Gain      +  p->I_DC_Offset;
    v->V_DC      = AdccResultRegs.ADCRESULT0 * p->V_DC_Gain      +  p->V_DC_Offset;

    v->I_Inv_G   = AdcaResultRegs.ADCRESULT1 * p->I_Inv_G_Gain   +  p->I_Inv_G_Offset;
    v->I_Inv_L   = AdcbResultRegs.ADCRESULT1 * p->I_Inv_L_Gain   +  p->I_Inv_L_Offset;
}


#pragma CODE_SECTION(PI_Ctrl_Out_Loop,".TI.ramfunc");


void PI_Ctrl_Out_Loop(void)
{

}

float duty_A=0.0f;
float duty_B=0.0f;
float V_INV_Ref=0.0f;
float err=0.0f;
#pragma CODE_SECTION(PWM_Modulation_Ctrl,".TI.ramfunc");
void PWM_Modulation_Ctrl(void)
{

    unsigned int Input_count_samp = 0;

    if(Prot_Info.Tim_Status==RUNNING)
    {
//        P_ref × cosθ：有功电流参考的瞬时值（与电网同相）
//        Q_ref × sinθ：无功电流参考的瞬时值（超前90°）
//        两者相加构成总瞬时电流参考，减去实测桥臂电流得到误差
        err = Ctrl.P_In_Loop_Ref * Sogi_PLL.V_theta_cos[1] + Ctrl.Q_In_Loop_Ref * Sogi_PLL.V_theta_sin[1] - Sample.I_Inv_L ;
    }
    else
    {
        err = 0.0f;
    }

    //V_INV_Ref = err × In_Loop_Kp 用误差乘以比例系数，得到初步的逆变器电压参考
    V_INV_Ref =  err * Ctrl.In_Loop_Kp ;

    Repeat_Ctrl.Input = err;

    //当前采样点的基础指针(0 到 799) + 相位超前补偿步数
    Input_count_samp = Repeat_Ctrl.Count_Samp + Repeat_Ctrl.K_Step;

    //40kHz 采样率和 50Hz 电网频率下，一个工频周期正好包含 800 个采样点
    if(Input_count_samp > 799)
    {
        Input_count_samp -=800;
    }

    //上一个工频周期（20ms 前）同一时刻的控制器输出值 + 上一个周期同一时刻的误差（并且已经加上了 K_Step 提前读取，做了相位补偿）
    Repeat_Ctrl.Output = Repeat_Ctrl.Output_Buffer[Repeat_Ctrl.Count_Samp] + Repeat_Ctrl.Input_Buffer[ Input_count_samp];
    Repeat_Ctrl.Output *=Repeat_Ctrl.Gain;//Q 滤波器（遗忘因子）：保证系统稳定

    //覆盖掉旧数据
    Repeat_Ctrl.Output_Buffer[Repeat_Ctrl.Count_Samp] = Repeat_Ctrl.Output;
    Repeat_Ctrl.Input_Buffer[Repeat_Ctrl.Count_Samp] = Repeat_Ctrl.Input;

    //环形队列循环
    Repeat_Ctrl.Count_Samp++;
    if(Repeat_Ctrl.Count_Samp==800)
    {
        Repeat_Ctrl.Count_Samp=0;
    }

    V_INV_Ref +=  Repeat_Ctrl.Output ;


    //电网瞬时电压值 = 锁相环提取出来的电网基波瞬时相位角余弦值 * 电网交流电压有效值
    V_INV_Ref +=  Sogi_PLL.V_theta_cos[1] * Power_Info.V_AC_RMS * SQRT_2;

    //__divf32(分子, 分母)  占空比 = 0.5（1 +/- 目标参考电压/直流母线电压)
    duty_A = __divf32( V_INV_Ref *  0.5f , Sample.V_DC ) + 0.5f;
    duty_B = __divf32( V_INV_Ref * -0.5f , Sample.V_DC ) + 0.5f;

    if(Ctrl.Dead_Time_Comp_Enable==true)
    {
        if( Sample.I_Inv_L >0.3f )
        {
            if(COUNT_UP)
            {
                duty_A+= 0.04f;
            }
            if(!COUNT_UP)
            {
                duty_B-= 0.04f;
            }
        }
        if( Sample.I_Inv_L < -0.3f )
        {
            if(!COUNT_UP)
            {
                duty_A-= 0.04f;
            }
            if(COUNT_UP)
            {
                duty_B+= 0.04f;
            }
        }
    }

    //限幅
    duty_A = ( ( duty_A < DUTY_MIN ? DUTY_MIN : (duty_A > DUTY_MAX ? DUTY_MAX:duty_A) ) );
    duty_B = ( ( duty_B < DUTY_MIN ? DUTY_MIN : (duty_B > DUTY_MAX ? DUTY_MAX:duty_B) ) );

    A_DUTY2CMP(duty_A);
    B_DUTY2CMP(duty_B);

    if( Prot_Info.Tim_Status==RUNNING )
    {
         epwm_output_enable();
    }
    else
    {
         epwm_output_disable();
    }
}







