#ifndef _IIR_LPF_wrapper_H
#define _IIR_LPF_wrapper_H

#include "main.h"


typedef struct IIR
{
//	float in_status0;
//	float in_status1;
//	
//	
//  float out_status0;
//	float out_status1;
	float status0;
	float status1;
	
	float b0;
	float b1;
	float b2;
	
	float a0;
	float a1;
	float a2;

	
	float gain0;
	float gain1;
	
	
}IIR_Butter_DEF;
typedef struct EFK_2
{
	float Last_P;//前序协方差
	float Now_P;//当前协方差
	float sing_out;//滤波结果
	float EKF_Kg;//卡尔曼增益
	float EKF_Q;//背景白噪声
	float EKF_R;//器件方差
	
}EFK2_DEF;



extern EFK2_DEF EFK2_def;



extern IIR_Butter_DEF   HFI_Square_IIR_LPF2;//锁相环角速度输出滤波
extern IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ealfa;//SMO1输出滤波-低通
extern IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ebeta;//SMO1输出滤波-低通
extern IIR_Butter_DEF   PLL_IIR_LPF_PAR;//PLL输出滤波-低通



void IIR_filter(float in ,float *out , IIR_Butter_DEF*d_iir_lpf);
void IIR_LPF_Start_wrapper(void);
void EKF_Figure(EFK2_DEF*EFK2_Def,float *signer);
#endif

