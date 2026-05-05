#ifndef _hfi_square_wave_H
#define _hfi_square_wave_H

#include "main.h"


typedef struct hfi_square
{
   float Ialfa_h;
	 float Ialfa_hL;
	 float Ialfa_h_out;
	
	 float Ialfa_h_Last;
	 float Ialfa_h_LLast;
	
	
	float Ibeta_h;
	float Ibeta_hL;
	float Ibeta_h_out;
	 
	float Ibeta_h_Last;
	float Ibeta_h_LLast;


}hfi_square_DEF;

typedef struct hfi_square_pll
{
   float err_sum;
	 float err_P;
	 float err_I;
   float we;
	 float theta;

} hfi_square_PLL_DEF;
typedef struct DQ_H_L
{
   float id_L;
	 float id_LL;
	 
   float iq_L;
	 float iq_LL;
	
	float id_out;
	float iq_out;

}DQ_L_DEF;
typedef struct NS_D_H
{
   float NS_id_h;
	 float NS_id_hL;
	 float NS_id_out;
	 float NS_out_test;
	 
}NS_D_H_DEF;





extern float NS_theta;
//高频注入标志
extern unsigned char Uin_inject_flag;
//高频注入开启关闭标志
extern unsigned char Uin_inject_start_off;
//极性辨识参
extern unsigned char NS_flag;
extern float Uin_inject_ampl ;
//结构体声明
extern DQ_L_DEF DQ_L_Def;
extern hfi_square_PLL_DEF hfi_square_PLL_def;
extern hfi_square_DEF hfi_square_def;
extern NS_D_H_DEF NS_D_H_def;
//高频方波注入计时
extern uint16_t Inject_time1;
extern uint16_t Inject_time2;
extern uint16_t Inject_time_flag1;
//函数声明
void hfi_quare_inject(float Ialfa,float Ibeta,hfi_square_DEF*hfi_square_Def);
void hfi_square_PLL(float Ialfa_h,float Ibeta_h,hfi_square_PLL_DEF*hfi_square_PLL_Def);
void Idq_L(float Id,float Iq ,DQ_L_DEF*DQ_L_Def);
void HFI_NS_Direction(float Id,float*FOC_Q,float*FOC_D,NS_D_H_DEF*NS_D_H_Def);
void Uin_inject(float *D,float D_inject_V);
void HFI_Square_Init(void);
#endif

