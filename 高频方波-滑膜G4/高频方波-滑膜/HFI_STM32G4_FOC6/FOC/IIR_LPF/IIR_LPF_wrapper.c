#include "IIR_LPF_wrapper.h"
//结构体声明引用
IIR_Butter_DEF   HFI_Square_IIR_LPF2;//锁相环角速度输出滤波

IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ealfa;//SMO1输出滤波-低通
IIR_Butter_DEF   SMO_IIR_LPF_PAR_Ebeta;//SMO1输出滤波-低通
IIR_Butter_DEF   PLL_IIR_LPF_PAR;//PLL输出滤波-低通

void IIR_filter(float in ,float *out , IIR_Butter_DEF*d_iir_lpf)
{
      float temp;
	  // w(n) = gain*x(n) - a1*w(n-1) - a2*w(n-2)
temp = d_iir_lpf->gain0*in  
	             - (d_iir_lpf->status0*d_iir_lpf->a1) 
	               -(d_iir_lpf->status1*d_iir_lpf->a2);                
     //y(n) = gain[b0*w(n) + b1*w(n-1) + b2*w(n-2)]
*out = d_iir_lpf->gain1*(d_iir_lpf->b0*temp 
	                      + d_iir_lpf->b1*d_iir_lpf->status0 
	                         + d_iir_lpf->b2*d_iir_lpf->status1);

	 d_iir_lpf->status1 = d_iir_lpf->status0;
	 d_iir_lpf->status0 = temp;
}

void IIR_filter_Init(float temp[8],IIR_Butter_DEF*d_iir_lpf)
{

d_iir_lpf->b0 = temp[0];
d_iir_lpf->b1 = temp[1];	
d_iir_lpf->b2 = temp[2];
d_iir_lpf->a0 = temp[3];
d_iir_lpf->a1 = temp[4];	
d_iir_lpf->a2 = temp[5];
d_iir_lpf->gain0 = temp[6];
d_iir_lpf->gain1 = temp[7];	
d_iir_lpf->status0 = 0.0f;
d_iir_lpf->status1 = 0.0f;

}

void IIR_LPF_Start_wrapper(void)
{
	//1阶
//{1.0f,1.0f,0.0f,1.0f,-0.909929f,0.0f,0.045035f,1.0f};10khz  150h截止频率
//{1.0f,1.0f,0.0f,1.0f,-0.939063f,0.0f,0.030468f,1.0f};10khz  100h截止频率
//{1.0f,1.0f,0.0f,1.0f,-0.969067f,0.0f,0.015466f,1.0f};10khz  50h截止频率
//{1.0f,1.0f,0.0f,1.0f,-0.984414f,0.0f,0.007793f,1.0f};10khz  25h截止频率	
	//2阶
//{1.0f,2.0f,1.0f,1.0f,-1.8226949f,0.837181f,0.003621f,1.0f};10khz  200h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.8668922f,0.875215f,0.002080f,1.0f};10khz  150h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.9111970f,0.914975f,0.000944f,1.0f};10khz  100h截止频率
//{1.0f,2.0f,1.0f,1.0f,-1.9333802f,0.935529f,0.000537f,1.0f};10khz  75h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.9555782f,0.956543f,0.000241f,1.0f};10khz  50h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.9733442f,0.973694f,0.000087f,1.0f};10khz  30h截止频率	
//{1.0f,2.0f,1.0f,1.0f,-1.9777864f,0.978030f,0.000061f,1.0f};10khz  25h截止频率		
	
float HFI_Square_LPF_PLL2[8] ={1.0f,1.0f,0.0f,1.0f,-0.969067f,0.0f,0.015466f,1.0f};//做PI控制前的低通滤波器

float SMO_IIR_coeff[8] = {1.0f,1.0f,0.0f,1.0f,-0.969067f,0.0f,0.015466f,1.0f};//SMO输出的Ealfa Ebeta 滤波
float PLL_IIR_coeff[8] = {1.0f,1.0f,0.0f,1.0f,-0.984414f,0.0f,0.007793f,1.0f};//SMO PLL输出的We滤波

IIR_filter_Init(HFI_Square_LPF_PLL2, & HFI_Square_IIR_LPF2);//高频注入锁相环角速度输出

IIR_filter_Init(SMO_IIR_coeff, &SMO_IIR_LPF_PAR_Ealfa);//SMO_Ealfa
IIR_filter_Init(SMO_IIR_coeff, &SMO_IIR_LPF_PAR_Ebeta);//SMO_Ealfa
IIR_filter_Init(PLL_IIR_coeff, &PLL_IIR_LPF_PAR);//PLL
}




























