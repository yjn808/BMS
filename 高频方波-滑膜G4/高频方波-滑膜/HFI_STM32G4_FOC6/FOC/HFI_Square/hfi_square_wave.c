#include "hfi_square_wave.h"

//结构体初始化
hfi_square_DEF hfi_square_def;
hfi_square_PLL_DEF hfi_square_PLL_def;
DQ_L_DEF DQ_L_Def;
NS_D_H_DEF NS_D_H_def;
//高频注入标志
unsigned char Uin_inject_flag;
//高频注入开启关闭标志
unsigned char Uin_inject_start_off;
//高频注入幅值
float Uin_inject_ampl;
//极性辨识参
unsigned char NS_flag;
int   NS_cnt;
float NS_Sum1;
float NS_Sum2;
float NS_theta;
float Ns_sum_theta;
//高频方波注入计时
uint16_t Inject_time1;
uint16_t Inject_time2;
uint16_t Inject_time_flag1;
/*
*函数作用：提取高频注入的电流
*输入参数1：Ialfa Ibeta 电流
*输入参数2：接受提取出来的高频电流结构体
*
*
*/
void hfi_quare_inject(float Ialfa,float Ibeta,hfi_square_DEF*hfi_square_Def)
{
	//二阶差分提取提取出高频注入的电流
  hfi_square_Def->Ialfa_h = (hfi_square_Def->Ialfa_h_LLast - (hfi_square_Def->Ialfa_h_Last*2) + Ialfa)*0.25f;
  hfi_square_Def->Ibeta_h = (hfi_square_Def->Ibeta_h_LLast - (hfi_square_Def->Ibeta_h_Last*2) + Ibeta)*0.25f;
	
		//迭代数据
	hfi_square_Def->Ialfa_h_LLast = hfi_square_Def->Ialfa_h_Last;
	hfi_square_Def->Ibeta_h_LLast = hfi_square_Def->Ibeta_h_Last;
	hfi_square_Def->Ialfa_h_Last  = Ialfa;
	hfi_square_Def->Ibeta_h_Last  = Ibeta;
	
	//将（-1）k次方消掉----因为注入在后，这里需要取反一下
	if(Uin_inject_flag == 1)//注入正电压
	{
	  hfi_square_Def->Ialfa_h_out = (hfi_square_Def->Ialfa_h - hfi_square_Def->Ialfa_hL);
		hfi_square_Def->Ibeta_h_out = (hfi_square_Def->Ibeta_h - hfi_square_Def->Ibeta_hL);
 
	}
	else//注入负电压
	{
	 hfi_square_Def->Ialfa_h_out = (hfi_square_Def->Ialfa_h - hfi_square_Def->Ialfa_hL)*-1;
	 hfi_square_Def->Ibeta_h_out = (hfi_square_Def->Ibeta_h - hfi_square_Def->Ibeta_hL)*-1;
	}
	//迭代
	hfi_square_Def->Ialfa_hL = hfi_square_Def->Ialfa_h;
	hfi_square_Def->Ibeta_hL = hfi_square_Def->Ibeta_h;

}
/*
*函数作用：将提取出来的高频电流，使用PLL锁相环控制，得到we theta
*输入参数1：Ialfa_h Ibeta_h 提取出来的高频电流
*输入参数2：接收输出的we theta
*
*
*/
void hfi_square_PLL(float Ialfa_h,float Ibeta_h,hfi_square_PLL_DEF*hfi_square_PLL_Def)
{
  float err;
	//PID计算
	err = -Ialfa_h*arm_sin_f32(hfi_square_PLL_Def->theta)  + (Ibeta_h*arm_cos_f32(hfi_square_PLL_Def->theta));	 
	
	hfi_square_PLL_Def->we = hfi_square_PLL_Def->err_P * err + hfi_square_PLL_Def->err_sum;
	 	if(Speed_Ref > 0.0f)
	 {
	   if(hfi_square_PLL_Def->we <0.0f)hfi_square_PLL_Def->we = -hfi_square_PLL_Def->we;
		 //if(err <0.0f)err = -err;
	 }
	 else if(Speed_Ref < 0.0f)
	 {
	   if(hfi_square_PLL_Def->we >0.0f)hfi_square_PLL_Def->we = -hfi_square_PLL_Def->we;
		 //if(err >0.0f)err = -err;
	 }
 
	hfi_square_PLL_Def->err_sum += hfi_square_PLL_Def->err_I * err *FOC_PERIOD;
	
 //角速度积分-->角度
	Ns_sum_theta+= hfi_square_PLL_Def->we * FOC_PERIOD ;
 //极性识别完成-判断是否需要补偿一个pi
   if(NS_theta >1.0f)
	 {
	    Ns_sum_theta+= hfi_square_PLL_Def->we * FOC_PERIOD +NS_theta;
		  NS_theta = 0.0f;
	 }
  //角度输出限幅
	if(Ns_sum_theta> 2.0f*PI)
	{
	  Ns_sum_theta -=2.0f*PI;
	}
  	if(Ns_sum_theta< 0.0f*PI)
	{
	  Ns_sum_theta+=2.0f*PI;
	}
	hfi_square_PLL_Def->theta  = Ns_sum_theta;
	//角速度滤波--可以在IIR_LPF_wrapper.c里面去修改滤波器参数值，减小速度波动
	IIR_filter(hfi_square_PLL_Def->we ,&hfi_square_PLL_Def->we , &HFI_Square_IIR_LPF2);
}


/*
*函数作用：提取出DQ轴的低频电流，除去包含的高频电流，后面用于电流闭环
*输入参数1：D Q轴电流
*输入参数2：接收输出的Q_L D_L
*
*
*/
void Idq_L(float Id,float Iq ,DQ_L_DEF*DQ_L_Def)
{
	//提取DQ的信号
  DQ_L_Def->id_out =  (Id + ( DQ_L_Def->id_L*2 ) + DQ_L_Def->id_LL)*0.25f;
	DQ_L_Def->iq_out =  (Iq + ( DQ_L_Def->iq_L*2 ) + DQ_L_Def->iq_LL)*0.25f;
	//迭代
	DQ_L_Def->id_LL  = DQ_L_Def->id_L;
	DQ_L_Def->id_L   = Id;
	
	DQ_L_Def->iq_LL  = DQ_L_Def->iq_L;
	DQ_L_Def->iq_L   = Iq;
	

}
/*
*函数作用：NS极性辨识
*输入参数1：没有除去高频时的D轴电流
*输入参数2：输出FOC的D轴电流
*
*
*/
void HFI_NS_Direction(float Id,float*FOC_Q,float*FOC_D,NS_D_H_DEF*NS_D_H_Def)
{
	if(NS_flag == 0) 
	{
		   //提取高频信号
		  NS_D_H_Def->NS_id_out = (Id -  (NS_D_H_Def->NS_id_h*2.0f) +  NS_D_H_Def->NS_id_hL)*0.25f;
			NS_D_H_Def->NS_id_hL = NS_D_H_Def->NS_id_h;
			NS_D_H_Def->NS_id_h = Id;	
			NS_cnt ++;
		  //*FOC_Q = 0.0f;
		  NS_D_H_Def->NS_out_test = NS_D_H_Def->NS_id_out;
		  NS_D_H_Def->NS_id_out=*FOC_Q ;
		
			//从0--0.1ms 先稳定 使D轴输出为0
			if( NS_cnt <= 400) 
			{
				*FOC_D = 0.0F;
			}
			//开始注入 正向偏置电流
			else if( NS_cnt>400 &&  NS_cnt<=800)//正电压  400
			{
					 *FOC_D = 0.5F;				
			}
			else if( NS_cnt>800&& NS_cnt<=1800)//正电压稳定，采集数据 1000
			{
				   *FOC_D = 0.5F;	
					if(NS_D_H_Def->NS_id_out<0.0f) NS_D_H_Def->NS_id_out = -NS_D_H_Def->NS_id_out;
					 //NS_Sum1 += (NS_D_H_Def->NS_id_out*2.0f);
				   NS_Sum1 += NS_D_H_Def->NS_id_out;
			}
			else if( NS_cnt>1800 &&  NS_cnt<=2200)//给0电压 400
			{
					 *FOC_D = 0.0F;
					
			}
			
			else if( NS_cnt>2200 &&  NS_cnt<=2600)//给负电压 400
			{
					 *FOC_D = -0.5F;				
			}
			else if( NS_cnt>2600 &&  NS_cnt<=3600)//负电压稳定，采集数据 1000
			{
				    *FOC_D = -0.5F;
						 if(NS_D_H_Def->NS_id_out<0.0f) NS_D_H_Def->NS_id_out = -NS_D_H_Def->NS_id_out;
					  //NS_Sum2+= (NS_D_H_Def->NS_id_out*2.0f);
				      NS_Sum2 += NS_D_H_Def->NS_id_out;
			}
			else if(NS_cnt>3600 && NS_cnt <=4000)//稳点 400
			{
			       *FOC_D = 0.0F;	
			}
			else
			{
				  
					if(NS_Sum1 > NS_Sum2) NS_theta = 0.0F;
					else NS_theta  = PI;
				  *FOC_D = 0.0F;				
				   NS_flag = 1;	
           Inject_time_flag1 = 0;	
			}

   }
}


void Uin_inject(float *D,float D_inject_V)
{


	if(Uin_inject_flag == 0)//注入正电压
	{
		 *D+=D_inject_V;
		 Uin_inject_flag = 1;
	}
	else
	{
	   *D-=D_inject_V;
		Uin_inject_flag = 0;
	}

}



void HFI_Square_Init(void)
{

//高频注入标志
Uin_inject_flag = 0;
//高频注入开启关闭标志
 Uin_inject_start_off = 0;
//高频注入幅值
Uin_inject_ampl =0.5f;
//极性辨识参
	NS_flag= 0;
	NS_cnt= 0;
	NS_Sum1= 0;
	NS_Sum2= 0;
	NS_theta= 0;
	
	hfi_square_def.Ialfa_h = 0.0f;
	hfi_square_def.Ialfa_hL = 0.0f;
	hfi_square_def.Ialfa_h_Last= 0.0f;
	hfi_square_def.Ialfa_h_LLast= 0.0f;
	hfi_square_def.Ialfa_h_out= 0.0f;
	hfi_square_def.Ibeta_h= 0.0f;
	hfi_square_def.Ibeta_hL= 0.0f;
	hfi_square_def.Ibeta_h_Last= 0.0f;
	hfi_square_def.Ibeta_h_LLast= 0.0f;
	hfi_square_def.Ibeta_h_out= 0.0f;

	hfi_square_PLL_def.err_I = 9000.0f;
	hfi_square_PLL_def.err_P= 550.0f;
	hfi_square_PLL_def.err_sum= 0.0f;
	hfi_square_PLL_def.theta= 0.0f;
	hfi_square_PLL_def.we =  0.0f;

	DQ_L_Def.id_L =  0.0f;
	DQ_L_Def.id_LL=  0.0f;
	DQ_L_Def.id_out=  0.0f;
	DQ_L_Def.iq_L=  0.0f;
	DQ_L_Def.iq_LL=  0.0f;
	DQ_L_Def.iq_out=  0.0f;
	
	
}


