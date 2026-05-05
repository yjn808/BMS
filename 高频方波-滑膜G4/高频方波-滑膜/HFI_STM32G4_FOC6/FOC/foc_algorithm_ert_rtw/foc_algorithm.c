/**********************************
      
**********************************/

#include "foc_algorithm.h"


real32_T D_PI_I = 1680.8F;
real32_T D_PI_KB = 15.0F;
real32_T D_PI_LOW_LIMIT = -4.0F;
real32_T D_PI_P = 2.2F;
real32_T D_PI_UP_LIMIT = 4.0F;

real32_T Q_PI_I = 1680.8F;
real32_T Q_PI_KB = 15.0F;
real32_T Q_PI_LOW_LIMIT = -4.0F;
real32_T Q_PI_P = 2.2F;
real32_T Q_PI_UP_LIMIT = 4.0F;


FOC_INTERFACE_STATES_DEF FOC_Interface_states;


FOC_INPUT_DEF FOC_Input;


FOC_OUTPUT_DEF FOC_Output;


RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void stm32_ekf_Start_wrapper(real_T *xD);
  extern void stm32_ekf_Outputs_wrapper(const real32_T *u,
                                        real32_T *y,
                                        const real_T *xD);
  extern void stm32_ekf_Update_wrapper(const real32_T *u,
                                       real32_T *y,
                                       real_T *xD);
  extern void stm32_ekf_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void L_identification_Start_wrapper(real_T *xD);
  extern void L_identification_Outputs_wrapper(const real32_T *u,
                                               real32_T *y,
                                               const real_T *xD);
  extern void L_identification_Update_wrapper(const real32_T *u,
                                              real32_T *y,
                                              real_T *xD);
  extern void L_identification_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

#ifdef __cplusplus

extern "C" {
  
#endif
  
  extern void R_flux_identification_Start_wrapper(real_T *xD);
  extern void R_flux_identification_Outputs_wrapper(const real32_T *u,
                                                    real32_T *y,
                                                    const real_T *xD);
  extern void R_flux_identification_Update_wrapper(const real32_T *u,
                                                   real32_T *y,
                                                   real_T *xD);
  extern void R_flux_identification_Terminate_wrapper(real_T *xD);
  
#ifdef __cplusplus
  
}
#endif

extern float float_test1;
extern float float_test2;

CURRENT_ABC_DEF Current_Iabc;
CURRENT_ALPHA_BETA_DEF Current_Ialpha_beta;
VOLTAGE_ALPHA_BETA_DEF Voltage_Alpha_Beta;
TRANSF_COS_SIN_DEF Transf_Cos_Sin;
CURRENT_DQ_DEF Current_Idq; 
CURRENT_DQ_DEF Current_Lpf_Idq;
VOLTAGE_DQ_DEF Voltage_DQ;
CURRENT_PID_DEF Current_D_PID;
CURRENT_PID_DEF Current_Q_PID;

/***************************************
功能：Clark变换
形参：三相电流以及alpha_beta电流
说明：由三相互差120度变换到两相互差90度
***************************************/
void Clarke_Transf(CURRENT_ABC_DEF Current_abc_temp,CURRENT_ALPHA_BETA_DEF* Current_alpha_beta_temp)
{
  Current_alpha_beta_temp->Ialpha = (Current_abc_temp.Ia - (Current_abc_temp.Ib + Current_abc_temp.Ic) * 0.5F) * 2.0F / 3.0F;
  Current_alpha_beta_temp->Ibeta = (Current_abc_temp.Ib - Current_abc_temp.Ic) * 0.866025388F * 2.0F / 3.0F;
}
/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/
void SVPWM_Calc(VOLTAGE_ALPHA_BETA_DEF v_alpha_beta_temp,real32_T Udc_temp,real32_T Tpwm_temp)
{
  int32_T sector;
  real32_T Tcmp1,Tcmp2,Tcmp3,Tx,Ty,f_temp,Ta,Tb,Tc;
  sector = 0;
  Tcmp1 = 0.0F;
  Tcmp2 = 0.0F;
  Tcmp3 = 0.0F;
  if (v_alpha_beta_temp.Vbeta > 0.0F) {
    sector = 1;
  }
  
  if ((1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 2;
  }
  
  if ((-1.73205078F * v_alpha_beta_temp.Valpha - v_alpha_beta_temp.Vbeta) / 2.0F > 0.0F) {
    sector += 4;
  }
  
  switch (sector) {
  case 1:
    Tx = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 2:
    Tx = (1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    Ty = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    break;
    
  case 3:
    Tx = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    break;
    
  case 4:
    Tx = -(1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp);
    Ty = (-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp);
    break;
    
  case 5:
    Tx = 1.73205078F * v_alpha_beta_temp.Vbeta * Tpwm_temp / Udc_temp;
    Ty = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
    
  default:
    Tx = -((1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    Ty = -((-1.5F * v_alpha_beta_temp.Valpha + 0.866025388F * v_alpha_beta_temp.Vbeta) * (Tpwm_temp / Udc_temp));
    break;
  }
  
  f_temp = Tx + Ty;
  if (f_temp > Tpwm_temp) {
    Tx /= f_temp;
    Ty /= (Tx + Ty);
  }
  
  Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
  Tb = Tx / 2.0F + Ta;
  Tc = Ty / 2.0F + Tb;
  switch (sector) {
  case 1:
    Tcmp1 = Tb;
    Tcmp2 = Ta;
    Tcmp3 = Tc;
    break;
    
  case 2:
    Tcmp1 = Ta;
    Tcmp2 = Tc;
    Tcmp3 = Tb;
    break;
    
  case 3:
    Tcmp1 = Ta;
    Tcmp2 = Tb;
    Tcmp3 = Tc;
    break;
    
  case 4:
    Tcmp1 = Tc;
    Tcmp2 = Tb;
    Tcmp3 = Ta;
    break;
    
  case 5:
    Tcmp1 = Tc;
    Tcmp2 = Ta;
    Tcmp3 = Tb;
    break;
    
  case 6:
    Tcmp1 = Tb;
    Tcmp2 = Tc;
    Tcmp3 = Ta;
    break;
  }
  
  FOC_Output.Tcmp1 = Tcmp1;
  FOC_Output.Tcmp2 = Tcmp2;
  FOC_Output.Tcmp3 = Tcmp3;
}

/***************************************
功能：COS_SIN值计算
形参：角度以及COS_SIN结构体
说明：COS_SIN值计算
***************************************/
void Angle_To_Cos_Sin(real32_T angle_temp,TRANSF_COS_SIN_DEF* cos_sin_temp)
{
  cos_sin_temp->Cos = arm_cos_f32(angle_temp);
  cos_sin_temp->Sin = arm_sin_f32(angle_temp);
}
/***************************************
功能：PARK变换
形参：alpha_beta电流、COS_SIN值、DQ轴电流
说明：交流变直流
***************************************/
void Park_Transf(CURRENT_ALPHA_BETA_DEF current_alpha_beta_temp,TRANSF_COS_SIN_DEF cos_sin_temp,CURRENT_DQ_DEF* current_dq_temp)
{
  current_dq_temp->Id = current_alpha_beta_temp.Ialpha * cos_sin_temp.Cos + current_alpha_beta_temp.Ibeta * cos_sin_temp.Sin;
  current_dq_temp->Iq = -current_alpha_beta_temp.Ialpha * cos_sin_temp.Sin + current_alpha_beta_temp.Ibeta * cos_sin_temp.Cos;
}
/***************************************
功能：反PARK变换
形参：DQ轴电压、COS_SIN值、alpha_beta电压
说明：直流变交流
***************************************/
void Rev_Park_Transf(VOLTAGE_DQ_DEF v_dq_temp,TRANSF_COS_SIN_DEF cos_sin_temp,VOLTAGE_ALPHA_BETA_DEF* v_alpha_beta_temp)
{
  v_alpha_beta_temp->Valpha = cos_sin_temp.Cos * v_dq_temp.Vd - cos_sin_temp.Sin * v_dq_temp.Vq;
  v_alpha_beta_temp->Vbeta  = cos_sin_temp.Sin * v_dq_temp.Vd + cos_sin_temp.Cos * v_dq_temp.Vq;
}

/***************************************
功能：电流环PID
形参：电流参考、电流反馈、电压输出、PID结构体
说明：根据电流误差去调节电流输出
***************************************/
void Current_PID_Calc(real32_T ref_temp,real32_T fdb_temp,real32_T* out_temp,CURRENT_PID_DEF* current_pid_temp)
{
  real32_T error;
  real32_T temp;
  error = ref_temp - fdb_temp;
  temp = current_pid_temp->P_Gain * error + current_pid_temp->I_Sum;
  if (temp > current_pid_temp->Max_Output) 
  {
    *out_temp = current_pid_temp->Max_Output;
  } 
  else if (temp < current_pid_temp->Min_Output) 
  {
    *out_temp = current_pid_temp->Min_Output;
  } 
  else 
  {
    *out_temp = temp;
  }
  current_pid_temp->I_Sum += ((*out_temp - temp) * current_pid_temp->B_Gain + current_pid_temp->I_Gain * error) *FOC_PERIOD;
}



void foc_algorithm_step(void)
{

  Current_Iabc.Ia = FOC_Input.ia;         //三相电流赋值
  Current_Iabc.Ib = FOC_Input.ib;
  Current_Iabc.Ic = FOC_Input.ic;
//高频方波注入
  if(Uin_inject_start_off == 0)
	{
		//三相电流ABC --> Ialfa Ibeta
		Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK 变换

		//提取高频Ialfa Ibeta 电流
		hfi_quare_inject(Current_Ialpha_beta.Ialpha,Current_Ialpha_beta.Ibeta,&hfi_square_def);

		Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
		Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信

		//提取低频DQ轴电流
		Idq_L(Current_Idq.Id,Current_Idq.Iq ,&DQ_L_Def);
			 
		//电流闭环---使用提取出来的DQ轴基频分量作为闭环
	   Current_PID_Calc(FOC_Input.Id_ref,DQ_L_Def.id_out,&Voltage_DQ.Vd,&Current_D_PID);     //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
     Current_PID_Calc(FOC_Input.Iq_ref,DQ_L_Def.iq_out,&Voltage_DQ.Vq,&Current_Q_PID);     //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压
	 	//极性辨识
		if(Inject_time_flag1 == 1)
		{
		  HFI_NS_Direction(Current_Idq.Id,&Current_Idq.Id,&Voltage_DQ.Vd,&NS_D_H_def);
			
		}
    /*******极性识别--电压注入******/
		if( Voltage_DQ.Vq>2.0f || Voltage_DQ.Vq<-2.0f )
		{
			Uin_inject_ampl =  Voltage_DQ.Vq/1.2f;
			if(Uin_inject_ampl<0.0f)Uin_inject_ampl = -Uin_inject_ampl;
			
		}
		else
		{
			Uin_inject_ampl = 0.5f;
		}
		if(Inject_time_flag1 == 0)
		{
			if(Uin_inject_flag == 0)//注入正电压
			{

				Voltage_DQ.Vd+=Uin_inject_ampl;
				Uin_inject_flag = 1;
			}
			else
			{
				Voltage_DQ.Vd-=Uin_inject_ampl;
				Uin_inject_flag = 0;
			}
		} 
		
		//启动后100ms高频方波注入稳定--停止注入开始极性识别--识别完成后补偿角度继续开启高频注入
		if(Inject_time2==0)
		{
			 Inject_time1++;
			 if(Inject_time1>5000)
			 {
				 Inject_time_flag1 = 1;
				 Inject_time2 = 1;
			 }
	   }
		
		Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入
	 
    SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,FOC_Input.Tpwm);       //SVPWM 计算模块
	 //高频注入经过锁相环得到we theat
		hfi_square_PLL(hfi_square_def.Ialfa_h_out,hfi_square_def.Ibeta_h_out,&hfi_square_PLL_def);
		FOC_Input.theta  = hfi_square_PLL_def.theta;
	}
	//启动完成--使用滑膜观测器
  else
  {
		Clarke_Transf(Current_Iabc,&Current_Ialpha_beta);        //CLARK 变换
		Angle_To_Cos_Sin(FOC_Input.theta,&Transf_Cos_Sin);     //由角度计算 park变换和 反park变换的 COS SIN值
		Park_Transf(Current_Ialpha_beta,Transf_Cos_Sin,&Current_Idq);  //Park变换，由Ialpha Ibeta 与角度信息，去计算Id Iq  // 由交流信
		Current_PID_Calc(FOC_Input.Id_ref,Current_Idq.Id,&Voltage_DQ.Vd,&Current_D_PID);     //D轴电流环PID  根据电流参考与电流反馈去计算 输出电压
		Current_PID_Calc(FOC_Input.Iq_ref,Current_Idq.Iq,&Voltage_DQ.Vq,&Current_Q_PID);     //Q轴电流环PID  根据电流参考与电流反馈去计算 输出电压

		Rev_Park_Transf(Voltage_DQ,Transf_Cos_Sin,&Voltage_Alpha_Beta);                //反park变换  通过电流环得到的dq轴电压信息结合角度信息，去把直流信息转化为交流信息用于SVPWM的输入
	 
		//滑膜观测器参数输入
		SMO_Observer(Voltage_Alpha_Beta.Valpha,Voltage_Alpha_Beta.Vbeta,Current_Ialpha_beta.Ialpha,Current_Ialpha_beta.Ibeta,&SMO_Struct_def);
		
		SVPWM_Calc(Voltage_Alpha_Beta,FOC_Input.Udc,FOC_Input.Tpwm);       //SVPWM 计算模块
		
		PLL_control(SMO_Struct_def.Valfa,SMO_Struct_def.Vbeta,&PLL_def);//滑膜观测器输出		
	}
	

}


void foc_algorithm_initialize(void)
{
  //电流环PID 参数 初始化
  {
  Current_D_PID.P_Gain = D_PI_P;
  Current_D_PID.I_Gain = D_PI_I;
  Current_D_PID.B_Gain = D_PI_KB;
  Current_D_PID.Max_Output = D_PI_UP_LIMIT;
  Current_D_PID.Min_Output = D_PI_LOW_LIMIT;
  Current_D_PID.I_Sum = 0.0f;//注意积分值需要清零
  
  Current_Q_PID.P_Gain = Q_PI_P;
  Current_Q_PID.I_Gain = Q_PI_I;
  Current_Q_PID.B_Gain = Q_PI_KB;
  Current_Q_PID.Max_Output = Q_PI_UP_LIMIT;
  Current_Q_PID.Min_Output = Q_PI_LOW_LIMIT;
  Current_Q_PID.I_Sum = 0.0f;
  }
  speed_pid_initialize();  //速度环PID 参数 初始化             
  HFI_Square_Init();
  IIR_LPF_Start_wrapper();//IIR_LPF和磁链观测器参数初始化
	SMO_PLL_Init(&SMO_Struct_def, &PLL_def); //滑膜参数
  //Speed_Ref = 15;
  //状态变量初始化
  {
    real_T initVector[4] = { 0, 0, 0, 0 };
    
    {
      int_T i1;
      real_T *dw_DSTATE = &FOC_Interface_states.EKF_States[0];
      for (i1=0; i1 < 4; i1++) {
        dw_DSTATE[i1] = initVector[i1];
      }
    }
  }
 
  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        FOC_Interface_states.L_Ident_States = initVector[0];
      }
    }
  }
  

  {
    real_T initVector[1] = { 0 };
    
    {
      int_T i1;
      for (i1=0; i1 < 1; i1++) {
        FOC_Interface_states.R_flux_Ident_States = initVector[0];
      }
    }
  }
  

}

