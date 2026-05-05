#include "PI.h"

Current_PID_DEF Current_D_PID;
Current_PID_DEF Current_Q_PID;
Speed_PID_DEF Speed_PID;
Angle_PID_DEF Angle_PID;


extern ExtX rtX;

void PID_Init(void)
{

	
	/* PI参数控制 */
	Current_D_PID.Kp = 1.0f;		
	Current_D_PID.Ki = 0.005f;		
	Current_D_PID.Up_Limit = 4;
	Current_D_PID.Dowm_Limit = -4;
	Current_D_PID.I_Sum = 0;
	
	/* PI参数控制 */
	Current_Q_PID.Kp = 1.0f;
	Current_Q_PID.Ki = 0.005f;
	Current_Q_PID.Up_Limit = 4;
	Current_Q_PID.Dowm_Limit = -4;
	Current_Q_PID.I_Sum = 0;

    /*速度环 PI参数控制 */
	Speed_PID.Kp = 0.01f;
	Speed_PID.Ki = 0.0001f;
	Speed_PID.Kb = 1.0f;
	Speed_PID.Up_Limit = 4;
	Speed_PID.Dowm_Limit = -4;
	Speed_PID.I_Sum = 0;

    /*位置环 PI参数控制 */
	Angle_PID.Kp = 18.0f;
	Angle_PID.Ki = 0.02f;
	Angle_PID.Kb = 1.0f;
	Angle_PID.Up_Limit = 50;
	Angle_PID.Dowm_Limit = -50;
	Angle_PID.I_Sum = 0;

}





void Current_PID_Calc(float ref, float input, float* output, Current_PID_DEF* current_pid)
{
	float err,temp;
	
	/* 计算误差 */
	err = ref - input;
	
	/* PI控制 */
	temp = current_pid->Kp * err + current_pid->Ki * current_pid->I_Sum;		// * 0.0001f
	
	/* 输出限幅 */
	if (temp > current_pid->Up_Limit) 			// temp 50  max 5		*output 5
		*output = current_pid->Up_Limit;
	else if (temp < current_pid->Dowm_Limit) 
		*output = current_pid->Dowm_Limit;
	else 
		*output = temp;
	
	/* 电流求和 抗饱和积分 */
	current_pid->I_Sum += err + (*output - temp) * current_pid->Kb;

}





void Speed_PID_Calc(float ref, float input, float* output, Speed_PID_DEF* speed_pid)
{
	float err,temp;
	
	/* 计算误差 */
	err = (ref - input);
	
	/* PI控制 */
	temp = speed_pid->Kp * err + speed_pid->Ki * speed_pid->I_Sum;		// * 0.0001f
	
	/* 输出限幅 */
	if (temp > speed_pid->Up_Limit) 			// temp 50  max 5		*output 5
		*output = speed_pid->Up_Limit;
	else if (temp < speed_pid->Dowm_Limit) 
		*output = speed_pid->Dowm_Limit;
	else 
		*output = temp;
	
	/* 电流求和 抗饱和积分 */
	speed_pid->I_Sum += err + (*output - temp) * speed_pid->Kb;

}


void Angle_PID_Calc(float ref, float input, float* output, Angle_PID_DEF* angle_pid)
{
	float err,temp;
	
	/* 计算误差 */
	err = (ref - input);
	if(err >= PI)
		err -= 2*PI;
	else  if(err < -PI)
		err += 2*PI;
	
	/* PI控制 */
	temp = angle_pid->Kp * err + angle_pid->Ki * angle_pid->I_Sum;		// * 0.0001f
	
	/* 输出限幅 */
	if (temp > angle_pid->Up_Limit) 			// temp 50  max 5		*output 5
		*output = angle_pid->Up_Limit;
	else if (temp < angle_pid->Dowm_Limit) 
		*output = angle_pid->Dowm_Limit;
	else 
		*output = temp;
	
	/* 电流求和 抗饱和积分 */
	angle_pid->I_Sum += err + (*output - temp) * angle_pid->Kb;

}





