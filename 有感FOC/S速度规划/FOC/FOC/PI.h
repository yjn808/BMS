#ifndef __PI_H__
#define __PI_H__

#include "main.h"



typedef struct
{
	float Kp;				/* Kp */
	float Ki;				/* Ki */
	float Kb;				/* Kb */
	float Up_Limit;			/* 输出上限 */
	float Dowm_Limit;		/* 输出下限 */
	float I_Sum;			/* 电流积分 */
} Current_PID_DEF;


typedef struct
{
	float Kp;				/* Kp */
	float Ki;				/* Ki */
	float Kb;				/* Kb */
	float Up_Limit;			/* 输出上限 */
	float Dowm_Limit;		/* 输出下限 */
	float I_Sum;			/* 电流积分 */
} Speed_PID_DEF;


typedef struct
{
	float Kp;				/* Kp */
	float Ki;				/* Ki */
	float Kb;				/* Kb */
	float Up_Limit;			/* 输出上限 */
	float Dowm_Limit;		/* 输出下限 */
	float I_Sum;			/* 电流积分 */
} Angle_PID_DEF;


extern Current_PID_DEF Current_D_PID;
extern Current_PID_DEF Current_Q_PID;
extern Speed_PID_DEF Speed_PID;
extern Angle_PID_DEF Angle_PID;

void PID_Init(void);
void Current_PID_Calc(float ref, float input, float* output, Current_PID_DEF* current_pid);
void Speed_PID_Calc(float ref, float input, float* output, Speed_PID_DEF* speed_pid);
void Angle_PID_Calc(float ref, float input, float* output, Angle_PID_DEF* angle_pid);



#endif


