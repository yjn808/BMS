#ifndef __SCURVE_H__
#define __SCURVE_H__

#include "main.h"
#include "arm_math.h"  // 你已经有这个，足够了

struct S_CURVE_TYPE
{
	float32_t speedTar;
	float32_t lastspeedTar;
	
	float32_t accSet, a_accSet;
	
	float32_t accDir, accTar;
	
	float32_t a_accTemp;
	float32_t a_accStepTemp, d_accStepTemp;

	float32_t acc, j;
	float32_t accT;
	float32_t a_accStep, accStep;		
	
	float32_t out;
	
	uint16_t accCnt, accMaxCnt;
	
	uint8_t sta, acc_dec;

	float32_t gain, offset;
	uint8_t useOut;
};

// 电机速度规划专用S曲线实例
extern struct S_CURVE_TYPE motor_speed_sCurve;

// S曲线处理函数
void SCurveProcess(struct S_CURVE_TYPE *sCurve);

// 电机S曲线速度规划函数
void Motor_SCurve_SetTarget(float speed_target);   // 设置目标速度
float Motor_SCurve_GetOutput(void);                // 获取S曲线输出
void Motor_SCurve_SetParams(float max_acc, float jerk); // 设置S曲线参数
void Motor_SCurve_Process(void);                   // S曲线处理

#endif


