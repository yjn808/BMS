#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include <math.h>


extern float I_uvw[3];



#define PI				3.14159265358979f
#define CW				0
#define CCW				1

#define ADC_REF_V                   3.3f
#define ADC_RESOLUTION              4096.0f    // 12位ADC分辨率
#define VBUS_UP_RES                 75.0f      // 75kΩ上分压电阻
#define VBUS_DOWN_RES               3.0f       // 3kΩ下分压电阻  
#define VBUS_CONVERSION_FACTOR      (ADC_REF_V*(VBUS_UP_RES+VBUS_DOWN_RES)/VBUS_DOWN_RES/ADC_RESOLUTION)
#define SAMPLE_RES                  0.005f     // 5mΩ分流电阻
#define AMP_GAIN                    (1.0f+11.0f/1.5f)      // 运放增益约8.333倍
#define SAMPLE_CURR_CON_FACTOR      (ADC_REF_V/ADC_RESOLUTION/AMP_GAIN/SAMPLE_RES)  // 电流转换系数

// 系统输入参数结构体
typedef struct
{
	float theta;  		/* 转子角度 */
	float Udc; 			/* 电机供电电压 */
	float Tpwm; 		/* 电机PWM周期 */  
	uint8_t	Dir;		/* 电机转向 */
	float Id_ref; 		/* D轴电流参考值 */ 
	float Iq_ref; 		/* Q轴电流参考值 */ 
	float Speed_ref; 	/* 速度参考值 */ 
	float Angle_ref; 	/* 角度参考值 */ 
} ExtX;



// dq坐标系电压
typedef struct
{
	float Vd;			// d轴电压
	float Vq;			// q轴电压
} Voltage_DQ_DEF;

// 角度三角函数值
typedef struct
{
	float Cos;			// 转子角度Cos值
	float Sin;			// 转子角度Sin值
} Transf_Cos_Sin_DEF;

// αβ坐标系电压
typedef struct
{
	float Valpha;		// Alpha轴电压
	float Vbeta; 		// Beta轴电压
} Voltage_Alpha_Beta_DEF;

// 系统输出参数结构体
typedef struct
{
	float Tcmp1;		// U相PWM比较值
	float Tcmp2;		// V相PWM比较值
	float Tcmp3;		// W相PWM比较值
} ExtY;

// 三相电流
typedef struct
{
	float Ia;			// A相电流
	float Ib;     		// B相电流
	float Ic;     		// C相电流
} Current_ABC_DEF;

// αβ坐标系电流
typedef struct
{
	float Ialpha;		// Alpha轴电流
	float Ibeta;		// Beta轴电流
} Current_Alpha_Beta_DEF;

// dq坐标系电流
typedef struct
{
	float Id;			// d轴电流（励磁分量）
	float Iq;			// q轴电流（转矩分量）
} Current_DQ_DEF;

/* 外部调用变量 */
extern ExtX rtX;
extern Transf_Cos_Sin_DEF Transf_Cos_Sin;
extern Voltage_DQ_DEF Voltage_DQ;
extern Voltage_Alpha_Beta_DEF Voltage_Alpha_Beta;

extern Current_ABC_DEF Current_Iabc;
extern Current_Alpha_Beta_DEF Current_Ialpha_beta;
extern Current_DQ_DEF Current_Idq;





// 函数声明
void FOC_Init(void);
void FOC_Run(void);
void Motor_Init(void);
void Motor_Run(void);
void Angle_To_Cos_Sin(float angle, Transf_Cos_Sin_DEF* cos_sin);
void Rev_Park_Transf(Voltage_DQ_DEF v_dq, Transf_Cos_Sin_DEF cos_sin, Voltage_Alpha_Beta_DEF* v_alpha_beta);
void SVPWM_Calc(Voltage_Alpha_Beta_DEF v_alpha_beta, float Udc, float Tpwm);
void Clarke_Transf(Current_ABC_DEF Current_abc, Current_Alpha_Beta_DEF* Current_alpha_beta);
void Park_Transf(Current_Alpha_Beta_DEF Current_alpha_beta, Transf_Cos_Sin_DEF cos_sin, Current_DQ_DEF* current_dq);
void SysTick_ISR(void);

#endif



