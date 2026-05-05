#ifndef _Foc_Control_H
#define _Foc_Control_H
#include "main.h"
extern uint16_t funcation_up_date;
extern float Ia_test;
extern float Ib_test;
extern float Ic_test;
extern float Vbus;
extern float arg_step;
extern  uint8_t get_offset_flag ;
extern uint8_t start_current;
extern uint32_t A_offset,B_offset,C_offset;
extern uint8_t hfi_to_flux_ob_flag;
extern unsigned char hfi_square_to_ekf_flag;


#define PWM_TIM_CLOCK       170000000

#define PWM_TIM_FREQ        10000         //15KHZ
#define PWM_TIM_PULSE       (PWM_TIM_CLOCK/(2*PWM_TIM_FREQ)) // 6000
#define PWM_TIM_PULSE_TPWM  (PWM_TIM_CLOCK/(PWM_TIM_FREQ))//12000

#define FOC_PERIOD          0.0001F


#define MOTOR_STARTUP_CURRENT   1.0f   //电机启动电流，根据自己实际负载设置 
#define SPEED_LOOP_CLOSE_RAD_S  20.0f  //速度环切入闭环的速度  单位: rad/s

//有感FOC 或 无感FOC选择，总得注释掉其中一个
//#define HALL_FOC_SELECT          //此行注释掉就不使用有感FOC运行
#define SENSORLESS_FOC_SELECT    //此行注释掉就不使用无感感FOC运行

void motor_run(void);
void get_offset(uint32_t *a_offset,uint32_t *b_offset,uint32_t *c_offset);
void usart_down_date(void);

#endif


