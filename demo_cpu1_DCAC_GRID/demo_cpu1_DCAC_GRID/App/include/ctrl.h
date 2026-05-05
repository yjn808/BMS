/*
 * ctrl.h
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */

#ifndef APP_INCLUDE_CTRL_H_
#define APP_INCLUDE_CTRL_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"

#include "basic.h"

void Init_CTRL(CTRL *p);
void Init_PI(void);

typedef struct{

    float I_DC;
    float I_Inv_L;
    float I_Inv_G;
    float V_AC;
    float V_DC;

}SAMPLE;


typedef struct{

    float Input;
    float Output;

    float Gain;
    float Input_Buffer[800];
    float Output_Buffer[800];

    unsigned int Count_Samp;
    unsigned int K_Step;

}REPEAT_CTRL;


void Init_Repeat_Ctrl(REPEAT_CTRL *p);
void AdcSam(SAMPLE *v, SAMP_COEFF *p);
void PI_Ctrl_Out_Loop(void);
void PWM_Modulation_Ctrl(void);



extern PI_CTRL PI_Out_Loop;
extern SAMPLE Sample;
extern REPEAT_CTRL Repeat_Ctrl;
#endif /* APP_INCLUDE_CTRL_H_ */
