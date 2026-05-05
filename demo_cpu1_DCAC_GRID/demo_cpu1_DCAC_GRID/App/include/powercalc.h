/*
 * powercalc.h
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */

#ifndef APP_INCLUDE_POWERCALC_H_
#define APP_INCLUDE_POWERCALC_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"
#include <basic.h>

#include "ctrl.h"

#define  POWERCALC_LPF_F_C     10.0f

#define  N                     800

typedef struct{
    float data_in;
    float data_RMS_out;
    float data_OFFSET_out;

    float data_RMS_sum;
    float data_OFFSET_sum;

    unsigned int count_samp;
}RMS_OFFSET;

void Init_Power_Calc(POWER_INFO *p);
void Power_Calc(SAMPLE *v , POWER_INFO *p);
void RMS_Offset_cla(RMS_OFFSET *v);


#endif /* APP_INCLUDE_POWERCALC_H_ */
