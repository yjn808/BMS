/*
 * sogi_spll.h
 *
 *  Created on: 2025Äê7ÔÂ27ÈÕ
 *      Author: MAO
 */

#ifndef APP_INCLUDE_SOGI_SPLL_H_
#define APP_INCLUDE_SOGI_SPLL_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"
#include "ctrl.h"

typedef struct {
    float in;
    float out_alpha;
    float out_beta;

    float W_cen;
    float Ts;


    float gain_alpha;
    float gain_beta;

    float coeff_recursive[2];

    float in_alpha_last;
    float in_alpha_last_last;

    float out_alpha_last;
    float out_alpha_last_last;

    float in_beta_last;
    float in_beta_last_last;

    float out_beta_last;
    float out_beta_last_last;

} SOGI;



typedef struct {
    float V_grid;  // input

    float V_theta; // output
    float V_frq;

    SOGI V_grid_sogi;
    VECTOR V_grid_vector;
    PI_CTRL V_grid_dq_pi;

    float V_theta_cos[10];
    float V_theta_sin[10];

} PLL;

extern PLL Sogi_PLL;

void Init_PLL_Calc(PLL *p);
void PLL_Calc( PLL *p , SAMPLE *v );
#endif /* APP_INCLUDE_SOGI_SPLL_H_ */
