/*
 * epwm.h
 *
 *  Created on: 2025
 */

#ifndef DSP_INCLUDE_EPWM_H_
#define DSP_INCLUDE_EPWM_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#define TB_TBCLK    100E6  // 100Mhz
#define FREQ_EPWM   20000  // 20k hz
#define FREQ_SAMP   40000  // 40k hz
#define TB_TBPRD    2500

#define DETIM_1US  100
#define ET_ENABLE  0x1

#define DUTY_MIN    0.08f //2us
#define DUTY_MAX    0.92f //2us
#define ISR_TS      0.000025f  // 25us

void Epwm_Cpu1_Sel(void);
void Epwm_Clk_Div_Sel(void);
void Epwm_Count_Start(void);
void Epwm_Count_Stop(void);

void Init_Epwm1_Epwm4(void);

void epwm_output_enable(void);
void epwm_output_disable(void);

interrupt void epwm1_isr(void);


#define  TZ_OST_CLEAR                                                                 \
        do                                                                            \
        {                                                                             \
        EALLOW;                                                                       \
        EPwm1Regs.TZCLR.bit.OST = 1;                                                  \
        EPwm4Regs.TZCLR.bit.OST = 1;                                                  \
        EDIS;                                                                         \
        }while(0)

#define  A_DUTY2CMP(duty)       ( EPwm1Regs.CMPA.bit.CMPA        = (Uint16)(duty * TB_TBPRD) )
#define  B_DUTY2CMP(duty)       ( EPwm4Regs.CMPA.bit.CMPA        = (Uint16)(duty * TB_TBPRD) )

#define  COUNT_UP             ( EPwm1Regs.TBSTS.bit.CTRDIR == 1 )

#endif /* DSP_INCLUDE_EPWM_H_ */
