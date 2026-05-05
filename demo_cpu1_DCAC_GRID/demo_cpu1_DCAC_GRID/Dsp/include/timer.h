/*
 * timer.h
 *
 *  Created on: 2024Äê12ÔÂ12ÈÕ
 *      Author: MAO
 */

#ifndef DSP_INCLUDE_TIMER_H_
#define DSP_INCLUDE_TIMER_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#define TIMER_TS      0.000025f*100.0f  // 2500us

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
#endif /* DSP_INCLUDE_TIMER_H_ */
