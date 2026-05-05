/*
 * adc.h
 *
 *  Created on: 2025Äê5ÔÂ15ÈÕ
 *      Author: MAO
 */

#ifndef DSP_INCLUDE_ADC_H_
#define DSP_INCLUDE_ADC_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

void ConfigureADC(void);
void SetupADC_Epwm(void);
interrupt void adcc1_isr(void);

#endif /* DSP_INCLUDE_ADC_H_ */
