/*
 * parameter.h
 *
 *  Created on: 2025Äê5ÔÂ17ÈÕ
 *      Author: MAO
 */

#ifndef APP_INCLUDE_PARAMETER_H_
#define APP_INCLUDE_PARAMETER_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"

#define V_AC_RATED      24.0f
#define V_FRE_RATED     50.0f
#define V_DC_RATED      50.0f

#define I_RATED         3.0f

#define L_R             0.05f

#define V_DC_GAIN        0.01351f
#define V_DC_OFFSET      -0.8065f

#define V_AC_GAIN        0.04193f
#define V_AC_OFFSET      -85.14f

#define I_GAIN        0.003063f
#define I_OFFSET      -6.332f

void Init_Samp_Coff(SAMP_COEFF *v);
void Init_Dev_Info(DEV_INFO *v);

#endif /* APP_INCLUDE_PARAMETER_H_ */
