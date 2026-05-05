/*
 * key.h
 *
 *  Created on: 2024Äê12ÔÂ21ÈÕ
 *      Author: MAO
 */

#ifndef DSP_INCLUDE_KEY_H_
#define DSP_INCLUDE_KEY_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

void Init_Key();

#define KEY_GPIO_28_PRESS      GpioDataRegs.GPADAT.bit.GPIO28 == 0
#define KEY_GPIO_29_PRESS      GpioDataRegs.GPADAT.bit.GPIO29 == 0
#define KEY_GPIO_30_PRESS      GpioDataRegs.GPADAT.bit.GPIO30 == 0
#define KEY_GPIO_36_PRESS      GpioDataRegs.GPBDAT.bit.GPIO36 == 0

#endif /* DSP_INCLUDE_KEY_H_ */
