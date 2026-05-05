/*
 * led.h
 *
 *  Created on: 2024Äê12ÔÂ21ÈÕ
 *      Author: MAO
 */

#ifndef DSP_INCLUDE_LED_H_
#define DSP_INCLUDE_LED_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

void Init_Led();


#define LED_GPIO_94_OFF          GpioDataRegs.GPCSET.bit.GPIO94 = 1
#define LED_GPIO_94_ON           GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1
#define LED_GPIO_94_TOGGLE       GpioDataRegs.GPCTOGGLE.bit.GPIO94 = 1

#define LED_GPIO_99_OFF          GpioDataRegs.GPDSET.bit.GPIO99 = 1
#define LED_GPIO_99_ON           GpioDataRegs.GPDCLEAR.bit.GPIO99 = 1
#define LED_GPIO_99_TOGGLE       GpioDataRegs.GPDTOGGLE.bit.GPIO99 = 1

#define LED_GPIO_133_OFF         GpioDataRegs.GPESET.bit.GPIO133 = 1
#define LED_GPIO_133_ON          GpioDataRegs.GPECLEAR.bit.GPIO133 = 1
#define LED_GPIO_133_TOGGLE      GpioDataRegs.GPETOGGLE.bit.GPIO133 = 1


#define GPIO_67_SET              GpioDataRegs.GPCSET.bit.GPIO67 = 1
#define GPIO_67_RESET            GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1
#define GPIO_67_TOGGLE           GpioDataRegs.GPCTOGGLE.bit.GPIO67 = 1

#define GPIO_68_SET              GpioDataRegs.GPCSET.bit.GPIO68 = 1
#define GPIO_68_RESET            GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1
#define GPIO_68_TOGGLE           GpioDataRegs.GPCTOGGLE.bit.GPIO68 = 1



#endif /* DSP_INCLUDE_LED_H_ */
