/*
 * sci_b.h
 *
 *  Created on: 2024年12月21日
 *      Author: MAO
 */


#ifndef DSP_INCLUDE_SCI_H_
#define DSP_INCLUDE_SCI_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#define TXFIFO_MAX_BYTES    16 //  tx fifo max is 16

#define SCI_B_TX              GpioDataRegs.GPBSET.bit.GPIO40 = 1
#define SCI_B_RX              GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1

#define SCI_C_TX              GpioDataRegs.GPBSET.bit.GPIO41 = 1
#define SCI_C_RX              GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1


void Init_Sci_a_Gpio();
void Init_Sci_a();

void Init_Sci_b_Gpio();
void Init_Sci_b();

void Init_Sci_c_Gpio();
void Init_Sci_c();


#endif /* DSP_INCLUDE_SCI_H_ */
