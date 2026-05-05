/*
 * timing.h
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */

#ifndef APP_INCLUDE_TIMING_H_
#define APP_INCLUDE_TIMING_H_

#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"

void Init_Cmd(CMD *p);
void Timing_State_Auto_Update( PROT_INFO *p);
void Timing_State_Cmd_Running_Stop( PROT_INFO *p , CMD *v);

#endif /* APP_INCLUDE_TIMING_H_ */
