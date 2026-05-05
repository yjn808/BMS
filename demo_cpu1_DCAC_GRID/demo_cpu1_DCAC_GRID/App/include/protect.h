/*
 * protect.h
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */

#ifndef APP_INCLUDE_PROTECT_H_
#define APP_INCLUDE_PROTECT_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"


#define  I_PROT_RATE     0.2f
#define  V_PROT_RATE     0.2f

void Init_Protect( PROT_INFO *v );
void Protect_Flag_Auto_Update( POWER_INFO *p , PROT_INFO *v , DEV_INFO *r);
void Protect_Flag_Cmd_Clear( POWER_INFO *p , PROT_INFO *v , DEV_INFO *r , CMD *z);


#endif /* APP_INCLUDE_PROTECT_H_ */
