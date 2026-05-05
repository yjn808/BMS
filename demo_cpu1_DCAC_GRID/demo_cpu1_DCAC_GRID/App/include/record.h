/*
 * record.h
 *
 *  Created on: 2025��7��25��
 *      Author: MAO
 */

#ifndef APP_INCLUDE_RECORD_H_
#define APP_INCLUDE_RECORD_H_


#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "ui.h"
#include "ctrl.h"

#define  DATA_LENGTH    4000
#define  RECORD_ERR     1.0f

typedef struct {

    bool Record_CMD;
    Uint16 Record_Index;

    float V_INV_Ref_Last;
    float In_Loop_Ref_Last;
    float Out_Loop_Ref_Last;

    enum TIM_STAT Tim_Status_Last;

    float Data_A[DATA_LENGTH];
    float Data_B[DATA_LENGTH];


} DATA_RECORD;

extern DATA_RECORD Data_Record;

void Init_Data_Record( DATA_RECORD *p );
void Record_Cmd_Auto_Update( DATA_RECORD *p  , PROT_INFO *v , CTRL *r);
void Data_Auto_Record( DATA_RECORD *p , SAMPLE *v );

#endif /* APP_INCLUDE_RECORD_H_ */
