/*
 * record.c
 *
 *  Created on: 2025��7��25��
 *      Author: MAO
 */



#include "record.h"
#include <string.h>

#pragma DATA_SECTION(Data_Record,"record_data");
DATA_RECORD Data_Record;

void Init_Data_Record( DATA_RECORD *p )
{
    memset( (void *) (p) , 0 , sizeof( DATA_RECORD ) );
    p->Record_CMD = false;
    p->Record_Index = 0;

    p->Tim_Status_Last = STOP;
}

#pragma CODE_SECTION(Record_Cmd_Auto_Update,".TI.ramfunc");
void Record_Cmd_Auto_Update( DATA_RECORD *p  , PROT_INFO *v , CTRL *r)
{
    if(p->Record_CMD != true)
    {
        if( p->Tim_Status_Last != v->Tim_Status )
        {
            p->Record_CMD = true;
        }

        if( r->Ctrl_Loop == OPEN_LOOP && fabsf(p->V_INV_Ref_Last - r->V_INV_Ref) > RECORD_ERR )
        {
            p->Record_CMD = true;
        }

        if( r->Ctrl_Loop == CLOSE_IN_LOOP && fabsf(p->In_Loop_Ref_Last - r->In_Loop_Ref) > RECORD_ERR )
        {
            p->Record_CMD = true;
        }

        if( r->Ctrl_Loop == CLOSE_IN_OUT_LOOP && fabsf(p->Out_Loop_Ref_Last - r->Out_Loop_Ref) > RECORD_ERR )
        {
            p->Record_CMD = true;
        }
    }

    p->V_INV_Ref_Last = r->V_INV_Ref;
    p->In_Loop_Ref_Last = r->In_Loop_Ref;
    p->Out_Loop_Ref_Last = r->Out_Loop_Ref;

    p->Tim_Status_Last = v->Tim_Status;
}
#pragma CODE_SECTION(Data_Auto_Record,".TI.ramfunc");
void Data_Auto_Record( DATA_RECORD *p , SAMPLE *v )
{
    if(p->Record_CMD == true)
    {
        p->Data_A[ p->Record_Index ] = v->I_Inv;
        p->Data_B[ p->Record_Index ] = v->V_L;
        p->Record_Index++;
        if( p->Record_Index == DATA_LENGTH )
        {
            p->Record_CMD = false;
            p->Record_Index = 0;
        }
    }
}

