/*
 * timing.c
 *
 *  Created on: 2025��5��17��
 *      Author: MAO
 */



#include "timing.h"

void Init_Cmd(CMD *p)
{
    p->Err_Flag_Rst_Cmd = false;
    p->Ctrl_Cmd = NO_CMD;
}


#pragma CODE_SECTION(Timing_State_Auto_Update,".TI.ramfunc");
void Timing_State_Auto_Update( PROT_INFO *p)
{
    if(  (p->Err_Flag.I_INV_HW_Err == true)  || (p->Err_Flag.I_INV_SW_Err == true)  || \
         (p->Err_Flag.V_DC_HW_Err == true)   ||   (p->Err_Flag.V_DC_SW_Err == true) || \
         (p->Err_Flag.V_AC_HW_Err == true)   ||   (p->Err_Flag.V_AC_SW_Err == true)
    )
    {
        p->Tim_Status = STOP;
    }
}

#pragma CODE_SECTION(Timing_State_Cmd_Running_Stop,".TI.ramfunc");
void Timing_State_Cmd_Running_Stop( PROT_INFO *p , CMD *v)
{
    if(v->Ctrl_Cmd == STOP_CMD)
    {
        p->Tim_Status = STOP;
    }
    else if(v->Ctrl_Cmd == START_CMD)
    {
        if(  (p->Err_Flag.I_INV_HW_Err == false) && (p->Err_Flag.I_INV_SW_Err == false)  &&\
             (p->Err_Flag.V_DC_HW_Err == false)   &&   (p->Err_Flag.V_DC_SW_Err == false)  &&\
             (p->Err_Flag.V_AC_HW_Err == false)   &&   (p->Err_Flag.V_AC_SW_Err == false)
        )
        {
            p->Tim_Status = RUNNING;
        }
    }
    v->Ctrl_Cmd = NO_CMD;
}

