/*
 * timer.c
 *
 *  Created on: 2024��12��21��
 *      Author: MAO
 */


#include "timer.h"
#include "led.h"
#include "ctrl.h"
#include "comm_display.h"
#include "protect.h"
#include "timing.h"
// cpu_timer0_isr - CPU Timer0 ISR with interrupt counter

__interrupt void cpu_timer0_isr(void)
{

// Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

#pragma CODE_SECTION(cpu_timer1_isr,".TI.ramfunc");
__interrupt void cpu_timer1_isr(void)
{
    GPIO_67_SET;
    EINT;  // Enable Global interrupt INTM
//    Sci_Tx_Rx( &Sci_C_Com );
//    parseJSON( &Sci_C_Com );
//
    Protect_Flag_Cmd_Clear( &Power_Info , &Prot_Info , &Dev_Info , &Cmd);
    Timing_State_Cmd_Running_Stop( &Prot_Info , &Cmd);
//
//    PI_Ctrl_Out_Loop();

    GPIO_67_RESET;
}
