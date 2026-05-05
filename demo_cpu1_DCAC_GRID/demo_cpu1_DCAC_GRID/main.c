

/**
 * main.c
 */

//
// Included Files
//
#include "F28x_Project.h"

#include "fastrts.h"
#include "fpu32/C28x_FPU_FastRTS.h"

#include <stdint.h>
#include <float.h>
#include "fpu.h"
#include <math.h>

#include "timer.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "epwm.h"
#include "cmpss.h"
#include "adc.h"
#include "sci.h"

#include "comm_display.h"
#include "ctrl.h"
#include "parameter.h"
#include "powercalc.h"
#include "protect.h"
#include "record.h"
#include "timing.h"
#include "sogi_spll.h"

#include "ui.h"

int main(void)
{

    Epwm_Cpu1_Sel();
    Epwm_Clk_Div_Sel();

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.

    InitSysCtrl();

// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.

    InitGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts

    DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.

    InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:

    IER = 0x0000;
    IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.

    InitPieVectTable();

// Enable NMI Interrupt
    NmiIntruptRegs.NMICFG.bit.NMIE=1;

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.ADCC1_INT = &adcc1_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitCpuTimers();   // For this example, only initialize the Cpu Timers
// 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer1, 200, TIMER_TS * 1e6 );

    Init_Led();        // initialize the led
    Init_Key();        // initialize the Key

    ConfigureADC();
    SetupADC_Epwm();

    Init_Cmpss_3();
    Init_Cmpss_2();
    Init_Cmpss_5();

    InitEPwm1Gpio();
    InitEPwm4Gpio();
    Init_Epwm1_Epwm4();

    I2cAGpioConfig(I2C_A_GPIO91_GPIO92);
    Init_I2CA();
    if ( Init_OLED() == I2C_ERROR    ||  OLED_Clear() == I2C_ERROR )
    {
        LED_GPIO_133_ON;
    }

    IER |= M_INT1;
    IER |= M_INT13;
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;

// Enable global Interrupts and higher priority real-time debug events:

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

// ui
    Init_Samp_Coff(&Samp_Coff);
    Init_Dev_Info(&Dev_Info);
    Init_Power_Calc(&Power_Info);
    Init_Protect(&Prot_Info);
    Init_Cmd(&Cmd);
    Init_CTRL(&Ctrl);
    Init_Repeat_Ctrl(&Repeat_Ctrl);

// app
//    Init_PI();
//    Init_Data_Record( &Data_Record );
//    Init_Sci_Com_Strcut(SCI_C , &Sci_C_Com);

    Init_PLL_Calc(&Sogi_PLL);

    Epwm_Count_Start();
    StartCpuTimer1();
    while(1)
    {


    }

}
