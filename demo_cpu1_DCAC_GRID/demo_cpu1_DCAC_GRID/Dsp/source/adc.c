/*
 * adc.c
 *
 *  Created on: 2025Äê5ÔÂ15ÈÕ
 *      Author: MAO
 */


#include "adc.h"

#include "led.h"
#include "ctrl.h"
#include "ui.h"
#include "protect.h"
#include "timing.h"
#include "powercalc.h"
#include "record.h"
#include "sogi_spll.h"

/*       565ns=355+210;
 *            0             1
 *      A   A4(V_AC)    A1(I_INV_G)
 *
 *      B   B1(I_DC)    B2(I_INV_L)
 *
 *      C   C3(V_DC)      VREFLOC
 *
 *      D      *            *
 */



void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADC
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

void SetupADC_Epwm(void)
{

    Uint16 acqps;

    //
    // Determine minimum acquisition window (in SYSCLKS) based on resolution
    //
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 70; //355ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    /*       565ns=355+210;
     *            0             1
     *      A   A4(V_AC)    A1(I_INV_G)
     *
     *      B   B1(I_DC)    B2(I_INV_L)
     *
     *      C   C3(V_DC)     VREFLOC
     *
     *      D      *            *
     */
    //
    //Select the channels to convert and end of conversion flag
    //
    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;  //A4(V_AC)
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //A1(I_INV_G)

    AdcbRegs.ADCSOC0CTL.bit.CHSEL  = 1;  //B1(I_DC)
    AdcbRegs.ADCSOC1CTL.bit.CHSEL  = 2;  //B2(I_INV_L)

    AdccRegs.ADCSOC0CTL.bit.CHSEL  = 3; //C3(V_DC)
    AdccRegs.ADCSOC1CTL.bit.CHSEL  = 8;  //VREFLOC

    AdcdRegs.ADCSOC0CTL.bit.CHSEL  = 14;  //test
    AdcdRegs.ADCSOC1CTL.bit.CHSEL  = 15;  //test


    AdcaRegs.ADCSOC0CTL.bit.ACQPS  = acqps;    //sample window is acqps +1 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.ACQPS  = acqps;    //sample window is acqps +1 SYSCLK cycles

    AdcbRegs.ADCSOC0CTL.bit.ACQPS  = acqps;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS  = acqps;

    AdccRegs.ADCSOC0CTL.bit.ACQPS  = acqps;
    AdccRegs.ADCSOC1CTL.bit.ACQPS  = acqps;

    AdcdRegs.ADCSOC0CTL.bit.ACQPS  = acqps;
    AdcdRegs.ADCSOC1CTL.bit.ACQPS  = acqps;


    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    //trigger on ePWM1 SOCA
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;    //trigger on ePWM1 SOCA

    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;

    AdccRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;

    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;

    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;       //1:ADCINT1 is enabled   0:disable
    AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 0;    // no continue mode
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1;     // EOC1 is trigger for ADCINT1
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;     //make sure INT1 flag is cleared

    EDIS;

}

#pragma CODE_SECTION(adcc1_isr,".TI.ramfunc");
interrupt void adcc1_isr(void)
{
    GPIO_68_SET;
#ifdef CPU1
    AdcSam(&Sample, &Samp_Coff);
    Power_Calc(&Sample , &Power_Info);

    Protect_Flag_Auto_Update( &Power_Info , &Prot_Info , &Dev_Info);
    Timing_State_Auto_Update( &Prot_Info);

//    Record_Cmd_Auto_Update( &Data_Record  , &Prot_Info , &Ctrl);
//    Data_Auto_Record( &Data_Record , &Sample );
//    PI_Ctrl_In_Loop();

    PLL_Calc( &Sogi_PLL , &Sample );

    PWM_Modulation_Ctrl();
#endif


#ifdef CPU2

#endif

    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;//make sure INT1 flag is cleared

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    GPIO_68_RESET;
}





