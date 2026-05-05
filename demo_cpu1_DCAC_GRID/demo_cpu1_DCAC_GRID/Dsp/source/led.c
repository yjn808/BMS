/*
 * led.c
 *
 *  Created on: 2024Äê12ÔÂ21ÈÕ
 *      Author: MAO
 */



#include "led.h"

void Init_Led()
{
    EALLOW;
    GpioCtrlRegs.GPCMUX2.bit.GPIO94  = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO94   = GPIO_OUTPUT;

    GpioCtrlRegs.GPDMUX1.bit.GPIO99 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO99   = GPIO_OUTPUT;

    GpioCtrlRegs.GPEMUX1.bit.GPIO133 = 0;
    GpioCtrlRegs.GPEDIR.bit.GPIO133   = GPIO_OUTPUT;

    GpioCtrlRegs.GPCMUX1.bit.GPIO67  = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO67   = GPIO_OUTPUT;

    GpioCtrlRegs.GPCMUX1.bit.GPIO68  = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO68   = GPIO_OUTPUT;

    LED_GPIO_94_OFF;
    LED_GPIO_99_OFF;
    LED_GPIO_133_OFF;

    GPIO_67_RESET;
    GPIO_68_RESET;

    EDIS;

}

