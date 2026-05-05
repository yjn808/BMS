/*
 * key.c
 *
 *  Created on: 2024Äê12ÔÂ21ÈÕ
 *      Author: MAO
 */




#include "key.h"

void Init_Key()
{
    EALLOW;
    GpioCtrlRegs.GPAMUX2.bit.GPIO28  = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO28   = GPIO_INPUT;

    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO29   = GPIO_INPUT;

    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO30   = GPIO_INPUT;

    GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO36   = GPIO_INPUT;

    EDIS;
}
