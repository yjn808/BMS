/*
 * sci_c.c
 *
 *  Created on: 2024年12月21日
 *      Author: MAO
 */



#include "sci.h"

void Init_Sci_a_Gpio()
{
    EALLOW;

    /* Enable internal pull-up for the selected pins */
    // Pull-ups can be enabled or disabled disabled by the user.
    // This will enable the pullups for the specified pins.
    GpioCtrlRegs.GPBPUD.bit.GPIO42 = 0;    // Enable pull-up for GPIO42 (SCITXDA)
    GpioCtrlRegs.GPBPUD.bit.GPIO43 = 0;    // Enable pull-up for GPIO43 (SCIRXDA)

    /* Configure SCI-A pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SCI functional pins.
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0x03;   // Configure GPIO55 for SCIRXDA operation
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0x03;   // Configure GPIO54 for SCITXDA operation
    GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 0x03;   // Configure GPIO55 for SCIRXDA operation
    GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 0x03;   // Configure GPIO54 for SCITXDA operation

    EDIS;
}

void Init_Sci_a()
{
    // Initialize SCI-A:
    SciaRegs.SCICCR.all = 0x0007;    // 1 stop bit,  No loopback
                                     // No parity,8 char bits,
                                     // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;   // enable TX, RX, internal SCICLK,
                                     // Disable RX ERR, SLEEP, TXWAKE
                                     // Keep Reset
    //
    // SCIA at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0036;//115200

    SciaRegs.SCIPRI.bit.FREESOFT = 3;  // sci normal run even if debug

    SciaRegs.SCIFFTX.all = 0xC040;// enable tx fifo;
                                  // disable interrupt
                                  // clear interrupt flag
                                  // count equla 0 ,the flag set   tx ok set flag
                                  // reset fifo point
    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1; //release fifo point

    SciaRegs.SCIFFRX.all = 0x4041;// enable rx fifo
                                  // disable interrupt
                                  // clear interrupt flag( OV & rx ready)
                                  // even one count no read ,the flag set
                                  // reset fifo point
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;//release fifo point

    SciaRegs.SCICTL1.bit.SWRESET = 1;// Relinquish SCI from Reset

}

void Init_Sci_b_Gpio()
{
    EALLOW;

    /* Enable internal pull-up for the selected pins */
    // Pull-ups can be enabled or disabled disabled by the user.
    // This will enable the pullups for the specified pins.
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;    // Enable pull-up for GPIO55 (SCIRXDB)
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;    // Enable pull-up for GPIO54 (SCITXDB)

    /* Configure SCI-B pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SCI functional pins.
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0x02;   // Configure GPIO55 for SCIRXDB operation
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0x02;   // Configure GPIO54 for SCITXDB operation
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 0x01;   // Configure GPIO55 for SCIRXDB operation
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 0x01;   // Configure GPIO54 for SCITXDB operation

    GpioCtrlRegs.GPBMUX1.bit.GPIO40  = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO40   = GPIO_OUTPUT;
    SCI_B_RX;

    EDIS;
}

void Init_Sci_b()
{
    // Initialize SCI-B:
    ScibRegs.SCICCR.all = 0x0007;    // 1 stop bit,  No loopback
                                     // No parity,8 char bits,
                                     // async mode, idle-line protocol
    ScibRegs.SCICTL1.all = 0x0003;   // enable TX, RX, internal SCICLK,
                                     // Disable RX ERR, SLEEP, TXWAKE
                                     // Keep Reset
    //
    // SCI at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    ScibRegs.SCIHBAUD.all = 0x0000;
    ScibRegs.SCILBAUD.all = 0x0036;
//    ScibRegs.SCIHBAUD.all = 0x0000;
//    ScibRegs.SCILBAUD.all = 0x0001;
    ScibRegs.SCIPRI.bit.FREESOFT = 3;  // sci normal run even if debug

    ScibRegs.SCIFFTX.all = 0xC040;// enable tx fifo;
                                  // disable interrupt
                                  // clear interrupt flag
                                  // count equla 0 ,the flag set   tx ok set flag
                                  // reset fifo point
    ScibRegs.SCIFFTX.bit.TXFIFORESET = 1; //release fifo point

    ScibRegs.SCIFFRX.all = 0x4041;// enable rx fifo
                                  // disable interrupt
                                  // clear interrupt flag( OV & rx ready)
                                  // even one count no read ,the flag set
                                  // reset fifo point
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;//release fifo point

    ScibRegs.SCICTL1.bit.SWRESET = 1;// Relinquish SCI from Reset

}

void Init_Sci_c_Gpio()
{
    EALLOW;

    /* Enable internal pull-up for the selected pins */
    // Pull-ups can be enabled or disabled disabled by the user.
    // This will enable the pullups for the specified pins.
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;    // Enable pull-up for GPIO56 (SCITXDC)
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;    // Enable pull-up for GPIO57 (SCIRXDC)

    /* Configure SCI-C pins using GPIO regs*/
    // This specifies which of the possible GPIO pins will be SCI functional pins.
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0x02;   // Configure GPIO57 for SCIRXDC operation
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0x02;   // Configure GPIO56 for SCITXDC operation
    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 0x01;   // Configure GPIO57 for SCIRXDC operation
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 0x01;   // Configure GPIO56 for SCITXDC operation

    GpioCtrlRegs.GPBMUX1.bit.GPIO41  = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO41   = GPIO_OUTPUT;
    SCI_C_RX;

    EDIS;
}

void Init_Sci_c()
{
    // Initialize SCI-C:
    ScicRegs.SCICCR.all = 0x0007;    // 1 stop bit,  No loopback
                                     // No parity,8 char bits,
                                     // async mode, idle-line protocol
    ScicRegs.SCICTL1.all = 0x0003;   // enable TX, RX, internal SCICLK,
                                     // Disable RX ERR, SLEEP, TXWAKE
                                     // Keep Reset
    //
    // SCI at 9600 baud
    // @LSPCLK = 50 MHz (200 MHz SYSCLK) HBAUD = 0x02 and LBAUD = 0x8B.
    // @LSPCLK = 30 MHz (120 MHz SYSCLK) HBAUD = 0x01 and LBAUD = 0x86.
    //
    ScicRegs.SCIHBAUD.all = 0x0002;
    ScicRegs.SCILBAUD.all = 0x008B;//9600

    ScicRegs.SCIPRI.bit.FREESOFT = 3;  // sci normal run even if debug

    ScicRegs.SCIFFTX.all = 0xC040;// enable tx fifo;
                                  // disable interrupt
                                  // clear interrupt flag
                                  // count equla 0 ,the flag set   tx ok set flag
                                  // reset fifo point
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1; //release fifo point

    ScicRegs.SCIFFRX.all = 0x4041;// enable rx fifo
                                  // disable interrupt
                                  // clear interrupt flag( OV & rx ready)
                                  // even one count no read ,the flag set
                                  // reset fifo point
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;//release fifo point

    ScicRegs.SCICTL1.bit.SWRESET = 1;// Relinquish SCI from Reset

}



