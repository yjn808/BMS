/*
 * cmpss.c
 *
 *  Created on: 2025
 *      Author: MAO
 *  Description: This file contains initialization functions for Comparator Subsystem (CMPSS) modules 2, 3 and 5.
 *               Configurations include comparator input selection, DAC reference voltage setup, digital filtering
 *               parameters, hysteresis configuration, output inversion control, and routing of comparator outputs
 *               to dedicated GPIO pins via Output XBAR. All configured modules operate without ramp generators
 *               and use direct DAC value assignment (shadow registers are not utilized).
 */


#include "cmpss.h"


/**
 * @brief Initializes CMPSS 3 module for current monitoring applications
 *
 * Configures CMPSS 3 with the following key parameters:
 * - DAC functionality enabled with direct value assignment (no shadow registers)
 * - Ramp generator disabled (runs in free mode without software control)
 * - High/low comparator negative inputs tied to DAC output
 * - Digital filtering enabled for noise reduction on comparator outputs
 * - Comparator hysteresis set to 4x the base hysteresis voltage (9.67032967032967 mV * 4)
 * - Output inversion configured for low comparator (enabled) and high comparator (disabled)
 * - Filtered comparator outputs routed to GPIO pins via Output XBAR
 * - DAC reference values set relative to midpoint (2048) using current limit thresholds
 *
 * @note DAC values are calculated as midpoint (2048) ¡À I_INVL_MAX for over/under current detection
 */
void Init_Cmpss_3()       // No ramp generator; no DAC shadow register
{
    // Enable access to protected registers
    EALLOW;

    // Enable DAC functionality for the comparator subsystem
    Cmpss3Regs.COMPCTL.bit.COMPDACE  =  CMPSS_ENABLE;        // Enable DAC for comparison

    // Configure comparator input sources
    Cmpss3Regs.COMPCTL.bit.COMPHSOURCE  =  NEGIN_DAC;        // High comparator negative input = DAC output
    Cmpss3Regs.COMPCTL.bit.COMPLSOURCE  =  NEGIN_DAC;        // Low comparator negative input = DAC output

    // Configure output inversion
    Cmpss3Regs.COMPCTL.bit.COMPHINV  =  0;        // Disable inversion for high comparator output
    Cmpss3Regs.COMPCTL.bit.COMPLINV  =  1;        // Enable inversion for low comparator output

    // Configure trip signal routing (use filtered outputs)
    Cmpss3Regs.COMPCTL.bit.CTRIPOUTLSEL  =  CTRIP_FILTER;        // CTRIPOUTL = filtered low comparator output
    Cmpss3Regs.COMPCTL.bit.CTRIPLSEL     =  CTRIP_FILTER;        // CTRIPL = filtered low comparator output
    Cmpss3Regs.COMPCTL.bit.CTRIPOUTHSEL  =  CTRIP_FILTER;        // CTRIPOUTH = filtered high comparator output
    Cmpss3Regs.COMPCTL.bit.CTRIPHSEL     =  CTRIP_FILTER;        // CTRIPH = filtered high comparator output

    // Set comparator hysteresis (9.67032967032967 mV * 4)
    Cmpss3Regs.COMPHYSCTL.bit.COMPHYS = 4 ;     // Hysteresis multiplier = 4

    // Configure DAC reference and ramp generator behavior
    Cmpss3Regs.COMPDACCTL.bit.FREESOFT = 2 ;    // Ramp generator runs freely (no software control)
    Cmpss3Regs.COMPDACCTL.bit.SELREF = REFERENCE_VDDA;        // Use VDDA as DAC reference voltage

    // Configure low-side digital filter parameters
    Cmpss3Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss3Regs.CTRIPLFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss3Regs.CTRIPLFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss3Regs.CTRIPLFILCTL.bit.FILINIT = 1;       // Initialize and start the low-side filter

    // Configure high-side digital filter parameters
    Cmpss3Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss3Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss3Regs.CTRIPHFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss3Regs.CTRIPHFILCTL.bit.FILINIT = 1;       // Initialize and start the high-side filter

    // Set DAC reference values (midpoint ¡À current limit)
    Cmpss3Regs.DACHVALS.bit.DACVAL = 2048 + I_INVL_MAX;    // High comparator reference (upper current limit)
    Cmpss3Regs.DACLVALS.bit.DACVAL = 2048 - I_INVL_MAX;    // Low comparator reference (lower current limit)

    // Wait for DAC and filters to stabilize
    DELAY_US(100);

    // Clear comparator status latches
    Cmpss3Regs.COMPSTSCLR.bit.HLATCHCLR = 1;       // Clear high comparator status latch
    Cmpss3Regs.COMPSTSCLR.bit.LLATCHCLR = 1;       // Clear low comparator status latch

    // Configure CTRIPOUTH routing to GPIO
    OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX4 = 0;        // Map CTRIPOUTH to OUTPUTXBAR2
    OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX4 = 1;          // Enable OUTPUTXBAR2 MUX4

    // Configure CTRIPOUTL routing to GPIO
    OutputXbarRegs.OUTPUT3MUX0TO15CFG.bit.MUX5 = 0;        // Map CTRIPOUTL to OUTPUTXBAR3
    OutputXbarRegs.OUTPUT3MUXENABLE.bit.MUX5 = 1;          // Enable OUTPUTXBAR3 MUX5

    // Configure GPIO37 for OUTPUTXBAR2 (CTRIPOUTH)
    GpioCtrlRegs.GPBMUX1.bit.GPIO37 = 0x01;        // Set GPIO37 to peripheral function
    GpioCtrlRegs.GPBGMUX1.bit.GPIO37 = 0x00;       // Set GPIO37 to primary peripheral mode

    // Configure GPIO48 for OUTPUTXBAR3 (CTRIPOUTL)
    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0x01;        // Set GPIO48 to peripheral function
    GpioCtrlRegs.GPBGMUX2.bit.GPIO48 = 0x00;       // Set GPIO48 to primary peripheral mode

    // Disable access to protected registers
    EDIS;
}


/**
 * @brief Initializes CMPSS 2 module for AC voltage monitoring applications
 *
 * Configures CMPSS 2 with the following key parameters:
 * - DAC functionality enabled with direct value assignment (no shadow registers)
 * - Ramp generator disabled (runs in free mode without software control)
 * - High/low comparator negative inputs tied to DAC output
 * - Digital filtering enabled for noise reduction on comparator outputs
 * - Comparator hysteresis set to 4x the base hysteresis voltage (9.67032967032967 mV * 4)
 * - Output inversion configured for low comparator (enabled) and high comparator (disabled)
 * - Filtered comparator outputs routed to GPIO pins via Output XBAR
 * - DAC reference values set relative to midpoint (2048) using AC voltage limit thresholds
 *
 * @note DAC values are calculated as midpoint (2048) ¡À V_AC_MAX for over/under voltage detection
 */
void Init_Cmpss_2()       // No ramp generator; no DAC shadow register
{
    // Enable access to protected registers
    EALLOW;

    // Enable DAC functionality for the comparator subsystem
    Cmpss2Regs.COMPCTL.bit.COMPDACE  =  CMPSS_ENABLE;        // Enable DAC for comparison

    // Configure comparator input sources
    Cmpss2Regs.COMPCTL.bit.COMPHSOURCE  =  NEGIN_DAC;        // High comparator negative input = DAC output
    Cmpss2Regs.COMPCTL.bit.COMPLSOURCE  =  NEGIN_DAC;        // Low comparator negative input = DAC output

    // Configure output inversion
    Cmpss2Regs.COMPCTL.bit.COMPHINV  =  0;        // Disable inversion for high comparator output
    Cmpss2Regs.COMPCTL.bit.COMPLINV  =  1;        // Enable inversion for low comparator output

    // Configure trip signal routing (use filtered outputs)
    Cmpss2Regs.COMPCTL.bit.CTRIPOUTLSEL  =  CTRIP_FILTER;        // CTRIPOUTL = filtered low comparator output
    Cmpss2Regs.COMPCTL.bit.CTRIPLSEL     =  CTRIP_FILTER;        // CTRIPL = filtered low comparator output
    Cmpss2Regs.COMPCTL.bit.CTRIPOUTHSEL  =  CTRIP_FILTER;        // CTRIPOUTH = filtered high comparator output
    Cmpss2Regs.COMPCTL.bit.CTRIPHSEL     =  CTRIP_FILTER;        // CTRIPH = filtered high comparator output

    // Set comparator hysteresis (9.67032967032967 mV * 4)
    Cmpss2Regs.COMPHYSCTL.bit.COMPHYS = 4 ;     // Hysteresis multiplier = 4

    // Configure DAC reference and ramp generator behavior
    Cmpss2Regs.COMPDACCTL.bit.FREESOFT = 2 ;    // Ramp generator runs freely (no software control)
    Cmpss2Regs.COMPDACCTL.bit.SELREF = REFERENCE_VDDA;        // Use VDDA as DAC reference voltage

    // Configure low-side digital filter parameters
    Cmpss2Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss2Regs.CTRIPLFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss2Regs.CTRIPLFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss2Regs.CTRIPLFILCTL.bit.FILINIT = 1;       // Initialize and start the low-side filter

    // Configure high-side digital filter parameters
    Cmpss2Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss2Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss2Regs.CTRIPHFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss2Regs.CTRIPHFILCTL.bit.FILINIT = 1;       // Initialize and start the high-side filter

    // Set DAC reference values (midpoint ¡À voltage limit)
    Cmpss2Regs.DACHVALS.bit.DACVAL = 2048 + V_AC_MAX;      // High comparator reference (upper voltage limit)
    Cmpss2Regs.DACLVALS.bit.DACVAL = 2048 - V_AC_MAX;      // Low comparator reference (lower voltage limit)

    // Wait for DAC and filters to stabilize
    DELAY_US(100);

    // Clear comparator status latches
    Cmpss2Regs.COMPSTSCLR.bit.HLATCHCLR = 1;       // Clear high comparator status latch
    Cmpss2Regs.COMPSTSCLR.bit.LLATCHCLR = 1;       // Clear low comparator status latch

    // Configure CTRIPOUTH routing to GPIO (first output)
    OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX2 = 0;        // Map CTRIPOUTH to OUTPUTXBAR4
    OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX2 = 1;          // Enable OUTPUTXBAR4 MUX2

    // Configure GPIO49 for OUTPUTXBAR4 (CTRIPOUTH)
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0x01;        // Set GPIO49 to peripheral function
    GpioCtrlRegs.GPBGMUX2.bit.GPIO49 = 0x00;       // Set GPIO49 to primary peripheral mode

    // Configure CTRIPOUTL routing to GPIO
    OutputXbarRegs.OUTPUT8MUX0TO15CFG.bit.MUX3 = 0;        // Map CTRIPOUTL to OUTPUTXBAR8
    OutputXbarRegs.OUTPUT8MUXENABLE.bit.MUX3 = 1;          // Enable OUTPUTXBAR8 MUX3

    // Configure GPIO31 for OUTPUTXBAR8 (CTRIPOUTL)
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0x01;        // Set GPIO31 to peripheral function
    GpioCtrlRegs.GPAGMUX2.bit.GPIO31 = 0x01;       // Set GPIO31 to secondary peripheral mode

    // Disable access to protected registers
    EDIS;
}


/**
 * @brief Initializes CMPSS 5 module for DC voltage monitoring applications
 *
 * Configures CMPSS 5 with the following key parameters:
 * - DAC functionality enabled with direct value assignment (no shadow registers)
 * - Ramp generator disabled (runs in free mode without software control)
 * - High/low comparator negative inputs tied to DAC output
 * - Digital filtering enabled for noise reduction on comparator outputs
 * - Comparator hysteresis set to 4x the base hysteresis voltage (9.67032967032967 mV * 4)
 * - Output inversion configured for low comparator (enabled) and high comparator (disabled)
 * - Filtered comparator outputs used for trip signal generation
 * - DAC reference values set using absolute DC voltage limit thresholds (no midpoint offset)
 *
 * @note DAC values are directly assigned to V_DC_MAX (upper limit) and V_DC_MIN (lower limit)
 *       for DC over/under voltage detection
 */
void Init_Cmpss_5()       // No ramp generator; no DAC shadow register
{
    // Enable access to protected registers
    EALLOW;

    // Enable DAC functionality for the comparator subsystem
    Cmpss5Regs.COMPCTL.bit.COMPDACE  =  CMPSS_ENABLE;        // Enable DAC for comparison

    // Configure comparator input sources
    Cmpss5Regs.COMPCTL.bit.COMPHSOURCE  =  NEGIN_DAC;        // High comparator negative input = DAC output
    Cmpss5Regs.COMPCTL.bit.COMPLSOURCE  =  NEGIN_DAC;        // Low comparator negative input = DAC output

    // Configure output inversion
    Cmpss5Regs.COMPCTL.bit.COMPHINV  =  0;        // Disable inversion for high comparator output
    Cmpss5Regs.COMPCTL.bit.COMPLINV  =  1;        // Enable inversion for low comparator output

    // Configure trip signal routing (use filtered outputs)
    Cmpss5Regs.COMPCTL.bit.CTRIPOUTLSEL  =  CTRIP_FILTER;        // CTRIPOUTL = filtered low comparator output
    Cmpss5Regs.COMPCTL.bit.CTRIPLSEL     =  CTRIP_FILTER;        // CTRIPL = filtered low comparator output
    Cmpss5Regs.COMPCTL.bit.CTRIPOUTHSEL  =  CTRIP_FILTER;        // CTRIPOUTH = filtered high comparator output
    Cmpss5Regs.COMPCTL.bit.CTRIPHSEL     =  CTRIP_FILTER;        // CTRIPH = filtered high comparator output

    // Set comparator hysteresis (9.67032967032967 mV * 4)
    Cmpss5Regs.COMPHYSCTL.bit.COMPHYS = 4 ;     // Hysteresis multiplier = 4

    // Configure DAC reference and ramp generator behavior
    Cmpss5Regs.COMPDACCTL.bit.FREESOFT = 2 ;    // Ramp generator runs freely (no software control)
    Cmpss5Regs.COMPDACCTL.bit.SELREF = REFERENCE_VDDA;        // Use VDDA as DAC reference voltage

    // Configure low-side digital filter parameters
    Cmpss5Regs.CTRIPLFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss5Regs.CTRIPLFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss5Regs.CTRIPLFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss5Regs.CTRIPLFILCTL.bit.FILINIT = 1;       // Initialize and start the low-side filter

    // Configure high-side digital filter parameters
    Cmpss5Regs.CTRIPHFILCLKCTL.bit.CLKPRESCALE = 9;        // Filter clock prescaler (200MHz / 10 = 20MHz)
    Cmpss5Regs.CTRIPHFILCTL.bit.SAMPWIN = 0x1F;     // Sampling window = 32 cycles (1.6us at 20MHz)
    Cmpss5Regs.CTRIPHFILCTL.bit.THRESH = 0x1F;     // Filter threshold = 32 consecutive samples
    Cmpss5Regs.CTRIPHFILCTL.bit.FILINIT = 1;       // Initialize and start the high-side filter

    // Set DAC reference values (midpoint ¡À voltage limit)
    Cmpss5Regs.DACHVALS.bit.DACVAL = V_DC_MAX;      // High comparator reference (upper voltage limit)
    Cmpss5Regs.DACLVALS.bit.DACVAL = V_DC_MIN;      // Low comparator reference (lower voltage limit)

    // Wait for DAC and filters to stabilize
    DELAY_US(100);

    // Clear comparator status latches
    Cmpss5Regs.COMPSTSCLR.bit.HLATCHCLR = 1;       // Clear high comparator status latch
    Cmpss5Regs.COMPSTSCLR.bit.LLATCHCLR = 1;       // Clear low comparator status latch

    // Disable access to protected registers
    EDIS;
}
