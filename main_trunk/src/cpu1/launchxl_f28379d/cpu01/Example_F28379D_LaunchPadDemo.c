//###########################################################################
//
// FILE:	F28379D_Main.c
//
// TITLE:	F28379D LaunchPad PV Smoothing Main source file
//
// Author: Jonathan Diller
//
// DESCRIPTION:
//! \addtogroup cpu01_example_list
//! <h1>PV Smoothing Main source file</h1>
//!
//!  This program performs PV Smoothing using a DC-to-DC converter and
//!  battery storage on the TI F28379D LaunchPad. The red LED will flash
//!  on the board to indicate the system is running. System status
//!  data is displayed in a serial terminal via the boards back
//!  channel UART. You may view this data by configuring a serial terminal
//!  to the correct COM port at 115200 Baud 8-N-1.
//!
//
//###########################################################################
// $TI Release: F2837xD Support Library v3.05.00.00 $
// $Release Date: Thu Oct 18 15:48:42 CDT 2018 $
// $Copyright:
// Copyright (C) 2013-2018 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <file.h>
#include <inttypes.h>

#include "F28x_Project.h"     // DSP28x Headerfile
#include "sci_io.h"
#include "PWMDriver.h"

//
// Defines
//

//
// Micro-seconds to wait for ADC conversion. Longer than necessary.
//  
#define CONV_WAIT 1L 

//
// Globals
//
extern void DSP28x_usDelay(Uint32 Count);

const unsigned char escRed[] = {0x1B, 0x5B, '3','1', 'm'};
const unsigned char escWhite[] = {0x1B, 0x5B, '3','7', 'm'};
const unsigned char escLeft[] = {0x1B, 0x5B, '3','7', 'm'};
const unsigned char pucTempString[] = "ADCIN14 Sample:     ";

int16_t currentSample;
int16_t nVin_A1;
int16_t nVin_A3;
int16_t nVin_B2;
int16_t nVin_B3;

void adc_init(void) {
    //
    // Configure the ADC: Initialize the ADC
    //
	EALLOW;

    //
	// write configurations
    //
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6;      // set ADCCLK divider to /4
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6;      // set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //
	// Set pulse positions to late
    //
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
	// power up the ADCs
    //
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
	// delay for 1ms to allow ADC time to power up
    //
	DELAY_US(1000);

    //
    // ADCA
    //
    EALLOW;
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x0E;   //SOC0 will convert pin ADCIN14

    //
    // sample window is acqps + 1 SYSCLK cycles
    //
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 25;

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x0E;   //SOC1 will convert pin ADCIN14

    //
    // sample window is acqps + 1 SYSCLK cycles
    //
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 25;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  //end of SOC1 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //make sure INT1 flag is cleared
    EDIS;
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;

    //
    //write configurations
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_16BIT, ADC_SIGNALMODE_SINGLE);

    //
    //Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    //power up the ADCs
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    //
    //delay for 1ms to allow ADC time to power up
    //
    DELAY_US(1000);

    EDIS;
}

//
// SetupADCSoftware - Setup ADC channels and acquisition window
//
void SetupADCSoftware(void)
{
	Uint16 acqps;

	//
	// ADCA
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
	{
		acqps = 14; //75ns
	}
	else //resolution is 16-bit
	{
		acqps = 63; //320ns
	}

	//
	//Select the channels to convert and end of conversion flag
	//ADCA
	//
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 will convert pin A1
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
										   //1 SYSCLK cycles
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
										   //1 SYSCLK cycles
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//
	// ADCB
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION)
	{
		acqps = 14; //75ns
	}
	else //resolution is 16-bit
	{
		acqps = 63; //320ns
	}

	//ADCB
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin B2
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps; //sample window is acqps +
										   //1 SYSCLK cycles
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin B3
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps; //sample window is acqps +
										   //1 SYSCLK cycles
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	EDIS;
}

//
// sampleADC - 
//
int16_t sampleADC(void) {
	int16_t sample;

	//
	// Force start of conversion on SOC0
	//
	AdcaRegs.ADCSOCFRC1.all = 0x03;

	//
	// Wait for end of conversion.
	//
	while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0)
	{
		//
		// Wait for ADCINT1
		//
	}
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;        // Clear ADCINT1

	//
	// Get ADC sample result from SOC0
	//
	sample = AdcaResultRegs.ADCRESULT1;

//	p_pwm1Config->EPwmCMP_A = EPWM1_TIMER_TBPRD/2;

	return(sample);
}

//
// sampleADC - A1, A3, B2, B3
//
int16_t readADCInput(int16_t* A1, int16_t* A3, int16_t* B2, int16_t* B3) {
	//
	//convert, wait for completion, and store results
	//start conversions immediately via software, ADCA
	//
	AdcaRegs.ADCSOCFRC1.all = 0x0003; //SOC0 and SOC1

	//
	//start conversions immediately via software, ADCB
	//
	AdcbRegs.ADCSOCFRC1.all = 0x0003; //SOC0 and SOC1

	//
	//wait for ADCA to complete, then acknowledge flag
	//
	while(AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0);
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

	//
	//wait for ADCB to complete, then acknowledge flag
	//
	while(AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0);
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

	//
	//store results
	//
	*A1 = AdcaResultRegs.ADCRESULT0;
	*A3 = AdcaResultRegs.ADCRESULT1;
	*B2 = AdcbResultRegs.ADCRESULT0;
	*B3 = AdcbResultRegs.ADCRESULT1;

	return true;
}

//
// scia_init - SCIA  8-bit word, baud rate 0x000F, default, 1 STOP bit, 
// no parity
//
void scia_init() {
	//
	// Note: Clocks were turned on to the SCIA peripheral
	// in the InitSysCtrl() function
	//

	//
	// 1 stop bit,  No loopback, No parity,8 char bits, async mode,
	// idle-line protocol
	//
	SciaRegs.SCICCR.all =0x0007;

	//
	// enable TX, RX, internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
	//
	SciaRegs.SCICTL1.all =0x0003;
	
	SciaRegs.SCICTL2.bit.TXINTENA =1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	//
	// 115200 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK)
	//
	SciaRegs.SCIHBAUD.all    =0x0000;

	SciaRegs.SCILBAUD.all    =53;

	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

	return;
}

//
// Main
//
void main() {
	volatile int status = 0;
	uint16_t i;
	volatile FILE *fid;

	//
	// If running from flash copy RAM only functions to RAM
	//
	#ifdef _FLASH
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
	#endif

	//
	// Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2806x_SysCtrl.c file.
	//
	InitSysCtrl();

	//
	// For this example, only init the pins for the SCI-A port.
	//
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 3;
	GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 3;
	GpioCtrlRegs.GPBGMUX1.bit.GPIO42 = 3;
	GpioCtrlRegs.GPBGMUX1.bit.GPIO43 = 3;
	EDIS;

	//
	// Clear all interrupts and initialize PIE vector table:
	// Disable CPU interrupts
	//
	DINT;

	//
	// Initialize PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
	// This function is found in the F2837xD_PieCtrl.c file.
	//
	InitPieCtrl();

	//
	// Disable CPU interrupts and clear all CPU interrupt flags
	//
	IER = 0x0000;
	IFR = 0x0000;

	//
	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2837xD_DefaultIsr.c.
	// This function is found in F2837xD_PieVect.c.
	//
	InitPieVectTable();

	//
	// Initialize SCIA
	//
	scia_init();

	//
	// Initialize PWM
	//
	init_PWMDriver();

	//
	// Initialize ADC
	//
	//	adc_init();

	//
	//Configure the ADCs and power them up
	//
	ConfigureADC();

	//
	//Setup the ADCs for software conversions
	//
	SetupADCSoftware();

	//
	// Initialize GPIOs for the LEDs and turn them off
	//
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
	GpioDataRegs.GPADAT.bit.GPIO31 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
	EDIS;

	//
	// Enable global Interrupts and higher priority real-time debug events:
	// Only do this once, in main!!
	//
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	//
	// Redirect STDOUT to SCI
	//
	status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write,
						SCI_lseek, SCI_unlink, SCI_rename);
	fid = fopen("scia","w");
	freopen("scia:", "w", stdout);
	setvbuf(stdout, NULL, _IONBF, 0);

	//
	// Twiddle LEDs
	//
	GpioDataRegs.GPADAT.bit.GPIO31 = 0;
	GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

	for(i = 0; i < 10; i++) {
		GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
		DELAY_US(50000);
	}

	//
	// LEDs off
	//
	GpioDataRegs.GPADAT.bit.GPIO31 = 1;
	GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

	//
	// Main program loop - continually sample temperature
	//
	while(true) {
		//
		// Sample ADCIN14
		//
//		currentSample = sampleADC();
		readADCInput(&nVin_A1, &nVin_A3, &nVin_B2, &nVin_B3);

		// Blink light to show program is running
		GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
//		printf("Value read: %zu\n\r", currentSample);
		printf("Values read: A1=%d, A3=%d, B2=%d, B3=%d\n\r", nVin_A1, nVin_A3, nVin_B2, nVin_B3);
		DELAY_US(100000);
		GpioDataRegs.GPBSET.bit.GPIO34 = 1;


		DELAY_US(900000);
	}
}

//
// End of File
//

