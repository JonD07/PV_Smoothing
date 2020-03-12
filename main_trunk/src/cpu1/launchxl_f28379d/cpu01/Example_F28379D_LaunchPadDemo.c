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
#include <includes/F28379D_Main.h>

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
SYSTEM_MEASUREMENT t_SysMsrmnt;

//
// Main
//
void main() {
	volatile int status = 0;
	uint16_t i;
	volatile FILE *fid;

	int16_t nVin_A1;	// BatteryCurrent_Vin
	int16_t nVin_B5;	// BatteryVoltage_Vin
	int16_t nVin_C2;	// PanelCurrent_Vin
	int16_t nVin_14;	// PanelVoltage_Vin
						// on ADC D

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
	// Configure the ADCs and power them up
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

	for(i = 0; i < 20; i++) {
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
		// Blink red LED to show program is running
		GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;

		// Read ADC Values
		readADCInput(&nVin_A1, &nVin_C2, &nVin_B5, &nVin_14);
		t_SysMsrmnt.BatteryCurrent_Vin = (3300*(uint32_t)nVin_A1)/4095;
		t_SysMsrmnt.PanelCurrent_Vin = (3300*(uint32_t)nVin_C2)/4095;
		t_SysMsrmnt.BatteryVoltage_Vin = (3300*(uint32_t)nVin_B5)/4095;
		t_SysMsrmnt.PanelVoltage_Vin = (3300*(uint32_t)nVin_14)/4095;

		// Print results
//		printf("Values read: \tA1=%d, \t\tB5=%d, \t\tC2=%d, \tD14=%d\n\r", nVin_A1, nVin_B5, nVin_C2, nVin_14);
		printf("Volts read: \tA1=%lu mV, \tB5=%lu mV, \tC2=%lu mV, \tD14=%lu mV\n\r", t_SysMsrmnt.BatteryCurrent_Vin,
			   t_SysMsrmnt.BatteryVoltage_Vin, t_SysMsrmnt.PanelCurrent_Vin, t_SysMsrmnt.PanelVoltage_Vin);

		// Wait, reset LED
		DELAY_US(100000);
		GpioDataRegs.GPBSET.bit.GPIO34 = 1;


		DELAY_US(400000);
	}
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void) {
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
	//power up the ADCs
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

//
// SetupADCSoftware - Setup ADC channels and acquisition window
//
void SetupADCSoftware(void) {
	Uint16 acqps;

	//
	// ADCA
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION) {
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//
	//Select the channels to convert and end of conversion flag
	//ADCA
	//
	EALLOW;
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;		//SOC0 will convert pin A1
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;	//sample window is acqps + 1 SYSCLK cycles
	AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;	//end of SOC0 will set INT1 flag
	AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;	//enable INT1 flag
	AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//make sure INT1 flag is cleared
	EDIS;

	//
	// ADCB
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdcbRegs.ADCCTL2.bit.RESOLUTION) {
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//ADCB
	EALLOW;
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 5;		//SOC0 will convert pin B5
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;	//sample window is acqps + 1 SYSCLK cycles
	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;	//end of SOC0 will set INT1 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;	//enable INT1 flag
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//make sure INT1 flag is cleared
	EDIS;

	//
	// ADCC
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdccRegs.ADCCTL2.bit.RESOLUTION) {
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//ADCC
	EALLOW;
	AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;		//SOC0 will convert pin C2
	AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;	//sample window is acqps + 1 SYSCLK cycles
	AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;	//end of SOC0 will set INT1 flag
	AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;	//enable INT1 flag
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//make sure INT1 flag is cleared
	EDIS;

	//
	// ADCD
	// Determine minimum acquisition window (in SYSCLKS) based on resolution
	//
	if(ADC_RESOLUTION_12BIT == AdcdRegs.ADCCTL2.bit.RESOLUTION) {
		acqps = 14; //75ns
	}
	else { //resolution is 16-bit
		acqps = 63; //320ns
	}

	//ADCC
	EALLOW;
	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0x0E;	//SOC0 will convert pin ADCIN14
	AdcdRegs.ADCSOC0CTL.bit.ACQPS = acqps;	//sample window is acqps + 1 SYSCLK cycles
	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0;	//end of SOC0 will set INT1 flag
	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;	//enable INT1 flag
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;	//make sure INT1 flag is cleared
	EDIS;
}

//
// sampleADC - A3, B3, C2, D14
//
int16_t readADCInput(int16_t* A1, int16_t* B5, int16_t* C2, int16_t* D14) {
	//
	//convert, wait for completion, and store results
	//start conversions immediately via software ADCA, ADCB, ADCC, ADCD
	//
	AdcaRegs.ADCSOCFRC1.all = 0x0001; //SOC0
	AdcbRegs.ADCSOCFRC1.all = 0x0001; //SOC0
	AdccRegs.ADCSOCFRC1.all = 0x0001; //SOC0
	AdcdRegs.ADCSOCFRC1.all = 0x0001; //SOC0

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
	//wait for ADCC to complete, then acknowledge flag
	//
	while(AdccRegs.ADCINTFLG.bit.ADCINT1 == 0);
	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

	//
	//wait for ADCD to complete, then acknowledge flag
	//
	while(AdcdRegs.ADCINTFLG.bit.ADCINT1 == 0);
	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

	//
	//store results
	//
	*A1 = AdcaResultRegs.ADCRESULT0;
	*B5 = AdcbResultRegs.ADCRESULT0;
	*C2 = AdccResultRegs.ADCRESULT0;
	*D14 = AdcdResultRegs.ADCRESULT0;

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
// End of File
//

