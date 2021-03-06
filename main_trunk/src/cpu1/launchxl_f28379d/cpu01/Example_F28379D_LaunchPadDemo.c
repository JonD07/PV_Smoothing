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

// Micro-seconds to wait for ADC conversion. Longer than necessary.
#define CONV_WAIT 1L 

//
// Globals
//
SYSTEM_MEASUREMENT t_SysMsrmnt;

// shared variables
#pragma DATA_SECTION(rk_BuckBoost, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(yk_BuckBoost, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(uk_BuckBoost, "Cla1ToCpuMsgRAM")
float rk_BuckBoost;
float yk_BuckBoost;
float uk_BuckBoost;

// PI Boost
#pragma DATA_SECTION(pi_Boost, "CLADataLS0")
DCL_PI_CLA pi_Boost = PI_CLA_DEFAULTS;

// PI Buck
#pragma DATA_SECTION(pi_Buck, "CLADataLS0")
DCL_PI_CLA pi_Buck = PI_CLA_DEFAULTS;

// PI Power
#pragma DATA_SECTION(rk_Power, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(yk_Power, "CpuToCla1MsgRAM")
#pragma DATA_SECTION(uk_Power, "Cla1ToCpuMsgRAM")
float rk_Power;
float yk_Power;
float uk_Power;

#pragma DATA_SECTION(pi_Power, "CLADataLS0")
DCL_PI_CLA pi_Power = PI_CLA_DEFAULTS;

uint32_t nVinA;	// BatteryCurrent_Vin
uint32_t nVinB;	// BatteryVoltage_Vin
uint32_t nVinC;	// PanelCurrent_Vin
uint32_t nVinD;	// PanelVoltage_Vin
float Duty;
float fAvgPower;
float nCurrentPVPower;
float nCurrentBatPower;

//
// Main
//
void main() {
	//
	// Local var declarations
	int16_t nVin_A1;	// BatteryCurrent_Vin
	int16_t nVin_B5;	// BatteryVoltage_Vin
	int16_t nVin_C2;	// PanelCurrent_Vin
	int16_t nVin_14;	// PanelVoltage_Vin
						// on ADC D
	// test CLA
	int32_t temp1;
	int32_t temp2;
	int32_t temp3;
	int32_t temp4;

	int i;
	powerRecord_Index = 0;

	//
	// Initialize the system
	//
	InitSystem();

	//
	// Initialize PI Controller
	// PI Boost
	pi_Boost.Kp = 0.1f;
	pi_Boost.Ki = 1.5f;
	pi_Boost.i11 = 0.0f;
	pi_Boost.i10 = 0.0f;
	pi_Boost.i6 = 0.0f;
	pi_Boost.Umax = 1.0f;
	pi_Boost.Umin = 0.0f;
	pi_Boost.Imax = 0.9f;
	pi_Boost.Imin = 0.0f;
	// PI Buck
	pi_Buck.Kp = 0.02f;
	pi_Buck.Ki = 0.3f;
	pi_Buck.i11 = 0.0f;
	pi_Buck.i10 = 0.0f;
	pi_Buck.i6 = 0.0f;
	pi_Buck.Umax = 1.0f;
	pi_Buck.Umin = 0.0f;
	pi_Buck.Imax = 0.9f;
	pi_Buck.Imin = 0.0f;
	// PI Power
	pi_Power.Kp = 0.005f;
	pi_Power.Ki = 0.05f;
	pi_Power.i11 = 0.0f;
	pi_Power.i10 = 0.0f;
	pi_Power.i6 = 0.0f;
	pi_Power.Umax = 8.0f;
	pi_Power.Umin = -8.0f;
	pi_Power.Imax = 8.0f;
	pi_Power.Imin = -8.0f;
	rk_Power = 5.0;

	//
	// Get PWM Config struct
	t_pwmConfig1 = getPWMConfig(1);
	t_pwmConfig2 = getPWMConfig(2);

	//
	// Light display - show user system has started up
	{
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
		// Twiddle LEDs
		//
		GpioDataRegs.GPADAT.bit.GPIO31 = 0;
		GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

		uint16_t i;
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
	}

	//
	// Main program loop - continually sample temperature
	//
	while(true) {
		// Blink red LED to show program is running
		GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;


		//
		// Measure and record sensor data
		// Read ADC Values
		readADCInput(&nVin_A1, &nVin_B5, &nVin_C2, &nVin_14);
		nVinA = (3000*(uint32_t)nVin_A1)/4095;
		nVinB = (3000*(uint32_t)nVin_B5)/4095;
		nVinC = (3000*(uint32_t)nVin_C2)/4095;
		nVinD = (3000*(uint32_t)nVin_14)/4095;

		// Print results
		printf("Volts read: \tA1=%lu mV, \tB5=%lu mV, \tC2=%lu mV, \tD14=%lu mV\n\r", nVinA, nVinB, nVinC, nVinD);

		// Update t_SysMsrmnt
		t_SysMsrmnt.BatteryCurrent_Vin = 20*((Uint32)nVinA)/3 - 10000;
		t_SysMsrmnt.BatteryVoltage_Vin = 12000; // 5*((int32)nVinC)/4095;
		t_SysMsrmnt.PanelCurrent_Vin = 20*((Uint32)nVinB)/3 - 10000;
		t_SysMsrmnt.PanelVoltage_Vin = 24000; // 25*((int32)nVinD)/3;

		// Update power records with PV power
		arr_nPowerRecord[(powerRecord_Index%100)] = nCurrentPVPower = (t_SysMsrmnt.PanelCurrent_Vin/1000.0) * (t_SysMsrmnt.PanelVoltage_Vin/1000.0);
		nCurrentBatPower = (t_SysMsrmnt.BatteryCurrent_Vin/1000.0) * (t_SysMsrmnt.BatteryVoltage_Vin/1000.0);
		powerRecord_Index++;

		// Find average power
		fAvgPower = 0;
		for(i = 0; (i < POWER_RECORD_SIZE) && (i < powerRecord_Index); i++) {
			fAvgPower += arr_nPowerRecord[i];
		}
		fAvgPower = fAvgPower/i;
		rk_Power = (fAvgPower - nCurrentPVPower);

		// Print results
		printf("I_bat=%" PRId32 " mA, \tV_bat=%lu mV, \tI_pv=%" PRId32 " mA, \tV_pv=%lu mV\n\r", t_SysMsrmnt.BatteryCurrent_Vin,
			   t_SysMsrmnt.BatteryVoltage_Vin, t_SysMsrmnt.PanelCurrent_Vin, t_SysMsrmnt.PanelVoltage_Vin);


		//
		// Run PI controllers and
		// Update power input (from battery)
		yk_Power = (((float) t_SysMsrmnt.BatteryCurrent_Vin) / 1000.0f) * (((float) t_SysMsrmnt.BatteryVoltage_Vin) / 1000.0f);

		// Trigger Power PI controller in task 3
		EALLOW;
	    Cla1ForceTask3andWait();

	    // Update buck-boost input
	    rk_BuckBoost = uk_Power;

		// Convert to amps, assign to input yk_BuckBoost
		if(rk_BuckBoost >= 0) {
			yk_BuckBoost = ((float) t_SysMsrmnt.BatteryCurrent_Vin) / 1000.0f;

			// trigger PI controller for task 1 on CLA
			EALLOW;
		    Cla1ForceTask1andWait();

			// write u(k) to PWM
			Duty = uk_BuckBoost * (float) EPwm1Regs.TBPRD;
			t_pwmConfig1->EPwmCMP_A = (Uint16) Duty;
			t_pwmConfig2->EPwmCMP_A = 0;
		}
		else {
			yk_BuckBoost = ((float) t_SysMsrmnt.BatteryCurrent_Vin) / 1000.0f;

			// trigger PI controller for task 2 on CLA
			EALLOW;
		    Cla1ForceTask2andWait();

			// write u(k) to PWM
			Duty = uk_BuckBoost * (float) EPwm2Regs.TBPRD;
			t_pwmConfig2->EPwmCMP_A = (Uint16) Duty;
			t_pwmConfig1->EPwmCMP_A = 0;
		}

		// Print results
		temp1 = (int32_t)(yk_BuckBoost*100);
		temp2 = (int32_t)(uk_BuckBoost*100);
		temp3 = (int32_t)(yk_Power*100);
		temp4 = (int32_t)(uk_Power*100);
		printf("yk_Power = %" PRId32 ".%" PRId32 ", uk_Power = %" PRId32 ".%d\n\r", temp3/100, temp3%100, temp4/100, abs(temp4%100));
		printf("yk_BuckBoost = %" PRId32 ".%" PRId32 ", uk_BuckBoost = %" PRId32 ".%d, Comp Val = %hu, TBPRD: %hu\n\r", temp1/100, temp1%100, temp2/100, abs(temp2%100), (Uint16) Duty, EPwm1Regs.TBPRD);

		temp1 = (int32_t)(fAvgPower*100);
		temp2 = (int32_t)(nCurrentPVPower*100);
		temp3 = (int32_t)(rk_Power*100);
		printf("AvgPower = %" PRId32 ".%" PRId32 ", CurrentPVPower = %" PRId32 ".%" PRId32 ", rk_Power = %" PRId32 ".%" PRId32 "\n\r", temp1/100, temp1%100, temp2/100, temp2%100, temp3/100, temp3%100);

		// Wait, reset LED
		DELAY_US(10000);
		GpioDataRegs.GPBSET.bit.GPIO34 = 1;

		DELAY_US(490000);
	}
}

//
// InitSystem - Used to initialize the system.
// This is the general start-up routine.
//
void InitSystem(void) {
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

	CLA_configClaMemory();
	CLA_initCpu1Cla1();

	//
	// Initialize SCIA
	//
	ConfigSCIA();

	//
	// Initialize PWM
	//
	ConfigPWMDriver();

	//
	// Configure the ADCs and power them up
	//
	ConfigADC();

	//
	//Setup the ADCs for software conversions
	//
	SetupADCSoftware();

	//
	// Configure the CLA memory spaces first followed by
	// the CLA task vectors
	//
	CLA_configClaMemory();
	CLA_initCpu1Cla1();

	//
	// Enable global Interrupts and higher priority real-time debug events:
	// Only do this once, in main!!
	//
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM
}

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigADC(void) {
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
void ConfigSCIA() {
	volatile int status = 0;
	volatile FILE *fid;
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

	//
	// Redirect STDOUT to SCI
	//
	status = add_device("scia", _SSA, SCI_open, SCI_close, SCI_read, SCI_write,
						SCI_lseek, SCI_unlink, SCI_rename);
	fid = fopen("scia","w");
	freopen("scia:", "w", stdout);
	setvbuf(stdout, NULL, _IONBF, 0);

	return;
}



// ************************ CLA Stuff **************************

//
// CLA_configClaMemory - Configure CLA memory sections
//
void CLA_configClaMemory(void)
{
    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
           (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //
    // Select LS4RAM and LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS4 and LS5 and then
    // set the space to be a program block
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;
}

//
// CLA_initCpu1Cla1 - Initialize CLA1 task vectors and end of task interrupts
//
void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all = 0x00FF;

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;

    //
    // Enable CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
}

//
// cla1Isr1 - CLA1 ISR 1
//
__interrupt void cla1Isr1 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 1
    //
    PieCtrlRegs.PIEACK.all = M_INT11;

    //
    // Uncomment to halt debugger and stop here
    //
//    asm(" ESTOP0");
}

//
// cla1Isr2 - CLA1 ISR 2
//
__interrupt void cla1Isr2 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 2
    //
    PieCtrlRegs.PIEACK.all = M_INT11;

    //
    // Uncomment to halt debugger and stop here
    //
//    asm(" ESTOP0");
}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 2
    //
    PieCtrlRegs.PIEACK.all = M_INT11;

    //
    // Uncomment to halt debugger and stop here
    //
//    asm(" ESTOP0");
}

//
// cla1Isr4 - CLA1 ISR 4
//
__interrupt void cla1Isr4 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr5 - CLA1 ISR 5
//
__interrupt void cla1Isr5 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr6 - CLA1 ISR 6
//
__interrupt void cla1Isr6 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr7 - CLA1 ISR 7
//
__interrupt void cla1Isr7 ()
{
    asm(" ESTOP0");
}

//
// cla1Isr8 - CLA1 ISR 8
//
__interrupt void cla1Isr8 ()
{
    //
    // Acknowledge the end-of-task interrupt for task 8
    //
    PieCtrlRegs.PIEACK.all = M_INT11;

    //
    // Uncomment to halt debugger and stop here
    //
//    asm(" ESTOP0");
}

//
// End of File
//

