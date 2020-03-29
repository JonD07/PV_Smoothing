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
//Task 1 (C) Variables
// NOTE: Do not initialize the Message RAM variables globally, they will be
// reset during the message ram initialization phase in the CLA memory
// configuration routine
//
#ifdef __cplusplus
#pragma DATA_SECTION("CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float fResult;
#else
#pragma DATA_SECTION(fVal,"CpuToCla1MsgRAM");
float fVal;
#pragma DATA_SECTION(fResult,"Cla1ToCpuMsgRAM");
float fResult;
#endif //__cplusplus
float y[BUFFER_SIZE];

//
//Common (C) Variables
//The Exponential table
//
#ifdef __cplusplus
#pragma DATA_SECTION("CLADataLS0")
#else
#pragma DATA_SECTION(CLAasinTable,"CLADataLS0")
#endif //__cplusplus
float CLAasinTable[]={
    0.0, 1.0, 0.0,
    0.000000636202, 0.999877862610, 0.007815361896,
    0.000005099694, 0.999510644409, 0.015647916155,
    0.000017268312, 0.998895919094, 0.023514960332,
    0.000041121765, 0.998029615282, 0.031434003631,
    0.000080794520, 0.996905974725, 0.039422875916,
    0.000140631089, 0.995517492804, 0.047499840611,
    0.000225244584, 0.993854840311, 0.055683712914,
    0.000339579512, 0.991906765146, 0.063993984848,
    0.000488979852, 0.989659972212, 0.072450958820,
    0.000679263611, 0.987098979366, 0.081075891529,
    0.000916805182, 0.984205946802, 0.089891150305,
    0.001208627040, 0.980960476685, 0.098920384204,
    0.001562502549, 0.977339379243, 0.108188712551,
    0.001987071928, 0.973316400729, 0.117722933997,
    0.002491973784, 0.968861907789, 0.127551759665,
    0.003087995053, 0.963942521723, 0.137706074532,
    0.003787242692, 0.958520694794, 0.148219231941,
    0.004603341138, 0.952554219267, 0.159127386977,
    0.005551660294, 0.945995657913, 0.170469875522,
    0.006649579796, 0.938791682505, 0.182289647088,
    0.007916796475, 0.930882303984, 0.194633761132,
    0.009375683410, 0.922199974574, 0.207553958472,
    0.011051710808, 0.912668537890, 0.221107321885,
    0.012973941175, 0.902201997769, 0.235357042896,
    0.015175614174, 0.890703070035, 0.250373315541,
    0.017694840102, 0.878061473098, 0.266234382514,
    0.020575425537, 0.864151902887, 0.283027765009,
    0.023867860513, 0.848831624374, 0.300851714968,
    0.027630504055, 0.831937595031, 0.319816937941,
    0.031931014547, 0.813283013821, 0.340048646894,
    0.036848083955, 0.792653161200, 0.361689022958,
    0.042473551274, 0.769800358920, 0.384900179460,
    0.048914992206, 0.744437830278, 0.409867752228,
    0.056298910750, 0.716232177740, 0.436805274317,
    0.064774696786, 0.684794109766, 0.465959540059,
    0.074519565699, 0.649666934178, 0.497617226179,
    0.085744766889, 0.610312179660, 0.532113122767,
    0.098703445606, 0.566091493186, 0.569840443472,
    0.113700678529, 0.516243664372, 0.611263845480,
    0.131106395009, 0.459855210927, 0.656936015611,
    0.151372169232, 0.395822366759, 0.707518998893,
    0.175053263659, 0.322801460177, 0.763811905770,
    0.202837883870, 0.239143420888, 0.826787304376,
    0.235586468765, 0.142806299514, 0.897639596948,
    0.274385149825, 0.031236880585, 0.977850174820,
    0.320619535938, -0.098791845166, 1.069276441800,
    0.376078169620, -0.251407364538, 1.174275392129,
    0.443100143614, -0.431959397725, 1.295878193174,
    0.524789871827, -0.647485610469, 1.438041695773,
    0.625336471263, -0.907400624736, 1.606018804842,
    0.750500589935, -1.224540947101, 1.806917563896,
    0.908377657341, -1.616794995066, 2.050569262035,
    1.110633894185, -2.109729648039, 2.350920816737,
    1.374584721437, -2.740985157716, 2.728353889708,
    1.726848242753, -3.567962877198, 3.213722960014,
    2.210117561056, -4.682006534082, 3.855770086891,
    2.896554011854, -6.236312386687, 4.735651038017,
    3.916505715382, -8.505488022524, 5.997790945975,
    5.526855868703, -12.026617159136, 7.922628470498,
    8.298197116322, -17.983705080358, 11.123941286820,
    13.741706072449, -29.488929624542, 17.203344479111,
    27.202707817485, -57.466598393615, 31.741016484669,
    83.158101335898, -171.803399517566, 90.149831709374
};

float asin_expected[BUFFER_SIZE]={
    1.570796, 1.393789, 1.320141, 1.263401, 1.215375,
    1.172892, 1.134327, 1.098718, 1.065436, 1.034046,
    1.004232, 0.9757544, 0.9484279, 0.9221048, 0.8966658,
    0.8720123, 0.8480621, 0.8247454, 0.8020028, 0.7797828,
    0.7580408, 0.7367374, 0.7158381, 0.6953120, 0.6751316,
    0.6552721, 0.6357113, 0.6164289, 0.5974064, 0.5786270,
    0.5600753, 0.5417370, 0.5235988, 0.5056486, 0.4878751,
    0.4702678, 0.4528166, 0.4355124, 0.4183464, 0.4013104,
    0.3843968, 0.3675981, 0.3509074, 0.3343180, 0.3178237,
    0.3014185, 0.2850964, 0.2688521, 0.2526802, 0.2365756,
    0.2205333, 0.2045484, 0.1886164, 0.1727327, 0.1568929,
    0.1410927, 0.1253278, 0.1095943, 0.09388787, 0.07820469,
    0.06254076, 0.04689218, 0.03125509, 0.01562564
};

//
// Function Prototypes
// TODO: MOVE THESE AFTER PORT!!
//
float CLA_runTest(void);
void CLA_configClaMemory(void);
void CLA_initCpu1Cla1(void);
__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();

uint16_t pass=0;
uint16_t fail=0;

//
// Main
//
void main() {

	int16_t nVin_A1;	// BatteryCurrent_Vin
	int16_t nVin_B5;	// BatteryVoltage_Vin
	int16_t nVin_C2;	// PanelCurrent_Vin
	int16_t nVin_14;	// PanelVoltage_Vin
						// on ADC D

	// test CLA
	float asin = 0;

	//
	// Initialize the system
	//
	InitSystem();

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
		// Run CLA test
		//
		asin = CLA_runTest();

		// Read ADC Values
		readADCInput(&nVin_A1, &nVin_B5, &nVin_C2, &nVin_14);
		t_SysMsrmnt.BatteryCurrent_Vin = (3000*(uint32_t)nVin_A1)/4095;
		t_SysMsrmnt.PanelCurrent_Vin = (3000*(uint32_t)nVin_C2)/4095;
		t_SysMsrmnt.BatteryVoltage_Vin = (3000*(uint32_t)nVin_B5)/4095;
		t_SysMsrmnt.PanelVoltage_Vin = (3000*(uint32_t)nVin_14)/4095;

		// Print results
		// printf("Values read: \tA1=%d, \t\tB5=%d, \t\tC2=%d, \tD14=%d\n\r", nVin_A1, nVin_B5, nVin_C2, nVin_14);
		printf("Volts read: \tA1=%lu mV, \tB5=%lu mV, \tC2=%lu mV, \tD14=%lu mV, asin(0.75) = %f\n\r", t_SysMsrmnt.BatteryCurrent_Vin,
			   t_SysMsrmnt.BatteryVoltage_Vin, t_SysMsrmnt.PanelCurrent_Vin, t_SysMsrmnt.PanelVoltage_Vin, asin);

		// Wait, reset LED
		DELAY_US(100000);
		GpioDataRegs.GPBSET.bit.GPIO34 = 1;


		DELAY_US(400000);
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
// CLA_runTest - Execute CLA task tests for specified vectors
//
float CLA_runTest(void)
{
//    int16_t i, error;
//
//    for(i=0; i < BUFFER_SIZE; i++)
//    {
//        fVal= (float)(BUFFER_SIZE - i)/(float)BUFFER_SIZE;
//        Cla1ForceTask1andWait();
//
//        y[i] = fResult;
//        error = fabs(asin_expected[i]-y[i]);
//
//        if(error < 0.1)
//        {
//            pass++;
//        }
//        else
//        {
//            fail++;
//        }
//    }

    fVal= 0.75;
    Cla1ForceTask1andWait();

    return fResult;

#if 0
    Cla1ForceTask2andWait();
    WAITSTEP;

    Cla1ForceTask3andWait();
    WAITSTEP;

    Cla1ForceTask4andWait();
    WAITSTEP;

    Cla1ForceTask5andWait();
    WAITSTEP;

    Cla1ForceTask6andWait();
    WAITSTEP;

    Cla1ForceTask7andWait();
    WAITSTEP;

    Cla1ForceTask8andWait();
    WAITSTEP;
#endif
}

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
    asm(" ESTOP0");
}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
    asm(" ESTOP0");
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

