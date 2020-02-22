/*
 * PWMDriver.c
 *
 *  Created on: Feb 16, 2020
 *      Author: Jonathan Diller
 */


//
// Included Files
//
#include "PWMDriver.h"

//
// Globals
//
EPWM_CONFIG t_pwm1Config;

//
// config_PWM()
//
void init_PWMDriver(void) {

//
// Enable PWM1
//
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;

//
// For this case just init GPIO pins for ePWM1
// This function is in the F2837xD_EPwm.c file
//
    InitEPwm1Gpio();

//
// Re-map ePWM module interrupt to ISR in this file
//
    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm1_isr;
    EDIS;   // This is needed to disable write to EALLOW protected registers

//
// Initialize the ePWM module
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

//
// Enable CPU INT3 which is connected to EPWM1-3 INT:
//
    IER |= M_INT3;

//
// Enable EPWM INTn in the PIE: Group 3 interrupt 1
//
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
}

//
// epwm1_isr - EPWM1 ISR to update compare values
//
__interrupt void epwm1_isr(void) {
	//
	// Update the CMPA and CMPB values
	//
	t_pwm1Config.EPwmRegHandle->CMPA.bit.CMPA = t_pwm1Config.EPwmCMP_A;
	t_pwm1Config.EPwmRegHandle->CMPB.bit.CMPB = t_pwm1Config.EPwmCMP_B;

	//
	// Clear INT flag for this timer
	//
	EPwm1Regs.ETCLR.bit.INT = 1;

	//
	// Acknowledge this interrupt to receive more interrupts from group 3
	//
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
// InitEPwm1Example - Initialize EPWM1 values
//
void InitEPwm1() {
   //
   // Setup TBCLK
   //
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;	// Count up
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;			// Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;		// Disable phase loading
   EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;			// Phase is 0
   EPwm1Regs.TBCTR = 0x0000;					// Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		// Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;		// Divide by 1

   //
   // Setup shadow register load on ZERO
   //
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   //
   // Set Compare values
   //
   EPwm1Regs.CMPA.bit.CMPA = EPWM1_TIMER_TBPRD/2;     // Set compare A value
   EPwm1Regs.CMPB.bit.CMPB = EPWM1_TIMER_TBPRD/2;     // Set Compare B value

   //
   // Set actions
   //
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A on Zero
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A,
                                                 // up count

   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;            // Set PWM1B on Zero
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;          // Clear PWM1B on event B,
                                                 // up count

   //
   // Interrupt where we will change the Compare Values
   //
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event

   //
   // Set ePWM struct initial values
   //
   t_pwm1Config.EPwmRegHandle = &EPwm1Regs;        // Set the pointer to the
                                                 // ePWM module
   t_pwm1Config.EPwmMaxCMPA = EPWM1_MAX_CMPA;      // Setup min/max
                                                 // CMPA/CMPB values
   t_pwm1Config.EPwmMinCMPA = EPWM1_MIN_CMPA;
   t_pwm1Config.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   t_pwm1Config.EPwmMinCMPB = EPWM1_MIN_CMPB;

	t_pwm1Config.EPwmCMP_A = EPWM1_TIMER_TBPRD/2;
	t_pwm1Config.EPwmCMP_B = EPWM1_TIMER_TBPRD/2;
}

EPWM_CONFIG* getPWMConfig(Uint16 n) {
	switch(n) {
	case 1:
		return &t_pwm1Config;
	default:
		return &t_pwm1Config;
	}
}
