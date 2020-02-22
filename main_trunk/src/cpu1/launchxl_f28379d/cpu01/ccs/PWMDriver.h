/*
 * PWMDriver.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Jonathan Diller
 */

#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

#include "F28x_Project.h"     // DSP28x Headerfile

//
// Defines
//

// PWM Frequency = TBCTR / EPWM1_TIMER_TBPRD
// TBCTR = SYS_COCK / (HSPCLKDIV * CLKDIV)
#define EPWM1_TIMER_TBPRD	1000  // Period register
#define EPWM1_MAX_CMPA		EPWM1_TIMER_TBPRD - 20
#define EPWM1_MIN_CMPA		20
#define EPWM1_MAX_CMPB		EPWM1_TIMER_TBPRD - 20
#define EPWM1_MIN_CMPB		20

#define EPWM_CMP_UP			1
#define EPWM_CMP_DOWN		0

//
// Globals
//
typedef struct
{
	volatile struct EPWM_REGS *EPwmRegHandle;
	Uint16 EPwmMaxCMPA;
	Uint16 EPwmMinCMPA;
	Uint16 EPwmMaxCMPB;
	Uint16 EPwmMinCMPB;

	Uint16 EPwmCMP_A;
	Uint16 EPwmCMP_B;
}EPWM_CONFIG;

//
//  Function Prototypes
//
void init_PWMDriver(void);
void InitEPwm1(void);
__interrupt void epwm1_isr(void);
EPWM_CONFIG* getPWMConfig(Uint16);

#endif /* PWMDRIVER_H_ */
