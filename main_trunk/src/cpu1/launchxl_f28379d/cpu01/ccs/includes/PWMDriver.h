/*
 * PWMDriver.h
 *
 *  Created on: Feb 16, 2020
 *      Author: Jonathan Diller
 */

#ifndef INCLUDES_PWMDRIVER_H_
#define INCLUDES_PWMDRIVER_H_

//
// Included Files
//
#include "Globals.h"
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

//
//  Function Prototypes
//
void ConfigPWMDriver(void);
void InitEPwm1(void);
__interrupt void epwm1_isr(void);
EPWM_CONFIG* getPWMConfig(Uint16);

#endif /* INCLUDES_PWMDRIVER_H_ */
