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
#define EPWM1_TIMER_TBPRD  2000  // Period register
#define EPWM1_MAX_CMPA     1950
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM_CMP_UP           1
#define EPWM_CMP_DOWN         0

//
// Globals
//
typedef struct
{
    volatile struct EPWM_REGS *EPwmRegHandle;
    Uint16 EPwm_CMPA_Direction;
    Uint16 EPwm_CMPB_Direction;
    Uint16 EPwmTimerIntCount;
    Uint16 EPwmMaxCMPA;
    Uint16 EPwmMinCMPA;
    Uint16 EPwmMaxCMPB;
    Uint16 EPwmMinCMPB;
}EPWM_INFO;

//
//  Function Prototypes
//
void config_PWM(void);
void InitEPwm1Example(void);
__interrupt void epwm1_isr(void);
void update_compare(EPWM_INFO*);

#endif /* PWMDRIVER_H_ */
