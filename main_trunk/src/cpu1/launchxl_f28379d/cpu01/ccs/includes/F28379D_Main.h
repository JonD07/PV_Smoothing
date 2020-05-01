/*
 * F28379D_Main.h
 *
 *  Created on: Mar 11, 2020
 *      Author: Jonathan Diller
 */

#ifndef INCLUDES_F28379D_MAIN_H_
#define INCLUDES_F28379D_MAIN_H_

//
// Included Files
//
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <file.h>
#include <inttypes.h>

#include "F28x_Project.h"     // DSP28x Headerfile
#include "PWMDriver.h"
#include "sci_io.h"
#include "CLA_shared.h"
#include "DCLCLA.h"

//
// Constants
//
const int32 POWER_RECORD_SIZE = 100;

//
// Globals
//
EPWM_CONFIG* t_pwmConfig1;
EPWM_CONFIG* t_pwmConfig2;
// Stores average power from PV panel
float arr_nPowerRecord[100];
int32 powerRecord_Index;

//
// Prototypes
//
void InitSystem(void);
void ConfigADC(void);
void SetupADCSoftware(void);
int16_t readADCInput(int16_t* A1, int16_t* B5, int16_t* C2, int16_t* D14);
void ConfigSCIA();
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

#endif /* INCLUDES_F28379D_MAIN_H_ */
