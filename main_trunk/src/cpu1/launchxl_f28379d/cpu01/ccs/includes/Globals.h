/*
 * Globals.h
 *
 *  Created on: Mar 11, 2020
 *      Author: Jonathan Diller
 */

#ifndef INCLUDES_GLOBALS_H_
#define INCLUDES_GLOBALS_H_

#include "F28x_Project.h"

//
// Defines
//

//
// Structs
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

typedef struct
{
	Uint32 BatteryVoltage_Vin;
	int32 BatteryCurrent_Vin;
	Uint32 PanelVoltage_Vin;
	int32 PanelCurrent_Vin;
}SYSTEM_MEASUREMENT;


//
// Globals
//

#endif /* INCLUDES_GLOBALS_H_ */
