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

//
// Globals
//

//
// Prototypes
//
void InitSystem(void);
void ConfigADC(void);
void SetupADCSoftware(void);
int16_t readADCInput(int16_t* A1, int16_t* B5, int16_t* C2, int16_t* D14);
void ConfigSCIA();

#endif /* INCLUDES_F28379D_MAIN_H_ */
