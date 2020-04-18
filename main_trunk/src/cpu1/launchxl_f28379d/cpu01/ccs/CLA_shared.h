/*
 * CLA_shared.h
 *
 *  Created on: Mar 28, 2020
 *      Author: Jonathan Diller
 */

#ifndef CLA_SHARED_H_
#define CLA_SHARED_H_

#include "DCLCLA.h"
#include "F2837xD_Cla_typedefs.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
// Defines
//

//
// Globals
//
extern float rk_BuckBoost;
extern float yk_BuckBoost;
extern float uk_BuckBoost;

extern float rk_Power;
extern float yk_Power;
extern float uk_Power;

//
//Task 1 (C) Variables
//
extern DCL_PI_CLA pi_Boost;

//
//Task 2 (C) Variables
//
extern DCL_PI_CLA pi_Buck;

//
//Task 3 (C) Variables
//
extern DCL_PI_CLA pi_Power;

//
//Task 4 (C) Variables
//

//
//Task 5 (C) Variables
//

//
//Task 6 (C) Variables
//

//
//Task 7 (C) Variables
//

//
//Task 8 (C) Variables
//

//
//Common (C) Variables
//
extern float CLAasinTable[]; //The arcsine lookup table

//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"

#endif /* CLA_SHARED_H_ */
