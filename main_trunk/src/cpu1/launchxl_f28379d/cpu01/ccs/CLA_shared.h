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
#define BUFFER_SIZE       64
#define TABLE_SIZE        64
#define TABLE_SIZE_M_1    TABLE_SIZE-1
#define PIBYTWO           1.570796327
#define PI                3.141592653589
#define INV2PI            0.159154943

//
// Globals
//

//
//Task 1 (C) Variables
//
extern float y[];            //Result vector
extern float fVal;           //Holds the input argument to the task
extern float fResult;        //The arsine of the input argument

/* shared controller data */
extern float rk;
extern float yk;
extern float uk;
extern DCL_PI_CLA pi1;

//
//Task 2 (C) Variables
//

//
//Task 3 (C) Variables
//

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
