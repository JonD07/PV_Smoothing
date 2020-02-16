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
EPWM_INFO pwm1_info;

//
// config_PWM()
//
void config_PWM(void)
{

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

    InitEPwm1Example();

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
__interrupt void epwm1_isr(void)
{
    //
    // Update the CMPA and CMPB values
    //
    update_compare(&pwm1_info);

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
void InitEPwm1Example()
{
   //
   // Setup TBCLK
   //
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = EPWM1_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
   EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;        // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

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
   EPwm1Regs.CMPA.bit.CMPA = EPWM1_MIN_CMPA;     // Set compare A value
   EPwm1Regs.CMPB.bit.CMPB = EPWM1_MIN_CMPB;     // Set Compare B value

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
   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   //
   pwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing
                                                 // CMPA & CMPB
   pwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP;
   pwm1_info.EPwmTimerIntCount = 0;             // Zero the interrupt counter
   pwm1_info.EPwmRegHandle = &EPwm1Regs;        // Set the pointer to the
                                                 // ePWM module
   pwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA;      // Setup min/max
                                                 // CMPA/CMPB values
   pwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
   pwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
   pwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}

//
// update_compare - Update the compare values for the specified EPWM
//
void update_compare(EPWM_INFO *epwm_info)
{
   //
   // Every 10'th interrupt, change the CMPA/CMPB values
   //
   if(epwm_info->EPwmTimerIntCount == 10)
   {
       epwm_info->EPwmTimerIntCount = 0;

       //
       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
       //
       if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
       {
           if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA < epwm_info->EPwmMaxCMPA)
           {
              epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
           }
           else
           {
              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
           }
       }

       //
       // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
       //
       else
       {
           if(epwm_info->EPwmRegHandle->CMPA.bit.CMPA == epwm_info->EPwmMinCMPA)
           {
              epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
              epwm_info->EPwmRegHandle->CMPA.bit.CMPA++;
           }
           else
           {
              epwm_info->EPwmRegHandle->CMPA.bit.CMPA--;
           }
       }

       //
       // If we were increasing CMPB, check to see if
       // we reached the max value.  If not, increase CMPB
       // else, change directions and decrease CMPB
       //
       if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
       {
           if(epwm_info->EPwmRegHandle->CMPB.bit.CMPB < epwm_info->EPwmMaxCMPB)
           {
              epwm_info->EPwmRegHandle->CMPB.bit.CMPB++;
           }
           else
           {
              epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPB.bit.CMPB--;
           }
       }

       //
       // If we were decreasing CMPB, check to see if
       // we reached the min value.  If not, decrease CMPB
       // else, change directions and increase CMPB
       //
       else
       {
           if(epwm_info->EPwmRegHandle->CMPB.bit.CMPB ==
              epwm_info->EPwmMinCMPB)
           {
              epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
              epwm_info->EPwmRegHandle->CMPB.bit.CMPB++;
           }
           else
           {
              epwm_info->EPwmRegHandle->CMPB.bit.CMPB--;
           }
       }
   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}
