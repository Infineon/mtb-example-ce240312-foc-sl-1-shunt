/******************************************************************************
* File Name:   main.c
*
* Description: This code example demonstrates sensorless 1-shunt Field-Oriented 
* Control (FOC) for a Permanent Magnet Synchronous Motor (PMSM) using Infineon�s 
* PSOC� Control C3 MCU. 
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "HardwareIface.h"
#include "cybsp.h"
#include "Controller.h"
/*******************************************************************************
* Function prototype
********************************************************************************/
#if defined(APP_KIT_PSC3M5_2GO)
void Motor_Control_POT_Control(void);
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    result = cybsp_init();                 /* Initialize the device and board peripherals */
    CY_ASSERT(result == CY_RSLT_SUCCESS);  /* Board init failed. Stop program execution   */


    // Initialize controller
    HW_IFACE_ConnectFcnPointers();         /* must be called before STATE_MACHINE_Init()  */
    STATE_MACHINE_Init();

    // Enable global interrupts
    __enable_irq();

    (void) (result);
    for (;;)
    {

#if defined(APP_KIT_PSC3M5_2GO)
        Motor_Control_POT_Control();
#endif

#if (CPU_LOAD_CALC_ENABLED)
        MCU_CPULoadCalc();
#endif
    }
}

#if defined(APP_KIT_PSC3M5_2GO)
/*******************************************************************************
* Function Name: Motor_Control_POT_Control
********************************************************************************
* Summary: This function controls the pot and enable the gate drive.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void Motor_Control_POT_Control(void)
{
    static bool flag;
    static float t_min_configured;
    static bool state_check=false;
   /*Disable the driver when pot value is less than or equal to  5% */
    if(motor[0].params_ptr->sys.cmd.source == Internal)
    {
          if((motor[0].sensor_iface_ptr->pot.raw >= 0.05f)) /* flag is added to allow GUI control of driver ON/OFF*/
        {
              if(flag == true)
              {
                flag = false;
                motor[0].vars_ptr->en = true;
              }

        }
          else if(motor[0].sensor_iface_ptr->pot.raw <= 0.025f )
        {
              flag = true;
              motor[0].vars_ptr->en = false;

        }

    }
    else
    {
        flag = false;
    }
    /*Enable or disable Gate driver*/
    Cy_GPIO_Write(EN_IPM_PORT, EN_IPM_NUM, motor[0].vars_ptr->en);
    /* Reduce the minimum time for current measurement in profile mode*/
    if((Prof_Rot_Lock <= motor[0].sm_ptr->current) && (motor[0].sm_ptr->current <= Prof_Lq)&&(motor[0].params_ptr->ctrl.mode == Profiler_Mode))
    {
       if (state_check == false)
       {
          MCU_EnterCriticalSection();
          t_min_configured = motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min;
          motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min =  (t_min_configured <=3.0f)? 0.0f:t_min_configured-3.0f;
          motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min * motor[0].params_ptr->sys.samp.fpwm; // [%]
          MCU_ExitCriticalSection();
          state_check =true;
       }
    }
    else
    {
      if(state_check ==true)
      {
        MCU_EnterCriticalSection();
        motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min =t_min_configured;
        motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_d_min = 2.0f * motor[0].params_ptr->sys.analog.shunt.hyb_mod.adc_t_min * motor[0].params_ptr->sys.samp.fpwm; // [%]
        MCU_ExitCriticalSection();
      }
      state_check =false;
    }
}
#endif
