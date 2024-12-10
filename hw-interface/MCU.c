/******************************************************************************
* File Name:   MCU.c
*
* Description: This file implements the PSOC Control C3 peripheral configuration
* used for 3 shunt foc algorithm.
*
* Related Document: See README.md
*
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

#include "HardwareIface.h"
#include "Controller.h"
#include "probe_scope.h"
#include "MotorCtrlHWConfig.h"
#include "cy_flash.h"

/* EEPROM storage Emulated EEPROM flash. */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)
const uint8_t Em_Eeprom_Storage[srss_0_eeprom_0_PHYSICAL_SIZE] = {0u};


TEMP_SENS_LUT_t   Temp_Sens_LUT   =
{
    .step = 1.0f / (TEMP_SENS_LUT_WIDTH + 1.0f),    // [%], normalized voltage wrt Vcc
    .step_inv = (TEMP_SENS_LUT_WIDTH + 1.0f),       // [1/%], inverse normalized voltage
    .val = {109.5f, 85.4f, 71.7f, 62.0f, 54.3f, 47.7f, 41.9f, 36.5f, 31.4f, 26.3f, 21.2f, 16.0f, 10.2f, 3.7f, -4.3f, -16.1f} // [degree C]
};


cy_stc_sysint_t ISR1_cfg =
{
    .intrSrc = SYNC_ISR1_IRQ,
    .intrPriority = 2
};

void MCU_PhaseUEnterHighZ()
{
    Cy_GPIO_SetHSIOM(PWMUL_PORT, PWMUL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMUH_PORT, PWMUH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMUL_PORT, PWMUL_NUM);
    Cy_GPIO_Clr(PWMUH_PORT, PWMUH_NUM);
}

void MCU_PhaseUExitHighZ()
{
    Cy_GPIO_SetHSIOM(PWMUL_PORT, PWMUL_NUM, PWMUL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMUH_PORT, PWMUH_NUM, PWMUH_HSIOM);
}

void MCU_PhaseVEnterHighZ()
{
    Cy_GPIO_SetHSIOM(PWMVL_PORT, PWMVL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMVH_PORT, PWMVH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMVL_PORT, PWMVL_NUM);
    Cy_GPIO_Clr(PWMVH_PORT, PWMVH_NUM);
}

void MCU_PhaseVExitHighZ()
{
    Cy_GPIO_SetHSIOM(PWMVL_PORT, PWMVL_NUM, PWMVL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMVH_PORT, PWMVH_NUM, PWMVH_HSIOM);
}


void MCU_PhaseWEnterHighZ()
{
    Cy_GPIO_SetHSIOM(PWMWL_PORT, PWMWL_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(PWMWH_PORT, PWMWH_NUM, HSIOM_SEL_GPIO);
    Cy_GPIO_Clr(PWMWL_PORT, PWMWL_NUM);
    Cy_GPIO_Clr(PWMWH_PORT, PWMWH_NUM);
}

void MCU_PhaseWExitHighZ()
{
    Cy_GPIO_SetHSIOM(PWMWL_PORT, PWMWL_NUM, PWMWL_HSIOM);
    Cy_GPIO_SetHSIOM(PWMWH_PORT, PWMWH_NUM, PWMWH_HSIOM);
}

float MCU_TempSensorCalc()
{
    float result;
#if (ACTIVE_TEMP_SENSOR) // Active IC
    result = (hw.mcu.adc_scale.temp_ps * (uint16_t)Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_14_IDX)) - (TEMP_SENSOR_OFFSET / TEMP_SENSOR_SCALE);
#else // Passive NTC
    float lut_input = hw.mcu.adc_scale.temp_ps * (uint16_t)Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_14_IDX);
    uint32_t index = SAT(1U, TEMP_SENS_LUT_WIDTH - 1U, (uint32_t)(lut_input * Temp_Sens_LUT.step_inv));
    float input_index = Temp_Sens_LUT.step * index;
    result = Temp_Sens_LUT.val[index-1U] + (lut_input - input_index) * Temp_Sens_LUT.step_inv * (Temp_Sens_LUT.val[index] - Temp_Sens_LUT.val[index-1U]);
#endif
    return result;
}

void MCU_RunISR0()
{

    const int32_t Curr_ADC_Half_Point_Ticks = (0x1<<11);

    sensor_iface.i_samp_0.raw = hw.mcu.adc_scale.i_uvw * (Curr_ADC_Half_Point_Ticks - (uint16_t)hw.mcu.dma_results[0u]);
    sensor_iface.i_samp_1.raw = hw.mcu.adc_scale.i_uvw * (Curr_ADC_Half_Point_Ticks - (uint16_t)hw.mcu.dma_results[1u]);

    STATE_MACHINE_RunISR0();

    UVW_t d_uvw_cmd_adj = (UVW_t){.w=vars.d_uvw_cmd.w, .v=vars.d_uvw_cmd.v, .u=vars.d_uvw_cmd.u};
    uint32_t pwm_u_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.u);
    uint32_t pwm_v_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.v);
    uint32_t pwm_w_cc = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * d_uvw_cmd_adj.w);

    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, pwm_u_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_U_HW, PWM_U_NUM, pwm_u_cc);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, pwm_v_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_V_HW, PWM_V_NUM, pwm_v_cc);
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, pwm_w_cc);
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_W_HW, PWM_W_NUM, pwm_w_cc);

    uint32_t adc_isr0_cc_samp0, adc_isr0_cc_samp1;
    adc_isr0_cc_samp0 = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * vars.d_samp[0]);
    adc_isr0_cc_samp1 = (uint32_t)(hw.mcu.pwm.duty_cycle_coeff * vars.d_samp[1]);
    Cy_TCPWM_PWM_SetCompare0BufVal(ADC0_ISR0_HW, ADC0_ISR0_NUM, adc_isr0_cc_samp0);
    Cy_TCPWM_PWM_SetCompare1BufVal(ADC1_ISR0_HW, ADC1_ISR0_NUM, adc_isr0_cc_samp1);

    ProbeScope_Sampling();
}

static void DMA_ADC_0_RunISR() {
    Cy_DMA_Channel_ClearInterrupt(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL);
    NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_Descriptor_0);
}

static void DMA_ADC_1_RunISR() {
    Cy_DMA_Channel_ClearInterrupt(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL);
    NVIC_ClearPendingIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_Descriptor_0);
    MCU_RunISR0();
}

void MCU_RunISR1()
{
    Cy_TCPWM_ClearInterrupt(SYNC_ISR1_HW, SYNC_ISR1_NUM, ISR1_cfg.intrSrc);

    Cy_HPPASS_SetFwTrigger(CY_HPPASS_TRIG_2_MSK);/*Firmware trigger for ADC Group-2*/
    sensor_iface.v_dc.raw = hw.mcu.adc_scale.v_dc * (uint16_t)Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_4_IDX);
    sensor_iface.pot.raw = hw.mcu.adc_scale.v_pot * (uint16_t)Cy_HPPASS_SAR_Result_ChannelRead(CY_HPPASS_SAR_CHAN_12_IDX);
    sensor_iface.temp_ps.raw = MCU_TempSensorCalc();

    sensor_iface.digital.fault = !Cy_GPIO_Read(N_FAULT_HW_PORT, N_FAULT_HW_NUM);
    
    // Direction switch
#if defined(DIR_SWITCH_PORT) // switch
    sensor_iface.digital.dir = Cy_GPIO_Read(DIR_SWITCH_PORT, DIR_SWITCH_NUM);
#elif defined(N_DIR_PUSHBTN_PORT) // push button
    static bool user_btn_prev, user_btn = true;
    user_btn_prev = user_btn;
    user_btn = Cy_GPIO_Read(N_DIR_PUSHBTN_PORT, N_DIR_PUSHBTN_NUM);
    sensor_iface.digital.dir = FALL_EDGE(user_btn_prev, user_btn) ? ~sensor_iface.digital.dir : sensor_iface.digital.dir; // toggle switch    
#endif
    
    // Direction LED
#if defined(DIR_LED_PORT)
    Cy_GPIO_Write(DIR_LED_PORT, DIR_LED_NUM, (bool)(sensor_iface.digital.dir));
#endif
    
    // Brake switch
#if defined(N_BRK_SWITCH_PORT)
    sensor_iface.digital.brk = !Cy_GPIO_Read(N_BRK_SWITCH_PORT, N_BRK_SWITCH_NUM);
#else
    sensor_iface.digital.brk = 0x0; // no brake switch   
#endif    
    
    // Control ISR1
    STATE_MACHINE_RunISR1();
    
    // SW fault LED
#if defined(N_FAULT_LED_SW_PORT) // seperate leds for hw and sw faults
    Cy_GPIO_Write(N_FAULT_LED_SW_PORT, N_FAULT_LED_SW_NUM, (bool)(!faults.flags_latched.sw.reg));
#elif defined(FAULT_LED_ALL_PORT) // one led for all faults
#if defined(COMPONENT_CAT1)
    Cy_GPIO_Write(FAULT_LED_ALL_PORT, FAULT_LED_ALL_NUM, (bool)(faults.flags_latched.all));
#endif
#endif

    NVIC_ClearPendingIRQ((IRQn_Type)ISR1_cfg.intrSrc);
}

void MCU_EnterCriticalSection()
{
    hw.mcu.interrupt.state = Cy_SysLib_EnterCriticalSection();
}

void MCU_ExitCriticalSection()
{
    Cy_SysLib_ExitCriticalSection(hw.mcu.interrupt.state);
}

void MCU_GateDriverEnterHighZ()
{
    MCU_PhaseUEnterHighZ();
    MCU_PhaseVEnterHighZ();
    MCU_PhaseWEnterHighZ();
}

void MCU_GateDriverExitHighZ()
{
    MCU_PhaseUExitHighZ();
    MCU_PhaseVExitHighZ();
    MCU_PhaseWExitHighZ();
}

void MCU_Init()
{
    MCU_InitChipInfo();
    MCU_InitInterrupts();
    MCU_InitADCs();
    MCU_InitDMAs();
    MCU_InitTimers();
    ProbeScope_Init((uint32_t)params.sys.samp.fs0);
    sensor_iface.digital.dir = true; // initial direction is positive
}

void MCU_InitChipInfo()
{
    mc_info.chip_id = Cy_SysLib_GetDevice();
    mc_info.chip_id <<= 16;
    mc_info.chip_id |= Cy_SysLib_GetDeviceRevision();
}

void MCU_InitInterrupts()
{
    // Interrupt callbacks and priorities (higher value = lower urgency) .......

    // DMA_ADC_0:
    cy_stc_sysint_t DMA_ADC_0_cfg = { .intrSrc = DMA_ADC_0_IRQ, .intrPriority = 0 };
    Cy_SysInt_Init(&DMA_ADC_0_cfg, DMA_ADC_0_RunISR);
    // DMA_ADC_1:
    cy_stc_sysint_t DMA_ADC_1_cfg = { .intrSrc = DMA_ADC_1_IRQ, .intrPriority = 0 };
    Cy_SysInt_Init(&DMA_ADC_1_cfg, DMA_ADC_1_RunISR);

    // ISR1:
    Cy_SysInt_Init(&ISR1_cfg, MCU_RunISR1);

    // NVIC connections ........................................................
    hw.mcu.interrupt.nvic_dma_adc_0 = DMA_ADC_0_IRQ;
    hw.mcu.interrupt.nvic_dma_adc_1 = DMA_ADC_1_IRQ;
    hw.mcu.interrupt.nvic_sync_isr1 = SYNC_ISR1_IRQ;
}

void MCU_InitADCs()
{
    // ADC conversion coefficients .............................................
    float cs_gain = ADC_CS_OPAMP_GAIN;

    hw.mcu.adc_scale.i_uvw = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * params.sys.analog.shunt.res * cs_gain); // [A/ticks]
    hw.mcu.adc_scale.v_uvw = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * ADC_SCALE_VUVW); // [V/ticks]
    hw.mcu.adc_scale.v_dc = (ADC_VREF_GAIN * CY_CFG_PWR_VDDA_MV * 1.0E-3f) / ((1<<12U) * ADC_SCALE_VDC); // [V/ticks]
    hw.mcu.adc_scale.v_pot = 1.0f / (1<<12U); // [%/ticks]
    hw.mcu.adc_scale.temp_ps = 1.0f / (1<<12U); // [1/ticks], normalized voltage wrt Vcc
}

void MCU_InitDMAs()
{
    // Configure DMA descriptors ...............................................

    Cy_DMA_Descriptor_Init(&DMA_ADC_0_Descriptor_0, &DMA_ADC_0_Descriptor_0_config);
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_ADC_0_Descriptor_0, (void*)CY_HPPASS_SAR_CHAN_RSLT_PTR(3));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_ADC_0_Descriptor_0, &hw.mcu.dma_results[0u]);

    Cy_DMA_Descriptor_Init(&DMA_ADC_1_Descriptor_0, &DMA_ADC_1_Descriptor_0_config);
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_ADC_1_Descriptor_0, (void*)CY_HPPASS_SAR_CHAN_RSLT_PTR(3));
    Cy_DMA_Descriptor_SetDstAddress(&DMA_ADC_1_Descriptor_0, &hw.mcu.dma_results[1u]);

    // Configure DMA channels ..................................................
    Cy_DMA_Channel_Init(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, &DMA_ADC_0_Descriptor_0);
    Cy_DMA_Channel_SetInterruptMask(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_ADC_0_HW, DMA_ADC_0_CHANNEL);

    Cy_DMA_Channel_Init(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_channelConfig);
    Cy_DMA_Channel_SetDescriptor(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, &DMA_ADC_1_Descriptor_0);
    Cy_DMA_Channel_SetInterruptMask(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_ADC_1_HW, DMA_ADC_1_CHANNEL);
}

void MCU_InitTimers()
{
    // Clock frequencies .......................................................
    hw.mcu.clk.tcpwm = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)CLK_TCPWM_GRP_NUM, CY_SYSCLK_DIV_8_BIT, CLK_TCPWM_NUM); // [Hz]
    hw.mcu.clk.hall = Cy_SysClk_PeriPclkGetFrequency((en_clk_dst_t)CLK_HALL_GRP_NUM, CY_SYSCLK_DIV_8_BIT, CLK_HALL_NUM); // [Hz]

    // Timer calculations ......................................................
    hw.mcu.pwm.count = 0U;
    hw.mcu.pwm.period = ((uint32_t)(hw.mcu.clk.tcpwm * params.sys.samp.tpwm))&(~((uint32_t)(0x1))); // must be even
    hw.mcu.pwm.duty_cycle_coeff = (float)(hw.mcu.pwm.period >> 1);
    hw.mcu.isr0.count = 0U;
    hw.mcu.isr0.period = hw.mcu.pwm.period * params.sys.samp.fpwm_fs0_ratio;
    hw.mcu.isr0.duty_cycle_coeff = (float)(hw.mcu.isr0.period);
    hw.mcu.isr1.count = 0U;
    hw.mcu.isr1.period = hw.mcu.isr0.period * params.sys.samp.fs0_fs1_ratio;
    hw.mcu.isr1.duty_cycle_coeff = (float)(hw.mcu.isr1.period);
    hw.mcu.isr0_exe.sec_per_tick = (1.0f/hw.mcu.clk.tcpwm); // [sec/ticks]
    hw.mcu.isr0_exe.inv_max_time = params.sys.samp.fs0; // [1/sec]
    hw.mcu.isr1_exe.sec_per_tick = (1.0f/hw.mcu.clk.tcpwm); // [sec/ticks]
    hw.mcu.isr1_exe.inv_max_time = params.sys.samp.fs1; // [1/sec]  
  
    // Configure timers (TCPWMs) .....................................

    uint32_t cc0 = (hw.mcu.pwm.period >> 1);
    
    Cy_TCPWM_PWM_Init(ADC0_ISR0_HW, ADC0_ISR0_NUM, &ADC0_ISR0_config);
    Cy_TCPWM_PWM_Init(ADC1_ISR0_HW, ADC1_ISR0_NUM, &ADC1_ISR0_config);
    Cy_TCPWM_PWM_SetPeriod0(ADC0_ISR0_HW, ADC0_ISR0_NUM, hw.mcu.isr0.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetPeriod0(ADC1_ISR0_HW, ADC1_ISR0_NUM, hw.mcu.isr0.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetCompare0Val(ADC0_ISR0_HW, ADC0_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    Cy_TCPWM_PWM_SetCompare1Val(ADC1_ISR0_HW, ADC1_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    Cy_TCPWM_PWM_SetCompare0BufVal(ADC0_ISR0_HW, ADC0_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times   
    Cy_TCPWM_PWM_SetCompare1BufVal(ADC1_ISR0_HW, ADC1_ISR0_NUM, cc0); // Read ADCs at the middle of lower switches' on-times
    
    Cy_TCPWM_PWM_Init(PWM_U_HW, PWM_U_NUM, &PWM_U_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_U_HW, PWM_U_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    Cy_TCPWM_PWM_Init(PWM_V_HW, PWM_V_NUM, &PWM_V_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_V_HW, PWM_V_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    Cy_TCPWM_PWM_Init(PWM_W_HW, PWM_W_NUM, &PWM_W_config);
    Cy_TCPWM_PWM_SetPeriod0(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 1); // Triangle carrier
    Cy_TCPWM_PWM_SetCompare0Val(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1Val(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare0BufVal(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%
    Cy_TCPWM_PWM_SetCompare1BufVal(PWM_W_HW, PWM_W_NUM, hw.mcu.pwm.period >> 2); // Start with duty cycle = 50%

    cc0 = hw.mcu.isr1.period - 1U;
    Cy_TCPWM_PWM_Init(SYNC_ISR1_HW, SYNC_ISR1_NUM, &SYNC_ISR1_config);
    Cy_TCPWM_PWM_SetPeriod0(SYNC_ISR1_HW, SYNC_ISR1_NUM, hw.mcu.isr1.period - 1U); // Sawtooth carrier
    Cy_TCPWM_PWM_SetCompare0Val(SYNC_ISR1_HW, SYNC_ISR1_NUM, cc0);

    Cy_TCPWM_PWM_Init(PWM_START_HW, PWM_START_NUM, &PWM_START_config);

    sensor_iface.uvw_idx = &ctrl.volt_mod.uvw_idx;
}

void MCU_StartPeripherals()
{
    MCU_EnterCriticalSection(); // No ISRs beyond this point

    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);
    Cy_DMA_Enable(DMA_ADC_0_HW);
    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Enable(DMA_ADC_1_HW);


    NVIC_EnableIRQ(hw.mcu.interrupt.nvic_sync_isr1);

    Cy_TCPWM_PWM_Enable(ADC0_ISR0_HW, ADC0_ISR0_NUM);
    Cy_TCPWM_PWM_Enable(ADC1_ISR0_HW, ADC1_ISR0_NUM);    
    Cy_TCPWM_PWM_Enable(PWM_U_HW, PWM_U_NUM);
    Cy_TCPWM_PWM_Enable(PWM_V_HW, PWM_V_NUM);
    Cy_TCPWM_PWM_Enable(PWM_W_HW, PWM_W_NUM);   
    Cy_TCPWM_PWM_Enable(SYNC_ISR1_HW, SYNC_ISR1_NUM);
    Cy_TCPWM_PWM_Enable(PWM_START_HW, PWM_START_NUM);
    Cy_TCPWM_TriggerStart_Single(PWM_START_HW, PWM_START_NUM); // Start PWM_U, PWM_V, PWM_W, ADC0_ISR0 and SYNC_ISR1

    MCU_ExitCriticalSection();
}

void MCU_StopPeripherals()
{
    MCU_EnterCriticalSection(); // No ISRs beyond this point
    Cy_TCPWM_PWM_Disable(PWM_START_HW, PWM_START_NUM);
    Cy_TCPWM_PWM_Disable(SYNC_ISR1_HW, SYNC_ISR1_NUM);
    Cy_TCPWM_PWM_Disable(PWM_W_HW, PWM_W_NUM);
    Cy_TCPWM_PWM_Disable(PWM_V_HW, PWM_V_NUM);
    Cy_TCPWM_PWM_Disable(PWM_U_HW, PWM_U_NUM);
    Cy_TCPWM_PWM_Disable(ADC1_ISR0_HW, ADC1_ISR0_NUM); 
    Cy_TCPWM_PWM_Disable(ADC0_ISR0_HW, ADC0_ISR0_NUM);

    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_sync_isr1);

    Cy_DMA_Disable(DMA_ADC_1_HW);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_1);
    Cy_DMA_Disable(DMA_ADC_0_HW);
    NVIC_DisableIRQ(hw.mcu.interrupt.nvic_dma_adc_0);

    MCU_ExitCriticalSection();
}

void MCU_FlashInit()
{
    // EEPROM Emulator
    hw.mcu.eeprom.config.eepromSize = srss_0_eeprom_0_SIZE,
    hw.mcu.eeprom.config.simpleMode = srss_0_eeprom_0_SIMPLEMODE,
    hw.mcu.eeprom.config.wearLevelingFactor = srss_0_eeprom_0_WEARLEVELING_FACTOR,
    hw.mcu.eeprom.config.redundantCopy = srss_0_eeprom_0_REDUNDANT_COPY,
    hw.mcu.eeprom.config.blockingWrite = srss_0_eeprom_0_BLOCKINGMODE,
    hw.mcu.eeprom.config.userFlashStartAddr = (uint32_t)&(Em_Eeprom_Storage[0U]),

    hw.mcu.eeprom.status = Cy_Em_EEPROM_Init(&hw.mcu.eeprom.config, &hw.mcu.eeprom.context);

    hw.mcu.eeprom.init_done = true;
}

bool MCU_FlashWriteParams(PARAMS_t* ram_data)
{

    if(!hw.mcu.eeprom.init_done)
    {
        MCU_FlashInit();
    }

    if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
    {
        return false;
    }

    hw.mcu.eeprom.status = Cy_Em_EEPROM_Write(0U, ram_data, sizeof(PARAMS_t), &hw.mcu.eeprom.context);
    if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
    {
        return false;
    }


    return true;
}

bool MCU_FlashReadParams(PARAMS_ID_t id, PARAMS_t* ram_data)
{
    if(!hw.mcu.eeprom.init_done)
    {
        MCU_FlashInit();
    }

    if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
    {
        return false;
    }

    hw.mcu.eeprom.status = Cy_Em_EEPROM_Read(0u, ram_data, sizeof(PARAMS_t), &hw.mcu.eeprom.context);
    if (CY_EM_EEPROM_SUCCESS != hw.mcu.eeprom.status)
    {
        return false;
    }

    if (ram_data->id.code != id.code || ram_data->id.build_config != id.build_config || ram_data->id.ver != id.ver)
    {
        return false;
    }
    return true;
}


bool MCU_ArePhaseVoltagesMeasured()
{
  return false;
}


