/**
 * @file cem102_driver_signal.c
 * @brief CEM102 Driver Signal Path implementation
 *
 * @copyright @parblock
 * Copyright (c) 2023 Semiconductor Components Industries, LLC (d/b/a
 * onsemi), All Rights Reserved
 *
 * This code is the property of onsemi and may not be redistributed
 * in any form without prior written permission from onsemi.
 * The terms of use and warranty for this code are covered by contractual
 * agreements between onsemi and the licensee.
 *
 * This is Reusable Code.
 * @endparblock 
 */
#include <string.h>
#include <cem102_driver.h>
#include <app_cem102.h>

extern int8_t stripping_state ;


/* ADC gain global variable. Units are uV/LSB */
static float adc_gain[3] = {
                               CEM102_ADC_GAIN_NOM,
                               CEM102_ADC_GAIN_NOM,
                               CEM102_ADC_GAIN_NOM,
                           };

/* ----------------------------------------------------------------------------
 * Configuration Functions
 * --------------------------------------------------------------------------*/

int CEM102_Signal_ClockConfig(const CEM102_Device *p_cfg, uint16_t prescale, uint16_t delay)
{
    return CEM102_Register_Write(p_cfg, CHCFG_LSAD_CLK,
                                 (((delay << CHCFG_LSAD_CLK_LSAD_START_DLY_POS)
                                    & CHCFG_LSAD_CLK_LSAD_START_DLY_MASK) |
                                  LSAD1_START_DLY_ENABLE | LSAD2_START_DLY_ENABLE |
                                  LSAD1_OFFSET0 | LSAD2_OFFSET0 |
                                 (prescale & CHCFG_LSAD_CLK_LSAD_CLK_PRESCALE_MASK)));
}

int CEM102_Signal_TIAConfig(const CEM102_Device *p_cfg, int tia, uint16_t enable, uint16_t tia_cfg)
{
    uint16_t temp_ana0, temp_ana1;
    int result;

    result = CEM102_Register_Read(p_cfg, ANA_CFG0, &temp_ana0);
    result |= CEM102_Register_Read(p_cfg, ANA_CFG1, &temp_ana1);

    if (result == ERRNO_NO_ERROR)
    {
        /* Apply the supplied configuration, leaving the remaining bits unmodified. */
        temp_ana0 &= ~(CEM102_TIA_MASK <<
                       ((tia - 1) * (ANA_CFG0_CH2_TIA_FB_RES_POS - ANA_CFG0_CH1_TIA_FB_RES_POS)));
        tia_cfg &= (CEM102_TIA_MASK <<
                    ((tia - 1) * (ANA_CFG0_CH2_TIA_FB_RES_POS - ANA_CFG0_CH1_TIA_FB_RES_POS)));
        result = CEM102_Register_Write(p_cfg, ANA_CFG0, temp_ana0 | tia_cfg);

        temp_ana1 &= ~((1 << ANA_CFG1_CH1_TIA_CFG_POS)
                       << ((tia - 1) * (ANA_CFG1_CH2_TIA_CFG_POS - ANA_CFG1_CH1_TIA_CFG_POS)));
        enable &= ((1 << ANA_CFG1_CH1_TIA_CFG_POS)
                   << ((tia - 1) * (ANA_CFG1_CH2_TIA_CFG_POS - ANA_CFG1_CH1_TIA_CFG_POS)));
        result |= CEM102_Register_Write(p_cfg, ANA_CFG1, temp_ana1 | enable);
    }

    return result;
}

int CEM102_Signal_SwitchConfig(const CEM102_Device *p_cfg, int channel, uint16_t channel_cfg)
{
    uint16_t temp;
    int result;

    /* Mask the provided channel configuration to ensure this only affects the
     * channel switches, and to ensure that SW114 (WE1_DAC to AT1) is
     * disconnected.
     */
    channel_cfg &= (CEM102_SW_CFG_MASK - (0x1 << ANA_SW_CFG0_CH1_SC114_CFG_POS));

    if (channel == WE1)
    {
        /* Since the channel 1 switches share a register with the RE switches,
         * we need to read and write back the RE switch configuration (clearing
         * out the 10 channel configuration bits this function should be writing).
         */
        result = CEM102_Register_Read(p_cfg, ANA_SW_CFG0, &temp);

        if (result == ERRNO_NO_ERROR)
        {
            temp &= ~CEM102_SW_CFG_MASK;
            result = CEM102_Register_Write(p_cfg, ANA_SW_CFG0, temp | channel_cfg);
        }
    }
    else if (channel == WE2)
    {
        result = CEM102_Register_Write(p_cfg, ANA_SW_CFG1, channel_cfg);
    }
    else
    {
        return ERRNO_PARAM_ERROR;
    }
    return result;
}

int CEM102_Signal_RE_ATBus_Config(const CEM102_Device *p_cfg, int channel, uint16_t config)
{
    uint16_t temp;
    int result;

    /* RE Switch Configuration */
    if (channel == WE1)
    {
        result = CEM102_Register_Read(p_cfg, ANA_SW_CFG0, &temp);

        if (result == ERRNO_NO_ERROR)
        {
            temp &= CEM102_SW_CFG_MASK;
            result = CEM102_Register_Write(p_cfg, ANA_SW_CFG0, temp | config);
        }
    }

    /* SA Switch Configuration */
    else if (channel == WE2)
    {
        result = CEM102_Register_Write(p_cfg, ANA_SW_CFG2, config);
    }

    /* ST Switch Configuration */
    else if (channel == RE)
    {
        result = CEM102_Register_Read(p_cfg, ANA_SW_CFG3, &temp);

        if (result == ERRNO_NO_ERROR)
        {
            temp &= CEM102_SW_CFG3_BUF_MASK;
            result = CEM102_Register_Write(p_cfg, ANA_SW_CFG3, temp | config);
        }
    }
    else
    {
        return ERRNO_PARAM_ERROR;
    }

    return result;
}

int CEM102_Signal_BufferConfig(const CEM102_Device *p_cfg, int channel, uint16_t channel_cfg,
                               uint16_t buf_cfg)
{
    uint16_t temp_ana0 = 0;
    uint16_t temp_ana1 = 0;
    int result = ERRNO_NO_ERROR;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        result = CEM102_Register_Read(p_cfg, ANA_CFG0, &temp_ana0);
        result |= CEM102_Register_Read(p_cfg, ANA_CFG1, &temp_ana1);
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Apply the supplied configuration, leaving the remaining bits unmodified. */
        temp_ana0 &= ~((ANA_CFG0_CH1_BUF_IB_MASK | (1 << ANA_CFG0_CH1_LPF_BW_CFG_POS)) <<
                       ((channel - 1) * (ANA_CFG0_CH2_BUF_IB_POS - ANA_CFG0_CH1_BUF_IB_POS)));
        channel_cfg &=  ((ANA_CFG0_CH1_BUF_IB_MASK | (1 << ANA_CFG0_CH1_LPF_BW_CFG_POS)) <<
                         ((channel - 1) * (ANA_CFG0_CH2_BUF_IB_POS - ANA_CFG0_CH1_BUF_IB_POS)));
        result = CEM102_Register_Write(p_cfg, ANA_CFG0, temp_ana0 | channel_cfg);

        temp_ana1 &= ~(((0x1 << ANA_CFG1_CH1_BUF_CFG_POS) | (0x1 << ANA_CFG1_CH1_BUF_BYPASS_CFG_POS))
                       << ((channel - 1) * (ANA_CFG1_CH2_BUF_CFG_POS - ANA_CFG1_CH1_BUF_CFG_POS)));
        buf_cfg &= (((0x1 << ANA_CFG1_CH1_BUF_CFG_POS) | (0x1 << ANA_CFG1_CH1_BUF_BYPASS_CFG_POS))
                    << ((channel - 1) * (ANA_CFG1_CH2_BUF_CFG_POS - ANA_CFG1_CH1_BUF_CFG_POS)));
        result |= CEM102_Register_Write(p_cfg, ANA_CFG1, temp_ana1 | buf_cfg);
    }

    return result;
}

int CEM102_Signal_WEDACConfig(const CEM102_Device *p_cfg, int dac, uint16_t dac_cfg,
                            uint16_t interrupt_cfg)
{
    uint16_t temp = 0;
    int result = ERRNO_NO_ERROR;

    /* Confirm the requested DAC value is between 0.4 V and 1.9 V if the DAC
     * will be enabled; load temp with the DAC value only (removing the enable\
     * if present) */
    if ((dac_cfg & DAC_WE1_ENABLE) != 0)
    {
        temp = dac_cfg & ~(0x1 << DC_DAC_WE1_CTRL_DAC_WE1_CFG_POS);
        if ((temp < CEM102_DAC_MIN) || (temp > CEM102_DAC_MAX))
        {
            result = ERRNO_PARAM_ERROR;
        }
    }

    /* Allow for either WE DAC to be selected */
    if ((dac != WE1) && (dac != WE2))
    {
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Write the DAC configuration */
        result = CEM102_Register_Write(p_cfg, (DC_DAC_WE1_CTRL + (dac - 1)), dac_cfg);
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Write the DAC interrupt configuration, maintaining all other
         * interrupt configurations.
         */
        result = CEM102_Register_Read(p_cfg, SYSCTRL_IRQ_CFG, &temp);
        temp &= ~(0x1 << SYSCTRL_IRQ_CFG_DC_DAC_LOAD_IRQ_CFG_POS);
        interrupt_cfg &= (0x1 << SYSCTRL_IRQ_CFG_DC_DAC_LOAD_IRQ_CFG_POS);
        if (result == ERRNO_NO_ERROR)
        {
            result = CEM102_System_IRQConfig(p_cfg, temp | interrupt_cfg);
        }
    }
    return result;
}

int CEM102_Signal_REDACConfig(const CEM102_Device *p_cfg, uint16_t dac_cfg,
                              uint16_t sw_cfg, uint16_t buf_cfg)
{
    uint16_t temp;
    int result;

    /* Confirm the requested DAC value is between 0.4 V and 1.9 V if the DAC
     * will be enabled; load temp with the DAC value only (removing the enable
     * if present) */
    if ((dac_cfg & DAC_RE_ENABLE) != 0)
    {
        temp = dac_cfg & ~(0x1 << DC_DAC_WE1_CTRL_DAC_WE1_CFG_POS);
        if ((temp < CEM102_DAC_MIN) || (temp > CEM102_DAC_MAX))
        {
            return ERRNO_PARAM_ERROR;
        }
    }

    /* Write the DAC configuration */
    result = CEM102_Register_Write(p_cfg, DC_DAC_RE_CTRL, dac_cfg);

    /* Write the switch configuration without modifying the ten CH1 switches */
    if (result == ERRNO_NO_ERROR)
    {
        result = CEM102_Register_Read(p_cfg, ANA_SW_CFG0, &temp);
        if (result == ERRNO_NO_ERROR)
        {
            temp &= CEM102_SW_CFG_MASK;
            sw_cfg &= ~CEM102_SW_CFG_MASK;
            result = CEM102_Register_Write(p_cfg, ANA_SW_CFG0, temp | sw_cfg);
        }
    }

    /* Write the buffer configuration without changing any of the other buffer
     * configurations */
    if (result == ERRNO_NO_ERROR)
    {
        result = CEM102_Register_Read(p_cfg, ANA_CFG1, &temp);
        if (result == ERRNO_NO_ERROR)
        {
            temp &= ~((0x1 << ANA_CFG1_RE_BUF_HP_MODE_CFG_POS) | (0x1 << ANA_CFG1_RE_BUF_CFG_POS));
            buf_cfg &= ((0x1 << ANA_CFG1_RE_BUF_HP_MODE_CFG_POS) | (0x1 << ANA_CFG1_RE_BUF_CFG_POS));
            result = CEM102_Register_Write(p_cfg, ANA_CFG1, temp | buf_cfg);
        }
    }
    return result;
}

int CEM102_Signal_ADCConfig(const CEM102_Device *p_cfg, int channel, uint16_t lsad_cfg,
                            uint16_t thresh_cfg, uint16_t buf_cfg, uint16_t accum_cfg)
{
    int result = ERRNO_NO_ERROR;
    int channel_offset;

    /* Sanity check that the parameters are okay. If the chopper is enabled,
     * the accumulator specified needs to be odd since we accumulate over
     * (n + 1) samples. */
    if ((lsad_cfg & LSAD1_SYS_CHOP_ENABLE) != 0)
    {
        if ((accum_cfg & (1 << CHCFG_CH1_ACCUM_CH1_ACCUM_SAMPLE_CNT_POS)) !=
             (1 << CHCFG_CH1_ACCUM_CH1_ACCUM_SAMPLE_CNT_POS))
        {
            result = ERRNO_PARAM_ERROR;
        }
    }

    if ((channel != WE1) && (channel != WE2))
    {
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        channel_offset = (channel - 1) * (CHCFG_CH2_THRESHOLD - CHCFG_CH1_THRESHOLD);

        /* Configure the LSAD */
        result = CEM102_Register_Write(p_cfg, CHCFG_CH1_LSAD + channel_offset, lsad_cfg);

        /* Configure the threshold */
        result |= CEM102_Register_Write(p_cfg, CHCFG_CH1_THRESHOLD + channel_offset, thresh_cfg);

        /* Configure the buffer and violation block */
        result |= CEM102_Register_Write(p_cfg, CHCFG_CH1_BUFFER + channel_offset, buf_cfg);

        /* Configure the accumulation */
        result |= CEM102_Register_Write(p_cfg, CHCFG_CH1_ACCUM + channel_offset, accum_cfg);
    }

    return result;
}

int CEM102_Signal_DirectADCConfig(const CEM102_Device *p_cfg, uint32_t channel)
{
    uint16_t ana_cfg0 = 0;
    uint16_t ana_cfg1 = 0;
    int result = ERRNO_NO_ERROR;

    if ((channel != WE1) && (channel != WE2))
    {
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Disable buffer and enable buffer bypass to connect to the ADC
         * directly */
        result = CEM102_Register_Read(p_cfg, ANA_CFG1, &ana_cfg1);

        /* Enable buffer bypass */
        ana_cfg1 |= (CH1_BUF_BYPASS_ENABLE <<
                    ((channel - 1) * ANA_CFG1_CH2_TIA_CFG_POS));

        /* Disable the buffer */
        ana_cfg1 &= ~(CH1_BUF_ENABLE <<
                    ((channel - 1) * ANA_CFG1_CH2_TIA_CFG_POS));

        /* Write back the ANA_CFG1 register */
        result |= CEM102_Register_Write(p_cfg, ANA_CFG1, ana_cfg1);

        /* Disable internal FB network, read modify write
         * ANA_CFG0 */
        result |= CEM102_Register_Read(p_cfg, ANA_CFG0, &ana_cfg0);

        /* Enable the internal feedback network */
        ana_cfg0 |= (CH1_TIA_INT_FB_ENABLE <<
                    ((channel - 1) * ANA_CFG0_CH2_BUF_IB_POS));

        /* Enable the high frequency (10 kHz) low pass filter setting */
        ana_cfg0 |= (CH1_LPF_BW_CFG_ENABLE <<
                    ((channel - 1) * ANA_CFG0_CH2_BUF_IB_POS));

        /* Write back ANA_CFG0 register */
        result |= CEM102_Register_Write(p_cfg, ANA_CFG0, ana_cfg0);
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * Measurement Functions
 * --------------------------------------------------------------------------*/

int CEM102_Signal_StartMeasurement(const CEM102_Device *p_cfg, uint16_t cmd)
{
    uint16_t temp;
    int result;

    result = CEM102_Register_Read(p_cfg, SYS_STATUS, &temp);
    if (result == ERRNO_NO_ERROR)
    {
        if ((temp & SYS_STATUS_SYS_STATE_MASK) != IDLE_STATE)
        {
            return ERRNO_MEASUREMENT_BUSY;
        }
        if ((temp & (DIGITAL_RESET | VDDA_ERR)) != 0)
        {
            return ERRNO_STATE_ERROR;
        }
        result = CEM102_Register_Write(p_cfg, SYSCTRL_CMD, cmd);
    }
    return result;
}

void CEM102_Signal_ProcessADCSingle(const CEM102_Device *p_cfg, uint16_t data,
                                    uint16_t lsad_cfg, int sample_cnt, int shift,
                                    float *p_processed_data)
{
    /* Implement this with the process double function, using a different shift value */
    CEM102_Signal_ProcessADCDouble(p_cfg,
                                   (uint32_t) data,
                                   lsad_cfg,
                                   sample_cnt,
                                   shift - 16,
                                   p_processed_data);
}

void CEM102_Signal_ProcessADCDouble(const CEM102_Device *p_cfg, uint32_t data,
                                    uint16_t lsad_cfg, int sample_cnt, int shift,
                                    float *p_processed_data)
{
    /* If the gain is double we need an extra shift */
    if (lsad_cfg == LSAD1_GAIN_DOUBLE)
    {
        shift++;
    }

    /* Add in the default shift */
    shift += 17;

    *p_processed_data = (float) (data / ((sample_cnt + 1) << shift));
}

int CEM102_Signal_MeasureReference(const CEM102_Device *p_cfg, uint32_t channel)
{
    int result = ERRNO_NO_ERROR;
    int channel_offset = ((channel - 1) * 4);
    uint32_t lsad_setting = 0;
    uint32_t acc_setting = 0;
    uint16_t wedac_setting = 0;
    double wedac_offset = 0;
    double wedac_gain = 0;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Set LSAD settings based on device version */
        if (trim_struct_v1 != NULL)
        {
            lsad_setting = LSAD1_ORSD64_RSD14 | LSAD1_BUF_CHOP_CLK_DIV64;
            acc_setting = CEM102_NACC_REF_R1;

            if (channel == WE1)
            {
                wedac_setting = (trim_struct_v1->dac[0] & 0xFF00) >> 8;
                wedac_setting += (trim_struct_v1->dac[1] & 0xF) << 8;
            }
            else if (channel == WE2)
            {
                wedac_setting = (trim_struct_v1->dac[1] & 0xFFF0) >> 4;
            }
            else
            {
                /* Invalid channel selected */
                result = ERRNO_PARAM_ERROR;
            }
            wedac_setting = (CEM102_DAC_MV(CEM102_DAC_VOLTAGE_500MV) -
                             wedac_setting);
        }
        else if (trim_struct != NULL)
        {
            lsad_setting = LSAD1_ORSD8192_RSD8 | LSAD1_BUF_CHOP_CLK_DIV8192;
            acc_setting = CEM102_NACC_REF;

            /* Extract DAC Offset and Gain Values from OTP */
            wedac_offset = (trim_struct->dac[channel - 1] & 0x00FF);
            wedac_gain = (trim_struct->dac[channel - 1] & 0xFF00) >> 8;

            /* Calculate the DAC Offset for FW */
            wedac_offset = ((wedac_offset - CEM102_DAC_OFFSET_O2) / CEM102_DAC_OFFSET_G2) * 1000;

            /* Calculate DAC Gain for FW */
            wedac_gain = ((wedac_gain - (CEM102_DAC_GAIN_O1)) / CEM102_DAC_GAIN_G1) * 1000;

            /* Calculate DAC Code to set WE electrode at 1.2V */
            wedac_setting = (1200.0f - wedac_offset) / wedac_gain;
        }
        else
        {
            /* OTP has not been read from the device */
            result =  ERRNO_GENERAL_FAILURE;
        }
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Configure WEDAC voltage using value from OTP */
        result |= CEM102_Signal_WEDACConfig(p_cfg, channel,
                                            ((DC_DAC_WE1_CTRL_DAC_WE1_VALUE_MASK &
                                            wedac_setting) | DAC_WE1_ENABLE),
                                            DC_DAC_LOAD_IRQ_DISABLE);

        /* Configure device to duplicate production test settings */
        result |= CEM102_Register_Write(p_cfg,
                                        ANA_CFG0,
                                        (CH1_TIA_INT_FB_ENABLE |
                                        CH1_TIA_FB_RES_4M |
                                        CH1_BUF_IB2) <<
                                        (8 * (channel - 1)));

        result |= CEM102_Register_Write(p_cfg,
                                        ANA_CFG1,
                                        (CH1_BUF_ENABLE | CH1_TIA_ENABLE) <<
                                        (3 * (channel - 1)));

        result |= CEM102_Register_Write(p_cfg,
                                        CHCFG_LSAD_CLK,
                                        ((LSAD1_OFFSET0 | LSAD1_START_DLY_ENABLE) <<
                                        (channel - 1)) |
                                        (0x19 << CHCFG_LSAD_CLK_LSAD_START_DLY_POS) |
                                        LSAD_PRESCALE1);

        result |= CEM102_Register_Write(p_cfg,
                                        (CHCFG_CH1_BUFFER + channel_offset),
                                        CH1_DOUBLE);

        /* Use many samples for high accuracy */
        result |= CEM102_Register_Write(p_cfg,
                                        CHCFG_CH1_ACCUM + channel_offset,
                                        (CHCFG_CH1_ACCUM_CH1_ACCUM_SAMPLE_CNT_MASK & (acc_setting - 1)) |
                                        (CH1_MEASUREMENT_ENABLE));

        result |= CEM102_Register_Write(p_cfg,
                                        CHCFG_CH1_LSAD + channel_offset,
                                        LSAD1_ITRIM_1p00 |
                                        LSAD1_GAIN_UNITY |
                                        LSAD1_ADC_CHOP_CLK_DIV16 |
                                        lsad_setting |
                                        LSAD1_SYS_CHOP_ENABLE);

        if (result == ERRNO_NO_ERROR)
        {
            result |= CEM102_Signal_CalibrateChannel(p_cfg, channel);
        }
    }

    return result;
}

int CEM102_Signal_CalibrateChannel(const CEM102_Device *p_cfg, uint32_t channel)
{
    int result = ERRNO_NO_ERROR;
    uint16_t at_sw = 0;
    uint16_t ana_cfg2 = 0;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* This function assumes that the application has configured the device
         * measurement settings to align with the application setting. However,
         * we do ensure that the internal reference current is enabled. */
        result |= CEM102_Register_Read(p_cfg, ANA_CFG2, &ana_cfg2);

        result |= CEM102_Register_Write(p_cfg,
                                        ANA_CFG2,
                                        SENSOR_CAL_ENABLE | ana_cfg2);

        /* Configure switches to connect 100 nA reference current to the TIA */

        /* Connect analog test bus to the TIA. SC113 for channel 1,
         * SC213 for channel 2. Connect the TIA to the ADC via SC119/SC120
         * or SC219/SC220*/
        result |= CEM102_Register_Write(p_cfg,
                                        ANA_SW_CFG0 + (channel - 1),
                                        CH1_SC113_CONNECT |
                                        CH1_SC119_SC120_CONNECT);

        if (channel == WE1)
        {
            at_sw = ATBUS_SA7_CONNECT;
        }
        else if (channel == WE2)
        {
            at_sw = ATBUS_SA14_CONNECT;
        }
        else
        {
            result |= ERRNO_PARAM_ERROR;
        }

        /* Connect IBN_100N (100 nA reference current) to analog test bus. */
        result |= CEM102_Register_Write(p_cfg,
                                        ANA_SW_CFG2,
                                        at_sw);

        /* We are ready to start the measurement, start a single measurement */
        if (result == ERRNO_NO_ERROR)
        {
            result |= CEM102_Signal_StartMeasurement(p_cfg, SINGLE_CMD);
        }

    }
    return result;
}

int CEM102_Signal_CalibrateDACVoltage(const CEM102_Device *p_cfg,
                                      int16_t *p_offset_value,
                                      uint32_t channel,
                                      int32_t voltage_target_uv,
                                      int32_t measured_voltage_uv,
                                      CEM102_DAC_CAL_TYPE *p_cal_status)
{
    CEM102_DAC_CAL_TYPE ret_val = *p_cal_status;
    static int16_t prev_offset = 0;
    static int16_t new_offset = 0;
    static int32_t prev_voltage_diff = 0;
    static int32_t voltage_diff = 0;
    int result = ERRNO_NO_ERROR;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
        ret_val = DAC_CAL_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Calculate voltage differences */
        prev_voltage_diff = voltage_diff;
        voltage_diff = voltage_target_uv - measured_voltage_uv;

        if (voltage_diff == 0)
        {
            /* Exact voltage found, no need to continue measuring */
            ret_val = DAC_CAL_COMPLETE;
        }
        else if ((voltage_diff > 0 && prev_voltage_diff < 0) ||
                (voltage_diff < 0 && prev_voltage_diff > 0))
        {
            /* We found the zero crossing point in the error. Need to select the offset to return. */
            ret_val = DAC_CAL_COMPLETE;
        }
        else
        {
            /* A calibration is in progress */
            ret_val = DAC_CAL_IN_PROGRESS;

            /* Retrieve the current offset value, then increment it */
            result |= CEM102_Register_Read(p_cfg, DC_DAC_WE1_CTRL + channel - 1, (uint16_t *)&prev_offset);

            if (result == ERRNO_NO_ERROR)
            {
                /* Set the new offset based on the difference value */
                if (voltage_diff < 0)
                {
                    new_offset = prev_offset - 1;
                }
                else
                {
                    new_offset = prev_offset + 1;
                }

                result |= CEM102_Register_Write(p_cfg, DC_DAC_WE1_CTRL + channel - 1, new_offset);

                if (result != ERRNO_NO_ERROR)
                {
                    ret_val = DAC_CAL_ERROR;
                }
                else
                {
                    /* We are ready to start the measurement, start a single measurement */
                    if (result == ERRNO_NO_ERROR)
                    {
                        result |= CEM102_Signal_StartMeasurement(p_cfg, SINGLE_CMD);
                    }
                }
            }
            else
            {
                ret_val = DAC_CAL_ERROR;
            }
        }

        if (ret_val == DAC_CAL_COMPLETE)
        {
            /* Return the most accurate offset value  */
            if ((voltage_diff == 0) || (abs(voltage_diff) <= abs(prev_voltage_diff)))
            {
                *p_offset_value = new_offset & DC_DAC_WE1_CTRL_DAC_WE1_VALUE_MASK;
            }
            else
            {
                *p_offset_value = prev_offset & DC_DAC_WE1_CTRL_DAC_WE1_VALUE_MASK;
            }

            /* Write the most accurate offset value to the DAC register. */
            result |= CEM102_Register_Write(p_cfg, DC_DAC_WE1_CTRL + channel - 1, (*p_offset_value) | DAC_WE1_ENABLE);

            *p_offset_value = *p_offset_value - CEM102_DAC_MV(voltage_target_uv / CEM102_UV_TO_MV);
        }
    }

    *p_cal_status = ret_val;

    return result;
}



int CEM102_Signal_CalculateChannelGain(const CEM102_Device *p_cfg, uint32_t channel, uint32_t ref_val, uint32_t cal_val)
{
    float otp_gain;
    int result = ERRNO_NO_ERROR;
    float nacc_cal;
    float nacc_otp;
    uint16_t otp_rev;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Set LSAD settings based on device version */
        if (trim_struct_v1 != NULL)
        {
            if (channel == WE1)
            {
                /* Bits 4-15 from OTP word 7 form the channel gain */
                otp_gain = (float)((trim_struct_v1->adc[0] & 0xFFF0) >> 4);
            }
            else if (channel == WE2)
            {
                /* OTP word 9 bits 8-15 contains bits 0-7 of the otp_gain.
                 * OTP word 10 bits 0-3 contains bits 8-11 of the otp_gain. */
                otp_gain = (float)(((trim_struct_v1->adc[2] & 0xFF00) >> 8) |
                           ((trim_struct_v1->adc[3] & 0xF) << 8));
            }
            else
            {
                result = ERRNO_PARAM_ERROR;
            }

            if (otp_gain > CEM102_OTP_GAIN_THRESHOLD)
            {
                otp_gain /= 2.0f;
            }

            /* Set number of accumulations used */
            nacc_cal = CEM102_NACC_REF_R1;
            nacc_otp = CEM102_NACC_OTP_R1;
        }
        else if (trim_struct != NULL)
        {
            otp_gain = trim_struct->adc[0];
            otp_rev = CEM102_OTP_GetVersion();

            if (otp_rev >= CEM102_OTP_VERSION_3)
            {
                otp_gain = (float)(trim_struct->adc[0] + CEM102_GAIN_OFFSET);
                otp_gain = otp_gain / 16.0f;
            }

            /* Set number of accumulations used */
            nacc_cal = CEM102_NACC_REF;
            nacc_otp = CEM102_NACC_OTP;
        }
        else
        {
            /* OTP has not been read from the device */
            result = ERRNO_GENERAL_FAILURE;
        }
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Now we calculate the channel gain given the calibration measurements,
         * otp_gain and configuration settings. */

    	//Get chnnel_gain for accumulation count 8
    	if(stripping_state == STRIPPING_ACCUM_7)
        {
        	channel_gain[channel +1] = ref_val * nacc_otp * otp_gain;
        	channel_gain[channel +1] = channel_gain[channel + 1] / (nacc_cal * cal_val);
        }
    	else
    	{
        channel_gain[channel - 1] = ref_val * nacc_otp * otp_gain;
        channel_gain[channel - 1] = channel_gain[channel - 1] / (nacc_cal * cal_val);
    	}
    }

    return result;
}

int CEM102_Signal_CalculateADCGain(uint32_t channel, uint32_t num_accum)
{
    int result = ERRNO_NO_ERROR;

    if ((channel != WE1) && (channel != WE2) && (channel != RE))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        if (trim_struct_v1 != NULL)
        {
            /* ADC gain is not implemented in OTP rev 1.
             * Use default values */
            adc_gain[channel - 1] /= (num_accum / 2);
        }
        else if (trim_struct != NULL)
        {
            /* Use the value obtained from OTP to calculate
             * the gain value. */
            adc_gain[channel - 1] = CEM102_ADC_GAIN_CALC(trim_struct->adc[channel]) /
                                    (num_accum / 2);
        }
        else
        {
            /* OTP has not been read from the device */
            result = ERRNO_GENERAL_FAILURE;
        }
    }
    return result;
}

int CEM102_Signal_CalculateCurrent(uint32_t channel, int32_t lsad_value, int32_t *p_current)
{
    int error = ERRNO_NO_ERROR;
    float temp = 0.0f;

    if ((channel != WE1) && (channel != WE2))
    {
        /* Invalid channel selected */
        error = ERRNO_PARAM_ERROR;
    }
    else
    {
    	if(stripping_state == STRIPPING_ACCUM_1)
    	{
        temp = ((float)(lsad_value) * channel_gain[channel - 1]);

        *p_current = (int32_t)temp;
    	}
    	else
    	{
		temp = ((float)(lsad_value) * channel_gain[channel + 1]);

		*p_current = (int32_t)temp;
    	}
    }

    return error;
}

int CEM102_Signal_CalculateVoltage(uint32_t channel, int32_t lsad_value, int32_t *p_voltage)
{
    int error = ERRNO_NO_ERROR;
    float temp = 0.0f;

    if ((channel != WE1) && (channel != WE2) && (channel != RE))
    {
        /* Invalid channel selected */
        error = ERRNO_PARAM_ERROR;
    }
    else
    {
        temp = ((float)(lsad_value) * adc_gain[channel - 1]);

        /* Convert volt to microvolts */
        temp *= 1000000;

        *p_voltage = (int32_t)temp;
    }

    return error;
}

int CEM102_Signal_CalculateDACOffset(CEM102_DAC_TYPE offset_type, int16_t *p_offset_value)
{
    int16_t value = 0;
    int result = ERRNO_NO_ERROR;

    switch (offset_type)
    {
        /* RE DAC Offset */
        case RE:

            if (trim_struct_v1 != NULL)
            {
                value = trim_struct_v1->dac[2] & 0x0FFF;
            }

            else if (trim_struct != NULL)
            {
                value = (trim_struct->dac[2] & 0x00FF);
            }

            break;

        /* WE1 DAC Offset */
        case WE1:

            if (trim_struct_v1 != NULL)
            {
                value = (trim_struct_v1->dac[0] & 0xFF00) >> 8;
                value |= (trim_struct_v1->dac[1] & 0xF) << 8;
            }
            else if (trim_struct != NULL)
            {
                value = (trim_struct->dac[0] & 0x00FF);
            }

            break;

        /* WE2 DAC Offset */
        case WE2:

            if (trim_struct_v1 != NULL)
            {
                value = (trim_struct_v1->dac[1] & 0xFFF0) >> 4;
            }

            else if (trim_struct != NULL)
            {
                value = (trim_struct->dac[1] & 0x00FF);
            }

            break;

        default:

            value = CEM102_DAC_MV(CEM102_OTP_DAC_VOLTAGE);
            result = ERRNO_PARAM_ERROR;

            break;
    }

    if (trim_struct_v1 != NULL)
    {
        *p_offset_value = (CEM102_DAC_MV(CEM102_OTP_DAC_VOLTAGE) - value);
    }
    else if (trim_struct != NULL)
    {
        *p_offset_value = value;
    }
    else
    {
        /* OTP has not been read from the device */
        result = ERRNO_GENERAL_FAILURE;
    }

    return result;
}

int CEM102_Signal_GetADCGain(uint32_t channel, float *adc_gain_value)
{
    int result = ERRNO_NO_ERROR;

    if ((channel != WE1) && (channel != WE2) && (channel != RE))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }
    else
    {
        *adc_gain_value = adc_gain[channel-1];
    }

    return result;
}

int CEM102_Signal_CalculateDACGain(uint32_t channel, double *p_dac_gain)
{
    int result = ERRNO_NO_ERROR;
    double dac_gain;

    if ((channel != WE1) && (channel != WE2) && (channel != RE))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        if (trim_struct_v1 != NULL)
        {
            /* OTP v1 has no gain information */
            result = ERRNO_GENERAL_FAILURE;
        }

        if ((trim_struct != NULL) && (result == ERRNO_NO_ERROR))
        {
            dac_gain = (trim_struct->dac[channel -1] & 0xFF00) >> 8;
            *p_dac_gain = ((dac_gain - (CEM102_DAC_GAIN_O1)) /
                           CEM102_DAC_GAIN_G1) * 1000;
        }
    }

    return result;
}

