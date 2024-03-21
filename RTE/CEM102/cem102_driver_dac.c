/**
 * @file cem102_driver_dac.c
 * @brief CEM102 DAC/ADC related functions.
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

#include <cem102_driver.h>

#define CEM102_DAC_VALUE_FOR_56MV     (100)
#define CEM102_DAC_ERROR_THRESHOLD    (2500)  /* microvolts */

volatile CEM102_MEASUREMENT_STATUS_FLAG cem102_measure_status;
uint16_t cem102_measure_buf[CEM102_MEASUREMENT_LENGTH];

/* At present, only DAC API uses this callback. */
void CEM102_DAC_SPICallback(const CEM102_Device *p_cfg)
{
    /* If we were reading results, copy the SPI data to the measurement buffer
     * and then mark them as ready for processing
     */
    if (cem102_measure_status == CEM102_SM_READING_RESULTS)
    {
        CEM102_Register_BufferRead(p_cfg, cem102_measure_buf,
                                   CEM102_MEASUREMENT_LENGTH);
        cem102_measure_status = CEM102_SM_RESULTS_READY;
    }
}

static int Read_ADC_Voltage(const CEM102_Device *p_cfg, int chan,
                            int32_t *p_voltage)
{
    int index = chan - 1;
    uint16_t ch_clr[3] = { CH1_COMPLETION_CLR_CMD,
                           CH2_COMPLETION_CLR_CMD,
                           CH1_COMPLETION_CLR_CMD
                         };
    uint16_t completion_flag[3] = { CH1_COMPLETION,
                                    CH2_COMPLETION,
                                    CH1_COMPLETION
                                  };

    /* Ensure all errors and system status is cleared before starting the
     * first measurement. */
    int result = CEM102_System_Command(p_cfg,
                                       DIGITAL_RESET_CLR_CMD |
                                       VDDA_ERR_CLR_CMD | DC_DAC_LOAD_CLR_CMD |
                                       ch_clr[index] | BUFFER_RESET_CMD |
                                       IDLE_CMD);

    /* Give time before next SPI transaction, after IDLE_CMD */
    Sys_Delay(CEM102_SYS_DELAY_20MS);

    result |= CEM102_Signal_StartMeasurement(p_cfg, SINGLE_CMD);
    cem102_measure_status = CEM102_SM_MEASURING;

    /* Wait for the reference measurement to finish, the system can
     * perform other activities during this time.
     */
    uint16_t status = 0;
    Sys_Delay(CEM102_SYS_DELAY_50MS);
    do
    {
        result |= CEM102_Register_Read(p_cfg, SYS_STATUS, &status);
        if ((status & (CH1_COMPLETION|CH2_COMPLETION)) == 0)
        {
            __WFI();
        }
        SYS_WATCHDOG_REFRESH();
    } while ((status & completion_flag[index]) == 0);

    /* After the measurement is finished, queue a read of the measurement
     * data */
    cem102_measure_status = CEM102_SM_READING_RESULTS;
    result |= CEM102_Register_QueueRead(p_cfg, SYS_BUFFER_WORD0,
                                        CEM102_MEASUREMENT_LENGTH);

    int lower_half = (index * 2) + 1;
    int upper_half = index * 2;
    if (chan == RE)
    {
        lower_half = 1;
        upper_half = 0;
    }

    /* Wait for the reference measurement to finish */
    while (cem102_measure_status != CEM102_SM_RESULTS_READY)
    {
        __WFI();
    }

    int32_t dac_measurement = cem102_measure_buf[lower_half] +
                             (cem102_measure_buf[upper_half] << 16);

    /* AT1 & AT2 are configured to be inverse to ADC output */
    if (chan == WE1)
    {
        dac_measurement *= (-1);
    }

    /* Most API, chan is one of WE1, WE2, RE. Some API expects WE1 for RE */
    int new_chan = (chan == RE) ? WE1 : chan;

    /* Calculate the actual voltage measured on the working electrode of
     * the given channel, given the raw uncompensated measurement.
     */
    result |= CEM102_Signal_CalculateVoltage(new_chan, dac_measurement, p_voltage);
    return result;
}

static int Config_ADC_PostDACCalibration(const CEM102_Device *p_cfg, int chan)
{
    int result = ERRNO_NO_ERROR;
    int index = chan - 1;

    /* Most API, chan is one of WE1, WE2, RE. Some API expects WE1 for RE */
    int new_chan = (chan == RE) ? WE1 : chan;

    if (chan == RE)
    {
        result |= CEM102_Signal_REDACConfig(p_cfg, 0,
                                            (RE_SR1_DISCONNECT | RE_SR2_DISCONNECT),
                                            (RE_BUF_DISABLE | RE_HP_MODE_DISABLE));
    }
    uint16_t sw_disc[3] = { CH1_SC112_DISCONNECT | CH1_SC113_DISCONNECT |
                            CH1_SC115_DISCONNECT | CH1_SC116_DISCONNECT |
                            CH1_SC119_SC120_DISCONNECT,
                            CH2_SC212_DISCONNECT | CH2_SC213_DISCONNECT |
                            CH2_SC215_DISCONNECT | CH2_SC216_DISCONNECT |
                            CH2_SC219_SC220_DISCONNECT,
                            CH1_SC111_DISCONNECT | CH1_SC112_DISCONNECT |
                            CH1_SC113_DISCONNECT | CH1_SC114_DISCONNECT |
                            CH1_SC115_DISCONNECT | CH1_SC116_DISCONNECT |
                            CH1_SC117_SC118_DISCONNECT |
                            CH1_SC119_SC120_DISCONNECT |
                            CH1_SC109_SC110_DISCONNECT
                          };
    result |= CEM102_Signal_SwitchConfig(p_cfg, new_chan, sw_disc[index]);
    uint16_t at_disc[3] = { ATBUS_ST12_DISCONNECT | ATBUS_ST14_DISCONNECT |
                            ATBUS_ST17_DISCONNECT | ATBUS_ST21_DISCONNECT,
                            ATBUS_ST16_DISCONNECT | ATBUS_ST17_DISCONNECT |
                            ATBUS_ST21_DISCONNECT | ATBUS_ST22_DISCONNECT,
                            ATBUS_ST16_DISCONNECT | ATBUS_ST17_DISCONNECT |
                            ATBUS_ST21_DISCONNECT | ATBUS_ST22_DISCONNECT
                          };
    result |= CEM102_Signal_RE_ATBus_Config(p_cfg, RE, at_disc[index]);
    return result;
}

static int Config_ADC_PreDACCalibration(const CEM102_Device *p_cfg, int chan,
                                        uint16_t *p_dac)
{
    int index = chan - 1;

    /* Most API, chan is one of WE1, WE2, RE. Some API expects WE1 for RE */
    int new_chan = (chan == RE) ? WE1 : chan;

    /* Disconnect all switch configurations (SA1 to SA14) in analog test bus. */
    int result = CEM102_Register_Write(p_cfg, ANA_SW_CFG2, ATBUS_SA1_DISCONNECT);

    /* Calculate the gain of the ADC for the given channel. Value from the
     * sample (CEM102_ACCUM_SAMPLE_CNT_2) as programmed in CHCFG_CHx_ACCUM
     * register is one less than the value of the sample count.
     */
    result |= CEM102_Signal_CalculateADCGain(new_chan,
                                             CEM102_ACCUM_SAMPLE_CNT_2 + 1);
    if (chan == RE)
    {
        /* DAC Calibration Configuration:
         * RE : SC119 & SC120=OFF, RE =AT1, AT2=VSSA, use LSAD1
         */
        result |= CEM102_Signal_REDACConfig(p_cfg, 0,
                                            (RE_SR1_CONNECT | RE_SR2_CONNECT),
                                            (RE_BUF_ENABLE | RE_HP_MODE_DISABLE));
    }
    else
    {
        uint16_t tia_enable[2] = { CH1_TIA_ENABLE, CH2_TIA_ENABLE };
        uint16_t tia_cfg[2] = { CH1_TIA_INT_FB_ENABLE | CH1_TIA_FB_RES_1M,
                                CH2_TIA_INT_FB_ENABLE | CH2_TIA_FB_RES_1M
                              };
        result |= CEM102_Signal_TIAConfig(p_cfg, chan, tia_enable[index],
                                          tia_cfg[index]);

    }
    uint16_t ch_cfg[3] = { CH1_BUF_IB0 | CH1_LPF_BW_CFG_ENABLE,
                           CH2_BUF_IB0 | CH2_LPF_BW_CFG_ENABLE,
                           CH1_BUF_IB0 | CH1_LPF_BW_CFG_ENABLE
                         };
    uint16_t buf_cfg[3] = { CH1_BUF_ENABLE | CH1_BUF_BYPASS_ENABLE,
                            CH2_BUF_ENABLE | CH2_BUF_BYPASS_ENABLE,
                            CH1_BUF_ENABLE | CH1_BUF_BYPASS_ENABLE
                          };
    result |= CEM102_Signal_BufferConfig(p_cfg, new_chan, ch_cfg[index],
                                         buf_cfg[index]);

    /* DAC Calibration Configuration:
     * WE1: SC119 & SC120=OFF, WE1=AT2, AT1=VSSA
     * WE2: SC219 & SC220=OFF, WE2=AT1, AT2=VSSA
     */
    uint16_t sw_conn[3] = { CH1_SC111_DISCONNECT | CH1_SC113_CONNECT |
                            CH1_SC115_CONNECT | CH1_SC116_CONNECT |
                            CH1_SC119_SC120_DISCONNECT,
                            CH2_SC212_DISCONNECT | CH2_SC213_CONNECT |
                            CH2_SC215_CONNECT | CH2_SC216_CONNECT |
                            CH2_SC219_SC220_DISCONNECT,
                            CH1_SC111_DISCONNECT | CH1_SC112_DISCONNECT |
                            CH1_SC113_DISCONNECT | CH1_SC114_DISCONNECT |
                            CH1_SC115_CONNECT | CH1_SC116_CONNECT |
                            CH1_SC117_SC118_DISCONNECT |
                            CH1_SC119_SC120_DISCONNECT
                          };
    result |= CEM102_Signal_SwitchConfig(p_cfg, new_chan, sw_conn[index]);
    uint16_t at_conn[3] = { ATBUS_ST14_CONNECT | ATBUS_ST12_CONNECT |
                            ATBUS_ST21_DISCONNECT | ATBUS_ST17_DISCONNECT,
                            ATBUS_ST16_CONNECT | ATBUS_ST22_CONNECT |
                            ATBUS_ST21_DISCONNECT | ATBUS_ST17_DISCONNECT,
                            ATBUS_ST16_CONNECT | ATBUS_ST17_DISCONNECT |
                            ATBUS_ST21_DISCONNECT | ATBUS_ST22_CONNECT
                          };
    result |= CEM102_Signal_RE_ATBus_Config(p_cfg, RE, at_conn[index]);
    result |= CEM102_Signal_ClockConfig(p_cfg, LSAD_PRESCALE1,
                                        CEM102_LSAD_START_DELAY_6MS);
    uint16_t lsad_cfg[3] = { LSAD1_GAIN_UNITY | LSAD1_SYS_CHOP_ENABLE |
                             LSAD1_ORSD64_RSD14 | LSAD1_ITRIM_1p00 |
                             LSAD1_BUF_CHOP_CLK_DIV16 | LSAD1_ADC_CHOP_CLK_DIV16,
                             LSAD2_GAIN_UNITY | LSAD2_SYS_CHOP_ENABLE |
                             LSAD2_ORSD64_RSD14 | LSAD2_ITRIM_1p00 |
                             LSAD2_BUF_CHOP_CLK_DIV16 | LSAD2_ADC_CHOP_CLK_DIV16,
                             LSAD1_GAIN_UNITY | LSAD1_SYS_CHOP_ENABLE |
                             LSAD1_ORSD64_RSD14 | LSAD1_ITRIM_1p00 |
                             LSAD1_BUF_CHOP_CLK_DIV16 | LSAD1_ADC_CHOP_CLK_DIV16,
                           };
    uint16_t thresh_cfg[3] = { CH1_VIOLATION_CHECK_DISABLE,
                               CH2_VIOLATION_CHECK_DISABLE,
                               CH1_VIOLATION_CHECK_DISABLE
                             };
    uint16_t buffer_cfg[3] = { CH1_DOUBLE | CH1_VIOLATION_ABS_DISABLE,
                               CH2_DOUBLE | CH2_VIOLATION_ABS_DISABLE,
                               CH1_DOUBLE | CH1_VIOLATION_ABS_DISABLE
                             };
    uint16_t accum_cfg[3] = { (CHCFG_CH1_ACCUM_CH1_ACCUM_SAMPLE_CNT_MASK &
                              CEM102_ACCUM_SAMPLE_CNT_2) |
                              CH1_MEASUREMENT_ENABLE,
                              (CHCFG_CH2_ACCUM_CH2_ACCUM_SAMPLE_CNT_MASK &
                              CEM102_ACCUM_SAMPLE_CNT_2) |
                              CH2_MEASUREMENT_ENABLE,
                              (CHCFG_CH1_ACCUM_CH1_ACCUM_SAMPLE_CNT_MASK &
                              CEM102_ACCUM_SAMPLE_CNT_2) |
                              CH1_MEASUREMENT_ENABLE
                            };
    result |= CEM102_Signal_ADCConfig(p_cfg, new_chan, lsad_cfg[index],
                                      thresh_cfg[index], buffer_cfg[index],
                                      accum_cfg[index]);

    /* Configure the given channel's switches and buffers for direct ADC
     * measurements
     */
    result |= CEM102_Signal_DirectADCConfig(p_cfg, new_chan);
    return result;
}

static uint16_t DAC_Get_NominalValue(int32_t target_voltage_mv)
{
    double mvolts_f = target_voltage_mv;

    uint16_t dac_value = (uint16_t)((mvolts_f * 4096.0f) / 2300.0f);
    return dac_value;
}

static int32_t DAC_Get_ErrorValue(int32_t target, int32_t measured)
{
    int32_t error;

    if (target < measured)
    {
        error = measured - target;
    }
    else
    {
        error = target - measured;
    }
    return error;
}

int CEM102_DAC_CalibrateWithADC(const CEM102_Device *p_cfg, uint32_t chan,
                                uint16_t mvolts, uint16_t *p_dac)
{
    int result = ERRNO_NO_ERROR;
    uint16_t min = 1;
    int32_t error = 0;
    uint16_t expected = 0;
    uint16_t current_setting = 0;
    uint16_t previous_setting = 0;
    int32_t current_voltage = 0;

    /* Mask is common for both channels. */
    uint16_t max = DC_DAC_WE1_CTRL_DAC_WE1_VALUE_MASK;
    uint32_t target = (uint32_t)mvolts * CEM102_VOLTAGE_MV_TO_UV;
    int index = chan - 1;
    uint16_t ctrl_mask[3] = { DC_DAC_WE1_CTRL_DAC_WE1_VALUE_MASK,
                              DC_DAC_WE2_CTRL_DAC_WE2_VALUE_MASK,
                              DC_DAC_RE_CTRL_DAC_RE_VALUE_MASK
                            };
    uint16_t we_en[3] = { DAC_WE1_ENABLE, DAC_WE2_ENABLE, DAC_RE_ENABLE };

    if ((chan != WE1) && (chan != WE2) && (chan != RE))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    if (result == ERRNO_NO_ERROR)
    {
        /* Configure the ADC to prepare for the DAC calibration */
        result = Config_ADC_PreDACCalibration(p_cfg, chan, p_dac);

        /* Time for ADC to settle down with new settings */
        Sys_Delay(CEM102_SYS_DELAY_50MS);

        expected = DAC_Get_NominalValue(mvolts);

        /* Set the range to search through. Assumption is that 56 mV on either
         * side of the theoretical value should be OK.
         *
         * Note: DAC has 0.56 mV resolution per step.
         */
        if (expected > CEM102_DAC_VALUE_FOR_56MV)
        {
            min = expected - CEM102_DAC_VALUE_FOR_56MV;
        }
        if (expected < (max - CEM102_DAC_VALUE_FOR_56MV))
        {
            max = expected + CEM102_DAC_VALUE_FOR_56MV;
        }
        previous_setting = min;

        do
        {
            current_setting = ((max - min) / 2) + min;
            if ((current_setting == (max - 1)) &&
                (current_setting == previous_setting))
            {
                current_setting = max;
            }

            if (chan == RE)
            {
                result |= CEM102_Signal_REDACConfig(p_cfg, ((ctrl_mask[index] &
                                                    current_setting) | we_en[index]),
                                                    (RE_SR1_CONNECT | RE_SR2_CONNECT),
                                                    (RE_BUF_ENABLE | RE_HP_MODE_DISABLE));
            }
            else
            {
                result |= CEM102_Signal_WEDACConfig(p_cfg, chan,
                                                    ((ctrl_mask[index] &
                                                    current_setting) | we_en[index]),
                                                    DC_DAC_LOAD_IRQ_DISABLE);
            }
            result |= Read_ADC_Voltage(p_cfg, chan, &current_voltage);

            if (target == current_voltage)
            {
                /* Right setting is found */
                error = 0;
                break;
            }
            else
            {
                error = DAC_Get_ErrorValue(target, current_voltage);
            }

            if (previous_setting == current_setting)
            {
                /* Calibration not successful */
                break;
            }
            if (current_voltage < target)
            {
                min = current_setting;
            }
            else if (current_voltage > target)
            {
                max = current_setting;
            }
            previous_setting = current_setting;

            if ((max == min) ||
                ((max > min) && ((max - min) < 2)) ||
                ((max < min) && ((min - max) < 2)))
            {
                break;
            }
        } while (result == ERRNO_NO_ERROR);
    }

    if (result == ERRNO_NO_ERROR)
    {
        int32_t new_voltage = 0;
        int32_t new_error = 0;
        uint16_t new_setting = 0;

        if ((target != current_voltage) && (max != min))
        {
            if (max == current_setting)
            {
                current_setting = new_setting = min;
            }
            else
            {
                current_setting = new_setting = max;
            }

            if (chan == RE)
            {
                result |= CEM102_Signal_REDACConfig(p_cfg, ((ctrl_mask[index] &
                                                    new_setting) | we_en[index]),
                                                    (RE_SR1_CONNECT | RE_SR2_CONNECT),
                                                    (RE_BUF_ENABLE | RE_HP_MODE_DISABLE));
            }
            else
            {
                result |= CEM102_Signal_WEDACConfig(p_cfg, chan,
                                                    ((ctrl_mask[index] &
                                                    new_setting) | we_en[index]),
                                                    DC_DAC_LOAD_IRQ_DISABLE);
            }
            result |= Read_ADC_Voltage(p_cfg, chan, &new_voltage);
            new_error = DAC_Get_ErrorValue(target, new_voltage);
            if (new_error < error)
            {
                error = new_error;
                current_setting = new_setting;
                current_voltage = new_voltage;
            }
        }
    }
    *p_dac = current_setting;

    /* Post calibration clean-up. Turn the switches that are used, to off position */
    result |= Config_ADC_PostDACCalibration(p_cfg, chan);

    /* Due to some reason, if error deviates more than certain threshold, 2.5 mV,
     * an error code is returned to indicate this anomaly.
     */
    if (error > CEM102_DAC_ERROR_THRESHOLD)
    {
        result = ERRNO_CALIBRATION_ERROR;
    }
    return result;
}

int CEM102_DAC_Calibrate(const CEM102_Device *p_cfg, uint32_t dac,
                         uint16_t mvolts, uint16_t *p_setting)
{
    int result = ERRNO_NO_ERROR;

    if ((dac != WE1) && (dac != WE2) && (dac != RE))
    {
        /* Invalid channel selected */
        result = ERRNO_PARAM_ERROR;
    }

    uint16_t otp_ver = CEM102_OTP_GetVersion();
    if ((otp_ver == CEM102_OTP_VERSION_3) || (otp_ver == CEM102_OTP_VERSION_1))
    {
        result = ERRNO_GENERAL_FAILURE;
    }

    if (result == ERRNO_NO_ERROR)
    {
        int16_t we_offset = 0;
        uint16_t dac_setting = 0;

        result |= CEM102_Signal_CalculateDACOffset(dac, &we_offset);
        if (trim_struct_v1 != NULL)
        {
            dac_setting = (CEM102_DAC_MV(mvolts) - we_offset);
        }
        else if (trim_struct != NULL)
        {
            double dac_gain = 0;

            result |= CEM102_Signal_CalculateDACGain(dac, &dac_gain);
            if (result == ERRNO_NO_ERROR)
            {
                double dac_offset = (((float)we_offset - CEM102_DAC_OFFSET_O2) /
                                     CEM102_DAC_OFFSET_G2) * 1000;
                dac_setting = (uint16_t)(((double)mvolts - dac_offset) /
                               dac_gain);
            }
        }
        *p_setting = dac_setting;
    }
    return result;
}

