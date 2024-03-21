/**
 * @file Driver_CEM102.h
 * @brief CEM102 Driver implementation
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

#ifndef DRIVER_CEM102_H_
#define DRIVER_CEM102_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* ----------------------------------------------------------------------------
 * Supporting include files
 * --------------------------------------------------------------------------*/

/* External include files supporting embedded software components on the host
 * system, including the SPI and GPIO hardware abstractions. */
#include <hw.h>

/** @addtogroup CEM102_DRIVERg
 *  @{
 */

/* ----------------------------------------------------------------------------
 * Typedefs
 * --------------------------------------------------------------------------*/
/** Function prototype for callback functions */
typedef void (*CEM102_callback)(uint32_t event);

/** CEM102 Pins and Interrupt Resource Configuration */
typedef struct _CEM102_DEVICE
{
    /* System resources to use */
    unsigned int spi;                   /**< SPI interface instance to use            */
    unsigned int dma[2];                /**< Index of DMA channels to use             */
    unsigned int irq_index;             /**< Index of GPIO IRQ handler to use         */

    /* Pad specifications (if unused, set to -1) */
    int spi_clk;                        /**< Serial clock                             */
    int spi_ctrl;                       /**< Slave select                             */
    int spi_io0;                        /**< Central out, CEM102 in                   */
    int spi_io1;                        /**< Central in, CEM102 out                   */
    int irq;                            /**< IRQ pad from CEM102                      */
    int clk;                            /**< Clock source (optional)                  */
    int pwr_en;                         /**< Power Enable (negative active, optional) */
    int vdda_gpio;                      /**< GPIO connected to VDDA                   */
    int pwr_cfg;                        /**< Power supply configuration               */
    int aout_gpio;                      /**< GPIO to connect AOUT to VCC & power VDDC */
    int io0;                            /**< General I/O connection 0 (optional)      */
    unsigned int io0_type;              /**< General I/O connection 0 type            */
    int io1;                            /**< General I/O connection 1 (optional)      */
    unsigned int io1_type;              /**< General I/O connection 1 type            */

    /* Configurations for interfaces */
    uint32_t gpio_cfg;                  /**< Drive strength, low-pass filter and pull-up config */
    uint32_t spi_prescale;              /**< SPI prescaler                            */

    /* Callback functions */
    CEM102_callback spi_callback;       /**< Callback function for SPI transfers     */
    CEM102_callback gpio_callback;      /**< Callback function for GPIO events       */
} CEM102_Device;

/**
 * @brief SPI status flags.
 */
typedef enum _CEM102_SPI_FLAG
{
    SPI_TRANSFER_IDLE       = 0,        /**< SPI transfers are idle     */
    SPI_TX_ACTIVE           = 1,        /**< SPI transmit is active     */
    SPI_TRANSFER_ACTIVE     = 2,        /**< SPI transfer is active     */
    SPI_TRANSFER_COMPLETE   = 3,        /**< SPI transfer is complete   */
    SPI_TRANSFER_RESET      = 4         /**< CEM102 is in reset         */
} CEM102_SPI_FLAG;

/**
 * @brief DAC Offset Types
 */
typedef enum _CEM102_DAC_TYPE
{
    WE1   = 1,
    WE2   = 2,
    RE    = 3,
	WE1_TOP =4,
	WE1_BOT =5,
} CEM102_DAC_TYPE;

/**
 * @brief DAC Calibration State
 */
typedef enum _CEM102_DAC_CAL_TYPE
{
    DAC_CAL_IDLE        = 0,
    DAC_CAL_IN_PROGRESS = 1,
    DAC_CAL_COMPLETE    = 2,
    DAC_CAL_ERROR       = 3
} CEM102_DAC_CAL_TYPE;

/**
 * @brief CEM102 trim structure v1
 */
typedef struct _CEM102_TRIM_STRUCT_v1
{
    uint16_t device_info[3];            /**< Trim: Device information                       */
    uint16_t trim[2];                   /**< Trim: General device trims                     */
    uint16_t current_bp_delta[2];       /**< Trim: Calibration current for trimming (ver. 1)*/
    uint16_t adc[5];                    /**< Trim: ADC trims (ver. 1)                       */ 
    uint16_t dac[3];                    /**< Trim: DAC trims (ver. 1)                       */
    uint16_t ecc[1];                    /**< Trim: ECC calcuated over the trim structure    */
} CEM102_TrimStruct_v1;

/**
 * @brief CEM102 trim structure v2
 */
typedef struct _CEM102_TRIM_STRUCT
{
    uint16_t device_info[3];            /**< Trim: Device information                       */
    uint16_t trim[2];                   /**< Trim: General device trims                     */
    uint16_t current_bp_delta;          /**< Trim: Calibration current (ver. 2 or higher)   */
    uint16_t adc[3];                    /**< Trim: ADC trims (ver. 2 or higher)             */ 
    uint16_t dac[3];                    /**< Trim: DAC trims (ver. 2 or higher)             */
    uint16_t RESERVED[3];               /**< [Reserved for future use]                      */
    uint16_t ecc[1];                    /**< Trim: ECC calcuated over the trim structure    */
} CEM102_TrimStruct;

/**
* @brief Access structure of the CEM102 Driver.
*/
typedef struct _DRIVER_CEM102_t {
    int     (*Initialize)               (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_Initialize : Initialize the SPI, GPIO interfaces
    int     (*QueueWrite)               (const CEM102_Device *p_cfg, uint8_t addr, unsigned int len);             ///< Pointer to \ref CEM102_Register_QueueWrite : Queue a register write.
    int     (*RegisterBufferWrite)      (const CEM102_Device *p_cfg, uint8_t addr, uint16_t data[],
                                         unsigned int len);                                                       ///< Pointer to \ref CEM102_Register_BufferWrite : Write Data
    int     (*RegisterWrite)            (const CEM102_Device *p_cfg, uint8_t addr, uint16_t data);                ///< Pointer to \ref CEM102_Register_Write : Write a register. Blocks waiting for write to complete.
    int     (*RegisterVerifiedWrite)    (const CEM102_Device *p_cfg, uint8_t addr, uint16_t data);                ///< Pointer to \ref CEM102_Register_VerifiedWrite : Write a register, and verify the written data is correct.
    int     (*QueueRead)                (const CEM102_Device *p_cfg, uint8_t addr, unsigned int len);             ///< Pointer to \ref CEM102_Register_QueueRead : Queue a register read.
    int     (*RegisterBufferRead)       (const CEM102_Device *p_cfg, uint16_t data[], unsigned int len);          ///< Pointer to \ref CEM102_Register_BufferRead : Retrieve data that was previously read using register reads.
    int     (*RegisterRead)             (const CEM102_Device *p_cfg, uint8_t addr, uint16_t *p_data);             ///< Pointer to \ref CEM102_Register_Read : Read a register, Blocks waiting for read to complete.
    void    (*TXHandler)                (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_TX_Handler : Check DMA transfer, manage flags and counters.
    void    (*RXHandler)                (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_RX_Handler : Check DMA transfer, manage flags and counters.
    void    (*GPIOHandler)              (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_GPIO_Handler : Called by hardware GPIO handler when an interrupt is received.
    int     (*OTPRead)                  (const CEM102_Device *p_cfg, CEM102_TrimStruct *p_trim_struct,
                                         uint32_t cem102_sysclk_freq, uint32_t timeout);                          ///< Pointer to \ref CEM102_OTP_Read : Read OTP and set ANA_TRIMx Registers
    int     (*PowerReset)               (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_Power_Reset : Reset the CEM102 device's analog components.
    int     (*PowerInit)                (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_Power_Init : Initialize the power for the CEM102 device.
    int     (*VDDADischarge)            (const CEM102_Device *p_cfg, unsigned int time);                          ///< Pointer to \ref CEM102_Power_VDDADischarge : Discharge the CEM102 VDDA for the specified amount of time.
    int     (*DeviceTrim)               (const CEM102_Device *p_cfg, CEM102_TrimStruct *p_trim_struct);           ///< Pointer to \ref CEM102_Power_DeviceTrim : Trim the CEM102 device with data loaded from OTP
                                                                                                                  ///< or otherwise pre-calculated.
    void    (*VDDCCharge)               (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_Power_VDDCCharge : Set VCC on AOUT and AOUT on the specified GPIO.
    void    (*SARInit)                  (const CEM102_Device *p_cfg, uint32_t sar_cfg, uint32_t sar_ctrl,
                                         uint32_t clk_cfg);                                                       ///< Pointer to \ref CEM102_SAR_Init : Initialize SAR ADC
    int     (*SARStartConversion)       (uint32_t conv_mode);                                                     ///< Pointer to \ref CEM102_SAR_StartConversion : Start SAR ADC Conversion
    int     (*VBATOK)                   (uint32_t trim_error, uint32_t threshold, uint16_t *p_data);              ///< Pointer to \ref CEM102_VBAT_OK : Measure Vbat and flag ok if CEM102 Vbat is above threshold
    int     (*VCCOK)                    (uint32_t trim_error, uint32_t threshold, uint16_t *p_data);              ///< Pointer to \ref CEM102_VCC_OK : Measure VCC and flag OK if CEM102 VCC is above the required threshold.
    int     (*p_voltageOK)              (uint32_t trim_error, uint32_t threshold, uint16_t *p_data,
                                         uint32_t rail);                                                          ///< Pointer to \ref CEM102_VOLTAGE_OK : Measure a rail and flag OK if the voltage is above the required threshold.
    void    (*VBAT_VDDAMonitorStart)    (const CEM102_Device *p_cfg, uint8_t measurement_en,
                                         uint32_t VBAT_margin);                                                   ///< Pointer to \ref CEM102_VBAT_VDDA_MonitorStart : Monitor the VBAT and VDDA differential voltage.
    int     (*CalibrateVCC)             (const CEM102_Device *p_cfg, uint8_t vcc_min_trim,
                                         const uint32_t vcc_target);                                              ///< Pointer to \ref CEM102_Calibrate_VCC : Calibrate VCC to the target voltage
    void    (*CalculateVBAT)            (uint16_t *p_voltage);                                                    ///< Pointer to \ref CEM102_Calculate_VBAT : Calculate CEM102 VBAT and return the result.
    int     (*IRQConfig)                (const CEM102_Device *p_cfg, uint16_t irq_cfg);                           ///< Pointer to \ref CEM102_System_IRQConfig : Configure the IRQ behavior.
    int     (*SystemCommand)            (const CEM102_Device *p_cfg, uint16_t cmd);                               ///< Pointer to \ref CEM102_System_Command : Issue the specified system command configuration.
    int     (*SystemStatusQueue)        (const CEM102_Device *p_cfg);                                             ///< Pointer to \ref CEM102_System_Status_Queue : Queue Status Registers (SYS_BUFFER_VIOL_CNT, SYS_BUFFER_VIOL_STATUS, & SYS_STATUS)
    int     (*SystemStatusProcess)      (uint16_t *p_data, uint16_t *p_violation_cnt);                            ///< Pointer to \ref CEM102_System_Status_Process : Process Status Registers results
    int     (*ClockConfig)              (const CEM102_Device *p_cfg, uint16_t prescale, uint16_t delay);          ///< Pointer to \ref CEM102_Signal_ClockConfig : Configure the LSAD clocks.
    int     (*TIAConfig)                (const CEM102_Device *p_cfg, int tia, uint16_t enable, uint16_t tia_cfg); ///< Pointer to \ref CEM102_Signal_TIAConfig : Configure a trans-impedance amplifier (TIA).
    int     (*SwitchConfig)             (const CEM102_Device *p_cfg, int channel, uint16_t channel_cfg);          ///< Pointer to \ref CEM102_Signal_SwitchConfig : Configure the switches for a channel.
    int     (*RE_ATBusSwitchConfig)     (const CEM102_Device *p_cfg, int channel, uint16_t config);               ///< Pointer to \ref CEM102_Signal_RE_ATBus_Config : Configure the switches for RE or ATBus.
    int     (*BufferConfig)             (const CEM102_Device *p_cfg, int channel, uint16_t channel_cfg,
                                         uint16_t buf_cfg);                                                       ///< Pointer to \ref CEM102_Signal_BufferConfig : Configure the buffer and low pass filtering for a channel.
    int     (*WEDACConfig)              (const CEM102_Device *p_cfg, int dac, uint16_t dac_cfg,
                                         uint16_t interrupt_cfg);                                                 ///< Pointer to \ref CEM102_Signal_WEDACConfig : Configure a working electrode DAC.
    int     (*REDACConfig)              (const CEM102_Device *p_cfg, uint16_t dac_cfg,
                                         uint16_t sw_cfg, uint16_t buf_cfg);                                      ///< Pointer to \ref CEM102_Signal_REDACConfig : Configure the reference electrode DAC.
    int     (*ADCConfig)                (const CEM102_Device *p_cfg, int channel, uint16_t lsad_cfg,
                                         uint16_t thresh_cfg, uint16_t buf_cfg, uint16_t accum_cfg);              ///< Pointer to \ref CEM102_Signal_ADCConfig : Configure the buffer and low pass filtering for a channel.
    int     (*DirectADCConfig)          (const CEM102_Device *p_cfg, uint32_t channel);                           ///< Pointer to \ref CEM102_Signal_DirectADCConfig : Configure CEM102 to measure voltage on the ADC rather than current.
    int     (*StartMeasurement)         (const CEM102_Device *p_cfg, uint16_t cmd);                               ///< Pointer to \ref CEM102_Signal_StartMeasurement : Start a measurement by changing the command state.
    void    (*ProcessADCSingle)         (const CEM102_Device *p_cfg, uint16_t data, uint16_t lsad_cfg,
                                         int sample_cnt, int shift, float *p_processed_data);                     ///< Pointer to \ref CEM102_Signal_ProcessADCSingle : Process an accumulated data sample.
    void    (*ProcessADCDouble)         (const CEM102_Device *p_cfg, uint32_t data, uint16_t lsad_cfg,
                                         int sample_cnt, int shift, float *p_processed_data);                     ///< Pointer to \ref CEM102_Signal_ProcessADCDouble : Process an accumulated data sample with double precision.
    int     (*MeasureReference)         (const CEM102_Device *p_cfg, uint32_t channel);                           ///< Pointer to \ref CEM102_Signal_MeasureReference : Run a reference current measurement.
    int     (*CalibrateChannel)         (const CEM102_Device *p_cfg, uint32_t channel);                           ///< Pointer to \ref CEM102_Signal_CalibrateChannel : Run a calibration measurement.
    int     (*CalibrateDACVoltage)      (const CEM102_Device *p_cfg, int16_t *p_offset_value, uint32_t electrode,
                                         int32_t voltage_target_uv, int32_t measured_voltage_uv,
                                         CEM102_DAC_CAL_TYPE *p_cal_status);                                      ///< Pointer to \ref CEM102_Signal_CalibrateDACVoltage : Calibrate the chosen DAC to a desired target value.
    int     (*CalculateChannelGain)     (const CEM102_Device *p_cfg, uint32_t channel,
                                         uint32_t ref_val, uint32_t cal_val);                                     ///< Pointer to \ref CEM102_Signal_CalculateChannelGain : Calculate a channel gain.
    int     (*CalculateADCGain)         (uint32_t channel, uint32_t num_accum);
    int     (*CalculateCurrent)         (uint32_t channel, int32_t lsad_value, int32_t *p_current);               ///< Pointer to \ref CEM102_Signal_CalculateCurrent : Calculate the current measured using the channel gain.
    int     (*CalculateVoltage)         (uint32_t channel, int32_t lsad_value, int32_t *p_voltage);               ///< Pointer to \ref CEM102_Signal_CalculateVoltage : Calculate the voltage measured using the adc gain.
    int     (*CalculateDACOffset)       (CEM102_DAC_TYPE offset_type, int16_t *p_offset_value);                   ///< Pointer to \ref CEM102_Signal_CalculateDACOffset : Calculate DAC Offset values based on OTP version.
    int     (*GetADCGain)               (uint32_t channel, float *adc_gain_value);
    int     (*CalculateDACGain)         (uint32_t channel, double *p_dac_gain);                                   ///< Pointer to \ref CEM102_Signal_CalculateDACGain : Calculate DAC Gain
    int     (*CalculateDACAmplitude)    (const CEM102_Device *p_cfg,
                                         uint32_t dac, uint16_t mv,
                                         uint16_t *p_dac);                                                        ///< Pointer to \ref CalculateDACAmplitude : Calculate DAC value using OTP.
    int     (*CalculateDACAmplitudeADC) (const CEM102_Device *p_cfg,
                                         uint32_t dac, uint16_t mv,
                                         uint16_t *p_dac);                                                        ///< Pointer to \ref CalculateDACAmplitudeADC : Calculate DAC value using ADC.
} const DRIVER_CEM102_t;

/** @} */ /* End of the CEM102_DRIVERg group */

#ifdef  __cplusplus
}
#endif

#endif    /* CEM102_DRIVER_H */
