/**
 * @file  app_cem102.h
 * @brief CEM102 low power application initialization header file
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

#ifndef APP_CEM102_H_
#define APP_CEM102_H_

#include <hw.h>
/* Include the CEM102 support library */
#include "cem102_driver.h"

/* ----------------------------------------------------------------------------
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif    /* ifdef __cplusplus */
/* ----------------------------------------------------------------------------
 * Defines
 * --------------------------------------------------------------------------*/
/* CEM102 GPIO definitions */

/* CEM102 SPI CLK GPIO */
#define CEM102_SPI_CLK_GPIO             (11)

/* CEM102 SPI CLK GPIO */
#define CEM102_SPI_CTRL_GPIO            (12)

/* CEM102 SPI CLK GPIO */
#define CEM102_SPI_IO0_GPIO             (6)

/* CEM102 SPI CLK GPIO */
#define CEM102_SPI_IO1_GPIO             (4)

/* CEM102 IRQ GPIO */
#define CEM102_IRQ_GPIO                 (1)

/* CEM102 NRESET GPIO */
#define CEM102_NRESET_GPIO              (14)

/* CEM102 VDDA DUT GPIO */
#define CEM102_VDDA_DUT_GPIO            (13)

/* CEM102 AOUT GPIO */
#define CEM102_AOUT_GPIO                (5)

/**
 * @brief SPI status flags.
 */
typedef enum _MEASUREMENT_STATUS_FLAG
{
    MEASURE_IDLE            = 0,
    MEASURE_INITIALIZED     = 1,
    MEASURE_MEASURING       = 2,
    MEASURE_READING_RESULTS = 3,
    MEASURE_RESULTS_READY   = 4
} MEASUREMENT_STATUS_FLAG;

typedef enum _Calib_State_t
{
    CALIB_INITIALIZED,
    CALIB_REF_MEASURING,
    CALIB_REF_READING,
    CALIB_CH1_MEASURING,
    CALIB_CH1_READING,
    CALIB_CH2_MEASURING,
    CALIB_CH2_READING,
    CALIB_DONE
} Calib_state_t;


enum {
	STRIPPING_ACCUM_1=0,
	STRIPPING_ACCUM_7,
	STRIPPING_ACCUM_DONE,
};

enum {
	STRIPPING_DAC_SET=0,
	STRIPPING_DAC_DONE
};

/* Voltage in millivolts to signal that the VCC level is high enough */
#define CEM102_VCC_OK_THRESH                (1180)

/* Voltage in millivolts to signal that VBAT level is high enough */
#define CEM102_VBAT_OK_THRESH               (2200)

/* Length of array containing CEM102 measurement data.
 * Space for two register reads per channel, and two channels, WE1 and WE2.
 */
#define MEASUREMENT_LENGTH                  (2 * 2)
#define APP_REFERENCE_LENGTH                (2)

/* VCC Typical when .pwr_cfg = LOW_VOLTAGE */
#define VCC_TYPICAL                         (1250)

/* Length of time in ms to discharge VDDA to its trimmed value
 * of 2.3 V */
#define VDDA_DISCHARGE_TIME_MS              (200)

/* Minimum VBAT to VDDA threshold */
#define SENSOR_THRESHOLD_MIN_THRESHOLD_MIN  (75)

/* LSAD Start Delay definitions in millisecond. Actual delay is two times the
 * programmed value.
 */
#define LSAD_START_DELAY_50MS               (25)
#define LSAD_START_DELAY_6MS                (3)

/* Delay using Sys_Delay call */
#define APP_SYS_DELAY_20MS                  (SystemCoreClock / 50)

/* Wait time of 50 msec delay when ADC is configured */
#define APP_LSAD_CALIB_DELAY                (SystemCoreClock / 20)

/* Number of sample accumulation configuration of 8 */
#define ACCUM_SAMPLE_CNT_8                  (7)

#define ACCUM_SAMPLE_CNT_1                  (1)

/* VDDA_ERR alarm code for the phone application */
#define VDDA_ERR_ALARM                      (0xFFFF)

/* Delay time in seconds before going into DEEP SLEEP when VDDA_ERR occur */
#define VDDA_ERR_DEEP_SLEEP_DELAY           (10)

/* ---------------------------------------------------------------------------
* Function prototype definitions
* --------------------------------------------------------------------------*/
/**
 * @brief       GPIO Interrupt Handler 1
 */
extern void GPIO1_IRQHandler(void);

/**
 * @brief       DMA Channel 0 Interrupt Handler
 */
extern void DMA0_IRQHandler(void);

/**
 * @brief       DMA Channel 1 Interrupt Handler
 */
extern void DMA1_IRQHandler(void);

/**
 * @brief       Call back to handle GPIO events.
 * @param[in]   event   Event that triggered the call back.
 */
extern void GPIO_CallBack(uint32_t event);

/**
 * @brief       Call back to handle SPI events.
 * @param[in]   event   Event that triggered the call back.
 */
extern void SPI_CallBack(uint32_t event);

/*
 * @brief       Calibrate the CEM102.
 * @param[in]   cem102   Pointer to the active CEM102 driver instance.
 */
int CEM102_Calibrate(DRIVER_CEM102_t * cem102);

/**
 * @brief       Calls the sequence of functions required for initializing the CEM102
 * @param[in]   cem102   Pointer to the active CEM102 driver instance.
 */
void CEM102_Startup(DRIVER_CEM102_t * cem102);

/**
 * @brief       Checks the CEM102 status and reads CEM102 data, as well as start
 *              new measurements
 * @param[in]   cem102       Pointer to the active CEM102 driver instance.
 */
void CEM102_Operation(DRIVER_CEM102_t * cem102);

/* ----------------------------------------------------------------------------
 * Global variables and types
 * --------------------------------------------------------------------------*/
extern uint16_t measure_buf[];
extern const CEM102_Device * cem102_dut;
extern volatile MEASUREMENT_STATUS_FLAG measure_status;
extern DRIVER_CEM102_t Driver_CEM102;
extern DRIVER_CEM102_t * cem102;
extern volatile uint16_t cem102_vbat;
extern volatile int32_t WE1_current;
extern volatile int32_t WE2_current;
extern uint32_t trim_error;
extern volatile Calib_state_t calib_state;



extern void CEM102_Stripping(DRIVER_CEM102_t * cem102);

/* ----------------------------------------------------------------------------
 * Close the 'extern "C"' block
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif    /* ifdef __cplusplus */

#endif    /* INCLUDE_APP_CEM102_H_ */
