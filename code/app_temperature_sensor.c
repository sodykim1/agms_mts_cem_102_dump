/**
 * @file    app_temperature_sensor.c
 * @brief   Source file for the internal temperature sensor interface.
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

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */

/* Device and library headers */
#include <hw.h>
#include <app.h>

/* Application headers */
#include "app_temperature_sensor.h"

/* ----------------------------------------------------------------------------
 * Private Symbolic Constants
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Private Macros
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Private Type Definitions
 * ------------------------------------------------------------------------- */
#define NUM_LSAD_SAMPLE_COUNT	8

/* ----------------------------------------------------------------------------
 * Global Variables
 * ------------------------------------------------------------------------- */

/* Initialize a structure for LSAD gain and offset values from TRIM sector */
static struct F_LSAD_TRIM lsad_gain_offset = {0};

/* Temperature sensor scale */
static float temperature_scale = 0;

/* Temperature sensor offset */
static float temperature_offset = 0;


uint32_t raw_LSAD_sum[8];
/* ----------------------------------------------------------------------------
 * Private Function Prototypes
 * ------------------------------------------------------------------------- */

/**
 * @brief       Initialize the temperature offset and scaler factors
 * @details     Computes an offset and scaler factor to be used to convert LSAD
 *              measurements into temperature readings. If there are any trim
 *              errors, the user-defined default values are used.
 * @param       uint32_t trim_error
 *                  A code indicating if any trim errors occurred during
 *                  initialization.
 * @return      N/A
 */
static void App_TempSensor_InitOffset(uint32_t trim_error);

/**
 * @brief       Initializes LSAD input channels used for the temperature sensor
 * @detail      Configures the LSAD channel used to measure the internal
 *              temperature sensor, and configures the LSAD interrupt.
 * @param       N/A
 * @return      N/A
 */
static void App_TempSensor_InitLSAD(void);

/* ----------------------------------------------------------------------------
 * Private Function Definitions
 * ------------------------------------------------------------------------- */

static void App_TempSensor_InitOffset(uint32_t trim_error)
{
    /* Initialize a pointer to the default trim value sector */
    TRIM_Type *trims = TRIM;

    /* Check for trim errors */
    if (!(trim_error & ERROR_TEMPERATURE_INVALID))
    {
        /* The temperature sensor gain and/or offset are invalid. Use
         * high-frequency trims since we are using a high-speed pre-scaler */
        Sys_LSAD_TempSensor_Gain_Offset(&(trims->temp_sensor), &lsad_gain_offset);
    }

    /* Check for invalid trim values */
    if (trims->measured.temp_sensor_high == 0xFFFF || trims->measured.temp_sensor_30C == 0xFFFF)
    {
        /* Set scaler and offset to user-define default values */
        temperature_scale = DEFAULT_TEMPERATURE_GAIN;
        temperature_offset = DEFAULT_TEMPERATURE_OFFSET;
    }
    else
    {
        /* Compute gain and offset from trim values */
        uint16_t temp_sensor_high = trims->measured.temp_sensor_high;
        uint16_t temp_sensor_30C = trims->measured.temp_sensor_30C;
#ifdef RSL15_CID
        temperature_scale = (float)(temp_sensor_high  - temp_sensor_30C) / 60.0f;
#else /* ifdef RSL15_CID */
        temperature_scale = (float)(temp_sensor_high  - temp_sensor_30C) / 20.0f;
#endif /* ifdef RSL15_CID */
        temperature_offset = (temp_sensor_30C / temperature_scale) - 30.0f;
    }
}

static void App_TempSensor_InitLSAD(void)
{
    /* Configure the input channel in single ended mode with LSAD positive
     * connected to the temperature sensor and LSAD negative to ground */
    Sys_LSAD_InputConfig(
        TEMPERATURE_SENSOR_CHANNEL,
        LSAD_TEMP_CFG,
        LSAD_GROUND_CFG
    );

    Sys_LSAD_InputConfig(
    	LSAD_BAT_CHANNEL,
		LSAD_VBAT_DIV2_CFG,
		LSAD_GROUND_CFG
	);

#if 0
    /*Configure the sampling mode and rate for the LSAD */
#if RSL15_CID == 202
    LSAD->CFG = LSAD_NORMAL             /* Normal mode, sample all 8 channels */
              | LSAD_PRESCALE_1280H;    /* Sample rate is SLOWCLK/1280 */
#else /* if RSL15_CID == 202 */
    LSAD->CFG = VBAT_DIV2_ENABLE        /* Enable VBAT voltage divider */
              | LSAD_NORMAL             /* Normal mode, sample all 8 channels */
              | LSAD_PRESCALE_1280H;    /* Sample rate is SLOWCLK/1280 */
#endif /* if RSL15_CID == 202 */
#endif

    /* Enable the internal temperature sensor */
    uint32_t sensor_cfg_register = LSAD_TEMP_SENS_DUTY | TEMP_SENS_ENABLE;
#ifdef RSL15_CID
    Sys_ACS_WriteRegister(&ACS->TEMP_SENSOR_CFG, sensor_cfg_register);
#else /* ifdef RSL15_CID */
    ACS->TEMP_SENSOR_CFG = sensor_cfg_register;
#endif /* ifdef RSL15_CID */
}

/* ----------------------------------------------------------------------------
 * Public Function Definitions
 * ------------------------------------------------------------------------- */

void App_TempSensor_Init(uint32_t trim_error)
{
    /* Initialize the gain and offset used for measurement conversion */
    App_TempSensor_InitOffset(trim_error);

    /* Initialize the LSAD channel and interrupt */
    App_TempSensor_InitLSAD();
}

uint32_t App_TempSensor_Measure(void)
{
    /* Retrieve the LSAD measurement */
    uint32_t lsad_value = LSAD->DATA_TRIM_CH[TEMPERATURE_SENSOR_CHANNEL];

//    temperature_scale = DEFAULT_TEMPERATURE_GAIN;
//    temperature_offset = DEFAULT_TEMPERATURE_OFFSET;

    /* Compute the temperature from the LSAD measurement. The value is scaled
     * up two decimal places to increase accuracy. */
    uint32_t mantissa = (uint32_t)(
        ((lsad_value / temperature_scale) - temperature_offset) * 100
    );

    /* Set the exponent value to -2 to compensate for the up-scaling */
    uint8_t exponent = 0xFE;

    /* Store the temperature encoded as an IEEE-11073 32-bit floating point */
    uint32_t temperature = (
        ((exponent << 24) & 0xFF000000) | ((mantissa << 0) & 0x00FFFFFF)
    );

    return temperature;
}



/************************************************************************************************************
 *	LSAD Initial & run & measure
 *	LSAD POLLING 방식은 10ms 간격으로 8번을 scan 해야 정상적인 데이터 값을 읽어 올 수 있다.
 *	LSAD POLLING은 LSAD_INPUT_CH0~CH7까지 순차적으로 변환이 되기 때문에 LSAD_INPUT_CH0 부터 값을 읽어 처리 한다.
 *	즉 LSAD_INPUT_CH0, CH1, CH2, CH3 이기 때문에 최소 40ms대기 해야 한다.
 *	모니터링 방식을 사용하면 되지 않는다...
 ************************************************************************************************************/
void LSAD_measure_sensor_level(void)
{
	uint8_t i;
	uint32_t encoded_temp;
	uint32_t lsad_value;

	uint16_t  vbat_lvl_gab;

    /* Stop the LSAD sampling */
    LSAD->CFG = LSAD_DISABLE;

    Sys_LSAD_InputConfig(TEMPERATURE_SENSOR_CHANNEL, LSAD_TEMP_CFG, LSAD_GROUND_CFG);
    Sys_LSAD_InputConfig(LSAD_BAT_CHANNEL, 		LSAD_VBAT_DIV2_CFG, LSAD_GROUND_CFG);

    /* Configure the sampling mode and rate for the LSAD */
#if RSL15_CID == 202
    LSAD->CFG = LSAD_NORMAL             /* Normal mode, sample all 8 channels */
              | LSAD_PRESCALE_1280H;    /* Sample rate is SLOWCLK/1280 */
#else /* if RSL15_CID == 202 */
    LSAD->CFG = VBAT_DIV2_ENABLE        /* Enable VBAT voltage divider */
              | LSAD_NORMAL             /* Normal mode, sample all 8 channels */
              | LSAD_PRESCALE_1280H;    /* Sample rate is SLOWCLK/1280 */
#endif /* if RSL15_CID == 202 */

	for(i = LSAD_INPUT_CH0; i <= (LSAD_BAT_CHANNEL+1); i++){
		SYS_WATCHDOG_REFRESH();
		Sys_Delay(LSAD_SCAN_DELAY);					// 10ms * ms =
	}

	lsad_value = App_TempSensor_Measure();
	encoded_temp = (lsad_value&0x00FFFFFF);
    rsl15_info->temperature = (uint16_t)(encoded_temp);

    /*	change for UXN protocol format */
	lsad_value = App_VBAT_Measure();

	if(afe_102->measure_update) {
		if(afe_102->vbat_lvl_mV>=lsad_value) {
			vbat_lvl_gab = afe_102->vbat_lvl_mV-lsad_value;
		}
		else {
			vbat_lvl_gab = lsad_value - afe_102->vbat_lvl_mV;
		}

		if(vbat_lvl_gab<1000) {															// gab이 1v 이내라면..ok
			rsl15_info->vbat_lvl_mV = (uint16_t)(lsad_value*10);
		}
	}
	else {
		rsl15_info->vbat_lvl_mV = (uint16_t)(lsad_value*10);
	}

#ifdef SWMTRACE_DEBUG
	swmLogInfo( "Temp : [%d]  VBat : [%d] LSAD: [%d], CEM102: [%d], GAB :[%d]\r\n", rsl15_info->temperature, rsl15_info->vbat_lvl_mV, lsad_value, afe_102->vbat_lvl_mV, vbat_lvl_gab);
#endif /* SWMTRACE_DEBUG */

    /* Stop the LSAD sampling */
    LSAD->CFG = LSAD_DISABLE;
}


