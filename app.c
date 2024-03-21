/**
 * @file app.c
 * @brief Main application source file
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

#include <app.h>

#ifdef CFG_FOTA
#include "sys_fota.h"
#endif /*ifdef CFG_FOTA */

/* ----------------------------------------------------------------------------
* Application Version
* ------------------------------------------------------------------------- */
SYS_FOTA_VERSION(APP_VER_ID, APP_VER_MAJOR, APP_VER_MINOR, APP_VER_REVISION);

#ifdef CFG_FOTA
int sys_fota_dfu_state = SYS_FOAT_DFU_STOP;

int get_sys_fota_dfu(void)
{
	return sys_fota_dfu_state;
}

void set_sys_fota_dfu(int sys_fota)
{
	sys_fota_dfu_state = sys_fota;
}


void srand_func(uint32_t seed)
{
	srand(seed);
}

int rand_func(void)
{
	return rand();
}

#endif /*ifdef CFG_FOTA */

struct ble_sleep_api_param_tag ble_sleep_api_param =
{
    .app_sleep_request = 1,
    .max_sleep_duration = MAX_SLEEP_DURATION,
    .min_sleep_duration = MIN_SLEEP_DURATION,
};

volatile uint8_t sleep_mode;

/* RTC flags and counters */
volatile uint8_t wakeup_due_to_RTC = 0;

/* Count number of wake-ups from memory retention sleep */
int wakeup_cnt = 0;

/* Mask to get all reset flags from ACS_RESET_STATUS register */
#define ACS_RESET_STATUS_RESET_FLAGS_MASK         ((uint32_t)(0x03FF0000))

/* Mask to get ACS Reset flag from RESET_STATUS_DIG register */
#define RESET_DIG_STATUS_ACS_RESET_FLAGS_MASK     ((uint32_t)(0x00000001))

int8_t tx_power_level_dbm = 0;




int8_t boot_mode_state = NOT_WAKEUP_FROM_SLLEP_MODE;
int8_t wakeup_sleep_mode = AMR_WAKEUP_SLEEP_MODE_SUCCESS;

/*******************************************************************************************************************************
  * 	Enter Deep Sleep Mode...
  *******************************************************************************************************************************/
void check_wakeup_from_sleep_mode(void)
{
    if((boot_mode_state == AMR_WAKEUP_FROM_SLLEP_MODE)&&(wakeup_sleep_mode == AMR_WAKEUP_SLEEP_MODE_FAIL)) {

#ifdef SWMTRACE_OUTPUT
		swmLogInfo("Switch_NoRetSleep_Mode\r\n");
#endif

		/* Go to Deep Sleep */
		Switch_NoRetSleep_Mode();
    }
}




/*******************************************************************************************************************************
  *		if AMR wakeup. check 1sec
  *******************************************************************************************************************************/
void check_system_boot_status(void)
{
	int wait_tmout;
	uint32_t nvr4_code;

    /* Power up and initialize flash timing registers based on SystemClock */
	App_Flash_Config_Init();

	nvr4_code = Read_Deep_Sleep_Code();

#ifdef SWMTRACE_OUTPUT
		swmLogInfo("FLASH0_NVR4_BASE : [0x%08x]\r\n", nvr4_code);
#endif

	boot_mode_state = NOT_WAKEUP_FROM_SLLEP_MODE;
	wakeup_sleep_mode = AMR_WAKEUP_SLEEP_MODE_SUCCESS;
	wait_tmout = 10;

	if(nvr4_code==DEEP_SLEEP_CODE_ENTER) {									// WAKEUP INTERRUPT HANDLER
		SYS_GPIO_CONFIG(GPIO_NO_RET_SLEEP_PIN, (GPIO_MODE_GPIO_IN | GPIO_LPF_DISABLE | GPIO_WEAK_PULL_UP  | GPIO_6X_DRIVE));
	   do {
		   SYS_WATCHDOG_REFRESH();
		   sys_delay_10ms(10);																				// 10ms * 10 = 100ms

		   if(Sys_GPIO_Read(GPIO_NO_RET_SLEEP_PIN)==GPIO_HIGH) {						// if AMR GPIO pin becomes high, wake up status false...
				boot_mode_state = AMR_WAKEUP_FROM_SLLEP_MODE;
			   wakeup_sleep_mode = AMR_WAKEUP_SLEEP_MODE_FAIL;	// wakeup status fail
			   break;
		   }
	   } while(wait_tmout-->0);

	    /* AMR_WAKEUP_SLEEP_MODE_SUCCESS */
		if(wakeup_sleep_mode == AMR_WAKEUP_SLEEP_MODE_SUCCESS) {
			Write_Deep_Sleep_Code(DEEP_SLEEP_CODE_CLEAR);
		}
	}
}



/*******************************************************************************************************************************
  *		Sleep Mode Process
  *******************************************************************************************************************************/
void BLE_Sleep_Process(void)
{
    if (BLE_Baseband_Is_Awake())
    {
        BLE_Kernel_Process();

        /* Checks for sleep have to be done with interrupt disabled */
        GLOBAL_INT_DISABLE();

        /* Check if processor clock can be gated */
        switch (BLE_Baseband_Sleep(&ble_sleep_api_param))
        {
            case RWIP_DEEP_SLEEP:
            {
                SOC_Sleep(sleep_mode);
                break;
            }

            case RWIP_CPU_SLEEP:
            {
                /* Wait for interrupt */
                __WFI();
                break;
            }

            case RWIP_ACTIVE:
            {
                break;
            }

            default:
            {
            }
        }

        /* Checks for sleep have to be done with interrupt disabled */
        GLOBAL_INT_RESTORE();
    }
    else
    {
        /* Wait for interrupt */
        __WFI();
    }
}

int main()
{
    /* Check if reset due to wakeup from sleep mode:
     *   - All reset flags from ACS_RESET_STATUS register are clear, and
     *   - ACS Reset flag from RESET_STATUS_DIG register is set (regardless of
     *     all other flags) */
    if (((ACS->RESET_STATUS & ACS_RESET_STATUS_RESET_FLAGS_MASK) == 0x0) &&
        ((RESET->DIG_STATUS & RESET_DIG_STATUS_ACS_RESET_FLAGS_MASK) == 0x1))
    {
        DisableAppInterrupts();

        App_No_Ret_Sleep_Init();

        /* Reinitialize the system after wakeup */
        Sys_PowerModes_WakeupWithReset(&no_retention_sleep_mode_cfg);
    }
    else /* Else: Not wakeup from SLEEP mode */
    {
        DisableAppInterrupts();

        /* IMPORTANT: always have this to make sure DEBUG_CATCH_GPIO is not frozen
         * (in case sleep then wake up from FLASH, main() is executed and in here
         * pad retention could be enabled)*/
        /* Disable pad retention */
        ACS_BOOT_CFG->PADS_RETENTION_EN_BYTE = PADS_RETENTION_DISABLE_BYTE;
    }

    /* Change RSL15 Sleep mode Sleep with Memory Retention */
    sleep_mode = SLEEP_WKUP_RAM_MODE;
    measure_status = MEASURE_IDLE;
    vdda_err_flag = 0;

    /* app globals attach...add sodykim 2023.10.23  */
    app_device_malloc_attach();

    DeviceInit();

#ifdef SWMTRACE_OUTPUT
		swmLogInfo("boot_mode_state : [0x%02X]\r\n", boot_mode_state);
#endif

//---------------------------------------------------------------------------------------------------------------------
//	AMR POWER CONTROL...goto Deep Sleep Mode
//---------------------------------------------------------------------------------------------------------------------
#if 0
   /* add sodykim for real AMR mode */
		check_system_boot_status();

		check_wakeup_from_sleep_mode();
#endif

    /* Initialize the Kernel and create application task */
    BLEStackInit();

    /* Initialize Bluetooth Services */
    BatteryServiceServerInit();
    CustomServiceServerInit();

    /* Subscribe application callback handlers to BLE events */
    AppMsgHandlersInit();

    /* Prepare advertising and scan response data (device name + company ID) */
    //PrepareAdvScanData();

    /* Send a message to the BLE stack requesting a reset.
     * The stack returns a GAPM_CMT_EVT / GAPM_RESET event upon completion.
     * See BLE_ConfigHandler follow what happens next. */
    GAPM_SoftwareReset(); /* Step 1 */

    EnableAppInterrupts();

    /* Initialize CEM102 structure */
    cem102 = &Driver_CEM102;

    CEM102_Startup(cem102);

    /* Initialize VBAT Status */
    vbat_status = VBAT_IDLE;

/*	add sodykim for data & time */
    RTC_Clock_Config_Init();

    /* Execute main loop */
    Main_Loop();

    return 0;
}

void Main_Loop(void)
{
#ifdef SWMTRACE_OUTPUT
    /* Initialize UART logging if configuration was lost when
     * sleep mode was enabled */
    Init_SWMTrace();
#endif

    while (1)
    {
        SYS_WATCHDOG_REFRESH();

#ifdef CFG_FOTA
/* Start Update when command apk */
		if (get_sys_fota_dfu() == SYS_FOAT_DFU_RUN)	{
			Sys_Fota_StartDfu(1);
		}
#endif    /* ifdef CFG_FOTA */


         /***************** CEM102 Measurement **********************/
        if (calib_state == CALIB_DONE)
        {
            CEM102_Operation(cem102);

#ifdef STRIPPING_ENABLE
            CEM102_Stripping(cem102);
#endif
        }
        else
        {
            CEM102_Calibrate(cem102);
        }

        /***************** BLE and SLEEP ***************************/
        BLE_Sleep_Process();
    }
}
