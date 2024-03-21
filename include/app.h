/**
 * @file  app.h
 * @brief Main application header
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

#ifndef APP_H
#define APP_H

/* ----------------------------------------------------------------------------
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif    /* ifdef __cplusplus */

/* ----------------------------------------------------------------------------
 * Include files
 * --------------------------------------------------------------------------*/
/* Device and libraries headers */
#include <hw.h>
#include <swmTrace_api.h>
#include <flash_rom.h>
#include <ble_protocol_support.h>
#include <ble_abstraction.h>

/* Application headers */
#include <app_customss.h>
#include <app_bass.h>
#include <app_init.h>
#include <app_msg_handler.h>
#include "wakeup_source_config.h"
#include "scheduler.h"
#include "RTE_Device.h"
#include "app_cem102.h"
#include "app_lowpwr_manager.h"

#include "app_globals.h"
#include "app_utility.h"

#ifdef CFG_FOTA
#define FOTA_GPIO                       2
#endif    /* ifdef CFG_FOTA */

/** Application Version  */
#ifdef CFG_FOTA
#define APP_VER_ID                      "gmfota"
#define APP_VER_MAJOR                   1
#define APP_VER_MINOR                   0
#define APP_VER_REVISION                0

#define	SYS_FOAT_DFU_STOP		0x00
#define	SYS_FOAT_DFU_RUN			0xA1

void set_sys_fota_dfu(int sys_fota);
#endif    /* ifdef CFG_FOTA */


typedef enum _Sleep_modes_t
{
    SLEEP_WKUP_RAM_MODE    = 0,
    SLEEP_NO_RETENTION_MODE = 2
} Sleep_modes_t;

/* ----------------------------------------------------------------------------
 * Defines
 * --------------------------------------------------------------------------*/

/* Power Reduction Defines Begin */
/* Set to 1 to power down the sensor interface.
 * The sensor must be powered up when the BB timer is needed.
 * We recommend that you leave the sensor enabled in
 * Bluetooth Low Energy applications; therefore, leave this setting as 0. */
#define SENSOR_POWER_DISABLE            0

/* Set this to 1 to Power Down CryptoCell
 * notes: settings this to 1 will lock debug port and JTAG debug will not work */
#define CC312AO_POWER_DISABLE           0

/* Set this to 1 to Power Down FPU
 * note: If FPU is used during run mode this should be left 0 */
#define POWER_DOWN_FPU                  0

/* Set this to 1 to Power Down Debug Unit
 * note: If Debug Port is used during run mode this should be left 0 */
#define POWER_DOWN_DBG                  0

/* To enable the Charge Pump in run mode and disable it in sleep mode */
#define CHARGE_PUMP_ENABLE              1

/* To enable the Charge Pump in both run mode and sleep mode */
#define CHARGE_PUMP_ALWAYS_ENABLE       2

#define CHARGE_PUMP_CFG                 CHARGE_PUMP_ALWAYS_ENABLE
/* Power Reduction Defines End */

#define APP_BLE_DEV_PARAM_SOURCE        APP_PROVIDED /* or APP_PROVIDED FLASH_PROVIDED_or_DFLT */

/* Location of BLE public address
 *   - BLE public address location in MNVR is used as a default value;
 *   - Any other valid locations can be used as needed.
 */
#define APP_BLE_PUBLIC_ADDR_LOC         BLE_PUBLIC_ADDR_LOC_MNVR

/* Advertising channel map - 37, 38, 39 */
#define APP_ADV_CHMAP                   0x07

/* Advertisement connectable mode */
#define ADV_CONNECTABLE_MODE            0

/* Advertisement non-connectable mode */
#define ADV_NON_CONNECTABLE_MODE        1

/* Advertisement connectability mode
 * Options are:
 *     - ADV_CONNECTABLE_MODE
 *     - ADV_NON_CONNECTABLE_MODE */
#ifndef APP_ADV_CONNECTABILITY_MODE
#define APP_ADV_CONNECTABILITY_MODE     ADV_CONNECTABLE_MODE
#endif

/* Connection Parameters */
#define MIN_CONNECTION_INTERVAL         72 /* Minimum Connection Interval (72 * 1.25 ms) */
#define MAX_CONNECTION_INTERVAL         300 /* Maximum Connection Interval (300 * 1.25 ms) */
#define SLAVE_LATENCY                   6 /* Slave Latency */
#define CONNECTION_SUPERVISION_TIMEOUT  3000 /* Connection Supervision Timeout (3000 * 10 ms) */
#define MIN_CONNECTION_EVENT_DURATION   0xFFFF   /* Minimum Connection Event Duration */
#define MAX_CONNECTION_EVENT_DURATION   0xFFFF   /* Maximum Connection Event Duration */


/* Define the advertisement interval for connectable mode (units of 625us)
 * Notes: the interval can be 20ms up to 10.24s */
#ifdef CFG_ADV_INTERVAL_MS
#define ADV_INT_CONNECTABLE_MODE        (CFG_ADV_INTERVAL_MS / 0.625)
#else    /* ifdef CFG_ADV_INTERVAL_MS */
#define ADV_INT_CONNECTABLE_MODE        800	// 2800	//64
#endif    /* ifdef CFG_ADV_INTERVAL_MS */

/* Define the advertisement interval for non-connectable mode (units of 625us)
 * Notes: the minimum interval for non-connectable advertising should be 100ms */
#ifdef CFG_ADV_INTERVAL_MS
#define ADV_INT_NON_CONNECTABLE_MODE    (CFG_ADV_INTERVAL_MS / 0.625)
#else    /* ifdef CFG_ADV_INTERVAL_MS */
#define ADV_INT_NON_CONNECTABLE_MODE    160
#endif    /* ifdef CFG_ADV_INTERVAL_MS */

/* Set the advertisement interval */
#if (APP_ADV_CONNECTABILITY_MODE == ADV_CONNECTABLE_MODE)
#define APP_ADV_INT_MIN                 ADV_INT_CONNECTABLE_MODE
#define APP_ADV_INT_MAX                 ADV_INT_CONNECTABLE_MODE
#define APP_ADV_DISCOVERY_MODE          GAPM_ADV_MODE_GEN_DISC
#define APP_ADV_PROPERTIES              GAPM_ADV_PROP_UNDIR_CONN_MASK
#elif (APP_ADV_CONNECTABILITY_MODE == ADV_NON_CONNECTABLE_MODE)
#define APP_ADV_INT_MIN                 ADV_INT_NON_CONNECTABLE_MODE
#define APP_ADV_INT_MAX                 ADV_INT_NON_CONNECTABLE_MODE
#define APP_ADV_DISCOVERY_MODE          GAPM_ADV_MODE_NON_DISC
#define APP_ADV_PROPERTIES              GAPM_ADV_PROP_NON_CONN_NON_SCAN_MASK
#endif    /* if (APP_ADV_CONNECTABILITY_MODE== ADV_CONNECTABLE_MODE) */

#define APP_PUBLIC_ADDRESS              { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB }

#define GAPM_CFG_ADDR_PUBLIC            (0 << GAPM_PRIV_CFG_PRIV_ADDR_POS)
#define GAPM_CFG_ADDR_PRIVATE           (1 << GAPM_PRIV_CFG_PRIV_ADDR_POS)

#define GAPM_CFG_HOST_PRIVACY           (0 << GAPM_PRIV_CFG_PRIV_EN_POS)
#define GAPM_CFG_CONTROLLER_PRIVACY     (1 << GAPM_PRIV_CFG_PRIV_EN_POS)

#define GAPM_ADDRESS_TYPE               GAPM_CFG_ADDR_PUBLIC
#define GAPM_PRIVACY_TYPE               GAPM_CFG_HOST_PRIVACY

#define APP_BD_RENEW_DUR                150 /* in seconds */

#define GAPM_OWN_ADDR_TYPE              GAPM_STATIC_ADDR /* GAPM_STATIC_ADDR, GAPM_GEN_NON_RSLV_ADDR */

/* BLE private address of local device */
#define APP_BLE_PRIVATE_ADDR            { 0x94, 0x11, 0x55, 0xff, 0xbb, 0xDC }

#define SECURE_CONNECTION               1  /* set 0 for LEGACY_CONNECTION or 1 for SECURE_CONNECTION */

/* The number of standard profiles and custom services added in this application */
#define APP_NUM_STD_PRF                 (1)
#define APP_NUM_CUST_SVC                (1)

/* GPIO number that is connected to LED of EVB */
#define LED_STATE_GPIO                  GREEN_LED

/* GPIO number that is used to determine the number of BLE connections */
#define CONNECTION_STATE_GPIO           BLUE_LED

/* GPIO number that is used for sending button press and release event */
#define BUTTON_GPIO                     0

#define TWOSC                           3200 /* us */

#define MAX_SLEEP_DURATION              96000 /* 30s */
#define MIN_SLEEP_DURATION              3500 /* 3500us */
#define LPCLK_SRC_XTAL32                0
#define LPCLK_SRC_RC32                  1

/* Dynamic measure and update of RC32K
 * Options
 * - 1 dynamic measure and update every LOW_POWER_CLK_MEASUREMENT_INTERVAL_S seconds
 * - 0 one time initial measurement during cold reset boot */
#define LPCLK_DYNAMIC_UPDATE            1

/* Low power standby clock selection
 * Options
 * - LPCLK_SRC_XTAL32
 * - LPCLK_SRC_RC32  */
#ifndef LPCLK_STANDBYCLK_SRC
#define LPCLK_STANDBYCLK_SRC            LPCLK_SRC_XTAL32
#endif

/* Define low power clock accuracy in ppm used by timing algorithms */
#if LPCLK_STANDBYCLK_SRC == LPCLK_SRC_XTAL32
#define LOW_POWER_CLOCK_ACCURACY        500  /* ppm */
#elif LPCLK_STANDBYCLK_SRC == LPCLK_SRC_RC32
#define LOW_POWER_CLOCK_ACCURACY        500 /* ppm */
#endif

/* How long in seconds between RC_OSC period updates */
#define LOW_POWER_CLK_MEASUREMENT_INTERVAL_S       8

/* Total measurement cycles to count initially */
#define LOW_POWER_CLK_INITIAL_MEASUREMENT          1024

/* Total measurement cycles to count after first update */
#define LOW_POWER_CLK_DYNAMIC_MEASUREMENT          128

/* Scaling factor taking into account System clock and 16 audio sink periods.
 *  8*16 = 128. If the system core clock frequency or the number of audiosink
 *  measurements are changed, change this value accordingly. */
#define LOW_POWER_CLK_SCALE_AVERAGE_PERIOD         ((SystemCoreClock*16)/1000000.0)

#define SYSTEM_CLK                      8000000

/* Set UART peripheral clock */
#define UART_CLK                        SYSTEM_CLK

/* Set sensor clock */
#define SENSOR_CLK                      32768

/* Set user clock */
#define USER_CLK                        1000000

/* Enable/disable buck converter
 * Options: VCC_BUCK or VCC_LDO
 */
#define VCC_BUCK_LDO_CTRL               VCC_LDO

/* LSAD channel used for calibrating the VCC regulator. */
#define LSAD_CHANNEL_VCC_CALIB          1

/* Target voltage for VCC regulator calibration */
#define TARGET_DCDC_1_25V               125

/** Application specific other defines */
/* Timer setting in units of 1ms (kernel timer resolution) */
#define TIMER_SETTING_MS(MS)            MS
#define TIMER_SETTING_S(S)              (S * 1000)

/* Hold duration required to trigger bond list clear (in seconds) */
#define CLR_BONDLIST_HOLD_DURATION_S    5

/* Set 0 for default permission or 1 to require a secure connection link */
#define BUTTON_SECURE_ATTRIBUTE         (0)

/* Delay between each toggle in clear bond list (in milliseconds) */
#define CLR_BONDLIST_LED_TOGGLE_INTERVAL_MS    100

#define CONCAT(x, y)                    x##y
#define GPIO_SRC(x)                     CONCAT(GPIO_SRC_GPIO_, x)

/* Options for DEBUG SLEEP GPIO */
#define DEBUG_SLEEP_GPIO_OFF            0
#define DEBUG_SYSCLK_PWR_MODE           1
#define DEBUG_RTC_TASKS_MODE            2
#define DEBUG_SWMTRACE_MODE             3

/* DEBUG_SLEEP_GPIO options:
 *   - DEBUG_SLEEP_GPIO_OFF
 *   - DEBUG_SYSCLK_PWR_MODE
 *   - DEBUG_RTC_TASKS_MODE
 *   - DEBUG_SWMTRACE_MODE
 *   Notes: Set this to DEBUG_SLEEP_GPIO_OFF to reduce power consumption
 */
#define DEBUG_SLEEP_GPIO                DEBUG_SLEEP_GPIO_OFF
//#define DEBUG_SLEEP_GPIO                DEBUG_SWMTRACE_MODE

#if (DEBUG_SLEEP_GPIO == DEBUG_SWMTRACE_MODE)
#define SWMTRACE_OUTPUT                 1
#endif

#ifdef SWMTRACE_OUTPUT
/* The GPIO pin to use for TX when using the UART mode */
#define UART_TX_GPIO                    (5)

/* The GPIO pin to use for RX when using the UART mode */
#define UART_RX_GPIO                    (7)

/* The selected baud rate for the application when using UART mode */
#define UART_BAUD                       (115200)
#endif /* SWMTRACE_OUTPUT */

#if SWMTRACE_OUTPUT
    #define SWMTRACE_TXINPROGRESS       swmTrace_txInProgress()
#else
    #define SWMTRACE_TXINPROGRESS       (0)
#endif
/* GPIO number that is used for Debug Catch Mode
 * to easily connect the device to the debugger */
#define DEBUG_CATCH_GPIO                (7)

/* RSL15 VDDC GPIO */
#define RSL15_VDDC_GPIO                 (5)

/* GPIO used to enter or exit No Retention SLEEP mode */
#define GPIO_NO_RET_SLEEP_PIN           (0)

/* GPIO used to enter or exit DEEP SLEEP mode */
#define GPIO_DEEP_SLEEP_PIN             0

/* GPIO wakeup pin */
#define GPIO_WAKEUP_PIN                 CEM102_IRQ_GPIO

/* Set the advertisement interval */
#if (DEBUG_SLEEP_GPIO == DEBUG_SYSCLK_PWR_MODE)
#define POWER_MODE_GPIO                 (7)
#define SYSCLK_GPIO                     (8)
#elif (DEBUG_SLEEP_GPIO == DEBUG_RTC_TASKS_MODE)
#define BASS_RTC_TASK_GPIO              (8)
#endif    /* if (DEBUG_SLEEP_GPIO== DEBUG_SYSCLK_PWR_MODE) */

/* LSAD channel to use for calibration purposes */
#define LSAD_CALIB_CHANNEL              (0)

/* Default LSAD channel for SetTxPower */
#define LSAD_TXPWR_DEF                  (1)

/* Default TX power setting */
#define DEF_TX_POWER                    (0)

/* Timer setting in units of 1ms (kernel timer resolution) */
#define TIMER_SETTING_MS(MS)            MS
#define TIMER_SETTING_S(S)              (S * 1000)

/* Advertising data is composed by device name and company id */
#define APP_DEVICE_NAME                 "AGMS_T10-3_Vxxx_0000"
#define APP_DEVICE_NAME_LEN             sizeof(APP_DEVICE_NAME)-1

/* Manufacturer info (onsemi Company ID) */
#define APP_COMPANY_ID                  { 0x62, 0x3 }
#define APP_COMPANY_ID_LEN              (2)

#define APP_DEVICE_APPEARANCE           (0)
#define APP_PREF_SLV_MIN_CON_INTERVAL   (8)
#define APP_PREF_SLV_MAX_CON_INTERVAL   (10)
#define APP_PREF_SLV_LATENCY            (0)
#define APP_PREF_SLV_SUP_TIMEOUT        (200)

/* Application-provided IRK */
#define APP_IRK                         { 0x01, 0x23, 0x45, 0x68, 0x78, 0x9a, \
                                          0xbc, 0xde, 0x01, 0x23, 0x45, 0x68, \
                                          0x78, 0x9a, 0xbc, 0xde }

/* Application-provided CSRK */
#define APP_CSRK                        { 0x01, 0x23, 0x45, 0x68, 0x78, 0x9a, \
                                          0xbc, 0xde, 0x01, 0x23, 0x45, 0x68, \
                                          0x78, 0x9a, 0xbc, 0xde }

#define AOUT_ENABLE_DELAY               SystemCoreClock / 100    /* delay set to 10ms */
#define AOUT_GPIO                       (2)
#define LSAD_SCAN_DELAY               	SystemCoreClock / 100    /* delay set to 10ms */

/* convert time(ms) to RTC timer counter value */
#define CONVERT_MS_TO_32K_CYCLES(x) (x * 32.768)

/* Clear all RESET_DIG_STATUS bits mask */
#define RESET_DIG_STATUS_CLEAR (ACS_RESET_FLAG_CLEAR      |\
                                CM33_SW_RESET_FLAG_CLEAR  |\
                                WATCHDOG_RESET_FLAG_CLEAR |\
                                LOCKUP_FLAG_CLEAR         |\
                                DEU_RESET_FLAG_CLEAR)

/* Clear all ACS_RESET_STATUS bits mask */
#define ACS_RESET_STATUS_CLEAR (POR_RESET_FLAG_CLEAR         |\
                                PAD_RESET_FLAG_CLEAR         |\
                                BG_VREF_RESET_FLAG_CLEAR     |\
                                VDDC_RESET_FLAG_CLEAR        |\
                                VDDM_RESET_FLAG_CLEAR        |\
                                VDDFLASH_RESET_FLAG_CLEAR    |\
                                CLK_DET_RESET_FLAG_CLEAR     |\
                                TIMEOUT_RESET_FLAG_CLEAR     |\
                                WRONG_STATE_RESET_FLAG_CLEAR |\
                                CCAO_REBOOT_RESET_FLAG_CLEAR)

#define INTERNAL_RC_CLK_FREQUENCY       (32768)
#define APP_WE1_UPPER_HALF_WORD         (0)
#define APP_WE1_LOWER_HALF_WORD         (1)
#define APP_WE2_UPPER_HALF_WORD         (2)
#define APP_WE2_LOWER_HALF_WORD         (3)

/* Number of RegisterRead attempts before timeout.
 * It can take as many as 160 attempts to read the OTP.
 */
#define OTP_READ_MAX_ATTEMPTS           (160)
#define APP_BANDGAP_VREF_TRIM_DEFAULT   (0x1F)
#define APP_VDD_TRIM_DEFAULT            (0)

/* DAC target voltage, 500 mV */
#define WE_DAC_TARGET_MV                (500)
#define APP_VOLTAGE_MV_TO_UV            (1000)


#define DIF_DAC_TARGET_MV                	(250)
#define REF_DAC_TARGET_MV                	(800)

#define WE1_DAC_TARGET_MV               (REF_DAC_TARGET_MV+DIF_DAC_TARGET_MV)
#define WE2_DAC_TARGET_MV               (REF_DAC_TARGET_MV+DIF_DAC_TARGET_MV)

#define DAC_TARGET_BASE_MV			      (25000)
//#define DAC_TARGET_BASE_MV			      (2500)



#define CEM102_DAC_MV_REF    800

#define POTENT_GPIO_7                (7)
#define POTENT_GPIO_8                (8)


/* Following two definitions will bring RSL15 SDK 1.8 compatibility */
#ifndef VCC_ICHTRIM_96MA
#define VCC_ICHTRIM_96MA                (0x5 << ACS_VCC_CTRL_ICH_TRIM_Pos)
#endif
#ifndef VCC_ICHTRIM_DEFAULT
#define VCC_ICHTRIM_DEFAULT             (VCC_ICHTRIM_96MA)
#endif

/* ----------------------------------------------------------------------------
 * Global variables and types
 * --------------------------------------------------------------------------*/
extern LowPowerModeCfg_t mem_retention_sleep_mode_cfg;
extern LowPowerModeCfg_t no_retention_sleep_mode_cfg;
extern volatile uint8_t sleep_mode;
extern volatile uint8_t vdda_err_flag;

/* Flag to make sure wakeup happened due to RTC */
extern volatile uint8_t wakeup_due_to_RTC;
extern volatile uint32_t elapsed_carryover;

extern volatile int vdda_monitor_flag;
extern int8_t tx_power_level_dbm;
/* ---------------------------------------------------------------------------
* Function prototype definitions
* --------------------------------------------------------------------------*/
void Main_Loop(void);
void BLE_Sleep_Process(void);
/* ----------------------------------------------------------------------------
 * Close the 'extern "C"' block
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif    /* ifdef __cplusplus */

#endif    /* APP_H */

