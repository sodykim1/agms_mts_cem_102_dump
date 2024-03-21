/**
 * @file  app_lowpwr_manager.c
 * @brief Source file for application's low-power manager.
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

#include "app.h"

LowPowerModeCfg_t mem_retention_sleep_mode_cfg;
LowPowerModeCfg_t no_retention_sleep_mode_cfg;

uint32_t vddpa_ctrl;
uint32_t vddrf_ctrl;
uint8_t vddpa_power;
SYSCTRL_VDDPA_CFG0_Type vddpa_cfg0;

volatile int vdda_monitor_flag = 0;

/* Storage variables for the GPIO registers */
static uint32_t gpio_cfg[GPIO_PAD_COUNT] = {0};
static uint32_t gpio_output = 0;
static uint32_t gpio_jtag_sw_pad_cfg = 0;

static void App_LowPower_GPIOSaveStates(void)
{
    for (uint8_t i = 0; i < GPIO_PAD_COUNT; i++)
    {
        gpio_cfg[i] = GPIO->CFG[i];
    }
    gpio_output = GPIO->OUTPUT_DATA;
    gpio_jtag_sw_pad_cfg = GPIO->JTAG_SW_PAD_CFG;
}

static void App_LowPower_GPIORestoreStates(void)
{
    for (uint8_t i = 0; i < GPIO_PAD_COUNT; i++)
    {
        GPIO->CFG[i] = gpio_cfg[i];
    }
    GPIO->OUTPUT_DATA = gpio_output;
    GPIO->JTAG_SW_PAD_CFG = gpio_jtag_sw_pad_cfg;
}

static void App_LowPower_RFSaveStates(void)
{
    vddrf_ctrl = ACS->VDDRF_CTRL;
    vddpa_ctrl = ACS->VDDPA_CTRL;
    vddpa_power = RF0_REG1A->PA_PWR_PA_PWR_BYTE;
    vddpa_cfg0 = (*(SYSCTRL_VDDPA_CFG0));
}

static void App_LowPower_RFRestoreStates(void)
{
    if (vddrf_ctrl != 0)
    {
        /* Restore VDDRF supply without changing trimming settings */
        Sys_ACS_WriteRegister(&ACS->VDDRF_CTRL, vddrf_ctrl);

        /* Wait until VDDRF supply has powered up */
        while (!(ACS->VDDRF_CTRL & VDDRF_READY))
        {
            /* Waiting */
        }

        /* Restore VDDPA */
        Sys_ACS_WriteRegister(&ACS->VDDPA_CTRL, vddpa_ctrl);

        RF0_REG1A->PA_PWR_PA_PWR_BYTE = vddpa_power;
        (*(SYSCTRL_VDDPA_CFG0)) = vddpa_cfg0;
    }
}

void App_LowPower_SavePeripheralStates(void)
{
    /* Save the states of the GPIO registers */
    App_LowPower_GPIOSaveStates();
}

void App_LowPower_RestorePeripheralStates(void)
{
    /* Restore the states of the GPIO registers */
    App_LowPower_GPIORestoreStates();

    CEM102_Initialize(cem102_dut);
}

void SOC_Sleep(uint8_t power_mode)
{
    if (power_mode == SLEEP_WKUP_RAM_MODE)
    {
        /* Save required registers to restore TX power after
         * a wake up from sleep */
        App_LowPower_RFSaveStates();

        /* Initialize the wakeup configuration */
        Sys_PowerModes_SetWakeupConfig(mem_retention_sleep_mode_cfg.wakeup_cfg);

        /* Clear all wakeup flags */
        WAKEUP_FLAGS_CLEAR();

#if DEBUG_SLEEP_GPIO == DEBUG_SYSCLK_PWR_MODE
        /* Set power mode GPIO to indicate power mode */
        Sys_GPIO_Set_High(POWER_MODE_GPIO);
#endif

        /* Power Mode enter sleep with memory retention */
        Sys_PowerModes_EnterPowerMode(&mem_retention_sleep_mode_cfg);
    }
    else
    {
        /* Initialize the wakeup configuration */
        Sys_PowerModes_SetWakeupConfig(no_retention_sleep_mode_cfg.wakeup_cfg);

        /* Clear all wakeup flags */
        WAKEUP_FLAGS_CLEAR();

#if DEBUG_SLEEP_GPIO == DEBUG_SYSCLK_PWR_MODE
        /* Set power mode GPIO to indicate power mode */
        Sys_GPIO_Set_High(POWER_MODE_GPIO);
#endif
        /* To disable the clock detector, it is recommended to:
         * 1. Ignore the reset by setting the ACS_CLK_DET_CTRL.RESET_IGNORE bit.
         * 2. Disable the clock detector by resetting the ACS_CLK_DET_CTRL.ENABLE bit. */
        Sys_ACS_WriteRegister(&ACS->CLK_DET_CTRL, ((ACS->CLK_DET_CTRL | ((uint32_t)(0x1U << ACS_CLK_DET_CTRL_RESET_IGNORE_Pos))) &
            (~((uint32_t)(0x1U << ACS_CLK_DET_CTRL_ENABLE_Pos)))));

        /* Clear reset flags */
        RESET->DIG_STATUS = (uint32_t)0xFFFF;
        Sys_ACS_WriteRegister(&ACS->RESET_STATUS, (uint32_t)0xFFFF);

        /* Power Mode enter sleep with no retention */
        Sys_PowerModes_EnterPowerMode(&no_retention_sleep_mode_cfg);
    }
}

void Switch_NoRetSleep_Mode(void)
{
    /* Change RSL15 sleep mode to No Retention Sleep */
    sleep_mode = SLEEP_NO_RETENTION_MODE;

    NVIC_DisableIRQ(GPIO0_IRQn);

    /* Put CEM102 in RESET mode */
    Sys_GPIO_Set_Low(cem102_dut->pwr_en);

    /* Wait 20ms to put CEM102 in RESET */
    Sys_Delay(SystemCoreClock / 50);

    cem102->VDDCCharge(cem102_dut);

    /* Manually turn off BG, VCC, and charge pump to reduce the current consumption */
    Sys_ACS_WriteRegister(&ACS->SLEEP_MODE_CFG, BG_DISABLE_IN_SLEEP | VCC_DISABLE_IN_SLEEP | VDDCP_DISABLE_IN_SLEEP);

    /* Turn off sensor interface */
    Sys_Sensor_Disable();

    /* Turn off XTAL32K LPCLK */
    Sys_ACS_WriteRegister(&ACS->XTAL32K_CTRL, (ACS->XTAL32K_CTRL) & (~(XTAL32K_ENABLE)));

    while ((ACS->XTAL32K_CTRL & XTAL32K_OK))
    {
        /* Waiting */
    }

    GLOBAL_INT_DISABLE();
    SOC_Sleep(sleep_mode);
    GLOBAL_INT_RESTORE();
}

void GPIO0_Wakeup_Process_Handler(void)
{
    WAKEUP_GPIO0_FLAG_CLEAR();

    if (sleep_mode == SLEEP_WKUP_RAM_MODE)
    {
        Switch_NoRetSleep_Mode();
    }
}
extern volatile uint32_t irq;

void GPIO1_Wakeup_Process_Handler(void)
{
	irq=1;
    WAKEUP_GPIO1_FLAG_CLEAR();
}

void RTC_Alarm_Wakeup_Process_Handler(void)
{
    WAKEUP_RTC_ALARM_FLAG_CLEAR();

    wakeup_due_to_RTC = 1;
}

void Threshold_Wakeup_Process_Handler(void)
{
    WAKEUP_THRESHOLD_FULL_FLAG_CLEAR();
    vdda_monitor_flag = 1;
}

void WAKEUP_IRQHandler(void)
{
    SYS_WATCHDOG_REFRESH();

    /* If POWER_MODE is set to other than VDDC_IN_RETENTION
     * we need to restore VDDPA settings to keep
     * TX power consistent during sleep and wake up
     */
    App_LowPower_RFRestoreStates();

#if DEBUG_SLEEP_GPIO == DEBUG_SYSCLK_PWR_MODE
    /* Clear POWER_MODE_GPIO to indicate Run Mode Configuration */
    Sys_GPIO_Set_Low(POWER_MODE_GPIO);
#endif

    /* Call GPIO handler to process wakeup event */
    if (ACS->WAKEUP_CTRL & WAKEUP_GPIO0_EVENT_SET)
    {
        GPIO0_Wakeup_Process_Handler();
    }

    if (ACS->WAKEUP_CTRL & WAKEUP_GPIO1_EVENT_SET)
    {
        GPIO1_Wakeup_Process_Handler();
    }

    /* Check if RTC wakeup event set */
    if (ACS->WAKEUP_CTRL & WAKEUP_RTC_ALARM_EVENT_SET)
    {
        /* Call RTC Wakeup Handler */
        RTC_Alarm_Wakeup_Process_Handler();
    }

    /* Call BBTimer handler to process wakeup event */
    if (ACS->WAKEUP_CTRL & WAKEUP_BB_TIMER_EVENT_SET)
    {
        WAKEUP_BB_TIMER_FLAG_CLEAR();
    }

    /* Check if Sensor Threshold event set */
    if (ACS->WAKEUP_CTRL & WAKEUP_THRESHOLD_EVENT_SET)
    {
        Threshold_Wakeup_Process_Handler();
    }

    /* If there is an pending wakeup event set during the execution of this
     * Wakeup interrupt handler. Set the NVIC WAKEUP_IRQn so that the pending wakeup
     * event will be serviced again */
    if (ACS->WAKEUP_CTRL)
    {
        if (!NVIC_GetPendingIRQ(WAKEUP_IRQn))
        {
            NVIC_SetPendingIRQ(WAKEUP_IRQn);
        }
    }
}
