/**
 * @file  app_init.h
 * @brief Application initialization header
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

#ifndef APP_INIT_H
#define APP_INIT_H

/* ----------------------------------------------------------------------------
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif    /* ifdef __cplusplus */

/**
 * @brief       Initialize the device configurations
 */
void DeviceInit(void);

/**
 * @brief       Initialize application related message handlers
 */
void AppMsgHandlersInit(void);

/**
 * @brief       Initialize the BLE battery service server
 */
void BatteryServiceServerInit(void);

/**
 * @brief       Initialize the BLE custom service server
 */
void CustomServiceServerInit(void);

/**
 * @brief       Set the priority of non-BLE interrupts to 1
 */
void IRQPriorityInit(void);

/**
 * @brief       Initialize the BLE stack
 */
void BLEStackInit(void);

/**
 * @brief       Disable interrupts and exceptions
 */
void DisableAppInterrupts(void);

/**
 * @brief       Enable interrupts and exceptions
 */
void EnableAppInterrupts(void);

/**
 * @brief       Initialize the GPIOs
 */
void App_GPIO_Config(void);

/**
 * @brief       Initialize the system clocks
 */
void App_Clock_Config(void);

/**
 * @brief       Initialize Memory Retention Sleep Mode configuration used for wake-up
 */
void App_Mem_Ret_Sleep_Init(void);

/**
 * @brief       Initialize No Retention Sleep Mode configuration used for wake-up
 */
void App_No_Ret_Sleep_Init(void);

/**
 * @brief Power Down the FPU Unit
 * @return Returns FPU_Q_ACCEPTED if the FPU power down was successful.
 *                 FPU_Q_DENIED if the FPU power down failed.
 */
uint32_t Power_Down_FPU(void);

/**
 * @brief Initialize swmTrace after wakeup from sleep
 */
void Init_SWMTrace(void);

/* ----------------------------------------------------------------------------
 * Close the 'extern "C"' block
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif    /* ifdef __cplusplus */

#endif    /* APP_INIT_H */
