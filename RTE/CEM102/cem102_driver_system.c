/**
 * @file cem102_driver_system.c
 * @brief CEM102 Driver System Support implementation
 *
 * @copyright @parblock
 * Copyright (c) 2022 Semiconductor Components Industries, LLC (d/b/a
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

/* ----------------------------------------------------------------------------
 * Support Configuration Functions
 * --------------------------------------------------------------------------*/

int CEM102_System_IRQConfig(const CEM102_Device *p_cfg, uint16_t irq_cfg)
{
    if (p_cfg->irq < 0)
    {
        /* Flag a general failures, since the IRQ pad isn't connected. */
        return ERRNO_GENERAL_FAILURE;
    }

    return CEM102_Register_Write(p_cfg, SYSCTRL_IRQ_CFG, irq_cfg);
}

int CEM102_System_Command(const CEM102_Device *p_cfg, uint16_t cmd)
{

    return CEM102_Register_Write(p_cfg, SYSCTRL_CMD, cmd);

}

int CEM102_System_Status_Queue(const CEM102_Device *p_cfg)
{
    int result = ERRNO_NO_ERROR;

    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        result = ERRNO_GENERAL_FAILURE;
    }
    else
    {
        result = CEM102_Register_QueueRead(p_cfg, SYS_BUFFER_VIOL_CNT, 3);
    }

    return result;
}

int CEM102_System_Status_Process(uint16_t *data, uint16_t *p_violation_cnt)
{
    uint16_t Sys_Buffer_Viol_Cnt = data[0];
    uint16_t Sys_Buffer_Viol_Status __attribute__((unused)) = data[1];
    uint16_t Sys_Status = data[2];
    *p_violation_cnt = 0;
    uint16_t result = 0;

    if (Sys_Status & DIGITAL_RESET)
    {
        result = DIGITAL_RESET;
    }
    else if (Sys_Status & VDDA_ERR)
    {
        result = VDDA_ERR;
    }
    else if (Sys_Status & SPI_ERR)
    {
        result = SPI_ERR;
    }
    else if (Sys_Status & STATE_ERR)
    {
        result = STATE_ERR;
    }
    else if (Sys_Status & CH1_VIOLATION)
    {
        *p_violation_cnt = (Sys_Buffer_Viol_Cnt & SYS_BUFFER_VIOL_CNT_CH1_VIOLATION_CNT_MASK);

        result = CH1_VIOLATION;
    }
    else if (Sys_Status & CH2_VIOLATION)
    {
        *p_violation_cnt = (Sys_Buffer_Viol_Cnt & SYS_BUFFER_VIOL_CNT_CH2_VIOLATION_CNT_MASK)
                            >> SYS_BUFFER_VIOL_CNT_CH2_VIOLATION_CNT_POS;

        result = CH2_VIOLATION;
    }
    else
    {
        result = (Sys_Status & SYS_STATUS_SYS_STATE_MASK);
    }

    return result;
}

