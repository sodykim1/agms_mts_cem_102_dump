/**
 * @file cem102_driver.c
 * @brief CEM102 Initialization and Base Driver implementation
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

/* ----------------------------------------------------------------------------
 * Version support
 * --------------------------------------------------------------------------*/
#define CEM102_DRIVER_VER_MAJOR         0x00
#define CEM102_DRIVER_VER_MINOR         0x01
#define CEM102_DRIVER_VER_REVISION      0x00

#define CEM102_DRIVER_VER               ((CEM102_DRIVER_VER_MAJOR << 12)   | \
                                         (CEM102_DRIVER_VER_MINOR << 8)    | \
                                         (CEM102_DRIVER_VER_REVISION))

/** CEM102 Driver version */
const unsigned int CEM102_Driver_Version = CEM102_DRIVER_VER;

/* ----------------------------------------------------------------------------
 * Private variables
 * --------------------------------------------------------------------------*/

/* SPI transfer buffers */
volatile CEM102_SPI_FLAG cem102_spi_status = SPI_TRANSFER_IDLE;
static uint8_t spi_tx_buffer[CEM102_MAX_TRANSFER_LENGTH];
static uint8_t spi_rx_buffer[CEM102_MAX_TRANSFER_LENGTH];
static unsigned int spi_rx_len;

/* Data structure for revision 1 of the OTP */
CEM102_TrimStruct_v1 * trim_struct_v1;

/* Data structure for revision 2 of the OTP */
CEM102_TrimStruct * trim_struct;

/* Overall channel gain for each channel */
float channel_gain[4];
/* ----------------------------------------------------------------------------
 * Initialization
 * --------------------------------------------------------------------------*/

int CEM102_Initialize(const CEM102_Device *p_cfg)
{
    int result = ERRNO_NO_ERROR;

    /* Configure the specified SPI interface. DMA support for the SPI interface
     * will be configured when used. */
    Sys_SPI_GPIOConfig(&SPI[p_cfg->spi], SPI_SELECT_MASTER, p_cfg->gpio_cfg,
                       p_cfg->spi_clk, p_cfg->spi_ctrl, p_cfg->spi_io0, p_cfg->spi_io1);
    Sys_SPI_Config(&SPI[p_cfg->spi], CEM102_SPI_CFG | p_cfg->spi_prescale);
    Sys_SPI_TransferConfig(&SPI[p_cfg->spi], CEM102_SPI_CTRL);

    /* Configure the supporting GPIOs if specified, including the clock,
     * interrupt, and pwr_en pin configurations. */
    if (p_cfg->clk >= 0)
    {
        SYS_GPIO_CONFIG(p_cfg->clk, p_cfg->gpio_cfg | GPIO_MODE_STANDBYCLK);
    }
    if (p_cfg->irq >= 0)
    {
        SYS_GPIO_CONFIG(p_cfg->irq, p_cfg->gpio_cfg | GPIO_MODE_INPUT);
        Sys_GPIO_IntConfig(p_cfg->irq_index,
                           (GPIO_DEBOUNCE_DISABLE | GPIO_EVENT_RISING_EDGE | p_cfg->irq),
                           GPIO_DEBOUNCE_SLOWCLK_DIV32, 0);
        NVIC_EnableIRQ(GPIO0_IRQn + p_cfg->irq_index);
    }
    if (p_cfg->pwr_en >= 0)
    {
        /* Enable the pwr_en as an output, ensure the signal is inactive
         * before switching to it if the output can be controlled. */
        Sys_GPIO_Set_High(p_cfg->pwr_en);
        SYS_GPIO_CONFIG(p_cfg->pwr_en, p_cfg->gpio_cfg | GPIO_MODE_GPIO_OUT);
        Sys_GPIO_Set_High(p_cfg->pwr_en);
    }

    if (p_cfg->vdda_gpio == GPIO9)
    {
        result = ERRNO_PARAM_ERROR;
    }
    else if ((uint32_t)(p_cfg->vdda_gpio) < GPIO_PAD_COUNT)
    {
        SYS_GPIO_CONFIG(p_cfg->vdda_gpio,
                        GPIO->CFG[p_cfg->vdda_gpio] &
                        (~(GPIO_CFG_PULL_CTRL_Mask | GPIO_CFG_IO_MODE_Mask)));
    }
    else
    {
        /* No valid GPIO selected. Do not return an error since no
         * GPIO selected is a valid condition */
    }

    /* Disconnect pull settings from GPIO connected to VDDC */
    if (p_cfg->pwr_cfg == HIGH_VOLTAGE)
    {
        if ((uint32_t)(p_cfg->vdda_gpio) < GPIO_PAD_COUNT)
        {
            SYS_GPIO_CONFIG(p_cfg->aout_gpio, GPIO_MODE_DISABLE | GPIO_NO_PULL);
        }
        else
        {
            result = ERRNO_PARAM_ERROR;
        }
    }

    /* When configuring the IO0, IO1 pins eliminate the pull register configuration as
     * these analog signals don't need them.
     */
    if (p_cfg->io0 >= 0)
    {
        SYS_GPIO_CONFIG(p_cfg->io0, (p_cfg->gpio_cfg | p_cfg->io0_type) & ~GPIO_CFG_PULL_CTRL_Mask);
    }
    if (p_cfg->io1 >= 0)
    {
        SYS_GPIO_CONFIG(p_cfg->io1, (p_cfg->gpio_cfg | p_cfg->io1_type) & ~GPIO_CFG_PULL_CTRL_Mask);
    }

    return result;
}

/* ----------------------------------------------------------------------------
 * Transfer support
 * --------------------------------------------------------------------------*/

/**
 * @brief       Initiate transfer using SPI interface (modified from SPI driver,
 *              selecting for DMA)
 * @param[in]   cfg  CEM102 device configuration
 * @param[in]   data_out Data to be sent out
 * @param[in]   data_in  Input buffer
 * @param[in]   num  Length of data to be sent or received
 * @return      Execution status
 */
static int _CEM102_Transfer(const CEM102_Device *p_cfg, const void *data_out, void *data_in, uint32_t num)
{
    if (((data_out == NULL) && (data_in == NULL)) || (num == 0U))
    {
        /* Invalid parameters */
        return ERRNO_PARAM_ERROR;
    }

    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        /* Previous transfer is not completed yet */
        return ERRNO_INTERFACE_BUSY;
    }

    /* Activate the chip (source) select line */
    Sys_SPI_TransferConfig(&SPI[p_cfg->spi], SPI_CS_0);

    /* Configure SPI to RW operation and start transfer (master) */
    Sys_SPI_TransferConfig(&SPI[p_cfg->spi], SPI_ENABLE | SPI_MODE_READ_WRITE | SPI_CS_0);

    /* Updates the transfer status */
    cem102_spi_status = SPI_TX_ACTIVE;

    /* If we have a receive portion of the transfer, configure the specified DMA
     * channel to support the RX portion of the specified SPI transfer */
    if (data_in != NULL)
    {
        /* Since we have an RX component as well, change the status to indicate
         * a bidirectional transfer */
        cem102_spi_status = SPI_TRANSFER_ACTIVE;
    }

    Sys_DMA_ChannelConfig(&DMA[p_cfg->dma[1]],
                          CEM102_DMA_RX_CFG,
                          num,
                          0,
                          (uint32_t)&SPI[p_cfg->spi].RX_DATA,
                          (uint32_t)&spi_rx_buffer);

    /* Clear DMA status register, buffers, and counts. */
    DMA[p_cfg->dma[1]].STATUS = DMA_COMPLETE_INT_CLEAR;
    DMA[p_cfg->dma[1]].CTRL |= DMA_CLEAR_BUFFER | DMA_CLEAR_CNTS;

    /* Enable the interrupts, clearing any pending triggers. */
    NVIC_ClearPendingIRQ(DMA0_IRQn + p_cfg->dma[1]);
    NVIC_EnableIRQ(DMA0_IRQn + p_cfg->dma[1]);

    /* Configure the specified DMA channel to support the TX portion of the
     * specified SPI transfer */
    Sys_DMA_ChannelConfig(&DMA[p_cfg->dma[0]],
                          CEM102_DMA_TX_CFG,
                          num,
                          0,
                          (uint32_t)&spi_tx_buffer,
                          (uint32_t)&SPI[p_cfg->spi].TX_DATA);

    /* Clear DMA status register, buffers, and counts. */
    DMA[p_cfg->dma[0]].STATUS = DMA_COMPLETE_INT_CLEAR;
    DMA[p_cfg->dma[0]].CTRL |= DMA_CLEAR_BUFFER | DMA_CLEAR_CNTS;

    /* Enable the interrupts, clearing any pending triggers. */
    NVIC_ClearPendingIRQ(DMA0_IRQn + p_cfg->dma[0]);
    NVIC_EnableIRQ(DMA0_IRQn + p_cfg->dma[0]);

    /* Start the DMA channels */
    DMA[p_cfg->dma[1]].CTRL = (DMA[p_cfg->dma[1]].CTRL & ~(DMA_CTRL_MODE_ENABLE_Mask | DMA_CTRL_TRIGGER_SOURCE_Mask)) | DMA_ENABLE;
    DMA[p_cfg->dma[0]].CTRL = (DMA[p_cfg->dma[0]].CTRL & ~(DMA_CTRL_MODE_ENABLE_Mask | DMA_CTRL_TRIGGER_SOURCE_Mask)) | DMA_ENABLE;

    return ERRNO_NO_ERROR;
}

int CEM102_Register_QueueWrite(const CEM102_Device *p_cfg, uint8_t addr, unsigned int len)
{
	int result;

    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        return ERRNO_INTERFACE_BUSY;
    }
    if (len >= 64)
    {
        return ERRNO_PARAM_ERROR;
    }

    spi_tx_buffer[0] = addr;
    spi_tx_buffer[1] = (CEM102_CMD_WRITE >> CEM102_CMD_Pos) + (len - 1);

    /* Write the register */
#if 0
    return _CEM102_Transfer(p_cfg, spi_tx_buffer, NULL, (len << 1) + 2);
#else

    	result = _CEM102_Transfer(p_cfg, spi_tx_buffer, NULL, (len << 1) + 2);

        if(ERRNO_NO_ERROR == result)
        {
            /* Block while waiting for the read to complete. */

            while (cem102_spi_status != SPI_TRANSFER_IDLE);
        }

        return result;
#endif

}

int CEM102_Register_BufferWrite(const CEM102_Device *p_cfg, uint8_t addr, uint16_t data[], unsigned int len)
{
    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        return ERRNO_INTERFACE_BUSY;
    }
    if (len >= 64)
    {
        return ERRNO_PARAM_ERROR;
    }

    for (unsigned int i = 0; i < len; i++)
    {
        spi_tx_buffer[i * 2 + 2] = (data[i] >> 8) & 0xFF;
        spi_tx_buffer[i * 2 + 3] = data[i] & 0xFF;
    }

    /* Write the registers */
    return CEM102_Register_QueueWrite(p_cfg, addr, len);
}


int CEM102_Register_Write(const CEM102_Device *p_cfg, uint8_t addr, uint16_t data)
{
    spi_tx_buffer[2] = (data >> 8) & 0xFF;
    spi_tx_buffer[3] = data & 0xFF;

    int result = CEM102_Register_QueueWrite(p_cfg, addr, 1);
    if (result != ERRNO_NO_ERROR)
    {
        return result;
    }

    /* Block while waiting for the write to complete. */
    while (cem102_spi_status != SPI_TRANSFER_IDLE);

    return ERRNO_NO_ERROR;
}

int CEM102_Register_VerifiedWrite(const CEM102_Device *p_cfg, uint8_t addr, uint16_t data)
{
    uint16_t temp;
    int result;
    result = CEM102_Register_Write(p_cfg, addr, data);
    if (result != ERRNO_NO_ERROR)
    {
        return result;
    }
    result = CEM102_Register_Read(p_cfg, addr, &temp);
    if (result != ERRNO_NO_ERROR)
    {
        return result;
    }
    else if (temp != data)
    {
        return ERRNO_GENERAL_FAILURE;
    }
    return ERRNO_NO_ERROR;
}

int CEM102_Register_QueueRead(const CEM102_Device *p_cfg, uint8_t addr, unsigned int len)
{
	int result;

    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        return ERRNO_INTERFACE_BUSY;
    }
    if (len >= 64)
    {
        return ERRNO_PARAM_ERROR;
    }


    spi_tx_buffer[0] = addr;
    spi_tx_buffer[1] = (CEM102_CMD_READ >> CEM102_CMD_Pos) + (len - 1);

    spi_rx_len = len << 1;

#if 0
    /* Write the command and read the 16-bit value */
    return _CEM102_Transfer(p_cfg, spi_tx_buffer, spi_rx_buffer, 2 + spi_rx_len);
#else
    result = _CEM102_Transfer(p_cfg, spi_tx_buffer, spi_rx_buffer, 2 + spi_rx_len);

    if(ERRNO_NO_ERROR == result)    {
        /* Block while waiting for the read to complete. */

        while (cem102_spi_status != SPI_TRANSFER_IDLE);
    }

    return result;
#endif
}

int CEM102_Register_BufferRead(const CEM102_Device *p_cfg, uint16_t data[], unsigned int len)
{
    if (!(cem102_spi_status == SPI_TRANSFER_IDLE) && !(cem102_spi_status == SPI_TRANSFER_COMPLETE))
    {
        return ERRNO_INTERFACE_BUSY;
    }
    if (len >= 64)
    {
        return ERRNO_PARAM_ERROR;
    }

    /* Copy the data read out of the register buffer */
    for (unsigned int i = 0; i < len; i++)
    {
        data[i] = (spi_rx_buffer[i * 2 + 2] << 8) | spi_rx_buffer[i * 2 + 3];
    }

    return ERRNO_NO_ERROR;
}

int CEM102_Register_Read(const CEM102_Device *p_cfg, uint8_t addr, uint16_t *data)
{
    int result;
    result = CEM102_Register_QueueRead(p_cfg, addr, 1);

    if (result != ERRNO_NO_ERROR)
    {
        return result;
    }

    /* Block while waiting for the read to complete. */
    while (cem102_spi_status != SPI_TRANSFER_IDLE);

    *data = (uint16_t) ((spi_rx_buffer[2] << 8) | spi_rx_buffer[3]);
    return ERRNO_NO_ERROR;
}

/* ----------------------------------------------------------------------------
 * Call backs and interrupt handlers
 * --------------------------------------------------------------------------*/

void CEM102_TX_Handler(const CEM102_Device *p_cfg)
{
    uint32_t event = CEM102_EVENT_IDLE;

    /* Check if the device is currently idle */
    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        /* Check if data was lost */
        if ((SPI[p_cfg->spi].STATUS & SPI_OVERRUN_TRUE) ||
            (SPI[p_cfg->spi].STATUS & SPI_UNDERRUN_TRUE))
        {
            SPI[p_cfg->spi].CTRL = CEM102_SPI_CTRL;
            SPI[p_cfg->spi].STATUS = SPI_UNDERRUN_CLEAR | SPI_OVERRUN_CLEAR;
            cem102_spi_status = SPI_TRANSFER_IDLE;
            event = CEM102_EVENT_TX_FAIL;
        }
    }

    /* If an event happened and the application registered a callback */
    if (event != CEM102_EVENT_IDLE)
    {
        CEM102_DAC_SPICallback(p_cfg);
        p_cfg->spi_callback(event);
    }
}

void CEM102_RX_Handler(const CEM102_Device *p_cfg)
{
    uint32_t event = CEM102_EVENT_IDLE;

    /* Check if the device is currently idle */
    if (cem102_spi_status != SPI_TRANSFER_IDLE)
    {
        /* Check if data was lost */
        if ((SPI[p_cfg->spi].STATUS & SPI_OVERRUN_TRUE) ||
            (SPI[p_cfg->spi].STATUS & SPI_UNDERRUN_TRUE))
        {
            SPI[p_cfg->spi].CTRL = CEM102_SPI_CTRL;
            SPI[p_cfg->spi].STATUS = SPI_UNDERRUN_CLEAR | SPI_OVERRUN_CLEAR;
            cem102_spi_status = SPI_TRANSFER_IDLE;
            event = CEM102_EVENT_TRANSFER_FAIL;
        }

        /* Check if DMA RX is complete */
        if ((DMA[p_cfg->dma[1]].STATUS & DMA_COMPLETE_INT_TRUE) != 0)
        {
            if ((cem102_spi_status == SPI_TRANSFER_ACTIVE) || (cem102_spi_status == SPI_TX_ACTIVE))
            {
                /* Disable the SPI, DMA channel and set the transfer to idle. */
                Sys_SPI_TransferConfig(&SPI[p_cfg->spi], CEM102_SPI_CTRL);
                NVIC_DisableIRQ(DMA0_IRQn + p_cfg->dma[0]);
                NVIC_ClearPendingIRQ(DMA0_IRQn + p_cfg->dma[0]);

                DMA[p_cfg->dma[0]].STATUS = DMA_COMPLETE_INT_CLEAR;
                DMA[p_cfg->dma[0]].CTRL |= DMA_CLEAR_BUFFER | DMA_CLEAR_CNTS;

                NVIC_DisableIRQ(DMA0_IRQn + p_cfg->dma[1]);
                NVIC_ClearPendingIRQ(DMA0_IRQn + p_cfg->dma[1]);

                DMA[p_cfg->dma[1]].STATUS = DMA_COMPLETE_INT_CLEAR;
                DMA[p_cfg->dma[1]].CTRL |= DMA_CLEAR_BUFFER | DMA_CLEAR_CNTS;

                cem102_spi_status = SPI_TRANSFER_COMPLETE;
                event = CEM102_EVENT_TRANSFER_COMPLETE;
            }
        }
    }

    /* If an event happened and the application registered a callback */
    if (event != CEM102_EVENT_IDLE)
    {
        CEM102_DAC_SPICallback(p_cfg);
        p_cfg->spi_callback(event);
    }

    if (cem102_spi_status == SPI_TRANSFER_COMPLETE)
    {
        cem102_spi_status = SPI_TRANSFER_IDLE;
    }
}

void CEM102_GPIO_Handler(const CEM102_Device *p_cfg)
{
    uint32_t event;
    if (cem102_spi_status == SPI_TRANSFER_RESET)
    {
        event = CEM102_EVENT_RESET;
    }
    else
    {
        event = CEM102_EVENT_IDLE;
    }
    p_cfg->gpio_callback(event);
}

/* Local definitions for number of relevant ECC bits and the length of the OTP data */
#define ECC_NUM_BITS                    10
#define ECC_DATA_LEN                    16
#define ECC_STRICT_PARITY_BIT           (1 << (ECC_NUM_BITS - 1))

/**
 * @brief       Check the OTP's ECC data is correct (correcting any one bit errors)
 * @param[in]   OTP_data    Data read from the OTP to check that this function is
 *                          checking/correcting
 * @return      Returns ERRNO_NO_ERROR if the ECC parity bits were correct,
 *              ERRNO_ECC_CORRECTED if the ECC parity bits indicate one bit in error,
 *              ERRNO_GENERAL_FAILURE if the ECC detects more than one error.
 */
static int OTP_ECC_Check(uint16_t OTP_data[ECC_DATA_LEN])
{
    const uint16_t weights[ECC_NUM_BITS][ECC_DATA_LEN] =
            {{0xAD5B, 0x56AA, 0x5555, 0xAB55, 0xAAAA, 0xAAAA, 0xAAAA, 0x55AA, 0x5555, 0x5555, 0x5555, 0x5555, 0x5555, 0x5555, 0x5555, 0xD5},
             {0x366D, 0x9B33, 0x9999, 0xCD99, 0xCCCC, 0xCCCC, 0xCCCC, 0x66CC, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x6666, 0x66},
             {0xC78E, 0xE3C3, 0xE1E1, 0xF1E1, 0xF0F0, 0xF0F0, 0xF0F0, 0x78F0, 0x7878, 0x7878, 0x7878, 0x7878, 0x7878, 0x7878, 0x7878, 0x78},
             {0x7F0, 0x3FC, 0x1FE, 0x1FE, 0xFF, 0xFF, 0xFF, 0x80FF, 0x807F, 0x807F, 0x807F, 0x807F, 0x807F, 0x807F, 0x807F, 0x7F},
             {0xF800, 0x3FF, 0xFE00, 0x1FF, 0xFF00, 0xFF, 0xFF00, 0xFF, 0xFF80, 0x7F, 0xFF80, 0x7F, 0xFF80, 0x7F, 0xFF80, 0x7F},
             {0x0, 0xFC00, 0xFFFF, 0x1FF, 0x0, 0xFF00, 0xFFFF, 0xFF, 0x0, 0xFF80, 0xFFFF, 0x7F, 0x0, 0xFF80, 0xFFFF, 0x7F},
             {0x0, 0x0, 0x0, 0xFE00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFF, 0x0, 0x0, 0x0, 0xFF80, 0xFFFF, 0xFFFF, 0xFFFF, 0x7F},
             {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF00, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0x7F},
             {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80},
             {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFF}};

    uint16_t p[ECC_NUM_BITS] = {[0 ... (ECC_NUM_BITS - 1)] = 0};

    int parityBitNr;
    int wordNr;
    int i;
    uint16_t temp;
    int ret_val;

    for (parityBitNr = 0; parityBitNr < ECC_NUM_BITS; parityBitNr++)
    {
        /* Exclude the existing parity bits from the OTP read */
        for (wordNr = 0; wordNr < (ECC_DATA_LEN - 1); wordNr++)
        {
            /* Add the weighted data for each input word */
            p[parityBitNr] = p[parityBitNr] ^ (OTP_data[wordNr] & weights[parityBitNr][wordNr]);
        }

        /* Calculate the parity of the remaining 16 bit word (1-bit value) */
        temp = 0;
        for (i = 0; i < 16; i++)
        {
            temp = temp ^ (p[parityBitNr] / (1 << i));
        }
        p[parityBitNr] = temp % 2;
    }

    /* For the extra parity bit, add all of the other parity bits */
    for (parityBitNr = 0; parityBitNr < (ECC_NUM_BITS - 1); parityBitNr++)
    {
        p[ECC_NUM_BITS - 1] = p[ECC_NUM_BITS - 1] + p[parityBitNr];
    }
    p[ECC_NUM_BITS - 1] = p[ECC_NUM_BITS - 1] % 2;


    /* Collect the ECC parity bits into one word */
    temp = 0;
    for (parityBitNr = 0; parityBitNr < ECC_NUM_BITS; parityBitNr++)
    {
        temp = temp + (p[parityBitNr] * (1 << parityBitNr));
    }

    /*
     * Check the parity bits; since we've XORed with the previous parity bits
     * to this calculation:
     *  - If all the bits are cleared, there are no errors detected.
     *  - If there is an even number of parity bits set, the overall parity
     *    indicates an even number of errors (which cannot be corrected)
     *  - If there is an odd number of parity bits set, we can use the
     *    other parity bits to identify which bit to correct.
     */
    temp = temp ^ (OTP_data[ECC_DATA_LEN - 1] >> (ECC_DATA_LEN - ECC_NUM_BITS));
    if (temp == 0)
    {
        ret_val = ERRNO_NO_ERROR;
    }
    else
    {
        parityBitNr = 0;
        for (i = 0; i < ECC_NUM_BITS; i++)
        {
            if ((temp & (1 << i)) != 0)
            {
                parityBitNr++;
            }
        }

        if (parityBitNr % 2 == 0)
        {
            ret_val = ERRNO_GENERAL_FAILURE;
        }
        else
        {
            if (temp == ECC_STRICT_PARITY_BIT)
            {
                /* Error was on the strict parity bit */
                OTP_data[ECC_DATA_LEN - 1] ^= ECC_STRICT_PARITY_BIT;
            }
            else
            {
                /* Error was on any other bit */
                temp &= (ECC_STRICT_PARITY_BIT - 1);

                /* Figure out which data bit needs to be corrected
                 * by removing the parity bits from the count */
                for (i = ECC_NUM_BITS - 2; i >= 0; i--)
                {
                    if (temp > (1 << i)) temp--;
                }

                /* Convert to a zero indexed bit number */
                temp--;

                /* Flip the specified bit */
                OTP_data[temp / 16] ^= 1 << (temp % 16);
            }
            ret_val = ERRNO_ECC_CORRECTED;
        }
    }
    return ret_val;
}

uint16_t CEM102_OTP_GetVersion(void)
{
    uint16_t otp_ver = 0;

    if (trim_struct_v1 != NULL)
    {
        otp_ver = trim_struct_v1->device_info[1];
    }
    else if (trim_struct != NULL)
    {
        otp_ver = trim_struct->device_info[1];
    }
    otp_ver = (otp_ver & CEM102_OTP_VERSION_MASK) >>
               CEM102_OTP_VERSION_POS;
    return otp_ver;
}

int CEM102_OTP_Read(const CEM102_Device *p_cfg, CEM102_TrimStruct *p_trim_struct, uint32_t cem102_sysclk_freq, uint32_t timeout)
{
    int result;
    uint16_t read_data;
    uint32_t timeout_temp = timeout;

    /* Configure to read OTP */
    result = CEM102_Register_Write(p_cfg, SYSCTRL_CMD, BUFFER_RESET_CMD);
    if (cem102_sysclk_freq > 64000)
    {
        result |= CEM102_Register_Write(p_cfg, SYSCTRL_OTP_CFG, (OTP_PWR_ENABLE | OTP_OSC_DISABLE | OTP_READ_CONSTANT));
    }
    else
    {
        result |= CEM102_Register_Write(p_cfg, SYSCTRL_OTP_CFG, (OTP_PWR_ENABLE | OTP_OSC_DISABLE | OTP_READ_SYS_CLK));
    }

    result |= CEM102_Register_Write(p_cfg, SYSCTRL_CMD, OTP_READ_CMD);

    /* Wait until the State Machine is not in OTP Read State */
    result |= CEM102_Register_Read(p_cfg, SYS_STATUS, &read_data);
    while (((read_data & (SYS_STATUS_SYS_STATE_MASK)) != OTP_READ_STATE) && timeout)
    {
        result |= CEM102_Register_Read(p_cfg, SYS_STATUS, &read_data);

        if (!(--timeout_temp))
        {
            result = ERRNO_TIMEOUT;
        }
    }

    /* Wait until the OTP Controller finishes it's execution */
    timeout_temp = timeout;
    result |= CEM102_Register_Read(p_cfg, SYS_STATUS, &read_data);
    while (((read_data & (1 << SYS_STATUS_OTP_STATUS_POS)) != OTP_LOAD) && timeout)
    {
        result |= CEM102_Register_Read(p_cfg, SYS_STATUS, &read_data);

        if (!(--timeout_temp))
        {
            result = ERRNO_TIMEOUT;
        }
    }

    /* Read data from the Buffer Registers */
    uint16_t *buffer_data_p = (uint16_t*)p_trim_struct;
    for (uint16_t addr = SYS_BUFFER_WORD0; addr <= SYS_BUFFER_WORD15; addr++)
    {
        result |= CEM102_Register_Read(p_cfg, addr, buffer_data_p++);
    }

    /* Clear OTP Status Flag */
    result |= CEM102_Register_Write(p_cfg, SYSCTRL_CMD, OTP_STATUS_CLR_CMD);

    /* Disable OTP */
    result |= CEM102_Register_Write(p_cfg, SYSCTRL_OTP_CFG, (OTP_PWR_DISABLE | OTP_OSC_DISABLE));

    /* Check the ECC, correcting any one bit errors encountered */
    if (result == ERRNO_NO_ERROR)
    {
        result = OTP_ECC_Check((uint16_t *)p_trim_struct);

        /* Set internal trim structure pointer equal to the external trim structure
         * passed in. These OTP values are used elsewhere in the driver.
         * Base structure type based on device_info. OTP rev starts at bit 10,
         * so 0x2 << 10 == 2000 */

        if ((result == ERRNO_NO_ERROR) || (result == ERRNO_ECC_CORRECTED))
        {
            if ((p_trim_struct->device_info[1] & 0xFC00) < 2000)
            {
                trim_struct_v1 = (CEM102_TrimStruct_v1 *)p_trim_struct;

                /* Gain information for channel 2 is not populated to save
                 * testing time for OTP-v1. Copying the gain information to
                 * channel 2 from channel 1, while maintaining the encoding
                 * of data for channel 2, as given in OTP V1 specification.
                 */
                trim_struct_v1->adc[2] = ((trim_struct_v1->adc[2] & 0xFF) |
                                    ((trim_struct_v1->adc[0] & 0xFF0) << 4));
                trim_struct_v1->adc[3] = ((trim_struct_v1->adc[3] & 0xFFF0) |
                                    ((trim_struct_v1->adc[0] & 0xF000) >> 12));
                trim_struct = NULL;
            }
            else
            {
                trim_struct = p_trim_struct;
                trim_struct_v1 = NULL;
            }
        }
    }

    return result;
}

/* CEM102 Driver Control Block */
DRIVER_CEM102_t Driver_CEM102 =
{
    CEM102_Initialize,
    CEM102_Register_QueueWrite,
    CEM102_Register_BufferWrite,
    CEM102_Register_Write,
    CEM102_Register_VerifiedWrite,
    CEM102_Register_QueueRead,
    CEM102_Register_BufferRead,
    CEM102_Register_Read,
    CEM102_TX_Handler,
    CEM102_RX_Handler,
    CEM102_GPIO_Handler,
    CEM102_OTP_Read,
    CEM102_Power_Reset,
    CEM102_Power_Init,
    CEM102_Power_VDDADischarge,
    CEM102_Power_DeviceTrim,
    CEM102_Power_VDDCCharge,
    CEM102_SAR_Init,
    CEM102_SAR_StartConversion,
    CEM102_VBAT_OK,
    CEM102_VCC_OK,
    CEM102_VOLTAGE_OK,
    CEM102_VBAT_VDDA_MonitorStart,
    CEM102_Calibrate_VCC,
    CEM102_Calculate_VBAT,
    CEM102_System_IRQConfig,
    CEM102_System_Command,
    CEM102_System_Status_Queue,
    CEM102_System_Status_Process,
    CEM102_Signal_ClockConfig,
    CEM102_Signal_TIAConfig,
    CEM102_Signal_SwitchConfig,
    CEM102_Signal_RE_ATBus_Config,
    CEM102_Signal_BufferConfig,
    CEM102_Signal_WEDACConfig,
    CEM102_Signal_REDACConfig,
    CEM102_Signal_ADCConfig,
    CEM102_Signal_DirectADCConfig,
    CEM102_Signal_StartMeasurement,
    CEM102_Signal_ProcessADCSingle,
    CEM102_Signal_ProcessADCDouble,
    CEM102_Signal_MeasureReference,
    CEM102_Signal_CalibrateChannel,
    CEM102_Signal_CalibrateDACVoltage,
    CEM102_Signal_CalculateChannelGain,
    CEM102_Signal_CalculateADCGain,
    CEM102_Signal_CalculateCurrent,
    CEM102_Signal_CalculateVoltage,
    CEM102_Signal_CalculateDACOffset,
    CEM102_Signal_GetADCGain,
    CEM102_Signal_CalculateDACGain,
    CEM102_DAC_Calibrate,
    CEM102_DAC_CalibrateWithADC,
};

