/*
 * uart.c
 *
 *  Created on: Jan 9, 2019
 *      Author: kevin
 */
#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "uart.h"
#include "fifo.h"

/*! @brief Array to map USART instance number to base address. */
static const uint32_t s_uartBaseAddrs[FSL_FEATURE_SOC_USART_COUNT] = USART_BASE_ADDRS;

/* @brief Array to map USART instance number to CLOCK names */
static const clock_ip_name_t s_uartClock[] = USART_CLOCKS;

/* @brief Array to map USART instance number to RESET names */
static const SYSCON_RSTn_t s_uartReset[] = UART_RSTS_N;

/* FIFO buffer for incoming data */
#define UART_FIFO_DEPTH 	(4096)
FIFO_DEF(ff_uart_rx, UART_FIFO_DEPTH, char, true);

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Get the index corresponding to the USART */
static uint32_t
uart_get_instance(USART_Type *base)
{
    int i;

    for (i = 0; i < FSL_FEATURE_SOC_USART_COUNT; i++)
    {
        if ((uint32_t)base == s_uartBaseAddrs[i])
        {
            return i;
        }
    }

    assert(false);
    return 0U;
}

static status_t
uart_set_baud_rate(uint32_t baudrate_Bps, uint32_t srcClock_Hz)
{
    /* check arguments */
    assert(!((0 == baudrate_Bps) || (0 == srcClock_Hz)));

    uint32_t best_diff = (uint32_t)-1, best_osrval = 0xf, best_brgval = (uint32_t)-1;
    uint32_t diff = 0U, brgval = 0U, osrval = 0U, baudrate = 0U;

    /* Snchronous is enabled, only BRG register is useful. */
    if (UART_BASE->CFG & USART_CFG_SYNCEN_MASK)
    {
        brgval = srcClock_Hz / baudrate_Bps;
        UART_BASE->BRG = brgval - 1;
    }
    else
    {
        for (osrval = best_osrval; osrval >= 8; osrval--)
        {
            brgval = (srcClock_Hz / ((osrval + 1) * baudrate_Bps)) - 1;
            if (brgval > 0xFFFF)
            {
                continue;
            }
            baudrate = srcClock_Hz / ((osrval + 1) * (brgval + 1));
            diff = baudrate_Bps < baudrate ? baudrate - baudrate_Bps : baudrate_Bps - baudrate;
            if (diff < best_diff)
            {
                best_diff = diff;
                best_osrval = osrval;
                best_brgval = brgval;
            }
        }

        /* value over range */
        if (best_brgval > 0xFFFF)
        {
            return kStatus_USART_BaudrateNotSupport;
        }

        UART_BASE->OSR = best_osrval;
        UART_BASE->BRG = best_brgval;
    }

    return kStatus_Success;
}

status_t
uart_read(char *byte)
{
	if (fifo_isEmpty(&ff_uart_rx)) {
		return kStatus_Fail;
	}

	fifo_read(&ff_uart_rx, byte);

	return kStatus_Success;
}

status_t
uart_write(char byte)
{
    /* Wait for TX is ready to transmit new data. */
    while (!(UART_BASE->STAT & USART_STAT_TXRDY_MASK))
    {
    }
    UART_BASE->TXDAT = byte;

    /* Wait to finish transfer */
    while (!(UART_BASE->STAT & USART_STAT_TXIDLE_MASK))
    {
    }

	return kStatus_Success;
}

status_t
uart_init(uint32_t srcClock_Hz)
{
	status_t result = kStatus_Success;
    usart_config_t config;

    uint32_t instance = uart_get_instance(UART_BASE);

    /* Set config settings */
    config.baudRate_Bps = 9600U;
    config.parityMode = kUSART_ParityDisabled;
    config.stopBitCount = kUSART_OneStopBit;
    config.bitCountPerChar = kUSART_8BitsPerChar;
    config.loopback = false;
    config.enableRx = true;
    config.enableTx = true;
    config.syncMode = kUSART_SyncModeDisabled;

    /* Enable usart clock. */
    CLOCK_EnableClock(s_uartClock[instance]);

    /* Reset USART for transfer. */
    RESET_PeripheralReset(s_uartReset[instance]);

    /* Setup configuration and enable USART to configure other register. */
    UART_BASE->CFG = USART_CFG_PARITYSEL(config.parityMode) | USART_CFG_STOPLEN(config.stopBitCount) |
                     USART_CFG_SYNCEN(config.syncMode >> 1) | USART_CFG_DATALEN(config.bitCountPerChar) |
                     USART_CFG_LOOP(config.loopback) | USART_CFG_SYNCMST(config.syncMode) | USART_CFG_ENABLE_MASK;

    if (0U != config.baudRate_Bps)
    {
        /* Setup baudrate */
        result = uart_set_baud_rate(config.baudRate_Bps, srcClock_Hz);
    }

    if (kStatus_Success != result)
    {
        return result;
    }

    /* Init the FIFO buffer */
    fifo_clear(&ff_uart_rx);

    /* Setup the USART transmit and receive. */
    uart_enable_tx(config.enableTx);
    uart_enable_rx(config.enableRx);

    /* Enable RX and overrun interrupts */
    UART_BASE->INTENSET = (kUSART_RxReadyInterruptEnable | kUSART_HardwareOverRunInterruptEnable) & 0x1FF;

    /* Enable interrupt in NVIC. */
    EnableIRQ(UART_IRQ_N);

	return result;
}

#if defined(USART1)
void USART1_IRQHandler(void)
{
	uint8_t rx = UART_BASE->RXDAT ;
	if (!fifo_write(&ff_uart_rx, &rx)) {
		/* FIFO overflow! */
	    LED_RED_ON();
	} else {
		LED_RED_OFF();
	}
}
#else
#error "USART1 must be defined in the BSP!"
#endif
