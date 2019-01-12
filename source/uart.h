/*
 * uart.h
 *
 *  Created on: Jan 9, 2019
 *      Author: kevin
 */

#include "fsl_common.h"
#include "fsl_usart.h"

#ifndef UART_H_
#define UART_H_

#ifndef UART_BASE
#define UART_BASE	USART1
#endif
#ifndef UART_IRQ_N
#define UART_IRQ_N	USART1_IRQn
#endif

/*!
 * @brief Enable the USART transmit.
 *
 * This function will enable or disable the USART transmit.
 *
 * @param enable true for enable and false for disable.
 */
static inline void
uart_enable_tx(bool enable)
{
    if (enable)
    {
        /* Make sure the USART module is enabled. */
    	UART_BASE->CFG |= USART_CFG_ENABLE_MASK;
    	UART_BASE->CTL &= ~USART_CTL_TXDIS_MASK;
    }
    else
    {
    	UART_BASE->CTL |= USART_CTL_TXDIS_MASK;
    }
}

/*!
 * @brief Enable the USART receive.
 *
 * This function will enable or disable the USART receive.
 * Note: if the transmit is enabled, the receive will not be disabled.
 * @param enable true for enable and false for disable.
 */
static inline void
uart_enable_rx(bool enable)
{
    if (enable)
    {
        /* Make sure the USART module is enabled. */
    	UART_BASE->CFG |= USART_CFG_ENABLE_MASK;
    }
    else
    {
        /* If the transmit is disabled too. */
        if (UART_BASE->CTL & USART_CTL_TXDIS_MASK)
        {
        	UART_BASE->CFG &= ~USART_CFG_ENABLE_MASK;
        }
    }
}

/**
 *
 */
status_t uart_read(char *byte);

/**
 *
 */
status_t uart_write(char byte);

/**
 *
 */
status_t
uart_init(uint32_t srcClock_Hz);

#endif /* UART_H_ */
