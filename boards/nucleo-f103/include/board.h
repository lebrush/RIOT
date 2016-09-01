/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f103 Nucleo-F103
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f103 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f103 board
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "board_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Use the 2nd UART for STDIO on this board
 */
#define UART_STDIO_DEV      UART_DEV(1)

/**
 * @name xtimer configuration
 */
#define XTIMER_WIDTH        (16)
#define XTIMER_BACKOFF      5
/** @} */

#define CAN_NUMOF           (1U)
#define CAN_0_EN            1
#define CAN_1_EN            0
#define CAN_IRQ_PRIO        0

#define CAN_0_DEV           CAN1
#define CAN_0_CLOCKEN()     (RCC->APB1ENR |= RCC_APB1ENR_CAN1EN)

#define CAN_0_RX_PIN        GPIO(PORT_B, 8) // remapped
#define CAN_0_TX_PIN        GPIO(PORT_B, 9) // remapped
#define CAN_0_EXT_ID        1 /* use extended identifier (29-bit) */

#define CAN_0_BUS_BRP       35
#define CAN_0_BUS_TS1       5
#define CAN_0_BUS_TS2       6
#define CAN_0_BUS_SJW       3

#define CAN_0_RCV_ISR0      isr_usb_lp_can1_rx0
#define CAN_0_RCV_ISR1      isr_can1_rx1

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
