/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 * Copyright (C) 2015 Hamburg University of Applied Sciences
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    boards_nucleo-f103 Nucleo-F103
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f303 board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f303 board
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Define the nominal CPU core clock in this board
 */
#define F_CPU               CLOCK_CORECLOCK

/**
 * @name Define the UART to be used as stdio and its baudrate
 * @{
 */
#define STDIO               UART_1
#define STDIO_BAUDRATE      (9600U)
#define STDIO_RX_BUFSIZE    (64U)
/** @} */

/**
 * @name LED pin definitions
 * @{
 */
#define LED_GREEN_GPIO      GPIO(PORT_A, 5)
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED_GREEN_ON        (GPIOA->BSRR = (1 << 5))
#define LED_GREEN_OFF       (GPIOA->BRR = (1 << 5))
#define LED_GREEN_TOGGLE    (GPIOA->ODR ^= (1 << 5))
/** @} */

/**
 * @name Buttons
 */
#define BUTTON_USER_GPIO    GPIO(PORT_C, 13)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H_ */
/** @} */
/** @} */
