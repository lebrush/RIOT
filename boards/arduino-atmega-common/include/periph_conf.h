/*
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *               2016 Laurent Navet <laurent.navet@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_arduino-atmega-common
 * @{
 *
 * @file
 * @brief       Common configuration of MCU periphery for Arduino Atmega boards
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Laurent Navet <laurent.navet@gmail.com>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Clock configuration
 * @{
 */
#define CLOCK_CORECLOCK     (16000000L)
/** @} */

/**
 * @name    Timer configuration
 *
 * arduino uno and arduino duemilanove:
 * The timer driver only supports the 16-bit timer (Timer1)
 * so this is the only one we can use here.
 *
 * arduino mega2560:
 * The timer driver only supports the four 16-bit timers (Timer1, Timer3,
 * Timer4, Timer5), so those are the only onces we can use here.
 *
 * @{
 */
#ifdef CPU_ATMEGA328P
#define TIMER_NUMOF         (2U)

#define TIMER_0             MEGA_TIMER1
#define TIMER_0_MASK        &TIMSK1
#define TIMER_0_FLAG        &TIFR1
#define TIMER_0_ISRA        TIMER1_COMPA_vect
#define TIMER_0_ISRB        TIMER1_COMPB_vect
#endif

#ifdef CPU_ATMEGA2560
#define TIMER_NUMOF         (2U)

#define TIMER_0             MEGA_TIMER1
#define TIMER_0_MASK        &TIMSK1
#define TIMER_0_FLAG        &TIFR1
#define TIMER_0_ISRA        TIMER1_COMPA_vect
#define TIMER_0_ISRB        TIMER1_COMPB_vect
#define TIMER_0_ISRC        TIMER1_COMPC_vect

#define TIMER_1             MEGA_TIMER4
#define TIMER_1_MASK        &TIMSK4
#define TIMER_1_FLAG        &TIFR4
#define TIMER_1_ISRA        TIMER4_COMPA_vect
#define TIMER_1_ISRB        TIMER4_COMPB_vect
#define TIMER_1_ISRC        TIMER4_COMPC_vect
#endif
/** @} */

/**
 * @name    UART configuration
 *
 * The UART devices have fixed pin mappings, so all we need to do, is to specify
 * which devices we would like to use and their corresponding RX interrupts. See
 * the reference manual for the fixed pin mapping.
 *
 * @{
 */
#ifdef CPU_ATMEGA328P
#define UART_NUMOF          (1U)

#define UART_0              MEGA_UART0
#define UART_0_ISR          USART_RX_vect
#endif

#ifdef CPU_ATMEGA2560
#define UART_NUMOF          (4U)

#define UART_0              MEGA_UART0
#define UART_0_ISR          USART0_RX_vect

#define UART_1              MEGA_UART1
#define UART_1_ISR          USART1_RX_vect

#define UART_2              MEGA_UART2
#define UART_2_ISR          USART2_RX_vect

#define UART_3              MEGA_UART3
#define UART_3_ISR          USART3_RX_vect
#endif
/** @} */

/**
 * @name    SPI configuration
 *
 * The atmega2560/atmega1281/atmega328p has only one hardware SPI with fixed pin
 * configuration, so all we can do here, is to enable or disable it...
 *
 * The fixed pins for arduino uno and duemilanove (atmega328p) are:
 * MOSI - PB3 (Arduino pin 11)
 * MISO - PB4 (Arduino pin 12)
 * SCK  - PB5 (Arduino pin 13)
 * SS   - PB2 (Arduino pin 10) -> this pin is configured as output, but not used
 *
 * The fixed pins for arduino-mega2560 and atmega1281 are:
 * MOSI - PB2 (Arduino pin 51)
 * MISO - PB3 (Arduino pin 50)
 * SCK  - PB1 (Arduino pin 52)
 * SS   - PB0 (Arduino pin 53) -> this pin is configured as output, but not used
 *
 * The SS pin must be configured as output for the SPI device to work as
 * master correctly, though we do not use it for now (as we handle the chip
 * select externally for now)
 *
 * @{
 */
#define SPI_NUMOF           1           /* set to 0 to disable SPI */

#ifdef CPU_ATMEGA328P
#define MEGA_PRR            PRR         /* Power Reduction Register is PRR */
#endif

#ifdef CPU_ATMEGA2560
#define MEGA_PRR            PRR0        /* Power Reduction Register is PRR0 */
#endif
/** @} */

/**
 * @brief    I2C configuration
 */
#define I2C_NUMOF           1

/*
 * @name   PWM configuration
 *
 * The current implementation supports only 8-bit timers for PWM generation.
 * These timers are typically timer 0 and timer 2 in Atmega2560/1281/328p.
 *
 * Setting the first channel to GPIO_UNDEF allows multiple resolutions for the
 * PWM channel. Otherwise the resolution is fixed to 256, allowing duty cycle
 * values ranging from 0 to 255.
 *
 * @{
 */
#if defined(CPU_ATMEGA328P)
#define PWM_PINS_CH0 { GPIO_PIN(PORT_D, 6), GPIO_PIN(PORT_D, 5) }
#define PWM_PINS_CH1 { GPIO_PIN(PORT_B, 3), GPIO_PIN(PORT_D, 3) }

#elif defined(CPU_ATMEGA2560)
#define PWM_PINS_CH0 { GPIO_PIN(PORT_B, 7), GPIO_PIN(PORT_G, 5) }
#define PWM_PINS_CH1 { GPIO_PIN(PORT_B, 4), GPIO_PIN(PORT_H, 6) }

#elif defined(CPU_ATMEGA1281)
#define PWM_PINS_CH0 { GPIO_PIN(PORT_B, 7), GPIO_PIN(PORT_G, 5) }
#define PWM_PINS_CH1 { GPIO_PIN(PORT_B, 4), GPIO_UNDEF }
#endif

static const pwm_conf_t pwm_conf[] = {
    {
        .dev = MINI_TIMER0,
        .pin_ch = PWM_PINS_CH0,
        .div = MINI_TIMER0_DIV,
        .prr_bit = MINI_TIMER0_PRR
    },
    {
        .dev = MINI_TIMER2,
        .pin_ch = PWM_PINS_CH1,
        .div = MINI_TIMER2_DIV,
        .prr_bit = MINI_TIMER2_PRR
    }
};
#endif
#define PWM_NUMOF (sizeof(pwm_conf) / sizeof(pwm_conf[0]))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
