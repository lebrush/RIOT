/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    driver_periph_can CAN
 * @ingroup     driver_periph
 * @brief       Low-level CAN peripheral driver
 * @{
 *
 * @file
 * @brief       Low-level CAN peripheral driver interface definitions
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef PERIPH_CAN_H
#define PERIPH_CAN_H

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Definition of available CAN devices
 */
typedef enum {
#if CAN_0_EN
	CAN_0 = 0,
#endif
#if CAN_1_EN
	CAN_1 = 1,
#endif
} can_t;

/**
 * @brief Definition of can message abstraction
 */
typedef struct {
	unsigned id;
	unsigned len;
	char rtr; /* 1 or 0 */
	union {
		char      u8[8];
		unsigned u32[2];
	} data;
} can_msg_t;

/**
 * @brief Signature for receive interrupt callback
 *
 * @param[in] can          which CAN device created the interrupt
 * @param[in] msg          received message
 * @param[in] arg          optional argument to put the callback in the right context
 */
typedef void(*can_rx_cb_t)(can_t can, can_msg_t *msg, void *arg);


typedef enum {
	CAN_SEND_PENDING = 0,
	CAN_SEND_OK = 1,
	CAN_ARBITRATION_LOST = 2,
	CAN_TRANSMISSION_ERROR = 3,
	CAN_ABORTED = 4,
	CAN_UNKNOWN = 5,
} can_send_status_t;

/**
 * @brief Initialize a given CAN device
 *
 * The CAN device is initialized with the configuration specified by the
 * CAN_x_y macros
 *
 * @param[in] can           the CAN device to initialize
 * @param[in] rx_cb         receive callback is called when a message is
 *                          received
 * @param[in] arg           optional argument passed to the callback functions
 *
 * @return                  0 on success
 * @return                  -1 invalid can device
 */
int can_init(can_t can, can_rx_cb_t rx_cb, void *arg);

/**
 * @brief Initialize a receive filter
 *
 * Some CAN controllers provide filtering so that only relevant messages are
 * passed to the microcontroller for use processing. This method allows to set
 * up those filters
 *
 * @param[in] can           the CAN device to which the filter applies
 * @return                  0 on success
 */
int can_filter_init(can_t can);

/**
 * @brief Get status of last sent message
 *
 * @param[in] can           the CAN device
 * @param[in] mailbox       the mailbox in which the message was sent
 * @return                  see can_send_status_t
 */
can_send_status_t can_send_status(can_t can, unsigned mailbox);

/**
 * @brief Send a message
 *
 * @param[in] can           the CAN device
 * @param[in] m             the message
 * @param[in] block         block until the message is sent or failed
 * @return                  the used mailbox
 */
int can_send(can_t can, can_msg_t *m, bool block);

/**
 * @brief Test if a message is pending to be received
 *
 * @param[in] can           the CAN device
 * @param[in] fifo          in which fifo to test
 * @return                  1 in case of pending message
 */
int can_message_pending(can_t can, int fifo);

/**
 * @brief Receive a message
 *
 * @param[in] can           the CAN device
 * @param[in] fifo          the fifo where the message is
 * @param[out] m            the resulting message
 * @return                  0 on success
 */
int can_receive(can_t can, unsigned fifo, can_msg_t *m);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CAN_H */
/** @} */
