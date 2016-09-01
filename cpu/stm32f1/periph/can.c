/*
 * Copyright (C) 2015 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32f1
 * @{
 *
 * @file
 * @brief       Low-level CAN driver implementation
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
 * @}
 */

#include <assert.h>
#include <stdio.h>

#include "bitarithm.h"
#include "stm32f10x.h"
#include "periph/gpio.h"
#include "periph/can.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

typedef struct {
	can_rx_cb_t rx_cb;
	void *arg;
} can_conf_t;

static can_conf_t config[CAN_NUMOF];

int can_init(can_t can, can_rx_cb_t rx_cb, void *arg)
{
	CAN_TypeDef *dev;
	unsigned rx_pin, tx_pin, btr;

	switch (can) {
#if CAN_0_EN
		case CAN_0:
			dev = CAN_0_DEV;
			rx_pin = CAN_0_RX_PIN;
			tx_pin = CAN_0_TX_PIN;
			btr = CAN_0_BUS_BRP | (CAN_0_BUS_TS1 << 16) | (CAN_0_BUS_TS2 << 20)
                    | (CAN_0_BUS_SJW << 24);
			CAN_0_CLKEN();
			break;
#endif
#if CAN_1_EN
		case CAN_1:
			dev = CAN_1_DEV;
			rx_pin = CAN_1_RX_PIN;
			tx_pin = CAN_1_TX_PIN;
			btr = CAN_1_BUS_BRP | (CAN_1_BUS_TS1 << 16) | (CAN_1_BUS_TS2 << 20)
			                    | (CAN_1_BUS_SJW << 24);
			CAN_1_CLKEN();
			break;
#endif
		default:
			return -1;
	}

	/* enable clock for AF */
	RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;

	/* enable RX pin */
	gpio_init(rx_pin, GPIO_DIR_IN, GPIO_PULLUP);

	/* enable TX pin */
	gpio_init_af(tx_pin, GPIO_AF_OUT_PP);

	/* Reset the peripheral */
	RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN1RST;

	/* Exit sleep mode */
	dev->MCR &= ~CAN_MCR_SLEEP;

	/* Request init */
	dev->MCR |= CAN_MCR_INRQ;
	while((dev->MSR & CAN_MSR_INAK) == 0);

	/* No time triggered */
	/* No auto wakeup */
	/* No receive fifo lockdown */
	/* work during debug */
	/* auto rtx (standard) */
    dev->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_NART | CAN_MCR_RFLM
            | CAN_MCR_DEBUG | CAN_MCR_ABOM | CAN_MCR_TXFP);

	/* handle bus off automatically */
	dev->MCR |= (CAN_MCR_AWUM | CAN_MCR_ABOM | CAN_MCR_TXFP);
	//dev->MCR |= CAN_MCR_NART; // xxx remove

	/* Bit timing */
	dev->BTR = btr;

	/* Enable debug */
	/* xxx */
	//dev->BTR |= (CAN_BTR_LBKM | CAN_BTR_SILM);

	/* Interrupt configuration */
	dev->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1);

	//NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, CAN_IRQ_PRIO);
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

	//NVIC_SetPriority(CAN1_RX1_IRQn, CAN_IRQ_PRIO);
	NVIC_EnableIRQ(CAN1_RX1_IRQn);

	/* Leave the initialization request */
	dev->MCR &= ~CAN_MCR_INRQ;
	while(dev->MSR & CAN_MSR_INAK);

	config[can].rx_cb = rx_cb;
	config[can].arg = arg;

	return 0;
}

static inline CAN_TypeDef *get_dev(can_t can)
{
	CAN_TypeDef *dev = NULL;
	switch (can) {
#if CAN_0_EN
		case CAN_0:
			dev = CAN_0_DEV;
			break;
#endif
#if CAN_1_EN
		case CAN_1:
			dev = CAN_1_DEV;
			break;
#endif
	}
	return dev;
}

int can_filter_init(can_t can, uint32_t mask)
{

	unsigned int filter_nummer, filter_mask;
	CAN_TypeDef *dev = get_dev(can);

	filter_nummer = 0;
	filter_mask = (1 << filter_nummer);
	assert(filter_nummer < 28);

	/* initialization mode */
	dev->FMR |= CAN_FMR_FINIT;

	/* disable filter */
	dev->FA1R &= ~filter_mask;

#if !CAN_0_EXT_ID
#error Only implemented for extended IDs
#endif /* !CAN_0_EXT_ID */

	/* enable 32-bit filter */
	dev->FS1R |= filter_mask;

	/* filter id */
	dev->sFilterRegister[filter_nummer].FR1 = (mask & 0xffff);
	dev->sFilterRegister[filter_nummer].FR2 = (mask >> 16) & 0xffff;

	/* filter mode: mask */
	dev->FM1R &= ~filter_mask;

	/* filter assign fifo 0 */
	dev->FFA1R &= ~filter_mask;

	/* enable filter */
	dev->FA1R |= filter_mask;

	/* done */
	dev->FMR &= ~CAN_FMR_FINIT;

	return 0;
}

can_send_status_t can_send_status(can_t can, unsigned mailbox)
{
	CAN_TypeDef *dev = get_dev(can);
	unsigned reg = (dev->TSR >> (mailbox << 3)) & 0x8F;

	if (!(reg & CAN_TSR_RQCP0)) {
		return CAN_SEND_PENDING;

	} else if(reg & CAN_TSR_TXOK0) {
		return CAN_SEND_OK;

	} else if(reg & CAN_TSR_ALST0) {
		return CAN_ARBITRATION_LOST;

	} else if(reg & CAN_TSR_TERR0) {
		return CAN_TRANSMISSION_ERROR;

	} else if(reg & CAN_TSR_ABRQ0) {
		return CAN_ABORTED;
	}

	return CAN_UNKNOWN;
}

int can_send(can_t can, can_msg_t *m, bool block)
{
	CAN_TypeDef *dev = get_dev(can);

	/* select an empty mailbox */
	unsigned mailbox = ((dev->TSR & CAN_TSR_TME) >> 26);
	if (mailbox == 0) {
		return -1;
	}

	mailbox = bitarithm_lsb(mailbox);

	/* configure mailbox identifier */
	dev->sTxMailBox[mailbox].TIR &= ~(CAN_TI0R_TXRQ | CAN_TI0R_IDE);

	if (CAN_0_EXT_ID) {
		dev->sTxMailBox[mailbox].TIR = (m->id << 3) | CAN_TI0R_IDE | (m->rtr << 1);
	} else {
		dev->sTxMailBox[mailbox].TIR = (m->id << 21) | (m->rtr << 1);
	}

	/* length */
	dev->sTxMailBox[mailbox].TDTR &= ~(CAN_TDT0R_DLC);
	dev->sTxMailBox[mailbox].TDTR |= m->len;

	/* data */
	dev->sTxMailBox[mailbox].TDLR = m->data.u32[0];
	dev->sTxMailBox[mailbox].TDHR = m->data.u32[1];

	/* send */
	dev->sTxMailBox[mailbox].TIR |= CAN_TI0R_TXRQ;

	if (block) {
		unsigned reg;
		do {
			reg = (dev->TSR >> (mailbox << 3)) & 0x8F;
		} while((!(reg & CAN_TSR_RQCP0)));
	}

	return mailbox;
}

int can_message_pending(can_t can, int fifo)
{
	CAN_TypeDef *dev = get_dev(can);
	unsigned reg;
	if (fifo == 0) {
		reg = dev->RF0R;
	} else {
		reg = dev->RF1R;
	}

	return (reg & CAN_RF0R_FMP0);
}

int can_receive(can_t can, unsigned fifo, can_msg_t *m)
{
	CAN_TypeDef *dev = get_dev(can);
	CAN_FIFOMailBox_TypeDef *mb = &dev->sFIFOMailBox[fifo];

	unsigned tmp = mb->RIR;
	if (mb->RIR & CAN_RI0R_IDE) {
		m->id = (mb->RIR >> 3);
	} else {
		m->id = (mb->RIR >> 21);
	}

	m->rtr = (tmp & CAN_RI0R_RTR) >> 1;
	m->len = (mb->RDTR & CAN_RDT0R_DLC);

	m->data.u32[0] = mb->RDLR;
	m->data.u32[1] = mb->RDHR;

	if (fifo == 0) {
		dev->RF0R |= CAN_RF0R_RFOM0;
	} else {
		dev->RF1R |= CAN_RF1R_RFOM1;
	}

	return 0;
}

static inline void can_irq_handler(can_t can, CAN_TypeDef *dev, unsigned fifo)
{
	static can_msg_t can_m;
	can_receive(can, fifo, &can_m);
    config[can].rx_cb(can, &can_m, config[can].arg);

	if (sched_context_switch_request) {
        thread_yield();
    }
}

#if CAN_0_EN
void CAN_0_RCV_ISR0(void)
{
	can_irq_handler(CAN_0, CAN_0_DEV, 0);
}

void CAN_0_RCV_ISR1(void)
{
	can_irq_handler(CAN_0, CAN_0_DEV, 1);
}
#endif /* CAN_0_EN */

#if CAN_1_EN
void CAN_1_RCV_ISR0(void)
{
	can_irq_handler(CAN_1, CAN_1_DEV, 0);
}

void CAN_1_RCV_ISR1(void)
{
	can_irq_handler(CAN_1, CAN_1_DEV, 1);
}
#endif /* CAN_1_EN */
