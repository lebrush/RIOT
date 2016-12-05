/*
 * Copyright (C) 2016 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup   tests
 * @{
 *
 * @file
 * @brief     PN532 board configuration example
 *
 * @author    Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef PN532_PARAMS_H
#define PN532_PARAMS_H

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

static const pn532_params_t pn532_conf[] = {
    {
        //.i2c = TEST_PN532_I2C,

        .reset = GPIO_PIN(PORT_A, 8),
        .irq = GPIO_PIN(PORT_B, 10),
        .nss = GPIO_PIN(PORT_B, 5),
        .spi = SPI_1,
    },
};

#ifdef __cplusplus
}
#endif

#endif /* PN532_PARAMS_H */
/** @} */
