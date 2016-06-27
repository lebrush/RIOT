/*
 * Copyright (C) 2016 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup  drivers_pn532
 *
 * @{
 * @file
 * @brief   PN532 driver
 * @author  Víctor Ariño <victor.arino@triagnosys.com>
 */

#ifndef NFC_READER_INCLUDE_PN532_H_
#define NFC_READER_INCLUDE_PN532_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mutex.h"
#include "periph/i2c.h"
#include "periph/gpio.h"
#include <stdint.h>

/**
 * @brief Data structure with the configuration parameters
 */
typedef struct {
    i2c_t i2c;                  /** I2C device */
    gpio_t reset;               /** Reset pin */
    gpio_t irq;                 /** Interrupt pin */
} pn532_params_t;


/**
 * @brief Device descriptor for the PN532
 */
typedef struct {
    const pn532_params_t *conf;
    mutex_t trap;
} pn532_t;

/**
 * @brief Internal buffer size
 *
 * A small buffer size is enough for most applications, however if large NDEF
 * files are to be written this size shall be increased. Otherwise the files
 * can be written in chunks.
 */
#ifndef PN532_BUFFER_LEN
#define PN532_BUFFER_LEN     64
#endif

/**
 * @brief Helpers to extract firmware information from word
 * @{
 */
#define PN532_IC_VERSION(fwver)  ((fwver >> 24) & 0xff)
#define PN532_FW_VERSION(fwver)  ((fwver >> 16) & 0xff)
#define PN532_FW_REVISION(fwver) ((fwver >>  8) & 0xff)
#define PN532_FW_FEATURES(fwver) ((fwver) & 0xff)
/** @} */

/**
 * @brief Possible SAM configurations
 */
typedef enum {
    PN532_SAM_NORMAL = 1,
    PN532_SAM_VIRTUAL,
    PN532_SAM_WIRED,
    PN532_SAM_DUAL
} pn532_sam_conf_mode_t;

/**
 * @brief PN532 supported targets
 */
typedef enum {
    PN532_BR_106_ISO_14443_A = 0,
    PN532_BR_212_FELICA,
    PN532_BR_424_FELICA,
    PN532_BR_106_ISO_14443_B,
    PN532_BR_106_JEWEL
} pn532_target_t;

/**
 * @brief ISO14443A Card types
 */
typedef enum {
    ISO14443A_UNKNOWN,
    ISO14443A_MIFARE,
    ISO14443A_TYPE4
} nfc_iso14443a_type_t;

/**
 * @brief ISO14443A tag description
 */
typedef struct {
    char sns_res;
    char sel_res;
    char target;
    char auth;
    char id_len;
    nfc_iso14443a_type_t type;
    char id[8];
} nfc_iso14443a_t;

/**
 * @brief Mifare keys
 */
typedef enum {
    PN532_MIFARE_KEY_A = 0x60,
    PN532_MIFARE_KEY_B = 0x61
} pn532_mifare_key_t;

/**
 * @brief Obtain Tag 4 data length from buffer
 *
 * This is useful in case the length has been read and one intents to read the
 * data.
 */
#define PN532_ISO14443A_4_LEN_FROM_BUFFER(b) ((b[0] << 8) | b[1])

/**
 * @brief Hard reset the chipset
 *
 * The chipset is reset by toggling the reset pins
 *
 * @param[in]  dev          target device
 *
 */
void pn532_reset(pn532_t *dev);

/**
 * @brief Initialize the module and peripherals
 *
 * This is the first method to be called in order to interact with the pn532.
 * It configures the GPIOs and the i2c interface.
 *
 *  @param[in]  dev         target device
 *  @param[in]  params      configuration parameters
 *
 * @return                  0 on success
 */
int pn532_init(pn532_t *dev, const pn532_params_t *params);

/**
 * @brief Get the firmware version of the pn532
 *
 * The firmware version returned is a 4 byte long value:
 *  - ic version,
 *  - fw version,
 *  - fw revision
 *  - feature support
 *
 * @param[in]  dev          target device
 * @param[out] fw_ver       encoded firmware version
 *
 * @return                  0 on success
 */
int pn532_fw_version(pn532_t *dev, uint32_t *fw_ver);

/**
 * @brief Read register of the pn532
 *
 * Refer to the datasheet for a comprehensive list of registers and meanings.
 * For SFR registers the high byte must be set to 0xff.
 *
 * @param[in]  dev          target device
 * @param[out] out          value of the register
 * @param[in]  addr         address of the register to read
 *
 * @return                  0 on success
 */
int pn532_read_reg(pn532_t *dev, char *out, unsigned addr);

/**
 * @brief Write register of the pn532
 *
 * Refer to the datasheet for a comprehensive list of registers and meanings.
 *
 * @param[in]  dev          target device
 * @param[in]  addr         address of the register to read
 * @param[in]  val          value to write in the register
 *
 * @return                  0 on success
 */
int pn532_write_reg(pn532_t *dev, unsigned addr, char val);

/**
 * @brief Read status of the GPIOs
 *
 * 3 byte encoded values of p3, p7 and l0l1. Bitwise information as follows:
 *  - p3:   p30, p31, p32, p33, p34, p35, 0, 0
 *  - p7:     0, p71, p72,   0,   0,   0, 0, 0
 *  - l0l1:  l0, l1
 *
 * @param[in]  dev          target device
 * @param[out] gpio         status of the gpio ports
 *
 * @return                  0 on success
 */
int pn532_read_gpio(pn532_t *dev, uint32_t *gpio);

/**
 * @brief Write new state for the GPIOs
 *
 * Bitwise information should be provided as follows:
 *  - p3:   p30, p31, p32, p33, p34, p35, 0, 0
 *  - p7:     0, p71, p72,   0,   0,   0, 0, 0
 *
 * @param[in]  dev          target device
 * @param[in]  p3           new state for p3 (bitwise encoded)
 * @param[in]  p7           new state for p7 (bitwise encoded)
 *
 * @return                  0 on success
 */
int pn532_write_gpio(pn532_t *dev, unsigned p3, unsigned p7);

/**
 * @brief Set new settings for the Security Access Module
 *
 * @param[in]  dev          target device
 * @param[in]  mode         new mode for the SAM
 * @param[in]  timeout      timeout for Virtual Card mode with LSB of 50ms.
 *                          (0 = infinite and 0xFF = 12.75s)
 *
 * @return                  0 on success
 */
int pn532_sam_configuration(pn532_t *dev, pn532_sam_conf_mode_t mode, unsigned timeout);

/**
 * @brief Get (max) passive targets of the given type
 *
 * This method blocks until a target is detected.
 *
 * @param[in]  dev          target device
 * @param[in]  target       type of target
 * @param[in]  max          maximum number of targets to detect (1 or 2)
 * @param[in]  max_retries  maximum number of attempts to activate a target
 *                          (0xff blocks indefinitely)
 *
 * @return                  0 on success
 */
int pn532_list_passive_targets(pn532_t *dev, pn532_target_t target, unsigned max,
                               unsigned max_retries);

/**
 * @brief Get one ISO14443-A passive target
 *
 * This method blocks until a target is detected.
 *
 * @param[in]  dev          target device
 * @param[out] out          target to be stored
 * @param[in]  max_retries  maximum number of attempts to activate a target
 *                          (0xff blocks indefinitely)
 *
 * @return                  0 on success
 */
int pn532_get_passive_iso14443a(pn532_t *dev, nfc_iso14443a_t *out, unsigned max_retries);

/**
 * @brief Authenticate a Mifare classic card
 *
 * This operation must be done before reading or writing the segment.
 *
 * @param[in]  dev          target device
 * @param[in]  card         card to use
 * @param[in]  keyid        which key to use
 * @param[in]  key          buffer containing the key
 * @param[in]  block        which block to authenticate
 *
 * @return                  0 on success
 */
int pn532_mifareclassic_authenticate(pn532_t *dev, nfc_iso14443a_t *card,
                                     pn532_mifare_key_t keyid, char *key, unsigned block);

/**
 * @brief Read a block of a Mifare classic card
 *
 * The block size is 16 bytes and it must be authenticated before read.
 *
 * @param[in]  dev          target device
 * @param[out] odata        buffer where to store the data
 * @param[in]  card         card to use
 * @param[in]  block        which block to read
 *
 * @return                  0 on success
 */
int pn532_mifareclassic_read(pn532_t *dev, char *odata, nfc_iso14443a_t *card, unsigned block);

/**
 * @brief Write a block of a Mifare classic card
 *
 * The block size is 16 bytes and it must be authenticated before written.
 *
 * @param[in]  dev          target device
 * @param[in]  idata        buffer containing the data to write
 * @param[in]  card         card to use
 * @param[in]  block        which block to write to
 *
 * @return                  0 on success
 */
int pn532_mifareclassic_write(pn532_t *dev, char *idata, nfc_iso14443a_t *card, unsigned block);

/**
 * @brief Read a block of a Mifare Ultralight card
 *
 * The block size is 32 bytes and it must be authenticated before read.
 *
 * @param[in]  dev          target device
 * @param[out] odata        buffer where to store the data
 * @param[in]  card         card to use
 * @param[in]  block        which block to read
 *
 * @return                  0 on success
 */
int pn532_mifareulight_read(pn532_t *dev, char *odata, nfc_iso14443a_t *card, unsigned page);

/**
 * @brief Activate the NDEF file of a ISO14443-A Type 4 tag
 *
 * @param[in]  dev          target device
 * @param[in]  card         card to activate
 *
 * @return                  0 on success
 */
int pn532_iso14443a_4_activate(pn532_t *dev, nfc_iso14443a_t *card);

/**
 * @brief Read data from the NDEF file of a ISO14443-A Type 4 tag
 *
 * The first two bytes of an NDEF file are the length of the data. Afterwards,
 * at offset 0x02 starts the data itself. If one tries to read further than the
 * end of the data no data is returned.
 *
 * @param[in]  dev          target device
 * @param[out] odata        buffer where to store the data
 * @param[in]  card         card to activate
 * @param[in]  offset       offset where to start reading
 * @param[in]  len          length to read
 *
 * @return                  0 on success
 */
int pn532_iso14443a_4_read(pn532_t *dev, char *odata, nfc_iso14443a_t *card, unsigned offset,
                           char len);

/**
 * @brief Deselect a previously selected passive card
 *
 * @param[in]  dev          target device
 * @param[in] target_id     id of the target to deselect (0x00 for all)
 */
void pn532_deselect_passive(pn532_t *dev, unsigned target_id);

/**
 * @brief Release an active passive card
 *
 * @param[in]  dev          target device
 * @param[in] target_id     id of the target to release (0x00 for all)
 */
void pn532_release_passive(pn532_t *dev, unsigned target_id);

#ifdef __cplusplus
}
#endif

#endif /* NFC_READER_INCLUDE_PN532_H_ */
/** @} */