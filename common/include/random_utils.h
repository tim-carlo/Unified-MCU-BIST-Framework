#ifndef NRF52840_UTILS_H
#define NRF52840_UTILS_H

#include <stdint.h>
#include "nrf.h"
#include "nrf52840.h"
#include "stack.h"
#include "pindata.h"

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_gpio.h"

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"cc
#include "nrf52840_gpio.h"

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Generate a pseudo-random 32-bit number using LFSR algorithm
 * @return 32-bit pseudo-random number
 * @note Uses Wikipedia's LFSR polynomial 0xC3308398
 */
uint32_t random32_lfsr(void);

/**
 * @brief Generate a true random 32-bit number using hardware RNG
 * @return 32-bit random number
 * @note Uses NRF_RNG peripheral, blocks until value is ready
 */
uint32_t random32(void);

/**
 * @brief Get the device's unique 64-bit ID
 * @return 64-bit unique device ID
 * @note Combines DEVICEID[1] and DEVICEID[0] from FICR
 */
uint64_t get_unique_id(void);

/**
 * @brief Get the device's unique ID as hexadecimal string
 * @return Pointer to static 16-character null-terminated hex string
 * @note Format: 16 uppercase hex chars (0000000000000000-FFFFFFFFFFFFFFFF)
 * @warning Not thread-safe (uses static buffer)
 */
const char* get_unique_id_str(void);

/**
 * @brief Get the chip family name
 * @return Constant string "NRF52840"
 */
const char* get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // NRF52840_UTILS_H