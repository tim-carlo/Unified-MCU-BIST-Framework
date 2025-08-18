#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

#include <stdint.h>
#include "stack.h"
#include "pindata.h"

#if defined(__MSP430FR5994__)
#include <msp430.h>
#include "msp430fr5994_helper.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_gpio.h"

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

#elif defined(NRF52840_XXAA)
#include "nrf.h"
#include "nrf52840.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS
#endif 

#ifdef __cplusplus
extern "C" {
#endif

uint8_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask);

#ifdef __cplusplus
}
#endif

#endif // RANDOM_UTILS_H