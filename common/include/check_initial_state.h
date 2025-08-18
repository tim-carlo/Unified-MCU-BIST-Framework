#ifndef CHECK_INITIAL_STATE_H
#define CHECK_INITIAL_STATE_H


#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#include "printf.h"
#include <stdbool.h>
#include <stdint.h>

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#include "nrf52840_utils.h"
#include "printf.h"
#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS
#endif

#include "stack.h"
#include "pindata.h"

#define NUMBER_OF_SCANNING_ITERATIONS 10
#define DELAY_BETWEEN_READS_MS 100



// Add function prototypes here if needed
/**
 * @brief Get the initial state of all GPIO pins
 *
 * This function reads the state of all GPIO pins and returns a 64-bit value
 * where each bit represents the state of a pin (bit 0 = pin 0, bit 1 = pin 1, ...).
 *
 * @return uint64_t Initial state of GPIO pins
 */
uint64_t get_initial_pin_state(uint8_t expected_state);

#endif // CHECK_INITIAL_STATE_H

