#ifndef CHECK_INITIAL_STATE_H
#define CHECK_INITIAL_STATE_H

// Platform-specific includes
#if defined(PICO_RP2040)
#include "rp2040_helper.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/stdio.h"
#include <stdint.h>
#include <stdbool.h>

// Replace printf to include chip family in the output
#undef printf
#define printf(fmt, ...) \
    ((void)fprintf(stdout, "[%s] " fmt, get_chip_family_name(), ##__VA_ARGS__))
#endif

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "printf.h"
#include <stdbool.h>
#include <stdint.h>
#endif

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "printf.h"
#endif

#include "stack.h"
#include "pindata.h"

#define NUMBER_OF_SCANNING_ITERATIONS 3



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

