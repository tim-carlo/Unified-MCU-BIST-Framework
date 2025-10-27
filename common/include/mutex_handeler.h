#pragma once

#if defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "printf.h"
#include "pindata.h"
#include "data_handshake.h"

#define MUTEX_REQEST 0x55  // Request mutex on this pin
#define MUTEX_RELEASE 0xFF // Release mutex on this pin
#define MUTEX_ACK 0xAA     // Acknoledge mutex request or release

// Wait for mutex on this pin
void mutex_handeler_init(DataHandshakeResult *result);
void mutex_handeler_deinit(void);

void mutex_handler_request_mutex(uint64_t blacklist_mask);
void mutex_handler_release_mutex(uint64_t blacklist_mask);
