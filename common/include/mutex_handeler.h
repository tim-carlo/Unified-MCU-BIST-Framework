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



// Idee anfangs hat der der mutex holder ist das recht den mutex zu behalten.

// Wait for mutex on this pin
void mutex_handeler_init(DataHandshakeResult *result);
void mutex_handeler_deinit(void);

void mutex_handler_request_mutex();
void mutex_handler_release_mutex();

