#ifndef DATA_HANDSHAKE_H
#define DATA_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS
#endif

#include "manchester.h"
#include "printf.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "timing_pindata.h"
#include "pindata.h"
#include "bitmap_iterator.h"

void perform_data_handshake(uint64_t blacklist_mask);

#endif // DATA_HANDSHAKE_H