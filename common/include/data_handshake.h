#ifndef DATA_HANDSHAKE_H
#define DATA_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#define DATA_TIMER TIMER_B0

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"
#include "nrf52840_utils.h"
#define DATA_TIMER NRF_TIMER3


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

#define DATA_TIMER_INTERVAL_US 1000 // 1ms interval for both platforms


typedef struct
{
    uint8_t pin;
    uint8_t number_of_successful_tries;
    uint8_t *data;
} DataHandshakeData;

void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask);

#endif // DATA_HANDSHAKE_H