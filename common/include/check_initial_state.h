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

typedef struct
{
    bool state;        // Current state of the pin (high or low)
    uint8_t pin_number; // Pin number
    uint8_t number_of_rises;
    uint8_t number_of_falls;
} TimingPinData;


void get_initial_pin_state(PinData *pin_data_array, uint64_t *black_list_mask);

#endif // CHECK_INITIAL_STATE_H

 