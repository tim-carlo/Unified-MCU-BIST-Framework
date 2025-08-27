#ifndef PINDATA_H
#define PINDATA_H

#include <stdint.h>
#include <stdbool.h>
#include "bitmap_iterator.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"
#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#define EVENT_BUFFER_SIZE 10

typedef enum
{
    PIN_INITIALLY_LOW,
    PIN_INITIALLY_HIGH,
    PIN_DISTURBED,
    HANDSHAKE_OK_INITIATOR,
    HANDSHAKE_OK_RESPONDER,
    HANDSHAKE_FAILURE,
    DATA_HANDSHAKE_OK_INITIATOR,
    DATA_HANDSHAKE_OK_RESPONDER,
    DATA_HANDSHAKE_FAILURE,
    PIN_IS_CONNECTED_WITH_OTHER_PIN,

} PinEventType;

typedef struct
{
    uint8_t pin;
    PinEventType pin_event[EVENT_BUFFER_SIZE];
    uint8_t *connected_pins;
    uint8_t event_index;
} PinData;

typedef struct
{
    uint8_t event_type;
} PinEvent;

void initialize_pin_data_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);

#endif // PINDATA_H