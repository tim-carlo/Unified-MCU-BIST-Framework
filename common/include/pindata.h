#ifndef PINDATA_H
#define PINDATA_H

#include <stdint.h>
#include <stdbool.h>
#include "bitmap_iterator.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"

#elif defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#define EVENT_BUFFER_SIZE 10
#define MY_DEVICE_ID 0

typedef enum
{
    PIN_INITIALLY_LOW = 0,
    PIN_INITIALLY_HIGH = 1,
    PIN_DISTURBED = 2,
    HANDSHAKE_OK_INITIATOR = 3,
    HANDSHAKE_OK_RESPONDER = 4,
    HANDSHAKE_FAILURE = 5,
    DATA_HANDSHAKE_OK = 6,
    DATA_HANDSHAKE_FAILURE = 7,
    PIN_IS_CONNECTED_WITH_OTHER_PIN = 8,
} PinEventType;


// Need to log the connected Device IDs as well
typedef struct
{
    uint8_t pin;
    uint8_t other_pin;
    uint8_t device_id;
} PinConnection;


typedef struct
{
    uint8_t pin;
    PinEventType pin_event[EVENT_BUFFER_SIZE];
    PinConnection *connections;
    uint8_t event_index;
} PinData;


void initialize_pin_data_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);
void print_pin_data_array(const PinData *pindata, uint8_t size);


#endif // PINDATA_H