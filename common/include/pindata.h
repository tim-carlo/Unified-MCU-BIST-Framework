#ifndef PINDATA_H
#define PINDATA_H

#include <stdint.h>
#include <stdbool.h>
#include "bitmap_iterator.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"
#include "nrf52840_utils.h"

#elif defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#define EVENT_BUFFER_SIZE 10
#define INITIAL_CONNECTION_CAPACITY 1 // Initial capacity for connections array
#define MAX_SEEN_DEVICES 5 // Maximum number of seen devices to track
#define MY_DEVICE_ID_INDEX 0 // Index of own device in seen_devices arrays

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
    PIN_IS_CONNECTED_WITH_INTERNAL_PIN = 8,
    PIN_IS_CONNECTED_WITH_EXTERNAL_PIN = 9,
    PIN_IS_NOT_LOW_WHEN_PULLED_DOWN = 10,
    PIN_IS_NOT_HIGH_WHEN_PULLED_UP = 11,
    PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW = 12,
    PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH = 13

} PinEventType;

// Hier den Grunddatentype Typ definieren um das zu minimieren.

// Need to log the connected Device IDs as well
typedef struct
{
    uint8_t other_pin;
    uint8_t device_index;  // Index into seen_devices array
} PinConnection;


// List so that the algorithm can be extended in the future to work with multiple devices
extern uint64_t *seen_devices;
extern uint8_t seen_devices_count;


typedef struct
{
    uint8_t pin;
    PinEventType pin_event[EVENT_BUFFER_SIZE];
    PinConnection *connections;
    uint8_t event_index;
    uint8_t connection_index;
    uint8_t connection_capacity;
} PinData;


void initialize_pin_data_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);


uint8_t add_seen_device(uint64_t *other_device_id);
uint8_t get_index_of_unique_id(uint64_t unique_id);
void add_pin_connection(PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint8_t device_index);

void print_pin_data_array(const PinData *pindata, uint8_t size);


#endif // PINDATA_H