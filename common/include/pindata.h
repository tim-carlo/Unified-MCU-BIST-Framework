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
#endif

#define EVENT_BUFFER_SIZE 10
#define INITIAL_CONNECTION_CAPACITY 1 // Initial capacity for connections array
#define MY_DEVICE_ID_INDEX 0 // Index of own device in seen_devices arrays

#define MAX_CONNECTIONS_PER_PIN 3 // Adjust this if more is needed
#define MAX_SEEN_DEVICES 2 // Maximum number of seen devices to track
#define DEVICE_NOT_FOUND 255

typedef uint8_t PinEventType;
enum
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
    PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH = 13,
    UART_RX_IS_NOT_WORKING = 14,
    EXPECTS_TO_WORK_IN_ONE_DIRECTION = 15
};

// Hier den Grunddatentype Typ definieren um das zu minimieren.

// Need to log the connected Device IDs as well
typedef union
{
    struct
    {
        uint8_t other_pin;
        uint8_t device_index;
    };
    uint16_t raw; 
} PinConnection;


// List so that the algorithm can be extended in the future to work with multiple devices
extern uint8_t seen_devices_count;
extern uint64_t seen_devices[MAX_SEEN_DEVICES];
extern uint8_t seen_devices_index;


typedef struct
{
    uint8_t pin;
    PinConnection connections[MAX_CONNECTIONS_PER_PIN];
    uint8_t connection_index; // that is the highest used index in connections
    uint8_t connections_count; // number of valid connections
    uint32_t event_mask; // Bitmask to track which events have occurred
} PinData;


void initialize_pin_data_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);
bool check_if_pinevent_exists(PinData *pindata, uint8_t pin, PinEventType event);

uint8_t add_seen_device(uint64_t other_device_id);
uint8_t get_index_of_unique_id(uint64_t unique_id);

void add_pin_connection(PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint64_t device_uuid);

// Sorting functions to make all outputs deterministic
void sort_pin_connections(PinData *pindata, uint8_t pin);

// Not sure if this is needed externally
void sort_seen_devices();   

uint64_t get_own_device_id();


#endif // PINDATA_H