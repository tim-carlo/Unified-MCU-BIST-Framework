#ifndef PINDATA_H
#define PINDATA_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
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
#define MY_DEVICE_ID_INDEX 0          // Index of own device in seen_devices arrays

#define MAX_CONNECTIONS_PER_PIN 5 // Adjust this if more is needed
#define MAX_SEEN_DEVICES 2         // Maximum number of seen devices to track
#define DEVICE_NOT_FOUND 255

#define PIN_EVENT_COUNT 29
typedef uint8_t PinEventType;
enum
{
    HANDSHAKE_OK_INITIATOR = 0,
    HANDSHAKE_OK_RESPONDER = 1,
    HANDSHAKE_FAILURE = 2,
    DATA_HANDSHAKE_OK = 3,
    DATA_HANDSHAKE_FAILURE = 4,
    PIN_IS_CONNECTED_WITH_INTERNAL_PIN = 5,
    PIN_IS_NOT_LOW_WHEN_PULLED_DOWN = 6,
    PIN_IS_NOT_HIGH_WHEN_PULLED_UP = 7,
    PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW = 8,
    PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH = 9,
    UART_RX_IS_NOT_WORKING = 10,
    EXPECTS_TO_WORK_IN_ONE_DIRECTION = 11,
    EXCEEDS_CONNECTION_LIMIT = 12,
    STEP_1_A_HIGH = 13,
    STEP_1_A_LOW = 14,
    STEP_1_B_HIGH = 15,
    STEP_1_B_LOW = 16,
    STEP_2_A_HIGH = 17,
    STEP_2_A_LOW = 18,
    STEP_2_B_HIGH = 19,
    STEP_2_B_LOW = 20,
    STEP_3_A_HIGH = 21,
    STEP_3_A_LOW = 22,
    STEP_3_B_HIGH = 23,
    STEP_3_B_LOW = 24,
    PIN_IS_NOT_LOW_WHEN_ALL_PULLED_UP = 26,
    PIN_IS_NOT_HIGH_WHEN_ALL_PULLED_DOWN = 27,
};

typedef uint8_t ConnectionType;
enum
{
    CONNECTION_TYPE_INTERNAL = 0,
    CONNECTION_TYPE_EXTERNAL = 1
};

struct PinConnection
{
    uint8_t other_pin : 6;              // up to 64 pins
    ConnectionType connection_type : 1; // internal or external connection
    uint8_t parameter;
    // One bit is unused               
} __attribute__((packed));
typedef struct PinConnection PinConnection;

// List so that the algorithm can be extended in the future to work with multiple devices
extern uint8_t seen_devices_count;
extern uint64_t seen_devices[MAX_SEEN_DEVICES];
extern uint8_t seen_devices_index;

struct PinData
{
    uint8_t pin;
    PinConnection connections[MAX_CONNECTIONS_PER_PIN];
    uint8_t connection_index;  // that is the highest used index in connections
    uint8_t connections_count; // number of valid connections
    uint32_t event_mask;       // Bitmask to track which events have occurred
} __attribute__((packed));
typedef struct PinData PinData;

void initialize_pin_data_array(PinData *pindata, uint8_t size);

void clear_pin_connections_from_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);
bool check_if_pinevent_exists(PinData *pindata, uint8_t pin, PinEventType event);

uint8_t add_seen_device(uint64_t other_device_id);
uint8_t get_index_of_unique_id(uint64_t unique_id);

void add_pin_connection(ConnectionType connection_type, PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint8_t parameter);

uint64_t get_own_device_id();

#endif // PINDATA_H