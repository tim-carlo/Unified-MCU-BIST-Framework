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
#define MY_DEVICE_ID_INDEX 0          // Index of own device in seen_devices arrays

#define MAX_CONNECTIONS_PER_PIN 10 // Adjust this if more is needed
#define MAX_SEEN_DEVICES 2         // Maximum number of seen devices to track
#define DEVICE_NOT_FOUND 255

#define PIN_EVENT_COUNT 29
typedef uint8_t PinEventType;
enum
{
    HANDSHAKE_OK_INITIATOR,
    HANDSHAKE_OK_RESPONDER,
    HANDSHAKE_FAILURE,
    DATA_HANDSHAKE_OK,
    DATA_HANDSHAKE_FAILURE,
    PIN_IS_CONNECTED_WITH_INTERNAL_PIN,
    PIN_IS_NOT_LOW_WHEN_PULLED_DOWN,
    PIN_IS_NOT_HIGH_WHEN_PULLED_UP,
    PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW,
    PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH,
    UART_RX_IS_NOT_WORKING,
    EXPECTS_TO_WORK_IN_ONE_DIRECTION,
    EXCEEDS_CONNECTION_LIMIT,
    STEP_1_A_HIGH,
    STEP_1_A_LOW,
    STEP_1_B_HIGH,
    STEP_1_B_LOW,
    STEP_2_A_HIGH,
    STEP_2_A_LOW,
    STEP_2_B_HIGH,
    STEP_2_B_LOW,
    STEP_3_A_HIGH,
    STEP_3_A_LOW,
    STEP_3_B_HIGH,
    STEP_3_B_LOW,
    PIN_FLOATING_IN_PHASE0,
    PIN_FLOATING_IN_PHASE1,
    PIN_FLOATING_IN_PHASE2,
    PIN_FLOATING_IN_PHASE3
};

typedef uint8_t ConnectionType;
enum
{
    CONNECTION_TYPE_INTERNAL = 0,
    CONNECTION_TYPE_EXTERNAL = 1
};
typedef struct
{
    uint8_t other_pin : 6;              // up to 64 pins
    ConnectionType connection_type : 1; // internal or external connection
    uint8_t parameter;                  // bleibt 8 Bit
} PinConnection;

// List so that the algorithm can be extended in the future to work with multiple devices
extern uint8_t seen_devices_count;
extern uint64_t seen_devices[MAX_SEEN_DEVICES];
extern uint8_t seen_devices_index;

typedef struct
{
    uint8_t pin;
    PinConnection connections[MAX_CONNECTIONS_PER_PIN];
    uint8_t connection_index;  // that is the highest used index in connections
    uint8_t connections_count; // number of valid connections
    uint32_t event_mask;       // Bitmask to track which events have occurred
} PinData;

void initialize_pin_data_array(PinData *pindata, uint8_t size);
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event);
bool check_if_pinevent_exists(PinData *pindata, uint8_t pin, PinEventType event);

uint8_t add_seen_device(uint64_t other_device_id);
uint8_t get_index_of_unique_id(uint64_t unique_id);

void add_pin_connection(ConnectionType connection_type, PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint8_t parameter);

// Not sure if this is needed externally

uint64_t get_own_device_id();

#endif // PINDATA_H