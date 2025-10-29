#ifndef SERIALISATION_H
#define SERIALISATION_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#include "nrf52840_utils.h"


#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "pindata.h"
#include "printf.h"
#include "cb0r.h"
#include "crc.h"
#include "endian.h"

typedef struct
{
    uint8_t chunk_id;
    uint8_t *data;
    size_t size_in_bytes;
    crc crc32;
} SerializedChunk;


// Serialization constants
extern SerializedChunk *current_chunk;
extern uint8_t current_pin_data_index;
extern uint8_t current_chunk_id;

#define KEY_CHUNK_ID 0
#define KEY_NUM_ENTRIES 1
#define KEY_PINS 2
#define KEY_CRC 3
#define KEY_PIN 4
#define KEY_EVENTS 5
#define KEY_CONNECTIONS 6
#define KEY_OTHER_PIN 7
#define KEY_DEVICE_ID 8


#define HEADER_KEY_DEVICE_UUID 0
#define HEADER_KEY_DEVICE_FAMILY 1
#define HEADER_KEY_TOTAL_CHUNKS 2
#define HEADER_KEY_TOTAL_PINS 3
#define HEADER_KEY_ACTIVE_PINS 4
#define HEADER_KEY_HEADER_HASH 5
#define HEADER_KEY_NUMBER_SEEN_DEVICES 6
#define HEADER_KEY_SEEN_DEVICE_IDS 7
#define ACK_REQUESTED 8 // Key to indicate if ACK was requested, so if the sender requires an ACK

#define HEADER_VERSION 1
#define HEADER_BUFFER_SIZE 64
#define CHUNK_BUFFER_SIZE 128
#define NUMBER_OF_ENTRIES_PER_CHUNK 5

// Error codes for serialization operations
typedef enum {
    SERIALIZATION_OK = 0,
    SERIALIZATION_ERROR_BUFFER_TOO_SMALL = -1,
    SERIALIZATION_ERROR_INVALID_INPUT = -2,
    SERIALIZATION_ERROR_CBOR_ENCODING_FAILED = -3,
    SERIALIZATION_ERROR_NULL_POINTER = -4,
    SERIALIZATION_MEMORY_ALLOCATION_FAILED = -5
} SerializationResult;

typedef enum {
    INITIALIZATION_OK = 0,
    INITIALIZATION_ERROR = -1
} InitializationResult;


// Function declarations
InitializationResult initialize_serialization(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size, bool ack_req);
SerializationResult serialize_next_chunk();
SerializationResult generate_cbor_header(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size, bool ack_requested);


#ifdef __cplusplus
}
#endif

#endif // SERIALISATION_H