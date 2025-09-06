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
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "pindata.h"
#include "printf.h"
#include "cb0r.h"
#include "crc.h"
#include "xxhash.h"

// Serialization constants
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

typedef struct {
    uint8_t *data;
    size_t size_in_bytes;
} SerializedChunk;

typedef struct {
    uint8_t chunk_id;
    uint8_t num_entries;
    crc crc_value;
    PinData *pindata;
} Chunk;

// Function declarations
InitializationResult initialize_serialization(SerializedChunk *output_chunk, PinData *pindata);
SerializationResult serialize_next_chunk();
SerializationResult generate_cbor_header(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size);
SerializationResult send_complete_transmission_with_header(PinData *pindata, uint8_t pindata_size);
SerializationResult send_binary_transmission(PinData *pindata, uint8_t pin_count, uint64_t device_id);
void example_cbor_header_transmission(void);


#ifdef __cplusplus
}
#endif

#endif // SERIALISATION_H