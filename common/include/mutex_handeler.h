#ifndef MUTEX_HANDLER_H
#define MUTEX_HANDLER_H
#if defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#endif
#include "spooky_decoder.h"
#include "spooky_encoder.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "printf.h"
#include "pindata.h"
#include "data_handshake.h"

#define MUTEX_HANDELER_BUFFER_SIZE (16)

typedef struct 
{
    uint8_t current_mutex_pin;
    bool iam_mutex_owner;
    bool currently_having_mutex;
    struct spooky_encoder enc;
    struct spooky_decoder dec;
    uint8_t last_mode;
    bool last_rx;
    uint8_t buffer[MUTEX_HANDELER_BUFFER_SIZE]; // since that is the min buffer size for spooky encoder/decoder
} MutexHandler;

typedef enum
{
    ACKNOWLEDGED,
    RELEASED,
    REQUESTED,
    TIMEOUT
} WaitForResult;

#define MUTEX_REQUEST (0x55)  // Request mutex on this pin
#define MUTEX_RELEASE (0xFF) // Release mutex on this pin
#define MUTEX_ACK (0xAA)     // Acknoledge mutex request or release

// Wait for mutex on this pin
void mutex_handler_init(DataHandshakeResult *result, MutexHandler *handler);
void mutex_handler_deinit(MutexHandler *handler);

void mutex_handler_request_mutex(uint64_t blacklist_mask, MutexHandler *handler);
void mutex_handler_release_mutex(uint64_t blacklist_mask, MutexHandler *handler);

#endif // MUTEX_HANDLER_H