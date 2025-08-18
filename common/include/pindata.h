#ifndef PINDATA_H
#define PINDATA_H

#include <stdint.h>
#include <stdbool.h>

#define ERROR_REASON_NONE 0
#define ERROR_REASON_TIMEOUT 1  
#define ERROR_REASON_BLACKLISTED 2
#define ERROR_REASON_DISTURBED 3
#define ERROR_REASON_TRIES_EXCEEDED 4

#define INVALID_TIMESTAMP 0xFFFFFFFFUL // Explicit invalid value as uint32_t


typedef struct
{
    uint8_t pin; // Pin number (0-47)
    // The 'steps' field uses individual bits to represent various flags:
    // Bit 0: ACK signal
    // Bit 1: SYN-ACK signal
    // Bit 2: SYN signal
    // Bit 3: Role (0 = initiator, 1 = responder)
    // Bit 4: Blacklisted status (1 = blacklisted)
    // Bit 5: Success status (1 = successful)
    uint8_t steps;
    uint8_t num_false_responses; // Number of false responses received
    uint8_t num_tries;          // Number of tries made with this pin
    uint8_t error_reason;        // Error reason code
    uint32_t last_falling_edge; // Timestamp of the last falling edge
} PinData;

void reset_pin_data(PinData *data);
void reset_last_falling_edge(PinData *data);
void initialize_pin_data_array(PinData *array, uint32_t length);
void print_pin_data_array(PinData *array, uint32_t length);

void set_ack(PinData *data, bool value);
void set_syn_ack(PinData *data, bool value);
void set_syn(PinData *data, bool value);
void set_role(PinData *data, bool value);
void set_blacklisted(PinData *data, bool value);
void set_successful(PinData *data, bool value);
bool is_ack(PinData *data);
bool is_syn_ack(PinData *data);
bool is_syn(PinData *data);
bool is_role_responder(PinData *data);
bool is_blacklisted(PinData *data);
bool is_successful(PinData *data);
void set_blacklisted_in_mask(volatile uint64_t *mask, uint32_t pin);
#endif // PINDATA_H