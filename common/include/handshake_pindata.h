#ifndef HANDSHAKE_PINDATA_H
#define HANDSHAKE_PINDATA_H

#include <stdint.h>
#include <stdbool.h>

#define STATUS_SYN (1U << 0)
#define STATUS_SYN_ACK (1U << 1)
#define STATUS_ACK (1U << 2)
#define STATUS_INITIATOR_SYN (1U << 3)
#define STATUS_INITIATOR_SYNACK (1U << 4)
#define STATUS_INITIATOR_ACK (1U << 5)

#define JOB_SYN (1U << 0)
#define JOB_SYN_ACK (1U << 1)
#define JOB_ACK (1U << 2)

typedef struct
{
    uint8_t pin; // Pin number
    // Status first bit is syn second bit is syn_ack third bit is ack
    // the fouth bit is 1 if he was initiator for handshake, fifth bit is 1 if he was initiator for syn_ack handshake, sixth bit is 1 if he was initiator for ack handshake
    uint8_t status;
    uint8_t current_job;
    uint8_t successful_handshakes; // Flag to indicate if the handshake was successful
    uint16_t sending_counter;   // Counter for sending a signal
    uint16_t receiving_counter; // Counter for receiving a signal
    uint16_t waiting_counter;   // Counter for waiting for a signal
    uint8_t number_of_unsuccessful_syns; // Counter for unsuccessful SYN attempts
} HandshakePinData;

typedef enum
{
    TASK_NONE = 0,
    TASK_JOB_SYN = 1,
    TASK_JOB_SYN_ACK = 2,
    TASK_JOB_ACK = 3
} PinDataTask;

typedef enum
{
    ROLE_NONE = 0,
    ROLE_INITIATOR = 1,
    ROLE_RESPONDER = 2,
    ROLE_UNCLEAR = 3
} RoleType;

// Basis flag functions
void set_flag(HandshakePinData *pin_data, uint8_t mask, bool value);
bool get_flag(const HandshakePinData *pin_data, uint8_t mask);

// Syn flags
void set_syn(HandshakePinData *pin_data, bool value);
bool get_syn(const HandshakePinData *pin_data);

// SYN-ACK flags
void set_syn_ack(HandshakePinData *pin_data, bool value);
bool get_syn_ack(const HandshakePinData *pin_data);

// ack flags
void set_ack(HandshakePinData *pin_data, bool value);
bool get_ack(const HandshakePinData *pin_data);

// is successful
bool is_successful(const HandshakePinData *pin_data);

// initiator flags
void set_initiator_syn(HandshakePinData *pin_data, bool value);
bool get_initiator_syn(const HandshakePinData *pin_data);

void set_initiator_synack(HandshakePinData *pin_data, bool value);
bool get_initiator_synack(const HandshakePinData *pin_data);

void set_initiator_ack(HandshakePinData *pin_data, bool value);
bool get_initiator_ack(const HandshakePinData *pin_data);

RoleType get_role(const HandshakePinData *pin_data);

// Task functions
void set_task(HandshakePinData *pin_data, PinDataTask task);
PinDataTask get_task(const HandshakePinData *pin_data);

// counters
void reset_counters(HandshakePinData *pin_data);
void increment_waiting(HandshakePinData *pin_data);
void increment_sending(HandshakePinData *pin_data);
void increment_receiving(HandshakePinData *pin_data);

// Reset status
void clear_status(HandshakePinData *pin_data);

// Helper functions
void debug_print_pindata(const HandshakePinData *pin_data);

#endif // TIMING_PINDATA_H