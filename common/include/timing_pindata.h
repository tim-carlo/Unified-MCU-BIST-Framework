#ifndef TIMING_PINDATA_H
#define TIMING_PINDATA_H

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
    uint8_t initial_delay; // Initial delay in milliseconds
    uint16_t sending_counter;   // Counter for sending a signal
    uint16_t receiving_counter; // Counter for receiving a signal
    uint16_t waiting_counter;   // Counter for waiting for a signal
} PinData;

typedef enum
{
    TASK_NONE = 0,
    TASK_JOB_SYN = 1,
    TASK_JOB_SYN_ACK = 2,
    TASK_JOB_ACK = 3
} PinDataTask;

// Basis flag functions
void set_flag(PinData *pin_data, uint8_t mask, bool value);
bool get_flag(const PinData *pin_data, uint8_t mask);

// Syn flags
void set_syn(PinData *pin_data, bool value);
bool get_syn(const PinData *pin_data);

// SYN-ACK flags
void set_syn_ack(PinData *pin_data, bool value);
bool get_syn_ack(const PinData *pin_data);

// ack flags
void set_ack(PinData *pin_data, bool value);
bool get_ack(const PinData *pin_data);

// initiator flags
void set_initiator_syn(PinData *pin_data, bool value);
bool get_initiator_syn(const PinData *pin_data);

void set_initiator_synack(PinData *pin_data, bool value);
bool get_initiator_synack(const PinData *pin_data);

void set_initiator_ack(PinData *pin_data, bool value);
bool get_initiator_ack(const PinData *pin_data);

// Task functions
void set_task(PinData *pin_data, PinDataTask task);
PinDataTask get_task(const PinData *pin_data);

// counters
void reset_counters(PinData *pin_data);
void increment_waiting(PinData *pin_data);
void increment_sending(PinData *pin_data);
void increment_receiving(PinData *pin_data);

// Reset status
void clear_status(PinData *pin_data);


// Helper functions
void initialize_pin_data_array(PinData *pindata, uint8_t size);
void debug_print_pindata(const PinData *pin_data);

#endif // TIMING_PINDATA_H