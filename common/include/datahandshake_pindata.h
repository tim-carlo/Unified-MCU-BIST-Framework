#ifndef DATAHANDSHAKE_PINDATA_H
#define DATAHANDSHAKE_PINDATA_H

#include <stdint.h>
#include <stdbool.h>
#include "data_handshake.h"

#define STATUS_ROLE (1U << 0)                // 0 = Initiator, 1 = Responder  
#define STATUS_SEND_REQUEST (1U << 1)        // Set when sending request
#define STATUS_SEND_ANSWER (1U << 2)         // Set when sending answer
#define STATUS_RECEIVED_REQUEST (1U << 3)    // Set when received request
#define STATUS_RECEIVED_ANSWER (1U << 4)     // Set when received answer
#define STATUS_SUCCESSFUL_HS (1U << 5)       // Set on successful handshake
#define STATUS_FAILED_HS (1U << 6)           // Set on failed handshake
#define STATUS_CONNECTED_OWN (1U << 7)       // Set when connected with own device



typedef enum
{
    JOB_SEND_REQUEST,
    JOB_SEND_ANSWER,
    JOB_WAIT_FOR_ANSWER,
    JOB_LISTEN,
} CurrentJobType;
typedef struct
{
    uint8_t pin;
    // Status bit 0 Role (0 = Initiator, 1 = Responder), bit 1 for send request, bit 2 for send answer, 
    // bit 3 for received request, bit 4 for received answer, bit 5 for successful handshake, 
    // bit 6 for failed handshake, bit 7 is connected with own device
    uint8_t status;
    uint8_t number_of_successful_tries;
    uint16_t receiving_counter;
    uint16_t time_until_next_send;
    uint32_t last_send_counter;
    CurrentJobType current_job;
    crc last_crc;
} DataHandshakeData;

typedef enum
{
    ROLE_INITIATOR = 0,
    ROLE_RESPONDER = 1,
    ROLE_UNCLEAR = 2
} RoleType;

// Basic flag functions
void set_flag(DataHandshakeData *data, uint8_t mask, bool value);
bool get_flag(const DataHandshakeData *data, uint8_t mask);

// Role functions
void set_role(DataHandshakeData *data, bool is_responder);
bool get_role_flag(const DataHandshakeData *data);
RoleType get_role(const DataHandshakeData *data);

// Send request flags
void set_send_request(DataHandshakeData *data, bool value);
bool get_send_request(const DataHandshakeData *data);

// Send answer flags  
void set_send_answer(DataHandshakeData *data, bool value);
bool get_send_answer(const DataHandshakeData *data);

// Received request flags
void set_received_request(DataHandshakeData *data, bool value);
bool get_received_request(const DataHandshakeData *data);

// Received answer flags
void set_received_answer(DataHandshakeData *data, bool value);
bool get_received_answer(const DataHandshakeData *data);

// Successful handshake flags
void set_successful_handshake(DataHandshakeData *data, bool value);
bool get_successful_handshake(const DataHandshakeData *data);

// Failed handshake flags
void set_failed_handshake(DataHandshakeData *data, bool value);
bool get_failed_handshake(const DataHandshakeData *data);

// Connected own device flags
void set_connected_own_device(DataHandshakeData *data, bool value);
bool get_connected_own_device(const DataHandshakeData *data);

// Utility functions
bool is_successful(const DataHandshakeData *data);
void clear_status(DataHandshakeData *data);
void reset_counters(DataHandshakeData *data);
void increment_receiving(DataHandshakeData *data);
void increment_sending(DataHandshakeData *data);

// Helper functions
void debug_print_data(const DataHandshakeData *data);

#endif // DATAHANDSHAKE_PINDATA_H