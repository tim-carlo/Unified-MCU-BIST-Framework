#ifndef DATAHANDSHAKE_PINDATA_H
#define DATAHANDSHAKE_PINDATA_H

#include <stdbool.h>
#include <stdint.h>
#include "crc.h"

#define STATUS_ROLE (1U << 0)             // 0 = Initiator, 1 = Responder
#define STATUS_SEND_REQUEST (1U << 1)     // Set when sending request
#define STATUS_SEND_ANSWER (1U << 2)      // Set when sending answer
#define STATUS_RECEIVED_REQUEST (1U << 3) // Set when received request
#define STATUS_RECEIVED_ANSWER (1U << 4)  // Set when received answer
#define STATUS_SUCCESSFUL_HS (1U << 5)    // Set on successful handshake
#define STATUS_FAILED_HS (1U << 6)        // Set on failed handshake
#define STATUS_CONNECTED_OWN (1U << 7)    // Set when connected with own device

// Packet format: [8 bytes UUID][1 byte Pin][4 bytes CRC32]
typedef struct
{
    uint64_t uuid;
    uint8_t pin;
    uint32_t crc_value; // Changed from crc to uint32_t to avoid type conflicts
} RequestDataPacket;

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC32]
typedef struct
{
    uint64_t received_uuid;
    uint8_t received_pin;
    uint64_t own_uuid;
    uint8_t sending_pin;
    uint32_t crc_value; // Changed from crc to uint32_t to avoid type conflicts
} AnswerDataPacket;

typedef enum
{
    JOB_LISTEN = 0,
    JOB_SEND_REQUEST = 1,
    JOB_SEND_ANSWER = 2,
    JOB_WAIT_FOR_ANSWER = 3,
    JOB_WAIT_FOR_REQUEST = 4,
    JOB_TRANSMITTING_ANSWER = 5,
    JOB_TRANSMITTING_REQUEST = 6,
} CurrentJobType;
typedef struct
{
    uint8_t pin;
    // Status bit 0 Role (0 = Initiator, 1 = Responder), bit 1 for send request, bit 2 for send answer,
    // bit 3 for received request, bit 4 for received answer, bit 5 for successful handshake,
    // bit 6 for failed handshake, bit 7 is connected with own device
    uint8_t status;
    CurrentJobType current_job;
    uint8_t number_of_successful_tries;
    uint16_t receiving_counter;
    uint16_t time_until_next_send;
    uint32_t last_send_job_order;
    RequestDataPacket *request_packet;
    AnswerDataPacket *answer_packet;
    uint32_t last_crc; // Changed from crc to uint32_t to avoid type conflicts
} DataHandshakeData;

typedef enum
{
    DHANDSHAKE_ROLE_INITIATOR = 0,
    DHANDSHAKE_ROLE_RESPONDER = 1,
    DHANDSHAKE_ROLE_UNCLEAR = 2
} DataHandshakeRoleType;

// Basic flag functions
void dhandshake_set_flag(DataHandshakeData *data, uint8_t mask, bool value);
bool dhandshake_get_flag(const DataHandshakeData *data, uint8_t mask);

// Role functions
void dhandshake_set_role(DataHandshakeData *data, bool is_responder);
bool dhandshake_get_role_flag(const DataHandshakeData *data);
DataHandshakeRoleType dhandshake_get_role(const DataHandshakeData *data);

// Send request flags
void dhandshake_set_send_request(DataHandshakeData *data, bool value);
bool dhandshake_get_send_request(const DataHandshakeData *data);

// Send answer flags
void dhandshake_set_send_answer(DataHandshakeData *data, bool value);
bool dhandshake_get_send_answer(const DataHandshakeData *data);

// Received request flags
void dhandshake_set_received_request(DataHandshakeData *data, bool value);
bool dhandshake_get_received_request(const DataHandshakeData *data);

// Received answer flags
void dhandshake_set_received_answer(DataHandshakeData *data, bool value);
bool dhandshake_get_received_answer(const DataHandshakeData *data);

// Successful handshake flags
void dhandshake_set_successful_handshake(DataHandshakeData *data, bool value);
bool dhandshake_get_successful_handshake(const DataHandshakeData *data);

// Failed handshake flags
void dhandshake_set_failed_handshake(DataHandshakeData *data, bool value);
bool dhandshake_get_failed_handshake(const DataHandshakeData *data);

// Connected own device flags
void dhandshake_set_connected_own(DataHandshakeData *data, bool value);
bool dhandshake_get_connected_own(const DataHandshakeData *data);

// Utility functions
bool dhandshake_is_successful(const DataHandshakeData *data);
void dhandshake_clear_status(DataHandshakeData *data);
void dhandshake_reset_counters(DataHandshakeData *data);

#endif // DATAHANDSHAKE_PINDATA_H