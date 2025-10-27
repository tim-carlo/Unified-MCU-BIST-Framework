#ifndef DATAHANDSHAKE_PINDATA_H
#define DATAHANDSHAKE_PINDATA_H

#include <stdbool.h>
#include <stdint.h>
#include "crc.h"

// Packet format: [8 bytes UUID][1 byte Pin][1 byte Mutex Request][4 bytes CRC32]
typedef struct
{
    uint64_t uuid;
    uint8_t pin;
    uint8_t mutex_request; // Mutex request field (0x55 = request, 0x00 = no request)
    uint32_t crc_value;    // Changed from crc to uint32_t to avoid type conflicts
} RequestDataPacket;

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC32]
typedef struct
{
    uint64_t received_uuid;
    uint8_t received_pin;
    uint64_t own_uuid;
    uint8_t sending_pin;
    uint32_t crc_value; // Changed from crc to uint32_t to avoid type conflicts
    uint8_t mutex_allowed;
} AnswerDataPacket;

typedef enum
{
    JOB_LISTEN = 0,
    JOB_SEND_REQUEST = 1,
    JOB_SEND_ANSWER = 2,
    JOB_WAIT_FOR_ANSWER = 3,
    JOB_TRANSMITTING_ANSWER = 4,
    JOB_TRANSMITTING_REQUEST = 5,
    JOB_RECEIVING_ANSWER = 6,
    JOB_RECEIVING_REQUEST = 7,
} CurrentJobType;

typedef enum
{
    DHANDSHAKE_ROLE_INITIATOR = 0,
    DHANDSHAKE_ROLE_RESPONDER = 1,
    DHANDSHAKE_ROLE_UNCLEAR = 2
} DataHandshakeRoleType;
typedef struct
{
    uint8_t pin;
    DataHandshakeRoleType role;
    CurrentJobType current_job;
    uint8_t number_of_successful_tries;
    uint8_t number_of_received_requests;
    uint16_t receiving_counter;
    uint16_t time_until_next_send;
    uint32_t last_send_job_order;

    uint8_t *data_buffer;
    uint8_t manchester_instance_index;
} DataHandshakeData;

// Role functions
static inline void dhandshake_set_role(DataHandshakeData *data, bool is_responder)
{
    data->role = is_responder ? DHANDSHAKE_ROLE_RESPONDER : DHANDSHAKE_ROLE_INITIATOR;
}
static inline DataHandshakeRoleType dhandshake_get_role(const DataHandshakeData *data)
{
    return data->role;
}

#endif // DATAHANDSHAKE_PINDATA_H