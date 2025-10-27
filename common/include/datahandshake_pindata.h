#ifndef DATAHANDSHAKE_PINDATA_H
#define DATAHANDSHAKE_PINDATA_H

#include <stdbool.h>
#include <stdint.h>
#include "crc.h"

typedef uint8_t CurrentJobType;
enum
{
    JOB_LISTEN = 0,
    JOB_SEND_REQUEST,
    JOB_SEND_ANSWER,
    JOB_WAIT_FOR_ANSWER,
    JOB_TRANSMITTING_ANSWER,
    JOB_TRANSMITTING_REQUEST,
    JOB_RECEIVING_ANSWER,
    JOB_RECEIVING_REQUEST
};

typedef uint8_t DataHandshakeRoleType;
enum
{
    DHANDSHAKE_ROLE_INITIATOR = 0,
    DHANDSHAKE_ROLE_RESPONDER,
    DHANDSHAKE_ROLE_UNCLEAR
};
// Max buffer size for data handshake per pin
#define ANSWER_PACKSIZE 25
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
    uint8_t data_buffer[ANSWER_PACKSIZE];
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