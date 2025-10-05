#include "datahandshake_pindata.h"
#include "printf.h"

void dhandshake_set_flag(DataHandshakeData *data, uint8_t mask, bool value)
{
    if (value)
        data->status |= mask;
    else
        data->status &= ~mask;
}

bool dhandshake_get_flag(const DataHandshakeData *data, uint8_t mask)
{
    return (data->status & mask) != 0;
}

void dhandshake_set_role(DataHandshakeData *data, bool is_responder)
{
    dhandshake_set_flag(data, STATUS_ROLE, is_responder);
}

bool dhandshake_get_role_flag(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_ROLE);
}

DataHandshakeRoleType dhandshake_get_role(const DataHandshakeData *data)
{
    return dhandshake_get_role_flag(data) ? DHANDSHAKE_ROLE_RESPONDER : DHANDSHAKE_ROLE_INITIATOR;
}

void dhandshake_set_send_request(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_SEND_REQUEST, value);
}

bool dhandshake_get_send_request(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_SEND_REQUEST);
}

void dhandshake_set_send_answer(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_SEND_ANSWER, value);
}

bool dhandshake_get_send_answer(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_SEND_ANSWER);
}

void dhandshake_set_received_request(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_RECEIVED_REQUEST, value);
}

bool dhandshake_get_received_request(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_RECEIVED_REQUEST);
}

void dhandshake_set_received_answer(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_RECEIVED_ANSWER, value);
}

bool dhandshake_get_received_answer(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_RECEIVED_ANSWER);
}

void dhandshake_set_successful_handshake(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_SUCCESSFUL_HS, value);
}

bool dhandshake_get_successful_handshake(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_SUCCESSFUL_HS);
}

void dhandshake_set_failed_handshake(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_FAILED_HS, value);
}

bool dhandshake_get_failed_handshake(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_FAILED_HS);
}

void dhandshake_set_connected_own(DataHandshakeData *data, bool value)
{
    dhandshake_set_flag(data, STATUS_CONNECTED_OWN, value);
}

bool dhandshake_get_connected_own(const DataHandshakeData *data)
{
    return dhandshake_get_flag(data, STATUS_CONNECTED_OWN);
}

bool dhandshake_is_successful(const DataHandshakeData *data)
{
    return dhandshake_get_successful_handshake(data) && !dhandshake_get_failed_handshake(data);
}

void dhandshake_clear_status(DataHandshakeData *data)
{
    data->status = 0;
}
