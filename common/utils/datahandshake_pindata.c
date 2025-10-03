#include "datahandshake_pindata.h"
#include "printf.h"

void set_flag(DataHandshakeData *data, uint8_t mask, bool value)
{
    if (value)
        data->status |= mask;
    else
        data->status &= ~mask;
}

bool get_flag(const DataHandshakeData *data, uint8_t mask)
{
    return (data->status & mask) != 0;
}

void set_role(DataHandshakeData *data, bool is_responder)
{
    set_flag(data, STATUS_ROLE, is_responder);
}

bool get_role_flag(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_ROLE);
}

RoleType get_role(const DataHandshakeData *data)
{
    return get_role_flag(data) ? ROLE_RESPONDER : ROLE_INITIATOR;
}

void set_send_request(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_SEND_REQUEST, value);
}

bool get_send_request(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_SEND_REQUEST);
}

void set_send_answer(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_SEND_ANSWER, value);
}

bool get_send_answer(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_SEND_ANSWER);
}

void set_received_request(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_RECEIVED_REQUEST, value);
}

bool get_received_request(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_RECEIVED_REQUEST);
}

void set_received_answer(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_RECEIVED_ANSWER, value);
}

bool get_received_answer(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_RECEIVED_ANSWER);
}

void set_successful_handshake(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_SUCCESSFUL_HS, value);
}

bool get_successful_handshake(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_SUCCESSFUL_HS);
}

void set_failed_handshake(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_FAILED_HS, value);
}

bool get_failed_handshake(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_FAILED_HS);
}

void set_connected_own_device(DataHandshakeData *data, bool value)
{
    set_flag(data, STATUS_CONNECTED_OWN, value);
}

bool get_connected_own_device(const DataHandshakeData *data)
{
    return get_flag(data, STATUS_CONNECTED_OWN);
}

bool is_successful(const DataHandshakeData *data)
{
    return get_successful_handshake(data) && !get_failed_handshake(data);
}

void clear_status(DataHandshakeData *data)
{
    data->status = 0;
}

void reset_counters(DataHandshakeData *data)
{
    data->receiving_counter = 0;
    data->last_send_counter = 0;
}

void increment_receiving(DataHandshakeData *data)
{
    data->receiving_counter++;
}

void increment_sending(DataHandshakeData *data)
{
    data->last_send_counter++;
}

void debug_print_data(const DataHandshakeData *data)
{
    if (!data) {
        printf("DataHandshakeData: NULL\n");
        return;
    }
    printf("DataHandshakeData {\n");
    printf("  pin: %u\n", data->pin);
    printf("  status: 0x%02X\n", data->status);
    printf("    ROLE: %d (%s)\n", get_role_flag(data), get_role(data) == ROLE_INITIATOR ? "Initiator" : "Responder");
    printf("    SEND_REQUEST: %d\n", get_send_request(data));
    printf("    SEND_ANSWER: %d\n", get_send_answer(data));
    printf("    RECEIVED_REQUEST: %d\n", get_received_request(data));
    printf("    RECEIVED_ANSWER: %d\n", get_received_answer(data));
    printf("    SUCCESSFUL_HS: %d\n", get_successful_handshake(data));
    printf("    FAILED_HS: %d\n", get_failed_handshake(data));
    printf("    CONNECTED_OWN: %d\n", get_connected_own_device(data));
    printf("  number_of_successful_tries: %u\n", data->number_of_successful_tries);
    printf("  receiving_counter: %u\n", data->receiving_counter);
    printf("  last_send_counter: %u\n", data->last_send_counter);
    printf("}\n");
}