#include "handshake_pindata.h"
#include "printf.h"

void set_flag(HandshakePinData *pin_data, uint8_t mask, bool value)
{
    if (value)
        pin_data->status |= mask;
    else
        pin_data->status &= ~mask;
}

bool get_flag(const HandshakePinData *pin_data, uint8_t mask)
{
    return (pin_data->status & mask) != 0;
}

void set_syn(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_SYN, value);
}

bool get_syn(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_SYN);
}

void set_syn_ack(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_SYN_ACK, value);
}
bool get_syn_ack(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_SYN_ACK);
}
void set_ack(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_ACK, value);
}

bool get_ack(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_ACK);
}

bool is_successful(const HandshakePinData *pin_data)
{
    return get_syn(pin_data) && get_syn_ack(pin_data) && get_ack(pin_data);
}

void set_initiator_syn(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_SYN, value);
}

bool get_initiator_syn(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_SYN);
}

void set_initiator_synack(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_SYNACK, value);
}

bool get_initiator_synack(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_SYNACK);
}

void set_initiator_ack(HandshakePinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_ACK, value);
}

bool get_initiator_ack(const HandshakePinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_ACK);
}


RoleType get_role(const HandshakePinData *pin_data)
{
    const bool initiator_syn = get_initiator_syn(pin_data);
    const bool initiator_synack = get_initiator_synack(pin_data);
    const bool initiator_ack = get_initiator_ack(pin_data);

    if (initiator_syn && !initiator_synack && initiator_ack)
        return ROLE_INITIATOR;
    else if (!initiator_syn && initiator_synack && !initiator_ack)
        return ROLE_RESPONDER;
    return ROLE_UNCLEAR;
}

void set_task(HandshakePinData *pin_data, PinDataTask task)
{
    if (!pin_data)
        return;

    pin_data->current_job = (uint8_t)task;
}

PinDataTask get_task(const HandshakePinData *pin_data)
{
    if (!pin_data)
        return TASK_NONE;

    return (PinDataTask)(pin_data->current_job);
}

void reset_counters(HandshakePinData *pin_data)
{
    pin_data->waiting_counter = 0;
    pin_data->sending_counter = 0;
    pin_data->receiving_counter = 0;
}

void increment_waiting(HandshakePinData *pin_data)
{
    pin_data->waiting_counter++;
}

void increment_sending(HandshakePinData *pin_data)
{
    pin_data->sending_counter++;
}

void increment_receiving(HandshakePinData *pin_data)
{
    pin_data->receiving_counter++;
}

void clear_status(HandshakePinData *pin_data)
{
    pin_data->status = 0;
}

void debug_print_pindata(const HandshakePinData *pin_data)
{
    if (!pin_data) {
        printf("HandshakePinData: NULL\n");
        return;
    }
    printf("HandshakePinData {\n");
    printf("  pin: %u\n", pin_data->pin);
    printf("  status: 0x%02X\n", pin_data->status);
    printf("    SYN: %d\n", get_syn(pin_data));
    printf("    SYN_ACK: %d\n", get_syn_ack(pin_data));
    printf("    ACK: %d\n", get_ack(pin_data));
    printf("    INITIATOR_SYN: %d\n", get_initiator_syn(pin_data));
    printf("    INITIATOR_SYNACK: %d\n", get_initiator_synack(pin_data));
    printf("    INITIATOR_ACK: %d\n", get_initiator_ack(pin_data));
    printf("  current_job: %u\n", pin_data->current_job);
    printf("  waiting_counter: %u\n", pin_data->waiting_counter);
    printf("  sending_counter: %u\n", pin_data->sending_counter);
    printf("  receiving_counter: %u\n", pin_data->receiving_counter);
    printf("  successful_handshakes: %u\n", pin_data->successful_handshakes);
    printf("}\n");
}