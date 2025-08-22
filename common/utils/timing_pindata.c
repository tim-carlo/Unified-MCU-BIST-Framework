#include "timing_pindata.h"

void set_flag(PinData *pin_data, uint8_t mask, bool value)
{
    if (value)
        pin_data->status |= mask;
    else
        pin_data->status &= ~mask;
}

bool get_flag(const PinData *pin_data, uint8_t mask)
{
    return (pin_data->status & mask) != 0;
}

void set_syn(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_SYN, value);
}

bool get_syn(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_SYN);
}

void set_syn_ack(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_SYN_ACK, value);
}
bool get_syn_ack(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_SYN_ACK);
}
void set_ack(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_ACK, value);
}

bool get_ack(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_ACK);
}

void set_initiator_syn(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_SYN, value);
}

bool get_initiator_syn(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_SYN);
}

void set_initiator_synack(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_SYNACK, value);
}

bool get_initiator_synack(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_SYNACK);
}

void set_initiator_ack(PinData *pin_data, bool value)
{
    set_flag(pin_data, STATUS_INITIATOR_ACK, value);
}

bool get_initiator_ack(const PinData *pin_data)
{
    return get_flag(pin_data, STATUS_INITIATOR_ACK);
}

void set_job_flag(PinData *pin_data, uint8_t mask, bool value)
{
    if (value)
        pin_data->job_flags |= mask;
    else
        pin_data->job_flags &= ~mask;
}

bool get_job_flag(const PinData *pin_data, uint8_t mask)
{
    return (pin_data->job_flags & mask) != 0;
}
void set_job_syn(PinData *pin_data, bool value)
{
    set_job_flag(pin_data, JOB_SYN, value);
}
bool get_job_syn(const PinData *pin_data)
{
    return get_job_flag(pin_data, JOB_SYN);
}
void set_job_syn_ack(PinData *pin_data, bool value)
{
    set_job_flag(pin_data, JOB_SYN_ACK, value);
}
bool get_job_syn_ack(const PinData *pin_data)
{
    return get_job_flag(pin_data, JOB_SYN_ACK);
}
void set_job_ack(PinData *pin_data, bool value)
{
    set_job_flag(pin_data, JOB_ACK, value);
}
bool get_job_ack(const PinData *pin_data)
{
    return get_job_flag(pin_data, JOB_ACK);
}

void reset_counters(PinData *pin_data)
{
    pin_data->waiting_counter = 0;
    pin_data->sending_counter = 0;
    pin_data->receiving_counter = 0;
}

void increment_waiting(PinData *pin_data)
{
    pin_data->waiting_counter++;
}

void increment_sending(PinData *pin_data)
{
    pin_data->sending_counter++;
}

void increment_receiving(PinData *pin_data)
{
    pin_data->receiving_counter++;
}

void clear_status(PinData *pin_data)
{
    pin_data->status = 0;
}

void initialize_pin_data_array(PinData *pindata, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++)
    {
        pindata[i].pin = i; // Assign pin number
        pindata[i].status = 0; // Clear status
        pindata[i].job_flags = 0; // Clear job flags
        pindata[i].waiting_counter = 0; // Reset waiting counter
        pindata[i].sending_counter = 0; // Reset sending counter
        pindata[i].receiving_counter = 0; // Reset receiving counter
    }
}
