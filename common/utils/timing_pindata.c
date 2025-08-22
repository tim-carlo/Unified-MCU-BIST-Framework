#include "timing_pindata.h"
#include "printf.h"

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

bool is_successful(const PinData *pin_data)
{
    return get_syn(pin_data) && get_syn_ack(pin_data) && get_ack(pin_data);
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

void set_task(PinData *pin_data, PinDataTask task)
{
    if (!pin_data)
        return;

    pin_data->current_job = (uint8_t)task;
}

PinDataTask get_task(const PinData *pin_data)
{
    if (!pin_data)
        return TASK_NONE;

    return (PinDataTask)(pin_data->current_job);
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
        pindata[i].pin = i;                    // Assign pin number
        pindata[i].status = 0;                 // Clear status
        pindata[i].current_job = TASK_JOB_SYN; // Set SYN task for every pin
        pindata[i].waiting_counter = 0;        // Reset waiting counter
        pindata[i].sending_counter = 0;        // Reset sending counter
        pindata[i].receiving_counter = 0;      // Reset receiving counter
    }
}

void debug_print_pindata(const PinData *pin_data)
{
    if (!pin_data) {
        printf("PinData: NULL\n");
        return;
    }
    printf("PinData {\n");
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
    printf("  successfull_handshakes: %u\n", pin_data->successfull_handshakes);
    printf("}\n");
}