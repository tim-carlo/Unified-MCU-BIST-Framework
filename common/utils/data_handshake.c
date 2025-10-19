#include "data_handshake.h"

#if defined(NRF52840_XXAA)
#include "endian.h"
#elif defined(__MSP430FR5994__)
#include "endian.h"
#endif
#include <string.h>

// Define LOG macro for LOGging (can be disabled by commenting out)
//#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define LOG(fmt, ...) // Uncomment this line to disable LOGging


static const uint16_t SEND_INACCURACY = (30 / DATA_TIMER_INTERVAL_MS);
static const uint16_t INITIAL_LOW_TIME_REQUEST_MS = (50 / DATA_TIMER_INTERVAL_MS);
static const uint16_t INITIAL_LOW_TIME_REQUEST_MIN_MS = ((50 - 30) / DATA_TIMER_INTERVAL_MS);
static const uint16_t INITIAL_LOW_TIME_REQUEST_MAX_MS = ((50 + 30) / DATA_TIMER_INTERVAL_MS);

static const uint16_t INITIAL_LOW_TIME_ANSWER_MS = (100 / DATA_TIMER_INTERVAL_MS);
static const uint16_t INITIAL_LOW_TIME_ANSWER_MIN_MS = ((100 - 30) / DATA_TIMER_INTERVAL_MS);
static const uint16_t INITIAL_LOW_TIME_ANSWER_MAX_MS = ((100 + 30) / DATA_TIMER_INTERVAL_MS);
static const uint16_t TIMEOUT_CYCLES = (10000 / DATA_TIMER_INTERVAL_MS);
static const uint16_t TIMEOUT_CYCLES_SENDING_REQUEST = (10000 / DATA_TIMER_INTERVAL_MS);
static const uint16_t TIMEOUT_CYCLES_SENDING_ANSWER = (20000 / DATA_TIMER_INTERVAL_MS);
static const uint16_t MINMUM_REQUEST_CYCLES = 10;
static const uint16_t MAXIMUM_REQUEST_CYCLES = 500;
static const uint16_t MAXIMUM_IDLE_TIME = 600; // This needs to be higher than the maximum request time

// Global variables
PinData *global_pindata;
static DataHandshakeData *global_datahandshake_pindata = NULL; // Global pointer to DataHandshakeData array

static uint64_t internal_blacklist_mask = 0;
static uint64_t internal_valid_pins = ~0ULL; // Assume 64-bit max
static uint64_t responder_mask = 0;
static uint64_t initiator_mask = 0;
static uint64_t uuid = 0;
static uint32_t global_last_worker = 0;
static volatile bool interrupt_cont = false;

static uint8_t mutex_pin = 255;       // Pin currently holding the mutex (255 = none)
static bool i_am_mutex_owner = false; // Whether this device currently owns the mutex
static uint8_t number_of_pins = 0;

/**
 * @brief This function is used to build the initiator and responder masks based on pindata events
 * The Idea in this handshake to improve its perfomance is to only listen on pins where the MCU was the responder.
 * And only send requests on pins where the MCU was the initiator.
 * @param pindata
 */
static void analyze_pindata_events(PinData *pindata)
{
    // Reset masks and counters
    initiator_mask = 0;
    responder_mask = 0;
    number_of_pins = 0;

    // Calculate all pins mask at runtime - assume 64-bit max
    const uint64_t all_pins_mask = ~0ULL;

    // Iterate through all pins that are not blacklisted
    BitmapIterator it = bitmap_iterator_create(~internal_blacklist_mask & all_pins_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        bool has_initiator = check_if_pinevent_exists(pindata, pin, HANDSHAKE_OK_INITIATOR);
        bool has_responder = check_if_pinevent_exists(pindata, pin, HANDSHAKE_OK_RESPONDER);

        // This is needed to check if the pin is expected to work in only one direction
        // So that we can test this single direction pins properly
        // Therefore we are initiator on this pin
        bool experted_to_work_in_one_direction = check_if_pinevent_exists(pindata, pin, EXPECTS_TO_WORK_IN_ONE_DIRECTION);

        // Apply LOGic for mask assignment and blacklist update
        if (has_initiator && !has_responder || experted_to_work_in_one_direction)
        {
            initiator_mask |= (1ULL << pin);
            number_of_pins++;
        }
        else if (has_responder && !has_initiator)
        {
            responder_mask |= (1ULL << pin);
            number_of_pins++;
        }
        else if (!has_initiator && !has_responder)
        {
            // No valid role detected → blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
        }
        else
        {
            // Both roles detected (should not happen) → blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
        }
    }
}

static uint16_t get_listen_until_time(float factor)
{
    uint16_t random_offset = (uint16_t)(random32() % (MAXIMUM_REQUEST_CYCLES - MINMUM_REQUEST_CYCLES)) + 10; // between 10 and 500 ms
    random_offset *= factor;
    return random_offset;
}

static void write_u64_le(uint8_t *dst, uint64_t val)
{
    for (int i = 0; i < 8; i++)
    {
        dst[i] = (uint8_t)(val >> (8 * i));
    }
}

static uint64_t read_u64_le(const uint8_t *src)
{
    uint64_t val = 0;
    for (int i = 0; i < 8; i++)
    {
        val |= ((uint64_t)src[i]) << (8 * i);
    }
    return val;
}

static void write_u32_le(uint8_t *dst, uint32_t val)
{
    for (int i = 0; i < 4; i++)
    {
        dst[i] = (uint8_t)(val >> (8 * i));
    }
}

static uint32_t read_u32_le(const uint8_t *src)
{
    uint32_t val = 0;
    for (int i = 0; i < 4; i++)
    {
        val |= ((uint32_t)src[i]) << (8 * i);
    }
    return val;
}

#define REQUEST_PACKET_BUFFER_SIZE REQUEST_PACKSIZE
#define ANSWER_PACKET_BUFFER_SIZE ANSWER_PACKSIZE

// Buffer for request packet construction
static uint8_t request_packet_buffer[REQUEST_PACKET_BUFFER_SIZE];
// Buffer for answer packet construction
static uint8_t answer_packet_buffer[ANSWER_PACKET_BUFFER_SIZE];

// Packet format: [1 byte type][8 bytes UUID][1 byte Pin][1 byte Mutex Request][4 bytes CRC]
static void construct_request_data_packet(RequestDataPacket *packet, uint8_t *data)
{
    // Use internal buffer for construction
    uint8_t *buf = request_packet_buffer;
    buf[0] = 0xAA; // Packet type
    write_u64_le(&buf[1], packet->uuid);
    buf[9] = packet->pin;
    buf[10] = packet->mutex_request;      // Mutex request field
    packet->crc_value = crcFast(buf, 11); // Calculate CRC over type, uuid, pin, and mutex request
    write_u32_le(&buf[11], packet->crc_value);

    memcpy(data, buf, REQUEST_PACKET_BUFFER_SIZE);
}

// Packet format: [1 byte type][8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][1 byte mutex_allowed][4 bytes CRC]
static void construct_answer_data_packet(AnswerDataPacket *packet, uint8_t *data)
{
    // Use internal buffer for construction
    uint8_t *buf = answer_packet_buffer;
    buf[0] = 0xFF; // Packet type
    write_u64_le(&buf[1], packet->received_uuid);
    buf[9] = packet->received_pin;
    write_u64_le(&buf[10], packet->own_uuid);
    buf[18] = packet->sending_pin;
    buf[19] = packet->mutex_allowed;      // Write mutex_allowed field
    packet->crc_value = crcFast(buf, 20); // Calculate CRC over type, received_uuid, received_pin, own_uuid, sending_pin, mutex_allowed
    write_u32_le(&buf[20], packet->crc_value);

    memcpy(data, buf, ANSWER_PACKET_BUFFER_SIZE);
}

static bool deconstruct_request_data_packet(const uint8_t *data, RequestDataPacket *packet)
{
    // Check packet type
    if (data[0] != 0xAA)
    {
        return false;
    }
    packet->uuid = read_u64_le(&data[1]);
    packet->pin = data[9];
    packet->mutex_request = data[10]; // Extract mutex request field
    packet->crc_value = read_u32_le(&data[11]);
    return true;
}

static bool deconstruct_answer_data_packet(const uint8_t *data, AnswerDataPacket *packet)
{
    // Check packet type
    if (data[0] != 0xFF)
    {
        return false;
    }
    packet->received_uuid = read_u64_le(&data[1]);
    packet->received_pin = data[9];
    packet->own_uuid = read_u64_le(&data[10]);
    packet->sending_pin = data[18];
    packet->mutex_allowed = data[19]; // Read mutex_allowed field
    packet->crc_value = read_u32_le(&data[20]);
    return true;
}

static bool start_receiving_request(DataHandshakeData *p)
{
    uint8_t manchester_idx = p->manchester_instance_index;

    // Start background reception - data will be written to instance buffer
    uint8_t *buffer = parallel_manchester_get_received_data(manchester_idx);
    return parallel_manchester_receive_background(manchester_idx, buffer, REQUEST_PACKSIZE);
}

static bool start_receiving_answer(DataHandshakeData *p)
{
    uint8_t manchester_idx = p->manchester_instance_index;

    // Start background reception - data will be written to instance buffer
    uint8_t *buffer = parallel_manchester_get_received_data(manchester_idx);
    return parallel_manchester_receive_background(manchester_idx, buffer, ANSWER_PACKSIZE);
}

static bool handle_request_receive_complete(uint8_t pin, DataHandshakeData *p)
{

    RequestDataPacket request_packet;

    // Get received data from Manchester instance buffer
    uint8_t *received_data = parallel_manchester_get_received_data(p->manchester_instance_index);
    if (!received_data)
        return false;

    if (!deconstruct_request_data_packet(received_data, &request_packet))
        return false;

    // Verify CRC if needed
    // crc computed_crc = crcFast(p->data_buffer, 10);
    // if (computed_crc != request_packet.crc_value)
    // {
    //     return false; // CRC mismatch
    // }

    // Mark that we received a request
    dhandshake_set_received_request(p, true);
    // Add a pin connection to the request packet

    add_pin_connection(&global_pindata[pin], pin, request_packet.pin, request_packet.uuid);
    LOG("Pin connection added: local_pin=%u, remote_pin=%u\n", pin, request_packet.pin);

    AnswerDataPacket answer_packet;

    // Prepare answer packet using static structure to avoid memory leaks
    answer_packet.received_uuid = request_packet.uuid;
    answer_packet.received_pin = request_packet.pin;
    answer_packet.own_uuid = uuid;
    answer_packet.sending_pin = pin;
    answer_packet.crc_value = 0; // Will be calculated later

    // Handle mutex request
    if (request_packet.mutex_request == REQEST_MUTEX_ON_THIS_PIN && request_packet.uuid != uuid)
    {
        // If I am not holding the mutex the other device can attempt to get it again since he probably lost it
        if (mutex_pin == 255 || !i_am_mutex_owner)
        {
            answer_packet.mutex_allowed = ALLOWING_MUTEX_ON_THIS_PIN; // Grant mutex
            mutex_pin = pin;
            i_am_mutex_owner = false;
            LOG("Mutex granted to other device on pin %u\n", pin);
        }
        else
        {
            LOG("Mutex request denied on pin %u, already owned on pin %u\n", pin, mutex_pin);
        }
    }

    p->answer_packet = &answer_packet;

    // Mark that we're going to send an answer
    dhandshake_set_send_answer(p, true);
    gpio_od_hold_low(pin); // Hold the line low to signal we're responding
    return true;
}

static bool handle_answer_complete(uint8_t pin, DataHandshakeData *p)
{
    AnswerDataPacket answer_packet;

    // Get received data from Manchester instance buffer
    uint8_t *received_data = parallel_manchester_get_received_data(p->manchester_instance_index);
    if (!received_data)
        return false;

    if (!deconstruct_answer_data_packet(received_data, &answer_packet))
        return false;

    // Check if the answer is for us
    if (answer_packet.received_uuid != uuid || answer_packet.received_pin != pin)
    {
        LOG("Answer not for us");
        return false;
    }

    // Mark successful handshake
    dhandshake_set_received_answer(p, true);
    dhandshake_set_successful_handshake(p, true);

    // Add pin connection to pindata events
    PinData *pindata = &global_pindata[pin];
    uint64_t other_device_id = answer_packet.own_uuid;
    add_pin_connection(pindata, pin, answer_packet.received_pin, other_device_id);

    if (answer_packet.mutex_allowed == ALLOWING_MUTEX_ON_THIS_PIN)
    {
        mutex_pin = pin;
        i_am_mutex_owner = true;
        LOG("Mutex granted to this device on pin %u\n", pin);
    }

    p->number_of_successful_tries++;
    LOG("Pin connection added: local_pin=%u, remote_pin=%u\n", pin, answer_packet.sending_pin);
    return true;
}

static bool send_request_in_background(uint8_t pin, DataHandshakeData *p)
{
    gpio_od_release(pin);

    // Small delay to ensure line is released before transmitting
    //delay_us(1000);

    // Mark that we're sending a request
    dhandshake_set_send_request(p, true);
    RequestDataPacket request_packet;
    request_packet.uuid = uuid;
    request_packet.pin = pin;
    request_packet.crc_value = 0; // Will be calculated in construct function

    // if no mutex pin is set, request mutex on this pin
    if (mutex_pin == 255)
    {
        request_packet.mutex_request = REQEST_MUTEX_ON_THIS_PIN; // Request mutex on this pin
    }

    p->request_packet = &request_packet;

    uint8_t request_buffer[REQUEST_PACKSIZE];

    construct_request_data_packet(&request_packet, request_buffer);

    const uint8_t manchester_idx = p->manchester_instance_index;
    bool result = parallel_manchester_transmit_background(manchester_idx, request_buffer, REQUEST_PACKSIZE);
    if (!result)
    {
        return false;
    }
    return true;
}

// Update send_answer_in_background to use the single buffer:
static bool send_answer_in_background(uint8_t pin, DataHandshakeData *p)
{
    gpio_od_release(pin);
    // Small delay to ensure line is released before transmitting
    //delay_us(1000);

    // Check for null pointer to prevent crashes
    if (!p->answer_packet)
    {
        return false;
    }

    uint8_t answer_buffer[ANSWER_PACKSIZE];

    construct_answer_data_packet(p->answer_packet, answer_buffer);

    uint8_t manchester_idx = p->manchester_instance_index;
    bool result = parallel_manchester_transmit_background(manchester_idx, answer_buffer, ANSWER_PACKSIZE);
    if (!result)
    {
        return false;
    }
    return true;
}

static void reschedule_request(DataHandshakeData *p, uint32_t counter)
{
    // Schedule next send time to avoid immediate resend
    p->time_until_next_send = get_listen_until_time(1);
    p->last_send_job_order = counter; // Reset to allow immediate sending when time is up
}

static uint32_t counter = 0;

static void fsm_data_handshake(void)
{
    gpio_drive_high(DEBUG_PIN1);
    counter++;

    BitmapIterator it = bitmap_iterator_create(internal_valid_pins);
    uint8_t pin_index;

    bool something_happened = false;

    while (bitmap_iterator_next(&it, &pin_index))
    {
        DataHandshakeData *p = &global_datahandshake_pindata[pin_index];
        const uint32_t delta = counter - p->last_send_job_order;

        switch (p->current_job)
        {
        case JOB_LISTEN:
        {
            const bool is_low = !gpio_read(p->pin);
            if (is_low)
            {
                // Line is low, start counting
                p->receiving_counter++;
            }
            else
            {
                const uint16_t receive_counter = p->receiving_counter;
                // Here we check if the low time matches a request signal
                if (receive_counter >= INITIAL_LOW_TIME_REQUEST_MIN_MS && receive_counter <= INITIAL_LOW_TIME_REQUEST_MAX_MS)
                {
                    gpio_drive_high(DEBUG_PIN2);
                    if (!start_receiving_request(p))
                    {
                        dhandshake_set_failed_handshake(p, true);
                        LOG("Failed to start receiving request on pin %u\n", p->pin);
                    }
                    else
                    {
                        p->current_job = JOB_RECEIVING_REQUEST;
                        p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                    }
                    // LOG("Request received on pin %u\n", pin);

                    p->receiving_counter = 0;
                    gpio_drive_low(DEBUG_PIN2);
                }
                // If the signal was too short or too long, just reset the counter
                else if (receive_counter > INITIAL_LOW_TIME_REQUEST_MAX_MS || receive_counter < INITIAL_LOW_TIME_REQUEST_MIN_MS)
                {
                    p->receiving_counter = 0;
                }

                // Schedule the next send if we're the initiator
                if (dhandshake_get_role(p) == DHANDSHAKE_ROLE_INITIATOR && delta >= p->time_until_next_send)
                {

                    p->current_job = JOB_SEND_REQUEST;
                    p->last_send_job_order = counter;
                    gpio_od_hold_low(p->pin); // Start sending by pulling line low
                }
            }
            break;
        }
        case JOB_SEND_REQUEST:
        {
            if (delta > INITIAL_LOW_TIME_REQUEST_MS)
            {
                if (!send_request_in_background(p->pin, p))
                {
                    dhandshake_set_failed_handshake(p, true);
                    reschedule_request(p, counter); // reschedule in any case to avoid immediate resend
                    p->current_job = JOB_LISTEN;    // Go back to listening on failure
                }
                else
                {
                    p->current_job = JOB_TRANSMITTING_REQUEST;
                }
                p->last_send_job_order = counter; // Reset timeout counter on successful request handling
            }
            something_happened = true;
            break;
        }
        case JOB_SEND_ANSWER:
        {
            if (delta > INITIAL_LOW_TIME_ANSWER_MS)
            {
                if (!send_answer_in_background(p->pin, p))
                {
                    dhandshake_set_failed_handshake(p, true);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                }
                else
                {
                    p->current_job = JOB_TRANSMITTING_ANSWER;
                }
                p->last_send_job_order = counter; // Reset timeout counter on successful request handling
            }
            something_happened = true;
            break;
        }
        case JOB_TRANSMITTING_REQUEST:
        case JOB_TRANSMITTING_ANSWER:
        {
            bool is_complete = parallel_manchester_transmit_complete(p->manchester_instance_index);

            if (is_complete)
            {
                // Transmission completed, update job state
                if (p->current_job == JOB_TRANSMITTING_REQUEST)
                {
                    p->current_job = JOB_WAIT_FOR_ANSWER;
                }
                else
                {
                    p->current_job = JOB_LISTEN;
                }
            }
            // Timeout for transmission if complete signal not received
            else if (p->current_job == JOB_TRANSMITTING_REQUEST && delta > TIMEOUT_CYCLES_SENDING_REQUEST)
            {
                // Timeout occurred during transmission
                dhandshake_set_failed_handshake(p, true);
                reschedule_request(p, counter);
                p->current_job = JOB_LISTEN; // Go back to listening on timeout
            }
            else if (p->current_job == JOB_TRANSMITTING_ANSWER && delta > TIMEOUT_CYCLES_SENDING_ANSWER)
            {
                // Timeout occurred during transmission
                dhandshake_set_failed_handshake(p, true);
                p->current_job = JOB_LISTEN; // Go back to listening on timeout
            }
            something_happened = true;
            break;
        }
        case JOB_WAIT_FOR_ANSWER:
        {
            const bool is_low = !gpio_read(p->pin);
            if (is_low)
            {
                // Line is low, start counting
                p->receiving_counter++;
            }
            else
            {
                const uint16_t receive_counter = p->receiving_counter;
                bool is_failed = false;

                if (receive_counter >= INITIAL_LOW_TIME_ANSWER_MIN_MS && receive_counter <= INITIAL_LOW_TIME_ANSWER_MAX_MS)
                {
                    p->receiving_counter = 0;
                    if (!start_receiving_answer(p))
                    {
                        dhandshake_set_failed_handshake(p, false);
                        is_failed = true;
                        LOG("Failed to start receiving answer on pin %u\n", p->pin);
                    }
                    else
                    {
                        p->current_job = JOB_RECEIVING_ANSWER;
                        p->last_send_job_order = counter; // Reset timeout counter on successful answer handling
                    }
                }
                // If the signal was too short or too long, just reset the counter or if timeout occurred while waiting for answer
                // and reschedule the request
                else if (receive_counter > INITIAL_LOW_TIME_ANSWER_MAX_MS || delta > TIMEOUT_CYCLES)
                {
                    is_failed = true;
                }
                if (is_failed)
                {
                    LOG("- Failed to receive answer on pin %u\n", p->pin);
                    reschedule_request(p, counter);
                    dhandshake_set_failed_handshake(p, true);
                    p->current_job = JOB_LISTEN;
                }
            }
            something_happened = true;
            break;
        }
        case JOB_RECEIVING_REQUEST:
        {
            gpio_drive_high(DEBUG_PIN2);
            if (parallel_manchester_receive_complete(p->manchester_instance_index))
            {

                if (!handle_request_receive_complete(p->pin, p))
                {
                    // Failed to handle request properly
                    dhandshake_set_failed_handshake(p, true);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                    LOG("Failed to handle request on pin %u\n", p->pin);
                }
                else
                {
                    // Successfully received request and prepared to send answer
                    p->current_job = JOB_SEND_ANSWER;
                    p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                }
            }
            else if (parallel_manchester_receive_error(p->manchester_instance_index))
            {
                dhandshake_set_failed_handshake(p, true);
                LOG("Failed to receive request on pin %u\n", p->pin);
                p->current_job = JOB_LISTEN; // Go back to listening on failure
            }
            something_happened = true;
            gpio_drive_low(DEBUG_PIN2);
            break;
        }
        case JOB_RECEIVING_ANSWER:
        {
            if (parallel_manchester_receive_complete(p->manchester_instance_index))
            {
                if (!handle_answer_complete(p->pin, p))
                {
                    dhandshake_set_failed_handshake(p, false);
                    LOG("Failed to handle answer on pin %u\n", p->pin);
                }
                else
                {
                    // Successfully received answer
                    // blacklist this pin for further requests to avoid flooding
                    internal_valid_pins &= ~(1ULL << pin_index);
                }
                p->current_job = JOB_LISTEN; // Go back to listening after handling answer
            }
            else if (parallel_manchester_receive_error(p->manchester_instance_index))
            {
                dhandshake_set_failed_handshake(p, false);
                LOG("Failed to receive answer on pin %u\n", p->pin);
                reschedule_request(p, counter);
                p->current_job = JOB_LISTEN; // Go back to listening on failure
            }
            something_happened = true;
            break;
        }
        default:
            break;
        }
    }
    gpio_drive_low(DEBUG_PIN1);

    if (!something_happened)
    {
        global_last_worker++;
    }
    else
    {
        global_last_worker = 0;
    }
}
static uint8_t isr_counter = 0;
static void send_data_isr(void)
{
    interrupt_cont = true;
}

static void start_send_data_timer(void)
{
    uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(PMAN_BAUD_300);
#if defined(NRF52840_XXAA)
    // Configure timer for 1MHz (1µs per tick), 1ms intervals
    configure_timer(DATA_TIMER, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(DATA_TIMER, 0, sample_interval_us, true, true);
    set_timer_event_callback(DATA_TIMER, send_data_isr);
    start_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    // At 16MHz SMCLK with /8 prescaler = 2MHz, need 2000 ticks for 1ms
    configure_timer(DATA_TIMER, 8, MC__STOP);
    set_timer_compare(DATA_TIMER, 0, sample_interval_us * 2);
    set_timer_compare_callback(DATA_TIMER, send_data_isr);
    start_timer_with_interrupt(DATA_TIMER);
#endif
}

static void stop_send_data_timer(void)
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(DATA_TIMER);
    stop_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    stop_timer(DATA_TIMER);
    clear_timer_event_callback(DATA_TIMER);
#endif
}

/**
 * @brief Main data handshake function
 * @param pindata Pointer to PinData array
 * @param blacklist_mask Mask of pins to ignore (1 = ignore, 0 = use)
 */
DataHandshakeResult perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;
    DataHandshakeResult result;
    result.status = DATA_HANDSHAKE_INITIALIZING_FAILURE;

    analyze_pindata_events(pindata);

    if (number_of_pins == 0)
        return result; // No valid pins to use

    uint64_t valid_pins_mask = ~internal_blacklist_mask & ~0ULL;

    // Allocate and initialize
    global_datahandshake_pindata = calloc(number_of_pins, sizeof(DataHandshakeData));
    if (!global_datahandshake_pindata)
        return result;

    parallel_manchester_init(PMAN_BAUD_300);

    BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
    uint8_t pin_index, idx = 0, max_packet_size = (REQUEST_PACKSIZE > ANSWER_PACKSIZE) ? REQUEST_PACKSIZE : ANSWER_PACKSIZE;
    uint64_t valid_pins_for_fsm_mask = 0;

    uint8_t initiator_cnt = 1;

    while (bitmap_iterator_next(&it, &pin_index))
    {
        uint8_t *pin_data_buffer = calloc(max_packet_size, sizeof(uint8_t));
        if (!pin_data_buffer)
        {
            // Cleanup and exit on allocation failure
            for (uint8_t i = 0; i < idx; i++)
                free(global_datahandshake_pindata[i].data_buffer);
            free(global_datahandshake_pindata);
            global_datahandshake_pindata = NULL;
            return result;
        }

        global_datahandshake_pindata[idx] = (DataHandshakeData){
            .pin = pindata[pin_index].pin,
            .status = 0,
            .current_job = JOB_LISTEN,
            .number_of_successful_tries = 0,
            .receiving_counter = 0,
            .time_until_next_send = 0,
            .last_send_job_order = 0,
            .request_packet = NULL,
            .answer_packet = NULL,
            .data_buffer = pin_data_buffer,   // Single buffer for both request and answer
            .manchester_instance_index = 255, // Initialize as invalid
        };

        // Set role based on masks
        if (initiator_mask & (1ULL << pin_index))
        {
            dhandshake_set_role(&global_datahandshake_pindata[idx], false); // Initiator
            // If the pin is initiator, the device will send requests on this pin
            global_datahandshake_pindata[idx].time_until_next_send = get_listen_until_time(initiator_cnt*0.5f);
            initiator_cnt++;
        }
        else if (responder_mask & (1ULL << pin_index))
        {
            dhandshake_set_role(&global_datahandshake_pindata[idx], true); // Responder
        }

        uint8_t manchester_idx = parallel_manchester_add_instance(pindata[pin_index].pin, pin_data_buffer, max_packet_size);

        if (manchester_idx == 255)
        {
            LOG("Failed to create Manchester instance for pin %u\n", pindata[pin_index].pin);
            global_datahandshake_pindata[idx].manchester_instance_index = 255;
        }
        else
        {
            global_datahandshake_pindata[idx].manchester_instance_index = manchester_idx;
            LOG("Created Manchester instance %u for pin %u\n", manchester_idx, pindata[pin_index].pin);
            valid_pins_for_fsm_mask |= (1ULL << idx);
        }

        idx++;
    }

    internal_valid_pins = valid_pins_for_fsm_mask;
    LOG("Data handshake initialized \n");

    start_send_data_timer();
    while (global_last_worker < MAXIMUM_IDLE_TIME)
    {
        while (!interrupt_cont)
            ;
        interrupt_cont = false;
        pman_timer_isr();
        if (isr_counter > 10)
        {
            isr_counter = 0;
            fsm_data_handshake();
        }
        isr_counter++;
    }
    LOG("Data handshake finished due to timeout\n");
    stop_send_data_timer();

    // Cleanup
    if (global_datahandshake_pindata)
    {
        for (uint8_t i = 0; i < number_of_pins; i++)
        {
            free(global_datahandshake_pindata[i].data_buffer);
        }
        free(global_datahandshake_pindata);
        global_datahandshake_pindata = NULL;
    }
    parallel_manchester_deinit();

    result.status = DATA_HANDSHAKE_SUCCESS;
    result.mutex_pin = mutex_pin;
    result.i_am_mutex_owner = i_am_mutex_owner;
    return result;
}