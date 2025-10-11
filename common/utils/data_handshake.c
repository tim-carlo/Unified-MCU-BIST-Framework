#include "data_handshake.h"

#if defined(NRF52840_XXAA)
#include "endian.h"
#elif defined(__MSP430FR5994__)
#include "endian.h"
#endif
#include <string.h>

// Define LOG macro for logging (can be disabled by commenting out)
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
// #define LOG(fmt, ...) // Uncomment this line to disable logging

#define SEND_INACCURACY 30 // Acceptable inaccuracy in ms for timing checks
#define INITIAL_LOW_TIME_REQUEST_MS 50
#define INITIAL_LOW_TIME_REQUEST_MIN_MS (INITIAL_LOW_TIME_REQUEST_MS - SEND_INACCURACY)
#define INITIAL_LOW_TIME_REQUEST_MAX_MS (INITIAL_LOW_TIME_REQUEST_MS + SEND_INACCURACY)

#define INITIAL_LOW_TIME_ANSWER_MS 100
#define INITIAL_LOW_TIME_ANSWER_MIN_MS (INITIAL_LOW_TIME_ANSWER_MS - SEND_INACCURACY)
#define INITIAL_LOW_TIME_ANSWER_MAX_MS (INITIAL_LOW_TIME_ANSWER_MS + SEND_INACCURACY)
#define TIMEOUT_CYCLES 10000
#define TIMEOUT_CYCLES_SENDING_REQUEST 10000
#define TIMEOUT_CYCLES_SENDING_ANSWER 20000

// Global variables
PinData *global_pindata;
static DataHandshakeData *global_datahandshake_pindata = NULL; // Global pointer to DataHandshakeData array

static uint64_t internal_blacklist_mask = 0;
static uint64_t internal_valid_pins = ~0ULL; // Assume 64-bit max
static uint64_t responder_mask = 0;
static uint64_t initiator_mask = 0;
static uint64_t uuid = 0;
static volatile bool interrupt_cont = false;

static uint8_t number_of_pins = 0;

// Job name mapping for logging
static const char *job_names[] = {
    "JOB_LISTEN",
    "JOB_SEND_REQUEST",
    "JOB_SEND_ANSWER",
    "JOB_WAIT_FOR_ANSWER",
    "JOB_TRANSMITTING_ANSWER",
    "JOB_TRANSMITTING_REQUEST"};

/**
 * @brief Log job transitions for debugging
 * @param pin Pin number for context
 * @param old_job Previous job
 * @param new_job New job
 */
static void log_job_transition(uint8_t pin, CurrentJobType old_job, CurrentJobType new_job)
{
    // if (old_job != new_job) {
    //     LOG("Pin %u: %s(%d) -> %s(%d)\n",
    //         pin,
    //         job_names[old_job], old_job,
    //         job_names[new_job], new_job);
    // }
}

// all_pins_mask will be calculated at runtime

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
        bool has_initiator = false;
        bool has_responder = false;

        // Analyze all events for this pin
        for (uint8_t i = 0; i < pindata[pin].event_index && i < EVENT_BUFFER_SIZE; i++)
        {
            PinEventType event = pindata[pin].pin_event[i];

            switch (event)
            {
            case HANDSHAKE_OK_INITIATOR:
                has_initiator = true;
                break;

            case HANDSHAKE_OK_RESPONDER:
                has_responder = true;
                break;

            default:
                break;
            }
        }

        // Apply logic for mask assignment and blacklist update
        if (has_initiator && !has_responder)
        {
            initiator_mask |= (1ULL << pin);
            LOG("Pin %u set as INITIATOR\n", pin);
            number_of_pins++;
        }
        else if (has_responder && !has_initiator)
        {
            responder_mask |= (1ULL << pin);
            LOG("Pin %u set as RESPONDER\n", pin);
            number_of_pins++;
        }
        else if (!has_initiator && !has_responder)
        {
            // No valid role detected → blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
            LOG("Pin %u has no valid event → BLACKLISTED\n", pin);
        }
        else
        {
            // Both roles detected (should not happen) → blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
            LOG("Pin %u has conflicting roles (INIT+RESP) → BLACKLISTED\n", pin);
        }
    }

    LOG("Blacklist mask:  0x%016llx\n", (unsigned long long)internal_blacklist_mask);
    LOG("Initiator mask:  0x%016llx\n", (unsigned long long)initiator_mask);
    LOG("Responder mask:  0x%016llx\n", (unsigned long long)responder_mask);
    LOG("Number of active pins: %u\n", number_of_pins);
}

static uint16_t get_listen_until_time()
{
    uint16_t random_offset = (uint16_t)(random32() % 10000) + 1000; // Random offset between 1000 and 10000 ms
    return random_offset;
}

static void send_data_isr(void)
{
    interrupt_cont = true;
}

static void start_send_data_timer(void)
{
#if defined(NRF52840_XXAA)
    // Configure timer for 1MHz (1µs per tick), 1ms intervals
    configure_timer(DATA_TIMER, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(DATA_TIMER, 0, DATA_TIMER_INTERVAL_US, true, true);
    set_timer_event_callback(DATA_TIMER, send_data_isr);
    start_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    // Configure timer for 1ms intervals
    // At 16MHz SMCLK with /8 prescaler = 2MHz, need 2000 ticks for 1ms
    configure_timer(DATA_TIMER, 8, MC__STOP);
    set_timer_compare(DATA_TIMER, 0, DATA_TIMER_INTERVAL_US * 2);
    set_timer_compare_callback(DATA_TIMER, send_data_isr);
    start_timer_with_interrupt(DATA_TIMER);
#endif
}

static void stop_send_data_timer(void)
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(DATA_TIMER); // NRF52-spezifisch
    stop_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    // MSP430-spezifische Timer-Cleanup
    stop_timer(DATA_TIMER);
    clear_timer_event_callback(DATA_TIMER);
#endif
}

// Logging functions removed to save memory

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

// Packet format: [1 byte type][8 bytes UUID][1 byte Pin][4 bytes CRC]
static void construct_request_data_packet(RequestDataPacket *packet, uint8_t *data)
{
    data[0] = 0xAA; // Packet type
    write_u64_le(&data[1], packet->uuid);
    data[9] = packet->pin;
    packet->crc_value = crcFast(data, 10); // Calculate CRC over type, uuid, and pin
    write_u32_le(&data[10], packet->crc_value);
}

// Packet format: [1 byte type][8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC]
static void construct_answer_data_packet(AnswerDataPacket *packet, uint8_t *data)
{
    data[0] = 0xFF; // Packet type
    write_u64_le(&data[1], packet->received_uuid);
    data[9] = packet->received_pin;
    write_u64_le(&data[10], packet->own_uuid);
    data[18] = packet->sending_pin;
    packet->crc_value = crcFast(data, 19); // Calculate CRC over type, received_uuid, received_pin, own_uuid, sending_pin
    write_u32_le(&data[19], packet->crc_value);
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
    packet->crc_value = read_u32_le(&data[10]);
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
    packet->crc_value = read_u32_le(&data[19]);
    return true;
}

typedef enum
{
    NONE,
    SENDING_REQUEST,
    DISCARDING_REQUEST,
    SENDING_ANSWER,
} DataHandshakeGlobalLock;

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

    // if the uuid is my own, ignore it
    if (request_packet.uuid == uuid)
    {
        return false;
    }
    // Add a pin connection to the request packet
    add_pin_connection(&global_pindata[pin], pin, request_packet.pin, request_packet.uuid);
    AnswerDataPacket answer_packet;

    // Prepare answer packet using static structure to avoid memory leaks
    answer_packet.received_uuid = request_packet.uuid;
    answer_packet.received_pin = request_packet.pin;
    answer_packet.own_uuid = uuid;
    answer_packet.sending_pin = pin;
    answer_packet.crc_value = 0; // Will be calculated later

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

    // Mark successful handshake
    dhandshake_set_received_answer(p, true);
    dhandshake_set_successful_handshake(p, true);

    // Add pin connection to pindata events
    PinData *pindata = &global_pindata[pin];
    uint64_t other_device_id = answer_packet.own_uuid;
    add_pin_connection(pindata, pin, answer_packet.received_pin, other_device_id);
    p->number_of_successful_tries++;
    printf("Pin connection added: local_pin=%u, remote_pin=%u\n", pin, answer_packet.sending_pin);
    return true;
}

static bool send_request_in_background(uint8_t pin, DataHandshakeData *p)
{
    gpio_od_release(pin);

    // Small delay to ensure line is released before transmitting
    delay_us(5000);

    // Mark that we're sending a request
    dhandshake_set_send_request(p, true);
    RequestDataPacket request_packet;
    request_packet.uuid = uuid;
    request_packet.pin = pin;
    request_packet.crc_value = 0; // Will be calculated in construct function

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
    delay_us(5000);

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
    p->time_until_next_send = get_listen_until_time();
    p->last_send_job_order = counter; // Reset to allow immediate sending when time is up
}

static uint32_t counter = 0;

static inline void fsm_data_handshake(void)
{
    gpio_drive_high(DEBUG_PIN1);
    counter++;

    BitmapIterator it = bitmap_iterator_create(internal_valid_pins);
    uint8_t pin_index;

    while (bitmap_iterator_next(&it, &pin_index))
    {

        DataHandshakeData *p = &global_datahandshake_pindata[pin_index];
        uint8_t pin = p->pin;
        uint32_t delta = counter - p->last_send_job_order;

        switch (p->current_job)
        {
        case JOB_LISTEN:
        {
            const bool is_low = !gpio_read(pin);
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
                        printf("Failed to start receiving request on pin %u\n", pin);
                    }
                    else
                    {
                        log_job_transition(pin, p->current_job, JOB_RECEIVING_REQUEST);
                        p->current_job = JOB_RECEIVING_REQUEST;
                        p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                    }
                    // printf("Request received on pin %u\n", pin);

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

                    log_job_transition(pin, p->current_job, JOB_SEND_REQUEST);
                    p->current_job = JOB_SEND_REQUEST;
                    p->last_send_job_order = counter;
                    gpio_od_hold_low(pin); // Start sending by pulling line low
                }
            }
            break;
        }
        case JOB_SEND_REQUEST:
        {
            if (delta > INITIAL_LOW_TIME_REQUEST_MS)
            {
                if (!send_request_in_background(pin, p))
                {
                    dhandshake_set_failed_handshake(p, true);
                    reschedule_request(p, counter); // reschedule in any case to avoid immediate resend
                    log_job_transition(pin, p->current_job, JOB_LISTEN);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                }
                else
                {
                    log_job_transition(pin, p->current_job, JOB_TRANSMITTING_REQUEST);
                    p->current_job = JOB_TRANSMITTING_REQUEST;
                }
                p->last_send_job_order = counter; // Reset timeout counter on successful request handling
            }
            break;
        }
        case JOB_SEND_ANSWER:
        {
            if (delta > INITIAL_LOW_TIME_ANSWER_MS)
            {
                if (!send_answer_in_background(pin, p))
                {
                    dhandshake_set_failed_handshake(p, true);
                    log_job_transition(pin, p->current_job, JOB_LISTEN);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                }
                else
                {
                    log_job_transition(pin, p->current_job, JOB_TRANSMITTING_ANSWER);
                    p->current_job = JOB_TRANSMITTING_ANSWER;
                }
                p->last_send_job_order = counter; // Reset timeout counter on successful request handling
            }
            break;
        }
        case JOB_TRANSMITTING_REQUEST:
        case JOB_TRANSMITTING_ANSWER:
        {
            const uint8_t manchester_idx = p->manchester_instance_index;
            bool is_complete = parallel_manchester_transmit_complete(manchester_idx);

            if (is_complete)
            {
                // Transmission completed, update job state
                if (p->current_job == JOB_TRANSMITTING_REQUEST)
                {
                    log_job_transition(pin, p->current_job, JOB_WAIT_FOR_ANSWER);
                    p->current_job = JOB_WAIT_FOR_ANSWER;
                }
                else
                {
                    log_job_transition(pin, p->current_job, JOB_LISTEN);
                    p->current_job = JOB_LISTEN;
                }
            }
            // Timeout for transmission if complete signal not received
            else if (p->current_job == JOB_TRANSMITTING_REQUEST && delta > TIMEOUT_CYCLES_SENDING_REQUEST)
            {
                // Timeout occurred during transmission
                dhandshake_set_failed_handshake(p, true);
                reschedule_request(p, counter);
                log_job_transition(pin, p->current_job, JOB_LISTEN);
                p->current_job = JOB_LISTEN; // Go back to listening on timeout
            }
            else if (p->current_job == JOB_TRANSMITTING_ANSWER && delta > TIMEOUT_CYCLES_SENDING_ANSWER)
            {
                // Timeout occurred during transmission
                dhandshake_set_failed_handshake(p, true);
                log_job_transition(pin, p->current_job, JOB_LISTEN);
                p->current_job = JOB_LISTEN; // Go back to listening on timeout
            }
            break;
        }
        case JOB_WAIT_FOR_ANSWER:
        {
            const bool is_low = !gpio_read(pin);
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
                        printf("Failed to start receiving answer on pin %u\n", pin);
                    }
                    else
                    {
                        log_job_transition(pin, p->current_job, JOB_RECEIVING_ANSWER);
                        p->current_job = JOB_RECEIVING_ANSWER;
                        p->last_send_job_order = counter; // Reset timeout counter on successful answer handling
                    }
                }
                // If the signal was too short or too long, just reset the counter or if timeout occurred while waiting for answer
                // and reschedule the request
                else if (receive_counter > INITIAL_LOW_TIME_ANSWER_MAX_MS || receive_counter < INITIAL_LOW_TIME_REQUEST_MIN_MS || delta > TIMEOUT_CYCLES)
                {
                    is_failed = true;
                }
                if (is_failed)
                {
                    // printf("Failed to receive answer on pin %u\n", pin);
                    reschedule_request(p, counter);
                    dhandshake_set_failed_handshake(p, true);
                    log_job_transition(p->pin, p->current_job, JOB_LISTEN);
                    p->current_job = JOB_LISTEN;
                }
            }
            break;
        }
        case JOB_RECEIVING_REQUEST:
        {
            gpio_drive_high(DEBUG_PIN2);
            const uint8_t manchester_idx = p->manchester_instance_index;
            if (parallel_manchester_receive_complete(manchester_idx))
            {

                if (!handle_request_receive_complete(pin, p))
                {
                    // Failed to handle request properly
                    dhandshake_set_failed_handshake(p, true);
                    log_job_transition(pin, p->current_job, JOB_LISTEN);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                    printf("Failed to handle request on pin %u\n", pin);
                }
                else
                {
                    printf("Request received on pin %u\n", pin);
                    // Successfully received request and prepared to send answer
                    log_job_transition(pin, p->current_job, JOB_SEND_ANSWER);
                    p->current_job = JOB_SEND_ANSWER;

                    p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                }
            }
            else if (parallel_manchester_receive_error(manchester_idx))
            {
                dhandshake_set_failed_handshake(p, true);
                log_job_transition(pin, p->current_job, JOB_LISTEN);
                printf("Failed to receive request on pin %u\n", pin);
                p->current_job = JOB_LISTEN; // Go back to listening on failure
            }
            gpio_drive_low(DEBUG_PIN2);
            break;
        }
        case JOB_RECEIVING_ANSWER:
        {
            const uint8_t manchester_idx = p->manchester_instance_index;
            if (parallel_manchester_receive_complete(manchester_idx))
            {
                if (!handle_answer_complete(pin, p))
                {
                    dhandshake_set_failed_handshake(p, false);
                }
                else
                {
                    // Successfully received answer
                    // blacklist this pin for further requests to avoid flooding
                    internal_valid_pins &= ~(1ULL << pin_index);
                }
                log_job_transition(pin, p->current_job, JOB_LISTEN);
                p->current_job = JOB_LISTEN; // Go back to listening after handling answer
            }
            else if (parallel_manchester_receive_error(manchester_idx))
            {
                dhandshake_set_failed_handshake(p, false);
                printf("Failed to receive answer on pin %u\n", pin);
                reschedule_request(p, counter);
                log_job_transition(pin, p->current_job, JOB_LISTEN);
                p->current_job = JOB_LISTEN; // Go back to listening on failure
            }
            break;
        }
        default:
            break;
        }
    }
    gpio_drive_low(DEBUG_PIN1);
}

volatile bool manchester_busy_flag = false;
volatile bool manchester_transmitting_request = false;
/**
 * @brief Main data handshake function
 * @param pindata Pointer to PinData array
 * @param blacklist_mask Mask of pins to ignore (1 = ignore, 0 = use)
 */
void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;

    analyze_pindata_events(pindata);

    // Calculate valid pins mask - use 64 as safe maximum
    const uint64_t all_pins_mask = ~0ULL; // All 64 bits set
    // Shift the mask until the first 1 is at the least significant bit
    uint64_t valid_pins_mask = ~internal_blacklist_mask & all_pins_mask;

    if (number_of_pins == 0)
    {
        return; // No valid pins
    }

    // Allocate global DataHandshakeData array
    global_datahandshake_pindata = calloc(number_of_pins, sizeof(DataHandshakeData));
    if (!global_datahandshake_pindata)
    {
        return; // Allocation failed
    }

    // Initialize DataHandshakeData for each valid pin
    BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
    uint8_t pin_index, idx = 0;

    // Initialize parallel Manchester system
    parallel_manchester_init(PMAN_BAUD_100);

    // Gerate a valid pin mask for the fsm to now which pins where successfully initialized
    uint64_t valid_pins_for_fsm_mask = 0;

    while (bitmap_iterator_next(&it, &pin_index))
    {
        uint8_t physical_pin = pindata[pin_index].pin;

        // Allocate single buffer for this pin (used for both request and answer)
        // Use the larger of the two packet sizes to ensure sufficient space
        uint8_t max_packet_size = (REQUEST_PACKSIZE > ANSWER_PACKSIZE) ? REQUEST_PACKSIZE : ANSWER_PACKSIZE;
        uint8_t *pin_data_buffer = calloc(max_packet_size, sizeof(uint8_t));

        if (!pin_data_buffer)
        {
            // Cleanup previously allocated buffers
            for (uint8_t cleanup_idx = 0; cleanup_idx < idx; cleanup_idx++)
            {
                if (global_datahandshake_pindata[cleanup_idx].data_buffer)
                {
                    free(global_datahandshake_pindata[cleanup_idx].data_buffer);
                }
            }
            free(global_datahandshake_pindata);
            global_datahandshake_pindata = NULL;
            return; // Allocation failed
        }

        global_datahandshake_pindata[idx] = (DataHandshakeData){
            .pin = physical_pin,
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
            global_datahandshake_pindata[idx].time_until_next_send = get_listen_until_time();
        }
        else if (responder_mask & (1ULL << pin_index))
        {
            dhandshake_set_role(&global_datahandshake_pindata[idx], true); // Responder
        }

        uint8_t *const buffer = global_datahandshake_pindata[idx].data_buffer;
        uint8_t manchester_idx = parallel_manchester_add_instance(physical_pin, buffer, max_packet_size);

        if (manchester_idx == 255)
        {
            LOG("Failed to create Manchester instance for pin %u\n", physical_pin);
            global_datahandshake_pindata[idx].manchester_instance_index = 255; // Mark as invalid
        }
        else
        {
            global_datahandshake_pindata[idx].manchester_instance_index = manchester_idx;
            LOG("Created Manchester instance %u for pin %u\n", manchester_idx, physical_pin);
            valid_pins_for_fsm_mask |= (1ULL << idx); // Mark this pin as valid for FSM
        }

        idx++;
    }

    printf("Data handshake initialized \n");

    internal_valid_pins = valid_pins_for_fsm_mask;

    // Initialization complete - Start timer
    start_send_data_timer();

    while (true)
    {
        while (!interrupt_cont)
        {
        }
        interrupt_cont = false;
        fsm_data_handshake();
    }
    stop_send_data_timer();

    // Cleanup
    if (global_datahandshake_pindata)
    {
        // Cleanup Manchester instances and free buffers
        for (uint8_t i = 0; i < number_of_pins; i++)
        {
            if (global_datahandshake_pindata[i].manchester_instance_index != 255)
            {
                parallel_manchester_remove_instance(global_datahandshake_pindata[i].manchester_instance_index);
            }

            // Free the single allocated buffer
            if (global_datahandshake_pindata[i].data_buffer)
            {
                free(global_datahandshake_pindata[i].data_buffer);
                global_datahandshake_pindata[i].data_buffer = NULL;
            }

            // Clear packet pointers
            global_datahandshake_pindata[i].request_packet = NULL;
            global_datahandshake_pindata[i].answer_packet = NULL;
        }

        free(global_datahandshake_pindata);
        global_datahandshake_pindata = NULL;
    }

    // Deinitialize parallel Manchester system
    parallel_manchester_deinit();
}
