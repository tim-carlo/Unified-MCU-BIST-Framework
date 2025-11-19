#include "data_handshake.h"
#include "printf.h"

#if defined(NRF52840_XXAA)
#include "endian.h"
#define LOG(fmt, ...) // printf("DEBUG: " fmt, ##__VA_ARGS__)
#define TIME_CRITICAL_LOG(fmt, ...) // put here printf if needed, but be aware that this may affect timing!
#elif defined(__MSP430FR5994__)
#include "endian.h"
#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#define TIME_CRITICAL_LOG(fmt, ...) // put here printf if needed, but be aware that this may affect timing!
#endif




// Define LOG macro for LOGging (can be disabled by commenting out)
// #define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)


static const uint16_t SEND_INACCURACY = 5;

static const uint16_t INITIAL_LOW_SETTLE_TIME = 1; // Excaly one cycle

static const uint16_t INITIAL_LOW_TIME_REQUEST_MS = 10;
static const uint16_t INITIAL_LOW_TIME_REQUEST_MS_AFTER_SETTLE = INITIAL_LOW_TIME_REQUEST_MS + INITIAL_LOW_SETTLE_TIME;
static const uint16_t INITIAL_LOW_TIME_REQUEST_MIN_MS = INITIAL_LOW_TIME_REQUEST_MS - SEND_INACCURACY;
static const uint16_t INITIAL_LOW_TIME_REQUEST_MAX_MS = INITIAL_LOW_TIME_REQUEST_MS + SEND_INACCURACY;

static const uint16_t INITIAL_LOW_TIME_ANSWER_MS = 20;
static const uint16_t INITIAL_LOW_TIME_ANSWER_MS_AFTER_SETTLE = INITIAL_LOW_TIME_ANSWER_MS + INITIAL_LOW_SETTLE_TIME;
static const uint16_t INITIAL_LOW_TIME_ANSWER_MIN_MS = INITIAL_LOW_TIME_ANSWER_MS - SEND_INACCURACY;
static const uint16_t INITIAL_LOW_TIME_ANSWER_MAX_MS = INITIAL_LOW_TIME_ANSWER_MS + SEND_INACCURACY;

static const uint8_t WAIT_FOR_ANSWER_TIMEOUT = INITIAL_LOW_TIME_ANSWER_MS + SEND_INACCURACY + 50; // in cycles of maximum request time
// +50 to be sure that the answer has time to arrive and the device has enough time to process it

static const uint16_t REQEST_CYCLES_FACTOR_INFLUENCE = 50; // Needed for the random function
static const uint16_t MAXIMUM_REQUEST_CYCLES = 400;
static const uint16_t MAXIMUM_IDLE_TIME = MAXIMUM_REQUEST_CYCLES + 100; // This needs to be higher than the maximum request time

static const uint8_t MAXIMUM_NUMBER_OF_HANDSHAKE_ATTEMPTS = 3; // It could happen that a dataline is only working in one direction

static const uint8_t ANSWER_IDENTIFIER = 0x55;
static const uint8_t REQUEST_IDENTIFIER = 0xAA;

static const uint8_t REQUEST_RENEWAL_TIME = 50;

// Global variables
static PinData *global_pindata;
static DataHandshakeData *global_datahandshake_pindata = NULL; // Global pointer to DataHandshakeData array
static DataHandshakeResult handshake_result;

static uint64_t internal_blacklist_mask = 0;
static uint64_t internal_valid_pins = ~0ULL; // Assume 64-bit max
static uint64_t responder_mask = 0;
static uint64_t initiator_mask = 0;
static uint64_t uuid = 0;
static volatile uint32_t last_change_in_isr = 0;
static volatile bool interrupt_flag = false;
static volatile uint8_t isr_counter = 0;
static volatile bool isr_done = true; // must be initialized to true at start

// Used for mutex handling
static uint8_t mutex_pin = 255;                         // Pin currently holding the mutex (255 = none)
static bool i_am_mutex_owner = false;                   // Whether this device currently owns the
static volatile bool receiving_request = false; // Whether this device is currently requesting the mutex

static uint8_t number_of_pins = 0;
static bool currently_requesting_mutex = false;
/**
 * @brief This function is used to build the initiator and responder masks based on pindata events
 * The Idea in this handshake to improve its perfomance is to only listen on pins where the MCU was the responder.
 * And only send requests on pins where the MCU was the initiator.
 * @param pindata Pointer to the PinData structure containing pin events
 */
static void analyze_pindata_events(PinData *pindata)
{
    // Reset masks and counters
    initiator_mask = 0;
    responder_mask = 0;

    number_of_pins = 0;

    // Iterate through all pins that are not blacklisted
    uint64_t mask = ~internal_blacklist_mask;
    uint8_t pin;

    LOG("Analyzing pindata events for handshake role determination\n");

    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        bool has_initiator = check_if_pinevent_exists(pindata, pin, HANDSHAKE_OK_INITIATOR);
        bool has_responder = check_if_pinevent_exists(pindata, pin, HANDSHAKE_OK_RESPONDER);

        // This is needed to check if the pin is expected to work in only one direction
        // So that we can test this single direction pins properly
        // Therefore we are initiator on this pin
        bool experted_to_work_in_one_direction = check_if_pinevent_exists(pindata, pin, EXPECTS_TO_WORK_IN_ONE_DIRECTION);

        // Apply LOGic for mask assignment and blacklist update
        if ((has_initiator && !has_responder) || experted_to_work_in_one_direction)
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
            // No valid role detected: blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
        }
        else if (has_initiator && has_responder)
        {
            // No valid role detected: blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
        }
        else
        {
            // Both roles detected (should not happen): blacklist pin
            internal_blacklist_mask |= (1ULL << pin);
        }
    }
    LOG("OKE \n");
}

static inline uint16_t get_listen_until_time(uint16_t factor)
{
    uint32_t rnd = random32();
    const uint16_t jitter_range = (MAXIMUM_REQUEST_CYCLES >> 3); // MAX / 8

    uint16_t rnd16 = (uint16_t)(rnd >> 16);
    uint16_t jitter = (uint16_t)(((uint32_t)rnd16 * jitter_range) >> 16);

    // The factor scales the base time and the jitter is added on to
    uint32_t base = (uint32_t)factor * (uint32_t)REQEST_CYCLES_FACTOR_INFLUENCE;
    uint32_t total = base + jitter;

    // Ensure total is wrapped under MAXIMUM_REQUEST_CYCLES (handle multiples)
    while (total > (uint32_t)MAXIMUM_REQUEST_CYCLES)
    {
        total -= (uint32_t)MAXIMUM_REQUEST_CYCLES;
    }

    return (uint16_t)total;
}

static inline bool handle_request_receive_complete(uint8_t pin, DataHandshakeData *p)
{

    const uint8_t *restrict received_data = p->data_buffer;
    if (!received_data)
        return false;

    if (received_data[0] != REQUEST_IDENTIFIER)
        return false;

    uint32_t received_crc_le;
    memcpy(&received_crc_le, &received_data[11], sizeof(received_crc_le));
    if (crcFast(received_data, 11) != le32toh(received_crc_le))
    {
        TIME_CRITICAL_LOG("Request CRC error\n");
        return false;
    }

    uint64_t remote_uuid_le;
    memcpy(&remote_uuid_le, &received_data[1], sizeof(remote_uuid_le));
    const uint64_t remote_uuid = le64toh(remote_uuid_le);

    const uint8_t remote_pin = received_data[9];
    const uint8_t mutex_request = received_data[10];

    // check if we already have a connection to the other device on this pin
    const uint8_t remote_uuid_index = add_seen_device(remote_uuid);

    add_pin_connection(CONNECTION_TYPE_EXTERNAL, global_pindata, pin, remote_pin, remote_uuid_index);
    TIME_CRITICAL_LOG("R: Pin connection added: local_pin=%u, remote_pin=%u\n", pin, remote_pin);
    p->status |= STATUS_HANDSHAKE_SUCCESS;

    // Determine mutex state before writing to buffer
    uint8_t mutex_allowed = DENYING_MUTEX_ON_THIS_PIN;

    if (mutex_request == REQEST_MUTEX_ON_THIS_PIN && remote_uuid != uuid)
    {
        mutex_allowed = ALLOWING_MUTEX_ON_THIS_PIN;
        mutex_pin = pin;
        i_am_mutex_owner = false;
    }

    // Clear data buffer before assembling answer
    memset(p->data_buffer, 0, ANSWER_PACKSIZE);
    // Assemble the packet with a single write for the mutex field
    p->data_buffer[0] = ANSWER_IDENTIFIER;
    memcpy(&p->data_buffer[1], &remote_uuid_le, sizeof(remote_uuid_le));
    p->data_buffer[9] = remote_pin;

    const uint64_t le_own_uuid = htole64(uuid);
    memcpy(&p->data_buffer[10], &le_own_uuid, sizeof(le_own_uuid));

    p->data_buffer[18] = pin;
    p->data_buffer[19] = mutex_allowed;

    // Finalize packet with CRC
    const uint32_t crc_value = crcFast(p->data_buffer, 20);
    const uint32_t le_crc = htole32(crc_value);

    memcpy(&p->data_buffer[20], &le_crc, sizeof(le_crc));

    gpio_od_hold_low(pin);
    return true;
}

static inline bool handle_answer_complete(uint8_t pin, DataHandshakeData *p)
{
    const uint8_t *received_data = p->data_buffer;
    if (!received_data)
        return false;

    if (received_data[0] != ANSWER_IDENTIFIER)
    {
        TIME_CRITICAL_LOG("Answer not for us (wrong packet type)\n");
        return false; // Check packet type
    }

    uint64_t received_uuid_le;
    memcpy(&received_uuid_le, &received_data[1], sizeof(received_uuid_le));
    if (le64toh(received_uuid_le) != uuid)
    {
        TIME_CRITICAL_LOG("Answer not for us (UUID mismatch)");
        return false;
    }

    if (received_data[9] != pin)
    {
        TIME_CRITICAL_LOG("Answer not for us (PIN mismatch)");
        return false;
    }
    uint32_t received_crc_le;
    memcpy(&received_crc_le, &received_data[20], sizeof(received_crc_le));
    if (crcFast(received_data, 20) != le32toh(received_crc_le))
    {
        TIME_CRITICAL_LOG("Answer CRC error\n");
        return false;
    }

    uint64_t other_device_uuid_le;
    memcpy(&other_device_uuid_le, &received_data[10], sizeof(other_device_uuid_le));
    const uint64_t other_device_uuid = le64toh(other_device_uuid_le);
    const uint8_t remote_pin = received_data[18];
    const uint8_t mutex_allowed = received_data[19];

    // Get index of the other device UUID
    const uint8_t other_device_uuid_index = add_seen_device(other_device_uuid);
    add_pin_connection(CONNECTION_TYPE_EXTERNAL, global_pindata, pin, remote_pin, other_device_uuid_index);

    // The secound argument is to prevent race conditions where two devices request the mutex at the same time
    // if (mutex_allowed == ALLOWING_MUTEX_ON_THIS_PIN && mutex_pin == 255)
    if (mutex_allowed == ALLOWING_MUTEX_ON_THIS_PIN)
    {
        mutex_pin = pin;
        i_am_mutex_owner = true;
        // LOG("Mutex granted to this device on pin %u\n", pin);
    }

    p->status |= STATUS_HANDSHAKE_SUCCESS;
    TIME_CRITICAL_LOG("A: Pin connection added: local_pin=%u, remote_pin=%u\n", pin, remote_pin);
    return true;
}
static inline bool send_request_in_background(uint8_t pin, DataHandshakeData *p)
{

    // Dont request mutex if it is already held by another pin
    // Dont request mutex if a other request is ongoing to avoid collisions
    const uint8_t mutex = (mutex_pin == 255 && !receiving_request) ? REQEST_MUTEX_ON_THIS_PIN : 0;
    p->data_buffer[0] = REQUEST_IDENTIFIER;

    const uint64_t le_uuid = htole64(uuid);
    memcpy(&p->data_buffer[1], &le_uuid, sizeof(le_uuid));

    p->data_buffer[9] = pin;
    p->data_buffer[10] = mutex;

    const uint32_t crc_value = crcFast(p->data_buffer, 11);
    const uint32_t le_crc = htole32(crc_value);
    memcpy(&p->data_buffer[11], &le_crc, sizeof(le_crc));

    const bool result = parallel_manchester_transmit_background(p, REQUEST_PACKSIZE);
    return result;
}

static inline bool send_answer_in_background(uint8_t pin, DataHandshakeData *p)
{
    // The packet is already assembled in the buffer
    // So just start transmission
    return parallel_manchester_transmit_background(p, ANSWER_PACKSIZE);
}

static inline void reschedule_request(DataHandshakeData *p, uint32_t counter)
{
    // Schedule next send time to avoid immediate resend
    p->time_until_next_send = REQUEST_RENEWAL_TIME;
    p->last_send_job_order = counter; // Reset to allow immediate sending when time is up
}

static volatile uint32_t counter = 0;

static inline void fsm_data_handshake(void)
{
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
                    receiving_request = true;
                    // This only happens when there is a problem with the initialization of the manchester decoder
                    if (!parallel_manchester_receive_background(p))
                    {
                        TIME_CRITICAL_LOG("Failed to start receiving request on pin %u\n", p->pin);
                        receiving_request = false;
                    }
                    else
                    {
                        p->current_job = JOB_RECEIVING_REQUEST;
                        p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                    }
                    p->receiving_counter = 0;
                }
                // If the signal was too short or too long, just reset the counter
                else if (receive_counter > INITIAL_LOW_TIME_REQUEST_MAX_MS || receive_counter < INITIAL_LOW_TIME_REQUEST_MIN_MS)
                {
                    p->receiving_counter = 0;
                }

                // Schedule the next send if we're the initiator
                if (dhd_status_role_initiator(p) && delta >= p->time_until_next_send)
                {
                    p->handshake_attempts++;
                    if (p->handshake_attempts > MAXIMUM_NUMBER_OF_HANDSHAKE_ATTEMPTS)
                    {
                        // Too many attempts, give up on this pin for now
                        // blacklist pin internally
                        internal_valid_pins &= ~(1ULL << p->pin);
                    }
                    else
                    {
                        p->current_job = JOB_SEND_REQUEST;
                        p->last_send_job_order = counter;
                        gpio_od_hold_low(p->pin); // Start sending by pulling line low
                    }
                }
            }
            break;
        }
        case JOB_SEND_REQUEST:
        {
            if (delta > INITIAL_LOW_TIME_REQUEST_MS && delta <= INITIAL_LOW_TIME_REQUEST_MS_AFTER_SETTLE)
            {
                gpio_od_release(p->pin);
            }
            else if (delta > INITIAL_LOW_TIME_REQUEST_MS_AFTER_SETTLE)
            {
                if (!send_request_in_background(p->pin, p))
                {
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
            if (delta > INITIAL_LOW_TIME_ANSWER_MS && delta <= INITIAL_LOW_TIME_ANSWER_MS_AFTER_SETTLE)
            {
                gpio_od_release(p->pin);
            }
            else if (delta > INITIAL_LOW_TIME_ANSWER_MS_AFTER_SETTLE)
            {
                if (!send_answer_in_background(p->pin, p))
                {
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
            const bool is_complete = parallel_manchester_transmit_complete(p);

            if (is_complete)
            {
                // Transmission completed, update job state
                if (p->current_job == JOB_TRANSMITTING_REQUEST)
                {
                    p->current_job = JOB_WAIT_FOR_ANSWER;
                    p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                }
                else
                {
                    p->current_job = JOB_LISTEN;
                }
            }
            // Here is no timeout needed since the sender will always finish sending
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
                    if (!parallel_manchester_receive_background(p))
                    {
                        is_failed = true;
                        TIME_CRITICAL_LOG("Failed to start receiving answer on pin %u\n", p->pin);
                    }
                    else
                    {
                        p->current_job = JOB_RECEIVING_ANSWER;
                        p->last_send_job_order = counter; // Reset timeout counter on successful answer handling
                    }
                }
                else if (receive_counter > INITIAL_LOW_TIME_ANSWER_MAX_MS)
                {
                    // If the signal was too short or too long, just reset the counter or if timeout occurred while waiting for answer
                    // and reschedule the request
                    is_failed = true;
                }
                else
                {
                    // Check for timeout while waiting for answer
                    if (delta > WAIT_FOR_ANSWER_TIMEOUT)
                    {
                        is_failed = true;
                    }
                }

                if (is_failed)
                {
                    TIME_CRITICAL_LOG("- Failed to receive answer on pin %u\n", p->pin);
                    reschedule_request(p, counter);
                    p->current_job = JOB_LISTEN;
                }
            }

            something_happened = true;
            break;
        }
        case JOB_RECEIVING_REQUEST:
        {
            if (parallel_manchester_receive_complete(p))
            {
                if (!handle_request_receive_complete(p->pin, p))
                {
                    // Failed to handle request properly
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                    TIME_CRITICAL_LOG("Failed to handle request on pin %u\n", p->pin);
                }
                else
                {
                    // Successfully received request and prepared to send answer
                    p->current_job = JOB_SEND_ANSWER;
                    p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                }
                receiving_request = false;
            }
            else if (parallel_manchester_receive_error(p))
            {
                // Check if data in buffer can still be processed
                if (handle_request_receive_complete(p->pin, p))
                {
                    // Successfully received request and prepared to send answer
                    p->current_job = JOB_SEND_ANSWER;
                    p->last_send_job_order = counter; // Reset timeout counter on successful request handling
                }
                else
                {
                    // gpio_drive_high(DEBUG_PIN2);
                    TIME_CRITICAL_LOG("Failed to receive request on pin %u\n", p->pin);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                }
                receiving_request = false;
            }
            something_happened = true;
            break;
        }
        case JOB_RECEIVING_ANSWER:
        {
            if (parallel_manchester_receive_complete(p))
            {
                if (!handle_answer_complete(p->pin, p))
                {
                    TIME_CRITICAL_LOG("Failed to handle answer on pin %u\n", p->pin);
                }
                else
                {
                    // Successfully received answer
                    // blacklist this pin for further requests to avoid flooding
                    internal_valid_pins &= ~(1ULL << pin_index);
                }
                p->current_job = JOB_LISTEN; // Go back to listening after handling answer
            }
            else if (parallel_manchester_receive_error(p))
            {
                // Check if data in buffer can still be processed
                if (handle_answer_complete(p->pin, p))
                {
                    // Successfully received answer
                    // blacklist this pin for further requests to avoid flooding
                    internal_valid_pins &= ~(1ULL << pin_index);
                    p->current_job = JOB_LISTEN; // Go back to listening after handling answer
                }
                else
                {
                    // gpio_drive_high(DEBUG_PIN2);
                    TIME_CRITICAL_LOG("Failed to receive answer on pin %u\n", p->pin);
                    reschedule_request(p, counter);
                    p->current_job = JOB_LISTEN; // Go back to listening on failure
                }
            }
            something_happened = true;
            break;
        }
        default:
            break;
        }
    }
    // This counter is used to detect idle time in the ISR
    // If the FSM is to long idle, we can assume that the handshake is done
    if (!something_happened)
    {
        last_change_in_isr++;
    }
    else
    {
        last_change_in_isr = 0;
    }
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

static void send_data_isr(void)
{
    interrupt_flag = true;
    if (!isr_done)
    {
        // In this case, the previous ISR is not done yet
        // ISR overrun detected!
        handshake_result.status = DATA_HANDSHAKE_ISR_TO_LONG;
        // Force exit
        stop_send_data_timer();
        last_change_in_isr = MAXIMUM_IDLE_TIME;
    }
}

static void start_send_data_timer(void)
{
    const uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(20);
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

/**
 * @brief Main data handshake function
 * @param pindata Pointer to PinData array
 * @param blacklist_mask Mask of pins to ignore (1 = ignore, 0 = use)
 */
DataHandshakeResult perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;
    handshake_result.status = DATA_HANDSHAKE_INITIALIZING_FAILURE;

    analyze_pindata_events(pindata);

    if (number_of_pins == 0)
        return handshake_result; // No valid pins to use

    uint64_t valid_pins_mask = ~internal_blacklist_mask & ~0ULL;

    LOG("DEBUG: Number of pins to use for handshake: %u\n", number_of_pins);

    // Allocate and initialize
    global_datahandshake_pindata = calloc(number_of_pins, sizeof(DataHandshakeData));

    if (!global_datahandshake_pindata)
    {
        LOG("Failed to allocate memory for data handshake pindata\n");
        return handshake_result;
    }

    LOG("Allocated memory for %u pins\n", number_of_pins);
    BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
    uint8_t pin_index, idx = 0;
    uint64_t valid_pins_for_fsm_mask = 0;

    uint8_t initiator_cnt = 0;
    uuid = get_own_device_id();

    LOG("Data handshake started on %u pins\n", number_of_pins);

    while (bitmap_iterator_next(&it, &pin_index))
    {
        global_datahandshake_pindata[idx] = (DataHandshakeData){
            .pin = pindata[pin_index].pin,
            .status = 0,
            .current_job = JOB_LISTEN,
            .receiving_counter = 0,
            .time_until_next_send = 0,
            .last_send_job_order = 0};

        // Set role based on masks
        if (initiator_mask & (1ULL << pin_index))
        {
            // If the pin is initiator, the device will send requests on this pin
            global_datahandshake_pindata[idx].time_until_next_send = get_listen_until_time(initiator_cnt);
            dhd_set_role_initiator(&global_datahandshake_pindata[idx], true);
            initiator_cnt++;
        }
        else if (responder_mask & (1ULL << pin_index))
        {
            // If the pin is responder, the device will only listen on this pin
            dhd_set_role_initiator(&global_datahandshake_pindata[idx], false);
        }

        bool manchester_create = parallel_manchester_add_instance(&global_datahandshake_pindata[idx]);

        if (!manchester_create)
        {
            LOG("Failed to create Manchester instance for pin %u\n", pindata[pin_index].pin);
        }
        else
        {
            valid_pins_for_fsm_mask |= (1ULL << idx);
        }

        idx++;
    }

    internal_valid_pins = valid_pins_for_fsm_mask;
    LOG("Data handshake initialized \n");

    start_send_data_timer();

    while (last_change_in_isr < MAXIMUM_IDLE_TIME)
    {
        // The ISR is executed in the main thread context, to avoid crashes that can occur if ISRs overlap
        while (!interrupt_flag)
            ;

        interrupt_flag = false;
        // This flag is used to indicate that the ISR is being processed, preventing re-entrancy
        isr_done = false;
        pman_timer_isr(global_datahandshake_pindata, number_of_pins);
        if (isr_counter > 10)
        {
            isr_counter = 0;
            fsm_data_handshake();
        }
        isr_counter++;
        isr_done = true;
    }
    if (handshake_result.status == DATA_HANDSHAKE_ISR_TO_LONG)
    {
        if (global_datahandshake_pindata)
        {
            free(global_datahandshake_pindata);
            global_datahandshake_pindata = NULL;
        }
        // Early exit due to ISR taking too long
        return handshake_result;
    }

    LOG("Data handshake finished due to timeout\n");
    stop_send_data_timer();
    if (global_datahandshake_pindata)
    {
        LOG("Successful handshakes on pins:\n");
        bool any_success = false;
        for (uint8_t i = 0; i < number_of_pins; i++)
        {
            if (dhd_status_handshake_success(&global_datahandshake_pindata[i]))
            {
                LOG(" - %u\n", global_datahandshake_pindata[i].pin);
                add_pin_event(global_pindata, global_datahandshake_pindata[i].pin, DATA_HANDSHAKE_OK);
                any_success = true;
            }
            else
            {
                // add failure
                add_pin_event(global_pindata, global_datahandshake_pindata[i].pin, DATA_HANDSHAKE_FAILURE);
            }
        }
        if (!any_success)
        {
            LOG(" - none\n");
        }
    }

    // Cleanup
    if (global_datahandshake_pindata)
    {
        free(global_datahandshake_pindata);
        global_datahandshake_pindata = NULL;
    }

    handshake_result.status = DATA_HANDSHAKE_SUCCESS;
    handshake_result.mutex_pin = mutex_pin;
    handshake_result.i_am_mutex_owner = i_am_mutex_owner;
    return handshake_result;
}