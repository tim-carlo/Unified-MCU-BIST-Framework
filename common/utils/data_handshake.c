#include "data_handshake.h"

#if defined(NRF52840_XXAA)
#include "endian.h"
#elif defined(__MSP430FR5994__)
#include "endian.h"
#endif
#include <string.h>

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

#define SEND_INACCURACY 20 // Acceptable inaccuracy in ms for timing checks
#define INITIAL_LOW_TIME_REQUEST_MS 50
#define INITIAL_LOW_TIME_REQUEST_MIN_MS (INITIAL_LOW_TIME_REQUEST_MS - SEND_INACCURACY)
#define INITIAL_LOW_TIME_REQUEST_MAX_MS (INITIAL_LOW_TIME_REQUEST_MS + SEND_INACCURACY)

#define INITIAL_LOW_TIME_ANSWER_MS 100
#define INITIAL_LOW_TIME_ANSWER_MIN_MS (INITIAL_LOW_TIME_ANSWER_MS - SEND_INACCURACY)
#define INITIAL_LOW_TIME_ANSWER_MAX_MS (INITIAL_LOW_TIME_ANSWER_MS + SEND_INACCURACY)

// Global variables
PinData *global_pindata;

static uint64_t internal_blacklist_mask = 0;
static uint64_t responder_mask = 0;
static uint64_t initiator_mask = 0;
static uint64_t uuid = 0;
static uint32_t listen_until_time = 0;
static uint32_t counter = 0;
static volatile bool interrupt_flag = false;

static uint8_t number_of_pins = 0;
static uint8_t selected_pin = 0;

static const uint64_t all_pins_mask = (1ULL << NUMBER_OF_GPIO_PINS) - 1;

static void analyze_pindata_events(PinData *pindata)
{
    // Reset masks
    initiator_mask = 0;
    responder_mask = 0;

    // Iterate through all pins that are not blacklisted
    BitmapIterator it = bitmap_iterator_create(~internal_blacklist_mask & all_pins_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        // Check events for this pin
        for (uint8_t i = 0; i < pindata[pin].event_index && i < EVENT_BUFFER_SIZE; i++)
        {
            PinEventType event = pindata[pin].pin_event[i];

            switch (event)
            {
            case HANDSHAKE_OK_INITIATOR:
                // Set bit in initiator mask
                initiator_mask |= (1ULL << pin);
                LOG("Pin %u set as INITIATOR (event: %d)\n", pin, event);
                break;

            case HANDSHAKE_OK_RESPONDER:
                // Set bit in responder mask
                responder_mask |= (1ULL << pin);
                LOG("Pin %u set as RESPONDER (event: %d)\n", pin, event);
                break;
            default:
                // Other events are ignored for role determination
                break;
            }
        }
    }
    LOG("Blacklist mask: 0x%016llx \n", (unsigned long long)internal_blacklist_mask);
    LOG("Initiator mask: 0x%016llx \n", (unsigned long long)initiator_mask);
    LOG("Responder mask: 0x%016llx \n", (unsigned long long)responder_mask);
}

static void send_data_isr(void)
{
    gpio_toggle(DEBUG_PIN1); // Toggle debug pin to indicate ISR entry
    interrupt_flag = true;
}

static uint32_t get_listen_until_time()
{
    return 200 + (random32() % 10000); // 200-10000ms
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

static PackageType get_package_type_from_data(const uint8_t *data)
{
    if (data[0] == 0xAA)
        return PACKET_TYPE_REQUEST;
    else if (data[0] == 0xFF)
        return PACKET_TYPE_ANSWER;
    else
        return (PackageType)-1; // Invalid type
}

static void log_request_data_packet(const RequestDataPacket *packet)
{
    LOG("RequestDataPacket: uuid=0x%016llx, pin=%u, hash=0x%08x\n",
        (unsigned long long)packet->uuid,
        packet->pin,
        packet->hash_value);
}

static void log_answer_data_packet(const AnswerDataPacket *packet)
{
    LOG("AnswerDataPacket: received_uuid=0x%016llx, received_pin=%u, own_uuid=0x%016llx, sending_pin=%u, hash=0x%08x\n",
        (unsigned long long)packet->received_uuid,
        packet->received_pin,
        (unsigned long long)packet->own_uuid,
        packet->sending_pin,
        packet->hash_value);
}

// Packet format: [1 byte type][8 bytes UUID][1 byte Pin][4 bytes CRC32]
static void construct_request_data_packet(RequestDataPacket *packet, uint8_t *data)
{
    data[0] = 0xAA; // Packet type
    uint64_t le_uuid = htole64(packet->uuid);
    memcpy(&data[1], &le_uuid, sizeof(le_uuid));
    data[9] = packet->pin;
    packet->hash_value = crcFast((unsigned char const*)data, 10); 
    uint32_t le_hash = htole32(packet->hash_value);
    memcpy(&data[10], &le_hash, sizeof(le_hash));
}

// Packet format: [1 byte type][8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC32]
static void construct_answer_data_packet(AnswerDataPacket *packet, uint8_t *data)
{
    data[0] = 0xFF; // Packet type
    uint64_t le_received_uuid = htole64(packet->received_uuid);
    memcpy(&data[1], &le_received_uuid, sizeof(le_received_uuid));
    data[9] = packet->received_pin;
    uint64_t le_own_uuid = htole64(packet->own_uuid);
    memcpy(&data[10], &le_own_uuid, sizeof(le_own_uuid));
    data[18] = packet->sending_pin;
    packet->hash_value = crcFast((unsigned char const*)data, 19); // Calculate CRC32 over type, received_uuid, received_pin, own_uuid, sending_pin
    uint32_t le_hash = htole32(packet->hash_value);
    memcpy(&data[19], &le_hash, sizeof(le_hash));
}

static bool deconstruct_request_data_packet(const uint8_t *data, RequestDataPacket *packet)
{
    if (data[0] != 0xAA)
    {
        return false;
    }
    uint64_t le_uuid;
    memcpy(&le_uuid, &data[1], sizeof(le_uuid));
    packet->uuid = le64toh(le_uuid);
    packet->pin = data[9];
    uint32_t le_hash;
    memcpy(&le_hash, &data[10], sizeof(le_hash));
    packet->hash_value = le32toh(le_hash);
    return true;
}

static bool deconstruct_answer_data_packet(const uint8_t *data, AnswerDataPacket *packet)
{
    if (data[0] != 0xFF)
    {
        return false;
    }
    uint64_t le_received_uuid;
    memcpy(&le_received_uuid, &data[1], sizeof(le_received_uuid));
    packet->received_uuid = le64toh(le_received_uuid);
    packet->received_pin = data[9];
    uint64_t le_own_uuid;
    memcpy(&le_own_uuid, &data[10], sizeof(le_own_uuid));
    packet->own_uuid = le64toh(le_own_uuid);
    packet->sending_pin = data[18];
    uint32_t le_hash;
    memcpy(&le_hash, &data[19], sizeof(le_hash));
    packet->hash_value = le32toh(le_hash);
    return true;
}

uint8_t *receive_request_buffer;
uint8_t *receive_answer_buffer;
uint8_t *send_request_buffer;
uint8_t *send_answer_buffer;

typedef enum
{
    SEND_ANSWER_JOB,
    SEND_REQUEST_JOB,
    NO_JOB
} BackgroundJobType;

BackgroundJobType current_job = NO_JOB;
uint32_t trigger_job_time = 0;

bool send_initial_low_phase = false;
bool is_waiting_for_response = false;

uint8_t receiving_counter = 0;
uint8_t response_receiving_counter = 0;
uint8_t last_resonse_receiving_pin = 255;
uint8_t last_receiving_pin = 255;


static void fsm_data_handshake(void)
{

    gpio_drive_high(DEBUG_PIN1);
    if (current_job != SEND_ANSWER_JOB)
    {
        BitmapIterator responder_iterator = bitmap_iterator_create(responder_mask & all_pins_mask);
        uint8_t responder_pin;
        while (bitmap_iterator_next(&responder_iterator, &responder_pin))
        {
            bool value = gpio_read(responder_pin);
            if (!value)
            {
                if (last_receiving_pin == 255 || last_receiving_pin != responder_pin)
                {
                    last_receiving_pin = responder_pin;
                    receiving_counter = 0;
                }
                receiving_counter++;
            }
        }
        if (receiving_counter >= INITIAL_LOW_TIME_REQUEST_MIN_MS && receiving_counter <= INITIAL_LOW_TIME_REQUEST_MAX_MS)
        {
            receiving_counter = 0;
            manchester_set_rx_pin(last_receiving_pin);

            bool decoder_result = manchester_receive_array(receive_request_buffer, REQUEST_PACKSIZE);
            if (!decoder_result)
                return;
            RequestDataPacket req;

            LOG("Received buffer: ");
            for (int i = 0; i < REQUEST_PACKSIZE; i++)
            {
                LOG("%02X ", receive_request_buffer[i]);
            }
            LOG("\n");

            if (deconstruct_request_data_packet(receive_request_buffer, &req))
            {
                log_request_data_packet(&req);
                add_pin_event(global_pindata, req.pin, DATA_HANDSHAKE_OK);
                LOG("Received valid request from pin %u\n", req.pin);

                // Send answer
                AnswerDataPacket ans = {
                    .own_uuid = uuid,
                    .sending_pin = last_receiving_pin,
                    .received_uuid = req.uuid,
                    .received_pin = req.pin};
                selected_pin = last_receiving_pin;
                construct_answer_data_packet(&ans, send_answer_buffer);
                current_job = SEND_ANSWER_JOB;
                trigger_job_time = counter + INITIAL_LOW_TIME_ANSWER_MS; // Hold low for 100ms
                gpio_od_hold_low(last_receiving_pin);
            }
        }
    }

    if (is_waiting_for_response && manchester_transmit_in_background_complete())
    {
        gpio_drive_high(DEBUG_PIN2);
        BitmapIterator initiator_iterator = bitmap_iterator_create(initiator_mask & all_pins_mask);
        uint8_t initiator_pin;
        while (bitmap_iterator_next(&initiator_iterator, &initiator_pin))
        {
            bool value = gpio_read(initiator_pin);
            if (!value)
            {
                if (response_receiving_counter == 255 || last_resonse_receiving_pin != initiator_pin)
                {
                    last_resonse_receiving_pin = initiator_pin;
                    response_receiving_counter = 0;
                }
                response_receiving_counter++;
            }
        }
        if (response_receiving_counter >= INITIAL_LOW_TIME_ANSWER_MIN_MS && response_receiving_counter <= INITIAL_LOW_TIME_ANSWER_MAX_MS)
        {

            response_receiving_counter = 0;
            manchester_set_rx_pin(last_resonse_receiving_pin);
            bool decoder_result = manchester_receive_array(receive_answer_buffer, ANSWER_PACKSIZE);
            LOG("Received answer buffer: ");
            for (int i = 0; i < ANSWER_PACKSIZE; i++)
            {
                LOG("%02X ", receive_answer_buffer[i]);
            }
            if (!decoder_result)
                return;
            AnswerDataPacket ans;
            if (deconstruct_answer_data_packet(receive_answer_buffer, &ans))
            {
                log_answer_data_packet(&ans);
                if (ans.received_uuid == uuid)
                {
                    add_pin_event(global_pindata, ans.received_pin, DATA_HANDSHAKE_OK);
                    LOG("Received valid answer on pin %u from pin %u\n", ans.received_pin, ans.sending_pin);
                    is_waiting_for_response = false;
                }
            }
        }
        gpio_drive_low(DEBUG_PIN2);
    }

    // If timeout reached, send new request
    if (counter >= listen_until_time && current_job == NO_JOB)
    {

        selected_pin = select_random_pin(~initiator_mask);

        LOG("Timeout reached, sending request on pin %u\n", selected_pin);
        if (selected_pin != 255)
        {
            gpio_od_hold_low(selected_pin);
            current_job = SEND_REQUEST_JOB;
            RequestDataPacket req;
            req.uuid = uuid;
            req.pin = selected_pin;
            construct_request_data_packet(&req, send_request_buffer);
            trigger_job_time = counter + INITIAL_LOW_TIME_REQUEST_MS; // Hold low for 50ms
        }
        else
        {
            LOG("No available pins to send request\n");
            counter = 0;
            listen_until_time = get_listen_until_time(); // Set new random listen time
        }
    }
    else if (counter >= trigger_job_time && current_job == SEND_REQUEST_JOB)
    {
        gpio_drive_high(DEBUG_PIN2);
        gpio_od_release(selected_pin);
        current_job = NO_JOB;

        manchester_set_tx_pin(selected_pin);

        manchester_transmit_array_in_background(send_request_buffer, REQUEST_PACKSIZE);
        is_waiting_for_response = true;
        counter = 0;
        listen_until_time = get_listen_until_time();
        gpio_drive_low(DEBUG_PIN2);
    }
    if (counter >= trigger_job_time && current_job == SEND_ANSWER_JOB)
    {

        gpio_drive_high(DEBUG_PIN2);
        gpio_od_release(selected_pin);
        current_job = NO_JOB;

        manchester_set_tx_pin(selected_pin);
        manchester_transmit_array(send_answer_buffer, ANSWER_PACKSIZE);
        gpio_drive_low(DEBUG_PIN2);
    }

    counter++;
    gpio_drive_low(DEBUG_PIN1);
}

/**
 * @brief Updated data handshake function that sends UUID package
 */
void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;

    // Calculate valid pins mask
    uint64_t valid_pins_mask = ~internal_blacklist_mask & all_pins_mask;

    uint8_t request_data_size = sizeof(RequestDataPacket);
    uint8_t answer_data_size = sizeof(AnswerDataPacket);

    LOG("RequestDataPacket size: %u bytes\n", request_data_size);
    LOG("AnswerDataPacket size: %u bytes\n", answer_data_size);

    analyze_pindata_events(pindata);

    number_of_pins = __builtin_popcountll(valid_pins_mask);
    uuid = get_unique_id();

    if (number_of_pins == 0)
    {
        LOG("No valid pins available for data handshake\n");
        return;
    }

    // receive_buffer = malloc(REQUEST_PACKSIZE);
    send_request_buffer = malloc(REQUEST_PACKSIZE);
    send_answer_buffer = malloc(ANSWER_PACKSIZE);

    receive_request_buffer = malloc(REQUEST_PACKSIZE);
    receive_answer_buffer = malloc(ANSWER_PACKSIZE);

    if (!send_request_buffer || !send_answer_buffer || !receive_request_buffer || !receive_answer_buffer)
    {
        LOG("Memory allocation failed for buffers\n");
        free(send_request_buffer);
        free(send_answer_buffer);
        free(receive_request_buffer);
        free(receive_answer_buffer);
        return;
    }
    // set initial listening time
    listen_until_time = get_listen_until_time();
    LOG("Initial listen time: %lu ms\n", (unsigned long)listen_until_time);

    // Start timer
    start_send_data_timer();
    manchester_init(BAUD_1200);

    while (true)
    {
        while (!interrupt_flag)
            ;
        interrupt_flag = false;
        fsm_data_handshake();
    }

    stop_send_data_timer();

    // Clean up
    free(receive_request_buffer);
    free(receive_answer_buffer);
    free(send_request_buffer);
    free(send_answer_buffer);
    receive_request_buffer = NULL;
    receive_answer_buffer = NULL;
    send_request_buffer = NULL;
    send_answer_buffer = NULL;
}
