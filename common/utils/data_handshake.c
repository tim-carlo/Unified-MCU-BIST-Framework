#include "data_handshake.h"

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

#define REQUEST_PACKSIZE 13
#define ANSWER_PACKSIZE 22

// Global variables
PinData *global_pindata;

static volatile uint64_t internal_blacklist_mask = 0;
static volatile uint64_t responder_mask = 0;
static volatile uint64_t initiator_mask = 0;
static volatile uint64_t uuid = 0;
static volatile bool interrupt_flag = false;
static volatile uint32_t listen_until_time = 0;
static volatile uint32_t counter = 0;


static volatile uint8_t number_of_pins = 0;
static volatile uint8_t selected_pin = 0;

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
    LOG("RequestDataPacket: uuid=0x%016llx, pin=%u, crc=0x%08x\n",
        (unsigned long long)packet->uuid,
        packet->pin,
        packet->crc_value);
}

static void log_answer_data_packet(const AnswerDataPacket *packet)
{
    LOG("AnswerDataPacket: received_uuid=0x%016llx, received_pin=%u, own_uuid=0x%016llx, sending_pin=%u, crc=0x%08x\n",
        (unsigned long long)packet->received_uuid,
        packet->received_pin,
        (unsigned long long)packet->own_uuid,
        packet->sending_pin,
        packet->crc_value);
}

// Packet format: [8 bytes UUID][1 byte Pin][4 bytes CRC]
static void construct_request_data_packet(RequestDataPacket *packet, uint8_t *data)
{
    // Set first byte to 0xAA (1010 1010) for packet type
    uint8_t *packet_bytes = (uint8_t *)packet;
    packet_bytes[0] = 0xAA;

    packet->crc_value = crcFast((uint8_t *)packet, offsetof(RequestDataPacket, crc_value));
    memcpy(data, packet, sizeof(RequestDataPacket));
}

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC]
static void construct_answer_data_packet(AnswerDataPacket *packet, uint8_t *data)
{
    // Set first byte to 0xFF (255) for packet type
    uint8_t *packet_bytes = (uint8_t *)packet;
    packet_bytes[0] = 0xFF;

    packet->crc_value = crcFast((uint8_t *)packet, offsetof(AnswerDataPacket, crc_value));
    memcpy(data, packet, sizeof(AnswerDataPacket));
}

static bool deconstruct_request_data_packet(const uint8_t *data, RequestDataPacket *packet)
{
    if (data[0] != 0xAA)
    {
        return false; // Invalid packet type
    }
    // Skip the first byte (packet type)
    memcpy(packet, data + 1, sizeof(RequestDataPacket));
    return true;
}

static bool deconstruct_answer_data_packet(const uint8_t *data, AnswerDataPacket *packet)
{
    if (data[0] != 0xFF)
    {
        return false; // Invalid packet type
    }
    // Skip the first byte (packet type)
    memcpy(packet, data + 1, sizeof(AnswerDataPacket));
    return true;
}

bool disable_isr = false;
static void rising_edge_isr(uint8_t pin)
{
}
uint8_t *receive_buffer;
uint8_t *send_buffer;
static void falling_edge_isr(uint8_t pin)
{
    if (disable_isr)
        return;
    if (!gpio_read(pin))
        return; // Debounce
    gpio_toggle(DEBUG_PIN1);

    disable_isr = true;
    manchester_set_rx_pin(pin);

    if (!manchester_receive_array(receive_buffer, REQUEST_PACKSIZE))
    {
        disable_isr = false;
        return;
    }

    switch (get_package_type_from_data(receive_buffer))
    {
    case PACKET_TYPE_REQUEST:
    {
        const RequestDataPacket *req = (const RequestDataPacket *)(receive_buffer + 1);
        // crc calc_crc = crcFast((const uint8_t *)req, offsetof(RequestDataPacket, crc_value));
        // if (calc_crc != req->crc_value)
        //     break;

        AnswerDataPacket ans = {
            .own_uuid = uuid,
            .sending_pin = pin,
            .received_uuid = req->uuid,
            .received_pin = req->pin};
        construct_answer_data_packet(&ans, send_buffer);
        manchester_set_tx_pin(pin);
        manchester_transmit_array(send_buffer, sizeof(AnswerDataPacket));
        break;
    }

    case PACKET_TYPE_ANSWER:
    {
        const AnswerDataPacket *ans = (const AnswerDataPacket *)(receive_buffer + 1);
        // crc calc_crc = crcFast((const uint8_t *)ans, offsetof(AnswerDataPacket, crc_value));
        // if (calc_crc != ans->crc_value)
        //     break;

        if (ans->received_uuid == uuid)
        {
            add_pin_event(global_pindata, ans->received_pin, DATA_HANDSHAKE_OK);
            LOG("Received valid answer on pin %u from pin %u\n", ans->received_pin, ans->sending_pin);
        }
        break;
    }
    default:
        break;
    }
    disable_isr = false;
}

bool send_initial_low_phase = false;

static void fsm_data_handshake(void)
{
    

    // If timeout reached, send new request
    if (counter >= listen_until_time && !send_initial_low_phase)
    {
        selected_pin = select_random_pin(~initiator_mask);

        LOG("Timeout reached, sending request on pin %u\n", selected_pin);
        if (selected_pin != 255)
        {
            gpio_od_hold_low(selected_pin);
            send_initial_low_phase = true;
            listen_until_time = counter + 50; // Hold low for 50ms
        }
        else
        {
            LOG("No available pins to send request\n");
            counter = 0;
            listen_until_time = get_listen_until_time(); // Set new random listen time
            LOG("New listen time: %lu ms\n", (unsigned long)listen_until_time);
        }
    }
    else if (counter >= listen_until_time && send_initial_low_phase)
    {

        disable_isr = true;
        gpio_od_release(selected_pin);
        send_initial_low_phase = false;

        RequestDataPacket req;
        req.uuid = uuid;
        req.pin = selected_pin;
        construct_request_data_packet(&req, send_buffer);

        manchester_set_tx_pin(selected_pin);

        manchester_transmit_array(send_buffer, sizeof(RequestDataPacket));
        disable_isr = false;
        counter = 0;
        listen_until_time = get_listen_until_time();
    }
    counter++;
}

uint32_t initial_delay = 5000; // Maximum initial listening delay

/**
 * @brief Updated data handshake function that sends UUID package
 */
void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;

    // Calculate valid pins mask
    uint64_t valid_pins_mask = ~internal_blacklist_mask & all_pins_mask;

    analyze_pindata_events(pindata);
    gpio_listen_on_all_pins_interrupt(internal_blacklist_mask, rising_edge_isr, falling_edge_isr);

    number_of_pins = __builtin_popcountll(valid_pins_mask);
    uuid = get_unique_id();

    if (number_of_pins == 0)
    {
        LOG("No valid pins available for data handshake\n");
        return;
    }

    // receive_buffer = malloc(REQUEST_PACKSIZE);
    send_buffer = malloc(REQUEST_PACKSIZE);

    if (!send_buffer)
    {
        LOG("Memory allocation failed for send_buffer buffer\n");
        free(send_buffer);
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
    free(receive_buffer);
    free(send_buffer);
    receive_buffer = NULL;
    send_buffer = NULL;
}
