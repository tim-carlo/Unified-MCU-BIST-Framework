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
static DataHandshakeData *global_datahandshake_pindata = NULL; // Global pointer to DataHandshakeData array

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

/**
 * @brief This function is used to build the initiator and responder masks based on pindata events
 * The Idea in this handshake to improve its perfomance is to only listen on pins where the MCU was the responder.
 * And only send requests on pins where the MCU was the initiator.
 * @param pindata
 */
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

static void log_request_data_packet(const RequestDataPacket *packet)
{
    LOG("RequestDataPacket: uuid=0x%016llx, pin=%u, hash=0x%08x\n",
        (unsigned long long)packet->uuid,
        packet->pin,
        packet->crc_value);
}

static void log_answer_data_packet(const AnswerDataPacket *packet)
{
    LOG("AnswerDataPacket: received_uuid=0x%016llx, received_pin=%u, own_uuid=0x%016llx, sending_pin=%u, hash=0x%08x\n",
        (unsigned long long)packet->received_uuid,
        packet->received_pin,
        (unsigned long long)packet->own_uuid,
        packet->sending_pin,
        packet->crc_value);
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

static void fsm_data_handshake(void)
{
    gpio_drive_high(DEBUG_PIN1);

    for (uint8_t i = 0; i < number_of_pins; i++)
    {
        DataHandshakeData *data = &global_datahandshake_pindata[i];
        uint8_t pin = data->pin;
        bool pin_state = gpio_read(pin);

        if (pin_state)
        {
            if (data->receiving_counter >= INITIAL_LOW_TIME_REQUEST_MIN_MS && data->receiving_counter <= INITIAL_LOW_TIME_REQUEST_MAX_MS)
            {
                // Detected valid request signal


                set_received_request(data, true);
                data->receiving_counter = 0; // Reset counter after valid detection
            }
            else if (data->receiving_counter >= INITIAL_LOW_TIME_ANSWER_MIN_MS && data->receiving_counter <= INITIAL_LOW_TIME_ANSWER_MAX_MS)
            {
                // Detected valid answer signal
                set_received_answer(data, true);
                LOG("Pin %u: Detected valid ANSWER signal\n", pin);
                data->receiving_counter = 0; // Reset counter after valid detection
            } else if (data->receiving_counter > INITIAL_LOW_TIME_ANSWER_MAX_MS) {
                data->receiving_counter = 0; // Reset counter if signal is too long
            }
        }
        else
        {
            // Signal is low, increment counter (open-drain low)
            data->receiving_counter++;
        }
    }
    gpio_drive_low(DEBUG_PIN1);
}
/**
 * @brief Main data handshake function
 * @param pindata Pointer to PinData array
 * @param blacklist_mask Mask of pins to ignore (1 = ignore, 0 = use)
 */
void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask)
{
    internal_blacklist_mask = blacklist_mask;
    global_pindata = pindata;

    // Calculate valid pins mask
    uint64_t valid_pins_mask = ~internal_blacklist_mask & all_pins_mask;
    printf("Valid pins mask: 0x%016llX\n", valid_pins_mask);

    analyze_pindata_events(pindata);

    number_of_pins = __builtin_popcountll(valid_pins_mask);
    printf("Number of active pins: %u\n", number_of_pins);
    uuid = get_unique_id();

    if (number_of_pins == 0)
    {
        LOG("No valid pins available for data handshake\n");
        return;
    }

    // Allocate or reallocate global DataHandshakeData array
    free(global_datahandshake_pindata);
    global_datahandshake_pindata = calloc(number_of_pins, sizeof(DataHandshakeData));
    if (!global_datahandshake_pindata)
    {
        LOG("Failed to allocate DataHandshakeData array\n");
        return; // Allocation failed
    }

    // Initialize DataHandshakeData for each valid pin
    BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
    uint8_t pin_index, idx = 0;
    while (bitmap_iterator_next(&it, &pin_index))
    {
        uint8_t physical_pin = pindata[pin_index].pin;
        global_datahandshake_pindata[idx] = (DataHandshakeData){
            .pin = physical_pin,
            .status = 0,
            .number_of_successful_tries = 0,
            .receiving_counter = 0,
            .last_send_counter = 0,
            .last_crc = 0};

        // Set role based on masks
        if (initiator_mask & (1ULL << pin_index))
        {
            set_role(&global_datahandshake_pindata[idx], false); // Initiator
        }
        else if (responder_mask & (1ULL << pin_index))
        {
            set_role(&global_datahandshake_pindata[idx], true); // Responder
        }

        idx++;
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

    // Cleanup
    free(global_datahandshake_pindata);
    global_datahandshake_pindata = NULL;
}
