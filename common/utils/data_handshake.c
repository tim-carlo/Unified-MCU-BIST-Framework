#include "data_handshake.h"
#include "bitmap_iterator.h"
#include "random_utils.h"
#include "crc.h"

static volatile uint64_t internal_blacklist_mask = 0;
static volatile bool interrupt_flag = false;
static volatile uint8_t selected_pin = 0;
static volatile uint32_t listen_until_time = 0;
static volatile uint32_t counter = 0;
static volatile uint8_t number_of_pins = 0;
static volatile uint64_t uuid = 0;

static const uint64_t all_pins_mask = (1ULL << NUMBER_OF_GPIO_PINS) - 1;

static void send_data_isr(void)
{
    interrupt_flag = true;
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
    clear_timer_event_callback(DATA_TIMER);  // NRF52-spezifisch
    stop_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    // MSP430-spezifische Timer-Cleanup
    stop_timer(DATA_TIMER);
    clear_timer_event_callback(DATA_TIMER);
#endif
}

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

// Packet format: [8 bytes UUID][1 byte Pin][4 bytes CRC]
typedef struct
{
    uint64_t uuid;
    uint8_t pin;
    crc crc_value;
} __attribute__((packed)) RequestDataPacket; // packed to avoid padding

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC]
typedef struct
{
    uint64_t received_uuid;
    uint8_t received_pin;
    uint64_t own_uuid;
    uint8_t sending_pin;
    crc crc_value;
} __attribute__((packed)) AnswerDataPacket; // packed to avoid padding

// Packet format: [8 bytes UUID][1 byte Pin][4 bytes CRC]
static void construct_request_data_packet(RequestDataPacket *packet, uint8_t *data)
{
    packet->crc_value = crcFast((uint8_t *)packet, offsetof(RequestDataPacket, crc_value));
    memcpy(data, packet, sizeof(RequestDataPacket));
}

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC]
static void construct_answer_data_packet(AnswerDataPacket *packet, uint8_t *data)
{
    packet->crc_value = crcFast((uint8_t *)&packet, offsetof(AnswerDataPacket, crc_value));
    memcpy(data, packet, sizeof(AnswerDataPacket));
}

// static void deconstruct_request_data_packet(const uint8_t *data, RequestDataPacket *packet)
// {
//     memcpy(packet, data, sizeof(RequestDataPacket));
// }

// static void deconstruct_answer_data_packet(const uint8_t *data, AnswerDataPacket *packet)
// {
//     memcpy(packet, data, sizeof(AnswerDataPacket));
// }

typedef enum
{
    STATE_IDLE,
    STATE_WAIT_FOR_ANSWER,
    STATE_FINISHED
} DataHandshakeState;
DataHandshakeState current_state = STATE_IDLE;

uint8_t *receive_buffer;
uint8_t *send_buffer;

#define REQUEST_PACKSIZE 13
#define ANSWER_PACKSIZE 22
static void fsm_data_handshake(void)
{
    switch (current_state)
    {
    case STATE_IDLE:
    {
        BitmapIterator it = bitmap_iterator_create(~internal_blacklist_mask & all_pins_mask);
        uint8_t pin;

        // Eingehende Requests prüfen
        while (bitmap_iterator_next(&it, &pin))
        {
            if (!gpio_read(pin))
            {
                manchester_set_rx_pin(pin);

                if (manchester_receive_array(receive_buffer, REQUEST_PACKSIZE))
                {
                    RequestDataPacket req;
                    memcpy(&req, receive_buffer, sizeof(RequestDataPacket));

                    crc calc_crc = crcFast((uint8_t *)&req, offsetof(RequestDataPacket, crc_value));
                    if (calc_crc == req.crc_value)
                    {
                        LOG("Received valid request on pin %u\n", pin);

                        // Antwort vorbereiten und senden
                        AnswerDataPacket ans;
                        ans.own_uuid = uuid;
                        ans.sending_pin = pin;
                        ans.received_uuid = req.uuid;
                        ans.received_pin = req.pin;
                        construct_answer_data_packet(&ans, send_buffer);
                        manchester_set_tx_pin(pin);
                        manchester_transmit_array(send_buffer, sizeof(AnswerDataPacket));

                        // Danach sofort wieder idle
                        current_state = STATE_IDLE;
                        break; // aus while, zurück zum switch
                    }
                }
            }
        }

        // If timeout reached, send new request
        if (counter > listen_until_time)
        {
            selected_pin = select_random_pin(internal_blacklist_mask, number_of_pins);
            if (selected_pin != 0xFF)
            {
                RequestDataPacket req;
                req.uuid = uuid;
                req.pin = selected_pin;
                construct_request_data_packet(&req, send_buffer);

                manchester_set_tx_pin(selected_pin);
                manchester_transmit_array(send_buffer, sizeof(RequestDataPacket));

                // Jetzt auf Antwort warten
                current_state = STATE_WAIT_FOR_ANSWER;
            }
            else
            {
                LOG("No available pins to send request\n");
                current_state = STATE_IDLE; // Stay in idle if no pins available
            }
            counter = 0;
            listen_until_time = 200 + (random32() % 800); // 200-1000ms
        }
        break;
    }

    case STATE_WAIT_FOR_ANSWER:
    {
        manchester_set_rx_pin(selected_pin);
        if (manchester_receive_array(receive_buffer, ANSWER_PACKSIZE))
        {
            AnswerDataPacket ans;
            memcpy(&ans, receive_buffer, sizeof(AnswerDataPacket));

            crc calc_crc = crcFast((uint8_t *)&ans, offsetof(AnswerDataPacket, crc_value));
            if (calc_crc == ans.crc_value)
            {
                LOG("Received valid answer on pin %u\n", selected_pin);
                internal_blacklist_mask |= (1ULL << selected_pin);
            }
            else
            {
                LOG("CRC mismatch in answer on pin %u\n", selected_pin);
            }
        }
        current_state = STATE_IDLE;
        break;
    }

    default:
        current_state = STATE_IDLE;
        break;
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

    // Calculate valid pins mask
    uint64_t all_pins_mask = (NUMBER_OF_GPIO_PINS >= 64) ? ~0ULL : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);
    uint64_t valid_pins_mask = ~internal_blacklist_mask & all_pins_mask;

    number_of_pins = __builtin_popcountll(valid_pins_mask);
    uuid = get_unique_id();

    if (number_of_pins == 0)
    {
        LOG("No valid pins available for data handshake\n");
        return;
    }

    receive_buffer = malloc(REQUEST_PACKSIZE);
    send_buffer = malloc(ANSWER_PACKSIZE);

    if (!receive_buffer || !send_buffer)
    {
        LOG("Memory allocation failed for handshake buffers\n");
        free(receive_buffer);
        free(send_buffer);
        return;
    }
    // GANZZZZZZ WICHITG FÜGE DIE PIN NUMMER AN PIN DATA ARRAY

    // Start timer
    start_send_data_timer();

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
