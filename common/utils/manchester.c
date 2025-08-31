#include "manchester.h"
#include "printf.h"
#include <string.h>

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"
#endif



#define ENCODER_BUFFER_SIZE 32
#define DECODER_BUFFER_SIZE 32
#define DEBUG 1



#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

enum Mode
{
    SEND,
    RECEIVE
};

static volatile enum Mode mode = RECEIVE;
static volatile uint8_t tx_pin = 255;
static volatile uint8_t rx_pin = 255;
static volatile uint16_t interrupt_flag = 0;

static uint8_t encoder_buffer[ENCODER_BUFFER_SIZE];
static uint8_t decoder_buffer[DECODER_BUFFER_SIZE];
static uint8_t *receive_buffer;
static struct spooky_encoder enc;
static struct spooky_decoder dec;
static bool encoder_initialized = false;
static bool decoder_initialized = false;

#if defined(NRF52840_XXAA)
#define MANCHESTER_TIMER NRF_TIMER4
#elif defined(__MSP430FR5994__)
#define MANCHESTER_TIMER TIMER_A1
#endif

static void set_TX(bool state)
{
    if (state)
    {
        gpio_od_release(tx_pin);
    }
    else
    {
        gpio_od_hold_low(tx_pin);
    }
}

static bool read_Rx()
{
    return gpio_read(rx_pin);
}

static void manchester_timer_isr(void)
{
#if DEBUG == 1
#if defined(NRF52840_XXAA)
    NRF_P0->OUT ^= (1 << 11);
#elif defined(__MSP430FR5994__)
    P3OUT ^= BIT5;
#endif
#endif
    interrupt_flag = 1;
}

static void setup_and_start_timer(uint16_t sample_interval_us)
{
    LOG("Setting up timer with sample interval: %u us\n", sample_interval_us);

#if defined(NRF52840_XXAA)
    // Use constant prescaler 4 (1MHz)
    configure_timer(MANCHESTER_TIMER, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(MANCHESTER_TIMER, 0, sample_interval_us, true, true);
    MANCHESTER_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    set_timer_event_callback(MANCHESTER_TIMER, manchester_timer_isr);
    start_timer(MANCHESTER_TIMER);

#elif defined(__MSP430FR5994__)
    uint32_t timer_ticks = (sample_interval_us * (SMCLK_HZ / 1000000UL)) - 1;

    configure_timer(MANCHESTER_TIMER, 0, MC__UP); // No prescaler division
    set_timer_compare(MANCHESTER_TIMER, 0, (uint16_t)timer_ticks);
    set_timer_compare_callback(MANCHESTER_TIMER, manchester_timer_isr);
    start_timer_with_interrupt(MANCHESTER_TIMER);
#endif
}

static void manchester_stop_timer()
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(MANCHESTER_TIMER);
#elif defined(__MSP430FR5994__)
    clear_timer_event_callback(MANCHESTER_TIMER);
#endif
    stop_timer(MANCHESTER_TIMER);
    interrupt_flag = 0;
}

static void rx_cb(uint8_t *data, uint8_t data_size, void *udata)
{
    if (data_size < 2)
        return;

    if (receive_buffer != NULL && data_size >= 2)
    {
        memcpy(receive_buffer, data, data_size);
        LOG("Data copied to receive_buffer\n");
    }
}

uint32_t get_sample_interval_us(BaudRate rate)
{
    uint32_t bit_time_us = 1000000UL / rate;
    return bit_time_us / TX_RATE;
}

void manchester_set_rx_pin(uint8_t pin)
{
    rx_pin = pin;
}

void manchester_set_tx_pin(uint8_t pin)
{
    tx_pin = pin;
}

void manchester_set_rx_pin_od(uint8_t pin)
{
    rx_pin = pin;
    gpio_od_init(rx_pin);
}

void manchester_set_tx_pin_od(uint8_t pin)
{
    tx_pin = pin;
    gpio_od_init(tx_pin);
    set_TX(false); // Start with line held low
}

void manchester_init(BaudRate rate)
{
    uint32_t sample_interval_us = get_sample_interval_us(rate);

    enum spooky_encoder_init_res enc_result = spooky_encoder_init(
        &enc, encoder_buffer, ENCODER_BUFFER_SIZE, TX_RATE);

    enum spooky_decoder_init_res dec_result = spooky_decoder_init(
        &dec, decoder_buffer, DECODER_BUFFER_SIZE, rx_cb, NULL);

    encoder_initialized = (enc_result == SPOOKY_ENCODER_INIT_OK);
    decoder_initialized = (dec_result == SPOOKY_DECODER_INIT_OK);

    if (!encoder_initialized)
        LOG("Encoder init failed: %d\n", enc_result);
    if (!decoder_initialized)
        LOG("Decoder init failed: %d\n", dec_result);

    setup_and_start_timer(sample_interval_us);

#if DEBUG == 1
#if defined(NRF52840_XXAA)
    gpio_output_init(11);
#elif defined(__MSP430FR5994__)
    gpio_output_init(ABS_PIN(3, 5));
#endif
#endif
}
void manchester_deinit()
{
    manchester_stop_timer();
    rx_pin = 255;
    tx_pin = 255;
    encoder_initialized = false;
    decoder_initialized = false;
}

bool manchester_receive_array(uint8_t *data, uint8_t size)
{
    if (rx_pin == 255 || tx_pin == 255 || !decoder_initialized ||
        data == NULL || size == 0 || size > DECODER_BUFFER_SIZE)
    {
        return false;
    }

    mode = RECEIVE;
    receive_buffer = data;
    memset(data, 0, size);


    bool finish_decoding = false;
    uint32_t timeout_counter = 0;
    const uint32_t max_timeout = 10000000;

    while (!finish_decoding && timeout_counter < max_timeout)
    {
        while (!interrupt_flag && timeout_counter < max_timeout)
        {
            timeout_counter++;
        }

        if (timeout_counter >= max_timeout)
        {
            return false;
        }

        interrupt_flag = 0;
        bool rx_state = read_Rx();
        enum spooky_decoder_step_res step_result = spooky_decoder_step(&dec, rx_state);

        if (step_result == SPOOKY_DECODER_STEP_DONE)
        {
            finish_decoding = true;
        }
        else if (step_result < 0)
        {
            return false;
        }
    }
    return finish_decoding;
}

void manchester_transmit_array(uint8_t *data, uint8_t size)
{
    if (tx_pin == 255 || !encoder_initialized || data == NULL ||
        size == 0 || size > ENCODER_BUFFER_SIZE)
    {
        return;
    }

    mode = SEND;
    spooky_encoder_clear(&enc);

    if (spooky_encoder_enqueue(&enc, data, size) != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        return;
    }

    setup_and_start_timer(1000);

    bool transmission_complete = false;
    interrupt_flag = 0;

    while (!transmission_complete)
    {
        while (interrupt_flag == 0)
        {
        }
        interrupt_flag = 0;

        enum spooky_encoder_step_res step_result = spooky_encoder_step(&enc);

        switch (step_result)
        {
        case SPOOKY_ENCODER_STEP_OK_DONE:
            set_TX(false);
            transmission_complete = true;
            break;
        case SPOOKY_ENCODER_STEP_OK_LOW:
            set_TX(false);
            break;
        case SPOOKY_ENCODER_STEP_OK_HIGH:
            set_TX(true);
            break;
        case SPOOKY_ENCODER_STEP_OK:
            break;
        default:
            transmission_complete = true;
            break;
        }
    }

    manchester_stop_timer();
    set_TX(false);
    interrupt_flag = 0;
}