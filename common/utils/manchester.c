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

enum Mode { SEND, RECEIVE };

static volatile enum Mode mode = RECEIVE;
static volatile uint8_t tx_pin = 255;
static volatile uint8_t rx_pin = 255;
static const uint8_t tx_rate = 8;
static volatile uint16_t interrupt_flag = 0;

static uint8_t encoder_buffer[ENCODER_BUFFER_SIZE];
static uint8_t decoder_buffer[DECODER_BUFFER_SIZE];
static uint8_t *receive_buffer;
static struct spooky_encoder enc;
static struct spooky_decoder dec;
static bool encoder_initialized = false;
static bool decoder_initialized = false;

#if defined(NRF52840_XXAA)
static NRF_TIMER_Type *manchester_timer = NRF_TIMER4;
#elif defined(__MSP430FR5994__)
static timer_type manchester_timer = TIMER_A1;
#endif

static void set_TX(bool state)
{
    if (state) {
        gpio_od_release(tx_pin);
    } else {
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
    configure_timer(manchester_timer, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(manchester_timer, 0, sample_interval_us, true, true);
    manchester_timer->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    set_timer_event_callback(manchester_timer, manchester_timer_isr);
    start_timer(manchester_timer);

#elif defined(__MSP430FR5994__)
    uint32_t timer_ticks = (sample_interval_us * (SMCLK_HZ / 1000000UL)) - 1;
    
    configure_timer(manchester_timer, 0, MC__UP);  // No prescaler division
    set_timer_compare(manchester_timer, 0, (uint16_t)timer_ticks);
    set_timer_compare_callback(manchester_timer, manchester_timer_isr);
    start_timer_with_interrupt(manchester_timer);
#endif
}

static void manchester_stop_timer()
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(manchester_timer);
    stop_timer(manchester_timer);
    
#elif defined(__MSP430FR5994__)
    stop_timer_with_interrupt(manchester_timer);
#endif
    
    interrupt_flag = 0;
}

static void rx_cb(uint8_t *data, uint8_t data_size, void *udata)
{
    if (data_size < 2) return;

    if (receive_buffer != NULL && data_size >= 2) {
        memcpy(receive_buffer, data, data_size);
        LOG("Data copied to receive_buffer\n");
    }
}

void manchester_init(uint8_t Tx, uint8_t Rx, uint8_t rate)
{
    tx_pin = Tx;
    rx_pin = Rx;

    gpio_od_init(tx_pin);
    gpio_od_init(rx_pin);

    if (rate >= 7) return;

    uint32_t baud_rate = baud_rates[rate];
    uint32_t bit_time_us = 1000000UL / baud_rate;
    uint32_t sample_interval_us = bit_time_us / tx_rate;

    enum spooky_encoder_init_res enc_result = spooky_encoder_init(
        &enc, encoder_buffer, ENCODER_BUFFER_SIZE, tx_rate);

    enum spooky_decoder_init_res dec_result = spooky_decoder_init(
        &dec, decoder_buffer, DECODER_BUFFER_SIZE, rx_cb, NULL);

    encoder_initialized = (enc_result == SPOOKY_ENCODER_INIT_OK);
    decoder_initialized = (dec_result == SPOOKY_DECODER_INIT_OK);

    if (!encoder_initialized) LOG("Encoder init failed: %d\n", enc_result);
    if (!decoder_initialized) LOG("Decoder init failed: %d\n", dec_result);

    setup_and_start_timer(sample_interval_us);

#if DEBUG == 1
    #if defined(NRF52840_XXAA)
        gpio_output_init(11);
    #elif defined(__MSP430FR5994__)
        gpio_output_init(ABS_PIN(3, 5));
    #endif
#endif
}

bool manchester_receive_array(uint8_t *data, uint8_t size)
{
    if (rx_pin == 255 || tx_pin == 255 || !decoder_initialized || 
        data == NULL || size == 0 || size > DECODER_BUFFER_SIZE) {
        return false;
    }

    mode = RECEIVE;
    receive_buffer = data;
    memset(data, 0, size);

    setup_and_start_timer(1000);  // Use a reasonable default interval

    bool finish_decoding = false;
    uint32_t timeout_counter = 0;
    const uint32_t max_timeout = 10000000;

    while (!finish_decoding && timeout_counter < max_timeout) {
        while (!interrupt_flag && timeout_counter < max_timeout) {
            timeout_counter++;
        }

        if (timeout_counter >= max_timeout) {
            manchester_stop_timer();
            return false;
        }

        interrupt_flag = 0;
        bool rx_state = read_Rx();
        enum spooky_decoder_step_res step_result = spooky_decoder_step(&dec, rx_state);

        if (step_result == SPOOKY_DECODER_STEP_DONE) {
            finish_decoding = true;
        } else if (step_result < 0) {
            manchester_stop_timer();
            return false;
        }
    }

    manchester_stop_timer();
    return finish_decoding;
}

void manchester_transmit_array(uint8_t *data, uint8_t size)
{
    if (tx_pin == 255 || !encoder_initialized || data == NULL || 
        size == 0 || size > ENCODER_BUFFER_SIZE) {
        return;
    }

    mode = SEND;
    spooky_encoder_clear(&enc);

    if (spooky_encoder_enqueue(&enc, data, size) != SPOOKY_ENCODER_ENQUEUE_OK) {
        return;
    }

    setup_and_start_timer(1000);  

    bool transmission_complete = false;
    interrupt_flag = 0;

    while (!transmission_complete) {
        while (interrupt_flag == 0) {}
        interrupt_flag = 0;

        enum spooky_encoder_step_res step_result = spooky_encoder_step(&enc);

        switch (step_result) {
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