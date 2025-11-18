#include "mutex_handeler.h"
#include "bitmap_iterator.h"
#include "datahandshake_modulation.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#if defined(NRF52840_XXAA)
#define MUTEX_TIMER NRF_TIMER4
#elif defined(__MSP430FR5994__)
#define MUTEX_TIMER TIMER_A1
#endif

#define DEBUG 1
#if DEBUG == 1
#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

static const uint8_t MAX_RELEASE_TRIES = 4;
static const uint8_t MAX_REQUEST_TRIES = 40;
static volatile bool interrupt_flag = false;
static const uint16_t max_timeout = 5000;

/**
 * @brief Timer interrupt handler for mutex operations
 *
 */
static void mutex_timer_interrupt(void)
{
    interrupt_flag = true;
}

/**
 * @brief Start the mutex timer with appropriate configuration
 *
 */
static void mutex_timer_start(void)
{
    // Must be configured in the same way as the spooky encoder/decoder sample rate in the data handshake

    const uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(20);
#if defined(NRF52840_XXAA)
    // Configure timer for 1MHz (1µs per tick), 1ms intervals
    configure_timer(DATA_TIMER, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(DATA_TIMER, 0, sample_interval_us, true, true);
    set_timer_event_callback(DATA_TIMER, mutex_timer_interrupt);
    start_timer(DATA_TIMER);

#elif defined(__MSP430FR5994__)
    // At 16MHz SMCLK with /8 prescaler = 2MHz, need 2000 ticks for 1ms
    configure_timer(DATA_TIMER, 8, MC__STOP);
    set_timer_compare(DATA_TIMER, 0, sample_interval_us * 2);
    set_timer_compare_callback(DATA_TIMER, mutex_timer_interrupt);
    start_timer_with_interrupt(DATA_TIMER);
#endif
}

/**
 * @brief Stop the mutex timer
 *
 */
static void mutex_timer_stop(void)
{
#if defined(NRF52840_XXAA) || defined(__MSP430FR5994__)
    clear_timer_event_callback(MUTEX_TIMER);
#endif
    stop_timer(MUTEX_TIMER);
    interrupt_flag = false;
}

/**
 * @brief Wait for a signal on the mutex pin
 *
 * @param handler Pointer to MutexHandler
 * @return WaitForResult Result of the wait operation
 */
static WaitForResult mutex_wait_for_signal(MutexHandler *handler)
{
    uint8_t received = 0;
    uint16_t timeout_counter = 0;

    WaitForResult result = TIMEOUT;

    reset_decoder(&handler->dec);
    memset(handler->buffer, 0, sizeof(handler->buffer));

    while (true)
    {
        while (!interrupt_flag)
            ;
        interrupt_flag = false;

        bool rx = gpio_read(handler->current_mutex_pin);

        enum spooky_decoder_step_res step = spooky_decoder_step(&handler->dec, rx);

        // Error detection
        const uint8_t current_mode = handler->dec.mode;
        const uint8_t last_mode = handler->last_mode;

        if (current_mode != last_mode)
        {
            // if (current_mode < last_mode && last_mode != 3)
            // {
            //     // Here a error occures
            //     result = TIMEOUT;
            //     goto done_wait;
            // }
        }
        else
        {
            if (rx == handler->last_rx)
            {
                if (++timeout_counter >= max_timeout)
                {
                    result = TIMEOUT;
                    goto done_wait;
                }
            }
            else
            {
                timeout_counter = 0;
            }
            handler->last_rx = rx;
        }
        handler->last_mode = current_mode;

        switch (step)
        {
        case SPOOKY_DECODER_STEP_DONE:
            received = handler->buffer[0];
            LOG("Received mutex signal: %u\n", received);
            switch (received)
            {
            case MUTEX_ACK:
                result = ACKNOWLEDGED;
                break;
            case MUTEX_RELEASE:
                result = RELEASED;
                break;
            case MUTEX_REQUEST:
                result = REQUESTED;
                break;
            default:
                result = TIMEOUT;
                break;
            }
            goto done_wait;
        case SPOOKY_DECODER_STEP_OK:
            break;
        default:
            result = TIMEOUT;
            goto done_wait;
        }
        gpio_drive_low(DEBUG_PIN1);
    }

done_wait:
    return result;
}

/**
 * @brief Send a signal using the mutex handler's encoder
 *
 * @param handler Pointer to MutexHandler
 * @param data Data byte to send
 * @return true
 * @return false
 */
static void mutex_send_signal(MutexHandler *handler, uint8_t data)
{
    bool done = false;
    spooky_encoder_clear(&handler->enc);
    spooky_encoder_enqueue(&handler->enc, &data, 1);

    while (!done)
    {
        while (!interrupt_flag)
            ;
        interrupt_flag = false;

        enum spooky_encoder_step_res step_result = spooky_encoder_step(&handler->enc);
        switch (step_result)
        {
        case SPOOKY_ENCODER_STEP_OK_DONE:
            done = true;
            break;
        case SPOOKY_ENCODER_STEP_OK:
            // Keep line as is
            break;
        case SPOOKY_ENCODER_STEP_OK_HIGH:
            gpio_od_release(handler->current_mutex_pin);
            break;
        case SPOOKY_ENCODER_STEP_OK_LOW:
            gpio_od_hold_low(handler->current_mutex_pin);
            break;
        default:
            done = true;
            break;
        }
    }
}

void mutex_handler_init(DataHandshakeResult *result, MutexHandler *handler)
{
    if (result == NULL || handler == NULL)
        return;

    handler->current_mutex_pin = result->mutex_pin;
    handler->iam_mutex_owner = result->i_am_mutex_owner;
    handler->currently_having_mutex = false;

    if (spooky_encoder_init(&handler->enc, handler->buffer, sizeof(handler->buffer)) != SPOOKY_ENCODER_INIT_OK)
    {
        LOG("Encoder init failed\n");
        return;
    }
    if (spooky_decoder_init(&handler->dec, handler->buffer, sizeof(handler->buffer)) != SPOOKY_DECODER_INIT_OK)
    {
        LOG("Decoder init failed\n");
        return;
    }

    LOG("Mutex handler initialized on pin %u, iam_owner=%d\n",
        handler->current_mutex_pin,
        handler->iam_mutex_owner);

    mutex_timer_start();
}

void mutex_handler_request_mutex(uint64_t blacklist_mask, MutexHandler *handler)
{
    if (handler == NULL)
        return;

    if (handler->iam_mutex_owner && handler->currently_having_mutex)
        return;

    if (handler->iam_mutex_owner && !handler->currently_having_mutex)
    {
        // Owner requesting mutex, this is needed to make sure other devices know we have it
        // Only important at the start, later on we just assume we have it
        // This is important that both devices have the same
        gpio_od_init(handler->current_mutex_pin);
        uint8_t tries = 0;
        while (!handler->currently_having_mutex)
        {
            if (tries++ >= MAX_REQUEST_TRIES)
            {
                LOG("Max request tries reached\n");
                break;
            }

            uint8_t request = MUTEX_REQUEST;
            mutex_send_signal(handler, request);

            WaitForResult res = mutex_wait_for_signal(handler);
            if (res == ACKNOWLEDGED)
            {
                LOG("Received mutex ACK\n");
                handler->currently_having_mutex = true;
                break;
            }
        }
        gpio_reset_from_blacklist(blacklist_mask);
    }
    else if (!handler->iam_mutex_owner)
    {
        gpio_input_init(handler->current_mutex_pin, GPIO_PULL_NONE);
        while (!handler->currently_having_mutex)
        {
            WaitForResult wf = mutex_wait_for_signal(handler);

            if (wf == TIMEOUT)
                continue;

            if (wf == REQUESTED)
            {
                LOG("Received mutex request\n");
                uint8_t ack = MUTEX_ACK;
                //delay_ms(10); // Small delay_ms to ensure the other device is ready
                gpio_od_init(handler->current_mutex_pin);
                mutex_send_signal(handler, ack);

                gpio_reset_from_blacklist(blacklist_mask);
            }
            else if (wf == RELEASED)
            {
                LOG("Received mutex release\n");
                handler->currently_having_mutex = true;
                uint8_t ack = MUTEX_ACK;
                //delay_ms(10); // Small delay_ms to ensure the other device is ready
                gpio_od_init(handler->current_mutex_pin);
                mutex_send_signal(handler, ack);
                gpio_reset_from_blacklist(blacklist_mask);
                break;
            }
        }
    }
}

void mutex_handler_release_mutex(uint64_t blacklist_mask, MutexHandler *handler)
{
    if (handler == NULL || !handler->currently_having_mutex)
        return;

    gpio_od_init(handler->current_mutex_pin);
    bool released = false;
    uint8_t tries = 0;

    while (!released)
    {
        if (tries++ >= MAX_RELEASE_TRIES)
        {
            LOG("Max release tries reached\n");
            break;
        }

        uint8_t release = MUTEX_RELEASE;
        LOG("Releasing mutex\n");
        mutex_send_signal(handler, release);

        WaitForResult res = mutex_wait_for_signal(handler);
        if (res == ACKNOWLEDGED)
        {
            released = true;
            handler->currently_having_mutex = false;
        }
    }

    gpio_reset_from_blacklist(blacklist_mask);
}

void mutex_handler_deinit(MutexHandler *handler)
{
    if (!handler)
        return;
    mutex_timer_stop();
}