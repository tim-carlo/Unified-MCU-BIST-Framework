#include "datahandshake_modulation.h"
#include "bitmap_iterator.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>

// #define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)

#if defined(NRF52840_XXAA)
#define PMAN_TIMER NRF_TIMER4
#define DEBUG_PIN_ABS 38 // Pin 1.6
#elif defined(__MSP430FR5994__)
#define PMAN_TIMER TIMER_B0 // This has according to the datasheet a higherr resolution and more features and higher priority than TIMER_A2
#define DEBUG_PIN_ABS ABS_PIN(3, 0)
#endif


bool dhd_status_tx_complete(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_TX_COMPLETE) != 0;
}
bool dhd_status_rx_complete(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_RX_COMPLETE) != 0;
}
bool dhd_status_rx_error(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_RX_ERROR) != 0;
}
bool dhd_status_data_received(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_DATA_RECEIVED) != 0;
}
bool dhd_status_role_initiator(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_ROLE_INITIATOR) != 0;
}
bool dhd_status_handshake_success(const DataHandshakeData *dhd)
{
    return (dhd->status & STATUS_HANDSHAKE_SUCCESS) != 0;
}
ParallelManchesterMode dhd_get_manchester_mode(const DataHandshakeData *dhd)
{
    return (ParallelManchesterMode)((dhd->status & STATUS_MANCHESTER_MASK) >> 4);
}

void dhd_set_manchester_mode(DataHandshakeData *dhd, ParallelManchesterMode mode)
{
    dhd->status = (dhd->status & ~STATUS_MANCHESTER_MASK) | ((mode & 0x03) << 4);
}
void dhd_set_role_initiator(DataHandshakeData *dhd, bool is_initiator)
{
    if (is_initiator)
    {
        dhd->status |= STATUS_ROLE_INITIATOR;
    }
    else
    {
        dhd->status &= ~STATUS_ROLE_INITIATOR;
    }
}

static inline void pman_set_TX(const bool state, const uint8_t pin)
{
    if (state)
    {
        gpio_od_release(pin);
    }
    else
    {
        gpio_od_hold_low(pin);
    }
}
static uint64_t set_one_mask = 0;
static uint64_t set_zero_mask = 0;

inline void pman_timer_isr(DataHandshakeData *dhd_instances, uint8_t pman_instance_count)
{
    const uint8_t count = pman_instance_count;
    // Apply all pin changes at once for better timing accuracy
    // So that it does not matter how long spooky takes per instance
    // bit mapiteration
    uint8_t pin;
    BitmapIterator set_one_iter = bitmap_iterator_create(set_one_mask);
    while (bitmap_iterator_next(&set_one_iter, &pin))
    {
        pman_set_TX(true, pin);
        set_one_mask &= ~(1ULL << pin);
    }

    BitmapIterator set_zero_iter = bitmap_iterator_create(set_zero_mask);
    while (bitmap_iterator_next(&set_zero_iter, &pin))
    {
        pman_set_TX(false, pin);
        set_zero_mask &= ~(1ULL << pin);
    }

    // Its also important to read it afterwars to have the most recent value
    const uint64_t all_ports_state = gpio_read_all_ports(); // Read once to save time

    for (uint8_t i = 0; i < count; i++)
    {
        DataHandshakeData *instance = &dhd_instances[i];

        switch (dhd_get_manchester_mode(instance))
        {
        case PMAN_IDLE:
            // Do nothing
            break;
        case PMAN_SEND:
        {
            const enum spooky_encoder_step_res step = spooky_encoder_step(&instance->manchester_enc);
            const uint8_t pin = instance->pin;

            switch (step)
            {
            case SPOOKY_ENCODER_STEP_OK_LOW:
                set_zero_mask |= (1ULL << pin);
                break;

            case SPOOKY_ENCODER_STEP_OK_HIGH:
                set_one_mask |= (1ULL << pin);
                break;

            case SPOOKY_ENCODER_STEP_OK_DONE:

                instance->status |= STATUS_TX_COMPLETE;
                gpio_od_release(pin);
                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;

            case SPOOKY_ENCODER_STEP_OK:
                // Signal remains unchanged
                break;
            default:
                // pman_set_TX(true, pin);
                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;
            }
            break;
        }

        case PMAN_RECEIVE:
        {
            // gpio_drive_high(DEBUG_PIN2);
            const uint8_t pin = instance->pin;
            const bool rx = (all_ports_state >> pin) & 0x1ULL;

            const enum spooky_decoder_step_res step = spooky_decoder_step(&instance->manchester_dec, rx);
            const uint8_t current_mode = instance->manchester_dec.mode;
            const uint8_t last_mode = instance->manchester_last_decoder_mode;

            if (current_mode != last_mode)
            {

                if (current_mode < last_mode && last_mode != 3)
                {
                    // Here a error occures
                    dhd_set_manchester_mode(instance, PMAN_IDLE);
                    instance->status |= STATUS_RX_ERROR;
                }
                instance->manchester_rx_timeout_counter = 0;
            }
            else
            {
                if (rx == instance->manchester_last_rx)
                    instance->manchester_rx_timeout_counter++;
                else
                    instance->manchester_rx_timeout_counter = 0;

                instance->manchester_last_rx = rx;

                if (instance->manchester_rx_timeout_counter > NUMBER_OF_MAX_INSTACES_WITHOUT_TRANSITION)
                {
                    dhd_set_manchester_mode(instance, PMAN_IDLE);
                    instance->status |= STATUS_RX_ERROR;
                    instance->manchester_rx_timeout_counter = 0;
                }
            }

            instance->manchester_last_decoder_mode = current_mode;

            switch (step)
            {
            case SPOOKY_DECODER_STEP_DONE:
            {
                instance->status |= STATUS_RX_COMPLETE;
                instance->status |= STATUS_DATA_RECEIVED;
                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;
            }
            case SPOOKY_DECODER_STEP_ERROR_NULL:
            {
                instance->status |= STATUS_RX_ERROR;
                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;
            }
            default:
                break;
            }
            // gpio_drive_low(DEBUG_PIN2);
            break;
        }

        default:
            break;
        }
    }
}

inline uint32_t parallel_manchester_get_sample_interval_us(ParallelManchesterBaudRate rate)
{
    uint32_t bit_time_us = 1000000UL / rate;
    return bit_time_us / TX_RATE;
}

// Update rx_callback to use instance buffer directly
static void pman_rx_callback(uint8_t *data, uint8_t data_size, void *udata)
{
    // not used
}

// Update add_instance to remove data_buffer and data_size fields
inline bool parallel_manchester_add_instance(DataHandshakeData *instance)
{

    instance->manchester_last_rx = false;
    instance->manchester_last_decoder_mode = 0;
    dhd_set_manchester_mode(instance, PMAN_IDLE);

    // Initialize spooky encoder/decoder to use the provided buffer.
    // Encoder: buffer holds outgoing bytes to be enqueued before transmit.
    enum spooky_encoder_init_res enc_res = spooky_encoder_init(&instance->manchester_enc, instance->data_buffer, BUFFER_SIZE);
    if (enc_res != SPOOKY_ENCODER_INIT_OK)
    {
        printf("Error: Encoder init failed for pin %u\n", instance->pin);
        return false; // Invalid index
    }

    // Decoder: will write received bytes directly into the same buffer and call pman_rx_callback.
    // Workaround since uudata in callback cannot be used to pass instance pointer.
    uint8_t newindex = 0;

    enum spooky_decoder_init_res dec_res = spooky_decoder_init(&instance->manchester_dec,
                                                               instance->data_buffer,
                                                               BUFFER_SIZE,
                                                               pman_rx_callback,
                                                               (void *)(uintptr_t)newindex);

    if (dec_res != SPOOKY_DECODER_INIT_OK)
    {
        printf("Error: Decoder init failed for pin %u\n", instance->pin);
        return false; // Invalid index
    }
    return true; // Success
}
// Non-blocking transmission function
inline bool parallel_manchester_transmit_background(DataHandshakeData *instance, uint8_t size)
{

    // Clear encoder and enqueue data from instance buffer
    spooky_encoder_clear(&instance->manchester_enc);
    if (spooky_encoder_enqueue_no_copy(&instance->manchester_enc, size) != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        return false;
    }

    // Start transmission
    dhd_set_manchester_mode(instance, PMAN_SEND);

    instance->status &= ~STATUS_TX_COMPLETE; // Clear previous TX complete status
    return true;
}

// Check if transmission is complete
inline bool parallel_manchester_transmit_complete(DataHandshakeData *instance)
{
    const bool complete = instance->status & STATUS_TX_COMPLETE;
    if (complete)
    {
        instance->status &= ~STATUS_TX_COMPLETE; // Clear the bit after checking
        return true;
    }
    return false;
}

// Non-blocking receive function
inline bool parallel_manchester_receive_background(DataHandshakeData *instance)
{
    memset(instance->data_buffer, 0, BUFFER_SIZE);
    // Clear decoder state
    reset_decoder(&instance->manchester_dec);

    // reinitialize decoder to reset internal state
    // if (spooky_decoder_init(&instance->manchester_dec, instance->data_buffer, BUFFER_SIZE, pman_rx_callback, instance->) != 0)
    // {
    //     printf("Error: Decoder re-init failed for pin %u\n", instance->pin);
    //     return false; // Invalid index
    // }

    // Start receiving - data will be written directly to instance buffer by spooky decoder
    dhd_set_manchester_mode(instance, PMAN_RECEIVE);
    instance->manchester_rx_timeout_counter = 0;
    // Clear relevant status flags before starting reception
    instance->status &= ~(STATUS_TX_COMPLETE | STATUS_RX_COMPLETE | STATUS_RX_ERROR);

    return true;
}

// Check if receive is complete
inline bool parallel_manchester_receive_complete(DataHandshakeData *instance)
{
    const bool complete = instance->status & STATUS_RX_COMPLETE;
    if (complete)
    {
        instance->status &= ~STATUS_RX_COMPLETE; // Clear the bit after checking
        return true;
    }
    return false;
}

// Check if receive had an error
inline bool parallel_manchester_receive_error(DataHandshakeData *instance)
{
    const bool error = instance->status & STATUS_RX_ERROR;
    if (error)
    {
        instance->status &= ~STATUS_RX_ERROR; // Clear the bit after checking
        return true;
    }
    return false;
}

// Check if data has been received
inline bool parallel_manchester_data_received(DataHandshakeData *instance)
{
    const bool received = instance->status & STATUS_DATA_RECEIVED;
    if (received)
    {
        instance->status &= ~STATUS_DATA_RECEIVED; // Clear the bit after checking
        return true;
    }
    return false;
}
