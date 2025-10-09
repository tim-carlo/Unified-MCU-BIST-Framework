#include "paralell_manchester.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>

#define PMAN_ENCODER_BUFFER_SIZE 32
#define PMAN_DECODER_BUFFER_SIZE 32

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

#if defined(NRF52840_XXAA)
#define PMAN_TIMER NRF_TIMER4
#define DEBUG_PIN_ABS 38 // Pin 1.6
#elif defined(__MSP430FR5994__)
#define PMAN_TIMER TIMER_A2
#define DEBUG_PIN_ABS ABS_PIN(3, 0)
#endif

static uint8_t pman_baud_rate = 0;
static ParallelManchesterBaudRate pman_current_baud_rate = PMAN_BAUD_600; // Default baud rate

ParallelManchesterInstance *pman_instances = NULL;
uint8_t pman_instance_count = 0;
static volatile uint8_t pman_current_instance_index = 0;

// Helper functions to read status bits
static bool pman_is_transmission_complete(uint8_t index)
{
    return (pman_instances[index].status & PMAN_STATUS_TRANSMISSION_COMPLETE) != 0;
}

static bool pman_is_receive_complete(uint8_t index)
{
    return (pman_instances[index].status & PMAN_STATUS_RECEIVE_COMPLETE) != 0;
}

static bool pman_is_receive_error(uint8_t index)
{
    return (pman_instances[index].status & PMAN_STATUS_RECEIVE_ERROR) != 0;
}

static void pman_clear_transmission_complete(uint8_t index)
{
    pman_instances[index].status &= ~PMAN_STATUS_TRANSMISSION_COMPLETE;
}

static void pman_clear_receive_complete(uint8_t index)
{
    pman_instances[index].status &= ~PMAN_STATUS_RECEIVE_COMPLETE;
}

static void pman_clear_receive_error(uint8_t index)
{
    pman_instances[index].status &= ~PMAN_STATUS_RECEIVE_ERROR;
}

static void pman_clear_receive_status(uint8_t index)
{
    pman_instances[index].status &= ~(PMAN_STATUS_RECEIVE_COMPLETE | PMAN_STATUS_RECEIVE_ERROR);
}

static void pman_set_TX(bool state, uint8_t pin)
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

static void pman_timer_isr(void)
{

    // Process each instance in round-robin fashion
    for (uint8_t i = 0; i < pman_instance_count; i++)
    {
        ParallelManchesterInstance *instance = &pman_instances[i];

        switch (instance->mode)
        {
        case PMAN_SEND:
        {
            enum spooky_encoder_step_res step_result = spooky_encoder_step(&instance->enc);

            switch (step_result)
            {
            case SPOOKY_ENCODER_STEP_OK_DONE:
                pman_set_TX(true, instance->pin); // release the line
                instance->status |= PMAN_STATUS_TRANSMISSION_COMPLETE;
                instance->mode = PMAN_IDLE;
                break;
            case SPOOKY_ENCODER_STEP_OK_LOW:
                pman_set_TX(false, instance->pin);
                break;
            case SPOOKY_ENCODER_STEP_OK_HIGH:
                pman_set_TX(true, instance->pin);
                break;
            case SPOOKY_ENCODER_STEP_OK:
                break;
            default:
                // Error case
                pman_set_TX(true, instance->pin); // release the line
                instance->mode = PMAN_IDLE;
                break;
            }
            break;
        }
        case PMAN_RECEIVE:
        {
            gpio_drive_high(DEBUG_PIN_ABS);
            const bool rx_state = gpio_read(instance->pin);
            enum spooky_decoder_step_res step_result = spooky_decoder_step(&instance->dec, rx_state);

            if (step_result == SPOOKY_DECODER_STEP_DONE)
            {
                instance->status |= PMAN_STATUS_RECEIVE_COMPLETE;
                instance->mode = PMAN_IDLE;
            }
            else if (step_result == SPOOKY_DECODER_STEP_ERROR_NULL)
            {
                instance->status |= PMAN_STATUS_RECEIVE_ERROR;
                instance->mode = PMAN_IDLE;
            }
            gpio_drive_low(DEBUG_PIN_ABS);
            break;
        }
        case PMAN_IDLE:
        case PMAN_NOT_INITIALIZED:
        default:
            // Do nothing for idle or uninitialized instances
            break;
        }
    }
    gpio_drive_low(DEBUG_PIN_ABS);
}

static void pman_setup_and_start_timer(uint16_t sample_interval_us)
{
    LOG("Setting up timer with sample interval: %u us\n", sample_interval_us);

#if defined(NRF52840_XXAA)
    // Use constant prescaler 4 (1MHz)
    configure_timer(PMAN_TIMER, 4, TIMER_BITMODE_BITMODE_32Bit);
    set_timer_compare(PMAN_TIMER, 0, sample_interval_us, true, true);
    PMAN_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    set_timer_event_callback(PMAN_TIMER, pman_timer_isr);
    start_timer(PMAN_TIMER);

#elif defined(__MSP430FR5994__)
    uint32_t timer_ticks = (sample_interval_us * (SMCLK_HZ / 1000000UL)) - 1;

    configure_timer(PMAN_TIMER, 0, MC__UP); // No prescaler division
    set_timer_compare(PMAN_TIMER, 0, (uint16_t)timer_ticks);
    set_timer_compare_callback(PMAN_TIMER, pman_timer_isr);
    start_timer_with_interrupt(PMAN_TIMER);
#endif
}

static void pman_stop_timer()
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(PMAN_TIMER);
    stop_timer(PMAN_TIMER);
#elif defined(__MSP430FR5994__)
    clear_timer_event_callback(PMAN_TIMER);
    stop_timer(PMAN_TIMER);
#endif
}

uint32_t parallel_manchester_get_sample_interval_us(ParallelManchesterBaudRate rate)
{
    uint32_t bit_time_us = 1000000UL / rate;
    return bit_time_us / PMAN_TX_RATE;
}

static void pman_rx_callback(uint8_t *data, uint8_t data_size, void *udata)
{
    const uint8_t index = (uint8_t)(uintptr_t)udata;
    ParallelManchesterInstance *instance = &pman_instances[index];
    // Check for invalid data or wrong mode
    if (!data || !data_size || !instance->data_buffer || instance->mode != PMAN_RECEIVE)
        return;

    const uint8_t copy_size = (data_size < instance->data_size) ? data_size : instance->data_size;

    // Manuell copying, reduces memcpy overhead on small MCUs
    for (uint8_t i = 0; i < copy_size; i++)
        instance->data_buffer[i] = data[i];

    instance->status |= PMAN_STATUS_DATA_RECEIVED;
}

void parallel_manchester_init(ParallelManchesterBaudRate tx_rate)
{
    uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(tx_rate);

    gpio_output_init(DEBUG_PIN_ABS);
    pman_setup_and_start_timer(sample_interval_us);
}

uint8_t parallel_manchester_add_instance(uint8_t pin)
{
    // Check if pin already exists
    for (uint8_t i = 0; i < pman_instance_count; i++)
    {
        if (pman_instances[i].pin == pin)
        {
            return 255; // Pin already exists, return invalid index
        }
    }

    // Reallocate memory for new instance
    ParallelManchesterInstance *new_instances = realloc(pman_instances, (pman_instance_count + 1) * sizeof(ParallelManchesterInstance));
    if (!new_instances)
    {
        return 255; // Memory allocation failed
    }

    pman_instances = new_instances;
    ParallelManchesterInstance *new_instance = &pman_instances[pman_instance_count];

    // Initialize the new instance
    new_instance->pin = pin;
    new_instance->mode = PMAN_IDLE;
    new_instance->status = 0;
    new_instance->data_buffer = NULL;
    new_instance->data_size = 0;

    // Initialize GPIO for this pin
    gpio_od_init(pin);

    // Initialize the spooky encoder and decoder with the single buffer
    // spooky_encoder_init expects: encoder, buffer, buffer_size, tx_rate
    if (spooky_encoder_init(&new_instance->enc, new_instance->buffer, PMAN_BUFFER_SIZE, PMAN_TX_RATE) != 0)
    {
        // If encoder initialization fails, clean up and return invalid index
        return 255; // Invalid index
    }

    uint8_t new_index = pman_instance_count;
    // spooky_decoder_init expects: decoder, buffer, buffer_size, callback, user_data
    if (spooky_decoder_init(&new_instance->dec, new_instance->buffer, PMAN_BUFFER_SIZE, pman_rx_callback, (void*)(uintptr_t)new_index) != 0)
    {
        // If decoder initialization fails, clean up and return invalid index
        return 255; // Invalid index
    }

    pman_instance_count++;
    return new_index; // Return the index of the new instance
}

bool parallel_manchester_remove_instance(uint8_t index)
{
    // Ensure there are instances and index is valid
    if (pman_instance_count == 0 || index >= pman_instance_count)
    {
        return false;
    }

    // Handle single instance case
    if (pman_instance_count == 1)
    {
        free(pman_instances);
        pman_instances = NULL;
        pman_instance_count = 0;
        return true;
    }

    // Shift remaining instances left
    for (uint8_t i = index; i < pman_instance_count - 1; i++)
    {
        pman_instances[i] = pman_instances[i + 1];
    }

    // Decrease count
    pman_instance_count--;

    ParallelManchesterInstance *new_instances = realloc(pman_instances, pman_instance_count * sizeof(ParallelManchesterInstance));
    if (new_instances == NULL)
    {
        // realloc failed, but old memory is still valid
        // So we don't overwrite pman_instances
        return true;
    }

    // Update pointer if realloc succeeded
    pman_instances = new_instances;
    return true;
}
// Non-blocking transmission function
bool parallel_manchester_transmit_background(uint8_t index, uint8_t *data, uint8_t size)
{
    ParallelManchesterInstance *instance = &pman_instances[index];

    if (instance->mode != PMAN_IDLE)
    {
        return false; // Instance busy
    }

    // Validate input parameters
    if (data == NULL || size == 0 || size > PMAN_ENCODER_BUFFER_SIZE)
    {
        return false;
    }

    // Clear encoder and enqueue data
    spooky_encoder_clear(&instance->enc);
    if (spooky_encoder_enqueue(&instance->enc, data, size) != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        return false;
    }

    // Start transmission
    instance->mode = PMAN_SEND;
    pman_clear_transmission_complete(index);
    return true;
}

// Check if transmission is complete
bool parallel_manchester_transmit_complete(uint8_t index)
{
    // Validate index
    if (index >= pman_instance_count)
    {
        return false; // Instance not found
    }

    if (pman_is_transmission_complete(index))
    {
        pman_clear_transmission_complete(index);
        return true;
    }
    return false;
}

// Non-blocking receive function
bool parallel_manchester_receive_background(uint8_t index, uint8_t *data, uint8_t size)
{
    if (index >= pman_instance_count)
    {
        return false; // Invalid index
    }

    ParallelManchesterInstance *instance = &pman_instances[index];

    if (instance->mode != PMAN_IDLE)
    {
        return false; // Instance busy
    }

    // Validate input parameters
    if (data == NULL || size == 0)
    {
        return false;
    }

    // Set up receive buffer and start receiving
    instance->data_buffer = data;
    instance->data_size = size;
    instance->mode = PMAN_RECEIVE;
    pman_clear_receive_status(index);
    return true;
}

// Check if receive is complete
bool parallel_manchester_receive_complete(uint8_t index)
{
    if (index >= pman_instance_count)
    {
        return false; // Invalid index
    }

    if (pman_is_receive_complete(index))
    {
        pman_clear_receive_complete(index);
        return true;
    }
    return false;
}

// Check if receive had an error
bool parallel_manchester_receive_error(uint8_t index)
{

    if (pman_is_receive_error(index))
    {
        pman_clear_receive_error(index);
        return true;
    }
    return false;
}

// Check if data has been received
bool parallel_manchester_data_received(uint8_t index)
{
    if (pman_instances[index].status & PMAN_STATUS_DATA_RECEIVED)
    {
        pman_instances[index].status &= ~PMAN_STATUS_DATA_RECEIVED; // Clear the bit after checking
        return true;
    }
    return false;
}

// Check if instance is idle
bool parallel_manchester_is_idle(uint8_t index)
{
    return pman_instances[index].mode == PMAN_IDLE;
}

// Check if instance is transmitting
bool parallel_manchester_is_transmitting(uint8_t index)
{
    return pman_instances[index].mode == PMAN_SEND;
}

// Check if instance is receiving
bool parallel_manchester_is_receiving(uint8_t index)
{
    return pman_instances[index].mode == PMAN_RECEIVE;
}
