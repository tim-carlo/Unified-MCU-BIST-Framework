#include "paralell_manchester.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>

#define PMAN_ENCODER_BUFFER_SIZE 32
#define PMAN_DECODER_BUFFER_SIZE 32

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

#if defined(NRF52840_XXAA)
#define PMAN_TIMER NRF_TIMER3
#elif defined(__MSP430FR5994__)
#define PMAN_TIMER TIMER_A2
#endif

static uint8_t pman_baud_rate = 0;
static ParallelManchesterBaudRate pman_current_baud_rate = PMAN_BAUD_600; // Default baud rate

ParallelManchesterInstance *pman_instances = NULL;
uint8_t pman_instance_count = 0;
static volatile uint8_t pman_current_instance_index = 0;


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
                instance->transmission_complete = true;
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
                // Error case - reset to idle
                pman_set_TX(true, instance->pin); // release the line
                instance->mode = PMAN_IDLE;
                break;
            }
            break;
        }
        case PMAN_RECEIVE:
        {
            bool rx_state = gpio_read(instance->pin);
            enum spooky_decoder_step_res step_result = spooky_decoder_step(&instance->dec, rx_state);
            
            if (step_result == SPOOKY_DECODER_STEP_DONE)
            {
                instance->receive_complete = true;
                instance->mode = PMAN_IDLE;
            }
            else if (step_result == SPOOKY_DECODER_STEP_ERROR_NULL)
            {
                instance->receive_error = true;
                instance->mode = PMAN_IDLE;
            }
            break;
        }
        case PMAN_IDLE:
        case PMAN_NOT_INITIALIZED:
        default:
            // Do nothing for idle or uninitialized instances
            break;
        }
    }
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
    ParallelManchesterInstance *instance = (ParallelManchesterInstance *)udata;
    if (instance && instance->receive_buffer && data_size > 0)
    {
        // Copy received data to the instance's receive buffer
        uint8_t copy_size = (data_size < instance->receive_size) ? data_size : instance->receive_size;
        memcpy(instance->receive_buffer, data, copy_size);
        instance->receive_complete = true;
    }
}




void parallel_manchester_init(ParallelManchesterBaudRate tx_rate)
{
    uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(tx_rate);

    pman_setup_and_start_timer(sample_interval_us);
}

uint8_t parallel_manchester_add_instance(uint8_t pin)
{
    // Check if pin already exists
    for (uint8_t i = 0; i < pman_instance_count; i++)
    {
        if (pman_instances[i].pin == pin)
        {
            // Pin already exists, return 255
            return 255;
        }
    }
    
    ParallelManchesterInstance *new_instances = realloc(pman_instances, (pman_instance_count + 1) * sizeof(ParallelManchesterInstance));
    if (new_instances == NULL)
    {
        // Handle memory allocation failure
        return 255;
    }
    pman_instances = new_instances;

    // Get reference to the new instance
    ParallelManchesterInstance *new_instance = &pman_instances[pman_instance_count];
    
    // Initialize the new instance
    new_instance->pin = pin;
    new_instance->mode = PMAN_IDLE;
    new_instance->transmission_complete = false;
    new_instance->receive_complete = false;
    new_instance->receive_error = false;
    new_instance->receive_buffer = NULL;
    new_instance->receive_size = 0;
    
    // Initialize encoder
    enum spooky_encoder_init_res enc_result = spooky_encoder_init(
        &new_instance->enc, 
        new_instance->encoder_buffer, 
        PMAN_ENCODER_BUFFER_SIZE, 
        PMAN_TX_RATE);
    
    // Initialize decoder with callback
    enum spooky_decoder_init_res dec_result = spooky_decoder_init(
        &new_instance->dec, 
        new_instance->decoder_buffer, 
        PMAN_DECODER_BUFFER_SIZE, 
        pman_rx_callback, // Callback function
        new_instance  // Pass instance as callback data
    );
    
    // Check if initialization was successful
    if (enc_result != SPOOKY_ENCODER_INIT_OK || dec_result != SPOOKY_DECODER_INIT_OK)
    {
        // Initialization failed, don't increment counter
        return 255;
    }
    
    // Initialize GPIO pin for open drain output
    gpio_od_init(pin);
    
    pman_instance_count++;
    return pman_instance_count - 1;
}

bool parallel_manchester_remove_instance(uint8_t index)
{
    // Validate index
    if (index >= pman_instance_count)
    {
        return false;
    }
    
    uint8_t instance_index = index;
    
    // If this is the last instance, just free and set to NULL
    if (pman_instance_count == 1)
    {
        free(pman_instances);
        pman_instances = NULL;
        pman_instance_count = 0;
        return true;
    }
    
    // Shift all instances after the removed one back by one position
    for (uint8_t i = instance_index; i < pman_instance_count - 1; i++)
    {
        pman_instances[i] = pman_instances[i + 1];
    }
    
    // Decrease the count
    pman_instance_count--;
    
    // Reallocate memory to shrink the array
    ParallelManchesterInstance *new_instances = realloc(pman_instances, pman_instance_count * sizeof(ParallelManchesterInstance));
    if (new_instances == NULL && pman_instance_count > 0)
    {
        // Memory reallocation failed, but data is still valid (just using more memory than needed)
        // This is not a critical error for removal operation
        return true;
    }
    
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
    instance->transmission_complete = false;
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
    
    if (pman_instances[index].transmission_complete)
    {
        pman_instances[index].transmission_complete = false; // Reset flag
        return true;
    }
    return false;
}

// Non-blocking receive function
bool parallel_manchester_receive_background(uint8_t index, uint8_t *data, uint8_t size)
{
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
    instance->receive_buffer = data;
    instance->receive_size = size;
    instance->mode = PMAN_RECEIVE;
    instance->receive_complete = false;
    instance->receive_error = false;
    
    return true;
}

// Check if receive is complete
bool parallel_manchester_receive_complete(uint8_t index)
{
    
    if (pman_instances[index].receive_complete)
    {
        pman_instances[index].receive_complete = false; // Reset flag
        return true;
    }
    return false;
}

// Check if receive had an error
bool parallel_manchester_receive_error(uint8_t index)
{
    
    if (pman_instances[index].receive_error)
    {
        pman_instances[index].receive_error = false; // Reset flag
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
