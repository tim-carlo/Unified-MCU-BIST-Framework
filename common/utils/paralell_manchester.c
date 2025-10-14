#include "paralell_manchester.h"
#include "printf.h"
#include <stdlib.h>
#include <string.h>

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

#if defined(NRF52840_XXAA)
#define PMAN_TIMER NRF_TIMER4
#define DEBUG_PIN_ABS 38 // Pin 1.6
#elif defined(__MSP430FR5994__)
#define PMAN_TIMER TIMER_B0 // This has according to the datasheet a higherr resolution and more features and higher priority than TIMER_A2
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


void pman_timer_isr(void)
{
    gpio_drive_high(DEBUG_PIN_ABS);
    const uint8_t count = pman_instance_count;
    const uint64_t all_ports_state = gpio_read_all_ports(); // Read once to save time

    for (uint8_t i = 0; i < count; i++)
    {
        ParallelManchesterInstance *instance = &pman_instances[i];
        if (!instance)
            continue;

        switch (instance->mode)
        {
        case PMAN_SEND:
        {
            const enum spooky_encoder_step_res step = spooky_encoder_step(&instance->enc);
            const uint8_t pin = instance->pin;

            switch (step)
            {
            case SPOOKY_ENCODER_STEP_OK_LOW:
                pman_set_TX(false, pin);
                break;

            case SPOOKY_ENCODER_STEP_OK_HIGH:
                pman_set_TX(true, pin);
                break;

            case SPOOKY_ENCODER_STEP_OK_DONE:
                pman_set_TX(true, pin);
                instance->status |= PMAN_STATUS_TRANSMISSION_COMPLETE;
                instance->mode = PMAN_IDLE;
                break;

            case SPOOKY_ENCODER_STEP_OK:
                break;

            default:
                pman_set_TX(true, pin);
                instance->mode = PMAN_IDLE;
                break;
            }
            break;
        }

        case PMAN_RECEIVE:
        {
            // gpio_drive_high(DEBUG_PIN_ABS);
            const uint8_t pin = instance->pin;
            const bool rx = (all_ports_state >> pin) & 0x1ULL;

            const enum spooky_decoder_step_res step = spooky_decoder_step(&instance->dec, rx);
            const uint8_t current_mode = instance->dec.mode;
            const uint8_t last_mode = instance->last_decoder_mode;

            if (current_mode != last_mode)
            {

                if (current_mode < last_mode && last_mode != 3)
                {
                    // Here a error occures
                }
                instance->cycles_without_transition = 0;
            }
            else
            {
                if (rx == instance->last_rx)
                    instance->cycles_without_transition++;
                else
                    instance->cycles_without_transition = 0;

                instance->last_rx = rx;

                if (instance->cycles_without_transition > NUMBER_OF_MAX_INSTACES_WITHOUT_TRANSITION)
                {
                    instance->mode = PMAN_IDLE;
                    instance->status |= PMAN_STATUS_RECEIVE_ERROR;
                    instance->cycles_without_transition = 0;
                }
            }

            instance->last_decoder_mode = current_mode;

            switch (step)
            {
            case SPOOKY_DECODER_STEP_DONE:
            {
                instance->status |= PMAN_STATUS_RECEIVE_COMPLETE;
                instance->mode = PMAN_IDLE;
                break;
            }
            case SPOOKY_DECODER_STEP_ERROR_NULL:
            {
                instance->status |= PMAN_STATUS_RECEIVE_ERROR;
                instance->mode = PMAN_IDLE;
                break;
            }
            default:
                break;
            }
            // gpio_drive_low(DEBUG_PIN_ABS);
            break;
        }

        default:
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
    uint32_t ticks;
    const uint16_t prescaler = choose_prescaler_and_ticks(sample_interval_us, SMCLK_HZ, &ticks);

    configure_timer(PMAN_TIMER, prescaler, MC__UP);
    set_timer_compare(PMAN_TIMER, 0, (uint16_t)(ticks - 1));
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
    return bit_time_us / PMAN_TXRX_RATE;
    // PMAN_TXRX_RATE runter setzen dann hat man mehr entstpannung für den MSP
}

// Update rx_callback to use instance buffer directly
static void pman_rx_callback(uint8_t *data, uint8_t data_size, void *udata)
{
    const uint8_t index = (uint8_t)(uintptr_t)udata;
    ParallelManchesterInstance *instance = &pman_instances[index];

    // Check for invalid data or wrong mode - no need for data_buffer check anymore
    if (!data || !data_size || instance->mode != PMAN_RECEIVE)
        return;

    // Copy data directly to instance buffer (data is already there from spooky decoder)
    // The spooky decoder already wrote to instance->buffer, so we just set the flag
    instance->status |= PMAN_STATUS_DATA_RECEIVED;
}

void parallel_manchester_init(ParallelManchesterBaudRate tx_rate)
{
  //  uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(tx_rate);

    gpio_output_init(DEBUG_PIN_ABS);
  //  pman_setup_and_start_timer(sample_interval_us);

    LOG("Manchester initialized\n");
}

// Update add_instance to remove data_buffer and data_size fields
uint8_t parallel_manchester_add_instance(uint8_t pin, uint8_t *buffer, uint8_t buffer_size)
{
    // Check if pin already exists
    for (uint8_t i = 0; i < pman_instance_count; i++)
    {
        if (pman_instances[i].pin == pin)
        {
            return 255; // Pin already exists, return invalid index
        }
    }

    // Validate buffer parameters
    if (!buffer || buffer_size == 0)
    {
        printf("Error: Invalid buffer parameters for pin %u\n", pin);
        return 255;
    }

    // Reallocate memory for new instance
    ParallelManchesterInstance *new_instances = realloc(pman_instances, (pman_instance_count + 1) * sizeof(ParallelManchesterInstance));
    if (!new_instances)
    {
        return 255; // Memory allocation failed
    }

    pman_instances = new_instances;
    ParallelManchesterInstance *new_instance = &pman_instances[pman_instance_count];

    // Initialize the new instance with provided buffer - remove data_buffer fields
    new_instance->pin = pin;
    new_instance->mode = PMAN_IDLE;
    new_instance->status = 0;
    new_instance->buffer = buffer;           // Use provided buffer
    new_instance->buffer_size = buffer_size; // Store buffer size
    new_instance->last_decoder_mode = 0;     // Initialize decoder mode tracking
    new_instance->cycles_without_transition = 0;
    new_instance->last_rx = false;

    // Initialize GPIO for this pin
    gpio_od_init(pin);
    printf("Added instance on pin %u at index %u with buffer %p (size %u)\n",
           pin, pman_instance_count, (void *)buffer, buffer_size);

    // Initialize the spooky encoder and decoder with the provided buffer
    if (spooky_encoder_init(&new_instance->enc, new_instance->buffer, new_instance->buffer_size, PMAN_TXRX_RATE) != 0)
    {
        printf("Error: Encoder init failed for pin %u\n", pin);
        return 255; // Invalid index
    }

    uint8_t new_index = pman_instance_count;
    if (spooky_decoder_init(&new_instance->dec, new_instance->buffer, new_instance->buffer_size, pman_rx_callback, (void *)(uintptr_t)new_index) != 0)
    {
        printf("Error: Decoder init failed for pin %u\n", pin);
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
    if (index >= pman_instance_count)
    {
        return false; // Invalid index
    }

    ParallelManchesterInstance *instance = &pman_instances[index];

    if (instance->mode != PMAN_IDLE)
    {
        return false; // Instance busy
    }

    // Validate input parameters against buffer size
    if (data == NULL || size == 0 || size > instance->buffer_size)
    {
        printf("Error: Data size %u exceeds buffer size %u\n", size, instance->buffer_size);
        return false;
    }
    // clear instance buffer
    memset(instance->buffer, 0, instance->buffer_size);

    // Copy data to instance buffer first
    for (uint8_t i = 0; i < size; i++)
        instance->buffer[i] = data[i];

    // Clear encoder and enqueue data from instance buffer
    spooky_encoder_clear(&instance->enc);
    if (spooky_encoder_enqueue(&instance->enc, instance->buffer, size) != SPOOKY_ENCODER_ENQUEUE_OK)
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
    if (size == 0 || size > instance->buffer_size)
    {
        return false;
    }

    // Clear instance buffer
    memset(instance->buffer, 0, instance->buffer_size);

    // reinitialize decoder to reset internal state
    if (spooky_decoder_init(&instance->dec, instance->buffer, instance->buffer_size, pman_rx_callback, (void *)(uintptr_t)index) != 0)
    {
        printf("Error: Decoder re-init failed for pin %u\n", instance->pin);
        return false; // Invalid index
    }

    // Start receiving - data will be written directly to instance buffer by spooky decoder
    instance->mode = PMAN_RECEIVE;
    instance->last_decoder_mode = 0;         // Reset decoder mode tracking
    instance->status = 0;                    // Reset ALL status bits when starting receive
    instance->cycles_without_transition = 0; // Reset timeout counter

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

// Helper function to get buffer pointer
uint8_t *parallel_manchester_get_received_data(uint8_t index)
{
    if (index >= pman_instance_count)
        return NULL;
    return pman_instances[index].buffer;
}

// Add deinit function for cleanup
void parallel_manchester_deinit()
{
    // Stop timer
    pman_stop_timer();

    // Clean up instances (buffers are owned by caller, don't free them)
    if (pman_instances)
    {
        free(pman_instances);
        pman_instances = NULL;
    }
    pman_instance_count = 0;

    LOG("Manchester deinitialized\n");
}
