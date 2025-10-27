#include "datahandshake_modulation.h"
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

static uint8_t pman_baud_rate = 0;

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

void pman_timer_isr(DataHandshakeData *dhd_instances, uint8_t pman_instance_count)
{
    const uint8_t count = pman_instance_count;
    const uint64_t all_ports_state = gpio_read_all_ports(); // Read once to save time

    for (uint8_t i = 0; i < count; i++)
    {
        DataHandshakeData *instance = &dhd_instances[i];

        switch (dhd_get_manchester_mode(instance))
        {
        case PMAN_SEND:
        {
            const enum spooky_encoder_step_res step = spooky_encoder_step(&instance->manchester_enc);
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
                instance->status |= STATUS_TX_COMPLETE;

                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;

            case SPOOKY_ENCODER_STEP_OK:
                break;

            default:
                pman_set_TX(true, pin);
                dhd_set_manchester_mode(instance, PMAN_IDLE);
                break;
            }
            break;
        }

        case PMAN_RECEIVE:
        {
            // gpio_drive_high(DEBUG_PIN_ABS);
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
                }
                instance->receiving_counter = 0;
            }
            else
            {
                if (rx == instance->manchester_last_rx)
                    instance->receiving_counter++;
                else
                    instance->receiving_counter = 0;

                instance->manchester_last_rx = rx;

                if (instance->receiving_counter > NUMBER_OF_MAX_INSTACES_WITHOUT_TRANSITION)
                {
                    dhd_set_manchester_mode(instance, PMAN_IDLE);

                    instance->status |= STATUS_RX_ERROR;
                    instance->receiving_counter = 0;
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
            // gpio_drive_low(DEBUG_PIN_ABS);
            break;
        }

        default:
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
    DataHandshakeData *instance = (uintptr_t)udata;

    // Check for invalid data or wrong mode - no need for data_buffer check anymore
    if (!data || !data_size)
        return;

    // Copy data directly to instance buffer (data is already there from spooky decoder)
    // The spooky decoder already wrote to instance->buffer, so we just set the flag
    instance->status |= STATUS_DATA_RECEIVED;
}

void parallel_manchester_init(ParallelManchesterBaudRate tx_rate)
{
    //  uint32_t sample_interval_us = parallel_manchester_get_sample_interval_us(tx_rate);

    gpio_output_init(DEBUG_PIN_ABS);
    //  pman_setup_and_start_timer(sample_interval_us);

    LOG("Manchester initialized\n");
}

// Update add_instance to remove data_buffer and data_size fields
bool parallel_manchester_add_instance(DataHandshakeData *instance)
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
    // Pass instance pointer as user data (cast via uintptr_t).
    enum spooky_encoder_init_res dec_res = spooky_decoder_init(&instance->manchester_dec, instance->data_buffer, BUFFER_SIZE, pman_rx_callback, (void *)(uintptr_t)instance);

    if (dec_res != SPOOKY_DECODER_INIT_OK)
    {
        printf("Error: Decoder init failed for pin %u\n", instance->pin);
        return false; // Invalid index
    }
    return true; // Success
}
// Non-blocking transmission function
bool parallel_manchester_transmit_background(DataHandshakeData *instance, uint8_t size)
{

    if (dhd_get_manchester_mode(instance) != PMAN_IDLE)
    {
        return false; // Instance busy
    }

    // Clear encoder and enqueue data from instance buffer
    spooky_encoder_clear(&instance->manchester_enc);
    if (spooky_encoder_enqueue(&instance->manchester_enc, instance->data_buffer, size) != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        return false;
    }

    // Start transmission
    dhd_set_manchester_mode(instance, PMAN_SEND);

    instance->status &= ~STATUS_TX_COMPLETE; // Clear previous TX complete status
    return true;
}

// Check if transmission is complete
bool parallel_manchester_transmit_complete(DataHandshakeData *instance)
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
bool parallel_manchester_receive_background(DataHandshakeData *instance)
{

    if (dhd_get_manchester_mode(instance) != PMAN_IDLE)
    {
        return false; // Instance busy
    }

    // Clear instance buffer
    memset(instance->data_buffer, 0, BUFFER_SIZE);

    // reinitialize decoder to reset internal state
    if (spooky_decoder_init(&instance->manchester_dec, instance->data_buffer, BUFFER_SIZE, pman_rx_callback, (void *)(uintptr_t)index) != 0)
    {
        printf("Error: Decoder re-init failed for pin %u\n", instance->pin);
        return false; // Invalid index
    }

    // Start receiving - data will be written directly to instance buffer by spooky decoder
    dhd_set_manchester_mode(instance, PMAN_RECEIVE);
    instance->receiving_counter = 0;
    // Clear relevant status flags before starting reception
    instance->status &= ~(STATUS_TX_COMPLETE | STATUS_RX_COMPLETE | STATUS_RX_ERROR);

    return true;
}

// Check if receive is complete
bool parallel_manchester_receive_complete(DataHandshakeData *instance)
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
bool parallel_manchester_receive_error(DataHandshakeData *instance)
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
bool parallel_manchester_data_received(DataHandshakeData *instance)
{
    const bool received = instance->status & STATUS_DATA_RECEIVED;
    if (received)
    {
        instance->status &= ~STATUS_DATA_RECEIVED; // Clear the bit after checking
        return true;
    }
    return false;
}
/* 
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
 */