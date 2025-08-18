#include "manchester.h"
#include "printf.h"
#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#endif

#define ENCODER_BUFFER_SIZE 32
#define DECODER_BUFFER_SIZE 32
#define TIMEOUT 1000 // Timeout in milliseconds for transmission
#define DEBUG 1      // Set to 1 to enable debug logging, 0 to disable
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
static const uint8_t tx_rate = 8;

static volatile uint16_t interrupt_flag = 0;

static uint8_t encoder_buffer[ENCODER_BUFFER_SIZE];
static uint8_t decoder_buffer[DECODER_BUFFER_SIZE];

static uint8_t *receive_buffer;

static struct spooky_encoder enc;
static struct spooky_decoder dec;

static bool encoder_initialized = false;
static bool decoder_initialized = false;

static void set_TX(bool state)
{
    if (state)
    {
        gpio_od_release(tx_pin); // Release the pin to drive high
    }
    else
    {
        gpio_drive_low(tx_pin); // Drive the pin low
    }
}

static bool read_Rx()
{
    return gpio_read(rx_pin);
}

static void setup_timer(uint16_t sample_interval_us)
{

    printf("Setting up timer with sample interval: %u us\n", sample_interval_us);

#if defined(NRF52840_XXAA)
    // Stop timer first to ensure clean configuration
    NRF_TIMER4->TASKS_STOP = 1;
    NRF_TIMER4->TASKS_CLEAR = 1;

    NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER4->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos; // 16MHz / 2^4 = 1MHz (1µs per tick)

    NRF_TIMER4->CC[0] = sample_interval_us;
    NRF_TIMER4->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;

    NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;

    NVIC_ClearPendingIRQ(TIMER4_IRQn);
    NVIC_SetPriority(TIMER4_IRQn, 3); // Set medium priority
    NVIC_EnableIRQ(TIMER4_IRQn);

    NRF_TIMER4->TASKS_CLEAR = 1;

#elif defined(__MSP430FR5994__)

    // Configure Timer A1 for Manchester timing
    TA1CTL = MC__STOP | TACLR;
    TA1CCTL0 = 0;

    TA1CCR0 = (sample_interval_us * (SMCLK_HZ / 1000000UL)) - 1;

    // Enable CCR0 interrupt
    TA1CCTL0 = CCIE;

    // Configure timer: SMCLK source, no division, stopped initially, clear timer
    TA1CTL = TASSEL__SMCLK | ID__1 | MC__STOP | TACLR;

#endif
}

static void manchester_start_timer()
{
#if defined(NRF52840_XXAA)
    NRF_TIMER4->TASKS_START = 1;
#elif defined(__MSP430FR5994__)
    // Clear timer and start in Up mode - timer will restart automatically at CCR0
    TA1CTL |= TACLR;    // Clear timer counter
    TA1CCTL0 &= ~CCIFG; // Clear any pending CCR0 interrupt flag
    TA1CCTL0 |= CCIE;   // Ensure CCR0 interrupt is enabled
    TA1CTL &= ~MC_3;    // Clear mode bits first
    TA1CTL |= MC__UP;   // Start in Up mode
    LOG("MSP430 Timer A1 started in Up mode, CCR0=%u\n", TA1CCR0);
#endif
}

static void manchester_stop_timer()
{
#if defined(NRF52840_XXAA)
    NRF_TIMER4->TASKS_STOP = 1;
#elif defined(__MSP430FR5994__)
    // Stop Timer A1 completely and reset for next use
    TA1CTL &= ~MC_3;    // Clear mode control bits (stop timer)
    TA1CTL |= TACLR;    // Clear timer counter
    TA1CCTL0 &= ~CCIFG; // Clear any pending interrupt flags

    // Reset interrupt_flag to prevent hanging
    interrupt_flag = 0;

    LOG("MSP430 Timer A1 stopped and reset\n");
#endif
}

static void rx_cb(uint8_t *data, uint8_t data_size, void *udata)
{
    if (data_size < 2)
    {
        return;
    }

    // Copy received data to receive_buffer
    if (receive_buffer != NULL && data_size >= 2)
    {
        memcpy(receive_buffer, data, data_size); // Skip device_id and first payload byte
        LOG("Data copied to receive_buffer\n");
    }
}

void manchester_init(uint8_t Tx, uint8_t Rx, uint8_t rate)
{
    tx_pin = Tx;
    rx_pin = Rx;

    gpio_od_init(tx_pin); // Initialize TX pin in open-drain mode
    gpio_od_init(rx_pin); // Initialize RX pin in open-drain mode

    if (rate >= 7 || rate < 0)
        return;

    uint32_t baud_rate = baud_rates[rate];
    uint32_t bit_time_us = 1000000UL / baud_rate;
    uint32_t sample_interval_us = bit_time_us / tx_rate;

    LOG("Baudrate index: %u, tx_rate (ticks per step): %u\n", rate, tx_rate);

    // Initialize the spooky encoder
    enum spooky_encoder_init_res init_result = spooky_encoder_init(
        &enc, encoder_buffer, ENCODER_BUFFER_SIZE, tx_rate);

    // Initialize the spooky decoder
    enum spooky_decoder_init_res decoder_init_result = spooky_decoder_init(
        &dec, decoder_buffer, DECODER_BUFFER_SIZE, rx_cb, NULL);

    if (init_result == SPOOKY_ENCODER_INIT_OK)
    {
        encoder_initialized = true;
        LOG("Manchester encoder initialized successfully\n");
    }
    else
    {
        encoder_initialized = false;
        LOG("Manchester encoder initialization failed: %d\n", init_result);
    }

    if (decoder_init_result == SPOOKY_DECODER_INIT_OK)
    {
        decoder_initialized = true;
        LOG("Manchester decoder initialized successfully\n");
    }
    else
    {
        decoder_initialized = false;
        LOG("Manchester decoder initialization failed: %d\n", decoder_init_result);
    }

    setup_timer(sample_interval_us);

#if defined(NRF52840_XXAA)
#if DEBUG == 1
    // Configure Pin 11 as output for debugging
    gpio_output_init(11);
#endif

    // Ensure TX pin is in released state initially
    LOG("NRF52840 Manchester TX pin %u configured as open-drain\n", tx_pin);
#elif defined(__MSP430FR5994__)
#if DEBUG == 1
    gpio_output_init(ABS_PIN(3, 5));
#endif
    // Ensure TX pin is in released state initially
    LOG("MSP430 Manchester TX pin %u configured as open-drain\n", tx_pin);
#endif
}

bool manchester_receive_array(uint8_t *data, uint8_t size)
{
    if (rx_pin == 244 || tx_pin == 255 || tx_rate == 0)
    {
        LOG("Manchester not initialized properly.\n");
        return false;
    }

    if (!decoder_initialized)
    {
        LOG("Manchester decoder not initialized.\n");
        return false;
    }

    if (data == NULL || size == 0)
    {
        LOG("Invalid data or size for reception.\n");
        return false;
    }

    if (size > ENCODER_BUFFER_SIZE)
    {
        LOG("Data size (%u) exceeds buffer capacity (%u).\n", size, ENCODER_BUFFER_SIZE);
        return false;
    }

    mode = RECEIVE;
    receive_buffer = data;
    memset(data, 0, size); // Clear receive buffer

    LOG("Starting Manchester reception, waiting for data...\n");
    manchester_start_timer();

    bool finish_decoding = false;
    uint32_t timeout_counter = 0;
    const uint32_t max_timeout = 10000000; // Timeout after ~10 seconds

    while (!finish_decoding && timeout_counter < max_timeout)
    {
        while (!interrupt_flag && timeout_counter < max_timeout)
        {
            timeout_counter++;
        }

        if (timeout_counter >= max_timeout)
        {
            LOG("Manchester reception timeout\n");
            manchester_stop_timer();
            return false;
        }

        interrupt_flag = 0; // Reset the interrupt flag

        bool rx_state = read_Rx();

        enum spooky_decoder_step_res step_result = spooky_decoder_step(&dec, rx_state);

        if (step_result == SPOOKY_DECODER_STEP_DONE)
        {
            finish_decoding = true;
            LOG("Manchester reception completed successfully\n");
        }
        else if (step_result < 0)
        {
            LOG("Manchester reception error: %d\n", step_result);
            manchester_stop_timer();
            return false;
        }
    }

    manchester_stop_timer();
    return finish_decoding;
}

void manchester_transmit_array(uint8_t *data, uint8_t size)
{
    if (tx_pin == 255 || tx_rate == 0)
    {
        LOG("Manchester not initialized properly.\n");
        return;
    }

    if (!encoder_initialized)
    {
        LOG("Manchester encoder not initialized.\n");
        return;
    }

    if (data == NULL || size == 0)
    {
        LOG("Invalid data or size for transmission.\n");
        return;
    }

    if (size > ENCODER_BUFFER_SIZE)
    {
        LOG("Data size (%u) exceeds buffer capacity (%u).\n", size, ENCODER_BUFFER_SIZE);
        return;
    }

    LOG("Starting Manchester transmission of %u bytes\n", size);

    mode = SEND;

    spooky_encoder_clear(&enc);

    enum spooky_encoder_enqueue_res enqueue_result =
        spooky_encoder_enqueue(&enc, data, size);

    if (enqueue_result != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        LOG("Failed to enqueue data for transmission: %d\n", enqueue_result);
        return;
    }

    manchester_start_timer();

    bool transmission_complete = false;
    uint32_t timeout_counter = 0;

    interrupt_flag = 0; // Reset interrupt flag

    while (!transmission_complete)
    {
        while (interrupt_flag == 0)
        {
        }

        interrupt_flag = 0; // Reset the flag

        // Step the encoder
        enum spooky_encoder_step_res step_result = spooky_encoder_step(&enc);

        switch (step_result)
        {
        case SPOOKY_ENCODER_STEP_OK_DONE:
            set_TX(false); // Ensure TX is off
            transmission_complete = true;
            LOG("Manchester transmission completed successfully\n");
            break;
        case SPOOKY_ENCODER_STEP_OK_LOW:
            set_TX(false);
            break;
        case SPOOKY_ENCODER_STEP_OK_HIGH:
            set_TX(true);
            break;
        case SPOOKY_ENCODER_STEP_OK:
            /* leave as-is */
            break;
        default:
            LOG("Encoder step error: %d\n", step_result);
            transmission_complete = true;
            break;
        }

        timeout_counter++;
    }

    manchester_stop_timer();
    set_TX(false); // Ensure TX is off when done

    // Reset interrupt flag for next transmission
    interrupt_flag = 0;

    LOG("Manchester transmission cleanup completed\n");
}

#if defined(NRF52840_XXAA)
void TIMER4_IRQHandler(void)
{
    if (NRF_TIMER4->EVENTS_COMPARE[0])
    {
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;

#if DEBUG == 1
        // Toggle pin for debugging (Pin 11)
        NRF_P0->OUT ^= (1UL << 11);
#endif

        // Set interrupt flag to signal main loop
        interrupt_flag = 1;
    }
}
#elif defined(__MSP430FR5994__)
__attribute__((interrupt(TIMER1_A0_VECTOR))) void TIMER1_A0_ISR(void)
{
    // Clear the CCR0 interrupt flag
    TA1CCTL0 &= ~CCIFG;
#if DEBUG == 1
    P3OUT ^= BIT5;
#endif

    interrupt_flag = 1;

    // Clear any pending interrupt flags to prevent stuck interrupts
    TA1IV;
}
#endif
