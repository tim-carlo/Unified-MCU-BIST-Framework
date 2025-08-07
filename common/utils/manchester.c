
#include "manchester.h"
#include "printf.h"
#include "nrf52840_helper.h"

#define TIMEOUT 1000 // Timeout in milliseconds for transmission
#define DEBUG 1      // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif


enum Mode {
  SEND,
  RECEIVE
};
static volatile enum Mode mode = RECEIVE;

static volatile uint8_t tx_pin = 255;
static volatile uint8_t rx_pin = 255;
static volatile uint8_t tx_rate = 0;

static volatile uint16_t interrupt_flag = 0;

// Simple counter to verify timer is working
static volatile uint32_t timer_interrupt_flag = 0;

#define ENCODER_BUFFER_SIZE 32
static uint8_t encoder_buffer[ENCODER_BUFFER_SIZE];
static struct spooky_encoder enc;
static bool encoder_initialized = false;
static uint32_t timeout = 0;
static uint32_t interrupt_flag = 0;

static void set_TX(bool state)
{
    if (state)
    {
        gpio_drive_low(tx_pin); // Set pin to open-drain mode
    }
    else
    {
        release_gpio_open_drain(tx_pin); // Release pin from open-drain mode
    }
}
static void setup_timer()
{
    LOG("Setting up 50µs timer...\n");
    u_int16_t sample_interval_us = 50;
    
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
    NRF_TIMER4->TASKS_START = 1;

#elif defined(__MSP430FR5994__)
    uint32_t baudrate_tbl[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400};
    uint32_t baud = baudrate_tbl[speedFactor];
    uint32_t bit_time_us = 1000000UL / baud;
    uint32_t sample_interval_us = bit_time_us / 8; // SAMPLES_PER_BIT = 8

    // RX-Pin als Open-Drain-Eingang
    gpio_open_drain(rx_pin_num);

    // Configure Timer A4 for Manchester RX
    stop_timer(TIMER_A4);
    volatile uint16_t *ctl = GET_TxxCTL(TIMER_A4);
    volatile uint16_t *ccr0 = GET_TxxCCR0(TIMER_A4, 0);
    *ctl = TASSEL__SMCLK | MC__UP | TACLR;                     // SMCLK, Up Mode, Clear
    *ccr0 = (sample_interval_us * (SMCLK_HZ / 1000000UL)) - 1; // Set CCR0 for sample interval
    *ctl |= TAIE;                                              // Enable Timer Overflow A4 interrupt

#endif
}
static void step_tx(void)
{
    enum spooky_encoder_step_res res;
    res = spooky_encoder_step(&enc);
    switch (res)
    {
    case SPOOKY_ENCODER_STEP_OK_DONE:
        timeout = TIMEOUT;
        /* fall through */
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
        LOG("Encoder step error: %d\n", res);
        /* Handle error appropriately */
    }
}

void manchester_init(uint8_t Tx, uint8_t Rx, uint8_t rate)
{
    tx_pin = Tx;
    rx_pin = Rx;

    if (rate >= 7 || rate < 0)
        return;

    tx_rate = baud_rates[rate];

    // Initialize the spooky encoder
    enum spooky_encoder_init_res init_result = spooky_encoder_init(
        &enc, encoder_buffer, ENCODER_BUFFER_SIZE, tx_rate);

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

#if defined(NRF52840_XXAA)
    // Configure Pin 11 as output BEFORE setting up the timer
    LOG("Configuring Pin 11 as output...\n");
    NRF_P0->PIN_CNF[11] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                          (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                          (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                          (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    // Set initial state and test pin manually
    NRF_P0->OUT |= (1 << 11);  // Set high
    LOG("Pin 11 set HIGH\n");
    delay_ms(100);
    NRF_P0->OUT &= ~(1 << 11); // Set low
    LOG("Pin 11 set LOW\n");
    delay_ms(100);
    NRF_P0->OUT |= (1 << 11);  // Set high again
    LOG("Pin 11 set HIGH again\n");
    delay_ms(100);
#endif

    setup_timer();
}

void manchester_begin_receive(void)
{
    if (rx_pin == 244 || tx_pin == 255 || tx_rate == 0)
    {
        LOG("Manchester not initialized properly.\n");
        return;
    }
    mode = RECEIVE;
}

void manchester_stop_receive(void)
{
    if (rx_pin == 244 || tx_pin == 255 || tx_rate == 0)
    {
        LOG("Manchester not initialized properly.\n");
        return;
    }
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
    mode = SEND;

    // Clear any previous transmission
    spooky_encoder_clear(&enc);

    // Enqueue the new data for transmission
    enum spooky_encoder_enqueue_res enqueue_result =
        spooky_encoder_enqueue(&enc, data, size);

    if (enqueue_result != SPOOKY_ENCODER_ENQUEUE_OK)
    {
        LOG("Failed to enqueue data for transmission: %d\n", enqueue_result);
        return;
    }

    LOG("Starting Manchester transmission of %u bytes\n", size);

    // Start transmission by stepping through the encoder
    // This is a simple polling approach - in a real implementation
    // you might want to use a timer interrupt for precise timing
    uint32_t max_steps = (size * 8 * 2) + 100; // Estimate max steps needed
    uint32_t step_count = 0;

    enum spooky_encoder_step_res step_result;
    do
    {
        step_result = spooky_encoder_step(&enc);
        step_tx(); // Handle the transmission step

        // Add a small delay between steps to control transmission rate
        // This should ideally be replaced with timer-based stepping
        delay_us(50); // 50 microsecond delay between steps

        step_count++;
        if (step_count > max_steps)
        {
            LOG("Transmission timeout - too many steps\n");
            break;
        }

    } while (step_result != SPOOKY_ENCODER_STEP_OK_DONE &&
             step_result != SPOOKY_ENCODER_STEP_ERROR_EMPTY &&
             step_result != SPOOKY_ENCODER_STEP_ERROR_NULL);

    if (step_result == SPOOKY_ENCODER_STEP_OK_DONE)
    {
        LOG("Manchester transmission completed successfully\n");
    }
    else
    {
        LOG("Manchester transmission ended with error: %d\n", step_result);
    }
}

static void send_irq_handler(void)
{
    interrupt_flag++;
}

static void receive_irq_handler(void)
{


}


#if defined(NRF52840_XXAA)
void TIMER4_IRQHandler(void)
{
    if (NRF_TIMER4->EVENTS_COMPARE[0])
    {
        NRF_TIMER4->EVENTS_COMPARE[0] = 0;
        switch (mode)
        {
        case SEND:
            interrupt_flag++;
            break;
        case RECEIVE:
            interrupt_flag = 1;
            break;
        default:
            break;
        }
    }
}
#endif