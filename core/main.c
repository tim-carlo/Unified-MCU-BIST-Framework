
#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
// #include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#include "printf.h"
#include <stdbool.h>
#include <stdint.h>

// Define pin mappings for MSP430
#define LED_RED_PORT 1
#define LED_RED_PIN 0
#define LED_GREEN_PORT 1
#define LED_GREEN_PIN 1
#define ABSOLUTE_PIN_RED ABS_PIN(LED_RED_PORT, LED_RED_PIN)
#define ABSOLUTE_PIN_GREEN ABS_PIN(LED_GREEN_PORT, LED_GREEN_PIN)

#define TIMER_A TIMER_A4
#define TIMER_B TIMER_B0

#define MANCHESTER_TX_PIN ABS_PIN(3, 7)
#define MANCHESTER_RX_PIN ABS_PIN(3, 7)
#define PINA ABS_PIN(3, 6)
#define PINB ABS_PIN(3, 7)

#define DEBUG_PIN1 ABS_PIN(3, 4) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 ABS_PIN(3, 5) // Pin used for debugging, can be changed as needed

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

// #include "nrf52840_gpio.h"

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
// #include "nrf52840_time.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"

#include "printf.h"
#define TEST_PIN 3
#define TEST_PORT NRF_P1

#define LED_RED_PIN 3
#define LED_RED_PORT NRF_P0
#define LED_GREEN_PIN 4
#define LED_GREEN_PORT NRF_P0

#define ABSOLUTE_PIN_RED (LED_RED_PORT == NRF_P0 ? LED_RED_PIN : LED_RED_PIN + 32)
#define ABSOLUTE_PIN_GREEN (LED_GREEN_PORT == NRF_P0 ? LED_GREEN_PIN : LED_GREEN_PIN + 32)

#define PINA 11
#define PINB 12

#define DEBUG_PIN1 26 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 27 // Pin used for debugging, can be changed as needed

#define MANCHESTER_TX_PIN 12
#define MANCHESTER_RX_PIN 12

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS

#endif

#include "stack.h"
#include "timing_pindata.h"
#include "check_initial_state.h"
// #include "manchester.h"
#include "random_utils.h"

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 10000
#define MAXIMUM_NUMBER_OF_FALSE_RESPONSES 2 // Maximum number of false responses before blacklisting a pin
#define MAXIMUM_NUMBER_OF_TRIES 5           // Maximum number of tries for a pin before giving up

#define INITIATOR_ROLE 0
#define RESPONDER_ROLE 1

#define READER_INTERVAL_US 1000   // Reader: every 1 ms
#define MANAGER_INTERVAL_US 10000 // Manager: every 10 ms
#define PRESCALER_DIV 8 // Prescaler division factor for the timer
#define READER_TICKS  ((READER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))
#define MANAGER_TICKS ((MANAGER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))


#define DEBUG 0 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

// Global variables
PinData pin_data[NUMBER_OF_GPIO_PINS]; // Global variable to hold pin data

// State tracking
bool red_led_on = false;
bool green_led_on = false;

volatile bool last_event_valid = false;
volatile uint8_t last_rising_pin = INVALID_PIN; // Global variable to store the last rising pin event

// Flags controlled via interrupts

volatile uint64_t black_list_mask = 0;    // Global blacklist mask for GPIO pins
volatile uint64_t initial_state_mask = 0; // Mask to store the initial state of pins

uint32_t number_of_successful_handshakes = 0; // Counter for successful handshakes
bool disable_interrupts = false;              // Flag to disable interrupts during critical sections

void manager_isr(void)
{
    gpio_drive_high(DEBUG_PIN2); // Set debug pin high to indicate ISR entry
    // Insert 10 NOPs for timing adjustment
    for (volatile int i = 0; i < 10; ++i)
    {
        __asm__ __volatile__("nop");
    }
    gpio_drive_low(DEBUG_PIN2); // Set debug pin low to indicate ISR exit
}

void reader_isr(void)
{
    gpio_drive_high(DEBUG_PIN1); // Set debug pin high to indicate ISR entry
    // Insert 10 NOPs for timing adjustment
    for (volatile int i = 0; i < 10; ++i)
    {
        __asm__ __volatile__("nop");
    }
    gpio_drive_low(DEBUG_PIN1); // Set debug pin low to indicate ISR exit
}

void start_isr_timer(void)
{

#if defined(NRF52840_XXAA)

    // Stop timer first to ensure clean configuration
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;

    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
    NRF_TIMER3->PRESCALER = 4 << TIMER_PRESCALER_PRESCALER_Pos; // 1 MHz -> 1 tick = 1 µs

    // Reader interval -> Compare channel 0
    NRF_TIMER3->CC[0] = READER_INTERVAL_US;

    // Manager interval -> Compare channel 1
    NRF_TIMER3->CC[1] = MANAGER_INTERVAL_US;

    NRF_TIMER3->SHORTS = 0;

    // Enable both interrupts
    NRF_TIMER3->INTENSET =
        (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos) |
        (TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos);

    NVIC_ClearPendingIRQ(TIMER3_IRQn);
    NVIC_EnableIRQ(TIMER3_IRQn);

    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->TASKS_START = 1;

#elif defined(__MSP430FR5994__)

    // Stop and clear timer
    TA1CTL = MC__STOP | TACLR;

    TA1CCR0 = READER_TICKS;
    TA1CCR1 = MANAGER_TICKS;

    // Enable interrupts
    TA1CCTL0 = CCIE;
    TA1CCTL1 = CCIE;

    TA1CTL = TASSEL__SMCLK | ID__8 | MC__CONTINUOUS | TACLR;

#endif
}

void stop_isr_timer(void)
{
#if defined(NRF52840_XXAA)
    NRF_TIMER3->TASKS_STOP = 1;
#elif defined(__MSP430FR5994__)
    // Stop Timer A1 completely and reset for next use
    TA1CTL &= ~MC_3;    // Clear mode control bits (stop timer
    TA1CTL |= TACLR;    // Clear timer counter
    TA1CCTL0 &= ~CCIFG; // Clear any pending CCR0 interrupt
    TA1CCTL0 &= ~CCIE;  // Disable CCR0 interrupt
    TA1R = 0;           // Reset timer counter
#endif
}

void set_standart_blacklist_pins(volatile uint64_t *mask) // ← volatile hinzufügen
{
    *mask = 0xFFFFFFFFFFFFFFFFULL;

#if defined(NRF52840_XXAA)
    *mask &= ~(1ULL << 12); // Pin 12
    *mask &= ~(1ULL << 11); // Pin 11

#elif defined(__MSP430FR5994__)
    *mask &= ~(1ULL << ABS_PIN(3, 7)); // Pin 23
    *mask &= ~(1ULL << ABS_PIN(3, 6)); // Pin 22
#endif
}

void print_active_pins_from_mask(uint64_t mask)
{
    LOG("Blacklist mask: ");
    for (uint8_t pin = 0; pin < 64; ++pin)
    {
        if ((mask >> pin) & 1)
        {
            LOG("%u ", (unsigned int)pin);
        }
    }
    LOG("\n");
}

int main(void)
{
    io_init();
    initialize_pin_data_array(pin_data, NUMBER_OF_GPIO_PINS);
    LOG("Running on %s\n", get_chip_family_name());
    LOG("Chip UID: %s\n", get_unique_id_str());

    gpio_output_init(DEBUG_PIN1);
    gpio_output_init(DEBUG_PIN2);

    start_timer(TIMER_B);
    start_timer(TIMER_A);

    gpio_output_init(ABSOLUTE_PIN_GREEN);
    gpio_output_init(ABSOLUTE_PIN_RED);

// Since the Out Register is initialized to 1, we need to set the LEDs to low
#if defined(__MSP430FR5994__)
    gpio_drive_low(ABSOLUTE_PIN_GREEN);
    gpio_drive_low(ABSOLUTE_PIN_RED);
#endif

    set_standart_blacklist_pins(&black_list_mask);
    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (black_list_mask & (1ULL << pin))
            continue;      // Skip blacklisted pins (bit = 1)
        gpio_od_init(pin); // Initialize non-blacklisted pins (bit = 0) with pull-up resistors
    }

    uint64_t get_initial_state = get_initial_pin_state(0);
    //  black_list_mask |= ~get_initial_state; // Add initial state to the blacklist mask

    LOG("Initial pin state: 0x%016llx\n", get_initial_state);
    print_active_pins_from_mask(black_list_mask);

    start_isr_timer(); // Start the ISR timer
}

// Interrupt Service Routine for Timer A3
#if defined(NRF52840_XXAA)
void TIMER3_IRQHandler(void)
{
    if (NRF_TIMER3->EVENTS_COMPARE[0])
    {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;
        NRF_TIMER3->CC[0] += READER_INTERVAL_US; // Next reader event in 1ms
        reader_isr();                            // Call the reader ISR
    }
    if (NRF_TIMER3->EVENTS_COMPARE[1])
    {
        NRF_TIMER3->EVENTS_COMPARE[1] = 0;
        NRF_TIMER3->CC[1] += MANAGER_INTERVAL_US; // Nächstes Manager-Event in 10ms
        manager_isr();                            // Call the manager ISR
    }
}
#elif defined(__MSP430FR5994__)

void __attribute__((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR(void)
{
    reader_isr();
    TA1CCR0 = TA1R + READER_TICKS; // Set next reader interval
}

void __attribute__((interrupt(TIMER1_A1_VECTOR))) Timer1_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG))
    {
    case TA1IV_TACCR1:
        manager_isr();
        TA1CCR1 = TA1R + MANAGER_TICKS; // Set next manager interval
        break;
    default:
        break;
    }
}
#endif
