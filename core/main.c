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
#define DEBUG_PIN3 ABS_PIN(8, 1) // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 ABS_PIN(8, 2) // Additional debug pin, can be

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
#define DEBUG_PIN3 39 // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 40 // Additional debug pin, can be changed as needed

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
#define PRESCALER_DIV 8           // Prescaler division factor for the timer
#define READER_TICKS ((READER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))
#define MANAGER_TICKS ((MANAGER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))

#define SYN_DURATION 200     // Duration of SYN signal in microseconds
#define SYN_ACK_DURATION 600 // Duration of SYN_ACK signal in microseconds
#define ACK_DURATION 1100    // Duration of ACK signal in microseconds

#define SYN_CYCLES ((uint32_t)SYN_DURATION * 1000UL / READER_INTERVAL_US)
#define SYN_ACK_CYCLES ((uint32_t)SYN_ACK_DURATION * 1000UL / READER_INTERVAL_US)
#define ACK_CYCLES ((uint32_t)ACK_DURATION * 1000UL / READER_INTERVAL_US)

#define SIGNAL_INACCURACY 100 // Signal inaccuracy in milliseconds
#define CYCLE_INACCURACY ((uint32_t)SIGNAL_INACCURACY * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_WAITING_CYCLES 500 // Maximum waiting cycles for a signal

#define MIN_SYN_CYCLES ((uint32_t)(SYN_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_ACK_CYCLES ((uint32_t)(ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)

#define MAXIMUM_SYN_CYCLES ((uint32_t)(SYN_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_ACK_CYCLES ((uint32_t)(ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable
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

void debug_output_binary(uint8_t value)
{
    // Output the 2 LSBs of value (0-3) on DEBUG_PIN3 and DEBUG_PIN4
    if (value & 0x01)
        gpio_drive_high(DEBUG_PIN3);
    else
        gpio_drive_low(DEBUG_PIN3);

    if (value & 0x02)
        gpio_drive_high(DEBUG_PIN4);
    else
        gpio_drive_low(DEBUG_PIN4);
}
typedef struct
{
    uint16_t cycles;
    void (*on_complete)(PinData *);
} TaskDef;

void on_syn_complete(PinData *data)
{
    // SYN task completed, prepare for SYN_ACK
    set_syn(data, true);           // Set SYN flag
    set_initiator_syn(data, true); // Mark as initiator for SYN
    data->sending_counter = 0;     // Reset sending counter for next task
}
void on_syn_ack_complete(PinData *data)
{
    // SYN_ACK task completed, prepare for ACK
    set_syn_ack(data, true);          // Set SYN_ACK flag
    set_initiator_synack(data, true); // Mark as initiator for SYN_ACK
    data->sending_counter = 0;        // Reset sending counter for next task
}
void on_ack_complete(PinData *data)
{
    // ACK task completed, reset task to none
    set_ack(data, true);
    set_initiator_ack(data, true); // Mark as initiator for ACK
    data->sending_counter = 0;     // Reset sending counter
    if (is_successful(data))
    {
        data->successfull_handshakes++; // Increment successful handshakes counter
        if (data->successfull_handshakes >= 5)
        {
            initial_state_mask |= (1ULL << data->pin);
        }
    }
}


// Inspired from Hacker’s Delight by Henry S. Warren, Jr.
typedef struct
{
    uint64_t mask; // Bitmask representing the bitmap
    uint8_t pos;   // Current position in the bitmap
} BitmapIterator;

static inline void bitmap_iterator_init(BitmapIterator *it, uint64_t mask)
{
    it->mask = mask;
    it->pos = 0;
}

static inline bool bitmap_iterator_next(BitmapIterator *it, uint8_t *out_pin)
{
    while (it->mask)
    {
        // Find lowest set bit
        uint8_t bit = __builtin_ctzll(it->mask); 
        it->mask &= it->mask - 1;                // delete lowest set bit
        *out_pin = bit;
        return true;
    }
    return false; // No more set bits
}

// Lookup tables are used to avoid switch-case statements in the ISR and optimize performance
const TaskDef task_lut[] = {
    [TASK_NONE] = {0, NULL},
    [TASK_JOB_SYN] = {SYN_CYCLES, &on_syn_complete},
    [TASK_JOB_SYN_ACK] = {SYN_ACK_CYCLES, &on_syn_ack_complete},
    [TASK_JOB_ACK] = {ACK_CYCLES, &on_ack_complete}};

void reader_isr(void)
{
    gpio_drive_high(DEBUG_PIN1); // Set debug pin high to indicate ISR entry
    BitmapIterator it;
    uint64_t all_pins_mask = (NUMBER_OF_GPIO_PINS >= 64)
                                 ? ~0ULL
                                 : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);

    bitmap_iterator_init(&it, ~initial_state_mask & all_pins_mask);

    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        PinData *data = &pin_data[pin];
        if (initial_state_mask & (1ULL << pin))
            continue; // Skip blacklisted pins

        bool something_done = false; // Flag to track if any task was done

        if (data->waiting_counter >= MAXIMUM_WAITING_CYCLES)
        {
            set_task(data, TASK_JOB_SYN);
            data->waiting_counter = 0; // Reset waiting counter
        }

        // Check if a task is conducted on this pin
        PinDataTask task = get_task(data);
        bool pin_state = gpio_read(pin);

        if (task != TASK_NONE)
        {
            something_done = true; // At least one task is being done
            data->waiting_counter = 0;
            //  Collision detection:
            //  Before sending, check if the pin is high and we are sending
            if (data->sending_counter == 0 && !pin_state)
            {
                set_task(data, TASK_NONE); // Reset task
            }
            else
            {
                uint16_t required_cycles = task_lut[task].cycles;
                if (data->sending_counter >= required_cycles)
                {
                    // Reset when task is complete
                    gpio_od_release(pin); // Release the pin
                    set_task(data, TASK_NONE);
                    task_lut[task].on_complete(data);
                }
                else
                {
                    gpio_od_hold_low(pin);   // Drive pin low to send signal
                    data->sending_counter++; // Increment cycle counter
                }
            }
        }
        else
        {
            if (!pin_state) // if a signal is received
            {
                something_done = true;     // At least one task is being done
                data->receiving_counter++; // Increment receiving counter
                debug_output_binary(1);
            }
        }
        if (pin_state) // if no signal is received, reset the receiving counter
        {
            uint16_t receive_counter = data->receiving_counter;
            if (receive_counter >= MIN_SYN_CYCLES && receive_counter <= MAXIMUM_SYN_CYCLES)
            {
                // SYN signal detected
                set_syn(data, true);
                set_initiator_syn(data, false); // Mark as responder for SYN
                set_task(data, TASK_JOB_SYN_ACK);
                debug_output_binary(2);
                data->receiving_counter = 0; // Reset receiving counter after SYN
                something_done = true;       // Mark that something was done
            }
            else if (receive_counter >= MIN_SYN_ACK_CYCLES && receive_counter <= MAXIMUM_SYN_ACK_CYCLES)
            {
                // SYN_ACK signal detected
                set_syn_ack(data, true);
                set_initiator_synack(data, false); // Mark as responder for SYN_ACK
                set_task(data, TASK_JOB_ACK);
                debug_output_binary(3);
                data->receiving_counter = 0; // Reset receiving counter after SYN_ACK
                something_done = true;       // Mark that something was done
            }
            else if (receive_counter >= MIN_ACK_CYCLES && receive_counter <= MAXIMUM_ACK_CYCLES)
            {
                // ACK signal detected
                set_ack(data, true);
                set_initiator_ack(data, false); // Mark as responder for ACK
                data->receiving_counter = 0;    // Reset receiving counter after ACK
                something_done = true;          // Mark that something was done
                if (is_successful(data))
                {
                    data->successfull_handshakes++; // Increment successful handshakes counter
                    if (data->successfull_handshakes >= 5)
                    {
                        initial_state_mask |= (1ULL << pin);
                    }
                }
            }
            else if (receive_counter > MAXIMUM_ACK_CYCLES)
            {
                // Signal too long, reset counters
                data->receiving_counter = 0;
            }
        }

        if (!something_done)
        {
            data->waiting_counter++; // Increment waiting counter if no task was done
        }

        debug_output_binary(0); // Reset debug output
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
    // TA1CCR1 = MANAGER_TICKS;

    // Enable interrupts
    TA1CCTL0 = CCIE;
    // TA1CCTL1 = CCIE;

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
    gpio_output_init(DEBUG_PIN3);
    gpio_output_init(DEBUG_PIN4);

    //  start_timer(TIMER_B);
    //    start_timer(TIMER_A);

    set_standart_blacklist_pins(&initial_state_mask);
    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (initial_state_mask & (1ULL << pin))
            continue;      // Skip blacklisted pins (bit = 1)
        gpio_od_init(pin); // Initialize non-blacklisted pins (bit = 0) with pull-up resistors
    }

    uint64_t get_initial_state = get_initial_pin_state(0);
    //  black_list_mask |= ~get_initial_state; // Add initial state to the blacklist mask

    LOG("Initial pin state: 0x%016llx\n", get_initial_state);
    print_active_pins_from_mask(initial_state_mask);

    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (!(initial_state_mask & (1ULL << pin)))
        {
            debug_print_pindata(&pin_data[pin]); // Print pin data for non-blacklisted pins
        }
    }
    start_isr_timer(); // Start the ISR timer

    while (1)
    {
    }
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
}
#elif defined(__MSP430FR5994__)

void __attribute__((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR(void)
{
    TA1CCR0 = TA1R + READER_TICKS; // Set next reader interval
    reader_isr();
}

/* void __attribute__((interrupt(TIMER1_A1_VECTOR))) Timer1_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG))
    {
    case TA1IV_TACCR1:
        TA1CCR1 = TA1R + MANAGER_TICKS; // Set next manager interval
        manager_isr();
        break;
    default:
        break;
    }
}
 */
#endif