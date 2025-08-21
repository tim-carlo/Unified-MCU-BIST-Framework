

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
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

#define DEBUG_PIN ABS_PIN(3, 5) // Pin used for debugging, can be changed as needed

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

// #include "nrf52840_gpio.h"

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_time.h"
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

#define DEBUG_PIN 26 // Pin used for debugging, can be changed as needed

#define MANCHESTER_TX_PIN 12
#define MANCHESTER_RX_PIN 12

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS

#endif

#include "stack.h"
#include "pindata.h"
#include "check_initial_state.h"
#include "manchester.h"
#include "random_utils.h"

// #include "nrf52840_helper.h"

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 10000

#define MINIMUM_SIGNAL_DURATION_MS 100     // Minimum duration for a valid signal
#define SYN_SIGNAL_DURATION_MS 200         // Duration of SYN signal in ms
#define SYN_ACK_SIGNAL_DURATION_MS 600     // Duration of SYN-ACK signal in ms
#define ACK_SIGNAL_DURATION_MS 1100        // Duration of ACK signal in ms
#define SIGNAL_DURATION_TIME_INACURACY 100 // Allowed inaccuracy in signal duration in ms

#define SIGNAL_BUFFER_DURATION_MS 500 // Duration for which the signal is buffered

#define TIMEOUT_RESPONDER_MODE_MS 1000 // Timeout for responder mode in ms
#define TIMEOUT_SYN_ACK_MS 1200        // Timeout for SYN-ACK signal in ms
#define TIMEOUT_ACK_MS 2000            // Timeout for ACK signal in ms

#define DEBOUNCE_DELAY_US 20

#define MAXIMUM_NUMBER_OF_FALSE_RESPONSES 2 // Maximum number of false responses before blacklisting a pin
#define MAXIMUM_NUMBER_OF_TRIES 5           // Maximum number of tries for a pin before giving up

#define INITIATOR_ROLE 0
#define RESPONDER_ROLE 1

#define PIN_EVENT_QUEUE_SIZE 32
#define DEBUG 0 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

typedef struct
{
    uint8_t pin;     // Pin number
    bool is_ack;     // True if this is an ACK signal, false otherwise
    bool is_syn_ack; // True if this is a SYN-ACK signal, false otherwise
    bool is_syn;     // True if this is a SYN signal, false otherwise
} PinEvent;

// Global variables
PinData pin_data[NUMBER_OF_GPIO_PINS]; // Global variable to hold pin data

// State tracking
bool red_led_on = false;
bool green_led_on = false;

volatile PinEvent last_event;
volatile bool last_event_valid = false;
volatile uint8_t last_rising_pin = INVALID_PIN; // Global variable to store the last rising pin event

// State machine enum
typedef enum
{
    INIT,
    MAYBE_RESPONDER,
    INITIATOR,
    SEND_SYN,
    RESPONDER,
    SUCCESS,
    FAILED,
    SCANNED_ALL_PINS
} State;

State state = INIT;

// Flags controlled via interrupts

volatile uint8_t current_driven_pin = INVALID_PIN; // Pin that is currently being driven by the self-driven signal
volatile uint8_t selected_pin = 0;
PinData *selected_pin_data = NULL;     // Pointer to the currently selected pin data
volatile uint64_t black_list_mask = 0; // Global blacklist mask for GPIO pins

uint32_t number_of_rises = 0; // Counter for the number of rising edges detected
uint32_t number_of_falls = 0; // Counter for the number of falling edges

uint32_t number_of_successful_handshakes = 0; // Counter for successful handshakes

bool disable_interrupts = false; // Flag to disable interrupts during critical sections

// Interrupt handler for rising/falling edges on test pin
void rising_handler(uint8_t pin)
{

    if (disable_interrupts)
    {
       // gpio_drive_low(DEBUG_PIN); // Turn off debug pin if interrupts are disabled
        return;                    // Ignore rising edges if interrupts are disabled
    }
    // Check if pin is valid for processing
    if (pin == current_driven_pin || pin_data[pin].last_falling_edge == INVALID_TIMESTAMP)
    {
      //  gpio_drive_low(DEBUG_PIN); // Turn off debug pin if pin is not valid
        return;
    }
    gpio_drive_high(DEBUG_PIN); // Turn on debug pin to indicate rising edge detected
    number_of_rises++;
    uint32_t current_ticks = get_timer_ticks(TIMER_B);
    uint32_t signal_duration = timer_diff_ms(pin_data[pin].last_falling_edge, current_ticks);
    pin_data[pin].last_falling_edge = INVALID_TIMESTAMP;

    if (signal_duration < MINIMUM_SIGNAL_DURATION_MS || signal_duration > SYN_ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        gpio_drive_low(DEBUG_PIN); // Turn off debug pin if signal duration is invalid
        return;
    }

    PinEvent event = {0};

    if (signal_duration >= SYN_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
        signal_duration <= SYN_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        event = (PinEvent){pin, false, false, true};
        last_event_valid = true;
        // printf("-> SYN signal detected on pin %u, duration: %lu ms\n", (unsigned int)pin, (unsigned long)signal_duration);
        last_event = event; // Store the last event for later processing
    }
    else if (signal_duration >= SYN_ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= SYN_ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        last_event_valid = true;
        event = (PinEvent){pin, false, true, false};
        //  printf("-> SYN-ACK signal detected on pin %u, duration: %lu ms\n", (unsigned int)pin, (unsigned long)signal_duration);
        last_event = event; // Store the last event for later processing
    }
    else if (signal_duration >= ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        event = (PinEvent){pin, true, false, false};
        last_event_valid = true;
        //  printf("-> ACK signal detected on pin %u, duration: %lu ms\n", (unsigned int)pin, (unsigned long)signal_duration);
        last_event = event; // Store the last event for later processing
    }
    else
    {
        //  printf("-> Unknown signal detected on pin %u, duration: %lu ms\n", (unsigned int)pin, (unsigned long)signal_duration);
        last_event_valid = false;
    }
    gpio_drive_low(DEBUG_PIN); // Turn off debug pin after processing the rising edge
}

void falling_handler(uint8_t pin)
{

    if (disable_interrupts)
    {
      //  gpio_drive_low(DEBUG_PIN); // Turn off debug pin if interrupts are disabled
        return;                    // Ignore rising edges if interrupts are disabled
    }

    if (pin == current_driven_pin)
    {
     //   gpio_drive_low(DEBUG_PIN); // Turn off debug pin if pin is currently driven
        return;                    // Ignore falling edges on the currently driven pin
    }
    
    gpio_drive_high(DEBUG_PIN); // Turn on debug pin to indicate falling edge detected

    if (pin_data[pin].last_falling_edge != INVALID_TIMESTAMP)
    {
        gpio_drive_low(DEBUG_PIN); // Turn off debug pin if already processing a falling edge for this pin
        return;                    // Ignore if already processing a falling edge for this pin
    }

    number_of_falls++;
    uint32_t current_ticks = get_timer_ticks(TIMER_B);
    last_rising_pin = pin; // Store the last rising pin event
    pin_data[pin].last_falling_edge = current_ticks;
    gpio_drive_low(DEBUG_PIN); // Turn off debug pin after processing the falling edge
}

void send_signal(uint8_t pin, uint32_t duration)
{
    // enter_critical_section(); // Enter critical section to prevent
    disable_interrupts = true; // Disable interrupts to prevent
                               // current_driven_pin = pin;
    LOG("Sending signal on pin %u for %lu ms\n", (unsigned int)pin, (unsigned long)duration);
    gpio_od_hold_low(pin);
    delay_ms(duration);
    gpio_od_release(pin);
    LOG("Signal sent on pin %u\n", (unsigned int)pin);
    // current_driven_pin = INVALID_PIN;
    disable_interrupts = false; // Re-enable interrupts after sending the signal
    // exit_critical_section(); // Exit critical section
}

void set_selected_pin(uint8_t pin)
{
    if (pin >= NUMBER_OF_GPIO_PINS)
    {
        return; // Invalid pin number, do nothing
    }
    selected_pin = pin;
    selected_pin_data = &pin_data[pin]; // Update the pointer to the selected pin data
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

#if defined(__MSP430FR5994__)
typedef uint16_t irq_state_t;
#elif defined(NRF52840_XXAA)
typedef uint32_t irq_state_t;
#endif

static irq_state_t irq_state;

static inline void enter_critical_section(void)
{
#if defined(__MSP430FR5994__)
    irq_state = __get_interrupt_state();
    __disable_interrupt();
#elif defined(NRF52840_XXAA)
    irq_state = __get_PRIMASK();
    __disable_irq();
#endif
}

static inline void exit_critical_section(void)
{
#if defined(__MSP430FR5994__)
    __set_interrupt_state(irq_state);
#elif defined(NRF52840_XXAA)
    __set_PRIMASK(irq_state);
#endif
}

int main(void)
{
    io_init();
    initialize_pin_data_array(pin_data, NUMBER_OF_GPIO_PINS);
    LOG("Running on %s\n", get_chip_family_name());
    LOG("Chip UID: %s\n", get_unique_id_str());

    gpio_output_init(DEBUG_PIN);

    // for (uint8_t i = 1; i < 11; ++i)
    // {
    //     uint32_t test_duration = i * 100; // Duration in ms
    //     send_signal(PINA, test_duration); // Send a test signal on PINA for 1000 ms
    //     send_signal(PINB, test_duration); // Send a test signal on PINB for 1000 ms
    // }

    start_timer(TIMER_B);
    start_timer(TIMER_A);

    gpio_output_init(ABSOLUTE_PIN_GREEN);
    gpio_output_init(ABSOLUTE_PIN_RED);

// Since the Out Register is initialized to 1, we need to set the LEDs to low
#if defined(__MSP430FR5994__)
    gpio_drive_low(ABSOLUTE_PIN_GREEN);
    gpio_drive_low(ABSOLUTE_PIN_RED);
#endif

    set_standart_blacklist_pins(&black_list_mask); // Set standard blacklist pins
                                                   // print_active_pins_from_mask(black_list_mask);
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

    gpio_listen_on_all_pins_interrupt(
        black_list_mask,
        falling_handler, // Falling edge handler
        rising_handler   // Rising edge handler
    );
    // Send data to test the Manchester encoding

    // led_test_routine();
    while (1)
    {
        LOG("Current state: %d\n", state);
        switch (state)
        {
        case INIT:
        {
            LOG("INIT_MODE\n");

            // Wait for a random delay before initiating
            uint32_t initial_delay = random32() % (INITIAL_DELAY_MAX_MS + 1);

            uint32_t random_pin = select_random_non_blacklisted_and_not_successful_pin(pin_data, black_list_mask);

            if (random_pin == INVALID_PIN)
            {
                LOG("No valid pins found, exiting.\n");
                state = SCANNED_ALL_PINS;
                break;
            }

            set_selected_pin(random_pin);

            LOG("Selected pin: %u\n", selected_pin);
            LOG("Initial delay: %lu ms\n", initial_delay);

            uint64_t start_ticks = get_timer_ticks(TIMER_A);

            bool active_signal_detected = false;

            // Wait during initial delay — if a signal is detected, we become responder
            while (timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < initial_delay)
            {
                if (last_rising_pin != INVALID_PIN)
                {
                    LOG("Detected signal on pin %u, switching to MAYBE_RESPONDER.\n", last_rising_pin);
                    set_selected_pin(last_rising_pin);
                    active_signal_detected = true;

                    last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                }
            }

            // Switch to the next mode depending on whether we saw a signal
            if (active_signal_detected)
            {
                state = MAYBE_RESPONDER;
            }
            else
            {
                state = SEND_SYN;
            }
            break;
        }
        case SEND_SYN:
        {
            LOG("SEND_SYN_MODE\n");

            // Send SYN signal as initiator
            current_driven_pin = selected_pin; // Set the flag to indicate self-driven signal

            gpio_od_hold_low(selected_pin);
            uint32_t start_ticks = get_timer_ticks(TIMER_A);
            LOG("Sending SYN signal on pin %u \n", selected_pin);
            bool received_other_signal = false;

            while ((timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < SYN_SIGNAL_DURATION_MS))
            {
                if (last_rising_pin != INVALID_PIN && last_rising_pin != selected_pin)
                {
                    set_selected_pin(last_rising_pin);
                    received_other_signal = true;
                    last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                    break;
                }
                else if (last_rising_pin != INVALID_PIN && last_rising_pin == selected_pin)
                {
                    last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                }
            }
            gpio_od_release(selected_pin); // Release the pin after sending the signal
            LOG("SYN signal sent on pin %u\n", selected_pin);
            set_syn(selected_pin_data, true); // Set the SYN flag in the pin data

            current_driven_pin = INVALID_PIN; // Reset the flag after sending the signal

            if (received_other_signal)
            {
                state = MAYBE_RESPONDER;
            }
            else
            {
                state = INITIATOR; // Proceed to the next state
            }
            break;
        }
        case MAYBE_RESPONDER:
        {

            LOG("MAYBE_RESPONDER_MODE\n");

            bool signal_received = false;

            uint64_t start_ticks = get_timer_ticks(TIMER_A);
            // Create a stack to hold active pins

            bool another_active_pin_detected = false;

            while ((timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < TIMEOUT_RESPONDER_MODE_MS) && !signal_received && !another_active_pin_detected)
            {
                // if (last_rising_pin != INVALID_PIN && last_rising_pin != selected_pin)
                // {
                //     LOG("Another active pin detected: %u\n", last_rising_pin);
                //     another_active_pin_detected = true;
                //     set_selected_pin(last_rising_pin);
                //     last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                //     break;                         // Exit the loop to process the new selected pin
                // }

                // Check for SYN event in the pin_events
                if (last_event_valid && last_event.is_syn && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    LOG("SYN event received on pin %u \n", last_event.pin);
                    set_syn(&pin_data[selected_pin], true);
                    set_role(&pin_data[selected_pin], RESPONDER_ROLE);
                    last_event_valid = false; // Reset after processing
                    break;
                }
                else if (last_event_valid && last_event.is_syn && last_event.pin != selected_pin)
                {
                    LOG("Received SYN signal on pin %u, but expected on pin %u.\n", last_event.pin, selected_pin);
                    set_selected_pin(last_event.pin);
                    set_syn(&pin_data[selected_pin], true);
                    another_active_pin_detected = true;
                    last_event_valid = false; // Reset after processing
                    break;                    // Exit the loop to process the new selected pin
                }
            }

            if (signal_received && !another_active_pin_detected)
            {
                state = RESPONDER;
            }
            else if (another_active_pin_detected && !signal_received)
            {
                state = RESPONDER;
            }
            else
            {
                // No SYN signal received, we become the initiator

                selected_pin_data->num_false_responses++;
                if (selected_pin_data->num_false_responses >= MAXIMUM_NUMBER_OF_FALSE_RESPONSES)
                {
                    set_blacklisted_in_mask(&black_list_mask, selected_pin);
                    set_blacklisted(selected_pin_data, true);
                    selected_pin_data->error_reason = ERROR_REASON_DISTURBED;
                }
                LOG("No SYN signal received, go back to INIT_MODE.\n");
                state = INIT;
            }

            break;
        }

        case INITIATOR:
        {
            set_role(selected_pin_data, INITIATOR_ROLE);
            LOG("SYN signal sent, waiting for SYN-ACK signal...\n");
            uint32_t start_ticks = get_timer_ticks(TIMER_A);
            uint32_t syn_ack_timeout = TIMEOUT_SYN_ACK_MS;
            bool timeout_inceased = false;
            bool signal_received = false;

            bool another_active_pin_detected = false;

            while ((timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < syn_ack_timeout))
            {
                if (last_rising_pin != INVALID_PIN)
                {
                    if ((last_rising_pin == selected_pin) && !timeout_inceased)
                    {
                        start_ticks = get_timer_ticks(TIMER_A); // Reset the start ticks to the current time

                        // If the line is low, increase the timeout
                        syn_ack_timeout += SYN_SIGNAL_DURATION_MS + SIGNAL_BUFFER_DURATION_MS; // Increase timeout by SYN signal duration
                        // Log the increase as this should be done only once
                        timeout_inceased = true;
                    }
                    // else if (last_rising_pin != selected_pin)
                    // {
                    //     LOG("Received signal on pin %u, but it is not the expected pin %u\n", last_rising_pin, selected_pin);
                    //     set_selected_pin(last_rising_pin);
                    //     another_active_pin_detected = true;
                    // }
                    last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                }
                // Check for SYN-ACK event in the pin_events
                if (last_event_valid && last_event.is_syn_ack && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    last_event_valid = false; // Reset after processing
                    break;
                }

                if (last_event_valid && last_event.is_syn && last_event.pin != selected_pin)
                {
                    LOG("Received SYN signal on pin %u, but expected SYN-ACK.\n", last_event.pin);
                    set_selected_pin(last_event.pin);
                    set_syn(selected_pin_data, true);
                    set_selected_pin(last_rising_pin);
                    another_active_pin_detected = true;

                    last_event_valid = false; // Reset after processing
                    break;                    // Exit the loop to process the new selected pin
                }
            }
            if (another_active_pin_detected && !signal_received)
            {
                state = RESPONDER;
            }
            else if (!another_active_pin_detected && signal_received)
            {
                LOG("Received SYN-ACK signal\n");
                set_syn_ack(selected_pin_data, true);
                // Acknowledge the SYN-ACK signal
                send_signal(selected_pin, ACK_SIGNAL_DURATION_MS);
                set_ack(selected_pin_data, true);
                state = SUCCESS;
            }
            else
            {
                LOG("No SYN-ACK signal received, switching to FAILED_MODE.\n");
                if (timeout_inceased)
                    LOG("Timeout was increased to %lu ms\n", syn_ack_timeout);
                state = FAILED;
            }

            break;
        }

        case RESPONDER:
        {
            set_role(selected_pin_data, RESPONDER_ROLE);
            LOG("RESPONDER_MODE\n");

            // Acknowledge the initial signal
            send_signal(selected_pin, SYN_ACK_SIGNAL_DURATION_MS);
            set_syn_ack(selected_pin_data, true);

            bool signal_received = false;
            uint32_t start_ticks = get_timer_ticks(TIMER_A);

            uint32_t ack_timeout = TIMEOUT_ACK_MS;
            bool timeout_inceased = false;

            // Wait for final signal from initiator
            while ((timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < ack_timeout))
            {
                if (last_rising_pin != INVALID_PIN)
                {
                    if ((last_rising_pin == selected_pin) && !timeout_inceased)
                    {
                        // If the line is low, increase the timeout
                        start_ticks = get_timer_ticks(TIMER_A);                           // Reset the start ticks to the current time
                        ack_timeout = ACK_SIGNAL_DURATION_MS + SIGNAL_BUFFER_DURATION_MS; // Increase timeout by ACK signal duration
                        // Log the increase as this should be done only once
                        timeout_inceased = true;
                    }
                    else
                    {
                        LOG("Received signal on pin %u, but it is not the expected pin %u\n", last_rising_pin, selected_pin);
                    }
                    last_rising_pin = INVALID_PIN; // Reset the last rising pin flag after processing
                }

                if (last_event_valid && last_event.is_ack && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    set_ack(selected_pin_data, true);
                    LOG("Received ACK signal\n");
                    last_event_valid = false; // Reset after processing
                    break;
                }
            }
            if (signal_received)
            {
                state = SUCCESS;
            }
            else
            {
                LOG("No final signal received, switching to ERROR_MODE.\n");
                if (timeout_inceased)
                    LOG("Timeout was increased to %lu ms\n", ack_timeout);
                state = FAILED;
            }

            break;
        }

        case FAILED:
        {
            LOG("Number of tries: %d\n", selected_pin_data->num_tries);
            selected_pin_data->num_tries = selected_pin_data->num_tries + 1; // Increment the number of tries
            if (selected_pin_data->num_tries >= MAXIMUM_NUMBER_OF_TRIES)
            {
                LOG("Maximum number of tries reached for pin %u, blacklisting it.\n", selected_pin);

                selected_pin_data->num_tries = selected_pin_data->num_tries + 1; // Increment the number of tries
                selected_pin_data->error_reason = ERROR_REASON_TRIES_EXCEEDED;   // Set error reason to tries exceeded

                // Set the blacklisted status

                set_blacklisted(selected_pin_data, true);
                set_blacklisted_in_mask(&black_list_mask, selected_pin);
                LOG("Pin %u blacklisted.\n", selected_pin);
                print_active_pins_from_mask(black_list_mask);

                // Reset state
                state = INIT;

                break;
            }
            LOG("Retrying handshake...\n");
            // Reset state
            state = INIT;

            break;
        }

        case SUCCESS:
        {

            set_successful(selected_pin_data, true);
            set_blacklisted(selected_pin_data, false); // Ensure the pin is not blacklisted
            set_blacklisted_in_mask(&black_list_mask, selected_pin);
            green_led_on = true;
            gpio_drive_high(ABSOLUTE_PIN_GREEN); // Turn on the green LED
            number_of_successful_handshakes++;

            printf("Handshake successful on pin %u!\n", selected_pin);

            // Reset state
            state = INIT;
            break;
        }
        case SCANNED_ALL_PINS:
        {
            printf("All pins scanned, exiting...\n");

            // Print final statistics
            debug_pin_data_array_analysis(pin_data, NUMBER_OF_GPIO_PINS);
            printf("Number of rises: %lu, Number of falls: %lu\n", (unsigned long)number_of_rises, (unsigned long)number_of_falls);
            delay_ms(500);
            printf("Number of successful handshakes: %lu\n", (unsigned long)number_of_successful_handshakes);
            break;
        }
        }
    }

    return 0;
}
