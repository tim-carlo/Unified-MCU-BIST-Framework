// Include necessary headers depending on the platform
#if defined(PICO_RP2040)
#include "rp2040_helper.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/stdio.h"
#include <stdint.h>
#include <stdbool.h>

// Define pin mappings for the RP2040
#define LED_RED_PORT 0
#define LED_RED_PIN 0
#define LED_GREEN_PORT 0
#define LED_GREEN_PIN 1
#define TEST_PORT 0
#define TEST_PIN 2

// Replace printf to include chip family in the output
#undef printf
#define printf(fmt, ...) \
    ((void)fprintf(stdout, "[%s] " fmt, get_chip_family_name(), ##__VA_ARGS__))
#endif

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
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
#endif

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_helper.h"

#include "printf.h"
#define TEST_PIN 3
#define TEST_PORT NRF_P1

#define LED_RED_PIN 3
#define LED_RED_PORT NRF_P0
#define LED_GREEN_PIN 4
#define LED_GREEN_PORT NRF_P0

#define ABSOLUTE_PIN_RED (LED_RED_PORT == NRF_P0 ? LED_RED_PIN : LED_RED_PIN + 32)
#define ABSOLUTE_PIN_GREEN (LED_GREEN_PORT == NRF_P0 ? LED_GREEN_PIN : LED_GREEN_PIN + 32)

#define MANCHESTER_TX_PIN 12
#define MANCHESTER_RX_PIN 12

#endif

#include "stack.h"
#include "pindata.h"
#include "check_initial_state.h"
#include "manchester.h"

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 10000

#define MINIMUM_SIGNAL_DURATION_MS 90     // Minimum duration for a valid signal
#define SYN_SIGNAL_DURATION_MS 100        // Duration of SYN signal in ms
#define ACK_SIGNAL_DURATION_MS 500        // Duration of ACK signal in ms
#define SYN_ACK_SIGNAL_DURATION_MS 1000   // Duration of SYN-ACK signal in ms
#define SIGNAL_DURATION_TIME_INACURACY 10 // Allowed inaccuracy in signal duration in ms

#define TIMEOUT_RESPONDER_MODE_MS 2000 // Timeout for responder mode in ms
#define TIMEOUT_SYN_ACK_MS 100         // Duration of ACK signal in ms
#define TIMEOUT_ACK_MS 1000            // Duration of SYN-ACK signal in ms

#define DEBOUNCE_DELAY_US 20

#define MAXIMUM_NUMBER_OF_FALSE_RESPONSES 2 // Maximum number of false responses before blacklisting a pin
#define MAXIMUM_NUMBER_OF_TRIES 5           // Maximum number of tries for a pin before giving up

#define INITIATOR_ROLE 0
#define RESPONDER_ROLE 1

#define PIN_EVENT_QUEUE_SIZE 32

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

// State machine enum
typedef enum
{
    INIT,
    MAYBE_RESPONDER,
    INITIATOR,
    RESPONDER,
    SUCCESS,
    FAILED,
    SCANNED_ALL_PINS
} State;

State state = INIT;

// Flags controlled via interrupts

volatile uint8_t current_driven_pin = NUMBER_OF_GPIO_PINS + 1; // Pin that is currently being driven by the self-driven signal
volatile uint8_t selected_pin = 0;
PinData *selected_pin_data = NULL;     // Pointer to the currently selected pin data
volatile uint64_t black_list_mask = 0; // Global blacklist mask for GPIO pins

// Interrupt handler for rising/falling edges on test pin
void rising_handler(uint32_t pin)
{
    if (pin == current_driven_pin || pin_data[pin].last_falling_edge == INVALID_TIMESTAMP)
        return;
    uint32_t current_ticks = get_timer_ticks(TIMER_B);
    uint32_t signal_duration = timer_diff_ms(pin_data[pin].last_falling_edge, current_ticks);
    printf("->> Rising edge detected on pin %u, duration: %lu ms\n",
           (unsigned)pin, (unsigned long)signal_duration);

    pin_data[pin].last_falling_edge = INVALID_TIMESTAMP;

    if (signal_duration < MINIMUM_SIGNAL_DURATION_MS)
        return;

    PinEvent event = {0};

    if (signal_duration >= SYN_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
        signal_duration <= SYN_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        event = (PinEvent){pin, false, false, true};
        last_event_valid = true;
        printf("-> SYN signal detected on pin %u, duration: %lu ms\n", pin, signal_duration);
        last_event = event; // Store the last event for later processing
    }
    else if (signal_duration >= SYN_ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= SYN_ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        last_event_valid = true;
        event = (PinEvent){pin, false, true, false};
        printf("-> SYN-ACK signal detected on pin %u, duration: %lu ms\n", pin, signal_duration);
        last_event = event; // Store the last event for later processing
    }
    else if (signal_duration >= ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        event = (PinEvent){pin, true, false, false};
        last_event_valid = true;
        printf("-> ACK signal detected on pin %u, duration: %lu ms\n", pin, signal_duration);
        last_event = event; // Store the last event for later processing
    }
}

void falling_handler(uint32_t pin)
{
    if (pin == current_driven_pin)
        return;
    uint32_t current_ticks = get_timer_ticks(TIMER_B);
    printf("Falling edge detected on pin %u\n", pin);

    
    pin_data[pin].last_falling_edge = current_ticks;
}

void turn_off_leds(void)
{
    if (red_led_on)
    {
        gpio_drive_low(ABSOLUTE_PIN_RED);
        red_led_on = false;
    }
    if (green_led_on)
    {
        gpio_drive_low(ABSOLUTE_PIN_GREEN);
        green_led_on = false;
    }
}

void led_test_routine(void)
{
    printf("Starting LED test...\n");

    // Blink red LED 5 times
    for (uint32_t i = 0; i < 5; i++)
    {
        gpio_drive_high(ABSOLUTE_PIN_GREEN);
        delay_ms(200);
        gpio_drive_low(ABSOLUTE_PIN_GREEN);
        delay_ms(200);
    }

    // Blink green LED 5 times
    for (uint32_t i = 0; i < 5; i++)
    {
        gpio_drive_high(ABSOLUTE_PIN_RED);
        delay_ms(200);
        gpio_drive_low(ABSOLUTE_PIN_RED);
        delay_ms(200);
    }

    printf("LED test complete.\n");
}

void send_signal(uint8_t pin, uint32_t duration_ms)
{
    current_driven_pin = pin; // Set the flag to indicate self-driven signal
    gpio_open_drain_drive(pin);
    delay_ms(duration_ms);
    release_gpio_open_drain(pin); // Release the pin after sending the signal

    current_driven_pin = NUMBER_OF_GPIO_PINS + 1; // Reset the flag after sending the signal
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
void set_standart_blacklist_pins(uint64_t *mask)
{
    *mask = 0xFFFFFFFFFFFFFFFFULL;

#if defined(NRF52840_XXAA)
    *mask &= ~(1ULL << 11);
    *mask &= ~(1ULL << 12);
    // *mask &= ~(1ULL << 13);
    // *mask &= ~(1ULL << 14);
#elif defined(__MSP430FR5994__)
    *mask &= ~(1ULL << ABS_PIN(3, 7));
    *mask &= ~(1ULL << ABS_PIN(3, 6));
    //  *mask &= ~(1ULL << ABS_PIN(3, 2));
    //  *mask &= ~(1ULL << ABS_PIN(3, 3));
#endif
}

void print_active_pins_from_mask(uint64_t mask)
{
    printf("Blacklist mask: ");
    for (uint32_t pin = 0; pin < 64; ++pin)
    {
        if ((mask >> pin) & 1)
        {
            printf("%u ", pin);
        }
    }
    printf("\n");
}

int main(void)
{
    io_init();
    initialize_pin_data_array(pin_data, NUMBER_OF_GPIO_PINS);
    printf("Running on %s\n", get_chip_family_name());
    printf("Chip UID: %s\n", get_unique_id_str());

    start_timer(TIMER_B);

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
            continue;          // Skip blacklisted pins (bit = 1)
        gpio_pullup_init(pin); // Initialize non-blacklisted pins (bit = 0) with pull-up resistors
    }

    uint64_t get_initial_state = get_initial_pin_state(0);
    //  black_list_mask |= ~get_initial_state; // Add initial state to the blacklist mask

    printf("Initial pin state: 0x%016llx\n", get_initial_state);
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
        printf("Current state: %d\n", state);
        switch (state)
        {
        case INIT:
        {
            printf("INIT_MODE\n");

            // Wait for a random delay before initiating
            uint32_t initial_delay = random32() % (INITIAL_DELAY_MAX_MS + 1);

            uint32_t random_pin = select_random_non_blacklisted_and_not_successful_pin(pin_data, black_list_mask);

            if (random_pin == 255)
            {
                printf("No valid pins found, exiting.\n");
                state = SCANNED_ALL_PINS;
                break;
            }

            set_selected_pin(random_pin);

            printf("Selected pin: %u\n", selected_pin);
            printf("Selected pin data: %u\n", selected_pin_data->pin);
            printf("Initial delay: %lu ms\n", initial_delay);

            start_timer(TIMER_A);
            uint64_t start_ticks = get_timer_ticks(TIMER_A);

            bool active_signal_detected = false;

            // Wait during initial delay — if a signal is detected, we become responder
            while (timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < initial_delay)
            {
                Stack *active_pins_stack = create_stack(NUMBER_OF_GPIO_PINS, sizeof(uint8_t));
                uint64_t tmp_mask = black_list_mask | (1ULL << selected_pin); // Exclude the selected pin from the search
                push_active_pins_except_blacklist_to_stack(active_pins_stack, 0, tmp_mask);

                if (!is_stack_empty(active_pins_stack))
                {
                    uint8_t pin;
                    stack_pop(active_pins_stack, &pin);

                    set_selected_pin(pin); // Set the selected pin to the active pin
                    printf("Active pin detected: %u\n", selected_pin);
                    active_signal_detected = true;
                }
                free_stack(active_pins_stack);
            }

            stop_timer(TIMER_A);

            // Turn off LEDs
            turn_off_leds();

            // Switch to the next mode depending on whether we saw a signal
            if (active_signal_detected)
            {
                state = MAYBE_RESPONDER;
            }
            else
            {
                printf("No signal received, becoming initiator.\n");

                state = INITIATOR;
            }
            break;
        }
        case MAYBE_RESPONDER:
        {

            printf("MAYBE_RESPONDER_MODE\n");

            start_timer(TIMER_A);
            bool signal_received = false;

            uint64_t ticks_at_starting_point = get_timer_ticks(TIMER_A);
            // Create a stack to hold active pins

            bool another_active_pin_detected = false;

            while ((timer_diff_ms(ticks_at_starting_point, get_timer_ticks(TIMER_A)) < TIMEOUT_RESPONDER_MODE_MS) && !signal_received)
            {
                // Check for active pins except the selected one
                Stack *active_pins_stack = create_stack(NUMBER_OF_GPIO_PINS, sizeof(uint8_t));
                uint64_t tmp_mask = black_list_mask | (1ULL << selected_pin); // Exclude the selected pin from the search
                push_active_pins_except_blacklist_to_stack(active_pins_stack, 0, tmp_mask);

                if (!is_stack_empty(active_pins_stack))
                {
                    uint8_t pin;
                    stack_pop(active_pins_stack, &pin);

                    printf("Another active pin detected: %u\n", pin);
                    another_active_pin_detected = true;
                    set_selected_pin(pin); // Set the selected pin to the active pin
                }
                free_stack(active_pins_stack);

                // Check for SYN event in the pin_events
                if (last_event_valid && last_event.is_syn && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    printf("SYN event received on pin %u \n", last_event.pin);
                    set_syn(&pin_data[selected_pin], true);
                    set_role(&pin_data[selected_pin], RESPONDER_ROLE);
                    last_event_valid = false; // Reset after processing
                    break;
                }
            }

            if (signal_received && !another_active_pin_detected)
            {
                // We received a SYN signal, so we become the responder
                printf("Received SYN signal, becoming RESPONDER\n");
                state = RESPONDER;
            }
            else if (another_active_pin_detected)
            {
                // Another active pin was detected, we become the initiator
                printf("Another active pin detected, going back to MAYBE_RESPONDER.\n");

                state = MAYBE_RESPONDER;
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
                printf("No SYN signal received, go back to INIT_MODE.\n");
                state = INIT;
            }

            break;
        }

        case INITIATOR:
        {
            set_role(selected_pin_data, INITIATOR_ROLE);

            start_timer(TIMER_A);

            // Send SYN signal as initiator
            current_driven_pin = selected_pin; // Set the flag to indicate self-driven signal
            printf("CURRENT_DRIVEN_PIN: %u \n", current_driven_pin);
            printf("INITIATOR_MODE: Sending SYN signal\n");
            uint32_t ticks_at_starting_point = get_timer_ticks(TIMER_A);

            gpio_open_drain_drive(selected_pin);
            printf("Sending SYN signal on pin %u \n", selected_pin);
            bool received_other_signal = false;

            while ((timer_diff_ms(ticks_at_starting_point, get_timer_ticks(TIMER_A)) < SYN_SIGNAL_DURATION_MS) && !received_other_signal)
            {
                Stack *active_pins_stack = create_stack(NUMBER_OF_GPIO_PINS, sizeof(uint8_t));
                uint64_t tmp_mask = black_list_mask | (1ULL << selected_pin);
                push_active_pins_except_blacklist_to_stack(active_pins_stack, 0, tmp_mask);

                if (!is_stack_empty(active_pins_stack))
                {
                    uint8_t pin;
                    stack_pop(active_pins_stack, &pin);
                    set_selected_pin(pin);
                    printf("Another pin is high: %u, switching to MAYBE_RESPONDER\n", selected_pin);
                    received_other_signal = true;
                    free_stack(active_pins_stack);
                    break;
                }
                free_stack(active_pins_stack);
            }
            release_gpio_open_drain(selected_pin); // Release the pin after sending the signal
            printf("SYN signal sent on pin %u\n", selected_pin);
            current_driven_pin = NUMBER_OF_GPIO_PINS + 1; // Reset the flag after sending the signal
            stop_timer(TIMER_A);

            if (received_other_signal)
            {
                state = MAYBE_RESPONDER;
                break;
            }
            printf("SYN signal sent, waiting for SYN-ACK signal...\n");
            start_timer(TIMER_A);
            uint32_t syn_ack_timeout = TIMEOUT_SYN_ACK_MS;
            bool timeout_inceased = false;
            bool signal_received = false;

            while ((timer_diff_ms(ticks_at_starting_point, get_timer_ticks(TIMER_A)) < syn_ack_timeout) && !signal_received)
            {
                if ((gpio_read(selected_pin) == 0) && !timeout_inceased)
                {
                    // If the line is low, increase the timeout
                    syn_ack_timeout += SYN_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY; // Increase timeout by SYN signal duration
                    // Log the increase as this should be done only once
                    timeout_inceased = true;
                }
                if (last_event_valid && last_event.is_syn_ack && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    printf("Received SYN-ACK signal\n");
                    set_syn_ack(selected_pin_data, true);
                    // Acknowledge the SYN-ACK signal
                    send_signal(selected_pin, ACK_SIGNAL_DURATION_MS);
                    set_ack(selected_pin_data, true);
                    last_event_valid = false; // Reset after processing
                    break;
                }
            }
            if (signal_received)
            {
                printf("Received SYN-ACK signal\n");
                // Acknowledge the SYN-ACK signal
                send_signal(selected_pin, ACK_SIGNAL_DURATION_MS);
                state = SUCCESS;
            }
            else
            {
                printf("No SYN-ACK signal received, switching to FAILED_MODE.\n");
                state = FAILED;
            }

            break;
        }

        case RESPONDER:
        {
            set_role(selected_pin_data, RESPONDER_ROLE);
            printf("RESPONDER_MODE\n");

            // Acknowledge the initial signal
            send_signal(selected_pin, SYN_ACK_SIGNAL_DURATION_MS);

            bool signal_received = false;
            start_timer(TIMER_A);
            uint32_t start_ticks = get_timer_ticks(TIMER_A);

            uint32_t ack_timeout = TIMEOUT_ACK_MS;
            bool increase_timeout = false;

            // Wait for final signal from initiator
            while ((timer_diff_ms(start_ticks, get_timer_ticks(TIMER_A)) < ack_timeout) && !signal_received)
            {
                if ((gpio_read(selected_pin) == 0) && !increase_timeout)
                {
                    // If the line is low, increase the timeout
                    ack_timeout += ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY; // Increase timeout by ACK signal duration
                    printf("Increasing ACK timeout to %lu ms\n", ack_timeout);
                    increase_timeout = true;
                }

                if (last_event_valid && last_event.is_ack && last_event.pin == selected_pin)
                {
                    signal_received = true;
                    printf("Received ACK signal\n");
                    last_event_valid = false; // Reset after processing
                }
            }
            stop_timer(TIMER_A);
            if (signal_received)
            {
                state = SUCCESS;
            }
            else
            {
                printf("No final signal received, switching to ERROR_MODE.\n");
                state = FAILED;
            }

            break;
        }

        case FAILED:
        {
            printf("Number of tries: %d\n", selected_pin_data->num_tries);
            selected_pin_data->num_tries = selected_pin_data->num_tries + 1; // Increment the number of tries
            if (selected_pin_data->num_tries >= MAXIMUM_NUMBER_OF_TRIES)
            {
                printf("Maximum number of tries reached for pin %u, blacklisting it.\n", selected_pin);

                selected_pin_data->num_tries = selected_pin_data->num_tries + 1; // Increment the number of tries
                selected_pin_data->error_reason = ERROR_REASON_TRIES_EXCEEDED;   // Set error reason to tries exceeded

                // Set the blacklisted status

                set_blacklisted(selected_pin_data, true);
                set_blacklisted_in_mask(&black_list_mask, selected_pin);
                printf("Pin %u blacklisted.\n", selected_pin);
                print_active_pins_from_mask(black_list_mask);

                // Reset state
                state = INIT;

                break;
            }
            printf("Retrying handshake...\n");
            // Reset state
            state = INIT;

            break;
        }

        case SUCCESS:
        {

            set_successful(selected_pin_data, true);
            set_blacklisted(selected_pin_data, false); // Ensure the pin is not blacklisted
            set_blacklisted_in_mask(&black_list_mask, selected_pin);
            gpio_drive_high(ABSOLUTE_PIN_GREEN);
            green_led_on = true;

            delay_ms(1000);

            // Reset state
            state = INIT;
            break;
        }
        case SCANNED_ALL_PINS:
        {
            printf("All pins scanned, exiting...\n");
            delay_ms(1000);
            break;
        }
        }
    }

    return 0;
}
