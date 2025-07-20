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
#define TEST_PORT 3
#define TEST_PIN 7
#endif

#if defined(NRF52840_XXAA)
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

#endif

#include "stack.h"
#include "pindata.h"
#include "routines/check_initial_state.h"

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 1000

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

typedef struct
{
    uint32_t pin; // Pin number

    uint32_t duration_ms; // Duration of the signal in milliseconds
    bool is_ack;          // True if this is an ACK signal, false otherwise
    bool is_syn_ack;      // True if this is a SYN-ACK signal, false otherwise
    bool is_syn;          // True if this is a SYN signal, false otherwise
} PinEvent;

// Global variables
PinData pin_data[NUMBER_OF_GPIO_PINS]; // Global variable to hold pin data

// State tracking
uint32_t needded_tries = 0;
bool i_was_the_initiator = false;
bool i_was_the_responder = false;
bool red_led_on = false;
bool green_led_on = false;

PinData pin_data_tmp; // Temporary data structure for pin data
Stack *pin_events;    // Stack to hold pin data events

// State machine enum
typedef enum
{
    INIT,
    MAYBE_RESPONDER,
    INITIATOR,
    RESPONDER,
    SUCCESS,
    FAILED
} State;

State state = INIT;

// Flags controlled via interrupts

volatile bool self_driven_signal = false; // Flag to indicate if the signal is self-driven
volatile uint32_t selected_pin = 0;
volatile PinData *selected_pin_data = NULL; // Pointer to the currently selected pin data
uint64_t black_list_mask = 0;      // Global blacklist mask for GPIO pins

// Interrupt handler for rising/falling edges on test pin
void rising_handler(uint32_t pin) // Wird bei STEIGENDER Flanke (HIGH) aufgerufen
{
    pin_time_measurement_t *measurement = get_measurement(pin);

    if (measurement == NULL)
        return; // Invalid pin, do nothing

    if (self_driven_signal || measurement->timestamp == 0)
        return; // Ignore if this is a self-driven signal or no previous measurement

    delay_us(DEBOUNCE_DELAY_US); // Debounce delay

    // Calculate the duration of the signal
    uint64_t current_ticks = get_timer_ticks();
    uint64_t signal_duration = current_ticks - measurement->timestamp;

    clear_time_measurement(pin); // Clear the measurement after processing

    // Convert ticks to milliseconds
    signal_duration = (signal_duration / 1000); // Convert to milliseconds

    if (signal_duration < MINIMUM_SIGNAL_DURATION_MS)
    {
        printf("Signal too short: %lu ms\n", signal_duration);
        return; // Ignore too short signals
    }

    if (signal_duration >= SYN_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
        signal_duration <= SYN_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        printf("Received SYN signal\n");

        PinEvent event = {TEST_PIN, (uint32_t)signal_duration, false, false, true};
        push(pin_events, &event);
    }
    else if (signal_duration >= SYN_ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= SYN_ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
    {
        printf("Received SYN-ACK signal\n");
        PinEvent event = {TEST_PIN, (uint32_t)signal_duration, false, true, false};
        push(pin_events, &event);
    }
    else if (signal_duration >= ACK_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
             signal_duration <= SIGNAL_DURATION_TIME_INACURACY + ACK_SIGNAL_DURATION_MS)
    {
        printf("Received ACK signal\n");

        PinEvent event = {TEST_PIN, (uint32_t)signal_duration, true, false, false};
        push(pin_events, &event);
    }
    else
    {
        printf("Signal duration out of expected range: %lu ms\n", signal_duration);
    }
}
void falling_handler(uint32_t pin)
{
    if (self_driven_signal)
        return; // Ignore if this is a self-driven signal

    delay_us(DEBOUNCE_DELAY_US); // Debounce delay

    if (gpio_read(pin) == 0) // Check if pin is still low after debounce
    {
        log_pin_state(pin, get_timer_ticks(), false);
    }
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

void send_signal(uint32_t pin, uint32_t duration_ms)
{
    // This function is used to send a self-driven signal
    self_driven_signal = true; // Set the flag to indicate self-driven signal
    gpio_drive_low(pin);
    delay_ms(duration_ms);
    release_gpio_open_drain(pin);
    uint64_t timeout = get_timer_ticks() + 5000; // 5ms timeout
    while (!gpio_read(pin) && get_timer_ticks() < timeout)
        ;
    self_driven_signal = false; // Set the flag to indicate self-driven signal
}

void print_all_pins_state(void)
{
    for (uint32_t pin = 0; pin < 32; ++pin)
    {
        int value = gpio_read(pin);
        printf("%d ", value);
    }
#if defined(NRF52840_XXAA)
    for (uint32_t pin = 0; pin < 16; ++pin)
    {
        int value = gpio_read(pin + 32);
        printf("%d ", value);
    }
#elif defined(__MSP430FR5994__)
    for (uint32_t pin = 0; pin < 8; ++pin)
    {
        int value = gpio_read(pin + 8);
        printf("%d ", value);
    }
#endif
    printf("\n");
}

void print_low_level_pins(void)
{
    printf("Pins with level 0: ");
    for (uint32_t pin = 0; pin < 32; ++pin)
    {
        if (gpio_read(pin) == 0)
            printf("%u ", pin);
    }
#if defined(NRF52840_XXAA)
    for (uint32_t pin = 0; pin < 16; ++pin)
    {
        if (gpio_read(pin + 32) == 0)
            printf("%u ", pin + 32);
    }
#elif defined(__MSP430FR5994__)
    for (uint32_t pin = 0; pin < 8; ++pin)
    {
        if (gpio_read(pin + 8) == 0)
            printf("%u ", pin + 8);
    }
#endif
    printf("\n");
}

void set_selected_pin(uint32_t pin)
{
    selected_pin = pin;
    selected_pin_data = &pin_data[pin]; // Update the pointer to the selected pin data

    printf("Selected pin set to: %lu\n", selected_pin);
}

int main(void)
{

    pin_events = createStack(NUMBER_OF_GPIO_PINS, sizeof(PinEvent)); // Initialize the pin events stack

    io_init();
    printf("Running on %s\n", get_chip_family_name());
    printf("Chip UID: %s\n", get_unique_id_str());

    printf("Initializing GPIO pins...\n");
    print_all_pins_state();
    printf("Setting up GPIO interrupt handlers...\n");

    gpio_listen_interrupt_on_all_pins(
        0,              // No blacklist
        rising_handler, // Rising edge handler
        falling_handler // Falling edge handler
    );
    print_all_pins_state();

    uint64_t get_initial_state = get_initial_pin_state();

    black_list_mask = get_initial_state; // Set the initial blacklist mask
    printf("Initial pin state: 0x%016llx\n", get_initial_state);


    printf("GPIO interrupt handlers set up.\n");
    print_low_level_pins();

    gpio_output_init(ABSOLUTE_PIN_GREEN);
    gpio_output_init(ABSOLUTE_PIN_RED);

    // led_test_routine();

    while (1)
    {
        switch (state)
        {
        case INIT:
        {
            // Release the test pin (open-drain)

            printf("INIT_MODE\n");

            // Set Open-Drain mode for the test pin

            // Wait for a random delay before initiating
            uint32_t initial_delay = random32() % (INITIAL_DELAY_MAX_MS + 1);

            set_selected_pin(select_random_non_blacklisted_and_not_successful_pin(pin_data, NUMBER_OF_GPIO_PINS));

            printf("Selected pin: %lu\n", selected_pin);
            printf("Initial delay: %lu ms\n", initial_delay);

            reset_timer();
            start_timer();
            uint64_t start_ticks = get_timer_ticks();

            Stack *active_pins_stack = createStack(NUMBER_OF_GPIO_PINS, sizeof(uint32_t)); // Create a stack to hold active pins
            bool active_signal_detected = false;

            // Wait during initial delay — if a signal is detected, we become responder
            while (timer_diff_ms(start_ticks, get_timer_ticks()) < initial_delay)
            {
                print_low_level_pins();
                freeStack(active_pins_stack); // Free the stack to avoid memory leaks
                push_active_pins_to_stack(active_pins_stack, 0);
                if (!isStackEmpty(active_pins_stack))
                {
                    uint32_t pin;
                    pop(active_pins_stack, &pin);

                    set_selected_pin(pin); // Set the selected pin to the active pin
                    printf("Active pin detected: %lu\n", selected_pin);
                    active_signal_detected = true;
                }
            }

            stop_timer();

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

            reset_timer();
            start_timer();
            bool signal_received = false;

            uint64_t ticks_at_starting_point = get_timer_ticks();
            Stack *active_pins_stack = createStack(NUMBER_OF_GPIO_PINS, sizeof(uint32_t)); // Create a stack to hold active pins

            bool another_active_pin_detected = false;

            while ((timer_diff_ms(ticks_at_starting_point, get_timer_ticks()) < TIMEOUT_RESPONDER_MODE_MS) && !signal_received)
            {
                printf("Waiting for SYN signal on pin %lu...\n", selected_pin);
                // Check for active pins except the selected one
                push_active_pins_except_to_stack(active_pins_stack, selected_pin);
                if (!isStackEmpty(active_pins_stack))
                {
                    uint32_t pin;
                    pop(active_pins_stack, &pin);

                    printf("Another active pin detected: %lu\n", pin);
                    another_active_pin_detected = true;
                    set_selected_pin(pin); // Set the selected pin to the active pin
                }

                // Check for SYN event in the pin_events
                if (!isStackEmpty(pin_events))
                {
                    PinEvent *event = NULL;
                    pop(pin_events, &event);
                    if (event != NULL && event->is_syn)
                    {
                        signal_received = true;
                        printf("SYN event received on pin %lu\n", event->pin);
                        set_syn(&pin_data[selected_pin], true);
                        set_role(&pin_data[selected_pin], RESPONDER_ROLE);

                        break;
                    }
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
                printf("No SYN signal received, go back to INIT_MODE.\n");
                state = INIT;
            }

            break;
        }

        case INITIATOR:
        {
            i_was_the_initiator = true;

            reset_timer();
            start_timer();
            uint64_t ticks_at_starting_point = get_timer_ticks();
            uint64_t sendtimeout = ticks_at_starting_point + SYN_SIGNAL_DURATION_MS;

            // Send SYN signal as initiator
            printf("INITIATOR_MODE: Sending SYN signal\n");
            send_signal(selected_pin, SYN_SIGNAL_DURATION_MS);

            while (get_timer_ticks() < sendtimeout)
            {
            }
            stop_timer();

            uint32_t syn_ack_timeout = TIMEOUT_SYN_ACK_MS;
            bool timeout_inceased = false;
            bool signal_received = false;

            while ((timer_diff_ms(ticks_at_starting_point, get_timer_ticks()) < syn_ack_timeout) && !signal_received)
            {
                if ((gpio_read(selected_pin) == 0) && !timeout_inceased)
                {
                    // If the line is low, increase the timeout
                    syn_ack_timeout += SYN_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY; // Increase timeout by SYN signal duration
                    // Log the increase as this should be done only once
                    timeout_inceased = true;
                }
                PinEvent *event = NULL;
                peek(pin_events, &event);

                if (event != NULL && event->is_syn_ack && event->pin == selected_pin)
                {
                    signal_received = true;
                    pop(pin_events, &event);
                    printf("Received SYN-ACK signal\n");
                    set_syn_ack(&pin_data_tmp, true);
                    // Acknowledge the SYN-ACK signal
                    send_signal(selected_pin, ACK_SIGNAL_DURATION_MS);
                    set_ack(&pin_data_tmp, true);
                    break;
                }

                if (signal_received)
                {
                    printf("Received SYN-ACK signal\n");
                    // Acknowledge the SYN-ACK signal
                    send_signal(selected_pin, ACK_SIGNAL_DURATION_MS);
                    state = SUCCESS;
                }
            }

            break;
        }

        case RESPONDER:
        {
            i_was_the_responder = true;
            printf("RESPONDER_MODE\n");

            // Acknowledge the initial signal
            send_signal(selected_pin, SYN_ACK_SIGNAL_DURATION_MS);

            bool signal_received = false;
            reset_timer();
            start_timer();
            uint64_t start_ticks = get_timer_ticks();

            uint32_t ack_timeout = TIMEOUT_ACK_MS;
            bool increase_timeout = false;

            // Wait for final signal from initiator
            while ((timer_diff_ms(start_ticks, get_timer_ticks()) < ack_timeout) && !signal_received)
            {
                if ((gpio_read(selected_pin) == 0) && !increase_timeout)
                {
                    // If the line is low, increase the timeout
                    ack_timeout += ACK_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY; // Increase timeout by ACK signal duration
                    printf("Increasing ACK timeout to %lu ms\n", ack_timeout);
                    increase_timeout = true;
                }

                PinEvent *event = NULL;
                peek(pin_events, &event);

                if (event != NULL && event->is_ack && event->pin == selected_pin)
                {
                    signal_received = true;
                    pop(pin_events, &event);
                    printf("Received ACK signal\n");
                }
            }
            stop_timer();
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
            needded_tries++;
            printf("Handshake failed, tries needed: %lu\n", needded_tries);

            red_led_on = true;
            gpio_drive_high(ABSOLUTE_PIN_RED);

            if (pin_data_tmp.num_tries >= MAXIMUM_NUMBER_OF_TRIES)
            {
                printf("Maximum number of tries reached for pin %lu, blacklisting it.\n", selected_pin);

                // Blacklist the pin
                pin_data_tmp.pin = selected_pin;
                pin_data_tmp.num_tries = pin_data_tmp.num_tries + 1; // Increment the number of tries
                pin_data_tmp.error_reason = 1;                       // Set error reason to 1 for failed handshake

                // Set the blacklisted status

                set_blacklisted(&pin_data_tmp, true);
                set_blacklisted_in_mask(&black_list_mask, selected_pin);

                // Reset state
                state = INIT;
                i_was_the_initiator = false;
                i_was_the_responder = false;
                break;
            }
        }

        case SUCCESS:
        {

            printf("SUCCESS_MODE\n");
            needded_tries++;
            printf("Handshake successful, tries needed: %lu\n", needded_tries);

            if (i_was_the_initiator)
            {
                printf("I was the initiator.\n");
            }
            if (i_was_the_responder)
            {
                printf("I was the responder.\n");
            }

            needded_tries = 0;
            gpio_drive_high(ABSOLUTE_PIN_GREEN);
            green_led_on = true;

            delay_ms(1000);

            // Reset state
            state = INIT;
            i_was_the_initiator = false;
            i_was_the_responder = false;
            break;
        }
        }
    }

    return 0;
}