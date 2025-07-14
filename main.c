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

#endif

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 1000

#define SIGNAL_DURATION_MS 100
#define START_SIGNAL_DURATION_MS 50
#define ACKNOWLEGEMENT_SIGNAL_DURATION_MS 100
#define SIGNAL_DURATION_TIME_INACURACY 5

#define DEBOUNCE_DELAY_US 20
#define RESPONSE_TIMEOUT_MS 50

// State tracking
uint32_t needded_tries = 0;
bool i_was_the_initiator = false;
bool i_was_the_responder = false;
bool red_led_on = false;
bool green_led_on = false;

// State machine enum
typedef enum
{
    INIT_MODE,
    INITIATOR_MODE,
    RESPONDER_MODE,
    ERROR_MODE,
    SUCCESS_MODE
} State;

State state = INIT_MODE;

// Flags controlled via interrupts
volatile bool signal_active = false;
volatile bool signal_complete = false;
volatile bool pin_rised = false;

volatile bool received_init_signal = false; // Flag to indicate if we received the initial signal
volatile bool received_ack_signal = false;  // Flag to indicate if we received the ACK signal

volatile uint32_t ticks_at_starting_point_signal = 0; // Timer ticks at the start of the handshake
volatile bool self_driven_signal = false;             // Flag to indicate if the signal is self-driven

// Interrupt handler for rising/falling edges on test pin

void rising_handler(uint32_t gpio, uint32_t events) // Wird bei STEIGENDER Flanke (HIGH) aufgerufen
{

    if (self_driven_signal || ticks_at_starting_point_signal == 0)
        return; // Ignore if this is a self-driven signal

    delay_us(DEBOUNCE_DELAY_US);             // Debounce delay
    if (gpio_read(TEST_PORT, TEST_PIN) == 1) // Check if pin is still high after debounce
    {
        signal_active = false;
        signal_complete = false;
        pin_rised = true;
        if (signal_active)
        {
            printf("Signal complete on TEST_PIN (full pulse detected)\n");
            signal_complete = true;
            signal_active = false;

            // Calculate the duration of the signal
            uint64_t current_ticks = get_timer_ticks();
            uint64_t signal_duration = current_ticks - ticks_at_starting_point_signal;
            // Convert ticks to milliseconds
            signal_duration = (current_ticks / 1000); // Convert to milliseconds
            printf("Signal duration: %lu ms\n", signal_duration);

            // Check if the signal is acknowledgement signal or initial signal
            if (signal_duration >= START_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
                signal_duration <= START_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
            {
                printf("Received initial signal (START_SIGNAL)\n");
                received_init_signal = true;
            }
            else if (signal_duration >= ACKNOWLEGEMENT_SIGNAL_DURATION_MS - SIGNAL_DURATION_TIME_INACURACY &&
                     signal_duration <= ACKNOWLEGEMENT_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY)
            {
                printf("Received ACK signal\n");
                received_ack_signal = true;
            }
        }
    }
}
void falling_handler(uint32_t gpio, uint32_t events)
{
    if (self_driven_signal)
        return;                              // Ignore if this is a self-driven signal
    delay_us(DEBOUNCE_DELAY_US);             // Debounce delay
    if (gpio_read(TEST_PORT, TEST_PIN) == 0) // Check if pin is still low after debounce
    {
        signal_active = true;
        signal_complete = false;
        printf("Confirmed LOW signal\n");
        ticks_at_starting_point_signal = get_timer_ticks(); // Record the time when the signal went low
    }
}

void turn_off_leds(void)
{
    if (red_led_on)
    {
        gpio_drive_low(LED_RED_PORT, LED_RED_PIN);
        red_led_on = false;
    }
    if (green_led_on)
    {
        gpio_drive_low(LED_GREEN_PORT, LED_GREEN_PIN);
        green_led_on = false;
    }
}

void led_test_routine(void)
{
    printf("Starting LED test...\n");

    // Blink red LED 5 times
    for (uint32_t i = 0; i < 5; i++)
    {
        gpio_drive_high(LED_RED_PORT, LED_RED_PIN);
        delay_ms(200);
        gpio_drive_low(LED_RED_PORT, LED_RED_PIN);
        delay_ms(200);
    }

    // Blink green LED 5 times
    for (uint32_t i = 0; i < 5; i++)
    {
        gpio_drive_high(LED_GREEN_PORT, LED_GREEN_PIN);
        delay_ms(200);
        gpio_drive_low(LED_GREEN_PORT, LED_GREEN_PIN);
        delay_ms(200);
    }

    printf("LED test complete.\n");
}

void send_signal(NRF_GPIO_Type *port, uint8_t pin, uint32_t duration_ms)
{
    // This function is used to send a self-driven signal
    self_driven_signal = true; // Set the flag to indicate self-driven signal
    gpio_drive_low(TEST_PORT, TEST_PIN);
    delay_ms(duration_ms);
    release_gpio_open_drain(TEST_PORT, TEST_PIN);
    uint64_t timeout = get_timer_ticks() + 5000; // 5ms timeout
    while (!gpio_read(port, pin) && get_timer_ticks() < timeout)
        ;
    self_driven_signal = false; // Set the flag to indicate self-driven signal
}

int main(void)
{
    io_init();
    printf("Running on %s\n", get_chip_family_name());
    printf("Chip UID: %s\n", get_unique_id_str());
    printf("Test pin configured on port %d, pin %d\n", TEST_PORT, TEST_PIN);

    gpio_open_drain_with_interrupt(
        TEST_PORT,
        TEST_PIN,
        &rising_handler, // Rising handler
        &falling_handler // Falling handler
    );

    gpio_output_init(LED_RED_PORT, LED_RED_PIN);
    gpio_output_init(LED_GREEN_PORT, LED_GREEN_PIN);

    led_test_routine();

    while (1)
    {
        switch (state)
        {
        case INIT_MODE:
        {
            // Release the test pin (open-drain)

            printf("INIT_MODE\n");
            received_init_signal = false; // Reset the initial signal flag
            received_ack_signal = false;  // Reset the ACK signal flag

            // Wait for a random delay before initiating
            uint32_t initial_delay = random32() % (INITIAL_DELAY_MAX_MS + 1);

            printf("Initial delay: %lu ms\n", initial_delay);

            reset_timer();
            start_timer();
            uint64_t start_ticks = get_timer_ticks();
            bool signal_received = false;

            // Wait during initial delay — if a signal is detected, we become responder
            while (timer_diff_ms(start_ticks, get_timer_ticks()) < initial_delay)
            {
                if (signal_active)
                {
                    signal_received = true;
                    break;
                }
            }
            bool valid_initial_signal = false;

            if (signal_received)
            {
                start_ticks = get_timer_ticks();

                while (timer_diff_ms(start_ticks, get_timer_ticks()) < (START_SIGNAL_DURATION_MS + SIGNAL_DURATION_TIME_INACURACY))
                {
                    if (received_init_signal)
                    {
                        valid_initial_signal = true;
                        break;
                    }
                }
            }
            signal_complete = false;

            stop_timer();

            // Turn off LEDs
            turn_off_leds();

            // Switch to the next mode depending on whether we saw a signal
            if (valid_initial_signal)
            {
                state = RESPONDER_MODE;
            }
            else
            {
                printf("No signal received, becoming initiator.\n");
                // Send signal (pull low)

                send_signal(TEST_PORT, TEST_PIN, START_SIGNAL_DURATION_MS);

                state = INITIATOR_MODE;
            }
            break;
        }

        case INITIATOR_MODE:
        {
            i_was_the_initiator = true;
            bool signal_received = true;
            reset_timer();
            start_timer();
            uint64_t start_ticks = get_timer_ticks();

            // Wait for handshake response
            while (!signal_complete)
            {
                // printf("Waiting for responder's signal...\n");
                // printf("Current time: %lu ms\n", timer_diff_ms(start_ticks, get_timer_ticks()));
                if (timer_diff_ms(start_ticks, get_timer_ticks()) > RESPONSE_TIMEOUT_MS)
                {
                    state = ERROR_MODE;
                    signal_received = false;
                    break;
                }
            }
            signal_complete = false;
            stop_timer();

            // Send confirmation pulse

            if (signal_received)
            {
                // Send ACK signal
                send_signal(TEST_PORT, TEST_PIN, ACKNOWLEGEMENT_SIGNAL_DURATION_MS);
                state = SUCCESS_MODE;
            }
            else
            {
                printf("No response received, switching to ERROR_MODE.\n");
                state = ERROR_MODE;
            }
            break;
        }

        case RESPONDER_MODE:
        {
            i_was_the_responder = true;
            printf("RESPONDER_MODE\n");

            // Acknowledge the initial signal
            send_signal(TEST_PORT, TEST_PIN, ACKNOWLEGEMENT_SIGNAL_DURATION_MS);

            bool signal_received = true;
            reset_timer();
            start_timer();
            uint64_t start_ticks = get_timer_ticks();

            // Wait for final signal from initiator
            while (!signal_complete)
            {
                if (timer_diff_ms(start_ticks, get_timer_ticks()) > RESPONSE_TIMEOUT_MS)
                {
                    signal_received = false;
                    state = ERROR_MODE;
                    break;
                }
            }
            signal_complete = false;
            stop_timer();
            if (signal_received)
            {
                state = SUCCESS_MODE;
            }
            else
            {
                printf("No final signal received, switching to ERROR_MODE.\n");
                state = ERROR_MODE;
            }

            break;
        }

        case ERROR_MODE:
        {
            needded_tries++;
            printf("Handshake failed, tries needed: %lu\n", needded_tries);

            red_led_on = true;
            gpio_drive_high(LED_RED_PORT, LED_RED_PIN);

            // Reset state
            state = INIT_MODE;
            i_was_the_initiator = false;
            i_was_the_responder = false;
            break;
        }

        case SUCCESS_MODE:
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
            gpio_drive_high(LED_GREEN_PORT, LED_GREEN_PIN);
            green_led_on = true;

            delay_ms(1000);

            // Reset state
            state = INIT_MODE;
            i_was_the_initiator = false;
            i_was_the_responder = false;
            break;
        }
        }
    }

    return 0;
}