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

#define DEBUG_PIN1 ABS_PIN(3, 4) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 ABS_PIN(3, 5) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 ABS_PIN(8, 1) // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 ABS_PIN(8, 2) // Additional debug pin, can be changed as needed

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
#include "manchester.h"
#include "random_utils.h"
#include "handshake.h"

// Handshake timing constants
#define INITIAL_DELAY_MAX_MS 10000
#define MAXIMUM_NUMBER_OF_FALSE_RESPONSES 2 // Maximum number of false responses before blacklisting a pin
#define MAXIMUM_NUMBER_OF_TRIES 5           // Maximum number of tries for a pin before giving up

#define INITIATOR_ROLE 0
#define RESPONDER_ROLE 1

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


// Inspired from Hacker’s Delight by Henry S. Warren, Jr.

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

    set_standart_blacklist_pins(&initial_state_mask);
    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (initial_state_mask & (1ULL << pin))
            continue;      // Skip blacklisted pins (bit = 1)
        gpio_od_init(pin); // Initialize non-blacklisted pins (bit = 0) with pull-up resistors
    }


    get_initial_pin_state(pin_data, &initial_state_mask);

    while(true)
    {}

    print_active_pins_from_mask(initial_state_mask);

    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (!(initial_state_mask & (1ULL << pin)))
        {
            debug_print_pindata(&pin_data[pin]); // Print pin data for non-blacklisted pins
        }
    }

    perform_handshake(pin_data, initial_state_mask);

    while (true)
    {
    }
}