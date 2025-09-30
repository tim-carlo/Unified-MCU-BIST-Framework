#include "set_one_high_measure_all.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"

#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define NUMBER_OF_SAMPLES 5
#define DEBUG_PIN 13

static uint64_t read_all_pins(uint64_t blacklist_mask)
{
    uint64_t result = ~0ULL;
    uint64_t test_mask = ~blacklist_mask;
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        uint64_t sample = 0;
        BitmapIterator sample_it = bitmap_iterator_create(test_mask);
        uint8_t pin;
        while (bitmap_iterator_next(&sample_it, &pin))
        {
            if (gpio_read(pin))
            {
                sample |= (1ULL << pin);
            }
        }
        result &= sample;
        delay_ms(1);
    }
    return result;
}

static bool sample_own_pin(bool expected_state)
{
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        bool pin_state = gpio_read(DEBUG_PIN);
        if (pin_state != expected_state)
        {
            return false;
        }
        delay_ms(1);
    }
    return true;
}

/**
 * @brief Reset all pins to high impedance input with no pull resistors
 *
 * @param blacklist_mask Mask where 0 = reset this pin, 1 = skip this pin
 */
static void reset_all_pins(uint64_t blacklist_mask)
{
    uint64_t test_mask = ~blacklist_mask;
    BitmapIterator it = bitmap_iterator_create(test_mask);
    uint8_t pin;

    printf("Resetting pins to high impedance: ");
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
        printf("%d ", pin);
    }
    printf("\n");
}

static void sample_pin_changes(uint64_t blacklist_mask, uint64_t initial_state, uint8_t test_pin, uint8_t *counter)
{
    uint64_t test_mask = ~blacklist_mask;
    for (int sample = 0; sample < NUMBER_OF_SAMPLES; sample++)
    {
        BitmapIterator read_it = bitmap_iterator_create(test_mask);
        uint8_t read_pin;

        while (bitmap_iterator_next(&read_it, &read_pin))
        {

            uint8_t current = gpio_read(read_pin);
            uint8_t initial = (initial_state >> read_pin) & 1;

            if (current != initial)
                counter[read_pin]++;
        }

        delay_ms(1);
    }
}

/**
 * @brief Phase 0: Pull-up configuration, drive each pin low and measure others
 * @param blacklist_mask 64-bit mask of pins (0 = test this pin, 1 = skip)
 * @param pindata Array to store results
 */
static void phase_0_pulldown_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    printf("Sample Pins with Pull-Up: ");
    uint64_t test_mask = ~blacklist_mask; // Invert: 0=test becomes 1=test
    BitmapIterator it = bitmap_iterator_create(test_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_NO_PULL);
    }
    printf("\n");
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it low and measuring others
    it = bitmap_iterator_create(test_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        printf("Testing pin %d (driving high):\n", pin);

        gpio_drive_high(DEBUG_PIN);
        // Pin als Input mit Pull-Down konfigurieren
        gpio_input_init(pin, GPIO_PULL_DOWN);
        

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(test_mask);
        uint8_t print_pin;

        bool found_own_pin = false;
        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                printf("    Pin %d: %d times\n", print_pin, counter[print_pin]);
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
            if (print_pin == pin && counter[print_pin] > 0)
            {
                found_own_pin = true;
            }
        }
        if (!found_own_pin)
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
            printf("    Warning: Pin %d did not go low when pulled down!\n", pin);
        }
        delay_ms(10);
        gpio_drive_low(DEBUG_PIN);

        // reset counter
        memset(counter, 0, sizeof(counter));
        gpio_input_init(pin, GPIO_NO_PULL);
    }

    printf("Phase 0 completed.\n");

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}

/**
 * @brief Phase 1: Pull-down configuration, drive each pin high and measure others
 *
 * @param blacklist_mask 64-bit mask of pins to test
 * @param pindata Array to store results
 */
static void phase_1_pullup_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    printf("Sample Pins with Pull-Down: ");
    uint64_t test_mask = ~blacklist_mask; // Invert: 0=test becomes 1=test
    BitmapIterator it = bitmap_iterator_create(test_mask);
    uint8_t pin;

    // Configure all pins with pull-down
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_NO_PULL);
    }
    printf("\n");
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it high and measuring others
    it = bitmap_iterator_create(test_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        printf("Testing pin %d (driving high):\n", pin);

        gpio_input_init(pin, GPIO_PULL_UP);
        gpio_drive_high(DEBUG_PIN);

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(test_mask);
        uint8_t print_pin;

        bool found_own_pin = false;
        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                printf("    Pin %d: %d times\n", print_pin, counter[print_pin]);
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
            if (print_pin == pin && counter[print_pin] > 0)
            {
                found_own_pin = true;
            }
        }
        if (!found_own_pin)
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
            printf("    Warning: Pin %d did not go high when pulled up!\n", pin);
        }

        delay_ms(20);

        gpio_drive_low(DEBUG_PIN);
        // reset counter
        memset(counter, 0, sizeof(counter));
        gpio_input_init(pin, GPIO_NO_PULL);
    }

    printf("Phase 1 completed.\n");

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}

/**
 * @brief Phase 2: No pull resistors, drive each pin high and measure others
 *
 * @param blacklist_mask 64-bit mask of pins to test
 * @param pindata Array to store results
 */
static void phase_2_no_pull_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    printf("Sample Pins with active drive high: ");
    uint64_t test_mask = ~blacklist_mask; // Invert: 0=test becomes 1=test
    BitmapIterator it = bitmap_iterator_create(test_mask);
    uint8_t pin;

    // Configure all pins with no pull
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    printf("\n");
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it high and measuring others
    it = bitmap_iterator_create(test_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        printf("Testing pin %d (driving high):\n", pin);

        // Configure test pin as output and drive high
        gpio_output_init(pin);
        gpio_drive_high(pin);

        gpio_drive_high(DEBUG_PIN);

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(test_mask);
        uint8_t print_pin;

        bool found_own_pin = false;
        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                printf("    Pin %d: %d times\n", print_pin, counter[print_pin]);
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
            if (print_pin == pin && counter[print_pin] > 0)
            {
                found_own_pin = true;
            }
        }
        if (!found_own_pin)
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
            printf("    Warning: Pin %d did not go high when driven high!\n", pin);
        }

        delay_ms(30);
        gpio_drive_low(DEBUG_PIN);
        // reset counter
        memset(counter, 0, sizeof(counter));
        // Set pin back to no-pull input
        gpio_input_init(pin, GPIO_PULL_NONE);
    }

    printf("Phase 2 completed.\n");

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}

/**
 * @brief Phase 3: No pull resistors, drive each pin low and measure others
 *
 * @param blacklist_mask 64-bit mask of pins to test
 * @param pindata Array to store results
 */
static void phase_3_no_pull_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    printf("Sample Pins with active drive low: ");
    uint64_t test_mask = ~blacklist_mask; // Invert: 0=test becomes 1=test
    BitmapIterator it = bitmap_iterator_create(test_mask);
    uint8_t pin;

    // Configure all pins with no pull
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    printf("\n");
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it low and measuring others
    it = bitmap_iterator_create(test_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        printf("Testing pin %d (driving low):\n", pin);

        // Configure test pin as output and drive low
        gpio_output_init(pin);
        gpio_drive_low(pin);

        gpio_drive_high(DEBUG_PIN);

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(test_mask);
        uint8_t print_pin;

        bool found_own_pin = false;
        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                printf("    Pin %d: %d times\n", print_pin, counter[print_pin]);
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
            if (print_pin == pin && counter[print_pin] > 0)
            {
                found_own_pin = true;
            }
        }
        if (!found_own_pin)
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
            printf("    Warning: Pin %d did not go low when driven low!\n", pin);
        }

        delay_ms(40);
        gpio_drive_low(DEBUG_PIN);
        // reset counter
        memset(counter, 0, sizeof(counter));
        // Set pin back to no-pull input
        gpio_input_init(pin, GPIO_PULL_NONE);
    }

    printf("Phase 3 completed.\n");

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}

/**
 * @brief Main function to run all 4 phases of pin connectivity testing
 *
 * @param blacklist_mask 64-bit mask where 0 = test this pin, 1 = skip this pin
 * @param pindata Array to store discovered connections and events
 * @param pindata_size Size of the pindata array
 */
void run_set_one_high_measure_all(uint64_t blacklist_mask, PinData *pindata, uint8_t pindata_size)
{

    gpio_output_init(13);
    // Run all 4 phases
    phase_0_pulldown_drive_low(blacklist_mask, pindata);
    phase_1_pullup_drive_high(blacklist_mask, pindata);
    phase_2_no_pull_drive_high(blacklist_mask, pindata);
    phase_3_no_pull_drive_low(blacklist_mask, pindata);

    printf("\n=== Pin connectivity test completed ===\n");

    // Optional: Print summary of discovered connections
    printf("\n=== Connection Summary ===\n");
    uint64_t test_mask = ~blacklist_mask; // Invert: 0=test becomes 1=test
    BitmapIterator summary_it = bitmap_iterator_create(test_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&summary_it, &pin))
    {
        if (pindata[pin].connection_index > 0)
        {
            printf("Pin %d has %d connections:\n", pin, pindata[pin].connection_index);
            for (uint8_t i = 0; i < pindata[pin].connection_index; i++)
            {
                printf("  -> Pin %d (device %d)\n",
                       pindata[pin].connections[i].other_pin,
                       pindata[pin].connections[i].device_index);
            }
        }
    }
}