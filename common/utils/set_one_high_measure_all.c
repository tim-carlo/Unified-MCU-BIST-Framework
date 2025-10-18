#include "set_one_high_measure_all.h"

#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

#define NUMBER_OF_SAMPLES 5
#define SETTLE_TIME_US 100

static uint64_t read_all_pins(uint64_t blacklist_mask)
{
    uint64_t result = ~0ULL;
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        uint64_t sample = 0;
        BitmapIterator sample_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t pin;
        while (bitmap_iterator_next(&sample_it, &pin))
        {
            if (gpio_read(pin))
            {
                sample |= (1ULL << pin);
            }
        }
        result &= sample;
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }
    return result;
}

static bool sample_own_pin(uint8_t pin, bool expected_state)
{
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        bool pin_state = gpio_read(pin);
        if (pin_state != expected_state)
        {
            return false;
        }
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }
    return true;
}

static bool sample_pin_state(uint8_t pin, bool expected_state)
{
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        bool pin_state = gpio_read(pin);
        if (pin_state != expected_state)
        {
            return false;
        }
        delay_us(SETTLE_TIME_US);
    }
    return true;
}

/**
 * @brief Reset all pins to high impedance input with no pull resistors
 *
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = reset this pin
 */
static void reset_all_pins(uint64_t blacklist_mask)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
}

static void sample_pin_changes(uint64_t blacklist_mask, uint64_t initial_state, uint8_t test_pin, uint8_t *counter)
{
    for (int sample = 0; sample < NUMBER_OF_SAMPLES; sample++)
    {
        // Invertiere die Blacklist-Maske
        BitmapIterator read_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t read_pin;

        while (bitmap_iterator_next(&read_it, &read_pin))
        {

            uint8_t current = gpio_read(read_pin);
            uint8_t initial = (initial_state >> read_pin) & 1;

            if (current != initial)
                counter[read_pin]++;
        }
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }
}

/**
 * @brief Phase 0: Pull-up configuration, drive each pin low and measure others
 * @param blacklist_mask 64-bit mask of pins (0 = test this pin, 1 = skip)
 * @param pindata Array to store results
 */
static void phase_0_pulldown_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    LOG("Phase 0: Pull-down, drive low\n");

    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it low and measuring others
    it = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_drive_high(DEBUG_PIN1);
        // Pin als Input mit Pull-Down konfigurieren
        gpio_input_init(pin, GPIO_PULL_DOWN);

        // measure own pin to verify it is actually low
        if (!sample_own_pin(pin, false))
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
        }

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t print_pin;

        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
        }
        gpio_drive_low(DEBUG_PIN1);

        // reset counter
        memset(counter, 0, sizeof(counter));
        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }

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
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    LOG("Phase 1: Pull-up, drive high\n");

    // Configure all pins with no pull initially
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    uint64_t initial_state = read_all_pins(blacklist_mask);
    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it high and measuring others
    it = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_UP);
        gpio_drive_high(DEBUG_PIN1);
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize

        // Validate that pin goes high when pulled up
        if (!sample_pin_state(pin, true))
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
        }

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t print_pin;

        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
        }

        gpio_drive_low(DEBUG_PIN1);
        // reset counter
        memset(counter, 0, sizeof(counter));
        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}
/**
 * @brief Phase 2: No pull resistors, drive each pin low and measure others
 *
 * @param blacklist_mask 64-bit mask of pins to test
 * @param pindata Array to store results
 */
static void phase_2_no_pull_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    // Invertiere die Blacklist-Maske
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    LOG("Phase 2: No pull, drive low\n");

    // Configure all pins with no pull
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it low and measuring others
    it = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        // Configure test pin as output and drive low
        gpio_output_init(pin);
        gpio_drive_low(pin);

        // Validate that pin goes low when driven low
        if (!sample_pin_state(pin, false))
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
        }

        gpio_drive_high(DEBUG_PIN1);

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t print_pin;

        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
        }
        gpio_drive_low(DEBUG_PIN1);
        // reset counter
        memset(counter, 0, sizeof(counter));

        // Set pin back to no-pull input
        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }

    // Reset all pins to clean state
    reset_all_pins(blacklist_mask);
}

/**
 * @brief Phase 3: No pull resistors, drive each pin high and measure others
 *
 * @param blacklist_mask 64-bit mask of pins to test
 * @param pindata Array to store results
 */
static void phase_3_no_pull_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    LOG("Phase 3: No pull, drive high\n");

    // Configure all pins with no pull
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }
    uint64_t initial_state = read_all_pins(blacklist_mask);

    // counter for pin changes
    uint8_t counter[64] = {0};

    // Test every pin by driving it high and measuring others
    it = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it, &pin))
    {
        gpio_output_init(pin);
        gpio_drive_high(pin);

        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize

        // Validate that pin goes high when driven high
        if (!sample_pin_state(pin, true))
        {
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
        }

        gpio_drive_high(DEBUG_PIN1);

        // Take samples and count changes
        sample_pin_changes(blacklist_mask, initial_state, pin, counter);

        // Print results for this pin
        BitmapIterator print_it = bitmap_iterator_create(~blacklist_mask);
        uint8_t print_pin;

        while (bitmap_iterator_next(&print_it, &print_pin))
        {
            if (print_pin != pin && counter[print_pin] > 0)
            {
                add_pin_connection(pindata, pin, print_pin, MY_DEVICE_ID_INDEX);
                add_pin_event(pindata, pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
            }
        }
        gpio_drive_low(DEBUG_PIN1);
        // reset counter
        memset(counter, 0, sizeof(counter));
        // Set pin back to no-pull input
        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US); // Allow time for signals to stabilize
    }

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

    LOG("Starting set-one-high-measure-all with blacklist mask: 0x%016llX\n", blacklist_mask);
    gpio_output_init(DEBUG_PIN1);
    // Run all 4 phases
    phase_0_pulldown_drive_low(blacklist_mask, pindata);
    phase_1_pullup_drive_high(blacklist_mask, pindata);
    phase_2_no_pull_drive_low(blacklist_mask, pindata);
    phase_3_no_pull_drive_high(blacklist_mask, pindata);
    gpio_reset(DEBUG_PIN1);

    LOG("Completed set-one-high-measure-all\n");
}