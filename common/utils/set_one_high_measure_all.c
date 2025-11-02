#include "set_one_high_measure_all.h"
#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)
#define NUMBER_OF_SAMPLES 10
#define SETTLE_TIME_US 1000

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
                sample |= (1ULL << pin);
        }

        result &= sample;
        delay_us(SETTLE_TIME_US);
    }
    return result;
}

static bool sample_pin_state(uint8_t pin, bool expected_state)
{
    for (int i = 0; i < NUMBER_OF_SAMPLES; i++)
    {
        if (gpio_read(pin) != expected_state)
            return false;
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
        gpio_input_init(pin, GPIO_PULL_NONE);
}

/**
 * @brief Compare pin states before and after a test pin is changed
 */
static void log_pin_changes(uint64_t before_state, uint64_t after_state, uint8_t test_pin, PinData *pindata)
{
    for (uint8_t pin = 0; pin < 64; pin++)
    {
        if (pin == test_pin)
            continue;

        bool before = (before_state >> pin) & 1U;
        bool after = (after_state >> pin) & 1U;

        if (before != after)
        {
            LOG("Pin %u changed when pin %u was driven\n", pin, test_pin);
            add_pin_connection(pindata, test_pin, pin, get_own_device_id());
            add_pin_event(pindata, test_pin, PIN_IS_CONNECTED_WITH_INTERNAL_PIN);
        }
    }
}

/**
 * @brief Phase 0: Pull-down configuration, drive each pin low and measure others
 */
static void phase_0_pulldown_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 0: Pull-down, drive low\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint64_t before_state = read_all_pins(blacklist_mask);

        gpio_input_init(pin, GPIO_PULL_DOWN);
        delay_ms(10);

        if (!sample_pin_state(pin, false))
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);

        uint64_t after_state = read_all_pins(blacklist_mask);
        log_pin_changes(before_state, after_state, pin, pindata);

        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US);
    }
}

/**
 * @brief Phase 1: Pull-up configuration, drive each pin high and measure others
 */
static void phase_1_pullup_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 1: Pull-up, drive high\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint64_t before_state = read_all_pins(blacklist_mask);

        gpio_input_init(pin, GPIO_PULL_UP);
        delay_ms(20);

        if (!sample_pin_state(pin, true))
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);

        uint64_t after_state = read_all_pins(blacklist_mask);
        log_pin_changes(before_state, after_state, pin, pindata);

        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US);
    }
}

/**
 * @brief Phase 2: No pull resistors, drive each pin low and measure others
 */
static void phase_2_no_pull_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 2: No pull, drive low\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint64_t before_state = read_all_pins(blacklist_mask);

        gpio_output_init(pin);
        gpio_drive_low(pin);
        delay_ms(30);

        if (!sample_pin_state(pin, false))
            add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);

        uint64_t after_state = read_all_pins(blacklist_mask);
        log_pin_changes(before_state, after_state, pin, pindata);

        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US);
    }
}

/**
 * @brief Phase 3: No pull resistors, drive each pin high and measure others
 */
static void phase_3_no_pull_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 3: No pull, drive high\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint64_t before_state = read_all_pins(blacklist_mask);

        gpio_output_init(pin);
        gpio_drive_high(pin);
        delay_ms(40);

        if (!sample_pin_state(pin, true))
            add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);

        uint64_t after_state = read_all_pins(blacklist_mask);
        log_pin_changes(before_state, after_state, pin, pindata);

        gpio_reset(pin);
        gpio_input_init(pin, GPIO_PULL_NONE);
        delay_us(SETTLE_TIME_US);
    }
}

/**
 * @brief Run all 4 measurement phases
 */
void run_set_one_high_measure_all(uint64_t blacklist_mask, PinData *pindata, uint8_t pindata_size)
{
    reset_all_pins(blacklist_mask);
    phase_0_pulldown_drive_low(blacklist_mask, pindata);
    phase_1_pullup_drive_high(blacklist_mask, pindata);
    phase_2_no_pull_drive_low(blacklist_mask, pindata);
    phase_3_no_pull_drive_high(blacklist_mask, pindata);
    reset_all_pins(blacklist_mask);
}