#include "set_one_high_measure_all.h"
#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)

const uint8_t NUMBER_OF_SAMPLES_FOR_DEBOUNCING = 30;
const uint8_t NUMBER_OF_SAMPLES_FOR_MEASURING = 10;

static const uint32_t SETTLE_TIME_US = 1000;
static const uint32_t TIME_BETWEEN_MEASUREMENTS_US = 10000; // 10ms
static const uint32_t POLLING_DELAY_US = 1000;

static const uint8_t threshold_percent = 70;
static const uint8_t threshold = (NUMBER_OF_SAMPLES_FOR_DEBOUNCING * threshold_percent) / 100;

typedef uint8_t SetOneMeasureALLPhase;
enum
{
    PHASE_0_PULLDOWN_DRIVE_LOW,
    PHASE_1_PULLUP_DRIVE_HIGH,
    PHASE_2_NO_PULL_DRIVE_LOW,
    PHASE_3_NO_PULL_DRIVE_HIGH
};

// Phase 0 time 10ms
// Phase 1 time 20ms
// Phase 2 time 30ms
// Phase 3 time 40ms

static uint64_t read_all_pins(const uint64_t blacklist_mask)
{
    const uint64_t active_mask = ~blacklist_mask;
    uint8_t high_count[64] = {0};

    for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_DEBOUNCING; i++)
    {
        BitmapIterator it = bitmap_iterator_create(active_mask);
        uint8_t pin;

        while (bitmap_iterator_next(&it, &pin))
        {
            if (gpio_read(pin))
                high_count[pin]++;
        }

        delay_us(POLLING_DELAY_US);
    }

    // Majority decision per pin
    uint64_t result = 0;

    BitmapIterator final_it = bitmap_iterator_create(active_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&final_it, &pin))
    {
        if (high_count[pin] >= threshold)
            result |= (1ULL << pin);
    }

    return result;
}

/* static uint64_t read_all_pins(const uint64_t blacklist_mask)
{
    const uint64_t active_mask = ~blacklist_mask;
    uint64_t result = 0;

    BitmapIterator it = bitmap_iterator_create(active_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        if (gpio_read(pin))
            result |= (1ULL << pin);
    }

    return result;
}
 */
static void flush_pin_states(const uint64_t blacklist_mask, gpio_pull_t pull_type)
{
    BitmapIterator it_pull = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it_pull, &pin))
    {
       // gpio_input_init(pin, pull_type);
    }
    pin = 0;

    // let the conductance discharge / charge
   // delay_us(SETTLE_TIME_US);

    BitmapIterator it_reset = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it_reset, &pin))
    {
        gpio_reset(pin);
    }
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
static void log_pin_changes(SetOneMeasureALLPhase phase,
                            PinData *pindata,
                            uint64_t blacklist_mask,
                            uint64_t before_state,
                            uint64_t after_state,
                            uint8_t test_pin)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        bool before = (before_state >> pin) & 1U;
        bool after = (after_state >> pin) & 1U;

        if (pin == test_pin)
        {
            // only store events to safe connection storage.
            switch (phase)
            {
            case PHASE_0_PULLDOWN_DRIVE_LOW:
                if (after == 1)
                {
                    add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
                    printf("DEBUG: Pin %u did not go low when pulled down\n", pin);
                }
                break;
            case PHASE_1_PULLUP_DRIVE_HIGH:
                if (after == 0)
                {
                    add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
                    printf("DEBUG: Pin %u did not go high when pulled up\n", pin);
                }
                break;
            case PHASE_2_NO_PULL_DRIVE_LOW:
                if (after == 1)
                {
                    add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
                    printf("DEBUG: Pin %u did not go low when driven low\n", pin);
                }

                break;
            case PHASE_3_NO_PULL_DRIVE_HIGH:
                if (after == 0)
                {
                    add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
                    printf("DEBUG: Pin %u did not go high when driven high\n", pin);
                }
                break;
            default:
                break;
            }
        }
        else
        {
            if (before != after)
            {
                LOG("Pin %u changed when pin %u was changed\n", pin, test_pin);
                add_pin_connection(CONNECTION_TYPE_INTERNAL, pindata, test_pin, pin, (uint8_t)phase);
            }
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
        uint64_t before_combined = ~0ULL;
        uint64_t after_combined = ~0ULL;

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            // Read own pin state:
            bool current_pin_state = gpio_read(pin);
            uint64_t before = read_all_pins(blacklist_mask);

            gpio_input_init(pin, GPIO_PULL_DOWN);
            delay_ms(10);

            uint64_t after = read_all_pins(blacklist_mask);
            /* print after in binary */
            char after_bin[65];
            for (int b = 0; b < 64; ++b)
                after_bin[b] = ((after >> (63 - b)) & 1ULL) ? '1' : '0';
            after_bin[64] = '\0';
            // printf("DEBUG: pin %u after: 0b%s\n", pin, after_bin);
            current_pin_state = gpio_read(pin);

            before_combined &= before;
            after_combined &= after;

            gpio_reset(pin);
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);
            delay_us(TIME_BETWEEN_MEASUREMENTS_US);
        }
        log_pin_changes(PHASE_0_PULLDOWN_DRIVE_LOW, pindata, blacklist_mask, before_combined, after_combined, pin);
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
        uint64_t before_combined = ~0ULL;
        uint64_t after_combined = ~0ULL;

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            uint64_t before = read_all_pins(blacklist_mask);

            bool current_pin_state = gpio_read(pin);
            gpio_input_init(pin, GPIO_PULL_UP);
            delay_ms(20);

            uint64_t after = read_all_pins(blacklist_mask);
            char after_bin[65];
            for (int b = 0; b < 64; ++b)
                after_bin[b] = ((after >> (63 - b)) & 1ULL) ? '1' : '0';
            after_bin[64] = '\0';
            // printf("DEBUG: pin %u after: 0b%s\n", pin, after_bin);

            before_combined &= before;
            after_combined &= after;

            gpio_reset(pin);
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            delay_us(TIME_BETWEEN_MEASUREMENTS_US);
        }
        log_pin_changes(PHASE_1_PULLUP_DRIVE_HIGH, pindata, blacklist_mask, before_combined, after_combined, pin);
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
        uint64_t before_combined = ~0ULL;
        uint64_t after_combined = ~0ULL;

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            uint64_t before = read_all_pins(blacklist_mask);

            bool current_pin_state = gpio_read(pin);
            gpio_output_init(pin);
            gpio_drive_low(pin);
            delay_ms(30);

            uint64_t after = read_all_pins(blacklist_mask);
            before_combined &= before;
            after_combined &= after;

            gpio_reset(pin);
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);
            delay_us(TIME_BETWEEN_MEASUREMENTS_US);
        }

        log_pin_changes(PHASE_2_NO_PULL_DRIVE_LOW, pindata, blacklist_mask, before_combined, after_combined, pin);
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
        uint64_t before_combined = ~0ULL;
        uint64_t after_combined = ~0ULL;

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            uint64_t before = read_all_pins(blacklist_mask);
            gpio_output_init(pin);
            gpio_drive_high(pin);
            delay_ms(40);

            uint64_t after = read_all_pins(blacklist_mask);
            char after_bin[65];
            for (int b = 0; b < 64; ++b)
                after_bin[b] = ((after >> (63 - b)) & 1ULL) ? '1' : '0';
            after_bin[64] = '\0';
            // printf("DEBUG: pin %u after: 0b%s\n", pin, after_bin);

            before_combined &= before;
            after_combined &= after;

            gpio_reset(pin);
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            delay_us(TIME_BETWEEN_MEASUREMENTS_US);
        }
        log_pin_changes(PHASE_3_NO_PULL_DRIVE_HIGH, pindata, blacklist_mask, before_combined, after_combined, pin);
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