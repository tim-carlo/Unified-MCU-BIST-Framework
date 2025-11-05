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

static const uint8_t threshold_percent = 90;
static const uint8_t threshold = (NUMBER_OF_SAMPLES_FOR_DEBOUNCING * threshold_percent) / 100;

static const uint8_t threshold_for_iterations = (NUMBER_OF_SAMPLES_FOR_MEASURING * 100) / 100;

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
        gpio_input_init(pin, pull_type);
    }
    pin = 0;

    // let the conductance discharge / charge
    delay_us(SETTLE_TIME_US);

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
                            uint64_t change_mask,
                            uint8_t test_pin)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        bool changed = (change_mask >> pin) & 1U;
        if (pin != test_pin && changed)
        {
            LOG("Pin %u changed when pin %u was changed\n", pin, test_pin);
            add_pin_connection(CONNECTION_TYPE_INTERNAL, pindata, test_pin, pin, (uint8_t)phase);
        }
        else if (pin == test_pin && !changed)
        {
            LOG("Pin %u did NOT change when pin %u was changed\n", pin, test_pin);

            switch (phase)
            {
            case PHASE_0_PULLDOWN_DRIVE_LOW:
                add_pin_event(pindata, test_pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
                break;
            case PHASE_1_PULLUP_DRIVE_HIGH:
                add_pin_event(pindata, test_pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
                break;
            case PHASE_2_NO_PULL_DRIVE_LOW:
                add_pin_event(pindata, test_pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
                break;
            case PHASE_3_NO_PULL_DRIVE_HIGH:
                add_pin_event(pindata, test_pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
                break;
            default:
                break;
            }
        }
    }
}

static void measure_phase(uint64_t blacklist_mask,
                          void (*setup_func)(uint8_t pin),
                          uint16_t delay_ms_val,
                          void (*post_func)(uint64_t blacklist_mask, uint8_t pin),
                          uint32_t change_history[64])
{
    for (uint8_t iteration = 0; iteration < NUMBER_OF_SAMPLES_FOR_MEASURING; iteration++)
    {
        BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
        uint8_t pin;

        while (bitmap_iterator_next(&it, &pin))
        {
            uint64_t before = read_all_pins(blacklist_mask);

            setup_func(pin);
            delay_ms(delay_ms_val);

            uint64_t after = read_all_pins(blacklist_mask);

            uint64_t change = before ^ after;

            for (uint8_t b = 0; b < 64; b++)
            {
                if (pin == b)
                    continue;
                
                if (change & (1ULL << b))
                    change_history[b]++;
            }

            post_func(blacklist_mask, pin);
            delay_us(TIME_BETWEEN_MEASUREMENTS_US);
        }
    }
}

static uint64_t compute_affected_pins(const uint32_t change_history[64])
{
    uint64_t affected_pins_mask = 0;

    for (uint8_t pin = 0; pin < 64; pin++)
    {
        if (change_history[pin] >= threshold_for_iterations)
            affected_pins_mask |= (1ULL << pin);
    }
    return affected_pins_mask;
}

static void log_all_pins(SetOneMeasureALLPhase phase, uint64_t blacklist_mask,
                         PinData *pindata, uint64_t changed_majority)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        log_pin_changes(phase, pindata, blacklist_mask, changed_majority, pin);
    }
}

static void setup_pulldown_low(uint8_t pin)
{
    gpio_input_init(pin, GPIO_PULL_DOWN);
}
static void setup_pullup_high(uint8_t pin)
{
    gpio_input_init(pin, GPIO_PULL_UP);
}
static void setup_no_pull_low(uint8_t pin)
{
    gpio_output_init(pin);
    gpio_drive_low(pin);
}
static void setup_no_pull_high(uint8_t pin)
{
    gpio_output_init(pin);
    gpio_drive_high(pin);
}

static void post_reset_pullup(uint64_t blacklist, uint8_t pin)
{
    gpio_reset(pin);
    flush_pin_states(blacklist, GPIO_PULL_UP);
}
static void post_reset_pulldown(uint64_t blacklist, uint8_t pin)
{
    gpio_reset(pin);
    flush_pin_states(blacklist, GPIO_PULL_DOWN);
}

/**
 * @brief Phase 0: Pull-down configuration, drive each pin low and measure others
 */
static void phase_0_pulldown_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 0: Pull-down, drive low\n");
    reset_all_pins(blacklist_mask);

    uint32_t changed_history[64] = {0};

    measure_phase(blacklist_mask, setup_pulldown_low, 10, post_reset_pullup, changed_history);

    uint64_t changed_majority = compute_affected_pins(changed_history);
    log_all_pins(PHASE_0_PULLDOWN_DRIVE_LOW, blacklist_mask, pindata, changed_majority);
}

/**
 * @brief Phase 1: Pull-up configuration, drive each pin high and measure others
 */
static void phase_1_pullup_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 1: Pull-up, drive high\n");
    reset_all_pins(blacklist_mask);

    uint32_t changed_history[64] = {0};

    measure_phase(blacklist_mask, setup_pullup_high, 20, post_reset_pulldown, changed_history);

    uint64_t before_majority, after_majority;
    uint64_t changed_majority = compute_affected_pins(changed_history);
    log_all_pins(PHASE_1_PULLUP_DRIVE_HIGH, blacklist_mask, pindata, changed_majority);
}

/**
 * @brief Phase 2: No pull resistors, drive each pin low and measure others
 */
static void phase_2_no_pull_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 2: No pull, drive low\n");
    reset_all_pins(blacklist_mask);

    uint32_t changed_history[64] = {0};
    measure_phase(blacklist_mask, setup_no_pull_low, 30, post_reset_pullup, changed_history);

    uint64_t changed_majority = compute_affected_pins(changed_history);
    log_all_pins(PHASE_2_NO_PULL_DRIVE_LOW, blacklist_mask, pindata, changed_majority);
}

/**
 * @brief Phase 3: No pull resistors, drive each pin high and measure others
 */
static void phase_3_no_pull_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 3: No pull, drive high\n");
    reset_all_pins(blacklist_mask);

    uint32_t changed_history[64] = {0};
    measure_phase(blacklist_mask, setup_no_pull_high, 40, post_reset_pulldown, changed_history);
    uint64_t changed_majority = compute_affected_pins(changed_history);
    log_all_pins(PHASE_3_NO_PULL_DRIVE_HIGH, blacklist_mask, pindata, changed_majority);
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