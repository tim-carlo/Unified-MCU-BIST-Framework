#include "set_one_high_measure_all.h"
#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)

static const uint8_t TIME_BETWEEN_PHASES = 10;
static const uint8_t NUMBER_OF_SAMPLES_FOR_DEBOUNCING = 11;
static const uint8_t NUMBER_OF_SAMPLES_FOR_MEASURING = 10;

static const uint32_t SETTLE_TIME_US = 1000;
static const uint32_t TIME_BETWEEN_MEASUREMENTS_US = 10000; // 10ms
static const uint32_t DEBOUNCING_DELAY_US = 10;

static const uint8_t threshold_percent = 50;
static const uint8_t threshold = (NUMBER_OF_SAMPLES_FOR_DEBOUNCING * threshold_percent) / 100;

static const uint8_t threshold_measure_percent = 100;
static const uint8_t threshold_measure = (NUMBER_OF_SAMPLES_FOR_MEASURING * threshold_measure_percent) / 100;

static const uint8_t SAMPLES_BEFORE_CHANGING_PIN = 8;
static const uint8_t SAMPLES_AFTER_CHANGING_PIN = 16;

static const uint8_t repetitions_step1 = 5;
static const uint8_t repetitions_step2 = 5;
static const uint8_t repetitions_step3 = 5;

typedef uint8_t SetOneMeasureALLPhase;
enum
{
    PHASE_0_ONE_SET_PULLDOWN,
    PHASE_1_ONE_SET_PULLUP,
    PHASE_2_DRIVE_LOW,
    PHASE_3_DRIVE_HIGH,
};
typedef struct
{
    uint64_t high;
    uint64_t undefined;
} PinSamplesMultiplePins;

typedef struct
{
    bool high;
    bool undefined;
} PinSamples;

/**
 * @brief Read all pins multiple times and determine their stable states
 *
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = read this pin
 * @param samples Number of samples to read for debouncing
 * @return PinSamplesMultiplePins Structure containing high/low and undefined states
 */
static inline PinSamplesMultiplePins read_all_pins(const uint64_t blacklist_mask, const uint8_t samples, const uint32_t Ddelay_us)
{
    const uint64_t active_mask = ~blacklist_mask;
    uint8_t high_count[64] = {0};

    for (uint8_t i = 0; i < samples; i++)
    {
        uint64_t internal_mask = active_mask;
        uint8_t pin;

        while (bitmap_iterator_next_mask_as_param(&internal_mask, &pin))
        {
            if (gpio_read(pin))
            {
                high_count[pin]++;
            }
        }

        delay_us(Ddelay_us);
    }
    uint8_t pin;

    PinSamplesMultiplePins result = {0, 0};

    const uint8_t tristate_threshold = samples / 3;
    const uint8_t high = samples - tristate_threshold;
    const uint8_t low = tristate_threshold;
    while (bitmap_iterator_next_mask_as_param(&active_mask, &pin))
    {
        if (high_count[pin] >= high)
        {
            result.high |= (1ULL << pin);
        }
        else if (high_count[pin] <= low)
        {
            result.high &= ~(1ULL << pin);
        }
        else
        {
            result.undefined |= (1ULL << pin);
        }
    }

    return result;
}

static inline PinSamples sample_pin(const uint8_t pin, const uint8_t samples, const uint32_t Ddelay_us)
{
    uint8_t high_count = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        if (gpio_read(pin))
        {
            high_count++;
        }
        delay_us(Ddelay_us);
    }

    PinSamples result;
    const uint8_t tristate_threshold = samples / 3;
    const uint8_t high = samples - tristate_threshold;
    const uint8_t low = tristate_threshold;

    if (high_count >= high)
    {
        result.high = true;
        result.undefined = false;
    }
    else if (high_count <= low)
    {
        result.high = false;
        result.undefined = false;
    }
    else
    {
        result.undefined = true;
    }

    return result;
}

/**
 * @brief Set all pins to input with specified pull type, wait for settling, then reset them
 * This helps to discharge/charge any capacitances on the lines, and reduce undefined states.
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = set this pin
 * @param pull_type Pull resistor type to set on the pins
 */
static inline void flush_pin_states(const uint64_t blacklist_mask, const gpio_pull_t pull_type)
{

    uint8_t pin;
    uint64_t mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        gpio_input_init(pin, pull_type);
    }
    // let the conductance discharge / charge
    delay_us(SETTLE_TIME_US);

    gpio_reset_from_blacklist(blacklist_mask);
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
 *
 * @param phase Current measurement phase
 * @param pindata Pointer to PinData structure to log events and connections
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = consider this pin
 * @param changes Array of change counts for each pin
 */
static void log_pin_changes(SetOneMeasureALLPhase phase,
                            PinData *pindata,
                            uint64_t blacklist_mask,
                            uint8_t changes[64],
                            uint8_t undefined_counts[64],
                            uint8_t state_after_as_expected_of_own,
                            uint8_t test_pin)
{

    // We want to be clear if the pin is clearly not behaving as expected
    // So that it is externally modified
    if (state_after_as_expected_of_own < threshold_measure && undefined_counts[test_pin] == 0)
    {
        switch (phase)
        {
        case PHASE_0_ONE_SET_PULLDOWN:
            // state_after_as_expected_of_own
            LOG("state_after_as_expected_of_own %d < threshold_measure %d\n", state_after_as_expected_of_own, threshold_measure);
            LOG("Phase %d: Pin %u did not go low as expected\n", phase, test_pin);
            add_pin_event(pindata, test_pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
            break;
        case PHASE_1_ONE_SET_PULLUP:
            LOG("Phase %d: Pin %u did not go high as expected\n", phase, test_pin);
            add_pin_event(pindata, test_pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
            break;
        case PHASE_2_DRIVE_LOW:
            LOG("Phase %d: Pin %u did not go low as expected\n", phase, test_pin);
            add_pin_event(pindata, test_pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
            break;
        case PHASE_3_DRIVE_HIGH:
            LOG("Phase %d: Pin %u did not go high as expected\n", phase, test_pin);
            add_pin_event(pindata, test_pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
            break;
        default:
            break;
        }
    }

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        if (test_pin != pin && changes[pin] >= threshold_measure)
        {
            LOG("Phase %d: Pin %u affected by pin %u\n", phase, pin, test_pin);
            add_pin_connection(CONNECTION_TYPE_INTERNAL, pindata, test_pin, pin, (uint8_t)phase);
        }
    }
}

static void step_1(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Step 1A/B: Passive pull-up/pull-down test\n");
    uint8_t pin_high[64] = {0};
    uint8_t pin_low[64] = {0};

    const uint8_t number_of_samples = 6;
    const uint8_t Ddelay_us = 10;

    // Phase A:
    const uint8_t phase_1_threshold = (repetitions_step1 * 100) / 100;

    for (uint8_t i = 0; i < repetitions_step1; i++)
    {
        gpio_input_from_blacklist(blacklist_mask, GPIO_PULL_DOWN);

        delay_us(SETTLE_TIME_US);
        // reset all pins to high impedance
        gpio_reset_from_blacklist(blacklist_mask);
        delay_ms(TIME_BETWEEN_PHASES * (i + 1));
        PinSamplesMultiplePins state = read_all_pins(blacklist_mask, number_of_samples, Ddelay_us);

        for (uint8_t k = 0; k < 64; k++)
        {
            // Ignore all pins that are undefined
            if (!(state.undefined & (1ULL << k)))
            {
                // Check if pin is high
                if (state.high & (1ULL << k))
                {
                    pin_high[k]++;
                }
                // Check if pin is low
                else if (!(state.high & (1ULL << k)))
                {
                    pin_low[k]++;
                }
            }
        }
    }
    uint8_t pin;
    uint64_t mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        if (pin_high[pin] >= phase_1_threshold)
        {
            add_pin_event(pindata, pin, STEP_1_A_HIGH);
        }

        if (pin_low[pin] >= phase_1_threshold)
        {
            add_pin_event(pindata, pin, STEP_1_A_LOW);
        }
    }

    // clear counts for phase B
    memset(pin_high, 0, sizeof(pin_high));
    memset(pin_low, 0, sizeof(pin_low));

    // Phase B:
    for (uint8_t i = 0; i < repetitions_step1; i++)
    {
        gpio_input_from_blacklist(blacklist_mask, GPIO_PULL_UP);
        // delay
        delay_us(SETTLE_TIME_US);
        // reset all pins to high impedance
        gpio_reset_from_blacklist(blacklist_mask);
        delay_ms(TIME_BETWEEN_PHASES * (i + 1));
        PinSamplesMultiplePins state = read_all_pins(blacklist_mask, number_of_samples, Ddelay_us);

        for (uint8_t k = 0; k < 64; k++)
        {
            if (!(state.undefined & (1ULL << k)))
            {
                // Check if pin is high
                if (state.high & (1ULL << k))
                {
                    pin_high[k]++;
                }
                // Check if pin is low
                else if (!(state.high & (1ULL << k)))
                {
                    pin_low[k]++;
                }
            }
        }
    }
    pin = 0;
    mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        if (pin_high[pin] >= phase_1_threshold)
        {
            add_pin_event(pindata, pin, STEP_1_B_HIGH);
        }

        if (pin_low[pin] >= phase_1_threshold)
        {
            add_pin_event(pindata, pin, STEP_1_B_LOW);
        }
    }
}
static void step_2(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Step 2A/B: Weak pull-up/pull-down test\n");

    const uint8_t threshold_step2 = (repetitions_step2 * 100) / 100;
    const uint8_t number_of_samples = 6;
    const uint8_t Ddelay_us = 10;

    // Step A:
    uint64_t pin_mask = ~blacklist_mask;
    uint8_t pin;
    while (bitmap_iterator_next_mask_as_param(&pin_mask, &pin))
    {
        uint8_t low = 0;
        uint8_t high = 0;
        for (uint8_t i = 0; i < repetitions_step2; i++)
        {
            gpio_input_init(pin, GPIO_PULL_DOWN);
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));
            PinSamples state = sample_pin(pin, number_of_samples, Ddelay_us);
            // Rest to high impedance
            gpio_reset(pin);
            // Check if pin is low
            if (!state.undefined)
            {
                if (state.high)
                {
                    high++;
                }
                else
                {
                    low++;
                }
            }
        }
        // Add here the event

        if (low >= threshold_step2)
        {
            add_pin_event(pindata, pin, STEP_2_A_LOW);
        }
        if (high >= threshold_step2)
        {
            add_pin_event(pindata, pin, STEP_2_A_HIGH);
        }

        // Reset counters for step B
        low = 0;
        high = 0;

        for (uint8_t i = 0; i < repetitions_step2; i++)
        {
            gpio_input_init(pin, GPIO_PULL_UP);
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));
            PinSamples state = sample_pin(pin, number_of_samples, Ddelay_us);
            // Rest to high impedance
            gpio_reset(pin);
            // Check if pin is low
            if (!state.undefined)
            {
                if (state.high)
                {
                    high++;
                }
                else
                {
                    low++;
                }
            }
        }

        if (low >= threshold_step2)
        {
            add_pin_event(pindata, pin, STEP_2_B_LOW);
        }
        if (high >= threshold_step2)
        {
            add_pin_event(pindata, pin, STEP_2_B_HIGH);
        }
    }
}

static void step_3(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Step 3A/B: Active drive strength test\n");

    const uint8_t threshold_phase3 = (repetitions_step3 * 100) / 100;

    uint8_t pin_high[64] = {0};
    uint8_t pin_low[64] = {0};

    const uint8_t number_of_samples = 6;
    const uint8_t Ddelay_us = 10;

    uint64_t mask = ~blacklist_mask;
    uint8_t pin;

    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {

        for (uint8_t r = 0; r < repetitions_step3; r++)
        {
            // Drive the pin low briefly and measure effect on others
            gpio_output_init(pin);
            gpio_drive_low(pin);
            delay_ms(1);
            // measure while driven
            PinSamples measurements = sample_pin(pin, number_of_samples, Ddelay_us);
            // Reset the pin quickly
            gpio_reset(pin);
            // Check which pins were affected
            if (!measurements.undefined && !measurements.high)
            {
                pin_low[pin]++;
            }
            else if (!measurements.undefined && measurements.high)
            {
                pin_high[pin]++;
            }
        }
    }

    mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        if (pin_high[pin] >= threshold_phase3)
        {
            add_pin_event(pindata, pin, STEP_3_A_HIGH);
        }
        if (pin_low[pin] >= threshold_phase3)
        {
            add_pin_event(pindata, pin, STEP_3_A_LOW);
        }
    }
    // clear counts for low drive
    memset(pin_high, 0, sizeof(pin_high));
    memset(pin_low, 0, sizeof(pin_low));
    mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        for (uint8_t r = 0; r < repetitions_step3; r++)
        {
            // Drive the pin high briefly and measure effect on others
            gpio_output_init(pin);
            gpio_drive_high(pin);
            delay_ms(1);
            // measure while driven
            PinSamples measurements = sample_pin(pin, number_of_samples, Ddelay_us);
            // Reset the pin quickly
            gpio_reset(pin);
            // Check which pins were affected

            if (!measurements.undefined && !measurements.high)
            {
                pin_low[pin]++;
            }
            else if (!measurements.undefined && measurements.high)
            {
                pin_high[pin]++;
            }
        }
    }
    mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        if (pin_high[pin] >= threshold_phase3)
        {
            add_pin_event(pindata, pin, STEP_3_B_HIGH);
        }
        if (pin_low[pin] >= threshold_phase3)
        {
            add_pin_event(pindata, pin, STEP_3_B_LOW);
        }
    }
}

/**
 * @brief Phase 0: Pull-down configuration, drive each pin low and measure others
 */
static void phase_0_one_set_pulldown(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 0: Pull-down, drive low\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint8_t changes[64] = {0};
        uint8_t undefined_counts[64] = {0};
        uint8_t state_after_as_expected_of_own = 0;

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {

            // Read own pin state:
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);

            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN, 10);

            gpio_input_init(pin, GPIO_PULL_DOWN);

            // variable delay
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN, 10);
            gpio_reset(pin);

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                // Check own pin
                if (before.undefined & (1ULL << check_pin) || after.undefined & (1ULL << check_pin))
                {
                    undefined_counts[check_pin]++;
                }
                else
                {
                    if (check_pin == pin)
                    {
                        if (!(after.high & (1ULL << check_pin)))
                        {
                            state_after_as_expected_of_own++;
                        }
                    }
                    else
                    {
                        uint64_t diff = (before.high ^ after.high) & ~blacklist_mask;
                        if (diff & (1ULL << check_pin))
                        {
                            changes[check_pin]++;
                        }
                    }
                }
            }
        }
        log_pin_changes(PHASE_0_ONE_SET_PULLDOWN, pindata, blacklist_mask, changes, undefined_counts, state_after_as_expected_of_own, pin);
    }
}

/**
 * @brief Phase 1: Pull-up configuration, drive each pin high and measure others
 */
static void phase_1_one_set_pullup(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 1: Pull-up, drive high\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint8_t changes[64] = {0};
        uint8_t undefined_counts[64] = {0};
        uint8_t state_after_as_expected_of_own = 0;

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN, 10);

            gpio_input_init(pin, GPIO_PULL_UP);
            // variable delay
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));
            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN, 10);

            gpio_reset(pin);

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (before.undefined & (1ULL << check_pin) || after.undefined & (1ULL << check_pin))
                {
                    undefined_counts[check_pin]++;
                }
                else
                {
                    if (check_pin == pin)
                    {
                        // Check if the pin is high as expected
                        if ((after.high & (1ULL << check_pin)))
                        {
                            state_after_as_expected_of_own++;
                        }
                    }
                    else
                    {
                        uint64_t diff = (before.high ^ after.high) & ~blacklist_mask;
                        if (diff & (1ULL << check_pin))
                            changes[check_pin]++;
                    }
                }
            }
        }
        log_pin_changes(PHASE_1_ONE_SET_PULLUP, pindata, blacklist_mask, changes, undefined_counts, state_after_as_expected_of_own, pin);
    }
}

/**
 * @brief Phase 2: No pull resistors, drive each pin low and measure others
 */
static void phase_2_drive_low(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 2: No pull, drive low\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint8_t changes[64] = {0};
        uint8_t undefined_counts[64] = {0};
        uint8_t state_after_as_expected_of_own = 0;

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN, 10);

            gpio_output_init(pin);
            gpio_drive_low(pin);

            delay_us(500 * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN, 10);
            gpio_reset(pin);
            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (before.undefined & (1ULL << check_pin) || after.undefined & (1ULL << check_pin))
                {
                    undefined_counts[check_pin]++;
                }
                else
                {
                    if (check_pin == pin)
                    {
                        // Check if the pin is low as expected
                        if (!(after.high & (1ULL << check_pin)))
                        {
                            state_after_as_expected_of_own++;
                        }
                    }
                    else
                    {
                        uint64_t diff = (before.high ^ after.high) & ~blacklist_mask;
                        if (diff & (1ULL << check_pin))
                            changes[check_pin]++;
                    }
                }
            }
        }
        log_pin_changes(PHASE_2_DRIVE_LOW, pindata, blacklist_mask, changes, undefined_counts, state_after_as_expected_of_own, pin);
    }
}

/**
 * @brief Phase 3: No pull resistors, drive each pin high and measure others
 */
static void phase_3_drive_high(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 3: No pull, drive high\n");
    reset_all_pins(blacklist_mask);

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it, &pin))
    {
        uint8_t changes[64] = {0};
        uint8_t undefined_counts[64] = {0};
        uint8_t state_after_as_expected_of_own = 0;

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN, 10);

            gpio_output_init(pin);
            gpio_drive_high(pin);

            delay_us(500 * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN, 10);
            gpio_reset(pin);

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (before.undefined & (1ULL << check_pin) || after.undefined & (1ULL << check_pin))
                {
                    undefined_counts[check_pin]++;
                }
                else
                {
                    if (check_pin == pin)
                    {
                        // Check if the pin is high as expected
                        if ((after.high & (1ULL << check_pin)))
                        {
                            state_after_as_expected_of_own++;
                        }
                    }
                    else
                    {
                        uint64_t diff = (before.high ^ after.high) & ~blacklist_mask;
                        if (diff & (1ULL << check_pin))
                            changes[check_pin]++;
                    }
                }
            }
        }
        log_pin_changes(PHASE_3_DRIVE_HIGH, pindata, blacklist_mask, changes, undefined_counts, state_after_as_expected_of_own, pin);
    }
}

/**
 * @brief Run all 8 measurement phases
 */
void run_set_one_high_measure_all(uint64_t blacklist_mask, PinData *pindata, uint8_t pindata_size)
{
    reset_all_pins(blacklist_mask);
    phase_0_one_set_pulldown(blacklist_mask, pindata);
    phase_1_one_set_pullup(blacklist_mask, pindata);
    phase_2_drive_low(blacklist_mask, pindata);
    phase_3_drive_high(blacklist_mask, pindata);

    step_1(blacklist_mask, pindata);
    step_2(blacklist_mask, pindata);
    step_3(blacklist_mask, pindata);
    reset_all_pins(blacklist_mask);
}