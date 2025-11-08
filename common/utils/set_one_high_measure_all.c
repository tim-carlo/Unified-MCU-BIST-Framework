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
    PHASE_0_PULLDOWN_DRIVE_LOW,
    PHASE_1_PULLUP_DRIVE_HIGH,
    PHASE_2_NO_PULL_DRIVE_LOW,
    PHASE_3_NO_PULL_DRIVE_HIGH
};
typedef struct
{
    uint64_t high_low;
    uint64_t floating;
} PinSamplesMultiplePins;

typedef struct
{
    bool high;
    bool floating;
} PinSamples;

/**
 * @brief Read all pins multiple times and determine their stable states
 *
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = read this pin
 * @param samples Number of samples to read for debouncing
 * @return PinSamplesMultiplePins Structure containing high/low and floating states
 */
static inline PinSamplesMultiplePins read_all_pins(const uint64_t blacklist_mask, const uint8_t samples)
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

        delay_us(DEBOUNCING_DELAY_US);
    }
    uint8_t pin;

    PinSamplesMultiplePins result = {0, 0};

    const uint8_t high = samples - (samples / 4);
    const uint8_t low = samples - high;
    while (bitmap_iterator_next_mask_as_param(&active_mask, &pin))
    {
        if (high_count[pin] > high)
        {
            result.high_low |= (1ULL << pin);
        }
        else if (high_count[pin] <= low)
        {
            result.high_low &= ~(1ULL << pin);
        }
        else
        {
            result.floating |= (1ULL << pin);
        }
    }

    return result;
}

static inline PinSamples sample_pin(const uint8_t pin, const uint8_t samples)
{
    uint8_t high_count = 0;

    for (uint8_t i = 0; i < samples; i++)
    {
        if (gpio_read(pin))
        {
            high_count++;
        }
        delay_us(DEBOUNCING_DELAY_US);
    }

    PinSamples result;
    const uint8_t high = samples - (samples / 4);
    const uint8_t low = samples - high;

    if (high_count > high)
    {
        result.high = true;
        result.floating = false;
    }
    else if (high_count <= low)
    {
        result.high = false;
        result.floating = false;
    }
    else
    {
        result.floating = true;
    }

    return result;
}

/**
 * @brief Set all pins to input with specified pull type, wait for settling, then reset them
 * This helps to discharge/charge any capacitances on the lines, and reduce floating states.
 * @param blacklist_mask Mask where 1 = skip this pin, 0 = set this pin
 * @param pull_type Pull resistor type to set on the pins
 */
static inline void flush_pin_states(const uint64_t blacklist_mask, gpio_pull_t pull_type)
{

    uint8_t pin;
    uint64_t mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        gpio_input_init(pin, pull_type);
        // bisschen zu timing abhänigig
    }
    // let the conductance discharge / charge
    delay_us(SETTLE_TIME_US);

    mask = ~blacklist_mask;
    gpio_reset_from_blacklist(mask);

    // Oder hier einmal lesen
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
                            uint32_t changes[64],
                            uint8_t test_pin)
{
    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        if (pin == test_pin)
        {
            switch (phase)
            {
            case PHASE_0_PULLDOWN_DRIVE_LOW:
                if (changes[pin] >= threshold_measure)
                {
                    LOG("Phase %d: Pin %u NOT affected when driving low with pull-down\n", phase, pin);
                    add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_PULLED_DOWN);
                }
                break;
            case PHASE_2_NO_PULL_DRIVE_LOW:
                if (changes[pin] >= threshold_measure)
                {
                    LOG("Phase %d: Pin %u NOT affected when driving low with no pull\n", phase, pin);
                    add_pin_event(pindata, pin, PIN_IS_NOT_LOW_WHEN_DRIVEN_LOW);
                }
                break;
            case PHASE_1_PULLUP_DRIVE_HIGH:
                if (changes[pin] < threshold_measure)
                {
                    LOG("Phase %d: Pin %u NOT affected when driving high with pull-up\n", phase, pin);
                    add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_PULLED_UP);
                }
                break;
            case PHASE_3_NO_PULL_DRIVE_HIGH:
                if (changes[pin] < threshold_measure)
                {
                    LOG("Phase %d: Pin %u NOT affected when driving high with no pull\n", phase, pin);
                    add_pin_event(pindata, pin, PIN_IS_NOT_HIGH_WHEN_DRIVEN_HIGH);
                }
                break;
            default:
                break;
            }
        }
        else if (pin != test_pin)
        {
            if (changes[pin] >= threshold_measure)
            {
                LOG("Phase %d: Pin %u affected by pin %u\n", phase, pin, test_pin);
                add_pin_connection(CONNECTION_TYPE_INTERNAL, pindata, test_pin, pin, (uint8_t)phase);
            }
        }
    }
}

static void step_1(uint64_t blacklist_mask, PinData *pindata)
{
    uint8_t pin_high[64] = {0};
    uint8_t pin_low[64] = {0};

    // Phase A:
    const uint8_t phase_1_threshold = (repetitions_step1 * 70) / 100;
    for (uint8_t i = 0; i < repetitions_step1; i++)
    {
        uint64_t pin_mask = ~blacklist_mask;
        while (bitmap_iterator_next_mask_as_param(&pin_mask, &j))
        {
            gpio_input_init(j, GPIO_PULL_DOWN);
        }
        delay_ms(1 * (i + 1));

        // reset all pins to high impedance
        pin_mask = ~blacklist_mask;
        gpio_reset_from_blacklist(pin_mask);
        PinSamplesMultiplePins state = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);
        for (uint8_t k = 0; k < 64; k++)
        {
            if (state.floating & (1ULL << k))
            {
                continue;
            }
            // Check if pin is low
            if (state.high_low & (1ULL << k))
            {
                pin_high[k]++;
            }
        }
    }
    // Phase B:
    for (uint8_t i = 0; i < repetitions_step1; i++)
    {
        uint64_t pin_mask = ~blacklist_mask;
        while (bitmap_iterator_next_mask_as_param(&pin_mask, &j))
        {
            gpio_input_init(j, GPIO_PULL_UP);
        }

        // delay
        delay_ms(repetitions_step1 * (i + 1));
        // reset all pins to high impedance
        pin_mask = ~blacklist_mask;
        gpio_reset_from_blacklist(pin_mask);
        PinSamplesMultiplePins state = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);
        for (uint8_t k = 0; k < 64; k++)
        {
            if (state.floating & (1ULL << k))
            {
                continue;
            }
            // Check if pin is high
            if (!state.high_low & (1ULL << k))
            {
                pin_low[k]++;
            }
        }
    }
    uint8_t pin;
    uint64_t mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        if (pin_high[pin] >= phase_1_threshold)
        {
            LOG("Step 1: Pin %u is stable high\n", pin);
            add_pin_event(pindata, pin, STEP_1_A_FOLLOWS);
        }

        if (pin_low[pin] >= phase_1_threshold)
        {
            LOG("Step 1: Pin %u is stable low\n", pin);
            add_pin_event(pindata, pin, STEP_1_B_FOLLOWS);
        }
    }
}
static void step_2(uint64_t blacklist_mask, PinData *pindata)
{
    uint8_t pin_high[64] = {0};
    uint8_t pin_low[64] = {0};

    uint8_t threshold_step2 = (repetitions_step2 * 70) / 100;
    // Step A:
    uint32_t pin_mask = ~blacklist_mask;
    uint8_t pin;
    while (bitmap_iterator_next_mask_as_param(&pin_mask, &pin))
    {
        for (uint8_t i = 0; i < repetitions_step2; i++)
        {
            gpio_input_init(pin, GPIO_PULL_DOWN);
            delay_us(SETTLE_TIME_US);
            PinSamples state = sample_pin(pin, SAMPLES_BEFORE_CHANGING_PIN);
            // Check if pin is low

            if (!state.floating && state.high)
            {
                pin_high[pin]++;
            }
            // Rest to high impedance
            gpio_reset(pin);
        }
        for (uint8_t i = 0; i < repetitions_step2; i++)
        {
            gpio_input_init(pin, GPIO_PULL_UP);
            delay_us(SETTLE_TIME_US);
            PinSamples state = sample_pin(pin, SAMPLES_BEFORE_CHANGING_PIN);
            // Check if pin is low
            if (!state.floating && !state.high)
            {
                pin_low[pin]++;
            }
            // Rest to high impedance
            gpio_reset(pin);
        }
    }
    pin_mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&pin_mask, &pin))
    {
        if (pin_high[pin] >= threshold_step2)
        {
            LOG("Step 2: Pin %u is weakly pulled high\n", pin);
            add_pin_event(pindata, pin, STEP_2_A_FOLLOWS);
        }
        if (pin_low[pin] >= threshold_step2)
        {
            LOG("Step 2: Pin %u is weakly pulled low\n", pin);
            add_pin_event(pindata, pin, STEP_2_B_FOLLOWS);
        }
    }
}

static void step_3(uint64_t blacklist_mask, PinData *pindata)
{
    LOG("Phase 3A/B: Active drive strength test\n");

    BitmapIterator it = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    const uint8_t threshold_phase3 = (repetitions_step3 * 70) / 100;

    uint8_t pin_high[64] = {0};
    uint8_t pin_low[64] = {0};

    while (bitmap_iterator_next(&it, &pin))
    {

        for (uint8_t r = 0; r < repetitions_step3; r++)
        {
            // Drive the pin high briefly and measure effect on others
            gpio_output_init(pin);
            gpio_drive_high(pin);
            delay_ms(1);
            // measure while driven
            PinSamples measurements = sample_pin(pin, SAMPLES_BEFORE_CHANGING_PIN);
            // Reset the pin quickly
            gpio_reset(pin);
            // Check which pins were affected

            if (!measurements.floating && !measurements.high)
            {
                pin_low[pin]++;
            }
        }
        for (uint8_t r = 0; r < repetitions_step3; r++)
        {
            // Drive the pin low briefly and measure effect on others
            gpio_output_init(pin);
            gpio_drive_low(pin);
            delay_ms(1);
            // measure while driven
            PinSamples measurements = sample_pin(pin, SAMPLES_BEFORE_CHANGING_PIN);
            // Reset the pin quickly
            gpio_reset(pin);
            // Check which pins were affected
            if (!measurements.floating && measurements.high)
            {
                pin_high[pin]++;
            }
        }
    }
    uint8_t check_pin;
    uint64_t mask = ~blacklist_mask;
    while (bitmap_iterator_next_mask_as_param(&mask, &check_pin))
    {
        if (pin_high[check_pin] >= threshold_phase3)
        {
            LOG("Step 3: Pin %u is actively driven high\n", check_pin);
            add_pin_event(pindata, check_pin, STEP_3_A_FOLLOWS);
        }
        if (pin_low[check_pin] >= threshold_phase3)
        {
            LOG("Step 3: Pin %u is actively driven low\n", check_pin);
            add_pin_event(pindata, check_pin, STEP_3_B_FOLLOWS);
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
        uint32_t changes[64] = {0};
        uint32_t floating_counts[64] = {0};

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {

            // Read own pin state:
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);

            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);

            gpio_input_init(pin, GPIO_PULL_DOWN);

            // Wartezeit hier variieren
            // Damit man unnabhänig von Kapazitäten ist
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN);
            uint64_t diff = (before.high_low ^ after.high_low) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                // ignore the floating of the pin afterwards:
                // if the pin is floating afterwards, it cannot be counted as changed
                if (!(after.floating & (1ULL << check_pin)))
                {
                    if (diff & (1ULL << check_pin))
                    {
                        changes[check_pin]++;
                    }
                }
                else
                {
                    floating_counts[check_pin]++;
                }
            }

            gpio_reset(pin);
        }
        log_pin_changes(PHASE_0_PULLDOWN_DRIVE_LOW, pindata, blacklist_mask, changes, pin);
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
        uint32_t changes[64] = {0};

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);

            gpio_input_init(pin, GPIO_PULL_UP);
            delay_ms(TIME_BETWEEN_PHASES * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN);
            uint64_t diff = (before.high_low ^ after.high_low) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (!(after.floating & (1ULL << check_pin)))
                {
                    if (diff & (1ULL << check_pin))
                        changes[check_pin]++;
                }
            }

            gpio_reset(pin);
        }
        log_pin_changes(PHASE_1_PULLUP_DRIVE_HIGH, pindata, blacklist_mask, changes, pin);
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
        uint32_t changes[64] = {0};
        uint32_t floating_counts[64] = {0};

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);

            gpio_output_init(pin);
            gpio_drive_low(pin);

            delay_ms(TIME_BETWEEN_PHASES * (i + 1));

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN);

            if (after.floating & (1ULL << pin))
            {
                floating_counts[pin]++;
            }

            uint64_t diff = (before.high_low ^ after.high_low) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (!(after.floating & (1ULL << check_pin)))
                {
                    if (diff & (1ULL << check_pin))
                        changes[check_pin]++;
                }
            }

            gpio_reset(pin);
        }
        log_pin_changes(PHASE_2_NO_PULL_DRIVE_LOW, pindata, blacklist_mask, changes, pin);
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
        uint32_t changes[64] = {0};

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_DOWN);
            PinSamplesMultiplePins before = read_all_pins(blacklist_mask, SAMPLES_BEFORE_CHANGING_PIN);

            gpio_output_init(pin);
            gpio_drive_high(pin);

            delay_ms(TIME_BETWEEN_PHASES * (i + 1));

            // Großer Widerstand und die größte Kapazität führen zum Worstcase

            PinSamplesMultiplePins after = read_all_pins(blacklist_mask, SAMPLES_AFTER_CHANGING_PIN);
            uint64_t diff = (before.high_low ^ after.high_low) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (!(after.floating & (1ULL << check_pin)))
                {
                    if (diff & (1ULL << check_pin))
                        changes[check_pin]++;
                }
            }

            gpio_reset(pin);
        }
        log_pin_changes(PHASE_3_NO_PULL_DRIVE_HIGH, pindata, blacklist_mask, changes, pin);
    }
}

/**
 * @brief Run all 8 measurement phases
 */
void run_set_one_high_measure_all(uint64_t blacklist_mask, PinData *pindata, uint8_t pindata_size)
{
    reset_all_pins(blacklist_mask);
    phase_0_pulldown_drive_low(blacklist_mask, pindata);
    phase_1_pullup_drive_high(blacklist_mask, pindata);
    phase_2_no_pull_drive_low(blacklist_mask, pindata);
    phase_3_no_pull_drive_high(blacklist_mask, pindata);
    step_1(blacklist_mask, pindata);
    step_2(blacklist_mask, pindata);
    step_3(blacklist_mask, pindata);
    reset_all_pins(blacklist_mask);
}