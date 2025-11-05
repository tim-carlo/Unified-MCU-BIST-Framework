#include "set_one_high_measure_all.h"
#include "bitmap_iterator.h"
#include "pindata.h"
#include "printf.h"
#include <string.h>

#define LOG(fmt, ...) printf("DEBUG: " fmt, ##__VA_ARGS__)

const uint8_t TIME_BETWEEN_PHASES = 10;

const uint8_t NUMBER_OF_SAMPLES_FOR_DEBOUNCING = 11;

static const uint8_t NUMBER_OF_SAMPLES_FOR_MEASURING = 10;

static const uint32_t SETTLE_TIME_US = 1000;
static const uint32_t TIME_BETWEEN_MEASUREMENTS_US = 10000; // 10ms
static const uint32_t DEBOUNCING_DELAY_US = 10;

static const uint8_t threshold_percent = 50;
static const uint8_t threshold = (NUMBER_OF_SAMPLES_FOR_DEBOUNCING * threshold_percent) / 100;

static const uint8_t threshold_measure_percent = 100;
static const uint8_t threshold_measure = (NUMBER_OF_SAMPLES_FOR_MEASURING * threshold_measure_percent) / 100;

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
            {
                high_count[pin]++;
            }
        }

        delay_us(DEBOUNCING_DELAY_US);
    }

    // Majority decision per pin
    uint64_t result = 0;

    BitmapIterator final_it = bitmap_iterator_create(active_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&final_it, &pin))
    {
        if (high_count[pin] > threshold)
        {
            result |= (1ULL << pin);
        }
    }

    return result;
}

static void flush_pin_states(const uint64_t blacklist_mask, gpio_pull_t pull_type)
{
    BitmapIterator it_pull = bitmap_iterator_create(~blacklist_mask);
    uint8_t pin;

    while (bitmap_iterator_next(&it_pull, &pin))
    {
        gpio_input_init(pin, pull_type);
        // bisschen zu timing abhänigig
    }
    pin = 0;

    // let the conductance discharge / charge
    delay_us(SETTLE_TIME_US);
    // Hier lesen einmal ausprobieren

    // BitmapIterator it_reset = bitmap_iterator_create(~blacklist_mask);
    // while (bitmap_iterator_next(&it_reset, &pin))
    // {
    //     // gpio_reset(pin);
    //     if (pull_type == GPIO_PULL_DOWN)
    //         gpio_input_init(pin, GPIO_PULL_UP);
    //     else if (pull_type == GPIO_PULL_UP)
    //         gpio_input_init(pin, GPIO_PULL_DOWN);

    //     // gpio_input_init(pin, pull_type);
    // }
    // delay_us(SETTLE_TIME_US);

    BitmapIterator it_final = bitmap_iterator_create(~blacklist_mask);
    while (bitmap_iterator_next(&it_final, &pin))
    {
        gpio_input_init(pin, GPIO_PULL_NONE);
    }

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
                add_pin_connection(CONNECTION_TYPE_INTERNAL, pindata, pin, test_pin, (uint8_t)phase);
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
        uint32_t changes[64] = {0};

        for (uint8_t i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            
            // Read own pin state:
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);

            uint64_t before = read_all_pins(blacklist_mask);
            
            gpio_input_init(pin, GPIO_PULL_DOWN);
            delay_ms(TIME_BETWEEN_PHASES);

            uint64_t after = read_all_pins(blacklist_mask);

            uint64_t diff = (before ^ after) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (diff & (1ULL << check_pin))
                    changes[check_pin]++;
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
            uint64_t before = read_all_pins(blacklist_mask);

            gpio_input_init(pin, GPIO_PULL_UP);
            delay_ms(TIME_BETWEEN_PHASES);

            uint64_t after = read_all_pins(blacklist_mask);
            uint64_t diff = (before ^ after) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (diff & (1ULL << check_pin))
                    changes[check_pin]++;
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

        for (int i = 0; i < NUMBER_OF_SAMPLES_FOR_MEASURING; i++)
        {
            flush_pin_states(blacklist_mask, GPIO_PULL_UP);
            uint64_t before = read_all_pins(blacklist_mask);

            gpio_output_init(pin);
            gpio_drive_low(pin);
            delay_ms(TIME_BETWEEN_PHASES);

            uint64_t after = read_all_pins(blacklist_mask);

            uint64_t diff = (before ^ after) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (diff & (1ULL << check_pin))
                    changes[check_pin]++;
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
            uint64_t before = read_all_pins(blacklist_mask);
            gpio_output_init(pin);
            gpio_drive_high(pin);
            delay_ms(TIME_BETWEEN_PHASES);

            // Großer Widerstand und die größte Kapazität führen zum Worstcase

            uint64_t after = read_all_pins(blacklist_mask);
            uint64_t diff = (before ^ after) & ~blacklist_mask;

            for (uint8_t check_pin = 0; check_pin < 64; check_pin++)
            {
                if (diff & (1ULL << check_pin))
                    changes[check_pin]++;
            }

            gpio_reset(pin);
            
        }
        log_pin_changes(PHASE_3_NO_PULL_DRIVE_HIGH, pindata, blacklist_mask, changes, pin);
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