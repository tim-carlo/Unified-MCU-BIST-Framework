#include "check_initial_state.h"
#include "bitmap_iterator.h"
#include "printf.h"

static InitialStatePinData *internal_pin_data_array = NULL;

static void initialstate_rising_isr(uint8_t pin)
{
    internal_pin_data_array[pin].number_of_rises++;
}

static void initialstate_falling_isr(uint8_t pin)
{
    internal_pin_data_array[pin].number_of_falls++;
}

/**
 * @brief Get the initial state of all GPIO pins
 * @param pin_data_array Array to store pin data
 * @param black_list_mask Mask of pins to ignore (1 = ignore, 0 = read)
 */
void get_initial_pin_state(PinData *pin_data_array, uint64_t *black_list_mask)
{
    uint64_t all_pins_mask;
    if (NUMBER_OF_GPIO_PINS >= 64) {
        all_pins_mask = ~0ULL;  // All 64 bits set
    } else {
        all_pins_mask = (1ULL << NUMBER_OF_GPIO_PINS) - 1;
    }
    
    uint64_t valid_mask = ~(*black_list_mask) & all_pins_mask;

    int pin_count = __builtin_popcountll(valid_mask);
    if (pin_count == 0)
        return; // No pins to read

    if (internal_pin_data_array == NULL)
    {
        internal_pin_data_array = malloc(sizeof(InitialStatePinData) * pin_count);
        if (internal_pin_data_array == NULL)
            return; // Allocation failed
    }

    BitmapIterator it = bitmap_iterator_create(valid_mask);
    uint8_t idx = 0;
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        internal_pin_data_array[idx].pin_number = pin;
        internal_pin_data_array[idx].state = gpio_read(pin);
        internal_pin_data_array[idx].number_of_rises = 0;  // Initialize
        internal_pin_data_array[idx].number_of_falls = 0;  // Initialize
        idx++;
    }

    // Register ISRs for rising and falling edges
    gpio_listen_on_all_pins_interrupt(*black_list_mask, initialstate_rising_isr, initialstate_falling_isr);
    delay_ms(1000);

    // If a pin is still low, it might be stuck low
    for (uint8_t i = 0; i < pin_count; i++)
    {
        uint8_t pin = internal_pin_data_array[i].pin_number;
        
        if (internal_pin_data_array[i].state == 0 &&
            internal_pin_data_array[i].number_of_rises == 0 &&
            internal_pin_data_array[i].number_of_falls == 0)
        {
            // Set the corresponding bit in the blacklist mask
            *black_list_mask |= (1ULL << pin);

            // Add event to pin data array
            add_pin_event(pin_data_array, pin, PIN_INITIALLY_LOW);
        }
        else if (internal_pin_data_array[i].state == 1)
        {
            add_pin_event(pin_data_array, pin, PIN_INITIALLY_HIGH);
        }
    }

    // Print all pins from internal_pin_data_array
    for (uint8_t i = 0; i < pin_count; i++)
    {
        printf("Pin %u: number_of_rises=%u, number_of_falls=%u, state=%d\n",
               internal_pin_data_array[i].pin_number,
               internal_pin_data_array[i].number_of_rises,
               internal_pin_data_array[i].number_of_falls,
               internal_pin_data_array[i].state);
    }
    
    gpio_disable_all_interrupts(*black_list_mask);
    
    // free internal array
    free(internal_pin_data_array);
    internal_pin_data_array = NULL;
}