#include "random_utils.h"

/**
 * @brief Selects a random pin that is not blacklisted and not successful from a PinData array
 *
 * @param pindata Pointer to PinData array
 * @param length Number of elements in the array
 * @return uint32_t Pin number, or 0xFFFFFFFF if none available
 */
/* uint8_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask)
{
    // Count valid pins that are not blacklisted and not successful
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < NUMBER_OF_GPIO_PINS; ++i)
    {
        uint8_t pin = pindata[i].pin;
        if (pin >= 64)
            continue; // Skip invalid pins
        if (((blacklist_mask >> pin) & 1) == 0 && !is_successful(&pindata[i]))
        {
            valid_count++;
        }
    }

    // If no valid pins found, return 0xFFFFFFFF
    if (valid_count == 0)
    {
        return 255; // 0xFFFFFFFF in uint8_t is 255, which is invalid for pin numbers
    }
    uint32_t pick = random32() % valid_count;

    // Iterate through the pins again to find the selected one
    for (uint32_t i = 0; i < NUMBER_OF_GPIO_PINS; ++i)
    {
        uint8_t pin = pindata[i].pin;
        if (pin >= 64)
            continue;
        if (((blacklist_mask >> pin) & 1) == 0 && !is_successful(&pindata[i]))
        {
            if (pick == 0)
            {
                return pin;
            }
            pick--;
        }
    }
    return 255; // Should never reach here, but return 255 as a fallback
} */

/**
 * @brief Select a random pin from the available (non-blacklisted) pins
 * 
 * @param internal_blacklist_mask Bitmask of blacklisted pins (1 = blacklisted, 0 = available)
 * @return uint8_t Pin number, or 0xFF if none available
 */
uint8_t select_random_pin(uint64_t internal_blacklist_mask, uint8_t number_of_pins)
{
    uint64_t available_mask = ~internal_blacklist_mask;

    if (available_mask == 0)
        return 0xFF; // No available pins

    
    // Pick a random index
    uint32_t random_number = random32() % number_of_pins;

    // Iterate again to select the random_number-th available pin
    BitmapIterator it = bitmap_iterator_create(available_mask);
    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        if (random_number == 0)
            return pin;
        random_number--;
    }

    return 0xFF;
}