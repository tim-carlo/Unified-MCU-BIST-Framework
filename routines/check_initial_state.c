#include "check_initial_state.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#endif

/**
 * @brief Get the initial pin state object  
 * 
 * @param expected_state 
 * @return uint64_t 
 */
uint64_t get_initial_pin_state(uint8_t expected_state)
{
    // Initialize the state to all bits set (all pins high)
    uint64_t state = ~0ULL;

    // Read the state of all GPIO pins multiple times to ensure stability
    for (uint8_t i = 0; i < NUMBER_OF_SCANNING_ITERATIONS; i++) {
        uint64_t temp = 0;
        for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++) {
            if (gpio_read(abs_pin) == expected_state) {
                temp |= ((uint64_t)1 << abs_pin);
            }
        }
        state &= temp;
        delay_ms(DELAY_BETWEEN_READS_MS); // Delay to allow for pin state stabilization
    }

    return state;
}