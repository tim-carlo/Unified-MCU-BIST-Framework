#include "check_initial_state.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#endif

// Returns a 64-bit value where each bit represents the state of a pin (bit 0 = pin 0, ...)
uint64_t get_initial_pin_state(void)
{
    uint64_t state1 = 0, state2 = 0;
    // First measurement
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++) {
        if (gpio_read(abs_pin)) {
            state1 |= ((uint64_t)1 << abs_pin);
        }
    }

    // Wait for 10 seconds
    delay_ms(10000);

    // Second measurement
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++) {
        if (gpio_read(abs_pin)) {
            state2 |= ((uint64_t)1 << abs_pin);
        }
    }

    // Combine both results with AND
    return state1 & state2;
}

