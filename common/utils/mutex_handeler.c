#include "mutex_handeler.h"
#include "manchester.h"
#include "nrf52840_gpio.h"

static bool iam_mutex_owner = false;
static uint8_t current_mutex_pin = 255;
static bool currently_having_mutex = false;

/**
 * @brief Initialize the mutex handler
 *
 * @param result Pointer to DataHandshakeResult to store the result
 */
void mutex_handeler_init(DataHandshakeResult *result)
{
    if (result == NULL)
        return; // Null pointer check
    iam_mutex_owner = result->i_am_mutex_owner;
    current_mutex_pin = result->mutex_pin;
}

void mutex_handler_request_mutex()
{
    if (iam_mutex_owner && currently_having_mutex)
    {
        // Already have the mutex
        return;
    }
    if (iam_mutex_owner)
    {
        // if the line is currently low, wait until it goes high... So the other device can release the mutex
        while (!gpio_read(current_mutex_pin))
        {
            // wait
        }
        // Now the line is high, we can request the mutex
        currently_having_mutex = true;
    }
    else
    {
        // Wait until the line goes low, indicating the other device has released the mutex
        while (gpio_read(current_mutex_pin))
        {
            // wait
        }
        gpio_od_hold_low(current_mutex_pin);
        // Now the line is low, we can take the mutex
        currently_having_mutex = true;
    }
}

void mutex_handler_release_mutex()
{
    if (!currently_having_mutex)
    {
        // Do not have the mutex to release
        return;
    }
    if (iam_mutex_owner)
    {
        // Drive the line high to release the mutex
        gpio_od_hold_low(current_mutex_pin);
    }
    else
    {
        // Wait until the line goes high, indicating the other device has taken the mutex
        gpio_od_release(current_mutex_pin);
        while (!gpio_read(current_mutex_pin))
        {
            // wait
        }
    }
    currently_having_mutex = false;
}

void mutex_handeler_deinit(void)
{
    iam_mutex_owner = false;
    current_mutex_pin = 255;
    currently_having_mutex = false;
}
