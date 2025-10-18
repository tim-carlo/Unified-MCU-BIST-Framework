#include "mutex_handeler.h"
#include "manchester.h"

static bool iam_mutex_owner = false;
static uint8_t current_mutex_pin = 255;
static bool currently_having_mutex = false;

static const uint8_t MAX_RELEASE_TRIES = 4;
static const uint8_t MAX_REQUEST_TRIES = 4;
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
    manchester_init(PMAN_BAUD_300);
    manchester_set_tx_pin_od(current_mutex_pin);
    manchester_set_rx_pin_od(current_mutex_pin);
}

void mutex_handler_request_mutex()
{
    if (iam_mutex_owner && currently_having_mutex)
    {
        // Already have the mutex
        return;
    }
    if (iam_mutex_owner && !currently_having_mutex)
    {
        uint8_t request = MUTEX_REQEST;
        uint8_t tries = 0;
        while (!currently_having_mutex)
        {
            if (tries >= MAX_REQUEST_TRIES)
            {
                printf("Max request tries reached, giving up\n");
                // Max tries reached, give up requesting mutex
                break;
            }
            // reinitialize open drain pin
            gpio_od_init(current_mutex_pin);

            uint8_t request = MUTEX_REQEST;
            manchester_transmit_array(&request, 1);
            uint8_t received;
            if (manchester_receive_array(&received, 1))
            {
                if (received == MUTEX_ACK)
                {
                    currently_having_mutex = true;
                }
            }
            tries++;
        }
        // now we have the mutex, line can be released
        gpio_reset(current_mutex_pin);
    }
    else
    {
        // Wait until the line goes low, indicating the other device has released the mutex
        while (!currently_having_mutex)
        {
            uint8_t received;
            if (manchester_receive_array(&received, 1))
            {
                uint8_t ack = MUTEX_ACK;
                // if we receive a request then send a allow signal
                if (received == MUTEX_REQEST)
                {
                    printf("received mutex request\n");
                    manchester_transmit_array(&ack, 1);
                    // now the other device has the mutex so we can reset the open drain pin
                    gpio_reset(current_mutex_pin);
                    gpio_input_init(current_mutex_pin, GPIO_PULL_NONE );
                    // Configure pin with no pull resistors to wait for release signal
                    
                }
                else if (received == MUTEX_RELEASE)
                {
                    printf("received mutex release\n");
                    gpio_od_init(current_mutex_pin);
                    currently_having_mutex = true;
                    manchester_transmit_array(&ack, 1);
                }
            }
        }
    }
}

void mutex_handler_release_mutex()
{
    if (!currently_having_mutex)
    {
        // Do not have the mutex to release
        return;
    }
    else
    {

        // reinitialize open drain pin
        gpio_od_init(current_mutex_pin);
        bool released_permitted = false;
        uint8_t tries = 0;

        while (!released_permitted)
        {
            if (tries >= MAX_RELEASE_TRIES)
            {
                printf("Max release tries reached, giving up\n");
                // Max tries reached, give up releasing mutex
                break;
            }
            uint8_t release = MUTEX_RELEASE;
            printf("releasing mutex\n");
            manchester_transmit_array(&release, 1);
            uint8_t received;
            if (manchester_receive_array(&received, 1))
            {
                if (received == MUTEX_ACK)
                {
                    released_permitted = true;
                    currently_having_mutex = false;
                }
            }
            tries++;
        }
        // now reset the line to normal state
        gpio_reset(current_mutex_pin);
    }
}

void mutex_handeler_deinit(void)
{
    manchester_deinit();
    current_mutex_pin = 255;
    currently_having_mutex = false;
}
