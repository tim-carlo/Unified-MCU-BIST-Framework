#include "data_handshake.h"

#define BaudRate RATE_1200
#define BUFFER_SIZE 10
#define INTERVAL_MS 1000

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

/**
 * @brief Listens on all GPIO pins except those specified in the blacklist mask.
 *
 * @param blacklist_mask A bitmask where each bit represents whether to ignore (1) or listen (0) on the corresponding pin.
 */
void perform_data_handshake(uint64_t blacklist_mask)
{
    uint64_t random_time = random32() % INTERVAL_MS;
    
    // Calculate valid pins mask (invert blacklist)
    uint64_t all_pins_mask = (NUMBER_OF_GPIO_PINS >= 64) ? ~0ULL : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);
    uint64_t valid_pins_mask = ~blacklist_mask & all_pins_mask;

    while (true)
    {
        // Create bitmap iterator for valid pins
        BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
        uint8_t pin;
        
        // Iterate through all non-blacklisted pins
        while (bitmap_iterator_next(&it, &pin))
        {
            bool value = gpio_read(pin);
            if (value == 0)
            { // Pin is low
                manchester_init(pin, pin, RATE_1200);
                uint8_t buffer[BUFFER_SIZE];
                bool success = manchester_receive_array(buffer, BUFFER_SIZE);
                if (success)
                {
                    LOG("Received data on pin %d: ", pin);
                    for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
                        LOG("0x%02X ", buffer[i]);
                    }
                    LOG("\n");
                }
            }
        }
        
    }
}
