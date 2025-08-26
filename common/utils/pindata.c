#include "pindata.h"


/**
 * @brief Initialize an array of PinData structures
 * 
 * @param pindata Pointer to the PinData array
 * @param size Size of the array
 */
void initialize_pin_data_array(PinData *pindata, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i)
    {
        pindata[i].pin = i;
        pindata[i].event_index = 0;
    }
}
/**
 * @brief Add an event to the pin's event buffer
 * 
 * @param pindata Pointer to the PinData array
 * @param pin Pin number
 * @param event Event type to add
 */
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event)
{
    PinData *data = &pindata[pin];
    data->pin_event[data->event_index] = event;
    data->event_index = (data->event_index + 1) % EVENT_BUFFER_SIZE; // Circular buffer
}
