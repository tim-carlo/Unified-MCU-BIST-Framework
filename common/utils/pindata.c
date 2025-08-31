#include "pindata.h"
#include "printf.h" 


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

/**
 * @brief Print the pin data array and list events for each element
 * 
 * @param pindata Pointer to the PinData array
 * @param size Size of the array
 */
void print_pin_data_array(const PinData *pindata, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i)
    {
        printf("Pin %u:\n", pindata[i].pin);
        printf("  Event Index: %u\n", pindata[i].event_index);
        printf("  Events: ");
        for (uint8_t j = 0; j < EVENT_BUFFER_SIZE; ++j)
        {
            switch (pindata[i].pin_event[j]) {
            case HANDSHAKE_OK_INITIATOR:
                printf("HANDSHAKE_OK_INITIATOR ");
                break;
            case HANDSHAKE_OK_RESPONDER:
                printf("HANDSHAKE_OK_RESPONDER ");
                break;
            case HANDSHAKE_FAILURE:
                printf("HANDSHAKE_FAILURE ");
                break;
            default:
                printf("%d ", pindata[i].pin_event[j]);
                break;
            }
        }
        printf("\n");
    }
}
