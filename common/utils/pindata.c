#include "pindata.h"
#include "printf.h"

// Global seen devices list
uint64_t *seen_devices = NULL;
uint8_t seen_devices_count = 0;

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
        pindata[i].connection_index = 0;
        pindata[i].connection_capacity = 0;
        pindata[i].connections = NULL;
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

// Pin Connection Functions:

/**
 * @brief Check if a connection already exists
 *
 * @param data Pointer to the PinData structure
 * @param other_pin The connected pin number
 * @param device_index Index of the device in seen_devices array
 * @return true
 * @return false
 */
bool connection_exists(PinData *data, uint8_t other_pin, uint8_t device_index)
{
    for (uint8_t i = 0; i < data->connection_index; i++)
    {
        if (data->connections[i].other_pin == other_pin &&
            data->connections[i].device_index == device_index)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Add a device ID to the seen devices list if not already present
 *
 * @param other_device_id Pointer to the device ID to add
 * @return Index of the device in seen_devices array, or 255 if failed
 */
uint8_t add_seen_device(uint64_t *other_device_id)
{
    // already seen?
    for (uint8_t i = 0; i < seen_devices_count; i++)
    {
        if (seen_devices[i] == *other_device_id)
        {
            return i; // return existing index
        }
    }

    if (seen_devices_count < MAX_SEEN_DEVICES)
    {
        uint64_t *new_seen = realloc(seen_devices, sizeof(uint64_t) * (seen_devices_count + 1));
        if (new_seen != NULL)
        {
            seen_devices = new_seen;
            seen_devices[seen_devices_count] = *other_device_id;
            uint8_t new_index = seen_devices_count;
            seen_devices_count++;
            return new_index;
        }
    }
    return 255; // failure indicator
}

/**
 * @brief Add a connection to another pin and device ID
 * @param pindata Pointer to the PinData array
 * @param pin Pin number
 * @param other_pin The connected pin number
 * @param other_device_id Pointer to the connected device ID
 */
void add_pin_connection(PinData *pindata, uint8_t pin, uint8_t other_pin, uint64_t *other_device_id)
{
    PinData *data = &pindata[pin];

    // Get or add device to seen_devices list
    uint8_t device_index = add_seen_device(other_device_id);
    if (device_index == 255) {
        return; // Failed to add device
    }

    // Init connection list if needed
    if (data->connections == NULL)
    {
        data->connections = malloc(sizeof(PinConnection) * INITIAL_CONNECTION_CAPACITY);
        if (data->connections == NULL)
        {
            return; // allocation failed → abort safely
        }
        data->connection_index = 0;
        data->connection_capacity = INITIAL_CONNECTION_CAPACITY;
    }

    // Prevent duplicate connections
    if (connection_exists(data, other_pin, device_index))
    {
        return;
    }

    if (data->connection_index >= data->connection_capacity)
    {
        uint8_t new_capacity = data->connection_capacity * 2;
        PinConnection *new_connections =
            realloc(data->connections, sizeof(PinConnection) * new_capacity);

        if (new_connections == NULL)
        {
            return; // realloc failed
        }

        data->connections = new_connections;
        data->connection_capacity = new_capacity;
    }

    // Add new connection with device index
    data->connections[data->connection_index].other_pin = other_pin;
    data->connections[data->connection_index].device_index = device_index;
    data->connection_index++;
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
            switch (pindata[i].pin_event[j])
            {
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
