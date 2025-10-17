#include "pindata.h"
#include "printf.h"

// Global seen devices list
uint64_t *seen_devices = NULL;
uint8_t seen_devices_count = 0;

/**
 * @brief Initialize an array of PinData structures
 * And set the first seen device as own device ID
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
    // Initialize seen devices with own device ID
    uint64_t own_id = get_unique_id();
    add_seen_device(&own_id);
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

    // Update event mask
    data->event_mask |= (1 << event);
}

/**
 * @brief Get the index of unique id object
 *
 * @param unique_id
 * @return uint8_t
 */
uint8_t get_index_of_unique_id(uint64_t unique_id)
{
    for (uint8_t i = 0; i < seen_devices_count; i++)
    {
        if (seen_devices[i] == unique_id)
        {
            return i; // return existing index
        }
    }
    return 255; // not found
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
    uint8_t existing_index = get_index_of_unique_id(*other_device_id);
    if (existing_index != 255)
    {
        return existing_index;
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

// Helper to check if a connection already exists
static bool connection_exists(PinData *data, uint8_t other_pin, uint8_t device_index)
{
    for (uint8_t i = 0; i < data->connection_index; i++)
    {
        if (data->connections[i].other_pin == other_pin && data->connections[i].device_index == device_index)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Add a connection to another pin and device ID
 * @param pindata Pointer to the PinData array
 * @param pin Pin number
 * @param other_pin_index Other pin number
 */
void add_pin_connection(PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint8_t device_index)
{
    PinData *data = &pindata[pin];

    // Prevent duplicate connections
    if (connection_exists(data, other_pin_index, device_index))
    {
        return;
    }

    // Init connection list if needed
    if (data->connections == NULL)
    {
        data->connections = malloc(sizeof(PinConnection) * INITIAL_CONNECTION_CAPACITY);
        if (data->connections == NULL)
        {
            return; // allocation failed
        }
        data->connection_index = 0;
        data->connection_capacity = INITIAL_CONNECTION_CAPACITY;
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

    data->connections[data->connection_index].other_pin = other_pin_index;
    data->connections[data->connection_index].device_index = device_index;
    data->connection_index++;
}

/**
 * @brief Sort the pin connections based on device index and other pin
 * @param pindata Pointer to the PinData array
 * @param pin Pin number
 */
void sort_pin_connections(PinData *pindata, uint8_t pin)
{
    PinData *data = &pindata[pin];
    if (data->connections == NULL || data->connection_index < 2)
    {
        return; // No need to sort
    }
    // Simple bubble sort for small arrays
    for (uint8_t i = 0; i < data->connection_index - 1; i++)
    {
        bool swapped = false;
        for (uint8_t j = 0; j < data->connection_index - i - 1; j++)
        {
            PinConnection *a = &data->connections[j];
            PinConnection *b = &data->connections[j + 1];

            if (a->device_index > b->device_index ||
                (a->device_index == b->device_index && a->other_pin > b->other_pin))
            {
                PinConnection tmp = *a;
                *a = *b;
                *b = tmp;
                swapped = true;
            }
        }
        if (!swapped)
            break; // Already sorted
    }
}

/**
 * @brief Sort the seen devices list to ensure deterministic ordering
 */
void sort_seen_devices(PinData *pindata)
{
    if (seen_devices_count < 2)
    {
        return; // No need to sort
    }
    // Simple bubble sort for small arrays
    for (uint8_t i = 0; i < seen_devices_count - 1; i++)
    {
        bool swapped = false;
        for (uint8_t j = 0; j < seen_devices_count - i - 1; j++)
        {
            if (seen_devices[j] > seen_devices[j + 1])
            {
                uint64_t tmp = seen_devices[j];
                seen_devices[j] = seen_devices[j + 1];
                seen_devices[j + 1] = tmp;
                swapped = true;
            }
        }
        if (!swapped)
            break; // Already sorted
    }
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

/**
 * @brief Get the own device UUID from the seen_devices list
 * @return Own device UUID
 */
uint64_t get_own_device_id()
{
    return seen_devices[0];
}
