#include "pindata.h"
#include "printf.h" // assumed for debug output

// NOTE: seen_devices and seen_devices_index are defined in the header file.

uint8_t seen_devices_count = 0;
uint64_t seen_devices[MAX_SEEN_DEVICES];
uint8_t seen_devices_index = 0;

/**
 * @brief Initializes the PinData array.
 */
void initialize_pin_data_array(PinData *pindata, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i)
    {
        pindata[i].pin = i;
        pindata[i].event_mask = 0;
        pindata[i].connection_index = 0;
        pindata[i].connections_count = 0;
    }
    // Add own device ID as first element
    uint64_t own_id = get_unique_id();
    add_seen_device(own_id);
}

/**
 * @brief Adds a pin event.
 */
void add_pin_event(PinData *pindata, uint8_t pin, PinEventType event)
{
    pindata[pin].event_mask |= (1 << event);
}

/**
 * @brief Checks whether a pin event exists.
 */
bool check_if_pinevent_exists(PinData *pindata, uint8_t pin, PinEventType event)
{
    return (pindata[pin].event_mask & (1 << event)) != 0;
}

/**
 * @brief Finds the index of a device ID in the list.
 */
uint8_t get_index_of_unique_id(uint64_t unique_id)
{
    // CORRECT: The loop MUST iterate over the actual number of stored
    // devices (`seen_devices_count`), not over the write pointer (`seen_devices_index`).
    for (uint8_t i = 0; i < seen_devices_count; i++)
    {
        if (seen_devices[i] == unique_id)
        {
            return i; // device found
        }
    }
    return DEVICE_NOT_FOUND; // not found
}

/**
 * @brief Adds a device ID to the list (ring buffer logic).
 */
uint8_t add_seen_device(uint64_t other_device_id)
{

    uint8_t existing_index = get_index_of_unique_id(other_device_id);
    if (existing_index != DEVICE_NOT_FOUND)
    {
        return existing_index; // return index of existing device
    }

    seen_devices[seen_devices_index] = other_device_id;
    uint8_t index_that_was_written = seen_devices_index;

    seen_devices_index = (seen_devices_index + 1) % MAX_SEEN_DEVICES;
    if (seen_devices_count < MAX_SEEN_DEVICES)
    {
        seen_devices_count++;
    }

    return index_that_was_written;
}

/**
 * @brief Helper function to check whether a connection already exists.
 */
static bool connection_exists(PinData *data, uint8_t other_pin, uint8_t device_index)
{
    const uint8_t count = (data->connection_index < MAX_CONNECTIONS_PER_PIN) ? data->connection_index : MAX_CONNECTIONS_PER_PIN;

    for (uint8_t i = 0; i < count; i++)
    {
        if (data->connections[i].other_pin == other_pin && data->connections[i].device_index == device_index)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Adds a pin connection (ring buffer logic).
 */
void add_pin_connection(PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint64_t device_uuid)
{
    PinData *data = &pindata[pin];

    uint8_t device_index = get_index_of_unique_id(device_uuid);
    if (device_index == DEVICE_NOT_FOUND)
    {
        // Device not found, add it
        device_index = add_seen_device(&device_uuid);
    }

    if (connection_exists(data, other_pin_index, device_index))
    {
        return;
    }
    uint8_t conn_idx_to_write = data->connection_index % MAX_CONNECTIONS_PER_PIN;
    data->connections[conn_idx_to_write].other_pin = other_pin_index;
    data->connections[conn_idx_to_write].device_index = device_index;

    data->connection_index++;
    if (conn_idx_to_write >= MAX_CONNECTIONS_PER_PIN)
    {
        data->connections_count = 1;
    }
    else
    {
        data->connections_count++;
    }
}

/**
 * @brief Sorts the connections of a pin.
 */
void sort_pin_connections(PinData *pindata, uint8_t pin)
{
    PinData *data = &pindata[pin];

    const uint8_t count = (data->connection_index < MAX_CONNECTIONS_PER_PIN) ? data->connection_index : MAX_CONNECTIONS_PER_PIN;

    if (count < 2)
    {
        return; // nothing to sort
    }

    // Simple bubble sort
    for (uint8_t i = 0; i < count - 1; i++)
    {
        bool swapped = false;
        for (uint8_t j = 0; j < count - i - 1; j++)
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
            break; // already sorted
    }
}

/**
 * @brief Sorts the seen devices array.
 */
void sort_seen_devices()
{
    if (seen_devices_count < 2)
    {
        return;
    }

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
            break;
    }
}

/**
 * @brief Returns the own device UUID.
 */
uint64_t get_own_device_id()
{
    if (seen_devices_count > 0)
    {
        return seen_devices[MY_DEVICE_ID_INDEX];
    }
    return 0;
}
