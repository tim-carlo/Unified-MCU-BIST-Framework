#include "pindata.h"
#include "printf.h"

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
 * @param other_device_id The unique ID of the other device.
 * @return The index where the device ID was added.
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
 * @param connection_type The type of connection (internal or external).
 * @param data The PinData structure for the pin.
 * @param other_pin The other pin in the connection.
 * @param parameter Additional parameter for the connection.
 * @return true if the connection exists, false otherwise.
 */
static bool connection_exists(ConnectionType connection_type, PinData *data, uint8_t other_pin, uint8_t parameter)
{
    const uint8_t count = data->connections_count;

    uint8_t start_index = (data->connection_index + MAX_CONNECTIONS_PER_PIN - count) % MAX_CONNECTIONS_PER_PIN;

    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t idx = (start_index + i) % MAX_CONNECTIONS_PER_PIN;
        if (data->connections[idx].other_pin == other_pin &&
            data->connections[idx].parameter == parameter &&
            data->connections[idx].connection_type == connection_type)
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief Adds a pin connection (ring buffer logic).
 * @param connection_type The type of connection (internal or external).
 * @param pindata The array of PinData structures.
 * @param pin The pin to which the connection is to be added.
 * @param other_pin_index The index of the other pin in the connection.
 * @param parameter Additional parameter for the connection. In case of external connections, it is used to store the idx of the other device.
 */
void add_pin_connection(ConnectionType connection_type, PinData *pindata, uint8_t pin, uint8_t other_pin_index, uint8_t parameter)
{
    PinData *data = &pindata[pin];
    uint8_t conn_idx_to_write;

    if (connection_exists(connection_type, data, other_pin_index, parameter))
    {
        return;
    }

    if (data->connections_count >= MAX_CONNECTIONS_PER_PIN)
    {
        add_pin_event(pindata, pin, EXCEEDS_CONNECTION_LIMIT);
        data->connection_index = 0; // wrap around
        data->connections_count = 0;
        memset(data->connections, 0, sizeof(data->connections));
    }

    data->connections[conn_idx_to_write].other_pin = other_pin_index;
    data->connections[conn_idx_to_write].parameter = parameter;
    data->connections[conn_idx_to_write].connection_type = connection_type;

    data->connections_count++;
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
