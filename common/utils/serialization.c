#include "serialisation.h"
#include <string.h>

SerializedChunk *current_chunk = NULL;
uint8_t current_pin_data_index = 0;
uint8_t current_chunk_id = 0;

static PinData *current_pindata = NULL;
static uint8_t actual_pindata_size = 0;
static uint32_t current_hash = 0;
static uint32_t current_header_hash = 0;

InitializationResult initialize_serialization(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size)
{
    if (output_chunk == NULL || pindata == NULL)
        return INITIALIZATION_ERROR;

    current_pindata = pindata;
    current_chunk = output_chunk;
    current_chunk->data = NULL;
    current_chunk->size_in_bytes = 0;
    current_pin_data_index = 0;
    actual_pindata_size = pindata_size;

    return INITIALIZATION_OK;
}

// Serialize pin data in chunks for transmission
SerializationResult serialize_next_chunk()
{
    if (current_chunk == NULL || current_pindata == NULL)
        return SERIALIZATION_ERROR_NULL_POINTER;

    size_t buffer_size = CHUNK_BUFFER_SIZE;
    uint8_t *cbor_buffer = (uint8_t *)malloc(buffer_size);
    if (cbor_buffer == NULL)
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;

    uint8_t *write_ptr = cbor_buffer;
    uint8_t entries_to_serialize = 0;
    uint8_t start_index = current_pin_data_index;

    // Count how many pins to include in this chunk
    for (uint8_t i = current_pin_data_index; i < actual_pindata_size && entries_to_serialize < NUMBER_OF_ENTRIES_PER_CHUNK; i++)
    {
        entries_to_serialize++;
        current_pin_data_index = i + 1;
    }

    if (entries_to_serialize == 0)
    {
        free(cbor_buffer);
        return SERIALIZATION_OK;
    }

    if ((write_ptr - cbor_buffer) >= (buffer_size - 16))
    {
        free(cbor_buffer);
        return SERIALIZATION_ERROR_BUFFER_TOO_SMALL;
    }

    // Write CBOR map header (now only 3 entries, no hash inside)
    uint8_t bytes_written = cb0r_write(write_ptr, CB0R_MAP, 3);
    write_ptr += bytes_written;

    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_CHUNK_ID);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, current_chunk_id);
    write_ptr += bytes_written;

    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_NUM_ENTRIES);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, entries_to_serialize);
    write_ptr += bytes_written;

    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_PINS);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, entries_to_serialize);
    write_ptr += bytes_written;

    // Serialize each pin's data
    uint8_t serialized_count = 0;
    for (uint8_t i = start_index; i < actual_pindata_size && serialized_count < entries_to_serialize; i++)
    {
        PinData *pin_data = &current_pindata[i];

        if ((write_ptr - cbor_buffer) >= (buffer_size - 32))
        {
            free(cbor_buffer);
            return SERIALIZATION_ERROR_BUFFER_TOO_SMALL;
        }

        // Pin data map with pin number, events, connections
        bytes_written = cb0r_write(write_ptr, CB0R_MAP, 3);
        write_ptr += bytes_written;

        bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_PIN);
        write_ptr += bytes_written;
        bytes_written = cb0r_write(write_ptr, CB0R_INT, pin_data->pin);
        write_ptr += bytes_written;

        bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_EVENTS);
        write_ptr += bytes_written;
        
        // Write events array
        if (pin_data->event_index > 0) {
            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, pin_data->event_index);
            write_ptr += bytes_written;
            for (uint8_t j = 0; j < pin_data->event_index; j++)
            {
                bytes_written = cb0r_write(write_ptr, CB0R_INT, pin_data->pin_event[j]);
                write_ptr += bytes_written;
            }
        } else {
            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, 0);
            write_ptr += bytes_written;
        }

        // Handle connections array (sanity check for embedded safety)
        bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_CONNECTIONS);
        write_ptr += bytes_written;
        
        if (pin_data->connection_index > 0) {
            if (pin_data->connection_index > 10) {
                free(cbor_buffer);
                return SERIALIZATION_ERROR_INVALID_INPUT;
            }

            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, pin_data->connection_index);
            write_ptr += bytes_written;
            
            // Write connection data inside the array
            if (pin_data->connections != NULL) {
                for (uint8_t j = 0; j < pin_data->connection_index; j++)
                {
                    bytes_written = cb0r_write(write_ptr, CB0R_MAP, 2);
                    write_ptr += bytes_written;

                    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_OTHER_PIN);
                    write_ptr += bytes_written;
                    bytes_written = cb0r_write(write_ptr, CB0R_INT, pin_data->connections[j].other_pin);
                    write_ptr += bytes_written;

                    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_DEVICE_ID);
                    write_ptr += bytes_written;
                    
                    // Get actual device ID from seen_devices array using device_index
                    uint8_t device_idx = pin_data->connections[j].device_index;
                    uint64_t device_id = (device_idx < seen_devices_count && seen_devices != NULL) 
                                        ? seen_devices[device_idx] 
                                        : 0;
                    bytes_written = cb0r_write(write_ptr, CB0R_INT, device_id);
                    write_ptr += bytes_written;
                }
            }
        } else {
            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, 0);
            write_ptr += bytes_written;
        }

        serialized_count++;
    }

    // Calculate xxHash for integrity over CBOR data
    size_t cbor_data_size = write_ptr - cbor_buffer;
    current_hash = XXH32(cbor_buffer, cbor_data_size, 0);

    // Create final packet: [2 BYTE LENGTH][CBOR BYTES][4 BYTE HASH]
    size_t total_packet_size = 2 + cbor_data_size + 4; // Length + CBOR + Hash
    uint8_t *packet_buffer = (uint8_t *)malloc(total_packet_size);
    if (packet_buffer == NULL) {
        free(cbor_buffer);
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    uint8_t *packet_ptr = packet_buffer;
    
    // Write 2-byte length (big endian)
    uint16_t cbor_size = (uint16_t)cbor_data_size;
    *packet_ptr++ = (uint8_t)((cbor_size >> 8) & 0xFF);
    *packet_ptr++ = (uint8_t)(cbor_size & 0xFF);
    
    // Copy CBOR data
    memcpy(packet_ptr, cbor_buffer, cbor_data_size);
    packet_ptr += cbor_data_size;
    
    // Write 4-byte hash (big endian)
    *packet_ptr++ = (uint8_t)((current_hash >> 24) & 0xFF);
    *packet_ptr++ = (uint8_t)((current_hash >> 16) & 0xFF);
    *packet_ptr++ = (uint8_t)((current_hash >> 8) & 0xFF);
    *packet_ptr++ = (uint8_t)(current_hash & 0xFF);

    // Set final packet data
    free(cbor_buffer);
    current_chunk->size_in_bytes = total_packet_size;
    current_chunk->data = packet_buffer;

    return SERIALIZATION_OK;
}

// Generate header with device info and transmission metadata
SerializationResult generate_cbor_header(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size)
{
    if (output_chunk == NULL || pindata == NULL)
    {
        return SERIALIZATION_ERROR_NULL_POINTER;
    }

    // Buffer
    uint8_t *cbor_buffer = (uint8_t *)malloc(HEADER_BUFFER_SIZE);
    if (cbor_buffer == NULL)
    {
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    uint8_t *write_ptr = cbor_buffer;
    uint8_t bytes_written;

    // Get device information
    uint64_t device_uuid = get_unique_id();
    const char *device_family = get_chip_family_name();
    uint8_t family_name_len = strlen(device_family);

    // Count active pins (pins with events or connections)
    uint8_t active_pins = 0;
    for (uint8_t i = 0; i < pindata_size; i++)
    {
        if (pindata[i].event_index > 0 || pindata[i].connection_index > 0)
        {
            active_pins++;
        }
    }

    // Calculate total number of chunks needed based on all pins
    uint8_t total_chunks = 0;
    if (pindata_size > 0)
    {
        total_chunks = (pindata_size + NUMBER_OF_ENTRIES_PER_CHUNK - 1) / NUMBER_OF_ENTRIES_PER_CHUNK;
    }

    // Buffer overflow protection
    if (HEADER_BUFFER_SIZE < (32 + family_name_len))
    {
        free(cbor_buffer);
        cbor_buffer = NULL;
        return SERIALIZATION_ERROR_BUFFER_TOO_SMALL;
    }

    // Write CBOR header map with device metadata (now only 5 entries, no hash inside)
    bytes_written = cb0r_write(write_ptr, CB0R_MAP, 5);
    write_ptr += bytes_written;

    // 1. Device UUID (Key 0)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_DEVICE_UUID);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, device_uuid);
    write_ptr += bytes_written;

    // 2. Device Family Name (Key 1)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_DEVICE_FAMILY);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_UTF8, family_name_len);
    write_ptr += bytes_written;
    memcpy(write_ptr, device_family, family_name_len);
    write_ptr += family_name_len;

    // 3. Total chunks (Key 2)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_TOTAL_CHUNKS);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, total_chunks);
    write_ptr += bytes_written;

    // 4. Total pins (Key 3)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_TOTAL_PINS);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, pindata_size);
    write_ptr += bytes_written;

    // 5. Active pins (Key 4)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_ACTIVE_PINS);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, active_pins);
    write_ptr += bytes_written;

    // Calculate xxHash for header integrity over CBOR data
    size_t header_data_size = write_ptr - cbor_buffer;
    current_header_hash = XXH32(cbor_buffer, header_data_size, 0);

    // Create final packet: [2 BYTE LENGTH][CBOR BYTES][4 BYTE HASH]
    size_t total_packet_size = 2 + header_data_size + 4; // Length + CBOR + Hash
    uint8_t *packet_buffer = (uint8_t *)malloc(total_packet_size);
    if (packet_buffer == NULL) {
        free(cbor_buffer);
        cbor_buffer = NULL;
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    uint8_t *packet_ptr = packet_buffer;
    
    // Write 2-byte length (big endian)
    uint16_t cbor_size = (uint16_t)header_data_size;
    *packet_ptr++ = (uint8_t)((cbor_size >> 8) & 0xFF);
    *packet_ptr++ = (uint8_t)(cbor_size & 0xFF);
    
    // Copy CBOR data
    memcpy(packet_ptr, cbor_buffer, header_data_size);
    packet_ptr += header_data_size;
    
    // Write 4-byte hash (big endian)
    *packet_ptr++ = (uint8_t)((current_header_hash >> 24) & 0xFF);
    *packet_ptr++ = (uint8_t)((current_header_hash >> 16) & 0xFF);
    *packet_ptr++ = (uint8_t)((current_header_hash >> 8) & 0xFF);
    *packet_ptr++ = (uint8_t)(current_header_hash & 0xFF);

    // Set final packet data
    free(cbor_buffer);
    cbor_buffer = NULL;
    output_chunk->size_in_bytes = total_packet_size;
    output_chunk->data = packet_buffer;

    return SERIALIZATION_OK;
}
