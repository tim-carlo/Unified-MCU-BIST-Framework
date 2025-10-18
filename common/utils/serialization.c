#include "serialisation.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

SerializedChunk *current_chunk = NULL;
uint8_t current_pin_data_index = 0;
uint8_t current_chunk_id = 1;

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
// Fixed serialize_next_chunk function - korrigierte Chunk ID Behandlung
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

    uint8_t bytes_written = cb0r_write(write_ptr, CB0R_MAP, 2);
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

        // write CBOR pin number
        bytes_written = cb0r_write(write_ptr, CB0R_INT, pin_data->pin);
        write_ptr += bytes_written;

        bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_EVENTS);
        write_ptr += bytes_written;

        // write CBOR event mask
        bytes_written = cb0r_write(write_ptr, CB0R_INT, pin_data->event_mask);
        write_ptr += bytes_written;

        // Handle connections array (sanity check for embedded safety)
        bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_CONNECTIONS);
        write_ptr += bytes_written;

        if (pin_data->connection_index > 0)
        {
            // Sanity check: limit maximum connections to 10
            if (pin_data->connection_index > 10)
            {
                free(cbor_buffer);
                return SERIALIZATION_ERROR_INVALID_INPUT;
            }

            // sort connections before serialization, is important for consistent ordering
            // To make sure the cobor output is consistent across multiple runs
            sort_pin_connections(current_pindata, pin_data->pin);

            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, pin_data->connection_index);
            write_ptr += bytes_written;

            // Write connection data inside the array
            if (pin_data->connections != NULL)
            {
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

                    // only write device index for compactness
                    // This must be sorted if the framework should work with multiple devices
                    // Only send the index to make the chucks compaireable
                    uint8_t device_idx = pin_data->connections[j].device_index;
                    bytes_written = cb0r_write(write_ptr, CB0R_INT, device_idx);
                    write_ptr += bytes_written;
                }
            }
        }
        else
        {
            bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, 0);
            write_ptr += bytes_written;
        }

        serialized_count++;
    }

    // Calculate CRC32 for integrity over CBOR data
    uint16_t cbor_data_size = write_ptr - cbor_buffer;
    current_hash = crcFast((unsigned char const *)cbor_buffer, cbor_data_size);

    // Create final packet: [1 BYTE PACKET_ID][2 BYTE LENGTH][CBOR BYTES][4 BYTE CRC32]
    uint16_t total_packet_size = 1 + 2 + cbor_data_size + 4; // Packet ID + Length + CBOR + CRC32
    uint8_t *packet_buffer = (uint8_t *)malloc(total_packet_size);
    if (packet_buffer == NULL)
    {
        free(cbor_buffer);
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    uint8_t *packet_ptr = packet_buffer;

    // Write 1-byte packet ID
    *packet_ptr = current_chunk_id;
    packet_ptr += 1;

    // Write 2-byte length (LITTLE ENDIAN) - length of CBOR data only
    uint16_t cbor_size_le = (uint16_t)cbor_data_size;
    memcpy(packet_ptr, &cbor_size_le, sizeof(uint16_t));
    packet_ptr += sizeof(uint16_t);

    // Copy CBOR data
    memcpy(packet_ptr, cbor_buffer, cbor_data_size);
    packet_ptr += cbor_data_size;

    // Write 4-byte CRC32 (LITTLE ENDIAN)
    uint32_t hash_le = current_hash;
    memcpy(packet_ptr, &hash_le, sizeof(uint32_t));

    // Set final packet data and CRC
    free(cbor_buffer);
    current_chunk->size_in_bytes = total_packet_size;
    current_chunk->data = packet_buffer;
    current_chunk->crc32 = current_hash;
    current_chunk->chunk_id = current_chunk_id;
    return SERIALIZATION_OK;
}

// Generate header with device info and transmission metadata
// Fixed generate_cbor_header function - using little endian
SerializationResult generate_cbor_header(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size, bool ack_requested)
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
        if (pindata[i].event_mask != 0 || pindata[i].connection_index > 0)
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


    // 0. ACK REQUESTED (Key 8)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, ACK_REQUESTED);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, 1); // Assume ACK requested for this example
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

    // 6. Number of seen devices (Key 5)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_NUMBER_SEEN_DEVICES);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, seen_devices_count);
    write_ptr += bytes_written;

    // 7. List of seen device IDs (Key 6)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_SEEN_DEVICE_IDS);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, seen_devices_count);
    write_ptr += bytes_written;
    if (seen_devices_count > 0 && seen_devices != NULL)
    {
        for (uint8_t i = 0; i < seen_devices_count; i++)
        {
            bytes_written = cb0r_write(write_ptr, CB0R_INT, seen_devices[i]);
            write_ptr += bytes_written;
        }
    }
    else
    {
        // No seen devices
        bytes_written = cb0r_write(write_ptr, CB0R_ARRAY, 0);
        write_ptr += bytes_written;
    }

    // Calculate CRC32 for header integrity over CBOR data
    size_t header_data_size = write_ptr - cbor_buffer;
    current_header_hash = crcFast((uint8_t const *)cbor_buffer, header_data_size);

    printf("CBOR header data size: %zu bytes\n", header_data_size);
    printf("CBOR header packet binary: ");
    for (size_t i = 0; i < header_data_size; i++)
    {
        for (int bit = 7; bit >= 0; bit--)
        {
            printf("%d", (cbor_buffer[i] >> bit) & 1);
        }
    }
    printf("\n");
    printf("crc32: %08" PRIx32 "\n", current_header_hash);

    // Create final packet: [2 BYTE LENGTH][CBOR BYTES][4 BYTE CRC32]
    size_t total_packet_size = 2 + header_data_size + 4; // Length + CBOR + CRC32
    uint8_t *packet_buffer = (uint8_t *)malloc(total_packet_size);
    if (packet_buffer == NULL)
    {
        free(cbor_buffer);
        cbor_buffer = NULL;
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    uint8_t *packet_ptr = packet_buffer;

    // Write 2-byte length (LITTLE ENDIAN) - length of CBOR data only
    uint16_t cbor_size_le = htole16((uint16_t)header_data_size);
    memcpy(packet_ptr, &cbor_size_le, sizeof(uint16_t));
    packet_ptr += sizeof(uint16_t);

    // Copy CBOR data
    memcpy(packet_ptr, cbor_buffer, header_data_size);
    packet_ptr += header_data_size;

    // Write 4-byte CRC32 (LITTLE ENDIAN)
    uint32_t hash_le = htole32(current_header_hash);
    memcpy(packet_ptr, &hash_le, sizeof(uint32_t));

    // Set final packet data and CRC
    free(cbor_buffer);
    cbor_buffer = NULL;
    output_chunk->size_in_bytes = total_packet_size;
    output_chunk->data = packet_buffer;
    output_chunk->crc32 = current_header_hash;
    output_chunk->chunk_id = 0; // Header chunk ID is 0

    return SERIALIZATION_OK;
}
