#include "serialisation.h"

#ifndef NUMBER_OF_GPIO_PINS
#define NUMBER_OF_GPIO_PINS 32
#endif

static SerializedChunk *current_chunk = NULL;
static PinData *current_pindata = NULL;
static uint32_t current_hash = 0;
static uint8_t current_pin_data_index = 0;
static uint8_t current_chunk_id = 0;
#define KEY_CHUNK_ID 0
#define KEY_NUM_ENTRIES 1
#define KEY_PINS 2
#define KEY_CRC 3
#define KEY_PIN 4
#define KEY_EVENTS 5
#define KEY_CONNECTIONS 6
#define KEY_OTHER_PIN 7
#define KEY_DEVICE_ID 8

#define HEADER_KEY_DEVICE_UUID 0
#define HEADER_KEY_DEVICE_FAMILY 1
#define HEADER_KEY_TOTAL_CHUNKS 2
#define HEADER_KEY_TOTAL_PINS 3
#define HEADER_KEY_ACTIVE_PINS 4
#define HEADER_KEY_HEADER_CRC 5

#define HEADER_VERSION 1
#define HEADER_BUFFER_SIZE 64
#define CHUNK_BUFFER_SIZE 128

InitializationResult initialize_serialization(SerializedChunk *output_chunk, PinData *pindata)
{
    if (output_chunk == NULL || pindata == NULL)
        return INITIALIZATION_ERROR;

    current_pindata = pindata;
    current_chunk = output_chunk;
    current_chunk->data = NULL;
    current_chunk->size_in_bytes = 0;
    current_pin_data_index = 0;

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
    for (uint8_t i = current_pin_data_index; i < NUMBER_OF_GPIO_PINS && entries_to_serialize < NUMBER_OF_ENTRIES_PER_CHUNK; i++)
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

    // Write CBOR map header
    uint8_t bytes_written = cb0r_write(write_ptr, CB0R_MAP, 4);
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
    for (uint8_t i = start_index; i < NUMBER_OF_GPIO_PINS && serialized_count < entries_to_serialize; i++)
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

    // Calculate and append xxHash for integrity
    size_t cbor_data_size = write_ptr - cbor_buffer;
    current_hash = XXH32(cbor_buffer, cbor_data_size, 0);

    bytes_written = cb0r_write(write_ptr, CB0R_INT, KEY_CRC);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, current_hash);
    write_ptr += bytes_written;

    current_chunk->size_in_bytes = write_ptr - cbor_buffer;
    current_chunk->data = (uint8_t *)realloc(cbor_buffer, current_chunk->size_in_bytes);
    if (current_chunk->data == NULL)
    {
        free(cbor_buffer);
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    return SERIALIZATION_OK;
}

// Send pin data over serial using packet protocol
SerializationResult send_binary_transmission(PinData *pindata, uint8_t pin_count, uint64_t device_id)
{
    if (!pindata)
        return SERIALIZATION_ERROR_NULL_POINTER;

    SerializedChunk chunk;
    SerializationResult result = initialize_serialization(&chunk, pindata);

    if (result != SERIALIZATION_OK)
        return result;

    printf("TX_START\n");
    uint8_t packet_count = 0;
    
    // Send data in chunks until all pins processed
    do
    {
        current_chunk = &chunk;
        result = serialize_next_chunk();

        if (result == SERIALIZATION_OK && chunk.data && chunk.size_in_bytes > 0)
        {
            // Output packet with size header and hex data
            printf("PKT_%02X:", packet_count++);
            printf("%02X%02X%02X%02X:",
                   (uint8_t)(chunk.size_in_bytes & 0xFF),
                   (uint8_t)((chunk.size_in_bytes >> 8) & 0xFF),
                   (uint8_t)((chunk.size_in_bytes >> 16) & 0xFF),
                   (uint8_t)((chunk.size_in_bytes >> 24) & 0xFF));
            for (size_t i = 0; i < chunk.size_in_bytes; i++)
            {
                printf("%02X", chunk.data[i]);
            }
            printf("\n");

            free(chunk.data);
            chunk.data = NULL;
            chunk.size_in_bytes = 0;
            current_chunk_id++;
        }

    } while (result == SERIALIZATION_OK && current_pin_data_index < NUMBER_OF_GPIO_PINS);

    printf("TX_END\n");
    return result;
}

// Generate header with device info and transmission metadata
SerializationResult generate_cbor_header(SerializedChunk *output_chunk, PinData *pindata, uint8_t pindata_size)
{
    if (output_chunk == NULL || pindata == NULL)
    {
        return SERIALIZATION_ERROR_NULL_POINTER;
    }

    // Buffer für Embedded deutlich kleiner!
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

    // Calculate total number of chunks needed
    uint8_t total_chunks = 0;
    if (active_pins > 0)
    {
        total_chunks = (active_pins + NUMBER_OF_ENTRIES_PER_CHUNK - 1) / NUMBER_OF_ENTRIES_PER_CHUNK;
    }

    // Buffer overflow protection
    if (HEADER_BUFFER_SIZE < (32 + family_name_len))
    {
        free(cbor_buffer);
        cbor_buffer = NULL;
        return SERIALIZATION_ERROR_BUFFER_TOO_SMALL;
    }

    // Write CBOR header map with device metadata
    bytes_written = cb0r_write(write_ptr, CB0R_MAP, 6);
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

    // Calculate and add header checksum
    size_t header_data_size = write_ptr - cbor_buffer;
    crc header_crc = crcFast(cbor_buffer, header_data_size);

    // 6. Header CRC (Key 5)
    bytes_written = cb0r_write(write_ptr, CB0R_INT, HEADER_KEY_HEADER_CRC);
    write_ptr += bytes_written;
    bytes_written = cb0r_write(write_ptr, CB0R_INT, header_crc);
    write_ptr += bytes_written;

    // Finalize chunk data
    output_chunk->size_in_bytes = write_ptr - cbor_buffer;
    output_chunk->data = (uint8_t *)realloc(cbor_buffer, output_chunk->size_in_bytes);
    if (output_chunk->data == NULL)
    {
        free(cbor_buffer);
        cbor_buffer = NULL;
        return SERIALIZATION_MEMORY_ALLOCATION_FAILED;
    }

    // Debug output
    printf("CBOR header: %zu bytes, CRC: 0x%08X\n", 
           output_chunk->size_in_bytes, header_crc);

    return SERIALIZATION_OK;
}

// Send header packet followed by all data chunks
SerializationResult send_complete_transmission_with_header(PinData *pindata, uint8_t pindata_size)
{
    if (pindata == NULL)
    {
        return SERIALIZATION_ERROR_NULL_POINTER;
    }

    SerializationResult result;
    SerializedChunk header_chunk;

    // 1. Generate and send CBOR header packet
    result = generate_cbor_header(&header_chunk, pindata, pindata_size);
    if (result != SERIALIZATION_OK)
    {
        printf("ERROR: Header generation failed with code %d\n", result);
        return result;
    }

    // Send header in machine-readable format
    printf("HEADER_PKT:");
    printf("%02X%02X%02X%02X:",
           (uint8_t)(header_chunk.size_in_bytes & 0xFF),
           (uint8_t)((header_chunk.size_in_bytes >> 8) & 0xFF),
           (uint8_t)((header_chunk.size_in_bytes >> 16) & 0xFF),
           (uint8_t)((header_chunk.size_in_bytes >> 24) & 0xFF));

    for (size_t i = 0; i < header_chunk.size_in_bytes; i++)
    {
        printf("%02X", header_chunk.data[i]);
    }
    printf("\n");

    // Free header data
    free(header_chunk.data);

    // 2. Send data packets using existing transmission function
    result = send_binary_transmission(pindata, pindata_size, get_unique_id());

    return result;
}

// Demo function showing complete transmission protocol
void example_cbor_header_transmission(void)
{
    // Create example pin data
    PinData test_pins[NUMBER_OF_GPIO_PINS];
    initialize_pin_data_array(test_pins, NUMBER_OF_GPIO_PINS);
    printf("=== Starting complete transmission with CBOR header ===\n");

    // Add some test data
    add_pin_event(test_pins, 5, PIN_INITIALLY_LOW);
    add_pin_event(test_pins, 5, HANDSHAKE_OK_INITIATOR);
    add_pin_event(test_pins, 7, HANDSHAKE_OK_RESPONDER);

    add_pin_event(test_pins, 12, DATA_HANDSHAKE_OK);
    add_pin_event(test_pins, 8, PIN_DISTURBED);
    add_pin_connection(test_pins, 8, 3, &(uint64_t){0x123456789ABCDEF0});
    add_pin_connection(test_pins, 5, 10, &(uint64_t){0x0FEDCBA987654321});

    // Test complete transmission with header
    printf("\n=== EXAMPLE: Complete transmission with CBOR header ===\n");
    SerializationResult result = send_complete_transmission_with_header(test_pins, NUMBER_OF_GPIO_PINS);

    if (result == SERIALIZATION_OK)
    {
        printf("SUCCESS: Complete transmission with CBOR header sent\n");
    }
    else
    {
        printf("ERROR: Transmission failed with code %d\n", result);
    }
}
