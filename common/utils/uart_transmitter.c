#include "uart_transmitter.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#if defined(NRF52840_XXAA)
#include "endian.h"
#elif defined(__MSP430FR5994__)
#include "endian.h"
#endif

// Global UART instance for transmission
static uart_instance_t *tx_uart = NULL;

/**
 * @brief Initialize UART transmitter with platform-specific setup
 */
void uart_transmitter_init(void)
{
#if defined(NRF52840_XXAA)
    tx_uart = NRF_UART0;
    uart_pins_t uart_pins;
    uart_pins = create_uart_pins(6, 8);
    //  uart_init(tx_uart, 115200, &uart_pins);
#elif defined(__MSP430FR5994__)
    tx_uart = MSP430_UART0;
    uart_pins_t uart_pins;
    uart_pins = create_uart_pins(8, 9);
    //  uart_init(tx_uart, 115200, &uart_pins);
#endif
}
/**
 * @brief Wait for ACK with timeout
 *
 * @param expected_hash Expected hash value
 * @param ack_buffer Buffer to store received ACK
 * @return true if ACK received and matches, false otherwise
 */
static WaitForAckResult wait_for_ack(uint32_t expected_hash, uint8_t *ack_buffer)
{
    if (ack_buffer == NULL)
        return WAIT_FOR_ACK_NULL_POINTER;
    printf("W\n");
    // Enable receive mode before waiting for ACK
    uart_set_receive_mode(tx_uart, true);

    uint8_t ack_bytes[12] = {0};
    uint8_t ack_index = 0;
    uint32_t timeout_counter = 0;

    // Wait for 12 bytes: 4-byte START, 4-byte ACK, 4-byte END
    while (ack_index < 12 && timeout_counter < MAX_TIMEOUT)
    {
        if (uart_data_ready(tx_uart))
        {
            ack_bytes[ack_index++] = uart_read(tx_uart);
            timeout_counter = 0;
        }
        else
        {
            timeout_counter++;
        }
    }

    // Disable receive mode before returning
    uart_set_receive_mode(tx_uart, false);

    if (ack_index < 12)
    {
        printf("ACK timeout - only received %d bytes\n", ack_index);
        return WAIT_FOR_ACK_TIMEOUT;
    }

    // Parse big-endian values using endian conversion
    uint32_t be_start_id, be_received_hash, be_end_id;
    memcpy(&be_start_id, &ack_bytes[0], sizeof(be_start_id));
    memcpy(&be_received_hash, &ack_bytes[4], sizeof(be_received_hash));
    memcpy(&be_end_id, &ack_bytes[8], sizeof(be_end_id));
    
    uint32_t start_id = be32toh(be_start_id);
    uint32_t received_hash = be32toh(be_received_hash);
    uint32_t end_id = be32toh(be_end_id);
    if (start_id != ACK_START_IDENTIFIER)
    {
        printf("ACK START identifier mismatch! Expected: 0x%08lX, Got: 0x%08lX\n", (unsigned long)ACK_START_IDENTIFIER, (unsigned long)start_id);
        return WAIT_FOR_ACK_NO_START_IDENTIFIER;
    }
    if (end_id != ACK_END_IDENTIFIER)
    {
        printf("ACK END identifier mismatch! Expected: 0x%08lX, Got: 0x%08lX\n", (unsigned long)ACK_END_IDENTIFIER, (unsigned long)end_id);
        return WAIT_FOR_ACK_NO_END_IDENTIFIER;
    }
    if (received_hash != expected_hash)
    {
        printf("ACK value mismatch! Expected: 0x%08lX, Got: 0x%08lX\n", (unsigned long)expected_hash, (unsigned long)received_hash);
        return WAIT_FOR_ACK_INVALID_HASH;
    }

    printf("ACK frame valid!\n");
    return WAIT_FOR_ACK_OK;
}

/**
 * @brief Send error identifier with 4-byte error code (enum value extended to 32-bit)
 * @param error_code Enum value to send as 32-bit error code
 */
static void send_error_with_code(uint32_t error_code)
{
    uart_write_uint32(tx_uart, ERROR_IDENTIFIER);
    uart_write_uint32(tx_uart, error_code);
}

// Combined transmission function with acknowledgement checking
UartTransmissionResult send_complete_transmission_with_ack(PinData *pindata, uint8_t pindata_size)
{
    if (pindata == NULL)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_NULL_POINTER);
        return UART_TRANSMISSION_ERROR_NULL_POINTER;
    }

    // Initialize UART if not already done
    if (tx_uart == NULL)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_INIT_FAILED);
        return UART_TRANSMISSION_ERROR_INIT_FAILED;
    }

    SerializationResult serialization_result;
    WaitForAckResult ack_result;
    SerializedChunk header_chunk;
    uint32_t expected_hash;
    uint8_t ack_buffer[4];

    // 1. Generate and send CBOR header packet
    serialization_result = generate_cbor_header(&header_chunk, pindata, pindata_size);
    if (serialization_result != SERIALIZATION_OK)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
        return UART_TRANSMISSION_ERROR_SEND_FAILED;
    }

    uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
    if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
    {
        uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
    }
    uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);

    // Extract expected hash from header (last 4 bytes, big-endian)
    if (header_chunk.size_in_bytes < 4)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
        free(header_chunk.data);
        return UART_TRANSMISSION_ERROR_SEND_FAILED;
    }

    uint8_t *hash_ptr = header_chunk.data + header_chunk.size_in_bytes - 4;
    uint32_t be_expected_hash;
    memcpy(&be_expected_hash, hash_ptr, sizeof(be_expected_hash));
    expected_hash = be32toh(be_expected_hash);

    // Wait for header acknowledgement with retry logic
    uint8_t header_retry_count = 0;
    do
    {
        if (header_retry_count > 0)
        {
            printf("HEADER: Retry attempt %d\n", header_retry_count);
            // Resend header packet
            uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
            if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
            {
                uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
            }
            uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);
        }

        ack_result = wait_for_ack(expected_hash, ack_buffer);
        header_retry_count++;

    } while (ack_result != WAIT_FOR_ACK_OK && header_retry_count <= MAX_RETRIES);

    if (ack_result != WAIT_FOR_ACK_OK)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_ACK_FAILED);
        free(header_chunk.data);
        return UART_TRANSMISSION_ERROR_ACK_FAILED;
    }

    free(header_chunk.data);

    // 2. Send data packets with individual acknowledgements
    SerializedChunk chunk;
    serialization_result = initialize_serialization(&chunk, pindata, pindata_size);

    if (serialization_result != SERIALIZATION_OK)
    {
        send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
        uart_write_text(tx_uart, "END_TRANSMISSION\n");
        return UART_TRANSMISSION_ERROR_SEND_FAILED;
    }

    // Send transmission start identifier
    uart_write_uint32(tx_uart, TRANSMISSION_START_IDENTIFIER);
    uint8_t packet_count = 0;

    // Send data in chunks until all pins processed
    do
    {
        printf("DEBUG: Starting chunk %d, current_pin_data_index=%d, NUMBER_OF_GPIO_PINS=%d\n", packet_count, current_pin_data_index, NUMBER_OF_GPIO_PINS);
        
        current_chunk = &chunk;
        serialization_result = serialize_next_chunk();
        
        printf("DEBUG: After serialize_next_chunk, result=%d, chunk.data=%p, chunk.size_in_bytes=%zu\n", 
               serialization_result, (void*)chunk.data, chunk.size_in_bytes);

        if (serialization_result == SERIALIZATION_OK && chunk.data && chunk.size_in_bytes > 0)
        {
            // Send chunk with format: CHUNCK_START_IDENTIFIER + 2 bytes CHUNK ID + CHUCK + CHUNCK_END_IDENTIFIER
            uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);

            // Send 2-byte chunk ID (big-endian)
            uint16_t be_packet_count = htobe16(packet_count);
            uint8_t id_bytes[2];
            memcpy(id_bytes, &be_packet_count, sizeof(id_bytes));
            uart_write_bytes(tx_uart, id_bytes, 2);

            // Send complete packet [2 byte length][CBOR][4 byte hash]
            if (chunk.data != NULL && chunk.size_in_bytes > 0)
            {
                uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);
            }
            uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);

            // Extract expected hash from packet (last 4 bytes, big-endian)
            if (chunk.size_in_bytes < 4)
            {
                send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
                free(chunk.data);
                uart_write_text(tx_uart, "END_TRANSMISSION\n");
                return UART_TRANSMISSION_ERROR_SEND_FAILED;
            }

            uint8_t *hash_ptr = chunk.data + chunk.size_in_bytes - 4;
            uint32_t be_chunk_expected_hash;
            memcpy(&be_chunk_expected_hash, hash_ptr, sizeof(be_chunk_expected_hash));
            uint32_t chunk_expected_hash = be32toh(be_chunk_expected_hash);

            // Wait for packet acknowledgement with retry logic
            uint8_t chunk_retry_count = 0;
            do
            {
                if (chunk_retry_count > 0)
                {
                    printf("CHUNK %d: Retry attempt %d\n", packet_count, chunk_retry_count);
                    // Resend chunk packet
                    uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);

                    // Send 2-byte chunk ID (big-endian)
                    uint16_t be_packet_count = htobe16(packet_count);
                    uint8_t id_bytes[2];
                    memcpy(id_bytes, &be_packet_count, sizeof(id_bytes));
                    uart_write_bytes(tx_uart, id_bytes, 2);

                    // Send complete packet [2 byte length][CBOR][4 byte hash]
                    if (chunk.data != NULL && chunk.size_in_bytes > 0)
                    {
                        uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);
                    }
                    uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);
                }

                ack_result = wait_for_ack(chunk_expected_hash, ack_buffer);
                chunk_retry_count++;

            } while (ack_result != WAIT_FOR_ACK_OK && chunk_retry_count <= MAX_RETRIES);

            if (ack_result != WAIT_FOR_ACK_OK)
            {
                send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_ACK_FAILED);
                free(chunk.data);
                uart_write_text(tx_uart, "END_TRANSMISSION\n");
                return UART_TRANSMISSION_ERROR_ACK_FAILED;
            }

            free(chunk.data);
            chunk.data = NULL;
            chunk.size_in_bytes = 0;
            current_chunk_id++;
            packet_count++;
            
            printf("DEBUG: Completed chunk %d, current_pin_data_index now=%d\n", packet_count-1, current_pin_data_index);
        }
        else
        {
            printf("DEBUG: No more data to send or serialization failed. Result=%d, data=%p, size=%zu\n", 
                   serialization_result, (void*)chunk.data, chunk.size_in_bytes);
        }

    } while (serialization_result == SERIALIZATION_OK && current_pin_data_index < pindata_size);

    // Send transmission end identifier
    uart_write_uint32(tx_uart, TRANSMISSION_END_IDENTIFIER);
    uart_write_text(tx_uart, "END_TRANSMISSION\n");

    return UART_TRANSMISSION_OK;
}

// Demo function showing complete transmission protocol with acknowledgement
void example_cbor_header_transmission_with_ack(void)
{
    // Initialize UART transmitter
    uart_transmitter_init();

    // Create example pin data
    PinData test_pins[32]; // Use fixed size instead of NUMBER_OF_GPIO_PINS
    initialize_pin_data_array(test_pins, 32);
    // Add some test data
    add_pin_event(test_pins, 5, PIN_INITIALLY_LOW);
    add_pin_event(test_pins, 5, HANDSHAKE_OK_INITIATOR);
    add_pin_event(test_pins, 7, HANDSHAKE_OK_RESPONDER);

    add_pin_event(test_pins, 12, DATA_HANDSHAKE_OK);
    add_pin_event(test_pins, 8, PIN_DISTURBED);
    add_pin_connection(test_pins, 8, 3, 0);
    add_pin_connection(test_pins, 5, 10, 0);

    // Test complete transmission with header and acknowledgement checking
    UartTransmissionResult result = send_complete_transmission_with_ack(test_pins, 32);

    printf("DEBUG: Transmission completed with result: %d\n", result);

    switch (result)
    {
    case UART_TRANSMISSION_OK:
        uart_write_text(tx_uart, "SUCCESS: Complete transmission with CBOR header and ACK completed\n");
        break;
    case UART_TRANSMISSION_ERROR_INIT_FAILED:
        uart_write_text(tx_uart, "ERROR: UART initialization failed\n");
        break;
    case UART_TRANSMISSION_ERROR_SEND_FAILED:
        uart_write_text(tx_uart, "ERROR: Data transmission failed\n");
        break;
    case UART_TRANSMISSION_ERROR_ACK_FAILED:
        uart_write_text(tx_uart, "ERROR: Acknowledgement failed\n");
        break;
    case UART_TRANSMISSION_MEMORY_ALLOCATION_FAILED:
        uart_write_text(tx_uart, "ERROR: Memory allocation failed\n");
        break;
    case UART_TRANSMISSION_ERROR_NULL_POINTER:
        uart_write_text(tx_uart, "ERROR: Null pointer provided\n");
        break;
    default:
        uart_write_text(tx_uart, "ERROR: Unknown error occurred\n");
        break;
    }
}