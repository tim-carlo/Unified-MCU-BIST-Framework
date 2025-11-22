#include "uart_transmitter.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "pin_config.h"

#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)

#if defined(NRF52840_XXAA)
#include "endian.h"
static uart_instance_t tx_uart = NULL;
#elif defined(__MSP430FR5994__)
#include "endian.h"
static uart_instance_t *tx_uart = NULL;
#endif

// Global UART instance for transmission

/**
 * @brief Initialize UART transmitter with platform-specific setup
 */
void uart_transmitter_init(void)
{
#if defined(NRF52840_XXAA)
    tx_uart = NRF_UART0;
#elif defined(__MSP430FR5994__)
#if DEV_KIT == 1
    tx_uart = MSP430_UART0;
#elif DEV_KIT == 0
    tx_uart = MSP430_UART1;
#endif
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
    LOG("W\n");
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
        LOG("DEBUG: ACK timeout - only received %d bytes\n", ack_index);
        return WAIT_FOR_ACK_TIMEOUT;
    }

    // Parse LITTLE-ENDIAN values using endian conversion
    uint32_t le_start_id, le_received_hash, le_end_id;
    memcpy(&le_start_id, &ack_bytes[0], sizeof(le_start_id));
    memcpy(&le_received_hash, &ack_bytes[4], sizeof(le_received_hash));
    memcpy(&le_end_id, &ack_bytes[8], sizeof(le_end_id));

    uint32_t start_id = le32toh(le_start_id);
    uint32_t received_hash = le32toh(le_received_hash);
    uint32_t end_id = le32toh(le_end_id);

    if (start_id != ACK_START_IDENTIFIER)
    {
        return WAIT_FOR_ACK_NO_START_IDENTIFIER;
    }
    if (end_id != ACK_END_IDENTIFIER)
    {
        return WAIT_FOR_ACK_NO_END_IDENTIFIER;
    }
    if (received_hash != expected_hash)
    {
        LOG("DEBUG: wrong hash\n");
        return WAIT_FOR_ACK_INVALID_HASH;
    }

    delay_ms(1000); // Small delay to ensure UART stability
    LOG("DEBUG: ACK OKE\n");
    return WAIT_FOR_ACK_OK;
}

/**
 * @brief Send error identifier with 4-byte error code (enum value extended to 32-bit) in LITTLE ENDIAN
 * @param error_code Enum value to send as 32-bit error code
 */
static void send_error_with_code(uint32_t error_code)
{
    uart_write_uint32(tx_uart, ERROR_IDENTIFIER);
    uart_write_uint32(tx_uart, error_code);
}

// // Combined transmission function with acknowledgement checking
// UartTransmissionResult send_complete_transmission_with_ack(PinData *pindata, uint8_t pindata_size)
// {
//     if (pindata == NULL)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_NULL_POINTER);
//         return UART_TRANSMISSION_ERROR_NULL_POINTER;
//     }

//     // Initialize UART if not already done
//     if (tx_uart == NULL)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_INIT_FAILED);
//         return UART_TRANSMISSION_ERROR_INIT_FAILED;
//     }

//     SerializationResult serialization_result;
//     WaitForAckResult ack_result;
//     SerializedChunk header_chunk;
//     uint8_t ack_buffer[4];

//     // 1. Generate and send CBOR header packet
//     serialization_result = generate_cbor_header(&header_chunk, pindata, pindata_size, true);
//     if (serialization_result != SERIALIZATION_OK)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
//         return UART_TRANSMISSION_ERROR_SEND_FAILED;
//     }

//     uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
//     if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
//     {
//         // Send complete header packet: [2 bytes length (LE)][CBOR][4 bytes CRC32 (LE)]
//         uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
//     }
//     uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);

//     // Use CRC from SerializedChunk structure, not from packet bytes
//     uint32_t expected_hash = header_chunk.crc32;

//     // Wait for header acknowledgement with retry logic
//     uint8_t header_retry_count = 0;
//     do
//     {
//         if (header_retry_count > 0)
//         {
//             // Resend header packet
//             uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
//             if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
//             {
//                 uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
//             }
//             uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);
//         }

//         ack_result = wait_for_ack(expected_hash, ack_buffer);
//         header_retry_count++;

//     } while (ack_result != WAIT_FOR_ACK_OK && header_retry_count <= MAX_RETRIES);

//     if (ack_result != WAIT_FOR_ACK_OK)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_ACK_FAILED);
//         free(header_chunk.data);
//         return UART_TRANSMISSION_ERROR_ACK_FAILED;
//     }

//     free(header_chunk.data);

//     // 2. Send data chunks with individual acknowledgements
//     SerializedChunk chunk;
//     serialization_result = initialize_serialization(&chunk, pindata, pindata_size, true);

//     if (serialization_result != SERIALIZATION_OK)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
//         return UART_TRANSMISSION_ERROR_SEND_FAILED;
//     }

//     // Send transmission start identifier
//     uart_write_uint32(tx_uart, TRANSMISSION_START_IDENTIFIER);
//     uint8_t packet_count = 0;

//     // Send data in chunks until all pins processed
//     do
//     {
//         current_chunk = &chunk;
//         serialization_result = serialize_next_chunk();

//         if (serialization_result == SERIALIZATION_OK && chunk.data && chunk.size_in_bytes > 0)
//         {
//             // Send chunk with identifiers
//             uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);

//             uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);

//             uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);

//             // Use CRC from SerializedChunk structure, not from packet bytes
//             uint32_t chunk_expected_hash = chunk.crc32;

//             // Wait for chunk acknowledgement with retry logic
//             uint8_t chunk_retry_count = 0;
//             do
//             {
//                 if (chunk_retry_count > 0)
//                 {
//                     uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);
//                     uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);
//                     uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);
//                 }

//                 ack_result = wait_for_ack(chunk_expected_hash, ack_buffer);
//                 chunk_retry_count++;

//             } while (ack_result != WAIT_FOR_ACK_OK && chunk_retry_count <= MAX_RETRIES);

//             if (ack_result != WAIT_FOR_ACK_OK)
//             {
//                 send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_ACK_FAILED);
//                 free(chunk.data);
//                 return UART_TRANSMISSION_ERROR_ACK_FAILED;
//             }

//             free(chunk.data);
//             chunk.data = NULL;
//             chunk.size_in_bytes = 0;
//             chunk.crc32 = 0;
//             current_chunk_id++;
//             packet_count++;
//         }
//         else
//         {
//             LOG("DEBUG: No more data to send or serialization failed. Result=%d, data=%p, size=%zu\n",
//                 serialization_result, (void *)chunk.data, chunk.size_in_bytes);
//         }

//     } while (serialization_result == SERIALIZATION_OK && current_pin_data_index < pindata_size);

//     // Send transmission end identifier
//     uart_write_uint32(tx_uart, TRANSMISSION_END_IDENTIFIER);
//     uart_write_text(tx_uart, "END_TRANSMISSION\n");

//     return UART_TRANSMISSION_OK;
// }

// UartTransmissionResult send_complete_transmission_no_ack(PinData *pindata, uint8_t pindata_size)
// {
//     if (pindata == NULL)
//     {
//         return UART_TRANSMISSION_ERROR_NULL_POINTER;
//     }

//     // Initialize UART if not already done
//     if (tx_uart == NULL)
//     {
//         return UART_TRANSMISSION_ERROR_INIT_FAILED;
//     }

//     SerializationResult serialization_result;
//     SerializedChunk header_chunk;

//     // 1. Generate and send CBOR header packet
//     serialization_result = generate_cbor_header(&header_chunk, pindata, pindata_size, false);
//     if (serialization_result != SERIALIZATION_OK)
//     {
//         return UART_TRANSMISSION_ERROR_INIT_FAILED;
//     }

//     uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
//     if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
//     {
//         // Send complete header packet: [2 bytes length (LE)][CBOR][4 bytes CRC32 (LE)]
//         uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
//     }
//     uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);

//     free(header_chunk.data);

//     // Short delay to ensure header is processed before data chunks
//     delay_ms(100);

//     // 2. Send data chunks without acknowledgements
//     SerializedChunk chunk;
//     serialization_result = initialize_serialization(&chunk, pindata, pindata_size, false);

//     if (serialization_result != SERIALIZATION_OK)
//     {
//         send_error_with_code((uint32_t)UART_TRANSMISSION_ERROR_SEND_FAILED);
//         return UART_TRANSMISSION_ERROR_INIT_FAILED;
//     }

//     // Send transmission start identifier
//     uart_write_uint32(tx_uart, TRANSMISSION_START_IDENTIFIER);

//     // Send data in chunks until all pins processed
//     do
//     {
//         current_chunk = &chunk;
//         serialization_result = serialize_next_chunk();

//         if (serialization_result == SERIALIZATION_OK && chunk.data && chunk.size_in_bytes > 0)
//         {
//             // Send chunk with identifiers
//             uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);
//             uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);
//             uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);

//             free(chunk.data);
//             chunk.data = NULL;
//             chunk.size_in_bytes = 0;
//             chunk.crc32 = 0;
//             current_chunk_id++;
//             // Short delay to ensure chunk is processed before next chunk
//             delay_ms(200);
//         }
//         else
//         {
//             LOG("DEBUG: No more data to send or serialization failed. Result=%d, data=%p, size=%zu\n",
//                 serialization_result, (void *)chunk.data, chunk.size_in_bytes);
//         }
//     } while (serialization_result == SERIALIZATION_OK && current_pin_data_index < pindata_size);
//     // Send transmission end identifier
//     uart_write_uint32(tx_uart, TRANSMISSION_END_IDENTIFIER);
//     return UART_TRANSMISSION_OK;
// }

/**
 * @brief Sends the global header ONCE at the start.
 * 
 * Protocol: 
 * [HEADER_START] 
 * [TOTAL_SESSIONS (uint32)] 
 * [HEADER_CBOR_DATA] 
 * [HEADER_END]
 * 
 * @param total_expected_sessions How many sessions (iterations) the client should expect.
 */
UartTransmissionResult uart_send_header_info(PinData *pindata, uint8_t pindata_size, uint8_t total_expected_sessions)
{
    if (pindata == NULL || tx_uart == NULL) return UART_TRANSMISSION_ERROR_INIT_FAILED;

    SerializationResult serialization_result;
    SerializedChunk header_chunk;

    // Generate CBOR header data (Device info, total chunks, etc. are inside here)
    serialization_result = generate_cbor_header(&header_chunk, pindata, pindata_size, false, total_expected_sessions);
    if (serialization_result != SERIALIZATION_OK) return UART_TRANSMISSION_ERROR_SEND_FAILED;

    uart_write_uint32(tx_uart, HEADER_START_IDENTIFIER);
    
    if (header_chunk.data != NULL && header_chunk.size_in_bytes > 0)
    {
        uart_write_bytes(tx_uart, header_chunk.data, header_chunk.size_in_bytes);
    }
    uart_write_uint32(tx_uart, HEADER_END_IDENTIFIER);

    free(header_chunk.data);
    return UART_TRANSMISSION_OK;
}

/**
 * @brief Sends ALL pin data (all chunks) for a specific session ID.
 * @param session_id The current session number (e.g., 1, 2, 3...)
 */
UartTransmissionResult uart_send_session_data(PinData *pindata, uint8_t pindata_size, uint8_t session_id)
{
    if (pindata == NULL || tx_uart == NULL) return UART_TRANSMISSION_ERROR_INIT_FAILED;

    SerializationResult serialization_result;
    SerializedChunk chunk;
    uint8_t current_chunk_id = 0;

    // Initialize serialization to start from the first pin
    // We pass session_id as stream_number so it's also inside the CBOR data
    serialization_result = initialize_serialization(&chunk, pindata, pindata_size, false, session_id);
    if (serialization_result != SERIALIZATION_OK) return UART_TRANSMISSION_ERROR_SEND_FAILED;

    uart_write_uint32(tx_uart, TRANSMISSION_START_IDENTIFIER);
    // Loop through ALL chunks for this session
    do
    {
        current_chunk = &chunk;
        serialization_result = serialize_next_chunk(current_chunk_id);

        if (serialization_result == SERIALIZATION_OK && chunk.data != NULL && chunk.size_in_bytes > 0)
        {
            // Send this specific chunk WITHOUT waiting for ACK
            uart_write_uint32(tx_uart, CHUNCK_START_IDENTIFIER);

            uart_write_bytes(tx_uart, chunk.data, chunk.size_in_bytes);
            uart_write_uint32(tx_uart, CHUNCK_END_IDENTIFIER);

            free(chunk.data); // Important: Free memory after sending
            current_chunk_id++;

            // Short delay to ensure chunk is processed before next chunk
            delay_ms(200);
        }
        else
        {
            // No more data or empty chunk
            if(chunk.data) free(chunk.data);
            break;
        }

    } while (serialization_result == SERIALIZATION_OK && current_pin_data_index < pindata_size);

    uart_write_uint32(tx_uart, TRANSMISSION_END_IDENTIFIER);

    return UART_TRANSMISSION_OK;
}

