#include "nrf52840_uart.h"
#include "nrf52840_helper.h"
#include "nrf52_bitfields.h"
#include <string.h>
#include "nrf52840_gpio.h"
#include "endian.h"

/**
 * @brief Get the nrf baudrate object
 * 
 * @param baud_rate baud rate as unsigned long
 * @return uint32_t returns the corresponding NRF baudrate constant
 */
static uint32_t get_nrf_baudrate(unsigned long baud_rate) {
    switch (baud_rate) {
        case 1200:   return UART_BAUDRATE_BAUDRATE_Baud1200;
        case 2400:   return UART_BAUDRATE_BAUDRATE_Baud2400;
        case 4800:   return UART_BAUDRATE_BAUDRATE_Baud4800;
        case 9600:   return UART_BAUDRATE_BAUDRATE_Baud9600;
        case 14400:  return UART_BAUDRATE_BAUDRATE_Baud14400;
        case 19200:  return UART_BAUDRATE_BAUDRATE_Baud19200;
        case 28800:  return UART_BAUDRATE_BAUDRATE_Baud28800;
        case 31250:  return UART_BAUDRATE_BAUDRATE_Baud31250;
        case 38400:  return UART_BAUDRATE_BAUDRATE_Baud38400;
        case 56000:  return UART_BAUDRATE_BAUDRATE_Baud56000;
        case 57600:  return UART_BAUDRATE_BAUDRATE_Baud57600;
        case 76800:  return UART_BAUDRATE_BAUDRATE_Baud76800;
        case 115200: return UART_BAUDRATE_BAUDRATE_Baud115200;
        case 230400: return UART_BAUDRATE_BAUDRATE_Baud230400;
        case 250000: return UART_BAUDRATE_BAUDRATE_Baud250000;
        case 460800: return UART_BAUDRATE_BAUDRATE_Baud460800;
        case 921600: return UART_BAUDRATE_BAUDRATE_Baud921600;
        case 1000000: return UART_BAUDRATE_BAUDRATE_Baud1M;
        default:     return UART_BAUDRATE_BAUDRATE_Baud9600; // Default to 9600
    }
}


/**
 * @brief Helper function to create pin configuration from absolute pin numbers (NRF52840)
 * 
 * @param abs_tx_pin Absolute TX pin number (0-47)
 * @param abs_rx_pin Absolute RX pin number (0-47)
 * @return uart_pins_t Pin configuration structure
 */
uart_pins_t create_uart_pins(uint8_t abs_tx_pin, uint8_t abs_rx_pin) {
    uart_pins_t pins = {0};
    pins.nrf_tx_pin = abs_tx_pin;
    pins.nrf_rx_pin = abs_rx_pin;
    return pins;
}

/**
 * @brief Initialize UART peripheral
 * 
 * @param uart UART instance
 * @param baud_rate Baud rate
 * @param tx_pin TX pin number
 * @param rx_pin RX pin number
 */
void uart_init(uart_instance_t uart, const unsigned long baud_rate, const uart_pins_t* pins) {
    if (uart == NULL || pins == NULL) return;
    uart->PSEL.TXD = pins->nrf_tx_pin;
    uart->PSEL.RXD = pins->nrf_rx_pin;
    uart->BAUDRATE = get_nrf_baudrate(baud_rate);
    uart->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) |
                   (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos);
    uart->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos;
    uart->TASKS_STARTTX = 1;
    uart->TASKS_STARTRX = 1;
}

/**
 * @brief Check if data is ready to be read
 * 
 * @param uart UART instance
 * @return if data is ready true, else false
 */
bool uart_data_ready(uart_instance_t uart) {
    if (uart == NULL) return 0;
    return (uart->EVENTS_RXDRDY == 1);
}

/**
 * @brief Check if TX is idle
 * 
 * @param uart UART instance
 * @return if TX is idle true, else false
 */
bool uart_tx_idle(uart_instance_t uart) {
    if (uart == NULL) return 1;
    return (uart->EVENTS_TXDRDY == 1);
}

/**
 * @brief Read a byte from UART
 * 
 * @param uart UART instance
 * @return char received byte
 */
char uart_read(uart_instance_t uart) {
    if (uart == NULL) return 0;
    
    // Enable the UART peripheral for receiving
//    uart->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos;
    
    // Start the UART RX task
    uart->TASKS_STARTRX = UART_TASKS_STARTRX_TASKS_STARTRX_Trigger << UART_TASKS_STARTRX_TASKS_STARTRX_Pos;
    
    // Wait for data to be ready
    while (uart->EVENTS_RXDRDY == UART_EVENTS_RXDRDY_EVENTS_RXDRDY_NotGenerated) 
    {
    }
    
    // Get received data
    char received_char = (char)uart->RXD;
    
    // Clear the event
    uart->EVENTS_RXDRDY = UART_EVENTS_RXDRDY_EVENTS_RXDRDY_NotGenerated << UART_EVENTS_RXDRDY_EVENTS_RXDRDY_Pos;
    
    // Stop the RX task
    uart->TASKS_STOPRX = UART_TASKS_STOPRX_TASKS_STOPRX_Trigger << UART_TASKS_STOPRX_TASKS_STOPRX_Pos;
    
    // Disable the UART peripheral
  //  uart->ENABLE = UART_ENABLE_ENABLE_Disabled << UART_ENABLE_ENABLE_Pos;
    
    return received_char;
}

/**
 * @brief Read text from UART until delimiter or attempts reached
 * 
 * @param uart UART instance
 * @param output Output buffer
 * @param delimiter Delimiter string
 * @param attempts Number of attempts (255 for infinite)
 */
void uart_read_text(uart_instance_t uart, char *output, char *delimiter, char attempts) {
    if (uart == NULL || output == NULL || delimiter == NULL) return;
    
    char received_char;
    char *output_ptr = output;
    char delimiter_len = strlen(delimiter);
    char match_count = 0;
    char attempt_count = 0;
    
    *output_ptr = '\0'; // Initialize output as empty string
    
    while (attempt_count < attempts || attempts == 255) {
        if (uart_data_ready(uart)) {
            received_char = uart_read(uart);
            *output_ptr++ = received_char;
            
            // Check for delimiter match
            if (received_char == delimiter[match_count]) {
                match_count++;
                if (match_count == delimiter_len) {
                    // Delimiter found, terminate string (remove delimiter from output)
                    output_ptr -= delimiter_len;
                    *output_ptr = '\0';
                    return;
                }
            } else {
                match_count = 0;
            }
            
            attempt_count++;
        }
    }
    
    // Null terminate the output
    *output_ptr = '\0';
}

/**
 * @brief Write a byte to UART
 * 
 * @param uart UART instance
 * @param data_ Byte to send
 */
void uart_write(uart_instance_t uart, char data_) {
    if (uart == NULL) return;
    
    // Write the character into the TXD register.
    uart->TXD = (uint8_t)data_;
    
    // Start the UART transfer
    uart->TASKS_STARTTX = UART_TASKS_STARTTX_TASKS_STARTTX_Trigger << UART_TASKS_STARTTX_TASKS_STARTTX_Pos;

    // Wait until the end of the transmission by checking the TXRDY event.
    while (uart->EVENTS_TXDRDY == UART_EVENTS_TXDRDY_EVENTS_TXDRDY_NotGenerated);

    // Reset the event.
    uart->EVENTS_TXDRDY = UART_EVENTS_TXDRDY_EVENTS_TXDRDY_NotGenerated << UART_EVENTS_TXDRDY_EVENTS_TXDRDY_Pos;
    
    // Stop the transmission by triggering the stop task.
    uart->TASKS_STOPTX = UART_TASKS_STOPTX_TASKS_STOPTX_Trigger << UART_TASKS_STOPTX_TASKS_STOPTX_Pos;
}

/**
 * @brief Write a text string to UART
 * 
 * @param uart UART instance
 * @param uart_text Null-terminated string to send
 */
void uart_write_text(uart_instance_t uart, char *uart_text) {
    if (uart == NULL || uart_text == NULL) return;
    
    while (*uart_text) {
        uart_write(uart, *uart_text++);
    }
}

/**
 * @brief Write raw bytes to UART (without null-termination requirement)
 * 
 * @param uart UART instance
 * @param data Pointer to data buffer
 * @param length Number of bytes to send
 */
void uart_write_bytes(uart_instance_t uart, const uint8_t* data, size_t length) {
    if (uart == NULL || data == NULL || length == 0) return;
    
    for (size_t i = 0; i < length; i++) {
        uart_write(uart, data[i]);
    }
}

/**
 * @brief Write a 32-bit value as raw bytes to UART
 * 
 * @param uart UART instance
 * @param value 32-bit value to send (big-endian)
 */
void uart_write_uint32(uart_instance_t uart, uint32_t value) {
    if (uart == NULL) return;
    
    uint32_t value_be = htobe32(value);
    uart_write_bytes(uart, (const uint8_t*)&value_be, sizeof(uint32_t));
}

/**
 * @brief Set UART in receive mode
 * 
 * @param uart UART instance
 * @param enable true to enable receive mode, false to disable
 */
void uart_set_receive_mode(NRF_UART_Type *uart, bool enable) {
    if (uart == NULL) return;
    
    if (enable) {  
        // Clear pending receive event
        uart->EVENTS_RXDRDY = 0;
        
        // Start receiver
        uart->TASKS_STARTRX = 1;
    } else {
        // Stop receiver
        uart->TASKS_STOPRX = 1;
        
        // Clear pending event
        uart->EVENTS_RXDRDY = 0;
    }
}
