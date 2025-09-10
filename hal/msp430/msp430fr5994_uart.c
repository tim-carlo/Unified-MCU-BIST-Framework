#include "msp430fr5994_uart.h"
#include <string.h>

// Predefined UART instance definitions
uart_instance_t msp430_uart0_instance = {
    .CTLW0 = &UCA0CTLW0,
    .BR0 = &UCA0BR0,
    .BR1 = &UCA0BR1,
    .MCTLW = &UCA0MCTLW,
    .STATW = &UCA0STATW,
    .RXBUF = &UCA0RXBUF,
    .TXBUF = &UCA0TXBUF,
    .IFG = &UCA0IFG,
    .IE = &UCA0IE,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG
};

uart_instance_t msp430_uart1_instance = {
    .CTLW0 = &UCA1CTLW0,
    .BR0 = &UCA1BR0,
    .BR1 = &UCA1BR1,
    .MCTLW = &UCA1MCTLW,
    .STATW = &UCA1STATW,
    .RXBUF = &UCA1RXBUF,
    .TXBUF = &UCA1TXBUF,
    .IFG = &UCA1IFG,
    .IE = &UCA1IE,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG
};

uart_instance_t msp430_uart2_instance = {
    .CTLW0 = &UCA2CTLW0,
    .BR0 = &UCA2BR0,
    .BR1 = &UCA2BR1,
    .MCTLW = &UCA2MCTLW,
    .STATW = &UCA2STATW,
    .RXBUF = &UCA2RXBUF,
    .TXBUF = &UCA2TXBUF,
    .IFG = &UCA2IFG,
    .IE = &UCA2IE,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG
};

uart_instance_t msp430_uart3_instance = {
    .CTLW0 = &UCA3CTLW0,
    .BR0 = &UCA3BR0,
    .BR1 = &UCA3BR1,
    .MCTLW = &UCA3MCTLW,
    .STATW = &UCA3STATW,
    .RXBUF = &UCA3RXBUF,
    .TXBUF = &UCA3TXBUF,
    .IFG = &UCA3IFG,
    .IE = &UCA3IE,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG
};

/**
 * @brief Calculate baud rate values for MSP430
 * 
 * @param baud_rate Desired baud rate
 * @param br0 Pointer to store BR0 value
 * @param br1 Pointer to store BR1 value  
 * @param mctlw Pointer to store MCTLW value
 */
static void calculate_baud_rate(unsigned long baud_rate, uint16_t *br0, uint16_t *br1, uint16_t *mctlw) {
    // Assuming 16MHz SMCLK from helper initialization
    const uint32_t smclk_freq = 16000000UL;
    uint32_t divider = smclk_freq / baud_rate;
    
    if (divider >= 16) {
        // Use oversampling mode
        *br0 = (divider / 16) & 0xFF;
        *br1 = ((divider / 16) >> 8) & 0xFF;
        *mctlw = UCOS16;
        
        // Add modulation for fractional part
        uint32_t remainder = divider % 16;
        if (remainder >= 8) *mctlw |= 0x0080;
        if (remainder >= 4) *mctlw |= 0x0040;
        if (remainder >= 2) *mctlw |= 0x0020;
        if (remainder >= 1) *mctlw |= 0x0010;
    } else {
        // No oversampling
        *br0 = divider & 0xFF;
        *br1 = (divider >> 8) & 0xFF;
        *mctlw = 0;
    }
}

/**
 * @brief Initialize UART peripheral
 * 
 * @param uart UART instance
 * @param baud_rate Baud rate
 * @param pins Pin configuration structure
 */
void uart_init(uart_instance_t uart, const unsigned long baud_rate, const uart_pins_t* pins) {
    if (uart == NULL || pins == NULL) return;
    uint16_t br0, br1, mctlw;
    *(uart->CTLW0) = UCSWRST;
    *(uart->CTLW0) |= UCSSEL__SMCLK;
    calculate_baud_rate(baud_rate, &br0, &br1, &mctlw);
    *(uart->BR0) = br0;
    *(uart->BR1) = br1;
    *(uart->MCTLW) = mctlw;
    // Use unified uart_pins_t struct
    volatile uint8_t *port_sel0 = (volatile uint8_t *)pins->port_sel0;
    volatile uint8_t *port_sel1 = (volatile uint8_t *)pins->port_sel1;
    uint8_t tx_mask = pins->tx_pin_mask;
    uint8_t rx_mask = pins->rx_pin_mask;
    if (port_sel0 && port_sel1) {
        *port_sel0 &= ~(tx_mask | rx_mask);
        *port_sel1 |= (tx_mask | rx_mask);
    }
    *(uart->CTLW0) &= ~UCSWRST;
}

/**
 * @brief Check if data is ready to be read
 * 
 * @param uart UART instance
 * @return true if data is ready, false otherwise
 */
bool uart_data_ready(uart_instance_t uart) {
    if (uart == NULL) return false;
    return (*(uart->IFG) & uart->rx_flag_bit) != 0;
}

/**
 * @brief Check if TX is idle
 * 
 * @param uart UART instance
 * @return true if TX is idle, false otherwise
 */
bool uart_tx_idle(uart_instance_t uart) {
    if (uart == NULL) return true;
    return (*(uart->IFG) & uart->tx_flag_bit) != 0;
}

/**
 * @brief Read a byte from UART
 * 
 * @param uart UART instance
 * @return received byte
 */
char uart_read(uart_instance_t uart) {
    if (uart == NULL) return 0;
    
    // Wait for data to be ready
    while (!uart_data_ready(uart)) {
        // Wait
    }
    
    // Read and return data (reading RXBUF automatically clears the flag)
    return (char)(*(uart->RXBUF) & 0xFF);
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
    
    // Wait for transmit buffer to be ready
    while (!uart_tx_idle(uart)) {
        // Wait
    }
    
    // Send data (writing to TXBUF automatically clears the flag)
    *(uart->TXBUF) = (uint8_t)data_;
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
    
    uint8_t bytes[4] = {
        (value >> 24) & 0xFF,   // High byte first (big-endian)
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF            // Low byte last
    };
    uart_write_bytes(uart, bytes, 4);
}

/**
 * @brief Set UART in receive mode
 * 
 * @param uart UART instance
 * @param enable true to enable receive mode, false to disable
 */
void uart_set_receive_mode(uart_instance_t uart, bool enable) {
    if (uart == NULL) return;
    
    if (enable) {
        // Clear any pending receive flags
        *(uart->IFG) &= ~uart->rx_flag_bit;
        
        // Enable receive interrupt
        *(uart->IE) |= uart->rx_flag_bit;
        
        // Make sure UART is not in reset state
        *(uart->CTLW0) &= ~UCSWRST;
    } else {
        // Disable receive interrupt
        *(uart->IE) &= ~uart->rx_flag_bit;
        
        // Clear any pending receive flags
        *(uart->IFG) &= ~uart->rx_flag_bit;
    }
}


/**
 * @brief Helper function to create pin configuration from absolute pin numbers
 * 
 * @param abs_tx_pin Absolute TX pin number (0-63)
 * @param abs_rx_pin Absolute RX pin number (0-63)
 * @return uart_pins_t Pin configuration structure
 */
uart_pins_t create_uart_pins(uint8_t abs_tx_pin, uint8_t abs_rx_pin) {
    uart_pins_t pins = {0};
    // Calculate port and pin index
    uint8_t tx_port = ABS_TO_PORT(abs_tx_pin);
    uint8_t tx_idx = ABS_TO_PINIDX(abs_tx_pin);
    uint8_t rx_port = ABS_TO_PORT(abs_rx_pin);
    uint8_t rx_idx = ABS_TO_PINIDX(abs_rx_pin);

    // Port selection registers base addresses (MSP430 specific)
    volatile uint8_t *port_sel0 = (volatile uint8_t *)(0x200 + tx_port * 0x20); // PnSEL0 offset
    volatile uint8_t *port_sel1 = (volatile uint8_t *)(0x201 + tx_port * 0x20); // PnSEL1 offset

    pins.port_sel0 = port_sel0;
    pins.port_sel1 = port_sel1;
    pins.tx_pin_mask = BV(tx_idx);
    pins.rx_pin_mask = BV(rx_idx);
    return pins;
}
