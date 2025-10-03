#include "msp430fr5994_uart.h"
#include "msp430fr5994_gpio.h"
#include <string.h>
#include "endian.h"

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
 * @brief Calculate baud rate values for MSP430 according to TI documentation
 * 
 * @param baud_rate Desired baud rate
 * @param br0 Pointer to store BR0 value
 * @param br1 Pointer to store BR1 value  
 * @param mctlw Pointer to store MCTLW value
 */
static void calculate_baud_rate(unsigned long baud_rate, uint16_t *br0, uint16_t *br1, uint16_t *mctlw) {
    // Assuming 16MHz SMCLK from helper initialization
    const uint32_t smclk_freq = 16000000UL;
    uint32_t n = smclk_freq / baud_rate;
    
    if (n >= 16) {
        // Use oversampling mode (UCOS16 = 1)
        uint16_t br_value = n / 16;
        *br0 = br_value & 0xFF;
        *br1 = (br_value >> 8) & 0xFF;
        
        // Calculate fractional part for modulation
        uint32_t fractional = n - (br_value * 16);
        
        // Set UCOS16 bit and modulation based on fractional part
        *mctlw = UCOS16;
        
        // UCBRFx field (bits 7-4) for fractional modulation in oversampling mode
        if (fractional >= 1) *mctlw |= (fractional << 4) & 0x00F0;
        
        // For common baud rates, use optimized modulation patterns
        if (baud_rate == 9600 && smclk_freq == 16000000UL) {
            // N = 104.1667, UCBRx = 104, UCBRFx = 2, UCBRSx = 0xD6
            *mctlw = UCOS16 | (2 << 4) | 0x00D6;
        } else if (baud_rate == 115200 && smclk_freq == 16000000UL) {
            // N = 8.6806, UCBRx = 8, UCBRFx = 10, UCBRSx = 0xF7
            *br0 = 8;
            *br1 = 0;
            *mctlw = UCOS16 | (10 << 4) | 0x00F7;
        }
    } else {
        // Use low-frequency mode (UCOS16 = 0)
        *br0 = n & 0xFF;
        *br1 = (n >> 8) & 0xFF;
        
        // Calculate modulation for fractional part
        uint32_t fractional_x256 = ((smclk_freq % baud_rate) * 256) / baud_rate;
        uint8_t ucbrsx = 0;
        
        // Simple modulation pattern based on fractional part
        if (fractional_x256 >= 128) ucbrsx |= 0x80;
        if (fractional_x256 >= 64) ucbrsx |= 0x40;
        if (fractional_x256 >= 32) ucbrsx |= 0x20;
        if (fractional_x256 >= 16) ucbrsx |= 0x10;
        
        *mctlw = ucbrsx;
    }
}

/**
 * @brief Initialize UART peripheral according to TI recommended sequence
 * 
 * @param uart UART instance
 * @param baud_rate Baud rate
 * @param pins Pin configuration structure
 */
void uart_init(uart_instance_t *uart, const unsigned long baud_rate, const uart_pins_t* pins) {
    if (uart == NULL || pins == NULL) return;
    
    // Step 1: Set UCSWRST (BIT.B #UCSWRST,&UCAxCTL1)
    *(uart->CTLW0) = UCSWRST;
    
    // Step 2: Initialize all eUSCI_A registers with UCSWRST = 1 (including UCAxCTL1)
    *(uart->CTLW0) |= UCSSEL__SMCLK;  // Select SMCLK as clock source
    
    // Configure ports (Step 3)
    volatile uint8_t *port_sel0 = (volatile uint8_t *)pins->port_sel0;
    volatile uint8_t *port_sel1 = (volatile uint8_t *)pins->port_sel1;
    uint8_t tx_mask = pins->tx_pin_mask;
    uint8_t rx_mask = pins->rx_pin_mask;
    if (port_sel0 && port_sel1) {
        // Set pins to UART function (secondary function)
        *port_sel0 &= ~(tx_mask | rx_mask);  // Clear PxSEL0
        *port_sel1 |= (tx_mask | rx_mask);   // Set PxSEL1
    }
    
    // Step 4: Clear UCSWRST through software (BIC.B #UCSWRST,&UCAxCTL1)
    uint16_t br0, br1, mctlw;
    calculate_baud_rate(baud_rate, &br0, &br1, &mctlw);
    *(uart->BR0) = br0;
    *(uart->BR1) = br1;
    *(uart->MCTLW) = mctlw;
    
    // Enable glitch suppression for better receive reliability
    *(uart->CTLW0) &= ~UCSWRST;  // Release from reset
    
    // Step 5: Enable interrupts (optional) through UCRXxIE and/or UCTXxIE
    // This is done later in uart_set_receive_mode if needed
    
    // Clear any pending flags
    *(uart->IFG) &= ~(uart->rx_flag_bit | uart->tx_flag_bit);
}

/**
 * @brief Check if data is ready to be read
 * 
 * @param uart UART instance
 * @return true if data is ready, false otherwise
 */
bool uart_data_ready(uart_instance_t *uart) {
    if (uart == NULL) return false;
    
    // Primary check: UCRXIFG flag in interrupt flag register
    if ((*(uart->IFG) & uart->rx_flag_bit) != 0) {
        return true;
    }
    
    // Secondary check: Status register (some MSP430 variants)
    // Note: STATW might not have UCRXIFG, so we check for receive errors that indicate activity
    if (*(uart->STATW) & (UCFE | UCOE | UCPE)) {
        // Clear error by reading RXBUF
        volatile uint16_t dummy = *(uart->RXBUF);
        (void)dummy;
        return false; // Error occurred, no valid data
    }
    
    return false;
}

/**
 * @brief Check if TX is idle
 * 
 * @param uart UART instance
 * @return true if TX is idle, false otherwise
 */
bool uart_tx_idle(uart_instance_t *uart) {
    if (uart == NULL) return true;
    return (*(uart->IFG) & uart->tx_flag_bit) != 0;
}

/**
 * @brief Read a byte from UART with proper error handling
 * 
 * @param uart UART instance
 * @return received byte
 */
char uart_read(uart_instance_t *uart) {
    if (uart == NULL) return 0;
    
    // Wait for receive interrupt flag to be set
    while ((*(uart->IFG) & uart->rx_flag_bit) == 0) {
        // Check for break condition (might indicate line issues)
        if (*(uart->STATW) & UCBRK) {
            // Break detected - clear by reading RXBUF
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
            // Reset break flag
            *(uart->STATW) &= ~UCBRK;
        }
        
        // Check for other errors
        if (*(uart->STATW) & (UCFE | UCOE | UCPE)) {
            // Clear error flags by reading RXBUF
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
            return 0; // Return null character on error
        }
    }
    
    // Check one more time for errors before reading valid data
    if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK)) {
        volatile uint16_t dummy = *(uart->RXBUF);
        (void)dummy;
        return 0;
    }
    
    // Read data (this automatically clears UCRXIFG)
    // The glitch suppression in hardware should have filtered out spurious start bits
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
void uart_read_text(uart_instance_t *uart, char *output, char *delimiter, char attempts) {
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
void uart_write(uart_instance_t *uart, char data_) {
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
void uart_write_text(uart_instance_t *uart, char *uart_text) {
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
void uart_write_bytes(uart_instance_t *uart, const uint8_t* data, size_t length) {
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
void uart_write_uint32(uart_instance_t *uart, uint32_t value) {
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
void uart_set_receive_mode(uart_instance_t *uart, bool enable) {
    if (uart == NULL) return;
    
    if (enable) {
        // Make sure UART is not in reset state first
        *(uart->CTLW0) &= ~UCSWRST;
        
        // Clear any pending receive flags and errors
        *(uart->IFG) &= ~uart->rx_flag_bit;
        
        // Clear any error flags by reading RXBUF if needed
        if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK)) {
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
        }
        
        // Enable receive interrupt if desired (optional for polling)
        // *(uart->IE) |= uart->rx_flag_bit;
    } else {
        // Disable receive interrupt
        *(uart->IE) &= ~uart->rx_flag_bit;
        
        // Clear any pending receive flags
        *(uart->IFG) &= ~uart->rx_flag_bit;
    }
}

/**
 * @brief Non-blocking read function for debugging
 * 
 * @param uart UART instance
 * @param data Pointer to store received byte
 * @return true if data was read, false if no data available
 */
bool uart_read_nonblocking(uart_instance_t *uart, char *data) {
    if (uart == NULL || data == NULL) return false;
    
    // Check if data is available
    if ((*(uart->IFG) & uart->rx_flag_bit) != 0) {
        // Check for errors first
        if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK)) {
            // Clear error by reading RXBUF
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
            *data = 0;
            return false;
        }
        
        *data = (char)(*(uart->RXBUF) & 0xFF);
        return true;
    }
    
    return false;
}

/**
 * @brief Get UART status for debugging
 * 
 * @param uart UART instance
 * @return Status word value
 */
uint16_t uart_get_status(uart_instance_t *uart) {
    if (uart == NULL) return 0;
    return *(uart->STATW);
}

/**
 * @brief Check for specific UART errors
 * 
 * @param uart UART instance
 * @return Error flags (UCFE | UCOE | UCPE | UCBRK)
 */
uint16_t uart_get_errors(uart_instance_t *uart) {
    if (uart == NULL) return 0;
    return *(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK);
}

/**
 * @brief Clear UART error flags
 * 
 * @param uart UART instance
 */
void uart_clear_errors(uart_instance_t *uart) {
    if (uart == NULL) return;
    
    // Reading RXBUF clears most error flags
    if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK)) {
        volatile uint16_t dummy = *(uart->RXBUF);
        (void)dummy;
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
