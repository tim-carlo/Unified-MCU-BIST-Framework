#include <string.h>
#include "endian.h"
#include "msp430fr5994_helper.h"

// Predefined UART instance definitions
uart_instance_t msp430_uart0_instance = {
    .CTLW0 = &UCA0CTLW0,
    .BRW = &UCA0BRW,
    .MCTLW = &UCA0MCTLW,
    .STATW = &UCA0STATW,
    .RXBUF = &UCA0RXBUF,
    .TXBUF = &UCA0TXBUF,
    .IE = &UCA0IE,
    .IFG = &UCA0IFG,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG};

uart_instance_t msp430_uart1_instance = {
    .CTLW0 = &UCA1CTLW0,
    .BRW = &UCA1BRW,
    .MCTLW = &UCA1MCTLW,
    .STATW = &UCA1STATW,
    .RXBUF = &UCA1RXBUF,
    .TXBUF = &UCA1TXBUF,
    .IE = &UCA1IE,
    .IFG = &UCA1IFG,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG};

uart_instance_t msp430_uart2_instance = {
    .CTLW0 = &UCA2CTLW0,
    .BRW = &UCA2BRW,
    .MCTLW = &UCA2MCTLW,
    .STATW = &UCA2STATW,
    .RXBUF = &UCA2RXBUF,
    .TXBUF = &UCA2TXBUF,
    .IE = &UCA2IE,
    .IFG = &UCA2IFG,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG};

uart_instance_t msp430_uart3_instance = {
    .CTLW0 = &UCA3CTLW0,
    .BRW = &UCA3BRW,
    .MCTLW = &UCA3MCTLW,
    .STATW = &UCA3STATW,
    .RXBUF = &UCA3RXBUF,
    .TXBUF = &UCA3TXBUF,
    .IE = &UCA3IE,
    .IFG = &UCA3IFG,
    .rx_flag_bit = UCRXIFG,
    .tx_flag_bit = UCTXIFG};

/**
 * @brief Calculate baud rate values for MSP430 according to TI documentation
 * In this function for 16MHz SMCLK for simplicity.
 * Look at SLAU367 for more details.
 *
 * @param baud_rate Desired baud rate
 * @param br0 Pointer to store BR0 value
 * @param br1 Pointer to store BR1 value
 * @param mctlw Pointer to store MCTLW value
 */
static void calculate_baud_rate(uint16_t baud_rate, uint16_t *br0, uint16_t *br1, uint16_t *mctlw)
{
    switch (baud_rate)
    {
    case 9600:
        *br0 = 104;
        *br1 = 0;
        *mctlw = UCOS16 | 0x4900; // UCBRSx=0x49, UCBRFx=2
        break;

    case 19200:
        *br0 = 52;
        *br1 = 0;
        *mctlw = UCOS16 | 0x4900;
        break;

    case 38400:
        *br0 = 26;
        *br1 = 0;
        *mctlw = UCOS16 | 0xB600;
        break;

    case 57600:
        *br0 = 17;
        *br1 = 0;
        *mctlw = UCOS16 | 0xF700;
        break;

    case 115200:
        *br0 = 8;
        *br1 = 0;
        *mctlw = UCOS16 | 0xF700;
        break;

    case 230400:
        *br0 = 4;
        *br1 = 0;
        *mctlw = UCOS16 | 0x5500;
        break;

    case 460800:
        *br0 = 2;
        *br1 = 0;
        *mctlw = UCOS16 | 0xD600;
        break;

    default:
        // Fallback to 9600 baud
        *br0 = 104;
        *br1 = 0;
        *mctlw = UCOS16 | 0x4900;
        break;
    }
}
/**
 * @brief Initialize UART peripheral according to TI recommended sequence
 *
 * @param uart UART instance
 * @param baud_rate Baud rate (unused, hardcoded to 9600)
 * @param pins Pin configuration structure (unused, using P2.0/P2.1)
 */
void uart_init(uart_instance_t *uart, const uint32_t baud_rate, const uart_pins_t *pins)
{
    if (uart == NULL)
        return;

    UCA0CTLW0 = UCSWRST;         // Reset UART
    UCA0CTLW0 |= UCSSEL__SMCLK;  // SMCLK source (16MHz)
    UCA0BR0 = 104;               // 16MHz/9600 = 1666.67
    UCA0BR1 = 0;                 // High byte
    UCA0MCTLW = UCOS16 | 0x4900; // Oversampling + fractional tuning

    P2SEL0 &= ~(BIT0 | BIT1); // Clear P2.0/P2.1 SEL0
    P2SEL1 |= BIT0 | BIT1;    // Set UART function

    UCA0CTLW0 &= ~UCSWRST; // Release from reset
}

/**
 * @brief Check if data is ready to be read
 *
 * @param uart UART instance
 * @return true if data is ready, false otherwise
 */
bool uart_data_ready(uart_instance_t *uart)
{
    if (uart == NULL)
        return false;

    // Primary check: UCRXIFG flag in interrupt flag register
    if ((*(uart->IFG) & uart->rx_flag_bit) != 0)
    {
        return true;
    }

    if (*(uart->STATW) & (UCFE | UCOE | UCPE))
    {
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
bool uart_tx_idle(uart_instance_t *uart)
{
    if (uart == NULL)
        return true;
    return (*(uart->IFG) & uart->tx_flag_bit) != 0;
}

/**
 * @brief Read a byte from UART with proper error handling
 *
 * @param uart UART instance
 * @return received byte
 */
uint8_t uart_read(uart_instance_t *uart)
{
    if (uart == NULL)
        return 0;

    // Wait for receive interrupt flag to be set
    while ((*(uart->IFG) & uart->rx_flag_bit) == 0)
    {
        // Check for break condition (might indicate line issues)
        if (*(uart->STATW) & UCBRK)
        {
            // Break detected - clear by reading RXBUF
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
            // Reset break flag
            *(uart->STATW) &= ~UCBRK;
        }

        // Check for other errors
        if (*(uart->STATW) & (UCFE | UCOE | UCPE))
        {
            // Clear error flags by reading RXBUF
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
            return 0; // Return null character on error
        }
    }

    // Check one more time for errors before reading valid data
    if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK))
    {
        volatile uint16_t dummy = *(uart->RXBUF);
        (void)dummy;
        return 0;
    }

    // Read data (this automatically clears UCRXIFG)
    // The glitch suppression in hardware should have filtered out spurious start bits
    return (uint8_t)(*(uart->RXBUF) & 0xFF);
}

/**
 * @brief Read text from UART until delimiter or attempts reached
 *
 * @param uart UART instance
 * @param output Output buffer
 * @param delimiter Delimiter string
 * @param attempts Number of attempts (255 for infinite)
 */
void uart_read_text(uart_instance_t *uart, char *output, char *delimiter, uint8_t attempts)
{
    if (uart == NULL || output == NULL || delimiter == NULL)
        return;

    uint8_t received_char;
    char *output_ptr = output;
    size_t delimiter_len = strlen(delimiter);
    size_t match_count = 0;
    uint8_t attempt_count = 0;

    *output_ptr = '\0'; // Initialize output as empty string

    while (attempt_count < attempts || attempts == 255)
    {
        if (uart_data_ready(uart))
        {
            received_char = uart_read(uart);
            *output_ptr++ = (char)received_char;

            // Check for delimiter match
            if ((char)received_char == delimiter[match_count])
            {
                match_count++;
                if (match_count == delimiter_len)
                {
                    // Delimiter found, terminate string (remove delimiter from output)
                    output_ptr -= delimiter_len;
                    *output_ptr = '\0';
                    return;
                }
            }
            else
            {
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
void uart_write(uart_instance_t *uart, char data_)
{
    if (uart == NULL)
        return;

    // Wait for transmit buffer to be ready
    while (!uart_tx_idle(uart))
    {
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
void uart_write_text(uart_instance_t *uart, char *uart_text)
{
    if (uart == NULL || uart_text == NULL)
        return;

    while (*uart_text)
    {
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
void uart_write_bytes(uart_instance_t *uart, const uint8_t *data, size_t length)
{
    if (uart == NULL || data == NULL || length == 0)
        return;

    for (size_t i = 0; i < length; i++)
    {
        uart_write(uart, (char)data[i]);
    }
}

/**
 * @brief Write a 32-bit value as raw bytes to UART (LITTLE ENDIAN)
 *
 * @param uart UART instance
 * @param value 32-bit value to send (little-endian)
 */
void uart_write_uint32(uart_instance_t *uart, uint32_t value)
{
    if (uart == NULL)
        return;

    uint32_t value_le = htole32(value);
    uart_write_bytes(uart, (const uint8_t *)&value_le, sizeof(uint32_t));
}

/**
 * @brief Set UART in receive mode
 *
 * @param uart UART instance
 * @param enable true to enable receive mode, false to disable
 */
void uart_set_receive_mode(uart_instance_t *uart, bool enable)
{
    if (uart == NULL)
        return;

    if (enable)
    {
        // Make sure UART is not in reset state first
        *(uart->CTLW0) &= ~UCSWRST;

        // Clear any pending receive flags and errors
        *(uart->IFG) &= ~uart->rx_flag_bit;

        // Clear any error flags by reading RXBUF if needed
        if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK))
        {
            volatile uint16_t dummy = *(uart->RXBUF);
            (void)dummy;
        }
    }
    else
    {
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
bool uart_read_nonblocking(uart_instance_t *uart, char *data)
{
    if (uart == NULL || data == NULL)
        return false;

    // Check if data is available
    if ((*(uart->IFG) & uart->rx_flag_bit) != 0)
    {
        // Check for errors first
        if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK))
        {
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
uint16_t uart_get_status(uart_instance_t *uart)
{
    if (uart == NULL)
        return 0;
    return *(uart->STATW);
}

/**
 * @brief Check for specific UART errors
 *
 * @param uart UART instance
 * @return Error flags (UCFE | UCOE | UCPE | UCBRK)
 */
uint16_t uart_get_errors(uart_instance_t *uart)
{
    if (uart == NULL)
        return 0;
    return *(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK);
}

/**
 * @brief Clear UART error flags
 *
 * @param uart UART instance
 */
void uart_clear_errors(uart_instance_t *uart)
{
    if (uart == NULL)
        return;

    // Reading RXBUF clears most error flags
    if (*(uart->STATW) & (UCFE | UCOE | UCPE | UCBRK))
    {
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
uart_pins_t create_uart_pins(uint8_t abs_tx_pin, uint8_t abs_rx_pin)
{
    uart_pins_t pins = {0};

    uint8_t tx_port = ABS_TO_PORT(abs_tx_pin);
    uint8_t tx_idx = ABS_TO_PINIDX(abs_tx_pin);
    uint8_t rx_port = ABS_TO_PORT(abs_rx_pin);
    uint8_t rx_idx = ABS_TO_PINIDX(abs_rx_pin);

    pins.tx_sel0 = (volatile uint16_t *)(uintptr_t)(0x0200 + tx_port * 0x20 + PORT_SEL0_OFFSET);
    pins.tx_sel1 = (volatile uint16_t *)(uintptr_t)(0x0200 + tx_port * 0x20 + PORT_SEL1_OFFSET);
    pins.tx_mask = (uint8_t)(1U << tx_idx);

    pins.rx_sel0 = (volatile uint16_t *)(uintptr_t)(0x0200 + rx_port * 0x20 + PORT_SEL0_OFFSET);
    pins.rx_sel1 = (volatile uint16_t *)(uintptr_t)(0x0200 + rx_port * 0x20 + PORT_SEL1_OFFSET);
    pins.rx_mask = (uint8_t)(1U << rx_idx);

    return pins;
}
