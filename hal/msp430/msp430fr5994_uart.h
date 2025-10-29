#ifndef MSP430FR5994_UART_H
#define MSP430FR5994_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <msp430.h>

// UART instance type
typedef struct
{
    volatile uint16_t *CTLW0; // Control Word Register 0
    volatile uint16_t *BR0;   // Baud Rate 0
    volatile uint16_t *BR1;   // Baud Rate 1
    volatile uint16_t *MCTLW; // Modulation Control
    volatile uint16_t *STATW; // Status Word
    volatile uint16_t *RXBUF; // Receive Buffer
    volatile uint16_t *TXBUF; // Transmit Buffer
    volatile uint16_t *IFG;   // Interrupt Flag
    volatile uint16_t *IE;    // Interrupt Enable
    uint8_t rx_flag_bit;      // RX interrupt flag bit
    uint8_t tx_flag_bit;      // TX interrupt flag bit
} uart_instance_t;

typedef struct
{
    volatile uint16_t *tx_sel0;
    volatile uint16_t *tx_sel1;
    uint16_t tx_mask;

    volatile uint16_t *rx_sel0;
    volatile uint16_t *rx_sel1;
    uint16_t rx_mask;
} uart_pins_t;

// Predefined UART instances
extern uart_instance_t msp430_uart0_instance;
extern uart_instance_t msp430_uart1_instance;
extern uart_instance_t msp430_uart2_instance;
extern uart_instance_t msp430_uart3_instance;

#define MSP430_UART0 (&msp430_uart0_instance)
#define MSP430_UART1 (&msp430_uart1_instance)
#define MSP430_UART2 (&msp430_uart2_instance)
#define MSP430_UART3 (&msp430_uart3_instance)

// Basic UART functions
void uart_init(uart_instance_t *uart, const uint32_t baud_rate, const uart_pins_t *pins);
bool uart_data_ready(uart_instance_t *uart);
bool uart_tx_idle(uart_instance_t *uart);
uint8_t uart_read(uart_instance_t *uart);
void uart_read_text(uart_instance_t *uart, uint8_t *output, uint8_t *delimiter, uint8_t attempts);
void uart_write(uart_instance_t *uart, uint8_t data_);
void uart_write_text(uart_instance_t *uart, uint8_t *uart_text);
void uart_write_bytes(uart_instance_t *uart, const uint8_t *data, size_t length);
void uart_write_uint32(uart_instance_t *uart, uint32_t value);

// UART mode control functions
void uart_set_receive_mode(uart_instance_t *uart, bool enable);

// Non-blocking read function for debugging
bool uart_read_nonblocking(uart_instance_t *uart, uint8_t *data);

// UART status and error handling functions
uint16_t uart_get_status(uart_instance_t *uart);
uint16_t uart_get_errors(uart_instance_t *uart);
void uart_clear_errors(uart_instance_t *uart);

// Helper function to create pin configuration from absolute pin numbers
uart_pins_t create_uart_pins(uint8_t abs_tx_pin, uint8_t abs_rx_pin);

#endif // MSP430FR5994_UART_H
