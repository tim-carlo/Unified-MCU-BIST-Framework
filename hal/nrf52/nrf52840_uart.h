#ifndef NRF52840_UART_H
#define NRF52840_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf.h"
#include "nrf52840.h"

// UART instance type
typedef NRF_UART_Type* uart_instance_t;

typedef struct {
    uint8_t nrf_tx_pin;
    uint8_t nrf_rx_pin;
} uart_pins_t;

// Basic UART functions
void uart_init(uart_instance_t uart, const unsigned long baud_rate, const uart_pins_t* pins);
bool uart_data_ready(uart_instance_t uart);
bool uart_tx_idle(uart_instance_t uart);
uint8_t uart_read(uart_instance_t uart);
void uart_read_text(uart_instance_t uart, char *output, char *delimiter, uint8_t attempts);
void uart_write(uart_instance_t uart, char data_);
void uart_write_text(uart_instance_t uart, char *uart_text);
void uart_write_bytes(uart_instance_t uart, const uint8_t* data, size_t length);
void uart_write_uint32(uart_instance_t uart, uint32_t value);

// UART mode control functions
void uart_set_receive_mode(uart_instance_t uart, bool enable);

// Helper function to create pin configuration from absolute pin numbers
uart_pins_t create_uart_pins(uint8_t abs_tx_pin, uint8_t abs_rx_pin);


#endif // NRF52840_UART_H 