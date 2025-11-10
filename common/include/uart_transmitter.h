#ifndef UART_TRANSMITTER_H
#define UART_TRANSMITTER_H

#include "serialisation.h"

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#include "nrf52840_utils.h"
#include "nrf52840_uart.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_uart.h"
#endif

#define MAX_TIMEOUT (500000)
#define MAX_RETRIES (3)

#define CHUNCK_START_IDENTIFIER (0x01020304)
#define CHUNCK_END_IDENTIFIER (0x05060708)
#define HEADER_START_IDENTIFIER (0x090A0B0C)
#define HEADER_END_IDENTIFIER (0x0D0E0F10)
#define TRANSMISSION_START_IDENTIFIER (0x11121314)
#define TRANSMISSION_END_IDENTIFIER (0x15161718)
#define ACK_START_IDENTIFIER (0x191A1B1C)
#define ACK_END_IDENTIFIER (0x1D1E1F20)

// Error identifier
#define ERROR_IDENTIFIER (0xE0E1E2E3)

typedef enum
{
    WAIT_FOR_ACK_OK = 0,
    WAIT_FOR_ACK_TIMEOUT = 1,
    WAIT_FOR_ACK_NO_START_IDENTIFIER = 2,
    WAIT_FOR_ACK_INVALID_HASH = 3,
    WAIT_FOR_ACK_NO_END_IDENTIFIER = 4,
    WAIT_FOR_ACK_NULL_POINTER = 5
} WaitForAckResult;

typedef enum
{
    UART_TRANSMISSION_OK = 0,
    UART_TRANSMISSION_ERROR_INIT_FAILED = 1,
    UART_TRANSMISSION_ERROR_SEND_FAILED = 2,
    UART_TRANSMISSION_ERROR_ACK_FAILED = 3,
    UART_TRANSMISSION_MEMORY_ALLOCATION_FAILED = 4,
    UART_TRANSMISSION_ERROR_NULL_POINTER = 5
} UartTransmissionResult;

// Function declarations
void uart_transmitter_init(void);
UartTransmissionResult send_complete_transmission_with_ack(PinData *pindata, uint8_t pindata_size);
UartTransmissionResult send_complete_transmission_no_ack(PinData *pindata, uint8_t pindata_size);

#endif // UART_TRANSMITTER_H