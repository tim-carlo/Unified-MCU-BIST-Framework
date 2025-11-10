#ifndef MSP430FR5994_HELPER_H
#define MSP430FR5994_HELPER_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "printf.h"  // Custom printf implementation
#include "pindata.h" // PinData structure and helper functions
#include "pin_config.h"

#include "msp430fr5994_uart.h"

#define ABS_PIN(port, pin) ((port * 8) + pin)

// UART configuration - Fixed to UCA0 at 9600 baud
#define UART_ID 0 // Identifier for UCA0
#define BAUD_RATE 9600
#define UART_PIN_TX ABS_PIN(2, 0) // P2.0 (gpio 8)
#define UART_PIN_RX ABS_PIN(2, 1) // P2.1 (gpio 9)

// Base address of the device descriptor table
#define DEVICE_DESCRIPTOR_ADDR 0x1A00

// Clock definitions
#define SMCLK_HZ 16000000UL // SMCLK frequency: 16 MHz

#define RNG_BASE 0x01A30
#define RNG_BYTES ((volatile uint8_t *)RNG_BASE)
extern uint32_t lfsr32;
extern uint32_t lfsr31;

void mcu_init(void);

#endif // MSP430FR5994_HELPER_H