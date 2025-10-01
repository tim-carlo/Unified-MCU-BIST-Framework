#ifndef MSP430FR5994_HELPER_H
#define MSP430FR5994_HELPER_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "printf.h"  // Custom printf implementation
#include "stack.h"   // Stack implementation for managing GPIO states
#include "pindata.h" // PinData structure and helper functions

#define ABS_PIN(port, pin) (((port) - 1) * 8 + (pin))
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

// UART configuration - Fixed to UCA0 at 9600 baud
#define UART_ID 0 // Identifier for UCA0
#define BAUD_RATE 9600
#define UART_PIN_TX ABS_PIN(2,0) // P2.0 (gpio 8)
#define UART_PIN_RX ABS_PIN(2,1) // P2.1 (gpio 9)

// Register offsets from port base
#define PORT_IN_OFFSET 0x00
#define PORT_OUT_OFFSET 0x02
#define PORT_DIR_OFFSET 0x04
#define PORT_REN_OFFSET 0x06
#define PORT_SEL0_OFFSET 0x0A
#define PORT_SEL1_OFFSET 0x0C
#define PORT_SEL2_OFFSET 0x0E 
#define PORT_SEL3_OFFSET 0x10 
#define PORT_SEL4_OFFSET 0x12 
#define PORT_SEL5_OFFSET 0x14 
#define PORT_SEL6_OFFSET 0x16 
#define PORT_SEL7_OFFSET 0x18 
#define PORT_SEL8_OFFSET 0x1A 

// Base address of the device descriptor table
#define DEVICE_DESCRIPTOR_ADDR 0x1A00

// Clock definitions
#define SMCLK_HZ        16000000UL  // SMCLK frequency: 16 MHz

#define RNG_BASE    0x01A30
#define RNG_BYTES   ((volatile uint8_t *)RNG_BASE)
extern uint32_t lfsr32;
extern uint32_t lfsr31;                                                                  


#ifdef __cplusplus
extern "C"
{
#endif

    void io_init(void);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_HELPER_H