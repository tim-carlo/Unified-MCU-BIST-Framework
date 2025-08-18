#ifndef MSP430FR5994_GPIO_HAL_H
#define MSP430FR5994_GPIO_HAL_H

#include "msp430fr5994.h"
#include <stdint.h>
#include <stdbool.h>
#include "printf.h"
#include "stack.h"


#ifdef __cplusplus
extern "C" {
#endif

// Macro definitions for MSP430
#define BV(pos) (1u << (pos))
#define MSP430_NUM_ABS_PINS 64U 
#define ABS_TO_PORT(abs_pin) ((uint32_t)((abs_pin) / 8)) // Port index (0-7)
#define ABS_TO_PINIDX(abs_pin) ((uint32_t)((abs_pin) % 8)) // Pin index within port (0-7)
#define ABS_BIT(abs_pin) (1ULL << (uint64_t)(abs_pin))

// Register offsets from port base
#define PORT_IN_OFFSET  0x00
#define PORT_OUT_OFFSET 0x02
#define PORT_DIR_OFFSET 0x04
#define PORT_REN_OFFSET 0x06

// Clock definitions
#define SMCLK_HZ        16000000UL  // SMCLK frequency: 16 MHz

// Timer configuration
#define TIMER_DIVIDER       8
#define TIMER_FREQ_HZ       (SMCLK_HZ / TIMER_DIVIDER) // 16MHz / 8 = 2MHz
#define TICKS_PER_OVERFLOW  65536UL  // 16-bit Timer overflow (2^16)

#define INVALID_PIN 255 // Invalid pin number

// Pull configuration
typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_DOWN = 1,
    GPIO_PULL_UP   = 2,
} gpio_pull_t;

// Interrupt handler type
typedef void (*gpio_interrupt_handler_t)(uint8_t abs_pin); 

// GPIO Configuration
void gpio_output_init(uint8_t abs_pin);
void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull);
static inline void gpio_pullup_init(uint8_t abs_pin) {
    gpio_input_init(abs_pin, GPIO_PULL_UP);
}
static inline void gpio_pulldown_init(uint8_t abs_pin) {
    gpio_input_init(abs_pin, GPIO_PULL_DOWN);
}

// GPIO Operations
void gpio_drive_high(uint8_t abs_pin);
void gpio_drive_low(uint8_t abs_pin);
void gpio_toggle(uint8_t abs_pin);
bool gpio_read(uint8_t abs_pin);
uint64_t gpio_read_all_pins_state(void);

// Open-Drain Functions
void gpio_od_hold_low(uint8_t abs_pin);
void gpio_od_release(uint8_t abs_pin);

// Interrupt Handling
void gpio_listen_on_all_pins_interrupt(
    uint64_t blacklist_mask,
    gpio_interrupt_handler_t falling_handler,
    gpio_interrupt_handler_t rising_handler);

void gpio_disable_all_interrupts(void);

// Stack Operations for GPIO pins
void push_active_pins_except_blacklist_to_stack(
    Stack* stack,
    bool expected_level,
    uint64_t blacklist_mask);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_GPIO_HAL_H