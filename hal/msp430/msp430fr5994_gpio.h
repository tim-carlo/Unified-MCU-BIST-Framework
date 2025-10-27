#ifndef MSP430FR5994_GPIO_HAL_H
#define MSP430FR5994_GPIO_HAL_H
#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif

// Macro definitions for MSP430
#define BV(pos) (1u << (pos))
#define MSP430_NUM_ABS_PINS 64U 
#define ABS_TO_PORT(abs_pin) ((uint32_t)((abs_pin) / 8)) // Port index (0-7)
#define ABS_TO_PINIDX(abs_pin) ((uint32_t)((abs_pin) % 8)) // Pin index within port (0-7)
#define ABS_BIT(abs_pin) (1ULL << (uint64_t)(abs_pin))

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS




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
void gpio_reset(uint8_t abs_pin);
uint64_t gpio_read_all_ports(void);

// Open-Drain Functions
void gpio_od_init(uint8_t abs_pin);
void gpio_od_hold_low(uint8_t abs_pin);
void gpio_od_release(uint8_t abs_pin);


#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_GPIO_HAL_H