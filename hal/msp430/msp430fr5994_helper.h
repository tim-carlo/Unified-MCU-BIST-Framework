#ifndef MSP430FR5994_HELPER_H
#define MSP430FR5994_HELPER_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "printf.h"  // Custom printf implementation
#include "putchar.h" // Custom putchar implementation
#include "stack.h"   // Stack implementation for managing GPIO states
#include "pindata.h" // PinData structure and helper functions

// Register offsets from port base
#define PORT_IN_OFFSET 0x00
#define PORT_OUT_OFFSET 0x02
#define PORT_DIR_OFFSET 0x04
#define PORT_REN_OFFSET 0x06
#define PORT_SEL0_OFFSET 0x0A

// UART configuration - Fixed to UCA0 at 9600 baud
#define UART_ID 0 // Identifier for UCA0
#define BAUD_RATE 9600
#define UART_TX_PIN 8 // P2.0 (gpio 8)
#define UART_RX_PIN 9 // P2.1 (gpio 9)

// Base address of the device descriptor table
#define DEVICE_DESCRIPTOR_ADDR 0x1A00

// Clock definitions
#define SMCLK_HZ        16000000UL  // SMCLK frequency: 16 MHz

// Timer configuration
#define TIMER_DIVIDER   1           // Timer clock divider (ID__1)
#define TIMER_FREQ_HZ   (SMCLK_HZ / TIMER_DIVIDER) // Effective timer frequency: 2 MHz

// In this Version we leave the PJ Port out of the GPIO handling as
// they are used for clocks and other functions!
#define NUMBER_OF_GPIO_PINS 64
typedef void (*gpio_interrupt_handler_t)(uint32_t gpio);

// Timer configuration macros
typedef enum
{
    TIMER_A0,
    TIMER_A1,
    TIMER_A4,
    TIMER_B0
} timer_t;

#define GET_TxxCTL(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CTL : (timer) == TIMER_A1 ? &TA1CTL \
                                                       : (timer) == TIMER_A4   ? &TA4CTL \
                                                                               : &TB0CTL))

#define GET_TxxR(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0R : (timer) == TIMER_A1 ? &TA1R \
                                                   : (timer) == TIMER_A4   ? &TA4R \
                                                                           : &TB0R))

#define GET_TxxCCTL0(timer) ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CCTL0 : (timer) == TIMER_A1 ? &TA1CCTL0 \
                                                           : (timer) == TIMER_A4   ? &TA4CCTL0 \
                                                                                   : &TB0CCTL0))

#define TICKS_PER_OVERFLOW 65536UL

#ifdef __cplusplus
extern "C"
{
#endif

    void io_init(void);

    void gpio_pullup_init(uint8_t abs_pin);
    void gpio_pulldown_init(uint8_t abs_pin);
    void gpio_pullup_clear(uint8_t abs_pin);
    void gpio_drive_low(uint8_t abs_pin);
    void gpio_drive_high(uint8_t abs_pin);
    void gpio_input_init(uint8_t abs_pin);
    void gpio_output_init(uint8_t abs_pin);
    bool gpio_read(uint8_t abs_pin);
    uint64_t read_all_gpio_states(void);

    void push_active_pins_to_stack(Stack *stack, uint8_t level);
    void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask);

    void gpio_reset(uint8_t abs_pin);
    void release_gpio_open_drain(uint8_t abs_pin);

    bool is_interupt_blacklisted(uint8_t abs_pin);
    void configure_pin_sense(uint8_t abs_pin, bool sense_low);
    void gpio_listen_on_all_pins_polling(uint64_t blacklist_mask, gpio_interrupt_handler_t rising_handler, gpio_interrupt_handler_t falling_handler);
    void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask, gpio_interrupt_handler_t rising_handler, gpio_interrupt_handler_t falling_handler);

    uint32_t get_elapsed_time(uint32_t start, uint32_t current);
    void delay_us(uint32_t us);
    void delay_ms(uint32_t ms);

    uint32_t get_timer_ticks(timer_t timer_r);
    void start_timer(timer_t timer);
    void stop_timer(timer_t timer);
    void reset_timer(timer_t timer);

    uint32_t ticks_to_us(uint32_t ticks);
    uint32_t ticks_to_ms(uint32_t ticks);
    uint32_t timer_diff_us(uint32_t start, uint32_t end);
    uint32_t timer_diff_ms(uint32_t start, uint32_t end);

    uint32_t random32_lfsr(void);
    uint32_t random32(void);
    uint32_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask);

    void init_software_serial(uint32_t abs_pin, uint32_t baudrate);
    void software_serial_tx(uint8_t byte);

    uint64_t get_unique_id(void);
    const char *get_unique_id_str(void);
    const char *get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_HELPER_H