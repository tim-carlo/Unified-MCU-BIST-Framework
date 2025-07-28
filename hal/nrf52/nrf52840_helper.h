
#ifndef NRF52840_HELPER_H
#define NRF52840_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf.h"
#include "nrf52840.h"
#include "stack.h"
#include "pindata.h"
#include "printf.h"

#define NUMBER_OF_GPIO_PINS 48
#define UART_PIN_TX 6
#define UART_PIN_RX 8

#define DEBOUNCE_SAMPLES 5
#define DEBOUNCE_DELAY_US 20

#define BV(pos) (1u << (pos))
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)

#define TIMER_A NRF_TIMER0
#define TIMER_B NRF_TIMER1

typedef void (*gpio_interrupt_handler_t)(uint32_t gpio);


// Function prototypes
const uint32_t get_absolute_pin_number(NRF_GPIO_Type *PORT, uint8_t pin);

void io_init(void);
void gpio_pullup_init(uint8_t abs_pin);
void gpio_pulldown_init(uint8_t abs_pin);
void gpio_pullup_clear(uint8_t abs_pin);
void gpio_drive_low(uint8_t abs_pin);
void gpio_drive_high(uint8_t abs_pin);
void gpio_input_init(uint8_t abs_pin);
void gpio_output_init(uint8_t abs_pin);
bool gpio_read(uint8_t abs_pin);
void push_active_pins_to_stack(Stack *stack, uint8_t level);
void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask);


void gpio_reset(uint8_t abs_pin);
void gpio_open_drain(uint8_t abs_pin);

bool is_interupt_blacklisted(uint8_t abs_pin);
void configure_pin_sense(uint8_t abs_pin, bool sense_low);
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask, gpio_interrupt_handler_t rising_handler, gpio_interrupt_handler_t falling_handler);


void release_gpio_open_drain(uint8_t abs_pin);
uint32_t get_elapsed_time(uint32_t start, uint32_t current);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

uint32_t get_timer_ticks(NRF_TIMER_Type *timer);
void start_timer(NRF_TIMER_Type *timer);
void stop_timer(NRF_TIMER_Type *timer);

void reset_timer(NRF_TIMER_Type *timer);
uint32_t ticks_to_us(uint64_t ticks);
uint32_t ticks_to_ms(uint64_t ticks);
uint32_t timer_diff_us(uint64_t start, uint64_t end);
uint32_t timer_diff_ms(uint64_t start, uint64_t end);
uint32_t random32_lfsr(void);
uint32_t random32(void);

uint8_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask);
void init_software_serial(uint8_t abs_pin, uint32_t baudrate);
void software_serial_tx(uint8_t byte);

uint64_t get_unique_id(void);
const char *get_unique_id_str(void);
const char *get_chip_family_name(void);



#endif // NRF52840_HELPER_H