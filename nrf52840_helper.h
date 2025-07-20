
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

typedef void (*gpio_interrupt_handler_t)(uint32_t gpio);

#define MAX_GPIO_INTERRUPT_HANDLERS 4

typedef struct {
    uint32_t pin;         // Pin number (0-47)
    NRF_GPIO_Type* PORT; // Pointer to the GPIO port (NRF_P0 or NRF_P1)
    uint64_t timestamp;
    bool pin_state;
} pin_time_measurement_t;


// Function prototypes
void io_init(void);
uint32_t get_timer_counter(void);
void gpio_pullup_init(uint32_t abs_pin);
void gpio_pulldown_init(uint32_t abs_pin);
void gpio_pullup_clear(uint32_t abs_pin);
void gpio_drive_low(uint32_t abs_pin);
void gpio_drive_high(uint32_t abs_pin);
void gpio_input_init(uint32_t abs_pin);
void gpio_output_init(uint32_t abs_pin);
bool gpio_read(uint32_t abs_pin);
void push_active_pins_to_stack(Stack *stack, uint8_t level);
void push_active_pins_except_to_stack(Stack *stack, uint32_t exclude_abs_pin);
void gpio_reset(uint32_t abs_pin);
void gpio_open_drain_abs(uint32_t abs_pin);
void log_pin_state(uint32_t abs_pin, uint64_t current_time, bool pin_state);
void clear_time_measurements(void);
void clear_time_measurement(uint32_t abs_pin);
pin_time_measurement_t* get_measurement(uint32_t abs_pin);
bool is_interupt_blacklisted(uint32_t abs_pin);
void gpio_listen_interrupt_on_all_pins(uint64_t blacklist_mask, gpio_interrupt_handler_t rising_handler, gpio_interrupt_handler_t falling_handler);
void GPIOTE_IRQHandler(void);
void release_gpio_open_drain(uint32_t abs_pin);
uint32_t get_elapsed_time(uint32_t start, uint32_t current);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
bool wait_for_signal_abs(uint32_t abs_pin, bool level, uint32_t timeout_us);
bool is_signal_active(uint32_t pin, bool assert_high);
uint64_t get_timer_ticks(void);
void start_timer(void);
void stop_timer(void);
void reset_timer(void);
uint32_t ticks_to_us(uint64_t ticks);
uint32_t ticks_to_ms(uint64_t ticks);
uint32_t timer_diff_us(uint64_t start, uint64_t end);
uint32_t timer_diff_ms(uint64_t start, uint64_t end);
uint32_t random32_lfsr(void);
uint32_t random32(void);
uint32_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint32_t length);
void init_software_serial(uint32_t abs_pin, uint32_t baudrate);
void software_serial_tx(uint8_t byte);
uint64_t get_unique_id(void);
const char *get_unique_id_str(void);
const char *get_chip_family_name(void);
const uint32_t get_absolute_pin_number(NRF_GPIO_Type *PORT, uint8_t pin);


#endif // NRF52840_HELPER_H