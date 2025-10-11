#ifndef GPIO_HAL_H
#define GPIO_HAL_H

#include "nrf.h"
#include "nrf52840.h"
#include <stdint.h>
#include <stdbool.h>
#include "stack.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define BV(pos) (1u << (pos))
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)

#define NRF52_NUM_ABS_PINS 48U // Number of absolute GPIO pins available on NRF52840
#define ABS_TO_PORT(abs) ((uint32_t)((abs) < 32 ? 0 : 1))
#define ABS_TO_PINIDX(abs) ((uint32_t)((abs) & 31U))
#define ABS_BIT(abs) (1ULL << (uint64_t)(abs))

    typedef void (*gpio_interrupt_handler_t)(uint8_t abs_pin); // ← uint8_t statt uint32_t

    typedef enum
    {
        GPIO_PULL_NONE = 0,
        GPIO_PULL_DOWN = 1,
        GPIO_PULL_UP = 2,
        GPIO_NO_PULL = 3
    } gpio_pull_t;

    // Configuration for GPIO pins
    void gpio_output_init(uint8_t abs_pin);
    void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull);
    void gpio_pullup_init(uint8_t abs_pin);
    void gpio_pulldown_init(uint8_t abs_pin);

    // Functions to drive GPIO pins
    void gpio_drive_high(uint8_t abs_pin);
    void gpio_drive_low(uint8_t abs_pin);
    void gpio_toggle(uint8_t abs_pin);
    bool gpio_read(uint8_t abs_pin);
    void gpio_reset(uint8_t abs_pin);
    uint64_t gpio_read_all_ports(void);

    // Interrupt handling
    void gpio_listen_on_all_pins_interrupt(
        uint64_t blacklist_mask,
        gpio_interrupt_handler_t falling_handler,
        gpio_interrupt_handler_t rising_handler);

    // Open-Drain Functions
    void gpio_od_init(uint8_t abs_pin);
    void gpio_od_hold_low(uint8_t abs_pin);
    void gpio_od_release(uint8_t abs_pin);

    void gpio_disable_all_interrupts(uint64_t blacklist_mask);
    void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask);

#ifdef __cplusplus
}
#endif

#endif