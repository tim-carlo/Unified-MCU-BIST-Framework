#include <stdint.h>
#include <stdbool.h>
#include "msp430fr5994_gpio.h"

typedef enum
{
    GPIO_PULL_DISABLED,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} gpio_pull_t;

static volatile uint64_t s_blacklist_mask = 0;
static gpio_interrupt_handler_t s_falling = NULL;
static gpio_interrupt_handler_t s_rising = NULL;
static volatile uint64_t s_prev_state = 0;

static inline uint8_t abs_to_port(uint32_t abs_pin)
{
    return abs_pin / 8; // Port number is abs_pin / 8
}

static volatile uint8_t *get_dir_register(uint8_t port)
{
    return (volatile uint8_t *)(P1_BASE + PORT_DIR_OFFSET + port * 0x100);
}

static volatile uint8_t *get_out_register(uint8_t port)
{
    return (volatile uint8_t *)(P1_BASE + PORT_OUT_OFFSET + port * 0x100);
}

static volatile uint8_t *get_ren_register(uint8_t port)
{
    return (volatile uint8_t *)(P1_BASE + PORT_REN_OFFSET + port * 0x100);
}

static volatile uint8_t *get_in_register(uint8_t port)
{
    return (volatile uint8_t *)(P1_BASE + PORT_IN_OFFSET + port * 0x100);
}

void gpio_output_init(uint32_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *dir_reg = get_dir_register(port);
    *dir_reg |= (1 << pin); // Set pin as output
}

void gpio_input_init(uint32_t abs_pin, gpio_pull_t pull)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *dir_reg = get_dir_register(port);
    volatile uint8_t *ren_reg = get_ren_register(port);
    volatile uint8_t *out_reg = get_out_register(port);

    *dir_reg &= ~(1 << pin); // Set pin as input

    // Pull-up Settings
    if (pull == GPIO_PULL_UP)
    {
        *ren_reg |= (1 << pin);
        *out_reg |= (1 << pin);
    }
    else if (pull == GPIO_PULL_DOWN)
    {
        *ren_reg |= (1 << pin);
        *out_reg &= ~(1 << pin);
    }
    else
    {
        *ren_reg &= ~(1 << pin);
    }
}

void gpio_drive_high(uint32_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *out_reg = get_out_register(port);
    *out_reg |= (1 << pin);
}

void gpio_drive_low(uint32_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *out_reg = get_out_register(port);
    *out_reg &= ~(1 << pin);
}

bool gpio_read(uint32_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *in_reg = get_in_register(port);
    return (*in_reg & (1 << pin)) ? true : false;
}

uint64_t gpio_read_all_pins_state()
{
    uint64_t state = 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P1IN) << 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P2IN) << 8;
    state |= (uint64_t)(*(volatile uint8_t *)&P3IN) << 16;
    state |= (uint64_t)(*(volatile uint8_t *)&P4IN) << 24;
    state |= (uint64_t)(*(volatile uint8_t *)&P5IN) << 32;
    state |= (uint64_t)(*(volatile uint8_t *)&P6IN) << 40;
    state |= (uint64_t)(*(volatile uint8_t *)&P7IN) << 48;
    state |= (uint64_t)(*(volatile uint8_t *)&P8IN) << 56;
    return state;
}
bool is_interupt_blacklisted(uint8_t abs_pin)
{
    return (gpio_blacklist_intern_mask >> abs_pin) & 1;
}
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    s_blacklist_mask = blacklist_mask;
    s_falling = falling_handler;
    s_rising = rising_handler;

    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Skip blacklisted pins

        // Initialize as input with pull-up (or modify as needed)
        gpio_input_init(abs_pin, GPIO_PULL_UP);

        uint8_t port = abs_to_port(abs_pin);
        uint8_t pin = abs_to_pinidx(abs_pin);

        volatile uint8_t *ifg = (volatile uint8_t *)(P1_BASE + PORT_IFG_OFFSET + port * 0x100);
        volatile uint8_t *ies = (volatile uint8_t *)(P1_BASE + PORT_IES_OFFSET + port * 0x100);
        volatile uint8_t *ie = (volatile uint8_t *)(P1_BASE + PORT_IE_OFFSET + port * 0x100);

        *ifg &= ~(1 << pin); // Clear Interrupt Flag
        *ies |= (1 << pin);  // Falling Edge (default)
        *ie |= (1 << pin);   // Enable Interrupt
    }

    __enable_interrupt(); // Global Interrupt Enable
}

// Macro to define an Interrupt Service Routine (ISR) for a given GPIO port
#define DEFINE_PORT_ISR(port)                                                                         \
    void __attribute__((interrupt(PORT##port##_VECTOR))) PORT##port##_ISR(void)                       \
    {                                                                                                 \
        volatile uint8_t *ifg = (volatile uint8_t *)(P1_BASE + PORT_IFG_OFFSET + (port - 1) * 0x100); \
        volatile uint8_t *ie = (volatile uint8_t *)(P1_BASE + PORT_IE_OFFSET + (port - 1) * 0x100);   \
        volatile uint8_t *ies = (volatile uint8_t *)(P1_BASE + PORT_IES_OFFSET + (port - 1) * 0x100); \
        uint8_t flags = *ifg & *ie;                                                                   \
        for (uint8_t pin = 0; pin < 8; pin++)                                                         \
        {                                                                                             \
            if (flags & (1 << pin))                                                                   \
            {                                                                                         \
                uint8_t abs_pin = ((port - 1) << 3) | pin;                                            \
                if ((s_blacklist_mask >> abs_pin) & 1)                                                \
                    continue;                                                                         \
                bool is_falling = (*ies >> pin) & 1;                                                  \
                if (is_falling)                                                                       \
                {                                                                                     \
                    if (s_falling)                                                                    \
                        s_falling(abs_pin);                                                           \
                    *ies &= ~(1 << pin); /* Switch to rising edge */                                  \
                }                                                                                     \
                else                                                                                  \
                {                                                                                     \
                    if (s_rising)                                                                     \
                        s_rising(abs_pin);                                                            \
                    *ies |= (1 << pin); /* Switch to falling edge */                                  \
                }                                                                                     \
                *ifg &= ~(1 << pin); /* Clear interrupt flag */                                       \
            }                                                                                         \
        }                                                                                             \
    }

// Port ISR Definitions
DEFINE_PORT_ISR(1)
DEFINE_PORT_ISR(2)
DEFINE_PORT_ISR(3)
DEFINE_PORT_ISR(4)
DEFINE_PORT_ISR(5)
DEFINE_PORT_ISR(6)
DEFINE_PORT_ISR(7)
DEFINE_PORT_ISR(8)

/**
 * @brief Release GPIO pin from open-drain state (set as input) using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void release_gpio_open_drain(uint8_t abs_pin)
{
    gpio_pullup_init(abs_pin); // Set pin as input with pull-up resistor
    gpio_input_init(abs_pin);  // Set pin as input
}

void gpio_open_drain_drive(uint8_t abs_pin)
{
    gpio_output_init(abs_pin); // Set pin as output
    gpio_drive_low(abs_pin);   // Drive pin low
}