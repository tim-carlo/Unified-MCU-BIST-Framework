#include "msp430fr5994_gpio.h"

static volatile uint64_t s_blacklist_mask = 0;
static gpio_interrupt_handler_t s_falling = NULL;
static gpio_interrupt_handler_t s_rising = NULL;


static volatile uint8_t *const PxIE[] = {&P1IE, &P2IE, &P3IE, &P4IE, &P5IE, &P6IE, &P7IE, &P8IE};
static volatile uint8_t *const PxIFG[] = {&P1IFG, &P2IFG, &P3IFG, &P4IFG, &P5IFG, &P6IFG, &P7IFG, &P8IFG};
static volatile uint8_t *const PxIES[] = {&P1IES, &P2IES, &P3IES, &P4IES, &P5IES, &P6IES, &P7IES, &P8IES};

static inline uint8_t abs_to_pinidx(uint8_t abs_pin)
{
    return abs_pin % 8; // Pin index is abs_pin % 8
}
static inline uint8_t abs_to_port(uint8_t abs_pin)
{
    return abs_pin / 8; // Port number is abs_pin / 8
}

static volatile uint8_t *get_dir_register(uint8_t port)
{
    return (volatile uint8_t *)((uintptr_t)(P1_BASE + PORT_DIR_OFFSET + port * 0x100));
}

static volatile uint8_t *get_out_register(uint8_t port)
{
    return (volatile uint8_t *)((uintptr_t)(P1_BASE + PORT_OUT_OFFSET + port * 0x100));
}

static volatile uint8_t *get_ren_register(uint8_t port)
{
    return (volatile uint8_t *)((uintptr_t)(P1_BASE + PORT_REN_OFFSET + port * 0x100));
}

static volatile uint8_t *get_in_register(uint8_t port)
{
    return (volatile uint8_t *)((uintptr_t)(P1_BASE + PORT_IN_OFFSET + port * 0x100));
}

void gpio_output_init(uint8_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *dir_reg = get_dir_register(port);
    *dir_reg |= (1 << pin); // Set pin as output
}

void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull)
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

void gpio_drive_high(uint8_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *out_reg = get_out_register(port);
    *out_reg |= (1 << pin);
}

void gpio_drive_low(uint8_t abs_pin)
{
    uint8_t port = abs_to_port(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    volatile uint8_t *out_reg = get_out_register(port);
    *out_reg &= ~(1 << pin);
}

bool gpio_read(uint8_t abs_pin)
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
static bool is_interupt_blacklisted(uint8_t abs_pin)
{
    return (s_blacklist_mask >> abs_pin) & 1;
}
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    s_blacklist_mask = blacklist_mask;
    s_falling = falling_handler;
    s_rising = rising_handler;

    for (uint8_t abs_pin = 0; abs_pin < MSP430_NUM_ABS_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Skip blacklisted pins

        // Initialize as input with pull-up (or modify as needed)
        gpio_input_init(abs_pin, GPIO_PULL_UP);
        gpio_pullup_init(abs_pin);

        uint8_t port = abs_pin >> 3;
        uint8_t pin = abs_pin & 0x07;

        *PxIFG[port] &= ~(1 << pin); // Clear Interrupt Flag
        *PxIES[port] |= (1 << pin);  // Falling Edge (default)
        *PxIE[port] |= (1 << pin);   // Enable Interrupt
    }

    __enable_interrupt(); // Global Interrupt Enable
}

// Macro to define an Interrupt Service Routine (ISR) for a given GPIO port
// Handles both falling and rising edges by toggling the edge detection after each interrupt
#define DEFINE_PORT_ISR(port)                                                   \
    void __attribute__((interrupt(PORT##port##_VECTOR))) PORT##port##_ISR(void) \
    {                                                                           \
        /* Get only active and enabled interrupt flags for this port */         \
        uint8_t flags = P##port##IFG & P##port##IE;                             \
                                                                                \
        for (uint8_t pin = 0; pin < 8; pin++)                                   \
        {                                                                       \
            if (flags & (1 << pin))                                             \
            {                                                                   \
                /* Compute absolute pin index, e.g., P3.4 → 20 */               \
                uint8_t abs_pin = ((port - 1) << 3) | pin;                      \
                                                                                \
                /* Skip if this pin is blacklisted */                           \
                if (is_interupt_blacklisted(abs_pin))                          \
                    continue;                                                   \
                /* Check if the current edge setting is falling */              \
                bool is_falling = (P##port##IES >> pin) & 1;                    \
                                                                                \
                if (is_falling)                                                 \
                {                                                               \
                    if (s_falling)                                              \
                        s_falling(abs_pin);                                     \
                                                                                \
                    P##port##IES &= ~(1 << pin);                                \
                }                                                               \
                else                                                            \
                {                                                               \
                    /* Rising edge detected call rising edge handler */         \
                    if (s_rising)                                               \
                        s_rising(abs_pin);                                      \
                                                                                \
                    /* Switch to falling edge detection for next time */        \
                    P##port##IES |= (1 << pin);                                 \
                }                                                               \
                                                                                \
                P##port##IFG &= ~(1 << pin);                                    \
            }                                                                   \
        }                                                                       \
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
 * @brief Initialize GPIO pin in open-drain mode using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-63)
 */
void gpio_od_init(uint8_t abs_pin)
{
    gpio_input_init(abs_pin, GPIO_PULL_UP); // Set pin as input with pull-up
}

/**
 * @brief Hold GPIO pin in open-drain state (drive low) using absolute pin number
 * 
 * @param abs_pin 
 */
void gpio_od_hold_low(uint8_t abs_pin)
{
    gpio_output_init(abs_pin); // Set pin as output
    gpio_drive_low(abs_pin);   // Drive pin low
}


/**
 * @brief Release GPIO pin from open-drain state (set as input) using absolute pin number
 * And reset the sense configuration
 * @param abs_pin
 */
void gpio_od_release(uint8_t abs_pin)
{
    gpio_input_init(abs_pin, GPIO_PULL_UP); // Reset the pin to input mode
}

/**
 * @brief Push active pins to stack except those in the blacklist
 * 
 * @param stack Pointer to the stack where active pins will be pushed
 * @param expected_level Expected level of the pins (true for high, false for low)
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask) 
{
    for (uint8_t abs_pin = 0; abs_pin < MSP430_NUM_ABS_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Skip blacklisted pins
        if (gpio_read(abs_pin) == expected_level)
        {
            printf("Pushing active pin %lu to stack\n", abs_pin);
            stack_push(stack, &abs_pin);
        }
    }
}