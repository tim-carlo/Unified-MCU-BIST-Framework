#include "msp430fr5994_gpio.h"
#include <stdint.h>

static volatile uint64_t s_blacklist_mask = 0;
static gpio_interrupt_handler_t s_falling = NULL;
static gpio_interrupt_handler_t s_rising = NULL;

static volatile uint8_t *const PxIE[] = {&P1IE, &P2IE, &P3IE, &P4IE, &P5IE, &P6IE, &P7IE, &P8IE};
static volatile uint8_t *const PxIFG[] = {&P1IFG, &P2IFG, &P3IFG, &P4IFG, &P5IFG, &P6IFG, &P7IFG, &P8IFG};
static volatile uint8_t *const PxIES[] = {&P1IES, &P2IES, &P3IES, &P4IES, &P5IES, &P6IES, &P7IES, &P8IES};

// In this function we ignore port J
// Port J is on this device used for special functions
static inline uintptr_t get_port_base_of_absolute_pin(uint8_t abs_pin)
{
    if (abs_pin < 8)
        return P1_BASE;
    else if (abs_pin < 16)
        return P2_BASE;
    else if (abs_pin < 24)
        return P3_BASE;
    else if (abs_pin < 32)
        return P4_BASE;
    else if (abs_pin < 40)
        return P5_BASE;
    else if (abs_pin < 48)
        return P6_BASE;
    else if (abs_pin < 56)
        return P7_BASE;
    else if (abs_pin < 64)
        return P8_BASE;
    return 0; // invalid pin
}

static inline uint8_t abs_to_pinidx(uint8_t abs_pin)
{
    return abs_pin % 8; // Pin index is abs_pin % 8
}

static inline uint8_t abs_to_port(uint8_t abs_pin)
{
    return abs_pin / 8; // Port number is abs_pin / 8
}

void gpio_output_init(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin); // ← uintptr_t statt uint16_t
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask; // output
}

void gpio_set_pull(uint8_t abs_pin, gpio_pull_t pull)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin); // ← uintptr_t statt uint16_t
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    switch (pull)
    {
    case GPIO_PULL_UP:
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask; // enable resistor
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask; // pull-up
        break;

    case GPIO_PULL_DOWN:
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;  // enable resistor
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask; // pull-down
        break;

    case GPIO_PULL_NONE:
    default:
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask; // disable resistor
        break;
    }
}

void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t pin = abs_to_pinidx(abs_pin);
    const uint8_t mask = 1u << pin;

    // Configure pin as input as described here: https://www.ocfreaks.com/msp430-gpio-programming-tutorial/
    // Set as input
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;

    switch (pull)
    {
    case GPIO_PULL_NONE:
        // Disable resistor
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
        break;

    case GPIO_PULL_DOWN:
        // Enable resistor
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
        // OUT=0
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
        break;

    case GPIO_PULL_UP:
        // Enable resistor
        *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
        // OUT=1
        *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
        break;
    }
}

void gpio_drive_high(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}

void gpio_drive_low(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}
void gpio_toggle(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) ^= mask;
}

void gpio_reset(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    
    // Reset to input mode (clear DIR bit)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;
    
    // Disable resistor enable (clear REN bit)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
    
    // Clear output bit
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}

bool gpio_read(uint8_t abs_pin)
{
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    return (*(volatile uint8_t *)((uintptr_t)(base + PORT_IN_OFFSET)) & mask) != 0;
}
 

uint64_t gpio_read_all_ports()
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
/**
 * @brief Enable GPIO interrupts on all pins except those in the blacklist
 * 
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 * @param falling_handler callback for falling edge interrupts
 * @param rising_handler callback for rising edge interrupts
 */
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
                if (is_interupt_blacklisted(abs_pin))                           \
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
 * @brief Disable GPIO interrupts on all pins except those in the blacklist
 * 
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void gpio_disable_all_interrupts(uint64_t blacklist_mask)
{
    for (uint8_t abs_pin = 0; abs_pin < MSP430_NUM_ABS_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Skip blacklisted pins

        uint8_t port = abs_pin >> 3;
        uint8_t pin = abs_pin & 0x07;

        *PxIE[port] &= ~(1 << pin);  // Disable interrupt on this pin
        *PxIFG[port] &= ~(1 << pin); // Clear any pending flag
    }
}

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
    uintptr_t base = get_port_base_of_absolute_pin(abs_pin); // ← abs_pin statt port verwenden
    uint8_t mask = 1 << abs_to_pinidx(abs_pin);

    // DIR = 0
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;
    // REN = 1
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
    // OUT = 1
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
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
            printf("Pushing active pin %u to stack\n", (unsigned int)abs_pin); // ← %u statt %lu
            stack_push(stack, &abs_pin);
        }
    }
}