#include "nrf52840_gpio.h"
#include "bitmap_iterator.h"
#include "nrf52840.h"
#include "nrf.h"
#include "printf.h"
#include "stack.h"

// States
static volatile uint64_t s_prev_state = 0;
static volatile uint64_t s_blacklist_mask = 0;
static gpio_interrupt_handler_t s_falling = NULL;
static gpio_interrupt_handler_t s_rising = NULL;

static inline NRF_GPIO_Type *port_ptr(uint32_t port)
{
#if defined(NRF_P0_BASE)
    return (port == 0) ? NRF_P0 : NRF_P1;
#else
    (void)port;
    return NULL;
#endif
}

static inline void cfg_pin_input(NRF_GPIO_Type *p, uint32_t idx, gpio_pull_t pull)
{
    // DIR=Input, INPUT=Connect, DRIVE=S0S1 (default), SENSE=Disabled
    uint32_t cnf = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) | BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);

    switch (pull)
    {
    case GPIO_PULL_UP:
        cnf |= BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup);
        // This means that the pin
        //     cnf |= BV_BY_NAME(GPIO_DRIVE_MODE, D0S1);
        break;
    case GPIO_PULL_DOWN:
        cnf |= BV_BY_NAME(GPIO_PIN_CNF_PULL, Pulldown);
        break;
    case GPIO_PULL_NONE:
        cnf |= BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled);
        break;
    default:
        cnf |= BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled);
        break;
    }
    p->PIN_CNF[idx] = cnf;
}
static inline void cfg_pin_output(NRF_GPIO_Type *p, uint32_t idx)
{
    // DIR=Output, INPUT=Connected, PULL=Disabled, DRIVE=S0S1, SENSE=Disabled
    // The Inputregister is still connected, so we can read the pin state even if it is an output
    uint32_t cnf = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) | BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) | BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) | BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) | BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
    p->PIN_CNF[idx] = cnf;
}

static inline uint32_t pin_level(NRF_GPIO_Type *p, uint32_t idx)
{
    return (p->IN >> idx) & 1u;
}

void gpio_output_init(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    cfg_pin_output(port_ptr(port), idx);
}

void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    cfg_pin_input(port_ptr(port), idx, pull);
}

static inline uint32_t gpio_pull_to_cnf_bits(gpio_pull_t pull)
{
    switch (pull)
    {
    case GPIO_PULL_UP:
        return BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup);
    case GPIO_PULL_DOWN:
        return BV_BY_NAME(GPIO_PIN_CNF_PULL, Pulldown);
    case GPIO_PULL_NONE:
    default:
        return BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled);
    }
}

void gpio_reset_from_blacklist(const uint64_t blacklist_mask)
{
    const uint64_t mask = ~blacklist_mask;
    const uint8_t pin;
    while (bitmap_iterator_next_mask_as_param(&mask, &pin))
    {
        const uint32_t port = ABS_TO_PORT(pin);
        const uint32_t idx = ABS_TO_PINIDX(pin);
        port_ptr(port)->PIN_CNF[idx] = 0;
    }
}

void gpio_pullup_init(uint8_t abs_pin) { gpio_input_init(abs_pin, GPIO_PULL_UP); }
void gpio_pulldown_init(uint8_t abs_pin) { gpio_input_init(abs_pin, GPIO_PULL_DOWN); }

void gpio_drive_high(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    port_ptr(port)->OUTSET = (1UL << idx);
}

void gpio_drive_low(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    port_ptr(port)->OUTCLR = (1UL << idx);
}
void gpio_reset(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);

    port_ptr(port)->PIN_CNF[idx] =
        (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void gpio_toggle(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    NRF_GPIO_Type *p = port_ptr(port);
    uint32_t m = (1UL << idx);
    if (p->OUT & m)
        p->OUTCLR = m;
    else
        p->OUTSET = m;
}

bool gpio_read(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    return pin_level(port_ptr(port), idx) ? true : false;
}

/**
 * @brief Read both GPIO ports with explicit bit positioning
 *
 * @return uint64_t Combined value where:
 *         - Bits [31:0]  = Port 0 pins
 *         - Bits [63:32] = Port 1 pins
 */
uint64_t gpio_read_all_ports(void)
{
    return ((uint64_t)NRF_P1->IN << 32) | (uint64_t)NRF_P0->IN;
}

/**
 * @brief Configure pin sense for interrupts using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 * @param sense_low If true, configure for low sense (falling edge), else for high sense (rising edge)
 */
void configure_pin_sense(uint8_t abs_pin, bool sense_low)
{
    uint32_t port_num = ABS_TO_PORT(abs_pin);
    uint32_t pin = ABS_TO_PINIDX(abs_pin);
    NRF_GPIO_Type *port = port_ptr(port_num);

    // Clear SENSE bits first
    port->PIN_CNF[pin] &= ~GPIO_PIN_CNF_SENSE_Msk;

    if (sense_low)
        port->PIN_CNF[pin] |= BV_BY_NAME(GPIO_PIN_CNF_SENSE, Low);
    else
        port->PIN_CNF[pin] |= BV_BY_NAME(GPIO_PIN_CNF_SENSE, High);
}

static bool is_interupt_blacklisted(uint8_t abs_pin)
{
    return (s_blacklist_mask >> abs_pin) & 1;
}

/**
 * @brief Function to listen for GPIO interrupts on all pins, excluding blacklisted ones
 *
 * @param blacklist Bitmask of pins to exclude (1 for excluded, 0 for included)
 * @param falling_handler Function to call on falling edge
 * @param rising_handler Function to call on rising edge
 */
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    s_blacklist_mask = blacklist;
    s_falling = falling_handler;
    s_rising = rising_handler;

    for (uint8_t abs_pin = 0; abs_pin < 48; abs_pin++)
    {
        if (is_interupt_blacklisted(abs_pin))
            continue;

        uint32_t port_num = ABS_TO_PORT(abs_pin);
        uint32_t pin = ABS_TO_PINIDX(abs_pin);
        NRF_GPIO_Type *port = port_ptr(port_num);

        // Set pin as input with pull-up resistor
        port->PIN_CNF[pin] =
            BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
            BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
            BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
            BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
            BV_BY_NAME(GPIO_PIN_CNF_SENSE, Low);
    }

    // Clear all latch bits to ensure no false triggers
    NRF_P0->LATCH = 0xFFFFFFFF;
    NRF_P1->LATCH = 0xFFFFFFFF;

    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    //  NVIC_SetPriority(GPIOTE_IRQn, 0);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief Disable GPIO interrupts
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void gpio_disable_all_interrupts(uint64_t blacklist_mask)
{
    NVIC_DisableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Msk;
    NRF_P0->LATCH = 0xFFFFFFFF;
    NRF_P1->LATCH = 0xFFFFFFFF;
}

void GPIOTE_IRQHandler(void)
{
    if (!NRF_GPIOTE->EVENTS_PORT)
        return;

    NRF_GPIOTE->EVENTS_PORT = 0; // Clear the event

    for (uint8_t abs_pin = 0; abs_pin < NRF52_NUM_ABS_PINS; abs_pin++)
    {
        if (is_interupt_blacklisted(abs_pin))
            continue;

        uint32_t port_num = ABS_TO_PORT(abs_pin);
        uint32_t pin_idx = ABS_TO_PINIDX(abs_pin);
        NRF_GPIO_Type *port = port_ptr(port_num);

        if (port->LATCH & (1UL << pin_idx))
        {
            bool sample = pin_level(port, pin_idx);
            // Depending on the current state, call the appropriate handler
            if (!sample)
            {
                if (s_falling)
                    s_falling(abs_pin);
                // Next: SENSE_High (for Rising)
                configure_pin_sense(abs_pin, false);
            }
            else
            {
                if (s_rising)
                    s_rising(abs_pin);
                // Next: SENSE_Low (for Falling)
                configure_pin_sense(abs_pin, true);
            }

            // Clear the latch bit for this pin
            port->LATCH = (1UL << pin_idx);
        }
    }
}
/**
 * @brief GPIO open-drain initialization using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_od_init(uint8_t abs_pin)
{
    uint32_t port = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    NRF_GPIO_Type *p = port_ptr(port);
    // Set pin as input with pull-up
    p->PIN_CNF[idx] =
        BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
        BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
        BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
        BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
        BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
    p->OUTSET = (1UL << idx); // Set the pin high
}

/**
 * @brief Hold GPIO pin in open-drain state (drive low) using absolute pin number
 *
 * @param abs_pin
 */
void gpio_od_hold_low(uint8_t abs_pin)
{
    uint32_t port_num = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    NRF_GPIO_Type *p = port_ptr(port_num);

    // Temporarily disable GPIOTE interrupt
    NVIC_DisableIRQ(GPIOTE_IRQn);

    // Configure as output and drive low
    p->OUTCLR = (1UL << idx);
    p->DIRSET = (1UL << idx);

    // Re-enable GPIOTE interrupt
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief Release GPIO pin from open-drain state (set as input) using absolute pin number
 * And reset the sense configuration
 * @param abs_pin
 */
void gpio_od_release(uint8_t abs_pin)
{
    uint32_t port_num = ABS_TO_PORT(abs_pin);
    uint32_t idx = ABS_TO_PINIDX(abs_pin);
    NRF_GPIO_Type *p = port_ptr(port_num);

    NVIC_DisableIRQ(GPIOTE_IRQn);
    p->OUTSET = (1UL << idx);

    // Switch to input (high-Z, pull-up)
    p->DIRCLR = (1UL << idx);

    // Clear any latched state before enabling SENSE
    p->LATCH = (1UL << idx);

    // Re-enable SENSE=Low for future falling edges
    uint32_t cnf = p->PIN_CNF[idx];
    cnf &= ~GPIO_PIN_CNF_SENSE_Msk;
    cnf |= BV_BY_NAME(GPIO_PIN_CNF_SENSE, Low);
    p->PIN_CNF[idx] = cnf;

    NVIC_EnableIRQ(GPIOTE_IRQn);
}

/**
 * @brief stack_push all active GPIO pins except the specified one to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 * @param expected_level Expected level of the pins (true for high, false for low)
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask)
{
    for (uint8_t abs_pin = 0; abs_pin < NRF52_NUM_ABS_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue;
        if (gpio_read(abs_pin) == expected_level)
        {
            printf("Pushing active pin %lu to stack\n", abs_pin);
            stack_push(stack, &abs_pin);
        }
    }
}
