#include "msp430fr5994_gpio.h"
/**
 * @brief Map absolute pin number to port register group (0–7: PJ, 8–15: P1, etc.)
 */
static inline uint8_t abs_to_port(uint8_t abs_pin)
{
    return abs_pin >> 3; // is the same as abs_pin / 8
}

static inline uint8_t abs_to_pinidx(uint8_t abs_pin)
{
    return abs_pin & 0b111; // is the same as abs_pin % 8
}

static volatile uint8_t *get_port_in(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJIN;
    case 1:
        return &P1IN;
    case 2:
        return &P2IN;
    case 3:
        return &P3IN;
    case 4:
        return &P4IN;
    case 5:
        return &P5IN;
    case 6:
        return &P6IN;
    case 7:
        return &P7IN;
    default:
        return 0;
    }
}

static volatile uint8_t *get_port_out(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJOUT;
    case 1:
        return &P1OUT;
    case 2:
        return &P2OUT;
    case 3:
        return &P3OUT;
    case 4:
        return &P4OUT;
    case 5:
        return &P5OUT;
    case 6:
        return &P6OUT;
    case 7:
        return &P7OUT;
    default:
        return 0;
    }
}

// Array verwenden
static volatile uint8_t *get_port_sel0(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJSEL0;
    case 1:
        return &P1SEL0;
    case 2:
        return &P2SEL0;
    case 3:
        return &P3SEL0;
    case 4:
        return &P4SEL0;
    case 5:
        return &P5SEL0;
    case 6:
        return &P6SEL0;
    case 7:
        return &P7SEL0;
    default:
        return 0;
    }
}

static volatile uint8_t *get_port_sel1(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJSEL1;
    case 1:
        return &P1SEL1;
    case 2:
        return &P2SEL1;
    case 3:
        return &P3SEL1;
    case 4:
        return &P4SEL1;
    case 5:
        return &P5SEL1;
    case 6:
        return &P6SEL1;
    case 7:
        return &P7SEL1;
    default:
        return 0;
    }
}

static volatile uint8_t *get_port_dir(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJDIR;
    case 1:
        return &P1DIR;
    case 2:
        return &P2DIR;
    case 3:
        return &P3DIR;
    case 4:
        return &P4DIR;
    case 5:
        return &P5DIR;
    case 6:
        return &P6DIR;
    case 7:
        return &P7DIR;
    default:
        return 0;
    }
}

static volatile uint8_t *get_port_ren(uint8_t port)
{
    switch (port)
    {
    case 0:
        return &PJREN;
    case 1:
        return &P1REN;
    case 2:
        return &P2REN;
    case 3:
        return &P3REN;
    case 4:
        return &P4REN;
    case 5:
        return &P5REN;
    case 6:
        return &P6REN;
    case 7:
        return &P7REN;
    default:
        return 0;
    }
}

void gpio_output_init(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *get_port_dir(port) |= mask;
}

void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    *get_port_dir(port) &= ~mask; // input

    switch (pull)
    {
    case GPIO_PULL_NONE:
        *get_port_ren(port) &= ~mask;
        break;

    case GPIO_PULL_DOWN:
        *get_port_ren(port) |= mask;
        *get_port_out(port) &= ~mask;
        break;

    case GPIO_PULL_UP:
        *get_port_ren(port) |= mask;
        *get_port_out(port) |= mask;
        break;
    }
}

void gpio_drive_high(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);
    *get_port_out(port) |= mask;
}

void gpio_drive_low(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);
    *get_port_out(port) &= ~mask;
}

void gpio_toggle(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);
    *get_port_out(port) ^= mask;
}

void gpio_reset(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);

    *get_port_dir(port) &= ~mask;
    *get_port_ren(port) &= ~mask;
    *get_port_out(port) &= ~mask;
}

bool gpio_read(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);
    return (*get_port_in(port) & mask) != 0;
}

/**
 * @brief Read all 8 ports (PJ + P1–P7)
 */
uint64_t gpio_read_all_ports(void)
{
    uint64_t state = 0;
    state |= (uint64_t)PJIN << 0;
    state |= (uint64_t)P1IN << 8;
    state |= (uint64_t)P2IN << 16;
    state |= (uint64_t)P3IN << 24;
    state |= (uint64_t)P4IN << 32;
    state |= (uint64_t)P5IN << 40;
    state |= (uint64_t)P6IN << 48;
    state |= (uint64_t)P7IN << 56;
    return state;
}

void gpio_od_init(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);

    *get_port_dir(port) &= ~mask; // input
    *get_port_ren(port) |= mask;  // resistor enabled
    *get_port_out(port) |= mask;  // pull-up
}

void gpio_od_hold_low(uint8_t abs_pin)
{
    gpio_output_init(abs_pin);
    gpio_drive_low(abs_pin);
}

void gpio_od_release(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1 << abs_to_pinidx(abs_pin);

    *get_port_dir(port) &= ~mask; // input
    *get_port_ren(port) |= mask;  // resistor enabled
    *get_port_out(port) |= mask;  // pull-up
}