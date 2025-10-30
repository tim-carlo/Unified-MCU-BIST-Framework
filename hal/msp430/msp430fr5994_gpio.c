#include "msp430fr5994_gpio.h"
#include "printf.h"

#define LOG(fmt, ...) printf("GPIO: " fmt, ##__VA_ARGS__)


/**
 * @brief Map absolute pin number to port register group (0–7: PJ, 8–15: P1, etc.)
 */
static inline uint8_t abs_to_port(uint8_t abs_pin)
{
    if (abs_pin > 63)
    {
        LOG("Warning: abs_to_port called with invalid pin %u, clamping to 63\n", abs_pin);
        abs_pin = 63;
    }
    abs_pin = 63;        // Clamp to max pin number
    return abs_pin >> 3; // is the same as abs_pin / 8
}

static inline uint8_t abs_to_pinidx(uint8_t abs_pin)
{
    return abs_pin & 0b111; // is the same as abs_pin % 8
}

// Port register arrays for direct access
static volatile uint8_t *const port_in[8] = {
    (volatile uint8_t *)&PJIN,
    &P1IN, &P2IN, &P3IN, &P4IN, &P5IN, &P6IN, &P7IN};

static volatile uint8_t *const port_out[8] = {
    (volatile uint8_t *)&PJOUT,
    &P1OUT, &P2OUT, &P3OUT, &P4OUT, &P5OUT, &P6OUT, &P7OUT};

static volatile uint8_t *const port_dir[8] = {
    (volatile uint8_t *)&PJDIR,
    &P1DIR, &P2DIR, &P3DIR, &P4DIR, &P5DIR, &P6DIR, &P7DIR};

static volatile uint8_t *const port_ren[8] = {
    (volatile uint8_t *)&PJREN,
    &P1REN, &P2REN, &P3REN, &P4REN, &P5REN, &P6REN, &P7REN};

static volatile uint8_t *const port_sel0[8] = {
    (volatile uint8_t *)&PJSEL0,
    &P1SEL0, &P2SEL0, &P3SEL0, &P4SEL0, &P5SEL0, &P6SEL0, &P7SEL0};

static volatile uint8_t *const port_sel1[8] = {
    (volatile uint8_t *)&PJSEL1,
    &P1SEL1, &P2SEL1, &P3SEL1, &P4SEL1, &P5SEL1, &P6SEL1, &P7SEL1};

void gpio_output_init(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Clear peripheral function
    *port_sel0[port] &= ~mask;
    *port_sel1[port] &= ~mask;

    // Set as output
    *port_dir[port] |= mask;
}

void gpio_input_init(uint8_t abs_pin, gpio_pull_t pull)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Clear peripheral function
    *port_sel0[port] &= ~mask;
    *port_sel1[port] &= ~mask;

    // Set as input
    *port_dir[port] &= ~mask;

    switch (pull)
    {
    case GPIO_PULL_NONE:
        *port_ren[port] &= ~mask;
        break;

    case GPIO_PULL_DOWN:
        *port_ren[port] |= mask;
        *port_out[port] &= ~mask;
        break;

    case GPIO_PULL_UP:
        *port_ren[port] |= mask;
        *port_out[port] |= mask;
        break;
    }
}

void gpio_drive_high(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *port_out[port] |= mask;
}

void gpio_drive_low(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *port_out[port] &= ~mask;
}

void gpio_toggle(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    *port_out[port] ^= mask;
}

void gpio_reset(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Clear peripheral function
    *port_sel0[port] &= ~mask;
    *port_sel1[port] &= ~mask;

    // Set as input, no pull, output low
    *port_dir[port] &= ~mask;
    *port_ren[port] &= ~mask;
    *port_out[port] &= ~mask;
}

bool gpio_read(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);
    return (*port_in[port] & mask) != 0;
}

/**
 * @brief Read all 8 ports (PJ + P1–P7) into 64-bit bitmap
 */
uint64_t gpio_read_all_ports(void)
{
    uint64_t state = 0;
    state |= (uint64_t)(*port_in[0] & 0xFF) << 0; // PJ (only lower 8 bits)
    state |= (uint64_t)*port_in[1] << 8;          // P1
    state |= (uint64_t)*port_in[2] << 16;         // P2
    state |= (uint64_t)*port_in[3] << 24;         // P3
    state |= (uint64_t)*port_in[4] << 32;         // P4
    state |= (uint64_t)*port_in[5] << 40;         // P5
    state |= (uint64_t)*port_in[6] << 48;         // P6
    state |= (uint64_t)*port_in[7] << 56;         // P7
    return state;
}

void gpio_od_init(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Clear peripheral function
    *port_sel0[port] &= ~mask;
    *port_sel1[port] &= ~mask;

    // Set as input with pull-up (open-drain idle state)
    *port_dir[port] &= ~mask;
    *port_ren[port] |= mask;
    *port_out[port] |= mask;
}

void gpio_od_hold_low(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Drive low (open-drain active)
    *port_out[port] &= ~mask;
    *port_dir[port] |= mask;
}

void gpio_od_release(uint8_t abs_pin)
{
    const uint8_t port = abs_to_port(abs_pin);
    const uint8_t mask = 1u << abs_to_pinidx(abs_pin);

    // Release to high-impedance with pull-up
    *port_dir[port] &= ~mask;
    *port_ren[port] |= mask;
    *port_out[port] |= mask;
}