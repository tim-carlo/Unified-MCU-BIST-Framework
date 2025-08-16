#include "nrf52840_helper.h"

#include <string.h>

// Variable definitions

gpio_interrupt_handler_t rising_callback_single = NULL;
gpio_interrupt_handler_t falling_callback_single = NULL;
uint8_t gpiote_pin0 = 0xFF;

uint32_t bibanging_uart_baudtrate = -1;
uint32_t bibanging_uart_bit_time_us = -1;
uint8_t UART_PIN = -1;

uint64_t gpio_blacklist_intern_mask = 0; // Global blacklist for GPIO pins
gpio_interrupt_handler_t rising_handler_global = NULL;
gpio_interrupt_handler_t falling_handler_global = NULL;

static volatile uint32_t prev_input_state_p0 = 0;
static volatile uint32_t prev_input_state_p1 = 0;

/**
 * @brief Initialize the IO peripherals
 *
 * This function initializes the UART for printf output and starts the HFCLK.
 * It should be called at the beginning of the main function.
 */
void io_init(void)
{
    // Start HFCLK if not running
    if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        {
        }
    }
    // Set up UART0 for printf
    NRF_UART0->PSELTXD = UART_PIN_TX;                                         // TX pin
    NRF_UART0->PSELRXD = UART_PIN_RX;                                         // RX pin
    NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud9600;                    // Set baud rate to 9600
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos; // Enable UART
    NRF_UART0->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) |
                        (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos); // No parity, no flow control
    NRF_UART0->TASKS_STARTTX = 1;                                            // Start UART transmission
    NRF_UART0->TASKS_STARTRX = 1;                                            // Start UART reception
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
    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
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

