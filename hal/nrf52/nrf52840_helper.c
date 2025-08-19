#include "nrf52840_helper.h"

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

