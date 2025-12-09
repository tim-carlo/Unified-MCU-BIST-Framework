#include "nrf52840_helper.h"
#include "pin_config.h"


/**
 * @brief Initialize the IO peripherals
 *
 * This function initializes the UART for printf output and starts the HFCLK.
 * It should be called at the beginning of the main function.
 */
void mcu_init(void)
{
    // Start HFCLK if not running
    NRF_CLOCK->TASKS_HFCLKSTOP = 1;
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;

    if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
         NRF_CLOCK->TASKS_HFCLKSTART = 1;
         while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
         {
         }
    }
//     // Set up UART0 for printf
//    uart_pins_t uart_pins = create_uart_pins(19, 20); // TX = P0.19, RX = P0.20
//    uart_init(NRF_UART0, 9600, &uart_pins);
}

