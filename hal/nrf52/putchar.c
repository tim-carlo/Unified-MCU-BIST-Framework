#include <stdint.h>
#include <nrf.h>
#include "nrf52840_uart.h"

// This function is called internally by the printf function for each character.
// Here we can redirect the printf output stream to whereever we want.
void _putchar(char character)
{
    uart_write(NRF_UART0, character);
}