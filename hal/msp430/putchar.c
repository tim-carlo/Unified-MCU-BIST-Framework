
#include <msp430.h>
#include <msp430fr5994.h>
#include "pin_config.h"

void _putchar(char character)
{
#if DEV_KIT == 0
    while (!(UCA1IFG & UCTXIFG))
        ;                  // Wait until TX buffer is empty
    UCA1TXBUF = character; // Send character
#else
    while (!(UCA0IFG & UCTXIFG))
        ;                  // Wait until TX buffer is empty
    UCA0TXBUF = character; // Send character
#endif
}