
#include <msp430.h>
#include <msp430fr5994.h>

void _putchar(char character) {
    while (!(UCA1IFG & UCTXIFG)); // Wait until TX buffer is empty
    UCA1TXBUF = character;       // Send character
}