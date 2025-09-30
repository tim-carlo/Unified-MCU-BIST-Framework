
#include <msp430.h>
#include <msp430fr5994.h>

void _putchar(char character) {
    while (!(UCA0IFG & UCTXIFG)); // Wait until TX buffer is empty
    UCA0TXBUF = character;       // Send character
}