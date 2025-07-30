#ifndef MANCHESTER_H
#define MANCHESTER_H

#include <stdint.h>

#if defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "printf.h"
#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "printf.h"
#define TIMER_A TIMER_A4
#endif

// Reference to the Manchester encoding:
// https://ww1.microchip.com/downloads/en/AppNotes/Atmel-9164-Manchester-Coding-Basics_Application-Note.pdf
// The code is inspired by the above document and adapted for the NRF52 platform.

void manchester_init(uint8_t tx_pin, uint8_t rx_pin);
void manchester_transmit_array(uint8_t length, uint8_t *data);
bool manchester_receive_array(uint8_t *data, uint8_t length);


#endif // MANCHESTER_H