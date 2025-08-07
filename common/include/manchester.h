
#ifndef MANCHESTER_h
#define MANCHESTER_h

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "printf.h"

#include "spooky_decoder.h"
#include "spooky_encoder.h"

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_helper.h"

#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include <msp430.h>
#endif

#define RATE_300 0
#define RATE_600 1
#define RATE_1200 2
#define RATE_2400 3
#define RATE_4800 4
#define RATE_9600 5
#define RATE_19200 6
#define RATE_38400 7

// Add baud_rates array for supported rates
static const uint32_t baud_rates[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400};

void manchester_init(uint8_t tx_pin, uint8_t rx_pin, uint8_t tx_rate);
void manchester_begin_receive(void);
void manchester_stop_receive(void);
void manchester_transmit_array(uint8_t *bsp, uint8_t size);
void manchester_test_timer_simple(void);   // Test timer function
void manchester_test_pin_manual(void);     // Test pin manually
void manchester_test_timer_extended(void); // Extended timer test
void manchester_get_timer_stats(void);     // Get timer statistics

#endif // MANCHESTER_h