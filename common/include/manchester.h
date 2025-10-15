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
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#include "nrf52840_utils.h"

#elif defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_helper.h"
#include <msp430.h>
#endif

#define TX_RATE 4 // Number of samples per bit (must match encoder/decoder settings)

typedef enum {
    BAUD_300 = 300,
    BAUD_600 = 600,
    BAUD_1200 = 1200,
    BAUD_2400 = 2400,
    BAUD_4800 = 4800,
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400
} BaudRate;


void manchester_init(BaudRate tx_rate);
void manchester_deinit();
void manchester_set_rx_pin(uint8_t pin);
void manchester_set_tx_pin(uint8_t pin);
void manchester_set_rx_pin_od(uint8_t pin);
void manchester_set_tx_pin_od(uint8_t pin);

bool manchester_transmit_in_background_complete(void);
bool manchester_transmit_in_background_cancelled(void);

bool manchester_transmit_array_in_background(uint8_t *data, uint8_t size);
bool manchester_transmit_array(uint8_t *data, uint8_t size);
bool manchester_receive_array(uint8_t *data, uint8_t size);

// Helpers
uint32_t get_sample_interval_us(BaudRate rate);

void manchester_cancel_transmission(void);

bool manchester_is_transmitting(void);
bool manchester_is_receiving(void);
bool manchester_is_idle(void);

#endif // MANCHESTER_h