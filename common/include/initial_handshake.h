#ifndef INITIAL_HANDSHAKE_H
#define INITIAL_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_time.h"
#include "printf.h"
#include <stdbool.h>
#include <stdint.h>

#elif defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"

#endif

#include "printf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "handshake_pindata.h"
#include "pindata.h"
#include "bitmap_iterator.h"
#include "pindata.h"
#include "pin_config.h"

#define READER_INTERVAL_US (5000) // Reader: every 5 ms
#define PRESCALER_DIV (8)         // Prescaler division factor for the timer
#define READER_TICKS ((READER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))

#define SYN_DURATION (200)     // Duration of SYN signal in microseconds
#define SYN_ACK_DURATION (600) // Duration of SYN_ACK signal in microseconds
#define ACK_DURATION (1100)    // Duration of ACK signal in microseconds

#define SYN_CYCLES ((uint32_t)SYN_DURATION * 1000UL / READER_INTERVAL_US)
#define SYN_ACK_CYCLES ((uint32_t)SYN_ACK_DURATION * 1000UL / READER_INTERVAL_US)
#define ACK_CYCLES ((uint32_t)ACK_DURATION * 1000UL / READER_INTERVAL_US)

#define SIGNAL_INACCURACY 100 // Signal inaccuracy in milliseconds
#define CYCLE_INACCURACY ((uint32_t)SIGNAL_INACCURACY * 1000UL / READER_INTERVAL_US)

#define MINIMUM_WAITING_TIME_MS (500)                                                                 // 50ms
#define MAXIMUM_WAITING_CYCLES ((uint32_t)((MINIMUM_WAITING_TIME_MS * 1000UL) / READER_INTERVAL_US)) // 500ms max waiting time

#define MIN_SYN_CYCLES ((uint32_t)(SYN_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_ACK_CYCLES ((uint32_t)(ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)

#define MAXIMUM_SYN_CYCLES ((uint32_t)(SYN_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_ACK_CYCLES ((uint32_t)(ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define DURATION_OF_HANDSHAKE_MS ((uint32_t)((5000 * 1000UL) / READER_INTERVAL_US)) // Duration of the handshake process in milliseconds

#define MAXIMUM_NUMBER_OF_TRIES (5) // Maximum number of tries for a successful handshake

typedef struct
{
    uint16_t cycles;
    void (*on_complete)(HandshakePinData *);
} TimingTaskDef;

typedef struct
{
    volatile uint64_t initial_state_mask;   // Mask to store the initial state of pins
    HandshakePinData *global_timing_pindata;   // Global pointer to HandshakePinData array
    volatile uint8_t number_of_active_pins; // Number of active pins participating in handshake
    volatile uint32_t handshake_time;       // Current handshake time counter
} InitialHandshakeState;

typedef enum
{
    HANDSHAKE_FOUND_WORKING_PIN = 0,
    HANDSHAKE_NO_WORKING_PIN_FOUND = 1,
    HANDSHAKE_ISR_TIMEOUT = 2,
} InitialHandshakeResult;

InitialHandshakeResult perform_initial_handshake(PinData *pin_data, uint64_t initial_blacklist_mask);

#endif // INITIAL_HANDSHAKE_H
