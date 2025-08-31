#ifndef HANDSHAKE_H
#define HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_time.h"
#include "printf.h"
#include <stdbool.h>
#include <stdint.h>
#ifndef NUMBER_OF_GPIO_PINS
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#define DEBUG_PIN1 ABS_PIN(3, 4) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 ABS_PIN(3, 5) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 ABS_PIN(8, 1) // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 ABS_PIN(8, 2) // Additional debug pin, can be changed as needed
#elif defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#ifndef NUMBER_OF_GPIO_PINS

#endif

#define DEBUG_PIN1 26 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 27 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 39 // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 40 // Additional debug pin, can be changed as needed

#endif

#include "printf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include "timing_pindata.h"
#include "pindata.h"
#include "bitmap_iterator.h"
#include "pindata.h"


#define READER_INTERVAL_US 1000   // Reader: every 1 ms
#define PRESCALER_DIV 8           // Prescaler division factor for the timer
#define READER_TICKS ((READER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))
#define MANAGER_TICKS ((MANAGER_INTERVAL_US * (SMCLK_HZ / PRESCALER_DIV / 1000000UL)))

#define SYN_DURATION 200     // Duration of SYN signal in microseconds
#define SYN_ACK_DURATION 600 // Duration of SYN_ACK signal in microseconds
#define ACK_DURATION 1100    // Duration of ACK signal in microseconds

#define SYN_CYCLES ((uint32_t)SYN_DURATION * 1000UL / READER_INTERVAL_US)
#define SYN_ACK_CYCLES ((uint32_t)SYN_ACK_DURATION * 1000UL / READER_INTERVAL_US)
#define ACK_CYCLES ((uint32_t)ACK_DURATION * 1000UL / READER_INTERVAL_US)

#define SIGNAL_INACCURACY 100 // Signal inaccuracy in milliseconds
#define CYCLE_INACCURACY ((uint32_t)SIGNAL_INACCURACY * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_WAITING_CYCLES 500 // Maximum waiting cycles for a signal

#define MIN_SYN_CYCLES ((uint32_t)(SYN_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MIN_ACK_CYCLES ((uint32_t)(ACK_DURATION - SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)

#define MAXIMUM_SYN_CYCLES ((uint32_t)(SYN_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_SYN_ACK_CYCLES ((uint32_t)(SYN_ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define MAXIMUM_ACK_CYCLES ((uint32_t)(ACK_DURATION + SIGNAL_INACCURACY) * 1000UL / READER_INTERVAL_US)
#define DURATION_OF_HANDSHAKE_MS 60000 // Duration of the handshake process in milliseconds


#define MAXIMUM_NUMBER_OF_TRIES 5      // Maximum number of tries for a successful handshake
typedef struct
{
    uint16_t cycles;
    void (*on_complete)(TimingPinData *);
} TimingTaskDef;

void perform_handshake(PinData *pin_data, uint64_t initial_blacklist_mask);

#endif // HANDSHAKE_H
