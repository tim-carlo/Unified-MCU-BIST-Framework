#ifndef PARALLEL_MANCHESTER_H
#define PARALLEL_MANCHESTER_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "printf.h"

#include "spooky_decoder.h"
#include "spooky_encoder.h"

#include "pin_config.h"

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
#include <msp430.h>
#endif

#define PMAN_TXRX_RATE 4 // Number of samples per bit (must match encoder/decoder settings)

typedef enum {
    PMAN_BAUD_100 = 100,
    PMAN_BAUD_300 = 300,
    PMAN_BAUD_600 = 600,
    PMAN_BAUD_1200 = 1200,
    PMAN_BAUD_2400 = 2400,
    PMAN_BAUD_4800 = 4800,
    PMAN_BAUD_9600 = 9600,
    PMAN_BAUD_19200 = 19200,
    PMAN_BAUD_38400 = 38400
} ParallelManchesterBaudRate;

typedef enum
{
    PMAN_NOT_INITIALIZED,
    PMAN_IDLE,
    PMAN_SEND,
    PMAN_RECEIVE
} ParallelManchesterMode;

// Status bit definitions for ParallelManchesterInstance.status
#define PMAN_STATUS_TRANSMISSION_COMPLETE   (1 << 0)
#define PMAN_STATUS_RECEIVE_COMPLETE        (1 << 1)  
#define PMAN_STATUS_RECEIVE_ERROR           (1 << 2)
#define PMAN_STATUS_DATA_RECEIVED           (1 << 3)

// Definitions
static const uint8_t PMAN_SAMPLE_RATE = 4;
static const uint16_t NUMBER_OF_MAX_INSTACES_WITHOUT_TRANSITION = 1000;

typedef struct {
    uint8_t pin;
    struct spooky_encoder enc;
    struct spooky_decoder dec;
    ParallelManchesterMode mode;
    uint8_t *buffer;      // Single buffer for encode/decode operations
    uint8_t buffer_size;  // Size of the buffer
    // Status bits: bit 0 = transmission_complete, bit 1 = receive_complete, 
    // bit 2 = receive_error, bit 3 = data_received
    volatile uint8_t status;
    uint8_t last_decoder_mode; // Track last decoder mode for state transition monitoring
    uint16_t cycles_without_transition; // For timeout handling during receive
    bool last_rx;          // Track last RX state for transition detection
} ParallelManchesterInstance;

extern uint8_t pman_instance_count;
extern ParallelManchesterInstance *pman_instances;

void pman_timer_isr(void);
// The baudrate must be set equally for all, since the ISR is shared
void parallel_manchester_init(ParallelManchesterBaudRate tx_rate);
uint32_t parallel_manchester_get_sample_interval_us(ParallelManchesterBaudRate rate);
void parallel_manchester_deinit();

// Instance management - now accepts buffer per instance
uint8_t parallel_manchester_add_instance(uint8_t pin, uint8_t *buffer, uint8_t buffer_size);
bool parallel_manchester_remove_instance(uint8_t index);

// Non-blocking transmission and receive functions
bool parallel_manchester_transmit_background(uint8_t index, uint8_t size);
bool parallel_manchester_transmit_complete(uint8_t index);
bool parallel_manchester_receive_background(uint8_t index, uint8_t *data, uint8_t size);
bool parallel_manchester_receive_complete(uint8_t index);
bool parallel_manchester_receive_error(uint8_t index);

// Status functions
bool parallel_manchester_is_idle(uint8_t index);
bool parallel_manchester_is_transmitting(uint8_t index);
bool parallel_manchester_is_receiving(uint8_t index);
bool parallel_manchester_data_received(uint8_t index);

// Helpers
uint32_t parallel_manchester_get_sample_interval_us(ParallelManchesterBaudRate rate);

// Add new helper functions
uint8_t* parallel_manchester_get_received_data(uint8_t index);
uint8_t parallel_manchester_get_buffer_size(uint8_t index);

#endif // PARALLEL_MANCHESTER_H