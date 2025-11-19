#ifndef DATAHANDSHAKE_MODULATION_H
#define DATAHANDSHAKE_MODULATION_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "printf.h"
#include "pin_config.h"

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

typedef enum
{
    PMAN_NOT_INITIALIZED = 0,
    PMAN_IDLE = 1,
    PMAN_SEND = 2,
    PMAN_RECEIVE  = 3
} ParallelManchesterState;
typedef uint8_t ParallelManchesterMode;


// Definitions
static const uint16_t NUMBER_OF_MAX_INSTACES_WITHOUT_TRANSITION = 1000;

typedef enum
{
    JOB_LISTEN = 0,
    JOB_SEND_REQUEST = 1,
    JOB_SEND_ANSWER = 2,
    JOB_WAIT_FOR_ANSWER = 3,
    JOB_TRANSMITTING_ANSWER = 4,
    JOB_TRANSMITTING_REQUEST = 5,
    JOB_RECEIVING_ANSWER = 6,
    JOB_RECEIVING_REQUEST = 7
} CurrentJobType;


typedef enum
{
    DHANDSHAKE_ROLE_INITIATOR = 0,
    DHANDSHAKE_ROLE_RESPONDER = 1,
    DHANDSHAKE_ROLE_UNCLEAR = 2
} DataHandshakeRoleType;

#define STATUS_TX_COMPLETE (1 << 0)
#define STATUS_RX_COMPLETE (1 << 1)
#define STATUS_RX_ERROR (1 << 2)
#define STATUS_DATA_RECEIVED (1 << 3)
#define STATUS_MANCHESTER_MASK (3 << 4) // bits 4-5
#define STATUS_ROLE_INITIATOR (1 << 6)
#define STATUS_HANDSHAKE_SUCCESS (1 << 7)

#define ANSWER_PACKSIZE (25)
#define REQUEST_PACKSIZE (15)
#define BUFFER_SIZE ANSWER_PACKSIZE
typedef struct
{
    uint8_t pin;
    CurrentJobType current_job;

    // Status bits: bit 0 = transmission_complete, bit 1 = receive_complete,
    // bit 2 = receive_error, bit 3 = data_received, bit 4 and 5 manchester status,  bit 6 = role initiator/responder,
    // bit 7 successful hanshake
    volatile uint8_t status;

    uint16_t receiving_counter;
    uint16_t time_until_next_send;
    uint32_t last_send_job_order;

    uint8_t data_buffer[BUFFER_SIZE];
    struct spooky_encoder manchester_enc;
    struct spooky_decoder manchester_dec;
    volatile uint8_t manchester_status;
    uint8_t manchester_last_decoder_mode;
    bool manchester_last_rx;
    uint16_t manchester_rx_timeout_counter;
    uint8_t handshake_attempts;
} DataHandshakeData;

// Status check functions
bool dhd_status_tx_complete(const DataHandshakeData *dhd);
bool dhd_status_rx_complete(const DataHandshakeData *dhd);
bool dhd_status_rx_error(const DataHandshakeData *dhd);
bool dhd_status_data_received(const DataHandshakeData *dhd);
bool dhd_status_role_initiator(const DataHandshakeData *dhd);
void dhd_set_role_initiator(DataHandshakeData *dhd, bool is_initiator);

bool dhd_status_handshake_success(const DataHandshakeData *dhd);
ParallelManchesterMode dhd_get_manchester_mode(const DataHandshakeData *dhd);
void dhd_set_manchester_mode(DataHandshakeData *dhd, ParallelManchesterMode mode);

void pman_timer_isr(DataHandshakeData *dhd_instances, uint8_t pman_instance_count);
uint32_t parallel_manchester_get_sample_interval_us(const uint8_t rate);

// Instance management - now accepts buffer per instance
bool parallel_manchester_add_instance(DataHandshakeData *instance);

// Non-blocking transmission and receive functions
bool parallel_manchester_transmit_background(DataHandshakeData *instance, uint8_t size);
bool parallel_manchester_transmit_complete(DataHandshakeData *instance);
bool parallel_manchester_receive_background(DataHandshakeData *instance);
bool parallel_manchester_receive_complete(DataHandshakeData *instance);
bool parallel_manchester_receive_error(DataHandshakeData *instance);
bool parallel_manchester_data_received(DataHandshakeData *instance);

#endif // DATAHANDSHAKE_MODULATION_H