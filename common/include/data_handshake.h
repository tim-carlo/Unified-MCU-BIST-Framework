#ifndef DATA_HANDSHAKE_H
#define DATA_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#define DATA_TIMER TIMER_A2

#define DEBUG_PIN1 ABS_PIN(3, 4) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 ABS_PIN(3, 5) // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 ABS_PIN(8, 1) // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 ABS_PIN(8, 2) // Additional debug pin, can be changed as needed

#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS

#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"
#include "nrf52840_utils.h"
#define DATA_TIMER NRF_TIMER2
#define DEBUG_PIN1 26 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 27 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 39 // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 40 // Additional debug pin, can be changed as needed

#define NUMBER_OF_GPIO_PINS NRF52_NUM_ABS_PINS

#endif

//#include "manchester.h"
#include "paralell_manchester.h"
#include "printf.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "timing_pindata.h"
#include "pindata.h"
#include "bitmap_iterator.h"
#include "random_utils.h"
#include "datahandshake_pindata.h"
#include "crc.h"

#define DATA_TIMER_INTERVAL_US 1000 // 10ms interval for both platforms
#define DATA_TIMER_INTERVAL_MS (DATA_TIMER_INTERVAL_US / 1000)

#define REQUEST_PACKSIZE 15
#define ANSWER_PACKSIZE 25

#define REQEST_MUTEX_ON_THIS_PIN 0x55
#define ALLOWING_MUTEX_ON_THIS_PIN 0xAA
#define NO_MUTEX_ON_THIS_PIN 0x00

typedef enum
{
    PACKET_TYPE_REQUEST,
    PACKET_TYPE_ANSWER,
} PackageType;


typedef enum
{
    DATA_HANDSHAKE_INITIALIZING_FAILURE,
    DATA_HANDSHAKE_HANDSHAKE_FAILURE,
    DATA_HANDSHAKE_SUCCESS
} DataHandshakeStatus;

typedef struct
{
    uint8_t mutex_pin;
    bool i_am_mutex_owner;
    DataHandshakeStatus status;
} DataHandshakeResult;

DataHandshakeResult perform_data_handshake(PinData *pindata, uint64_t blacklist_mask);

#endif // DATA_HANDSHAKE_H