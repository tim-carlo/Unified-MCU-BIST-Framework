#ifndef DATA_HANDSHAKE_H
#define DATA_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#define DATA_TIMER TIMER_B0


#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#include "nrf52840_time.h"
#include "nrf52840_gpio.h"
#include "nrf52840_utils.h"
#define DATA_TIMER NRF_TIMER2


#endif

#include "datahandshake_modulation.h"
#include "printf.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "timing_pindata.h"
#include "pindata.h"
#include "bitmap_iterator.h"
#include "random_utils.h"
#include "crc.h"
#include "pin_config.h"


#define REQUEST_PACKSIZE (15)
#define ANSWER_PACKSIZE (25)

#define REQEST_MUTEX_ON_THIS_PIN (0x55)
#define ALLOWING_MUTEX_ON_THIS_PIN (0xAA)
#define DENYING_MUTEX_ON_THIS_PIN (0x11)
#define NO_MUTEX_ON_THIS_PIN (0x00)

typedef enum
{
    PACKET_TYPE_REQUEST,
    PACKET_TYPE_ANSWER,
} PackageType;


typedef enum
{
    DATA_HANDSHAKE_INITIALIZING_FAILURE,
    DATA_HANDSHAKE_ISR_TO_LONG, // This failure is triggered when the ISR takes too long
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