#ifndef DATA_HANDSHAKE_H
#define DATA_HANDSHAKE_H

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_utils.h"
#define DATA_TIMER TIMER_B0

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
#define DATA_TIMER NRF_TIMER3
#define DEBUG_PIN1 26 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 27 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN3 39 // Additional debug pin, can be changed as needed
#define DEBUG_PIN4 40 // Additional debug pin, can be changed as needed

#endif

#include "manchester.h"
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

#define DATA_TIMER_INTERVAL_US 1000 // 1ms interval for both platforms

typedef struct
{
    uint8_t pin;
    uint8_t number_of_successful_tries;
    uint8_t *data;
} DataHandshakeData;

// Packet format: [8 bytes UUID][1 byte Pin][4 bytes CRC]
typedef struct
{
    uint64_t uuid;
    uint8_t pin;
    crc crc_value;
} __attribute__((packed)) RequestDataPacket; // packed to avoid padding

// Packet format: [8 bytes Received UUID][1 byte received Pin][8 bytes own UUID][1 byte sending Pin][4 bytes CRC]
typedef struct
{
    uint64_t received_uuid;
    uint8_t received_pin;
    uint64_t own_uuid;
    uint8_t sending_pin;
    crc crc_value;
} __attribute__((packed)) AnswerDataPacket; // packed to avoid padding

typedef enum
{
    PACKET_TYPE_REQUEST,
    PACKET_TYPE_ANSWER,
} PackageType;

void perform_data_handshake(PinData *pindata, uint64_t blacklist_mask);

#endif // DATA_HANDSHAKE_H