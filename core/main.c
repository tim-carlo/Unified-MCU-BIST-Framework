
#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h"
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#include "msp430fr5994_utils.h"
#include "msp430fr5994_uart.h"
#include "printf.h"
#include "crc.h"
#include <stdbool.h>
#include <stdint.h>
#endif

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_time.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"
#include "nrf52840_uart.h"
#include "printf.h"

#endif

#include "pin_config.h"
#include "stack.h"
#include "timing_pindata.h"

#include "manchester.h"
#include "handshake.h"
#include "data_handshake.h"
#include "serialisation.h"
#include "uart_transmitter.h"
#include "set_one_high_measure_all.h"
#include "mutex_handeler.h"

#include "crc.h"
#include <inttypes.h>

#define DEBUG 1 // Set to 1 to enable debug logging, 0 to disable
#if DEBUG == 1
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define LOG(fmt, ...)
#endif

// Global variables
PinData pin_data[NUMBER_OF_GPIO_PINS]; // Global variable to hold pin data

// Flags controlled via interrupts
volatile uint64_t initial_state_mask = 0; // Global blacklist mask for GPIO pins
volatile uint64_t handshake_mask = 0;    // Mask used during handshake

DataHandshakeResult data_handshake_result_test;

void set_standart_blacklist_pins(volatile uint64_t *mask)
{
    *mask = 0xFFFFFFFFFFFFFFFFULL;

#if defined(NRF52840_XXAA)
    *mask &= ~(1ULL << 11); // Pin 11
    *mask &= ~(1ULL << 12); // Pin 12
    *mask &= ~(1ULL << 13); // Pin 13
    *mask &= ~(1ULL << 14); // Pin 14
    *mask &= ~(1ULL << 15); // Pin 15
    *mask &= ~(1ULL << 16); // Pin 16
    data_handshake_result_test.mutex_pin = 12;
    data_handshake_result_test.i_am_mutex_owner = true;

#elif defined(__MSP430FR5994__)
    *mask &= ~(1ULL << ABS_PIN(3, 7)); // Pin 23
    *mask &= ~(1ULL << ABS_PIN(3, 6)); // Pin 22
    *mask &= ~(1ULL << ABS_PIN(3, 5)); // Pin 21
    *mask &= ~(1ULL << ABS_PIN(3, 4)); // Pin 20
    *mask &= ~(1ULL << ABS_PIN(2, 6)); // Pin 19
    *mask &= ~(1ULL << ABS_PIN(7, 3));
#endif
}

#if DEV_KIT == 0

const uint8_t array[] = {
    GPIO0,
    /*GPIO1,*/ // leave UART TX pin blacklisted
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5,
    GPIO6,
    GPIO7,
    GPIO8,
    GPIO9,
    GPIO10,
    GPIO11,
    // GPIO12,
    // GPIO13,
    // GPIO14,
    // GPIO15,
    PWRGDL,
    PWRGDH,
    PIN_LED0,
    PIN_LED2,
    I2C_SCL,
    I2C_SDA,
    /*RTC_INT,*/ MAX_INT,
    C2C_CLK,
    C2C_CoPi,
    C2C_CiPo,
    C2C_PSel,
    C2C_GPIO,
    THRCTRL_H0,
    THRCTRL_H1,
    THRCTRL_L0,
    THRCTRL_L1,
};

void set_handshake_pins() 
{
    handshake_mask = 0xFFFFFFFFFFFFFFFFULL; // Start with all pins blacklisted
    // white list all pins from array:
    for (size_t i = 0; i < (sizeof(array) / sizeof(array[0])); i++)
    {
        handshake_mask &= ~(1ULL << array[i]);
    }
}
#endif
void set_shepherd_pins()
{
    // Set every pin except the pins for SWDIO, SWDCLK, UART TX
    // The document says some Pins should be configured in standard drive mode
    // So we set them maximal to standard drive mode
    // So blacklisting for aQFN73: On the dev kits the pin P0.18 is used for reset
    // Programming pins cant be overwritten:

    // On the MSP430FR5994:
    // None of the Pins must be blacklisted since all are available for GPIO use
    // The Spy Bi Wire pins must not be blacklisted

    initial_state_mask = 0ULL; // whitelist all pins

    // Black list all pins that don't exist
    for (uint8_t pin = NUMBER_OF_GPIO_PINS; pin < 64; pin++)
    {
        initial_state_mask |= (1ULL << pin);
    }
}

void led0_show_error()
{
    gpio_output_init(PIN_LED0);
    for (uint8_t i = 0; i < 5; i++)
    {
        gpio_drive_high(PIN_LED0);
        delay_ms(200);
        gpio_drive_low(PIN_LED0);
        delay_ms(200);
    }
    gpio_reset(PIN_LED0);
}
void led2_show_error()
{
    gpio_output_init(PIN_LED2);
    for (uint8_t i = 0; i < 5; i++)
    {
        gpio_drive_high(PIN_LED2);
        delay_ms(200);
        gpio_drive_low(PIN_LED2);
        delay_ms(200);
    }
    gpio_reset(PIN_LED2);
}

void perfom_mutex_operations()
{
    run_set_one_high_measure_all(initial_state_mask, pin_data, NUMBER_OF_GPIO_PINS);

    uart_transmitter_init();
    UartTransmissionResult uart_result = send_complete_transmission_no_ack(pin_data, NUMBER_OF_GPIO_PINS);
    switch (uart_result)
    {
    case UART_TRANSMISSION_ERROR_SEND_FAILED:
        printf("DEBUG: UART send failed\n");
        led2_show_error();
        break;
    case UART_TRANSMISSION_ERROR_ACK_FAILED:
        printf("DEBUG: UART ACK failed\n");
    default:
        break;
    }
}

int main(void)
{
    mcu_init();
    // test of mutex handeler
    printf("Size of PinData: %zu bytes\n", sizeof(PinData));

    DataHandshakeResult test_data_handshake_result;
#if defined(NRF52840_XXAA)
    test_data_handshake_result.mutex_pin = 12;
    test_data_handshake_result.i_am_mutex_owner = true;

#elif defined(__MSP430FR5994__)
    test_data_handshake_result.mutex_pin = ABS_PIN(2, 6);
    test_data_handshake_result.i_am_mutex_owner = false;
#endif

    set_shepherd_pins();
    set_handshake_pins();
    LOG("DEBUG: Starting handshake process\n");


    initialize_pin_data_array(pin_data, NUMBER_OF_GPIO_PINS);

    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (handshake_mask & (1ULL << pin))
        {
            continue;
        }
        gpio_od_init(pin);
    }

    HandshakeResult handshake_result = perform_handshake(pin_data, handshake_mask);

    if (handshake_result == HANDSHAKE_ISR_TIMEOUT)
    {
        LOG("DEBUG: HANDSHAKE ISR TO LONG\n");
        return 1; // Handshake failed, exit program
    }

    // if no working pin found, exit program, but run initial tests first
    if (handshake_result == HANDSHAKE_NO_WORKING_PIN_FOUND)
    {
        LOG("DEBUG: NO WORKING PIN FOUND DURING HANDSHAKE\n");
        perfom_mutex_operations();
        return 1; // Handshake failed, exit program
    }

    LOG("DEBUG: PERFORMING DATA HANDSHAKE\n");
    DataHandshakeResult data_handshake_result = perform_data_handshake(pin_data, handshake_mask);
    LOG("DEBUG: Data handshake result status: %u, mutex_pin: %u, i_am_mutex_owner: %u\n",
        data_handshake_result.status,
        data_handshake_result.mutex_pin,
        data_handshake_result.i_am_mutex_owner);
    if (data_handshake_result.status != DATA_HANDSHAKE_SUCCESS)
    {
        // Reason data handshake failed
        LOG("DEBUG: Data handshake failed with status %u\n", data_handshake_result.status);

        perfom_mutex_operations();
        return 1; // Handshake failed, exit program
    }

    if (data_handshake_result.mutex_pin == 255)
    {
        LOG("DEBUG: No mutex pin assigned\n");

        uart_transmitter_init();
        UartTransmissionResult uart_result = send_complete_transmission_no_ack(pin_data, NUMBER_OF_GPIO_PINS);
        return 0; // No mutex pin assigned, exit program
    }
    MutexHandler mutex_handler;

    mutex_handler_init(&data_handshake_result, &mutex_handler);
    mutex_handler_request_mutex(initial_state_mask, &mutex_handler);
    LOG("DEBUG: now having mutex\n");
    perfom_mutex_operations();
    LOG("DEBUG: releasing mutex\n");
    mutex_handler_release_mutex(initial_state_mask, &mutex_handler);

    return 0;
}