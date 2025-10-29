
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
#define NUMBER_OF_GPIO_PINS MSP430_NUM_ABS_PINS
#endif

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_time.h"
#include "nrf52840_helper.h"
#include "nrf52840_utils.h"
#include "nrf52840_gpio.h"
#include "nrf52840_uart.h"
#include "printf.h"
#define NUMBER_OF_GPIO_PINS NRF52_NUM_PINS

#endif

#include "pin_config.h"
#include "stack.h"
#include "timing_pindata.h"
#include "check_initial_state.h"
#include "manchester.h"
#include "random_utils.h"
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

// Inspired from Hacker’s Delight by Henry S. Warren, Jr.

DataHandshakeResult data_handshake_result_test;

void set_standart_blacklist_pins(volatile uint64_t *mask)
{
    *mask = 0xFFFFFFFFFFFFFFFFULL;

#if defined(NRF52840_XXAA)
    *mask &= ~(1ULL << 12); // Pin 12
    *mask &= ~(1ULL << 11); // Pin 11
  //  *mask &= ~(1ULL << 13); // Pin 13
  //  *mask &= ~(1ULL << 14); // Pin 14
  //  *mask &= ~(1ULL << 15); // Pin 15
  //  *mask &= ~(1ULL << 16); // Pin 16
 //  *mask &= ~(1ULL << 17); // Pin 17
 //   *mask &= ~(1ULL << 18); // Pin 18
    data_handshake_result_test.mutex_pin = 12;
    data_handshake_result_test.i_am_mutex_owner = true;

#elif defined(__MSP430FR5994__)
    *mask &= ~(1ULL << ABS_PIN(3, 7)); // Pin 23
    *mask &= ~(1ULL << ABS_PIN(3, 6)); // Pin 22
   // *mask &= ~(1ULL << ABS_PIN(3, 4)); // Pin 20
   // *mask &= ~(1ULL << ABS_PIN(2, 6)); // Pin 19
  //  *mask &= ~(1ULL << ABS_PIN(2, 5)); // Pin 18
   // *mask &= ~(1ULL << ABS_PIN(4, 3)); // Pin 17
  //  *mask &= ~(1ULL << ABS_PIN(4, 2)); // Pin 16
   // *mask &= ~(1ULL << ABS_PIN(4, 1)); // Pin 15
#endif
}

#if DEV_KIT == 0

const uint8_t array[] = {
    // GPIO0,
    /*GPIO1,*/
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
    GPIO12,
    GPIO13,
  //  GPIO14,
   // GPIO15,
    // PWRGDL,
    // PWRGDH,
    // PIN_LED0,
    // PIN_LED2,
    // I2C_SCL,
    // I2C_SDA,
    // /*RTC_INT,*/ MAX_INT,
    // C2C_CLK,
    // C2C_CoPi,
    // C2C_CiPo,
    // C2C_PSel,
    // C2C_GPIO,
    // THRCTRL_H0,
    // THRCTRL_H1,
    // THRCTRL_L0,
    // THRCTRL_L1,
};
#endif
void set_shepherd_pins()
{
    initial_state_mask = 0xFFFFFFFFFFFFFFFFULL; // Start with all pins blacklisted
    initial_state_mask &= ~(1ULL << GPIO2);
    initial_state_mask &= ~(1ULL << GPIO3);
    initial_state_mask &= ~(1ULL << GPIO4);
    initial_state_mask &= ~(1ULL << GPIO5);
    initial_state_mask &= ~(1ULL << GPIO6);
    initial_state_mask &= ~(1ULL << GPIO7);
    initial_state_mask &= ~(1ULL << GPIO8);
    initial_state_mask &= ~(1ULL << GPIO9);
    initial_state_mask &= ~(1ULL << GPIO10);
    initial_state_mask &= ~(1ULL << GPIO11);
  //  initial_state_mask &= ~(1ULL << GPIO12);
  //  initial_state_mask &= ~(1ULL << GPIO13);
   // initial_state_mask &= ~(1ULL << GPIO14);
   // initial_state_mask &= ~(1ULL << GPIO15);
}

void set_pins_test_env_pins()
{
    initial_state_mask = 0xFFFFFFFFFFFFFFFFULL; // Start with all pins blacklisted

    // initial_state_mask &= ~(1ULL << GPIO0);
    // initial_state_mask &= ~(1ULL << GPIO1);
    // initial_state_mask &= ~(1ULL << GPIO2);
    // initial_state_mask &= ~(1ULL << GPIO3);
    //  initial_state_mask &= ~(1ULL << GPIO6);
    //  initial_state_mask &= ~(1ULL << GPIO7);
    //  initial_state_mask &= ~(1ULL << GPIO8);
    //  initial_state_mask &= ~(1ULL << GPIO9);
    //  initial_state_mask &= ~(1ULL << GPIO10);
    //   initial_state_mask &= ~(1ULL << GPIO11);
    //    initial_state_mask &= ~(1ULL << GPIO12);
    //  initial_state_mask &= ~(1ULL << GPIO13);
    //  initial_state_mask &= ~(1ULL << GPIO14);
    //  initial_state_mask &= ~(1ULL << GPIO15);
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
}

void perfom_mutex_operations()
{

    // reset all pins to clean state
    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        gpio_reset(pin);
    }
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

    // uart_transmitter_init();
    // UartTransmissionResult uart_result = send_complete_transmission_with_ack(pin_data, NUMBER_OF_GPIO_PINS);

    // switch (uart_result)
    // {
    // case UART_TRANSMISSION_ERROR_SEND_FAILED | UART_TRANSMISSION_ERROR_ACK_FAILED:
    //     printf("DEBUG: UART RX not working\n");
    //     add_pin_event(pin_data, PIN_UART_RX, UART_RX_IS_NOT_WORKING);
    //     send_complete_transmission_no_ack(pin_data, NUMBER_OF_GPIO_PINS);
    //     led2_show_error();
    //     break;
    // case UART_TRANSMISSION_ERROR_INIT_FAILED:
    //     led2_show_error();
    //     break;
    // case UART_TRANSMISSION_MEMORY_ALLOCATION_FAILED:
    //     led2_show_error();
    //     break;
    // case UART_TRANSMISSION_ERROR_NULL_POINTER:
    //     led2_show_error();
    //     break;
    // default:
    //     break;
    // }
}

int main(void)
{
    // #if defined(NRF52840_XXAA)

    //     // Configure all GPIO pins as inputs
    //     for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    //     {
    //         gpio_input_init(pin, GPIO_PULL);  // set pin as input
    //     }
    //     while(1) {}
    // #endif
    mcu_init();

#if (DEV_KIT == 0)
    set_shepherd_pins();
#if defined(__MSP430FR5994__)
    led0_show_error();
#elif defined(NRF52840_XXAA)
    led2_show_error();

    // // Configure all GPIO pins as inputs (with pull) and stop
    // for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    // {
    //     gpio_input_init(pin, GPIO_NO_PULL);
    // }
    // while (1)
    // {
    //     // idle forever with all pins configured as inputs
    // }

#endif
#else
    // set_standart_blacklist_pins(&initial_state_mask);
    set_standart_blacklist_pins(&initial_state_mask);
#endif

    // size_t all_count = sizeof(all) / sizeof(all[0]);

    // // configure as open-drain for each pin in all[]
    // for (size_t i = 0; i < all_count; ++i)
    // {
    //     uint32_t pin = all[i];
    //     gpio_od_init((uint8_t)pin);
    // }
    // delay_ms(50);

    // // briefly drive low each pin in all[]
    // for (size_t i = 0; i < all_count; ++i)
    // {
    //     uint32_t pin = all[i];
    //     LOG("DEBUG: OD test pin %u\n", (unsigned)pin);
    //     gpio_od_hold_low((uint8_t)pin);
    //     delay_ms(10);
    //     gpio_od_release((uint8_t)pin);
    // }
    // delay_ms(50);
    // size_t array_size = sizeof(array) / sizeof(array[0]);

    // /* set array to output */
    // for (uint8_t count = 0; count < array_size; count++)
    // {
    //     //set_gpio_out(array[count], true);
    //     gpio_output_init(array[count]);
    // }
    // delay_ms(100);
    // /* switch each pin on in array */
    // for (uint8_t count = 0; count < array_size; count++)
    // {
    //     gpio_drive_high(array[count]);
    //     //gpio_drive_high(array[count]);
    //     delay_ms(100);
    //     //set_gpio_state(array[count], false);
    //     gpio_drive_low(array[count]);
    // }
    // /* set pins to INPUT */
    // delay_ms(100);
    // for (uint8_t count = 0; count < array_size; count++)
    // {
    //     //set_gpio_out(array[count], false);
    //     gpio_reset(array[count]);
    // }

    LOG("DEBUG: Starting handshake process\n");

    initialize_pin_data_array(pin_data, NUMBER_OF_GPIO_PINS);

    gpio_output_init(DEBUG_PIN1);
    gpio_output_init(DEBUG_PIN2);
    for (uint8_t pin = 0; pin < NUMBER_OF_GPIO_PINS; ++pin)
    {
        if (initial_state_mask & (1ULL << pin))
            continue;      // Skip blacklisted pins (bit = 1)
        gpio_od_init(pin); // Initialize non-blacklisted pins (bit = 0) with pull-up resistors
    }

    HandshakeResult handshake_result = perform_handshake(pin_data, initial_state_mask);

    // if no working pin found, exit program, but run initial tests first
    if (handshake_result == HANDSHAKE_NO_WORKING_PIN_FOUND)
    {
        perfom_mutex_operations();
        return 1; // Handshake failed, exit program
    }

    LOG("DEBUG: PERFORMING DATA HANDSHAKE\n");
    DataHandshakeResult data_handshake_result = perform_data_handshake(pin_data, initial_state_mask);
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
    mutex_handeler_init(&data_handshake_result);
    mutex_handler_request_mutex(initial_state_mask);
    LOG("DEBUG: now having mutex\n");

    perfom_mutex_operations();

    // TODO Handle different error codes
    mutex_handler_release_mutex(initial_state_mask);
    LOG("DEBUG: released mutex\n");
    return 0;
}