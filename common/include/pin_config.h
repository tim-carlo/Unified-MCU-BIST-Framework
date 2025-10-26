#pragma once

#if defined(__MSP430FR5994__)
#include "msp430fr5994_helper.h" // used for the abs pin calculation
#elif defined(NRF52840_XXAA)
#include "nrf52840_helper.h"
#endif

#define DEV_KIT 1

#if (DEV_KIT == 0)

#if defined(NRF52840_XXAA)
//  UART Config for NRF52840 from https://github.com/nes-lab/shepherd-targets/blob/main/firmware/nrf52_testable/src/main.c
// Adapted pinout for Target V1.2 (Riotee)
#define PIN_UART_TX (8)  // P0.08
#define PIN_UART_RX (21) // P0.21

#define GPIO0 PIN_UART_RX
#define GPIO1 PIN_UART_TX
#define GPIO2 (4)
#define GPIO3 (5)
#define GPIO4 (32 + 9) // P1.09
#define GPIO5 (26)
#define GPIO6 (32 + 3) // P1.03
#define GPIO7 (11)
#define GPIO8 (13)
#define GPIO9 (16)
#define GPIO10 (12)
#define GPIO11 (10)
#define GPIO12 (19)
#define GPIO13 (20)
#define GPIO14 (24)
#define GPIO15 (27)
#define PWRGDL (23)
#define PWRGDH (7)

#define PIN_LED0 (32 + 13) // P1.13 -> powered externally
// #define PIN_LED1    12 // P0.12 -> powered externally
#define PIN_LED2 (3) // P0.03 -> burns energy-budget

#define I2C_SCL (32 + 8) // P1.08
#define I2C_SDA (6)
#define RTC_INT (30) // output of RTC, not controllable
#define MAX_INT (25)

#define C2C_CLK (18)
#define C2C_CoPi (17)
#define C2C_CiPo (14)
#define C2C_PSel (22)
#define C2C_GPIO (15)

#define THRCTRL_H0 (9)
#define THRCTRL_H1 (32 + 2)
#define THRCTRL_L0 (32 + 7)
#define THRCTRL_L1 (32 + 4)

#define DEBUG_PIN1 GPIO12 // Pin used for debugging, can be changed as needed
#define DEBUG_PIN2 GPIO13 // Pin used for debugging, can be changed as needed 
#elif defined(__MSP430FR5994__)
// UART Config for MSP430FR5994
// Adapted pinout for Target V1.2 (Riotee)
#define PIN_UART_TX ABS_PIN(2, 5) // P2.5
#define PIN_UART_RX ABS_PIN(2, 6) // P2.6

#define GPIO0 PIN_UART_RX
#define GPIO1 PIN_UART_TX
#define GPIO2 ABS_PIN(2, 3)  // P2.3
#define GPIO3 ABS_PIN(2, 4)  // P2.4
#define GPIO4 ABS_PIN(4, 6)  // P4.6
#define GPIO5 ABS_PIN(3, 6)  // P3.6
#define GPIO6 ABS_PIN(0, 6)  // PJ.6
#define GPIO7 ABS_PIN(5, 3)  // P5.3
#define GPIO8 ABS_PIN(5, 2)  // P5.2
#define GPIO9 ABS_PIN(5, 1)  // P5.1
#define GPIO10 ABS_PIN(5, 0) // P5.0
#define GPIO11 ABS_PIN(6, 0) // P6.0
#define GPIO12 ABS_PIN(6, 1) // P6.1
#define GPIO13 ABS_PIN(6, 3) // P6.3
#define GPIO14 ABS_PIN(6, 6) // P6.6
#define GPIO15 ABS_PIN(6, 7) // P6.7
#define PWRGDL ABS_PIN(5, 4) // P5.4
#define PWRGDH ABS_PIN(5, 5) // P5.5

#define PIN_LED0 ABS_PIN(5, 7) // P5.7 -> powered externally
// #define PIN_LED1    ABS_PIN(5, 0) // P5.0 -> powered externally
#define PIN_LED2 (0) // PJ.0 -> burns energy-budget

#define I2C_SCL ABS_PIN(6, 5) // P6.5
#define I2C_SDA ABS_PIN(6, 4) // P6.4
// #define RTC_INT     ABS_PIN(7, 3) // output of RTC, not controllable
#define MAX_INT ABS_PIN(0, 1) // output of MAX-IC

#define C2C_CLK ABS_PIN(1, 5)  // P1.5
#define C2C_CoPi ABS_PIN(2, 0) // P2.0
#define C2C_CiPo ABS_PIN(2, 1) // P2.1
#define C2C_PSel ABS_PIN(1, 4) // P1.4
#define C2C_GPIO ABS_PIN(0, 2) // PJ.2

#define THRCTRL_H0 ABS_PIN(1, 3) // P1.3
#define THRCTRL_H1 ABS_PIN(3, 3) // P3.3
#define THRCTRL_L0 ABS_PIN(6, 2) // P6.2
#define THRCTRL_L1 ABS_PIN(7, 0) // P7.0

#define DEBUG_PIN1 GPIO14 // Additional debug pin, can be changed as needed
#define DEBUG_PIN2 GPIO15 // Additional debug pin, can be changed as needed
#endif




#else // Production Board

#if defined(NRF52840_XXAA)
#define PIN_UART_TX 6
#define PIN_UART_RX 8

#define DEBUG_PIN1 19
#define DEBUG_PIN2 20
#define DEBUG_PIN3 21
#define DEBUG_PIN4 22

#define GPIO0 0
#define GPIO1 1
#define GPIO2 2
#define GPIO3 3
#define GPIO4 4
#define GPIO5 5
#define GPIO6 7
#define GPIO7 9
#define GPIO8 10
#define GPIO9 15
#define GPIO10 16
#define GPIO11 17
#define GPIO12 18

#define PIN_LED2 26
#define PIN_LED0 27


#elif defined(__MSP430FR5994__)
#define PIN_UART_TX ABS_PIN(2, 0) // P2.0
#define PIN_UART_RX ABS_PIN(2, 1) // P2.1

#define DEBUG_PIN1 ABS_PIN(7, 0) // P3.0
#define DEBUG_PIN2 ABS_PIN(7, 1) // P3.1

#define PIN_LED0 ABS_PIN(1, 0)
#define PIN_LED2 ABS_PIN(1, 1)
#define GPIO0 ABS_PIN(3, 0)
#define GPIO1 ABS_PIN(3, 1)
#define GPIO2 ABS_PIN(3, 2)
#define GPIO3 ABS_PIN(3, 3)
#define GPIO4 ABS_PIN(3, 4)
#define GPIO5 ABS_PIN(3, 5)
#define GPIO6 ABS_PIN(3, 6)
#define GPIO7 ABS_PIN(3, 7)
#define GPIO8 ABS_PIN(5, 2)
#define GPIO9 ABS_PIN(5, 3)
#define GPIO10 ABS_PIN(5, 7)
#define GPIO11 ABS_PIN(6, 0)
#define GPIO12 ABS_PIN(6, 1)
#define GPIO13 ABS_PIN(6, 2)
#define GPIO14 ABS_PIN(6, 3)

#endif

#endif // DEV_KIT