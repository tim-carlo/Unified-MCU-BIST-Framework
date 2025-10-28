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
#define GPIO0       PIN_UART_RX
#define GPIO1       PIN_UART_TX
#define GPIO2       (8 * 2 + 3) // P2.3
#define GPIO3       (8 * 2 + 4) //   .4
#define GPIO4       (8 * 4 + 6) // P4.6
#define GPIO5       (8 * 3 + 6) // P3.6
#define GPIO6       (8 * 0 + 6) // PJ.6
#define GPIO7       (8 * 5 + 3) // P5.3
#define GPIO8       (8 * 5 + 2) //   .2
#define GPIO9       (8 * 5 + 1) //   .1
#define GPIO10      (8 * 5 + 0) //   .0
#define GPIO11      (8 * 6 + 0) // P6.0
#define GPIO12      (8 * 6 + 1) //   .1
#define GPIO13      (8 * 6 + 3) //   .3
#define GPIO14      (8 * 6 + 6) //   .6
#define GPIO15      (8 * 6 + 7) //   .7
#define PWRGDL      (8 * 5 + 4) // P5.4
#define PWRGDH      (8 * 5 + 5) //   .5

#define PIN_LED0    (8 * 5 + 7) // P5.7 -> powered externally
//#define PIN_LED1    (40) // P5.0 -> powered externally
#define PIN_LED2    (8 * 0 + 0) // PJ.0 -> burns energy-budget

#define I2C_SCL     (8 * 6 + 5) // P6.05
#define I2C_SDA     (8 * 6 + 4)
//#define RTC_INT     (8 * 7 + 3) // output of RTC, not controllable
#define MAX_INT     (8 * 0 + 1) // output of MAX-IC

#define C2C_CLK     (8 * 1 + 5)
#define C2C_CoPi    (8 * 2 + 0)
#define C2C_CiPo    (8 * 2 + 1)
#define C2C_PSel    (8 * 1 + 4)
#define C2C_GPIO    (8 * 0 + 2)

#define THRCTRL_H0  (8 * 1 + 3)
#define THRCTRL_H1  (8 * 3 + 3)
#define THRCTRL_L0  (8 * 6 + 2)
#define THRCTRL_L1  (8 * 7 + 0)

#define DEBUG_PIN1 GPIO14 // Additional debug pin, can be changed as needed
#define DEBUG_PIN2 GPIO15 // Additional debug pin, can be changed as needed


// extern uint32_t mcu_leds[]  = {PIN_LED0, PIN_LED2};
// extern uint32_t mcu_i2c[]   = {I2C_SCL, I2C_SDA, /*RTC_INT,*/ MAX_INT};
// extern uint32_t mcu_c2c[]   = {C2C_CLK, C2C_CoPi, C2C_CiPo, C2C_PSel, C2C_GPIO};
// extern uint32_t mcu_thr[]   = {THRCTRL_H0, THRCTRL_H1, THRCTRL_L0, THRCTRL_L1};
#endif




#else // Production Board

#if defined(NRF52840_XXAA)
#define PIN_UART_TX 6
#define PIN_UART_RX 8

#define DEBUG_PIN1 19
#define DEBUG_PIN2 20

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