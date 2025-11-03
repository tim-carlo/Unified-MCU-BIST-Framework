
#ifndef NRF52840_HELPER_H
#define NRF52840_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf.h"
#include "nrf52840.h"
#include "stack.h"
#include "pindata.h"
#include "printf.h"
#include "nrf52840_uart.h"



#define DEBOUNCE_SAMPLES (5)
#define DEBOUNCE_DELAY_US (20)

#define BV(pos) (1u << (pos))
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)

#define TIMER_A NRF_TIMER0
#define TIMER_B NRF_TIMER1
#define INVALID_PIN 255 // Invalid pin number


void mcu_init(void);

#endif // NRF52840_HELPER_H