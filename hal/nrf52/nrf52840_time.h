#ifndef NRF52840_TIME_H
#define NRF52840_TIME_H

#include <stdint.h>
#include "nrf.h"
#include "nrf52840.h"

#ifdef __cplusplus
extern "C" {
#endif


uint32_t get_elapsed_time(uint32_t start, uint32_t current);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
uint32_t get_timer_ticks(NRF_TIMER_Type *timer);
void start_timer(NRF_TIMER_Type *timer);
void stop_timer(NRF_TIMER_Type *timer);
void reset_timer(NRF_TIMER_Type *timer);
uint32_t ticks_to_us(uint64_t ticks);
uint32_t ticks_to_ms(uint64_t ticks);
uint32_t timer_diff_us(uint64_t start, uint64_t end);
uint32_t timer_diff_ms(uint64_t start, uint64_t end);


#ifdef __cplusplus
}
#endif

#endif // NRF52840_TIME_H