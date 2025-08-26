#ifndef NRF52840_TIME_H
#define NRF52840_TIME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf52840.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void configure_timer(NRF_TIMER_Type *const timer, const uint32_t prescaler, const uint32_t bitmode);
    void configure_timer_for_measurement(NRF_TIMER_Type *timer);
    void set_timer_compare(NRF_TIMER_Type *const timer, const uint32_t channel, const uint32_t value, const bool enable_interrupt, const bool clear_on_compare);

    void start_timer(NRF_TIMER_Type *timer);
    void stop_timer(NRF_TIMER_Type *timer);
    void reset_timer(NRF_TIMER_Type *timer);
    void reset_and_start_timer(NRF_TIMER_Type *timer);

    uint32_t get_timer_ticks(NRF_TIMER_Type *timer);
    uint32_t get_elapsed_time(uint32_t start, uint32_t current);

    void start_time_measurement(NRF_TIMER_Type *timer);
    uint32_t stop_time_measurement_us(NRF_TIMER_Type *timer);
    uint32_t stop_time_measurement_ms(NRF_TIMER_Type *timer);
    uint32_t get_elapsed_time_us(NRF_TIMER_Type *timer);
    uint32_t get_elapsed_time_ms(NRF_TIMER_Type *timer);

    void delay_us(uint32_t us);
    void delay_ms(uint32_t ms);

    uint32_t us_to_ticks(uint32_t us);
    uint32_t ms_to_ticks(uint32_t ms);
    uint32_t ticks_to_us_simple(uint32_t ticks);
    uint32_t ticks_to_ms_simple(uint32_t ticks);

    uint32_t ticks_to_us(uint64_t ticks);
    uint32_t ticks_to_ms(uint64_t ticks);
    uint32_t timer_diff_us(uint64_t start, uint64_t end);
    uint32_t timer_diff_ms(uint64_t start, uint64_t end);

    void set_timer_event_callback(NRF_TIMER_Type *const timer, void (*const callback)(void));
    void clear_timer_event_callback(NRF_TIMER_Type *const timer);

#ifdef __cplusplus
}
#endif

#endif // NRF52840_TIME_H