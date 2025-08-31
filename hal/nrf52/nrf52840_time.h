#ifndef NRF52840_TIME_H
#define NRF52840_TIME_H

#include "nrf52840.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Timer configuration and control
void configure_timer(NRF_TIMER_Type *const timer, const uint32_t prescaler, const uint32_t bitmode);
void configure_timer_for_measurement(NRF_TIMER_Type *const timer);
void set_timer_compare(NRF_TIMER_Type *const timer, const uint32_t channel, const uint32_t value, const bool enable_interrupt, const bool clear_on_compare);

// Timer control
void start_timer(NRF_TIMER_Type *const timer);
void stop_timer(NRF_TIMER_Type *const timer);
void reset_timer(NRF_TIMER_Type *const timer);
void reset_and_start_timer(NRF_TIMER_Type *const timer);

// Timer reading
uint32_t get_timer_ticks(NRF_TIMER_Type *const timer);

// Time measurement functions
void start_time_measurement(NRF_TIMER_Type *const timer);
uint32_t stop_time_measurement_us(NRF_TIMER_Type *const timer);
uint32_t stop_time_measurement_ms(NRF_TIMER_Type *const timer);
uint32_t get_elapsed_time_us(NRF_TIMER_Type *const timer);
uint32_t get_elapsed_time_ms(NRF_TIMER_Type *const timer);

// Utility functions
uint32_t get_elapsed_time(uint32_t start, uint32_t current);

// Delay functions - parameter ohne const für Performance
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

// Conversion functions - parameter ohne const für Performance
uint32_t us_to_ticks(uint32_t us);
uint32_t ms_to_ticks(uint32_t ms);
uint32_t ticks_to_us_simple(uint32_t ticks);
uint32_t ticks_to_ms_simple(uint32_t ticks);

// Legacy functions - 64-bit parameter bleiben ohne const
uint32_t ticks_to_us(uint64_t ticks);
uint32_t ticks_to_ms(uint64_t ticks);
uint32_t timer_diff_us(uint64_t start, uint64_t end);
uint32_t timer_diff_ms(uint64_t start, uint64_t end);

// Callback functions
void set_timer_event_callback(NRF_TIMER_Type *const timer, void (*const callback)(void));
void clear_timer_event_callback(NRF_TIMER_Type *const timer);

#ifdef __cplusplus
}
#endif

#endif // NRF52840_TIME_H