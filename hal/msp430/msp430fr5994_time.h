#ifndef MSP430FR5994_TIME_H
#define MSP430FR5994_TIME_H

#include <msp430.h>
#include <msp430fr5994.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stack.h"

typedef enum
{
    TIMER_A0,
    TIMER_A1,
    TIMER_A2,
    TIMER_A4,
    TIMER_B0
} timer_type;

extern volatile uint16_t timer_overflows_a1;
extern volatile uint16_t timer_overflows_a2;
extern volatile uint16_t timer_overflows_a4;
extern volatile uint16_t timer_overflows_b0;

#define GET_TxxCTL(timer) (                                                                                 \
    (timer) == TIMER_A0 ? (volatile uint16_t *)&TA0CTL : (timer) == TIMER_A1 ? (volatile uint16_t *)&TA1CTL \
                                                     : (timer) == TIMER_A2   ? (volatile uint16_t *)&TA2CTL \
                                                     : (timer) == TIMER_A4   ? (volatile uint16_t *)&TA4CTL \
                                                     : (timer) == TIMER_B0   ? (volatile uint16_t *)&TB0CTL \
                                                                             : (volatile uint16_t *)0)

#define GET_TxxR(timer) (                                                                               \
    (timer) == TIMER_A0 ? (volatile uint16_t *)&TA0R : (timer) == TIMER_A1 ? (volatile uint16_t *)&TA1R \
                                                   : (timer) == TIMER_A2   ? (volatile uint16_t *)&TA2R \
                                                   : (timer) == TIMER_A4   ? (volatile uint16_t *)&TA4R \
                                                   : (timer) == TIMER_B0   ? (volatile uint16_t *)&TB0R \
                                                                           : (volatile uint16_t *)0)

#define GET_TxxCCTL0(timer)                                                                  \
    ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CCTL0 : (timer) == TIMER_A1 ? &TA1CCTL0 \
                                                         : (timer) == TIMER_A2   ? &TA2CCTL0 \
                                                         : (timer) == TIMER_A4   ? &TA4CCTL0 \
                                                                                 : &TB0CCTL0))

#define GET_TxxCCR0(timer)                                                                 \
    ((volatile uint16_t *)((timer) == TIMER_A0 ? &TA0CCR0 : (timer) == TIMER_A1 ? &TA1CCR0 \
                                                        : (timer) == TIMER_A2   ? &TA2CCR0 \
                                                        : (timer) == TIMER_A4   ? &TA4CCR0 \
                                                                                : &TB0CCR0))

// Timer configuration
#define TIMER_DIVIDER 8
#define TIMER_FREQ_HZ (SMCLK_HZ / TIMER_DIVIDER) // 16MHz / 8 = 2MHz
#define TICKS_PER_OVERFLOW 65536UL               // 16-bit Timer overflow (2^16)

#ifndef SMCLK_HZ
#define SMCLK_HZ 16000000UL // 16 MHz SMCLK
#endif

#define TICKS_PER_OVERFLOW 65536UL

// Timer and delay functions
void delay_ticks(uint32_t ticks);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask);

void start_timer(timer_type timer);
void stop_timer(timer_type timer);

uint32_t get_timer_ticks(timer_type timer);

uint32_t ticks_to_ms(uint32_t ticks);
uint32_t ticks_to_us(uint32_t ticks);
uint32_t us_to_timer_ticks(timer_type timer, uint32_t us);
uint32_t ms_to_timer_ticks(timer_type timer, uint32_t ms);

uint32_t timer_diff_ms(uint32_t start, uint32_t end);
uint32_t timer_diff_us(uint32_t start, uint32_t end);

void configure_timer(timer_type timer, uint16_t divider_setting, uint16_t mode);
void set_timer_compare(timer_type timer, uint32_t channel, uint32_t value);
void reset_timer(const timer_type timer);
void setup_timer_with_overflow_counting(timer_type timer);
uint32_t get_timer_frequency(timer_type timer);

void start_timer_with_interrupt(timer_type timer);
void set_timer_compare_callback(const timer_type timer, void (*const callback)(void));
void set_timer_overflow_callback(const timer_type timer, void (*const callback)(void));
void clear_timer_event_callback(const timer_type timer);
uint16_t choose_prescaler_and_ticks(uint32_t sample_interval_us, uint32_t smclk_hz, uint32_t *out_ticks);

#endif // MSP430FR5994_TIME_H