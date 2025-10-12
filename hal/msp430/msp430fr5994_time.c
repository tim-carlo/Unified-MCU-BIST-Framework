#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h" 
#include "printf.h"            
#include "stack.h"              
#include <stdbool.h>       

// Timer callback arrays - separate for overflow and compare interrupts
static void (*timer_overflow_callback[5])(void) = {NULL, NULL, NULL, NULL, NULL};
static void (*timer_compare_callback[5])(void) = {NULL, NULL, NULL, NULL, NULL};

// Overflow counters
volatile uint16_t timer_overflows_a0 = 0;
volatile uint16_t timer_overflows_a1 = 0;
volatile uint16_t timer_overflows_a2 = 0;
volatile uint16_t timer_overflows_a4 = 0;
volatile uint16_t timer_overflows_b0 = 0;

// Delay management variables
static volatile uint32_t remaining_ticks = 0;
static volatile uint16_t overflow_count = 0;
static volatile bool delay_done = false;

/**
 * @brief Configure a timer with specified prescaler and mode
 */
void configure_timer(const timer_type timer, const uint16_t prescaler, const uint16_t mode)
{
    volatile uint16_t *const ctl = GET_TxxCTL(timer);
    
    // Stop timer first
    *ctl &= ~MC_3;  // Clear mode control bits
    *ctl |= TACLR;  // Clear timer
    
    // Set clock source to SMCLK
    *ctl |= TASSEL__SMCLK;
    
    // Set prescaler
    *ctl &= ~(ID_3);  // Clear existing prescaler bits
    switch (prescaler) {
        case 1:
            *ctl |= ID__1;
            break;
        case 2:
            *ctl |= ID__2;
            break;
        case 4:
            *ctl |= ID__4;
            break;
        case 8:
            *ctl |= ID__8;
            break;
        default:
            *ctl |= ID__1;  // Default to /1
            break;
    }
    
    // Set mode
    *ctl |= mode;
}

/**
 * @brief Set compare value for timer capture/compare register
 */
void set_timer_compare(timer_type timer, uint32_t channel, uint32_t value)
{
    if (channel != 0) return;  // Only CCR0 supported
    
    volatile uint16_t *ccr0 = GET_TxxCCR0(timer);
    *ccr0 = (uint16_t)(value & 0xFFFF);
}

/**
 * @brief Reset timer counter to zero
 */
void reset_timer(const timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    volatile uint16_t *r = GET_TxxR(timer);
    
    *ctl |= TACLR;  // Clear timer
    *r = 0;         // Ensure register is zero
    
    // Reset overflow counter
    switch (timer) {
        case TIMER_A0:
            timer_overflows_a0 = 0;
            break;
        case TIMER_A1:
            timer_overflows_a1 = 0;
            break;
        case TIMER_A2:
            timer_overflows_a2 = 0;
            break;
        case TIMER_A4:
            timer_overflows_a4 = 0;
            break;
        case TIMER_B0:
            timer_overflows_b0 = 0;
            break;
    }
}

/**
 * @brief Set overflow callback - called on timer overflow (Slot 1)
 */
void set_timer_overflow_callback(const timer_type timer, void (*const callback)(void))
{
    timer_overflow_callback[timer] = callback;
    
    // Enable overflow interrupt
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    *ctl |= TAIE;
}

/**
 * @brief Set compare callback - called on compare match (Slot 0)
 */
void set_timer_compare_callback(const timer_type timer, void (*const callback)(void))
{
    timer_compare_callback[timer] = callback;
    
    // Enable compare interrupt for CCR0
    volatile uint16_t *cctl0 = GET_TxxCCTL0(timer);
    *cctl0 |= CCIE;
}

/**
 * @brief Clear all timer callbacks and disable interrupts
 * @param timer Timer to clear callbacks for
 */
void clear_timer_event_callback(const timer_type timer)
{
    volatile uint16_t *cctl0 = GET_TxxCCTL0(timer);
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    
    // Clear callbacks
    timer_overflow_callback[timer] = NULL;
    timer_compare_callback[timer] = NULL;
    
    // Disable interrupts
    *cctl0 &= ~CCIE;  // Disable compare interrupt
    *ctl &= ~TAIE;    // Disable overflow interrupt
}

/**
 * @brief Start timer with interrupt
 * @param timer Timer to start
 */
void start_timer_with_interrupt(const timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    volatile uint16_t *cctl0 = GET_TxxCCTL0(timer);
    
    // Clear any pending interrupts
    *cctl0 &= ~CCIFG;
    
    // Start timer (mode already set by configure_timer)
    *ctl |= MC__UP;
}

/**
 * @brief Stop the specified timer
 * @param timer Timer to stop
 */
void stop_timer(const timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    volatile uint16_t *r = GET_TxxR(timer);
    volatile uint16_t *cctl0 = GET_TxxCCTL0(timer);

    // Disable all interrupts first
    *ctl &= ~TAIE;     // Disable overflow interrupt
    *cctl0 &= ~CCIE;   // Disable compare interrupt
    
    // Clear any pending interrupt flags
    *cctl0 &= ~CCIFG;  // Clear compare interrupt flag
    
    // Stop the timer by clearing mode control bits
    *ctl &= ~(MC_1 | MC_2); // Clear MC bits (MC_3 = MC_1 | MC_2)
    
    // Clear the timer counter
    *ctl |= TACLR;     // Set clear bit
    *r = 0;            // Ensure register is zero
    
    // Reset overflow counter for this timer
    switch (timer) {
        case TIMER_A0:
            timer_overflows_a0 = 0;
            break;
        case TIMER_A1:
            timer_overflows_a1 = 0;
            break;
        case TIMER_A2:
            timer_overflows_a2 = 0;
            break;
        case TIMER_A4:
            timer_overflows_a4 = 0;
            break;
        case TIMER_B0:
            timer_overflows_b0 = 0;
            break;
    }
    
    // Clear callbacks for this timer
    timer_overflow_callback[timer] = NULL;
    timer_compare_callback[timer] = NULL;
}

/**
 * @brief Start the specified timer in continuous mode
 * @param timer Timer to start
 */
void start_timer(const timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);

    // Reset overflow counter
    reset_timer(timer);
    
    *ctl &= ~MC_3;          // Stop timer
    *ctl |= TACLR;          // Clear timer
    *ctl |= MC__CONTINUOUS; // Start timer in continuous mode
}

/**
 * @brief Default overflow counter callback
 * @param timer Timer for which overflow occurred
 */
static void default_overflow_counter(const timer_type timer)
{
    switch (timer) {
        case TIMER_A0:
            timer_overflows_a0++;
            break;
        case TIMER_A1:
            timer_overflows_a1++;
            break;
        case TIMER_A2:
            timer_overflows_a2++;
            break;
        case TIMER_A4:
            timer_overflows_a4++;
            break;
        case TIMER_B0:
            timer_overflows_b0++;
            break;
    }
}

/**
 * @brief Delay completion callback for delay_ticks
 */
static void delay_completion_callback(void)
{
    if (overflow_count == 0) {
        delay_done = true;
    } else {
        // Prepare for next overflow
        set_timer_compare(TIMER_A0, 0, 0xFFFF);
    }
}

/**
 * @brief Delay overflow callback for delay_ticks
 */
static void delay_overflow_callback(void)
{
    if (overflow_count > 0) {
        overflow_count--;
        if (overflow_count == 0) {
            // Last overflow, set final count
            set_timer_compare(TIMER_A0, 0, (remaining_ticks - 1) % 0x10000);
        }
    }
}

/**
 * @brief Delay for a specified number of timer ticks using callbacks
 * @param ticks Number of timer ticks to delay
 */
void delay_ticks(uint32_t ticks)
{
    if (ticks == 0)
        return;

    // Save current timer configuration
    uint16_t saved_config = TA0CTL;

    // Configure timer with divider 8 for longer delays
    configure_timer(TIMER_A0, 8, MC__UP);
    
    // Adjust ticks for divider
    remaining_ticks = (ticks + 7) / 8;

    // Calculate overflow count
    if (remaining_ticks <= 0x10000) {
        set_timer_compare(TIMER_A0, 0, remaining_ticks - 1);
        overflow_count = 0;
    } else {
        overflow_count = (remaining_ticks - 1) / 0x10000;
        set_timer_compare(TIMER_A0, 0, 0xFFFF);
    }

    // Set callbacks
    set_timer_compare_callback(TIMER_A0, delay_completion_callback);
    if (overflow_count > 0) {
        set_timer_overflow_callback(TIMER_A0, delay_overflow_callback);
    }

    delay_done = false;
    
    // Start timer
    start_timer_with_interrupt(TIMER_A0);

    // Wait for completion
    while (!delay_done) {
        ;
    }

    // Cleanup
    clear_timer_event_callback(TIMER_A0);
    
    // Restore previous timer configuration
    TA0CTL = saved_config;
}

/**
 * @brief Get timer ticks with overflow handling
 */
uint32_t get_timer_ticks(const timer_type timer)
{
    volatile uint16_t *timer_r = GET_TxxR(timer);
    uint16_t counter = *timer_r;
    uint16_t overflows = 0;

    switch (timer) {
        case TIMER_A0:
            overflows = timer_overflows_a0;
            break;
        case TIMER_A1:
            overflows = timer_overflows_a1;
            break;
        case TIMER_A2:
            overflows = timer_overflows_a2;
            break;
        case TIMER_A4:
            overflows = timer_overflows_a4;
            break;
        case TIMER_B0:
            overflows = timer_overflows_b0;
            break;
    }

    return ((uint32_t)overflows * TICKS_PER_OVERFLOW) + counter;
}

/**
 * @brief Setup timer for continuous operation with overflow counting
 */
void setup_timer_with_overflow_counting(const timer_type timer)
{
    // Configure timer for continuous mode
    configure_timer(timer, 1, MC__CONTINUOUS);
    
    // Set default overflow callback to count overflows
    set_timer_overflow_callback(timer, NULL);  // Will use default counter
    
    // Start timer
    start_timer_with_interrupt(timer);
}

// Delay utility functions

#define DELAY_1US_CYCLES (SMCLK_HZ / 1000000 - 5) // 5 = Loop Overhead
void delay_us(uint32_t us)
{
    while (us--)
    {
        __delay_cycles(DELAY_1US_CYCLES);
    }
}

void delay_ms(uint32_t ms)
{
    const uint32_t ticks_per_ms = SMCLK_HZ / 1000;
    
    while (ms >= 1000) {
        delay_ticks(SMCLK_HZ);  // 1 second
        ms -= 1000;
    }
    
    if (ms > 0) {
        delay_ticks(ms * ticks_per_ms);
    }
}

// Timer utility functions
uint32_t get_timer_frequency(const timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    uint16_t id_bits = (*ctl & ID_3) >> 6;
    
    uint32_t prescaler = 1;
    switch (id_bits) {
        case 0: prescaler = 1; break;
        case 1: prescaler = 2; break;
        case 2: prescaler = 4; break;
        case 3: prescaler = 8; break;
    }
    
    return SMCLK_HZ / prescaler;
}

uint32_t us_to_timer_ticks(const timer_type timer, uint32_t us)
{
    uint32_t freq = get_timer_frequency(timer);
    return (freq / 1000000UL) * us;
}

uint32_t ms_to_timer_ticks(const timer_type timer, uint32_t ms)
{
    uint32_t freq = get_timer_frequency(timer);
    return (freq / 1000UL) * ms;
}

uint32_t ticks_to_ms(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000u) / (uint64_t)TIMER_FREQ_HZ);
}

uint32_t ticks_to_us(uint32_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * 1000000u) / (uint64_t)TIMER_FREQ_HZ);
}

uint32_t timer_diff_ms(uint32_t start, uint32_t end)
{
    return ticks_to_ms(end - start);
}

uint32_t timer_diff_us(uint32_t start, uint32_t end)
{
    return ticks_to_us(end - start);
}

// Timer A0 ISRs
__attribute__((interrupt(TIMER0_A0_VECTOR))) void TIMER0_A0_ISR(void)
{
    // Slot 0: Compare interrupt (CCR0)
    if (timer_compare_callback[TIMER_A0]) {
        timer_compare_callback[TIMER_A0]();
    }
}

__attribute__((interrupt(TIMER0_A1_VECTOR))) void TIMER0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
        case TA0IV_TAIFG:
            // Slot 1: Overflow interrupt
            default_overflow_counter(TIMER_A0);
            if (timer_overflow_callback[TIMER_A0]) {
                timer_overflow_callback[TIMER_A0]();
            }
            break;
        default:
            break;
    }
}

// Timer A1 ISRs
__attribute__((interrupt(TIMER1_A0_VECTOR))) void TIMER1_A0_ISR(void)
{
    // Slot 0: Compare interrupt (CCR0)
    if (timer_compare_callback[TIMER_A1]) {
        timer_compare_callback[TIMER_A1]();
    }
}

__attribute__((interrupt(TIMER1_A1_VECTOR))) void TIMER1_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG)) {
        case TA1IV_TAIFG:
            // Slot 1: Overflow interrupt
            default_overflow_counter(TIMER_A1);
            if (timer_overflow_callback[TIMER_A1]) {
                timer_overflow_callback[TIMER_A1]();
            }
            break;
        default:
            break;
    }
}

// Timer A2 ISRs
__attribute__((interrupt(TIMER2_A0_VECTOR))) void TIMER2_A0_ISR(void)
{
    // Slot 0: Compare interrupt (CCR0)
    if (timer_compare_callback[TIMER_A2]) {
        timer_compare_callback[TIMER_A2]();
    }
}

__attribute__((interrupt(TIMER2_A1_VECTOR))) void TIMER2_A1_ISR(void)
{
    switch (__even_in_range(TA2IV, TA2IV_TAIFG)) {
        case TA2IV_TAIFG:
            // Slot 1: Overflow interrupt
            default_overflow_counter(TIMER_A2);
            if (timer_overflow_callback[TIMER_A2]) {
                timer_overflow_callback[TIMER_A2]();
            }
            break;
        default:
            break;
    }
}

// Timer A4 ISRs
__attribute__((interrupt(TIMER4_A0_VECTOR))) void TIMER4_A0_ISR(void)
{
    // Slot 0: Compare interrupt (CCR0)
    if (timer_compare_callback[TIMER_A4]) {
        timer_compare_callback[TIMER_A4]();
    }
}

__attribute__((interrupt(TIMER4_A1_VECTOR))) void TIMER4_A1_ISR(void)
{
    switch (TA4IV) {
        case TA4IV_TAIFG:
            // Slot 1: Overflow interrupt
            default_overflow_counter(TIMER_A4);
            if (timer_overflow_callback[TIMER_A4]) {
                timer_overflow_callback[TIMER_A4]();
            }
            break;
        default:
            break;
    }
}

// Timer B0 ISRs
__attribute__((interrupt(TIMER0_B0_VECTOR))) void TIMER0_B0_ISR(void)
{
    // Slot 0: Compare interrupt (CCR0)
    if (timer_compare_callback[TIMER_B0]) {
        timer_compare_callback[TIMER_B0]();
    }
}

__attribute__((interrupt(TIMER0_B1_VECTOR))) void TIMER0_B1_ISR(void)
{
    switch (TB0IV) {
        case TB0IV_TBIFG:
            // Slot 1: Overflow interrupt
            default_overflow_counter(TIMER_B0);
            if (timer_overflow_callback[TIMER_B0]) {
                timer_overflow_callback[TIMER_B0]();
            }
            break;
        default:
            break;
    }
}
