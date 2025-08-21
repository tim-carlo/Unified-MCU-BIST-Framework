#include "msp430fr5994_time.h"
#include "msp430fr5994_gpio.h" 
#include "printf.h"            
#include "stack.h"              
#include <stdbool.h>       


static volatile uint8_t *const PxIE[] = {&P1IE, &P2IE, &P3IE, &P4IE, &P5IE, &P6IE, &P7IE, &P8IE};
static volatile uint8_t *const PxIFG[] = {&P1IFG, &P2IFG, &P3IFG, &P4IFG, &P5IFG, &P6IFG, &P7IFG, &P8IFG};
static volatile uint8_t *const PxIES[] = {&P1IES, &P2IES, &P3IES, &P4IES, &P5IES, &P6IES, &P7IES, &P8IES};

volatile uint16_t timer_overflows_a1 = 0;
volatile uint16_t timer_overflows_a2 = 0;
volatile uint16_t timer_overflows_a4 = 0;
volatile uint16_t timer_overflows_b0 = 0;

static volatile uint32_t remaining_ticks = 0;
static volatile uint16_t overflow_count = 0;
static volatile bool delay_done = false;

void delay_ticks(uint32_t ticks)
{
    if (ticks == 0)
        return;

    // Save current timer configuration
    uint16_t saved_config = TA0CTL;

    // Stop and clear timer, set divider to 8
    TA0CTL = TASSEL__SMCLK | ID__8 | MC__STOP | TACLR;
    TA0R = 0;

    // Adjust ticks for divider
    remaining_ticks = (ticks + 7) / 8; 

    // Calculate the number of overflows needed
    if (remaining_ticks <= 0x10000)
    {
        // Single period
        TA0CCR0 = remaining_ticks - 1;
        overflow_count = 0;
    }
    else
    {
        // Multiple periods are needed so we calculate the overflow count
        overflow_count = (remaining_ticks - 1) / 0x10000;
        TA0CCR0 = 0xFFFF; // Full period first
    }

    TA0CCTL0 = CCIE;
    if (overflow_count > 0)
    {
        TA0CTL |= TAIE; // Enable overflow interrupt if needed
    }

    delay_done = false;
    TA0CTL |= MC__UP; // Start timer

    while (!delay_done)
        ;

    // Cleanup
    TA0CCTL0 &= ~CCIE;

    // Restore previous timer configuration
    TA0CTL = saved_config;
}

// This is needed to quit the delay loop when the timer reaches the target ticks
__attribute__((interrupt(TIMER0_A0_VECTOR))) void Timer0_A0_ISR(void)
{
    // Clear the interrupt flag
    TA0CCTL0 &= ~CCIFG;
    if (overflow_count == 0)
    {
        // Last tick reached, signal completion
        delay_done = true;
    }
    else
    {
        // Prepare for next overflow
        TA0CCR0 = 0xFFFF;
    }
}

__attribute__((interrupt(TIMER0_A1_VECTOR))) void Timer0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG))
    {
    // Handle Timer A0 overflow
    case TA0IV_TAIFG:
        if (overflow_count > 0)
        {
            overflow_count--;
            if (overflow_count == 0)
            {
                // Last overflow, set final count
                // Since the timer counts inclusive, we set it to remaining_ticks - 1
                // (% 0x10000) used to ensure it fits in 16 bits
                TA0CCR0 = (remaining_ticks - 1) % 0x10000; // Set remaining ticks
            }
        }
        break;
    // Handle other cases if needed
    default:
        break;
    }
}

/**
 * @brief Delay for a specified number of microseconds
 *
 * @param us Number of microseconds to delay
 */
#define DELAY_1US_CYCLES (SMCLK_HZ / 1000000 - 5) // 5 = Loop Overhead in Zyklen (messen!)

void delay_us(uint32_t us)
{
    while (us--)
    {
        __delay_cycles(DELAY_1US_CYCLES);
    }
}

/**
 * @brief Busy-wait delay in milliseconds using Timer A0
 * @param ms Milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    if (ms == 0)
        return;

    const uint32_t ticks_per_ms = SMCLK_HZ / 1000;

    while (ms >= 1000)
    {
        delay_ticks(SMCLK_HZ); // Exactly 1 second
        ms -= 1000;
    }
    // Handle remaining milliseconds
    if (ms > 0)
    {
        uint32_t ticks = ms * ticks_per_ms;
        delay_ticks(ticks);
    }
}

/**
 * @brief Start the specified timer in continuous mode
 *
 * @param timer Timer to start (TIMER_A1, TIMER_A2, TIMER_A4, TIMER_B0)
 */
void start_timer(timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);

    // Reset overflow counter for the selected timer
    if (timer == TIMER_A1)
        timer_overflows_a1 = 0;
    else if (timer == TIMER_A2)
        timer_overflows_a2 = 0;
    else if (timer == TIMER_A4)
        timer_overflows_a4 = 0;
    else if (timer == TIMER_B0)
        timer_overflows_b0 = 0;

    *ctl &= ~MC_3;          // Stop timer (MC bits = 0)
    *ctl |= TACLR;          // Clear timer (write 1 to TACLR bit)
    *ctl |= MC__CONTINUOUS; // Start timer in continuous mode
}

/**
 * @brief Stop the specified timer
 *
 * @param timer Timer to stop (TIMER_A1, TIMER_A2, TIMER_A4, TIMER_B0)
 */
void stop_timer(timer_type timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    volatile uint16_t *r = GET_TxxR(timer);

    *ctl &= ~MC_3; // Clear MC bits → stop mode
    *ctl |= TACLR; // Clear timer (setzt den Zähler zurück)
    *r = 0;        // Zur Sicherheit Timer-Register auf 0 schreiben
}

void __attribute__((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR(void)
{
    switch (__even_in_range(TA1IV, TA1IV_TAIFG))
    {
    case TA1IV_TAIFG:
        timer_overflows_a1++;
        break;
        // maybe later add more cases for TA1IV
    }
}

void __attribute__((interrupt(TIMER2_A1_VECTOR))) TIMER2_A1_ISR(void)
{
    switch (__even_in_range(TA2IV, TA2IV_TAIFG))
    {
    case TA2IV_TAIFG:
        timer_overflows_a2++;
        break;
        // maybe later add more cases for TA2IV
    }
}

void __attribute__((interrupt(TIMER4_A1_VECTOR))) TIMER4_A1_ISR(void)
{
    switch (TA4IV)
    {
    case TA4IV_TAIFG:
        timer_overflows_a4++;
        break;
        // maybe later add more cases for TA4IV
    }
}

void __attribute__((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_ISR(void)
{
    switch (TB0IV)
    {
    case TB0IV_TBIFG:
        timer_overflows_b0++;
        break;
        // maybe later add more cases for TB0IV
    }
}

uint32_t ticks_elapsed(uint32_t start, uint32_t end)
{
    return (uint32_t)(end - start); 
}

/**
 * @brief Get the timer ticks object
 *
 * @param timer_r Pointer to the timer register (TA0R, TA1R, TA2R, TA4R, TB0R)
 * @return uint32_t Timer ticks
 */
uint32_t get_timer_ticks(timer_type timer)
{
    volatile uint16_t *timer_r = GET_TxxR(timer);
    uint16_t counter = *timer_r;
    uint16_t overflows = 0;

    switch (timer)
    {
    case TIMER_A0:
        overflows = 0;
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
 * @brief Convert timer ticks to milliseconds
 * @param ticks Timer ticks
 * @return Time in milliseconds
 */
uint32_t ticks_to_ms(uint32_t ticks) {
    return (uint32_t)(((uint64_t)ticks * 1000u) / (uint64_t)TIMER_FREQ_HZ);
}

/**
 * @brief Convert timer ticks to microseconds
 * @param ticks Timer ticks
 * @return Time in microseconds
 */
uint32_t ticks_to_us(uint32_t ticks)
{
    // Convert ticks to microseconds using the actual timer frequency
    return (uint32_t)(((uint64_t)ticks * 1000000u) / (uint64_t)TIMER_FREQ_HZ);
}

/**
 * @brief Convert timer ticks to milliseconds
 * @param ticks Timer ticks
 * @return Time in milliseconds
 */
uint32_t timer_diff_ms(uint32_t start, uint32_t end)
{
   return ticks_to_ms(ticks_elapsed(start, end));
}

/**
 * @brief Calculate time difference in microseconds using get_elapsed_time
 * @param start Start tick count
 * @param end End tick count
 * @return Elapsed time in microseconds
 */
uint32_t timer_diff_us(uint32_t start, uint32_t end)
{
    return ticks_to_us(ticks_elapsed(start, end));
}