#include "nrf52840_time.h"
#include "nrf.h"
#include "nrf52840.h"
#include <stdbool.h>
#include <stdint.h>

static void (*timer_event_callback[5])(void) = {NULL, NULL, NULL, NULL, NULL};

/**
 * @brief Configure the timer with specified prescaler and bitmode
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param prescaler Prescaler value (0-9) to set the timer frequency
 * @param bitmode Bit mode (0=16-bit, 1=8-bit, 2=24-bit, 3=32-bit)
 */
void configure_timer(NRF_TIMER_Type *const timer, const uint32_t prescaler, const uint32_t bitmode)
{
    timer->TASKS_STOP = 1;                                      // Stop timer before configuration
    timer->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos; // Timer mode
    timer->PRESCALER = prescaler & 0x0F;                        // Prescaler 0-9

    // BITMODE: 0=16Bit, 1=8Bit, 2=24Bit, 3=32Bit
    timer->BITMODE = (bitmode & 0x03) << TIMER_BITMODE_BITMODE_Pos;

    timer->TASKS_CLEAR = 1; // Clear timer counter
}

/**
 * @brief Configure timer for time measurement (standard 1MHz setup)
 */
void configure_timer_for_measurement(NRF_TIMER_Type *timer)
{
    configure_timer(timer, 4, TIMER_BITMODE_BITMODE_32Bit); // 1MHz, 32-bit
}

/**
 * @brief Set the compare value for a specific channel of the timer
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param channel Compare channel (0-5)
 * @param value Compare value to set
 * @param enable_interrupt Whether to enable interrupt for this compare
 * @param clear_on_compare Whether to clear the timer on compare match
 */
void set_timer_compare(NRF_TIMER_Type *const timer, const uint32_t channel, const uint32_t value, const bool enable_interrupt, const bool clear_on_compare)
{
    if (channel < 6)
    {
        timer->CC[channel] = value;
        if (enable_interrupt)
        {
            timer->INTENSET = (1 << (16 + channel)); // Enable interrupt for COMPARE[channel]
        }
        if (clear_on_compare)
        {
            timer->SHORTS |= (1 << (8 + channel)); // Enable shortcut to clear on COMPARE[channel]
        }
    }
}


/**
 * @brief General timer start function - only starts without reconfiguring
 */
void start_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_START = 1;
}

/**
 * @brief General timer stop function - only stops
 */
void stop_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_STOP = 1;
}

/**
 * @brief Reset the timer counter to zero
 */
void reset_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_CLEAR = 1; // Clear the timer counter
    timer->TASKS_START = 1; // Restart the timer after clearing
}

/**
 * @brief Reset timer and start (keeps current configuration)
 */
void reset_and_start_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_CLEAR = 1;
    timer->TASKS_START = 1;
}

/**
 * @brief Get the current timer counter value
 *
 * This function captures the current value of the timer's counter.
 * It is used to measure elapsed time in microseconds.
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @return uint32_t Current timer counter value
 */
uint32_t get_timer_ticks(NRF_TIMER_Type *timer)
{
    timer->TASKS_CAPTURE[0] = 1;
    return timer->CC[0];
}

/**
 * @brief Get the elapsed time in microseconds between two timer values
 * @param start Start time in timer ticks
 * @param current Current time in timer ticks
 */
uint32_t get_elapsed_time(uint32_t start, uint32_t current)
{
    if (current >= start)
    {
        return current - start;
    }
    else
    {
        return (0xFFFFFFFF - start) + current + 1;
    }
}

/**
 * @brief Start time measurement (always 1MHz configuration)
 */
void start_time_measurement(NRF_TIMER_Type *timer)
{
    configure_timer(timer, 4, TIMER_BITMODE_BITMODE_32Bit); // Always 1MHz
    timer->TASKS_CLEAR = 1;                                  // Reset counter
    start_timer(timer);
}

/**
 * @brief Stop time measurement and return elapsed time in microseconds
 */
uint32_t stop_time_measurement_us(NRF_TIMER_Type *timer)
{
    uint32_t ticks = get_timer_ticks(timer);
    stop_timer(timer);
    return ticks; // 1MHz = 1 tick per µs
}

/**
 * @brief Stop time measurement and return elapsed time in milliseconds
 */
uint32_t stop_time_measurement_ms(NRF_TIMER_Type *timer)
{
    uint32_t ticks = get_timer_ticks(timer);
    stop_timer(timer);
    return ticks / 1000; // Convert µs to ms
}

/**
 * @brief Get elapsed time in microseconds without stopping the timer
 */
uint32_t get_elapsed_time_us(NRF_TIMER_Type *timer)
{
    return get_timer_ticks(timer); // 1MHz = 1 tick per µs
}

/**
 * @brief Get elapsed time in milliseconds without stopping the timer
 */
uint32_t get_elapsed_time_ms(NRF_TIMER_Type *timer)
{
    return get_timer_ticks(timer) / 1000; // Convert µs to ms
}

/**
 * @brief Delay for a specified number of microseconds
 *
 * @param us Number of microseconds to delay
 */
void delay_us(uint32_t us)
{
    NRF_TIMER_Type *timer = NRF_TIMER3;

    // Configure timer for 1MHz operation (prescaler 4: 16MHz/16 = 1MHz)
    configure_timer(timer, 4, TIMER_BITMODE_BITMODE_32Bit);

    set_timer_compare(timer, 0, us, false);
    timer->EVENTS_COMPARE[0] = 0;

    start_timer(timer);

    // Wait for compare event
    while (timer->EVENTS_COMPARE[0] == 0)
    {
        // Busy wait
    }

    stop_timer(timer);
    timer->EVENTS_COMPARE[0] = 0; // Clear event
}

/**
 * @brief Delay for a specified number of milliseconds
 *
 * @param ms Number of milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    NRF_TIMER_Type *timer = NRF_TIMER3;

    // Use prescaler 8 for longer delays
    configure_timer(timer, 8, TIMER_BITMODE_BITMODE_32Bit);
    
    uint32_t ticks = (ms * 625) / 10;  // 62.5 ticks per ms

    set_timer_compare(timer, 0, ticks, false);
    timer->EVENTS_COMPARE[0] = 0;

    start_timer(timer);

    // Wait for compare event
    while (timer->EVENTS_COMPARE[0] == 0)
    {
       ; // Busy wait
    }

    stop_timer(timer);
    timer->EVENTS_COMPARE[0] = 0; // Clear event
}


/**
 * @brief Convert microseconds to ticks (always 1MHz)
 */
uint32_t us_to_ticks(uint32_t us)
{
    return us; // 1MHz = 1 tick per µs
}

/**
 * @brief Convert milliseconds to ticks (always 1MHz)
 */
uint32_t ms_to_ticks(uint32_t ms)
{
    return ms * 1000; // Convert to µs, then 1 tick per µs
}

/**
 * @brief Convert ticks to microseconds (always 1MHz)
 */
uint32_t ticks_to_us_simple(uint32_t ticks)
{
    return ticks; // 1MHz = 1 tick per µs
}

/**
 * @brief Convert ticks to milliseconds (always 1MHz)
 */
uint32_t ticks_to_ms_simple(uint32_t ticks)
{
    return ticks / 1000; // Convert µs to ms
}

/**
 * @brief Get the current timer in microseconds (legacy)
 *
 * @return uint64_t Current timer value in microseconds
 */
uint32_t ticks_to_us(uint64_t ticks)
{
    return (uint32_t)ticks;
}

/**
 * @brief Convert timer ticks to milliseconds (legacy)
 *
 * @param ticks Timer ticks
 * @return uint32_t Time in milliseconds
 */
uint32_t ticks_to_ms(uint64_t ticks)
{
    return (uint32_t)(ticks / 1000);
}

/**
 * @brief Get the current timer value in microseconds (legacy)
 *
 * @return uint64_t Current timer value in microseconds
 */
uint32_t timer_diff_us(uint64_t start, uint64_t end)
{
    return (uint32_t)(end - start);
}

/**
 * @brief Get the difference between two timer values in milliseconds (legacy)
 *
 * @param start Start time in microseconds
 * @param end End time in microseconds
 * @return uint32_t Difference in milliseconds
 */
uint32_t timer_diff_ms(uint64_t start, uint64_t end)
{
    return (uint32_t)((end - start) / 1000);
}

/**
 * @brief Set a callback function to be called on timer compare event
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param callback Function pointer to the callback function
 */
void set_timer_event_callback(NRF_TIMER_Type *const timer, void (*const callback)(void))
{
    int idx = -1;
    IRQn_Type irqn;
    switch ((uintptr_t)timer)
    {
    case (uintptr_t)NRF_TIMER0:
        idx = 0;
        irqn = TIMER0_IRQn;
        break;
    case (uintptr_t)NRF_TIMER1:
        idx = 1;
        irqn = TIMER1_IRQn;
        break;
    case (uintptr_t)NRF_TIMER2:
        idx = 2;
        irqn = TIMER2_IRQn;
        break;
    case (uintptr_t)NRF_TIMER3:
        idx = 3;
        irqn = TIMER3_IRQn;
        break;
    case (uintptr_t)NRF_TIMER4:
        idx = 4;
        irqn = TIMER4_IRQn;
        break;
    default:
        return; // Unknown timer, do nothing
    }
    if (idx >= 0 && idx < 5)
        timer_event_callback[idx] = callback;

    // Enable interrupt for COMPARE[0]
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);
}

/**
 * @brief Clear timer event callback
 */
void clear_timer_event_callback(NRF_TIMER_Type *const timer)
{
    int idx = -1;
    IRQn_Type irqn;
    switch ((uintptr_t)timer)
    {
    case (uintptr_t)NRF_TIMER0:
        idx = 0;
        irqn = TIMER0_IRQn;
        break;
    case (uintptr_t)NRF_TIMER1:
        idx = 1;
        irqn = TIMER1_IRQn;
        break;
    case (uintptr_t)NRF_TIMER2:
        idx = 2;
        irqn = TIMER2_IRQn;
        break;
    case (uintptr_t)NRF_TIMER3:
        idx = 3;
        irqn = TIMER3_IRQn;
        break;
    case (uintptr_t)NRF_TIMER4:
        idx = 4;
        irqn = TIMER4_IRQn;
        break;
    default:
        return; // Unknown timer, do nothing
    }
    if (idx >= 0 && idx < 5)
        timer_event_callback[idx] = NULL;
    NVIC_DisableIRQ(irqn);
}

/**
 * @brief Timer interrupt handler macro
 */
#define TIMER_ISR(N)                             \
    void TIMER##N##_IRQHandler(void)             \
    {                                            \
        if (NRF_TIMER##N->EVENTS_COMPARE[0])     \
        {                                        \
            NRF_TIMER##N->EVENTS_COMPARE[0] = 0; \
            if (timer_event_callback[N])         \
                timer_event_callback[N]();       \
        }                                        \
    }

TIMER_ISR(0)
TIMER_ISR(1)
TIMER_ISR(2)
TIMER_ISR(3)
TIMER_ISR(4)
