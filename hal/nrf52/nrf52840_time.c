#include "nrf52840_time.h"

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
 * @brief Delay for a specified number of microseconds
 *
 * @param us Number of microseconds to delay
 */
void delay_us(uint32_t us)
{
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;

    // TIMER3: 1 MHz:  1 tick = 1 µs
    NRF_TIMER3->PRESCALER = 4;
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

    NRF_TIMER3->CC[0] = us;
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->TASKS_START = 1;

    while (NRF_TIMER3->EVENTS_COMPARE[0] == 0)
    {
    }

    NRF_TIMER3->TASKS_STOP = 1;
    // clear the flag
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
}

/**
 * @brief Delay for a specified number of milliseconds
 *
 * @param ms Number of milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->TASKS_CLEAR = 1;

    // TIMER3: 1 MHz:  1 tick = 1 µs
    NRF_TIMER3->PRESCALER = 4;
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

    NRF_TIMER3->CC[0] = ms * 1000;
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    NRF_TIMER3->TASKS_START = 1;

    while (NRF_TIMER3->EVENTS_COMPARE[0] == 0)
    {
    }

    NRF_TIMER3->TASKS_STOP = 1;
    // clear the flag
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
}

/**
 * @brief Get the current timer counter value
 *
 * This function captures the current value of TIMER0's counter.
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
 * @brief Start the TIMER0 peripheral for timing operations
 *
 * This function configures TIMER0 to run at 1 MHz (1 µs per tick) and starts it.
 * It also records the current time as the starting point for subsequent measurements.
 */
void start_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_STOP = 1;
    timer->MODE = TIMER_MODE_MODE_Timer;
    timer->PRESCALER = 4; // 1 MHz = 1 µs per tick
    timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    timer->TASKS_CLEAR = 1; // Reset the counter
    timer->TASKS_START = 1; // Now
}

/**
 * @brief This function stops the TIMER0 peripheral, which is used for timing operations.
 *
 */
void stop_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_STOP = 1; // Stop the specified timer
}

/**
 * @brief Reset the timer counter to zero
 *
 */
void reset_timer(NRF_TIMER_Type *timer)
{
    timer->TASKS_CLEAR = 1; // Clear the timer counter
    timer->TASKS_START = 1; // Restart the timer after clearing
}

/**
 * @brief Get the current timer in microseconds
 *
 * @return uint64_t Current timer value in microseconds
 */
uint32_t ticks_to_us(uint64_t ticks)
{
    return (uint32_t)ticks;
}

/**
 * @brief Convert timer ticks to milliseconds
 *
 * @param ticks Timer ticks
 * @return uint32_t Time in milliseconds
 */
uint32_t ticks_to_ms(uint64_t ticks)
{
    return (uint32_t)(ticks / 1000);
}

/**
 * @brief Get the current timer value in microseconds
 *
 * @return uint64_t Current timer value in microseconds
 */
uint32_t timer_diff_us(uint64_t start, uint64_t end)
{
    return (uint32_t)(end - start);
}

/**
 * @brief Get the difference between two timer values in milliseconds
 *
 * @param start Start time in microseconds
 * @param end End time in microseconds
 * @return uint32_t Difference in milliseconds
 */
uint32_t timer_diff_ms(uint64_t start, uint64_t end)
{
    return (uint32_t)((end - start) / 1000);
}