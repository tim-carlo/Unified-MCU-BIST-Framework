#include "nrf52840_time.h"
#include "nrf.h"
#include "nrf52840.h"


static void (*timer_event_callback[5])(void) = {NULL, NULL, NULL, NULL, NULL};

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

/**
 * @brief Configure the timer with specified prescaler and bitmode
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param prescaler Prescaler value (0-9) to set the timer frequency
 * @param bitmode Bit mode (0=16-bit, 1=8-bit, 2=24-bit, 3=32-bit)
 */
void configure_timer(NRF_TIMER_Type * const timer, const uint32_t prescaler, const uint32_t bitmode)
{
    timer->TASKS_STOP = 1;                                      // Stop timer before configuration
    timer->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos; // Timer mode
    timer->PRESCALER = prescaler & 0x0F;                        // Prescaler 0-9

    // BITMODE: 0=16Bit, 1=8Bit, 2=24Bit, 3=32Bit
    timer->BITMODE = (bitmode & 0x03) << TIMER_BITMODE_BITMODE_Pos;

    timer->TASKS_CLEAR = 1; // Clear timer counter
}

/**
 * @brief Set the compare value for a specific channel of the timer
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param channel Compare channel (0-5)
 * @param value Compare value to set
 */
void set_timer_compare(NRF_TIMER_Type * const timer, const uint32_t channel, const uint32_t value)
{
    if (channel < 6)
    {
        timer->CC[channel] = value;
    }
}

/**
 * @brief Set a callback function to be called on timer compare event
 *
 * Note: This is a placeholder function. Actual implementation of callback
 * registration depends on the specific application and interrupt handling.
 *
 * @param timer Pointer to the NRF_TIMER_Type structure for the timer
 * @param callback Function pointer to the callback function
 */
void set_timer_event_callback(NRF_TIMER_Type * const timer, void (* const callback)(void))
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
    NVIC_ClearPendingIRQ(irqn);
    NVIC_EnableIRQ(irqn);
}

void clear_timer_event_callback(NRF_TIMER_Type * const timer)
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

// Timer interrupt handlers
#define TIMER_ISR(N) \
void TIMER##N##_IRQHandler(void) \
{ \
    if (NRF_TIMER##N->EVENTS_COMPARE[0]) \
    { \
        NRF_TIMER##N->EVENTS_COMPARE[0] = 0; \
        if (timer_event_callback[N]) \
            timer_event_callback[N](); \
    } \
}

TIMER_ISR(0)
TIMER_ISR(1)
TIMER_ISR(2)
TIMER_ISR(3)
TIMER_ISR(4)
