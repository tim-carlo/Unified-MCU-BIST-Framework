#ifndef NRF52840_TIME_H
#define NRF52840_TIME_H

#include <stdint.h>
#include "nrf.h"
#include "nrf52840.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate elapsed time between two timer values (handles 32-bit overflow)
 * @param start Start time in timer ticks
 * @param current Current time in timer ticks
 * @return Elapsed time in microseconds
 */
uint32_t get_elapsed_time(uint32_t start, uint32_t current);

/**
 * @brief Busy-wait delay in microseconds (uses TIMER3)
 * @param us Microseconds to delay (1μs resolution)
 */
void delay_us(uint32_t us);

/**
 * @brief Busy-wait delay in milliseconds (uses TIMER3)
 * @param ms Milliseconds to delay (1ms resolution)
 */
void delay_ms(uint32_t ms);

/**
 * @brief Capture current timer counter value
 * @param timer Pointer to NRF_TIMER_Type instance (e.g. NRF_TIMER0)
 * @return Current 32-bit timer value
 */
uint32_t get_timer_ticks(NRF_TIMER_Type *timer);

/**
 * @brief Initialize and start timer (1MHz, 32-bit mode)
 * @param timer Pointer to NRF_TIMER_Type instance
 */
void start_timer(NRF_TIMER_Type *timer);

/**
 * @brief Stop timer
 * @param timer Pointer to NRF_TIMER_Type instance
 */
void stop_timer(NRF_TIMER_Type *timer);

/**
 * @brief Reset timer counter to zero and restart
 * @param timer Pointer to NRF_TIMER_Type instance
 */
void reset_timer(NRF_TIMER_Type *timer);

/**
 * @brief Convert timer ticks to microseconds (1MHz timer assumed)
 * @param ticks Timer tick value
 * @return Time in microseconds
 */
static inline uint32_t ticks_to_us(uint64_t ticks) {
    return (uint32_t)ticks;
}

/**
 * @brief Convert timer ticks to milliseconds (1MHz timer assumed)
 * @param ticks Timer tick value
 * @return Time in milliseconds
 */
static inline uint32_t ticks_to_ms(uint64_t ticks) {
    return (uint32_t)(ticks / 1000);
}

/**
 * @brief Calculate time difference in microseconds
 * @param start Start timestamp in microseconds
 * @param end End timestamp in microseconds
 * @return Difference in microseconds (end - start)
 */
static inline uint32_t timer_diff_us(uint64_t start, uint64_t end) {
    return (uint32_t)(end - start);
}

/**
 * @brief Calculate time difference in milliseconds
 * @param start Start timestamp in microseconds
 * @param end End timestamp in microseconds
 * @return Difference in milliseconds (end - start)
 */
static inline uint32_t timer_diff_ms(uint64_t start, uint64_t end) {
    return (uint32_t)((end - start) / 1000);
}

#ifdef __cplusplus
}
#endif

#endif // NRF52840_TIME_H