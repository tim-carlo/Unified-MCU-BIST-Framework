#include <string.h>

#ifndef NRF52840_HELPER_H
#define NRF52840_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf.h"
#include "nrf52840.h"
#include "stack.h"
#include "pindata.h"

#define NUMBER_OF_GPIO_PINS 48
#define UART_PIN_TX 6
#define UART_PIN_RX 8

#define DEBOUNCE_SAMPLES 5
#define DEBOUNCE_DELAY_US 20

#define BV(pos) (1u << (pos))
#define BV_BY_NAME(field, value) ((field##_##value << field##_Pos) & field##_Msk)
#define BV_BY_VALUE(field, value) (((value) << field##_Pos) & field##_Msk)

typedef void (*gpio_interrupt_handler_t)(uint32_t gpio);

#define MAX_GPIO_INTERRUPT_HANDLERS 4


static gpio_interrupt_handler_t rising_callback_single = NULL;
static gpio_interrupt_handler_t falling_callback_single = NULL;
static uint8_t gpiote_pin0 = 0xFF; // ungültig als Initialwert
static uint32_t ticks_at_starting_point = 0;

/**
 * @brief Initialize the IO peripherals
 *
 * This function initializes the UART for printf output and starts the HFCLK.
 * It should be called at the beginning of the main function.
 */
static inline void io_init(void)
{
    // Start HFCLK if not running
    if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
        while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        {
        }
    }
    // Set up UART0 for printf
    NRF_UART0->PSELTXD = UART_PIN_TX;                                         // TX pin
    NRF_UART0->PSELRXD = UART_PIN_RX;                                         // RX pin
    NRF_UART0->BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud9600;                    // Set baud rate to 9600
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos; // Enable UART
    NRF_UART0->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) |
                        (UART_CONFIG_HWFC_Disabled << UART_CONFIG_HWFC_Pos); // No parity, no flow control
    NRF_UART0->TASKS_STARTTX = 1;                                            // Start UART transmission
    NRF_UART0->TASKS_STARTRX = 1;                                            // Start UART reception
}

/**
 * @brief Get the current timer counter value
 *
 * This function captures the current value of TIMER0's counter.
 * It is used to measure elapsed time in microseconds.
 *
 * @return uint32_t Current timer counter value
 */
static inline uint32_t get_timer_counter(void)
{
    NRF_TIMER0->TASKS_CAPTURE[0] = 1;
    return NRF_TIMER0->CC[0];
}

/**
 * @brief Initialize GPIO pin with pull-up resistor using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_pullup_init(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->PIN_CNF[pin] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, D0S1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
}

/**
 * @brief Initialize GPIO pin with pull-down resistor using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_pulldown_init(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->PIN_CNF[pin] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Pulldown) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
}

/**
 * @brief Disable pull resistors using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_pullup_clear(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->PIN_CNF[pin] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
}

/**
* @brief Drive GPIO pin low (output) using absolute pin number
* @param abs_pin Absolute pin number (0-47)
*/
static inline void gpio_drive_low(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRSET = (1UL << pin);
    PORT->OUTCLR = (1UL << pin);
}

/**
 * @brief Drive GPIO pin high (output) using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_drive_high(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->OUTSET = (1UL << pin);
}

/**
 * @brief Set GPIO pin as input (no pull) using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_input_init(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRCLR = (1UL << pin);
}

/**
 * @brief Initialize GPIO pin as output using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_output_init(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRSET = (1UL << pin);
    PORT->PIN_CNF[pin] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Output) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Disconnect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
}

/**
 * @brief Read GPIO pin state using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 * @return true if pin is high, false if low
 */
static inline bool gpio_read(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    return (PORT->IN & (1UL << pin)) != 0;
}

/**
 * @brief Push all active GPIO pins to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 */
static inline void push_active_pins_to_stack(Stack *stack)
{
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++) {
        if (gpio_read(abs_pin)) {
            push(stack, &abs_pin);
        }
    }
}

/**
 * @brief Push all active GPIO pins except the specified one to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 * @param exclude_abs_pin Absolute pin number to exclude from pushing
 */
static inline void push_active_pins_except_to_stack(Stack *stack, uint32_t exclude_abs_pin)
{
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++) {
        if (abs_pin == exclude_abs_pin) continue;
        if (gpio_read(abs_pin)) {
            push(stack, &abs_pin);
        }
    }
}

/**
 * @brief Reset GPIO pin using absolute pin number (no-op placeholder)
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_reset(uint32_t abs_pin)
{
    // Placeholder: implement if needed
}

/**
 * @brief Initialize GPIO pin for open-drain output using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void gpio_open_drain_abs(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRCLR = (1UL << pin);
    PORT->PIN_CNF[pin] = BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
                         BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
                         BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
                         BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
                         BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);
}

// Removed duplicate single-handler declarations (using per-pin lists below instead)

/**
 * @brief Configure a pin as open-drain input with pullup and
 *        generate an interrupt on both edges, dispatching to
 *        separate rising/falling handlers.
 *
 * @param PORT            GPIO port (NRF_P0 or NRF_P1)
 * @param pin             GPIO pin number
 * @param rising_handler  Called when pin goes high
 * @param falling_handler Called when pin goes low
 */
/* static inline void gpio_open_drain_with_interrupt(
    NRF_GPIO_Type *PORT,
    uint8_t pin,
    gpio_interrupt_handler_t rising_handler,
    gpio_interrupt_handler_t falling_handler)
{
    // 1) Configure pin as open-drain input with pull-up
    PORT->DIRCLR = (1UL << pin);
    PORT->PIN_CNF[pin] =
        BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
        BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
        BV_BY_NAME(GPIO_PIN_CNF_PULL, Pullup) |
        BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0D1) |
        BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);

    // 2) Save callbacks
    rising_callback_single = rising_handler;
    falling_callback_single = falling_handler;
    gpiote_pin0 = pin;

    uint32_t abs_pin = (PORT == NRF_P0) ? pin : (pin + 32);

    // 3) Configure GPIOTE with correct pin number
    NRF_GPIOTE->CONFIG[0] =
        (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
        (abs_pin << GPIOTE_CONFIG_PSEL_Pos) |
        (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
        (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);

    // 4) Enable interrupt
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;
    NVIC_EnableIRQ(GPIOTE_IRQn);
} */

/**
 * @brief Disable the open-drain interrupt on the given pin
 *
 * @param PORT GPIO port (NRF_P0 or NRF_P1)
 * @param pin  GPIO pin number
 */
/* static inline void gpio_open_drain_disable_interrupt(NRF_GPIO_Type *PORT, uint8_t pin)
{
    if (gpiote_pin0 == pin)
    {
        // 1) Disable GPIOTE channel 0
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_IN0_Clear << GPIOTE_INTENCLR_IN0_Pos;
        NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Disabled << GPIOTE_CONFIG_MODE_Pos;
        NVIC_DisableIRQ(GPIOTE_IRQn);
        NVIC_ClearPendingIRQ(GPIOTE_IRQn);

        // 2) Clear callbacks
        rising_callback_single = NULL;
        falling_callback_single = NULL;

        // 3) Restore pin config
        PORT->DIRCLR = (1UL << pin);
        PORT->PIN_CNF[pin] =
            BV_BY_NAME(GPIO_PIN_CNF_DIR, Input) |
            BV_BY_NAME(GPIO_PIN_CNF_INPUT, Connect) |
            BV_BY_NAME(GPIO_PIN_CNF_PULL, Disabled) |
            BV_BY_NAME(GPIO_PIN_CNF_DRIVE, S0S1) |
            BV_BY_NAME(GPIO_PIN_CNF_SENSE, Disabled);

        // 4) Clear saved pin
        gpiote_pin0 = 0xFF;
    }
} */

/* void GPIOTE_IRQHandler(void)
{
    if (NRF_GPIOTE->EVENTS_IN[0])
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;

        uint8_t pin = (NRF_GPIOTE->CONFIG[0] >> GPIOTE_CONFIG_PSEL_Pos) & 0x1F;
        bool state = gpio_read(pin < 32 ? NRF_P0 : NRF_P1, pin);

        if (state)
        {
            if (rising_callback_single)
            {
                rising_callback_single(pin, state);
            }
        }
        else
        {
            if (falling_callback_single)
            {
                falling_callback_single(pin, state);
            }
        }
    }
} */



typedef struct {
    uint32_t pin;         // Pin number (0-47)
    NRF_GPIO_Type* PORT; // Pointer to the GPIO port (NRF_P0 or NRF_P1)
    uint64_t timestamp;
    bool pin_state;
} pin_time_measurement_t;

static pin_time_measurement_t time_measurements[NUMBER_OF_GPIO_PINS];

/**
 * @brief Log a pin state change with timestamp using absolute pin number
 * 
 * @param abs_pin Absolute pin number (0-47)
 * @param current_time Current timestamp in milliseconds
 * @param pin_state State of the pin (true for high, false for low)
 */
static inline void log_pin_state(uint32_t abs_pin, uint64_t current_time, bool pin_state)
{
    if (abs_pin >= NUMBER_OF_GPIO_PINS) {
        return;
    }

    time_measurements[abs_pin].pin = abs_pin;
    time_measurements[abs_pin].PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    time_measurements[abs_pin].timestamp = current_time;
    time_measurements[abs_pin].pin_state = pin_state;
}

/**
 * @brief Clear all stored time measurements
 */
static inline void clear_time_measurements(void)
{
    for (uint32_t i = 0; i < NUMBER_OF_GPIO_PINS; i++) {
        time_measurements[i].timestamp = 0;
        time_measurements[i].pin_state = false;

    }
}

/**
 * @brief Clear a specific time measurement by absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void clear_time_measurement(uint32_t abs_pin)
{
    if (abs_pin >= NUMBER_OF_GPIO_PINS) {
        return;
    }
    
    time_measurements[abs_pin].timestamp = 0;
    time_measurements[abs_pin].pin_state = false;
}

/**
 * @brief Get a specific time measurement by absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 * @return pin_time_measurement_t* Pointer to measurement or NULL if invalid pin
 */
static inline pin_time_measurement_t* get_measurement(uint32_t abs_pin)
{
    if (abs_pin >= NUMBER_OF_GPIO_PINS) {
        return NULL;
    }
    return &time_measurements[abs_pin];
}


uint64_t gpio_blacklist = 0; // Global blacklist for GPIO pins
gpio_interrupt_handler_t rising_handler_global = NULL;
gpio_interrupt_handler_t falling_handler_global = NULL;

static volatile uint32_t prev_input_state_p0 = 0;
static volatile uint32_t prev_input_state_p1 = 0;




static inline bool is_interupt_blacklisted(uint32_t abs_pin) {
    return (gpio_blacklist >> abs_pin) & 1;
}


void gpio_listen_interrupt_on_all_pins(uint64_t blacklist_mask,
                               gpio_interrupt_handler_t rising_handler,
                               gpio_interrupt_handler_t falling_handler)
{
    gpio_blacklist = blacklist_mask;
    rising_handler_global = rising_handler;
    falling_handler_global = falling_handler;

    // Configure all non-blacklisted pins as input with pullup
#define CONFIGURE_PORT_INPUT(PORT, base)                       \
    do {                                                       \
        for (int i = 0; i < 32; i++) {                         \
            uint32_t abs_pin = base + i;                       \
            if (!is_interupt_blacklisted(abs_pin)) {                    \
                PORT->PIN_CNF[i] =                             \
                    (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) | \
                    (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | \
                    (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos); \
            }                                                  \
        }                                                      \
    } while (0)

    CONFIGURE_PORT_INPUT(NRF_P0, 0);
    CONFIGURE_PORT_INPUT(NRF_P1, 32);

    // Save initial states
    prev_input_state_p0 = NRF_P0->IN;
    prev_input_state_p1 = NRF_P1->IN;

    // Enable PORT event
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}


void GPIOTE_IRQHandler(void) {
    if (NRF_GPIOTE->EVENTS_PORT) {
        NRF_GPIOTE->EVENTS_PORT = 0;

        uint32_t curr_p0 = NRF_P0->IN;
        uint32_t curr_p1 = NRF_P1->IN;

        uint32_t changed_p0 = curr_p0 ^ prev_input_state_p0;
        uint32_t changed_p1 = curr_p1 ^ prev_input_state_p1;

        uint32_t rising_p0 = changed_p0 & curr_p0;
        uint32_t falling_p0 = changed_p0 & ~curr_p0;

        uint32_t rising_p1 = changed_p1 & curr_p1;
        uint32_t falling_p1 = changed_p1 & ~curr_p1;

        // Gemeinsame Schleife für Port 0 (Pins 0-31) und Port 1 (Pins 32-47)
        for (uint32_t pin = 0; pin < 48; pin++) {
            if (is_interupt_blacklisted(pin)) continue;

            bool is_rising = false;
            bool is_falling = false;

            if (pin < 32) {
                uint32_t bit = 1UL << pin;
                is_rising  = (rising_p0 & bit) != 0;
                is_falling = (falling_p0 & bit) != 0;
            } else {
                uint32_t bit = 1UL << (pin - 32);
                is_rising  = (rising_p1 & bit) != 0;
                is_falling = (falling_p1 & bit) != 0;
            }

            if (is_falling && falling_handler_global)
                falling_handler_global(pin);
            if (is_rising && rising_handler_global)
                rising_handler_global(pin);
        }

        prev_input_state_p0 = curr_p0;
        prev_input_state_p1 = curr_p1;
    }
}




/**
 * @brief Release GPIO pin from open-drain state (set as input) using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
static inline void release_gpio_open_drain(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRCLR = (1UL << pin); // Set pin as input
}

/**
 * @brief Get the elapsed time in microseconds between two timer values
 * @param start Start time in timer ticks
 * @param current Current time in timer ticks
 */
static inline uint32_t get_elapsed_time(uint32_t start, uint32_t current)
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
static inline void delay_us(uint32_t us)
{
    NRF_TIMER1->TASKS_STOP = 1;
    NRF_TIMER1->TASKS_CLEAR = 1;

    NRF_TIMER1->PRESCALER = 4; // 1 MHz
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

    NRF_TIMER1->TASKS_START = 1;

    // Capture current timer value into CC[1]
    NRF_TIMER1->TASKS_CAPTURE[1] = 1;
    uint32_t start = NRF_TIMER1->CC[1];

    while (1)
    {
        NRF_TIMER1->TASKS_CAPTURE[1] = 1;
        uint32_t now = NRF_TIMER1->CC[1];
        if ((now - start) >= us)
            break;
    }

    NRF_TIMER1->TASKS_STOP = 1;
}

/**
 * @brief Delay for a specified number of milliseconds
 *
 * @param ms Number of milliseconds to delay
 */
static inline void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

/**
 * @brief Wait for a signal on a GPIO pin (absolute pin number) with optional timeout
 *
 * @param abs_pin Absolute pin number (0-47)
 * @param level Expected signal level (true for high, false for low)
 * @param timeout_us Timeout in microseconds (0 for no timeout)
 * @return true if signal is detected within timeout, false if timed out
 */
static inline bool wait_for_signal_abs(uint32_t abs_pin, bool level, uint32_t timeout_us)
{
    uint32_t start = get_timer_counter();
    if (timeout_us == 0)
    {
        while (gpio_read(abs_pin) != level)
        {
        }
        return true;
    }
    while (gpio_read(abs_pin) != level)
    {
        uint32_t current = get_timer_counter();
        uint32_t elapsed = get_elapsed_time(start, current);
        if (elapsed >= timeout_us)
            return false;
    }
    return true;
}

/**
 * @brief Check if a signal is active on a GPIO pin with debounce
 *
 * @param pin GPIO pin number
 * @param assert_high True if checking for high signal, false for low
 * @return true if signal is stable, false if not
 */
static inline bool is_signal_active(uint32_t pin, bool assert_high)
{
    for (uint32_t i = 0; i < DEBOUNCE_SAMPLES; ++i)
    {
        if (gpio_read(pin) != assert_high)
            return false;
        delay_us(DEBOUNCE_DELAY_US);
    }
    return true;
}

/**
 * @brief Get the timer ticks object
 *
 * @return uint64_t
 */
static inline uint64_t get_timer_ticks(void)
{
    uint32_t current = get_timer_counter();
    return (uint64_t)get_elapsed_time(ticks_at_starting_point, current);
}

// Start timer (record current time)
static inline void start_timer(void)
{
    NRF_TIMER0->TASKS_START = 1; // Start TIMER0
    NRF_TIMER0->PRESCALER = 4;   // 16 MHz / 16 = 1 MHz
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
}

/**
 * @brief This function stops the TIMER0 peripheral, which is used for timing operations.
 *
 */
static inline void stop_timer(void)
{
    NRF_TIMER0->TASKS_STOP = 1; // Stop TIMER0
}

/**
 * @brief Reset the timer counter to zero
 *
 */
static inline void reset_timer(void)
{
    NRF_TIMER0->TASKS_CLEAR = 1; // Clear the timer counter
}

/**
 * @brief Get the current timer in microseconds
 *
 * @return uint64_t Current timer value in microseconds
 */
static inline uint32_t ticks_to_us(uint64_t ticks)
{
    return (uint32_t)ticks;
}

/**
 * @brief Convert timer ticks to milliseconds
 *
 * @param ticks Timer ticks
 * @return uint32_t Time in milliseconds
 */
static inline uint32_t ticks_to_ms(uint64_t ticks)
{
    return (uint32_t)(ticks / 1000);
}

/**
 * @brief Get the current timer value in microseconds
 *
 * @return uint64_t Current timer value in microseconds
 */
static inline uint32_t timer_diff_us(uint64_t start, uint64_t end)
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
static inline uint32_t timer_diff_ms(uint64_t start, uint64_t end)
{
    return (uint32_t)((end - start) / 1000);
}

/**
 * @brief Generate a random 32-bit number using the LFSR algorithm
 * from Wikipedia: https://de.wikipedia.org/wiki/Linear_r%C3%BCckgekoppeltes_Schieberegister
 *
 */
static inline uint32_t random32_lfsr(void)
{
    static unsigned r = 1;
    unsigned b = r & 1;
    r = (r >> 1) ^ (-b & 0xc3308398);
    return b;
}

/**
 * @brief Get a random 32-bit number using the RNG peripheral
 *
 * @return uint32_t
 */
static inline uint32_t random32(void)
{
    if (NRF_RNG->TASKS_START == 0)
    {
        NRF_RNG->TASKS_START = 1;
    }
    while (!NRF_RNG->EVENTS_VALRDY)
    {
    }
    uint32_t rnd = NRF_RNG->VALUE;
    NRF_RNG->EVENTS_VALRDY = 0;
    return rnd;
}


/**
 * @brief Selects a random pin that is not blacklisted and not successful from a PinData array
 *
 * @param pindata Pointer to PinData array
 * @param length Number of elements in the array
 * @return uint32_t Pin number, or 0xFFFFFFFF if none available
 */
static inline uint32_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint32_t length)
{
    // Count valid pins
    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < length; ++i) {
        if (!is_blacklisted(&pindata[i]) && !is_successful(&pindata[i])) {
            valid_count++;
        }
    }
    if (valid_count == 0) {
        return 0xFFFFFFFF;
    }
    // Pick a random valid index
    uint32_t pick = random32() % valid_count;
    for (uint32_t i = 0; i < length; ++i) {
        if (!is_blacklisted(&pindata[i]) && !is_successful(&pindata[i])) {
            if (pick == 0) {
                return pindata[i].pin;
            }
            pick--;
        }
    }
    return 0xFFFFFFFF;
}





uint32_t bibanging_uart_baudtrate = -1;   // Default baud rate for UART
uint32_t bibanging_uart_bit_time_us = -1; // Bit time in microseconds
uint8_t UART_PIN = -1;

/**
 * @brief Initialize software serial for bit-banging UART using absolute pin number
 *
 * This function sets up a GPIO pin for bit-banging UART transmission using the absolute pin number.
 * It calculates the bit time based on the specified baud rate.
 *
 * @param abs_pin Absolute pin number (0-47)
 * @param baudrate Baud rate for UART communication
 */
static inline void init_software_serial(uint32_t abs_pin, uint32_t baudrate)
{
    bibanging_uart_baudtrate = baudrate;
    bibanging_uart_bit_time_us = (uint32_t)(1000000 / baudrate); // Calculate bit time in microseconds
    UART_PIN = (abs_pin < 32) ? abs_pin : (abs_pin - 32);

    // Initialize GPIO pin for open-drain output
    gpio_open_drain_abs(abs_pin);
    release_gpio_open_drain(abs_pin); // Set pin to high (open-drain release state)
}
/**
 * @brief Transmit a byte via bit-banging UART
 *
 * This function sends a byte using bit-banging technique over a specified GPIO pin.
 * It assumes the UART has been initialized with init_software_serial().
 *
 * @param byte The byte to transmit
 */
static inline void software_serial_tx(uint8_t byte)
{
    if ( UART_PIN == -1 || bibanging_uart_bit_time_us == -1)
    {
        // UART not initialized
        return;
    }
    // Startbit (Low)
    gpio_drive_low(UART_PIN);
    delay_us(bibanging_uart_bit_time_us);

    // 8 Datenbits (LSB first)
    for (int i = 0; i < 8; i++)
    {
        if (byte & (1 << i))
        {
            release_gpio_open_drain(UART_PIN); // High (loslassen)
        }
        else
        {
            gpio_drive_low(UART_PIN); // Low
        }
        delay_us(bibanging_uart_bit_time_us);
    }

    // Stopbit (High)
    release_gpio_open_drain(UART_PIN);
    delay_us(bibanging_uart_bit_time_us);
}

/**
 * @brief Get the unique id object
 *
 * @return uint64_t
 */
static inline uint64_t get_unique_id(void)
{
    return ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
}

/**
 * @brief Get the unique id as a string
 *
 * @return const char*
 */
static inline const char *get_unique_id_str(void)
{
    static char unique_id_str[17];
    uint64_t unique_id = get_unique_id();
    const char hex[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++)
    {
        uint8_t byte = (unique_id >> (56 - i * 8)) & 0xFF;
        unique_id_str[i * 2] = hex[byte >> 4];
        unique_id_str[i * 2 + 1] = hex[byte & 0x0F];
    }
    unique_id_str[16] = '\0';
    return unique_id_str;
}

/**
 * @brief Get the chip family name
 *
 * @return const char*
 */
static inline const char *get_chip_family_name(void)
{
    return "NRF52840";
}


/**
 * @brief Get the absolute pin number object    
 * 
 * @param PORT 
 * @param pin 
 * @return const uint32_t 
 */
static inline const uint32_t get_absolute_pin_number(NRF_GPIO_Type *PORT, uint8_t pin)
{
    if (PORT == NRF_P0)
    {
        return pin; // Pins 0-31
    }
    else if (PORT == NRF_P1)
    {
        return pin + 32; // Pins 32-47
    }
    return 0xFFFF; // Invalid pin
}





#endif // NRF52840_HELPER_H