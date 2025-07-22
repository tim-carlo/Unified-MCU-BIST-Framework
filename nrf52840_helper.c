#include "nrf52840_helper.h"
#include <string.h>

// Variable definitions

gpio_interrupt_handler_t rising_callback_single = NULL;
gpio_interrupt_handler_t falling_callback_single = NULL;
uint8_t gpiote_pin0 = 0xFF;

uint32_t bibanging_uart_baudtrate = -1;
uint32_t bibanging_uart_bit_time_us = -1;
uint8_t UART_PIN = -1;
pin_time_measurement_t time_measurements[NUMBER_OF_GPIO_PINS];

uint64_t gpio_blacklist_intern_mask = 0; // Global blacklist for GPIO pins
gpio_interrupt_handler_t rising_handler_global = NULL;
gpio_interrupt_handler_t falling_handler_global = NULL;

static volatile uint32_t prev_input_state_p0 = 0;
static volatile uint32_t prev_input_state_p1 = 0;

/**
 * @brief Initialize the IO peripherals
 *
 * This function initializes the UART for printf output and starts the HFCLK.
 * It should be called at the beginning of the main function.
 */
void io_init(void)
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
 * @brief Initialize GPIO pin with pull-up resistor using absolute pin number
 *
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_pullup_init(uint32_t abs_pin)
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
void gpio_pulldown_init(uint32_t abs_pin)
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
void gpio_pullup_clear(uint32_t abs_pin)
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
void gpio_drive_low(uint32_t abs_pin)
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
void gpio_drive_high(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->OUTSET = (1UL << pin);
}

/**
 * @brief Set GPIO pin as input (no pull) using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_input_init(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    PORT->DIRCLR = (1UL << pin);
}

/**
 * @brief Initialize GPIO pin as output using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_output_init(uint32_t abs_pin)
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
bool gpio_read(uint32_t abs_pin)
{
    NRF_GPIO_Type *PORT = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint8_t pin = (abs_pin < 32) ? abs_pin : (abs_pin - 32);
    return (PORT->IN & (1UL << pin)) != 0;
}

/**
 * @brief Push all active GPIO pins to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 * @param level Level to check for active pins (true for high, false for low)
 */
void push_active_pins_to_stack(Stack *stack, uint8_t level)
{
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if (gpio_read(abs_pin) == level)
        {
            printf("Pushing active pin %lu to stack\n", abs_pin);
            push(stack, &abs_pin);
        }
    }
}

/**
 * @brief Push all active GPIO pins except the specified one to a stack using absolute pin numbers
 *
 * @param stack Pointer to the stack where active pins will be pushed
 * @param expected_level Expected level of the pins (true for high, false for low)
 * @param blacklist_mask Bitmask of pins to exclude (1 for excluded, 0 for included)
 */
void push_active_pins_except_blacklist_to_stack(Stack *stack, bool expected_level, uint64_t blacklist_mask)
{
    for (uint32_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue;
        if (gpio_read(abs_pin) == expected_level)
        {
            printf("Pushing active pin %lu to stack\n", abs_pin);
            push(stack, &abs_pin);
        }
    }
}

/**
 * @brief Reset GPIO pin using absolute pin number (no-op placeholder)
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_reset(uint32_t abs_pin)
{
    // Placeholder: implement if needed
}

/**
 * @brief Initialize GPIO pin for open-drain output using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void gpio_open_drain(uint32_t abs_pin)
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
bool is_interupt_blacklisted(uint32_t abs_pin)
{
    return (gpio_blacklist_intern_mask >> abs_pin) & 1;
}


/**
 * @brief Configure pin sense for interrupts using absolute pin number
 * 
 * @param abs_pin Absolute pin number (0-47)
 * @param sense_low If true, configure for low sense (falling edge), else for high sense (rising edge)
 */
static void configure_pin_sense(uint32_t abs_pin, bool sense_low)
{
    NRF_GPIO_Type *port = (abs_pin < 32) ? NRF_P0 : NRF_P1;
    uint32_t pin = abs_pin % 32;

    // Clear SENSE bits first
    port->PIN_CNF[pin] &= ~GPIO_PIN_CNF_SENSE_Msk;

    if (sense_low)
        port->PIN_CNF[pin] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    else
        port->PIN_CNF[pin] |= (GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);
}
/**
 * @brief Function to listen for GPIO interrupts on all pins, excluding blacklisted ones
 * 
 * @param blacklist Bitmask of pins to exclude (1 for excluded, 0 for included)
 * @param falling_handler Function to call on falling edge
 * @param rising_handler Function to call on rising edge
 */
void gpio_listen_interrupt_on_all_pins(uint64_t blacklist,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    gpio_blacklist_intern_mask = blacklist;
    falling_handler_global = falling_handler;
    rising_handler_global = rising_handler;

    for (uint32_t abs_pin = 0; abs_pin < 48; abs_pin++)
    {
        if (is_interupt_blacklisted(abs_pin))
            continue;

        NRF_GPIO_Type *port = (abs_pin < 32) ? NRF_P0 : NRF_P1;
        uint32_t pin_idx = abs_pin % 32;

        // Set pin as input with pull-up resistor
        port->PIN_CNF[pin_idx] =
            (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
            (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    }

    // Clear all latch bits to ensure no false triggers
    NRF_P0->LATCH = 0xFFFFFFFF;
    NRF_P1->LATCH = 0xFFFFFFFF;

    NRF_GPIOTE->EVENTS_PORT = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

void GPIOTE_IRQHandler(void)
{
    if (!NRF_GPIOTE->EVENTS_PORT)
        return;

    NRF_GPIOTE->EVENTS_PORT = 0; // Clear the event

    for (uint32_t abs_pin = 0; abs_pin < 48; abs_pin++)
    {
        if (is_interupt_blacklisted(abs_pin))
            continue;

        NRF_GPIO_Type *port = (abs_pin < 32) ? NRF_P0 : NRF_P1;
        uint32_t pin_idx = abs_pin % 32;

        if (port->LATCH & (1UL << pin_idx))
        {
            bool pin_state = (port->IN >> pin_idx) & 1;
            // Depending on the current state, call the appropriate handler
            if (!pin_state)
            {
                if (falling_handler_global)
                    falling_handler_global(abs_pin);
                // Next: SENSE_High (for Rising)
                configure_pin_sense(abs_pin, false);
            }
            else
            {
                if (rising_handler_global)
                    rising_handler_global(abs_pin);
                // Next: SENSE_Low (for Falling)
                configure_pin_sense(abs_pin, true);
            }

            // Clear the latch bit for this pin
            port->LATCH = (1UL << pin_idx);
        }
    }
}

/**
 * @brief Release GPIO pin from open-drain state (set as input) using absolute pin number
 * @param abs_pin Absolute pin number (0-47)
 */
void release_gpio_open_drain(uint32_t abs_pin)
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

    NRF_TIMER3->PRESCALER = 4; // 1 MHz
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;

    NRF_TIMER3->TASKS_START = 1;

    // Capture current timer value into CC[1]
    NRF_TIMER3->TASKS_CAPTURE[1] = 1;
    uint32_t start = NRF_TIMER3->CC[1];

    while (1)
    {
        NRF_TIMER3->TASKS_CAPTURE[1] = 1;
        uint32_t now = NRF_TIMER3->CC[1];
        if ((now - start) >= us)
            break;
    }

    NRF_TIMER3->TASKS_STOP = 1;
}

/**
 * @brief Delay for a specified number of milliseconds
 *
 * @param ms Number of milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}

/**
 * @brief Check if a signal is active on a GPIO pin with debounce
 *
 * @param pin GPIO pin number
 * @param assert_high True if checking for high signal, false for low
 * @return true if signal is stable, false if not
 */
bool is_signal_active(uint32_t pin, bool assert_high)
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
void reset_timer(void)
{
    // Not needed, as the timer is cleared in start_timer()
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
 * @brief Generate a random 32-bit number using the LFSR algorithm
 * from Wikipedia: https://de.wikipedia.org/wiki/Linear_r%C3%BCckgekoppeltes_Schieberegister
 *
 */
uint32_t random32_lfsr(void)
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
uint32_t random32(void)
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
uint32_t select_random_non_blacklisted_and_not_successful_pin(PinData *pindata, uint64_t blacklist_mask)
{
    // Count valid pins that are not blacklisted and not successful
    uint32_t valid_count = 0;
    for (uint32_t i = 0; i < NUMBER_OF_GPIO_PINS; ++i)
    {
        uint32_t pin = pindata[i].pin;
        if (pin >= 64)
            continue; // Skip invalid pins
        if (((blacklist_mask >> pin) & 1) == 0 && !is_successful(&pindata[i]))
        {
            valid_count++;
        }
    }

    // If no valid pins found, return 0xFFFFFFFF
    if (valid_count == 0)
    {
        return 0xFFFFFFFF;
    }

    // Select a random index from the valid pins
    printf("Valid pins count: %u\n", valid_count);
    uint32_t pick = random32() % valid_count;
    printf("Random pick index: %u\n", pick);

    // Iterate through the pins again to find the selected one
    for (uint32_t i = 0; i < NUMBER_OF_GPIO_PINS; ++i)
    {
        uint32_t pin = pindata[i].pin;
        if (pin >= 64)
            continue;
        if (((blacklist_mask >> pin) & 1) == 0 && !is_successful(&pindata[i]))
        {
            if (pick == 0)
            {
                return pin;
            }
            pick--;
        }
    }

    return 0xFFFFFFFF;
}

/**
 * @brief Initialize software serial for bit-banging UART using absolute pin number
 *
 * This function sets up a GPIO pin for bit-banging UART transmission using the absolute pin number.
 * It calculates the bit time based on the specified baud rate.
 *
 * @param abs_pin Absolute pin number (0-47)
 * @param baudrate Baud rate for UART communication
 */
void init_software_serial(uint32_t abs_pin, uint32_t baudrate)
{
    bibanging_uart_baudtrate = baudrate;
    bibanging_uart_bit_time_us = (uint32_t)(1000000 / baudrate); // Calculate bit time in microseconds
    UART_PIN = (abs_pin < 32) ? abs_pin : (abs_pin - 32);

    // Initialize GPIO pin for open-drain output
    gpio_open_drain(abs_pin);
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
void software_serial_tx(uint8_t byte)
{
    if (UART_PIN == -1 || bibanging_uart_bit_time_us == -1)
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
uint64_t get_unique_id(void)
{
    return ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
}

/**
 * @brief Get the unique id as a string
 *
 * @return const char*
 */
const char *get_unique_id_str(void)
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
const char *get_chip_family_name(void)
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
uint32_t get_absolute_pin_number(NRF_GPIO_Type *PORT, uint8_t pin)
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
