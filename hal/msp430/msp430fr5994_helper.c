#include "msp430fr5994_helper.h"

volatile bool delay_done = false;
volatile uint32_t overflow_count = 0;
volatile uint32_t remaining_ticks = 0;

uint64_t gpio_blacklist_intern_mask = 0;
gpio_interrupt_handler_t global_falling_handler = NULL;
gpio_interrupt_handler_t global_rising_handler = NULL;
uint64_t previous_state = 0;

static volatile uint8_t *const PxIE[] = {&P1IE, &P2IE, &P3IE, &P4IE, &P5IE, &P6IE, &P7IE, &P8IE};
static volatile uint8_t *const PxIFG[] = {&P1IFG, &P2IFG, &P3IFG, &P4IFG, &P5IFG, &P6IFG, &P7IFG, &P8IFG};
static volatile uint8_t *const PxIES[] = {&P1IES, &P2IES, &P3IES, &P4IES, &P5IES, &P6IES, &P7IES, &P8IES};

volatile uint16_t timer_overflows_a1 = 0;
volatile uint16_t timer_overflows_a2 = 0;

volatile uint16_t timer_overflows_a4 = 0;
volatile uint16_t timer_overflows_b0 = 0;

/**
 * @brief Get the port of absolute pin object
 *
 * @param abs_pin
 * @return uint32_t
 */
uint16_t get_port_base_of_absolute_pin(uint32_t abs_pin)
{
    if (abs_pin >= 0 && abs_pin < 8)
    { // P1.0 to P1.7
        return P1_BASE;
    }
    else if (abs_pin >= 8 && abs_pin < 16)
    { // P2.0 to P2.7
        return P2_BASE;
    }
    else if (abs_pin >= 16 && abs_pin < 24)
    { // P3.0 to P3.7
        return P3_BASE;
    }
    else if (abs_pin >= 24 && abs_pin < 32)
    { // P4.0 to P4.7
        return P4_BASE;
    }
    else if (abs_pin >= 32 && abs_pin < 40)
    { // P5.0 to P5.7
        return P5_BASE;
    }
    else if (abs_pin >= 40 && abs_pin < 48)
    { // P6.0 to P6.7
        return P6_BASE;
    }
    else if (abs_pin >= 48 && abs_pin < 56)
    { // P7.0 to P7.7
        return P7_BASE;
    }
    else if (abs_pin >= 56 && abs_pin < 64)
    { // P8.0 to P8.7
        return P8_BASE;
    }
    return 0; // Invalid pin
}

/**
 * @brief Get the relative pin object
 *
 * @param abs_pin
 * @return uint8_t
 */
uint8_t get_relative_pin(uint8_t abs_pin)
{
    return abs_pin % 8; // Return the pin number within the port
}

static void configure_timer(volatile uint16_t *timer_ctl)
{
    *timer_ctl = TASSEL__SMCLK // Use SMCLK as clock source (z.B. 16 MHz)
                 | ID__1       // Divide input clock by 1 (no division)
                 | MC__STOP    // Timer stopped initially
                 | TACLR       // Clear the timer
                 | TAIE;       // Enable overflow interrupt
}

/**
 * @brief Initialize the I/O subsystem
 *
 * This function initializes the GPIO, UART, and Timer modules.
 */
void io_init()
{
    // Disable watchdog
    WDTCTL = WDTPW | WDTHOLD;

    // Unlock GPIO
    PM5CTL0 &= ~LOCKLPM5;

    // Configure clock to 16MHz
    FRCTL0 = FRCTLPW | NWAITS_1;
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_4 | DCORSEL;
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;

    // Configure UART
    UCA0CTLW0 = UCSWRST;         // Reset UART
    UCA0CTLW0 |= UCSSEL__SMCLK;  // SMCLK source (16MHz)
    UCA0BR0 = 104;               // 16MHz/9600 = 1666.67
    UCA0BR1 = 0;                 // High byte
    UCA0MCTLW = UCOS16 | 0x4900; // Oversampling + fractional tuning
    UCA0CTLW0 &= ~UCSWRST;       // Enable UART

    // Configure UART pins
    P2SEL0 &= ~(BIT0 | BIT1); // Clear P2.0/P2.1 SEL0
    P2SEL1 |= BIT0 | BIT1;    // Set UART function

    configure_timer((volatile uint16_t *)&TA1CTL); // Configure Timer A1
    configure_timer((volatile uint16_t *)&TA4CTL); // Configure Timer A4
    configure_timer((volatile uint16_t *)&TB0CTL); // Configure Timer B0

    __enable_interrupt(); // Enable global interrupts
}

/**
 * @brief Initialize GPIO pin with pull-up resistor
 *
 * @param abs_pin Absolute pin number
 */
void gpio_pullup_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Enable pull-up resistor (REN = 1, OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}

/**
 * @brief Initialize GPIO pin with pull-down resistor
 *
 * @param abs_pin Absolute pin number
 */
void gpio_pulldown_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Enable pull-down resistor (REN = 1, OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) |= mask;
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}

/**
 * @brief Pull-up resistor clear
 * @param abs_pin Absolute pin number
 */
void gpio_pullup_clear(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Disable pull-up resistor (REN = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
    // Ensure pin is driven low (OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}
/**
 * @brief Pull-down resistor clear
 * @param abs_pin Absolute pin number
 *
 */
void gpio_pulldown_clear(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Disable pull-down resistor (REN = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_REN_OFFSET)) &= ~mask;
    // Ensure pin is driven high (OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}
/**
 * @brief Drive GPIO pin low
 *
 * @param abs_pin Absolute pin number
 */
void gpio_drive_low(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin low (OUT = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) &= ~mask;
}
/**
 * @brief Drive GPIO pin high
 * @param abs_pin Absolute pin number
 */
void gpio_drive_high(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin high (OUT = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_OUT_OFFSET)) |= mask;
}
/**
 * @brief Initialize GPIO pin as input
 * @param abs_pin Absolute pin number
 */
void gpio_input_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin as input (DIR = 0)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) &= ~mask;
}
/**
 * @brief Initialize GPIO pin as output
 * @param abs_pin Absolute pin number
 */
void gpio_output_init(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Set pin as output (DIR = 1)
    *(volatile uint8_t *)((uintptr_t)(base + PORT_DIR_OFFSET)) |= mask;
}
/**
 * @brief Read GPIO pin state
 *
 * @param abs_pin Absolute pin number
 * @return true if pin is high, false if low
 */
bool gpio_read(uint8_t abs_pin)
{
    uint16_t base = get_port_base_of_absolute_pin(abs_pin);
    uint8_t mask = 1 << get_relative_pin(abs_pin);
    // Read pin state (IN)
    return (*(volatile uint8_t *)((uintptr_t)(base + PORT_IN_OFFSET)) & mask) != 0;
}

void delay_ticks(uint32_t ticks)
{
    if (ticks == 0) return;

    // Stop and clear timer
    TA0CTL = TASSEL__SMCLK | ID__1 | MC__STOP | TACLR;
    TA0R = 0;

    remaining_ticks = ticks;
    
    // Calculate the number of overflows needed
    // if the remaining ticks are less than 0x10000, we can set it directly
    if (remaining_ticks <= 0x10000) {
        // Single period
        TA0CCR0 = remaining_ticks - 1;
        overflow_count = 0;
    } else {
        // Multiple periods are needed so we calculate the overflow count
        overflow_count = (remaining_ticks - 1) / 0x10000;
        TA0CCR0 = 0xFFFF;  // Full period first
    }

    TA0CCTL0 = CCIE;
    if (overflow_count > 0) {
        TA0CTL |= TAIE;  // Enable overflow interrupt if needed
    }

    delay_done = false;
    TA0CTL |= MC__UP;  // Start timer

    while (!delay_done);

    // Cleanup
    TA0CCTL0 &= ~CCIE;
    TA0CTL &= ~(TAIE | MC__UP);
    TA0CTL |= TACLR;
}


// This is needed to quit the delay loop when the timer reaches the target ticks
__attribute__((interrupt(TIMER0_A0_VECTOR))) 
void Timer0_A0_ISR(void)
{
    // Clear the interrupt flag
    TA0CCTL0 &= ~CCIFG;
    if (overflow_count == 0) {
        // Last tick reached, signal completion
        delay_done = true;
    } else {
        // Prepare for next overflow
        TA0CCR0 = 0xFFFF;
    }
}


__attribute__((interrupt(TIMER0_A1_VECTOR))) 
void Timer0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
        // Handle Timer A0 overflow
        case TA0IV_TAIFG:
            if (overflow_count > 0) {
                overflow_count--;
                if (overflow_count == 0) {
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
void delay_us(uint32_t us)
{
    if (us == 0) return;
    uint32_t ticks = us * (SMCLK_HZ / 1000000);
    delay_ticks(ticks);
}

/**
 * @brief Busy-wait delay in milliseconds using Timer A0
 * @param ms Milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    if (ms == 0) return;

    const uint32_t ticks_per_ms = SMCLK_HZ / 1000;
    
    while (ms >= 1000) {
        delay_ticks(SMCLK_HZ);  // Exactly 1 second
        ms -= 1000;
    }
    // Handle remaining milliseconds
    if (ms > 0) {
        uint32_t ticks = ms * ticks_per_ms;
        delay_ticks(ticks);
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
    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
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

bool is_interupt_blacklisted(uint8_t abs_pin)
{
    return (gpio_blacklist_intern_mask >> abs_pin) & 1;
}

uint64_t read_all_gpio_states()
{
    uint64_t state = 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P1IN) << 0;
    state |= (uint64_t)(*(volatile uint8_t *)&P2IN) << 8;
    state |= (uint64_t)(*(volatile uint8_t *)&P3IN) << 16;
    state |= (uint64_t)(*(volatile uint8_t *)&P4IN) << 24;
    state |= (uint64_t)(*(volatile uint8_t *)&P5IN) << 32;
    state |= (uint64_t)(*(volatile uint8_t *)&P6IN) << 40;
    state |= (uint64_t)(*(volatile uint8_t *)&P7IN) << 48;
    state |= (uint64_t)(*(volatile uint8_t *)&P8IN) << 56;
    return state;
}

/**
 * @brief Listen on all GPIO pins for rising and falling edges
 *
 * @param blacklist_mask Bitmask of pins to ignore (1 for ignored, 0 for active)
 * @param falling_handler Handler for falling edges
 * @param rising_handler Handler for rising edges
 */
void gpio_listen_on_all_pins_interrupt(uint64_t blacklist_mask,
                                       gpio_interrupt_handler_t falling_handler,
                                       gpio_interrupt_handler_t rising_handler)
{
    gpio_blacklist_intern_mask = blacklist_mask;
    global_falling_handler = falling_handler;
    global_rising_handler = rising_handler;

    for (uint8_t abs_pin = 0; abs_pin < NUMBER_OF_GPIO_PINS; abs_pin++)
    {
        if ((blacklist_mask >> abs_pin) & 1)
            continue; // Pin ignorieren

        gpio_pullup_init(abs_pin); // Pull-up aktivieren
        gpio_input_init(abs_pin);  // Pin als Eingang konfigurieren

        uint8_t port = abs_pin >> 3;
        uint8_t pin = abs_pin & 0x07;

        *PxIFG[port] &= ~(1 << pin); // Clear Interrupt Flag
        *PxIES[port] |= (1 << pin);  // Falling Edge (default)
        *PxIE[port] |= (1 << pin);   // Enable Interrupt
    }

    __enable_interrupt(); // Global Interrupt Enable
}

// Macro to define an Interrupt Service Routine (ISR) for a given GPIO port
// Handles both falling and rising edges by toggling the edge detection after each interrupt
#define DEFINE_PORT_ISR(port)                                                   \
    void __attribute__((interrupt(PORT##port##_VECTOR))) PORT##port##_ISR(void) \
    {                                                                           \
        /* Get only active and enabled interrupt flags for this port */         \
        uint8_t flags = P##port##IFG & P##port##IE;                             \
                                                                                \
        for (uint8_t pin = 0; pin < 8; pin++)                                   \
        {                                                                       \
            if (flags & (1 << pin))                                             \
            {                                                                   \
                /* Compute absolute pin index, e.g., P3.4 → 20 */               \
                uint8_t abs_pin = ((port - 1) << 3) | pin;                      \
                                                                                \
                /* Skip if this pin is blacklisted */                           \
                if ((gpio_blacklist_intern_mask >> abs_pin) & 1)                \
                    continue;                                                   \
                /* Check if the current edge setting is falling */              \
                bool is_falling = (P##port##IES >> pin) & 1;                    \
                                                                                \
                if (is_falling)                                                 \
                {                                                               \
                    /* Falling edge detected → call falling edge handler */     \
                    if (global_falling_handler)                                 \
                        global_falling_handler(abs_pin);                        \
                                                                                \
                    /* Switch to rising edge detection for next time */         \
                    P##port##IES &= ~(1 << pin);                                \
                }                                                               \
                else                                                            \
                {                                                               \
                    /* Rising edge detected → call rising edge handler */       \
                    if (global_rising_handler)                                  \
                        global_rising_handler(abs_pin);                         \
                                                                                \
                    /* Switch to falling edge detection for next time */        \
                    P##port##IES |= (1 << pin);                                 \
                }                                                               \
                                                                                \
                /* Clear the interrupt flag (ACK) */                            \
                P##port##IFG &= ~(1 << pin);                                    \
            }                                                                   \
        }                                                                       \
    }

// Port ISR Definitions
DEFINE_PORT_ISR(1)
DEFINE_PORT_ISR(2)
DEFINE_PORT_ISR(3)
DEFINE_PORT_ISR(4)
DEFINE_PORT_ISR(5)
DEFINE_PORT_ISR(6)
DEFINE_PORT_ISR(7)
DEFINE_PORT_ISR(8)

/**
 * @brief Start the specified timer in continuous mode
 *
 * @param timer Timer to start (TIMER_A4, TIMER_B0)
 */
void start_timer(timer_t timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);

    // Reset overflow counter for the selected timer
    if (timer == TIMER_A4)
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
 * @param timer Timer to stop (TIMER_A4, TIMER_B0)
 */
void stop_timer(timer_t timer)
{
    volatile uint16_t *ctl = GET_TxxCTL(timer);
    volatile uint16_t *r = GET_TxxR(timer);

    *ctl &= ~MC_3; // Clear MC bits → stop mode
    *ctl |= TACLR; // Clear timer (setzt den Zähler zurück)
    *r = 0;        // Zur Sicherheit Timer-Register auf 0 schreiben
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

/**
 * @brief Get the timer ticks object
 *
 * @param timer_r Pointer to the timer register (TA0R, TA1R, TA4R, TB0R)
 * @return uint32_t Timer ticks
 */
uint32_t get_timer_ticks(timer_t timer)
{
    volatile uint16_t *timer_r = GET_TxxR(timer);
    uint16_t counter = *timer_r;
    uint16_t overflows = 0;

    switch (timer)
    {
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
 * @brief Convert timer ticks to microseconds (assumes 500 kHz clock)
 * @param ticks Timer ticks
 * @return Time in microseconds
 */
uint32_t ticks_to_us(uint32_t ticks)
{
    return ticks / 16; // For 500kHz timer (1 tick = 2 µs)
}

/**
 * @brief Convert timer ticks to milliseconds
 * @param ticks Timer ticks
 * @return Time in milliseconds
 */
uint32_t ticks_to_ms(uint32_t ticks)
{
    return ticks / (TIMER_FREQ_HZ / 1000);
}

/**
 * @brief Calculate time difference in microseconds
 * @param start Start tick count
 * @param end End tick count
 * @return Elapsed time in microseconds
 */
uint32_t timer_diff_us(uint32_t start, uint32_t end)
{
    return (end - start) * 1000000UL / TIMER_FREQ_HZ;
}

/**
 * @brief Calculate time difference in milliseconds
 * @param start Start tick count
 * @param end End tick count
 * @return Elapsed time in milliseconds
 */
uint32_t timer_diff_ms(uint32_t start, uint32_t end)
{
    return (end - start) / (TIMER_FREQ_HZ / 1000UL);
}

/**
 * @brief Get the unique device ID from TLV memory
 *
 * @return uint32_t Unique device ID
 */
uint64_t get_unique_id(void)
{
    // Not available on MSP430 devices.
    // Maybe return the random number.
    return 0;
}

/**
 * @brief Get the unique device ID as a string
 *
 * @return const char* Pointer to a static buffer containing the hex string
 */
const char *get_unique_id_str(void)
{
    static char buf[9];
    uint32_t id = get_unique_id();
    // Forat as 8-digit hexadecimal string
    snprintf(buf, sizeof(buf), "%08lX", (unsigned long)id);
    return buf;
}

/**
 * @brief Get the family name of the chip
 *
 * @return const char* Chin family name
 */
const char *get_chip_family_name(void)
{
    return "MSP430FR5994";
}
