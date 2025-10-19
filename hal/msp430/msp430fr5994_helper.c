#include "msp430fr5994_helper.h"

uint32_t lfsr32;
uint32_t lfsr31;

static void configure_timer(volatile uint16_t *timer_ctl, uint16_t divider_setting)
{
    *timer_ctl = TASSEL__SMCLK     // Use SMCLK as clock source (16 MHz)
                 | divider_setting // Variable divider setting
                 | MC__STOP        // Timer stopped initially
                 | TACLR           // Clear the timer
                 | TAIE;           // Enable overflow interrupt
}

// Read 128 bits from the RNG peripheral
static void get_random_128(uint8_t *buffer)
{
    for (int i = 0; i < 16; i++)
    {
        buffer[i] = RNG_BYTES[i];
    }
}

// Initialize seeds for LFSR generators
void init_seeds(void)
{
    uint8_t rand128[16];
    get_random_128(rand128);

    lfsr32 = ((uint32_t)rand128[0] << 24) |
             ((uint32_t)rand128[1] << 16) |
             ((uint32_t)rand128[2] << 8) |
             ((uint32_t)rand128[3]);

    lfsr31 = ((uint32_t)rand128[4] << 24) |
             ((uint32_t)rand128[5] << 16) |
             ((uint32_t)rand128[6] << 8) |
             ((uint32_t)rand128[7]);

    // Ensure seeds are non-zero
    if (lfsr32 == 0)
        lfsr32 = 0x1;
    if (lfsr31 == 0)
        lfsr31 = 0x1;

    // Ensure lfsr31 is 31 bits
    lfsr31 &= 0x7FFFFFFF;
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

    // // Configure UART
    // UCA0CTLW0 = UCSWRST;         // Reset UART
    // UCA0CTLW0 |= UCSSEL__SMCLK;  // SMCLK source (16MHz)
    // UCA0BR0 = 104;               // 16MHz/9600 = 1666.67
    // UCA0BR1 = 0;                 // High byte
    // UCA0MCTLW = UCOS16 | 0x4900; // Oversampling + fractional tuning
    //

    // Configure UART pins using the uart library
    uart_pins_t uart_pins;
    uart_pins = create_uart_pins(UART_PIN_TX, UART_PIN_RX);
    // uart_init(MSP430_UART0, 9600, &uart_pins);
    // UCA0CTLW0 &= ~UCSWRST; // Enable UART

    UCA0CTLW0 = UCSWRST;         // Reset UART
    UCA0CTLW0 |= UCSSEL__SMCLK;  // SMCLK source (16MHz)
    UCA0BR0 = 104;               // 16MHz/9600 = 1666.67
    UCA0BR1 = 0;                 // High byte
    UCA0MCTLW = UCOS16 | 0x4900; // Oversampling + fractional tuning
    UCA0CTLW0 &= ~UCSWRST;       // Enable UART


#if DEV_KIT == 1
    // Configure UART pins
    P2SEL0 &= ~(BIT0 | BIT1); // Clear P2.0/P2.1 SEL0
    P2SEL1 |= BIT0 | BIT1;    // Set UART function
#else
    P2SEL0 &= ~(BIT5 | BIT6); // Clear P2.5/P2.6 SEL0
    P2SEL1 |= BIT5 | BIT6;    // Set UART function
#endif

    // TX
    // *(uart_pins.tx_sel0) &= ~uart_pins.tx_mask;
    // *(uart_pins.tx_sel1) |= uart_pins.tx_mask;

    // // RX
    // *(uart_pins.rx_sel0) &= ~uart_pins.rx_mask;
    // *(uart_pins.rx_sel1) |= uart_pins.rx_mask;

    // Configure all timers with ID__8 (divide by 8)
    configure_timer((volatile uint16_t *)&TA1CTL, ID__8); // Configure Timer A1
    configure_timer((volatile uint16_t *)&TA2CTL, ID__8); // Configure Timer A2
    configure_timer((volatile uint16_t *)&TA4CTL, ID__8); // Configure Timer A4
    configure_timer((volatile uint16_t *)&TB0CTL, ID__8); // Configure Timer B0

    init_seeds(); // Initialize LFSR seeds

    __enable_interrupt(); // Enable global interrupts
}
