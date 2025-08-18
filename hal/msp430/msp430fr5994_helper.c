#include "msp430fr5994_helper.h"


static void configure_timer(volatile uint16_t *timer_ctl, uint16_t divider_setting)
{
    *timer_ctl = TASSEL__SMCLK     // Use SMCLK as clock source (16 MHz)
                 | divider_setting // Variable divider setting
                 | MC__STOP        // Timer stopped initially
                 | TACLR           // Clear the timer
                 | TAIE;           // Enable overflow interrupt
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

    // Configure all timers with ID__8 (divide by 8)
    configure_timer((volatile uint16_t *)&TA1CTL, ID__8); // Configure Timer A1
    configure_timer((volatile uint16_t *)&TA2CTL, ID__8); // Configure Timer A2
    configure_timer((volatile uint16_t *)&TA4CTL, ID__8); // Configure Timer A4
    configure_timer((volatile uint16_t *)&TB0CTL, ID__8); // Configure Timer B0

    __enable_interrupt(); // Enable global interrupts
}
