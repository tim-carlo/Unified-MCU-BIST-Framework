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

static inline void set_neutral_gpio()
{
    /* set all to neutral state: input */
    PJOUT = 0u;
    PJDIR = 0x00;
    PJSEL0 = 0u;
    PJSEL1 = 0u;

    P1OUT = 0u;
    P1DIR = 0x00;
    P1SEL0 = 0u;
    P1SEL1 = 0u;

    P2OUT = 0u;
    P2DIR = 0x00;
    P2SEL0 = 0u;
    P2SEL1 = 0u;

    P3OUT = 0u;
    P3DIR = 0x00;
    P3SEL0 = 0u;
    P3SEL1 = 0u;

    P4OUT = 0u;
    P4DIR = 0x00;
    P4SEL0 = 0u;
    P4SEL1 = 0u;

    P5OUT = 0u;
    P5DIR = 0x00;
    P5SEL0 = 0u;
    P5SEL1 = 0u;

    P6OUT = 0u;
    P6DIR = 0x00;
    P6SEL0 = 0u;
    P6SEL1 = 0u;

    P7OUT = 0u;
    P7DIR = 0x00;
    P7SEL0 = 0u;
    P7SEL1 = 0u;

    P8OUT = 0u;
    P8DIR = 0x00;
    P8SEL0 = 0u;
    P8SEL1 = 0u;
}

/**
 * @brief Initialize the I/O subsystem
 *
 * This function initializes the GPIO, UART, and Timer modules.
 */
void mcu_init()
{
    // Disable watchdog
    WDTCTL = WDTPW | WDTHOLD;

    // Unlock GPIO
    PM5CTL0 &= ~LOCKLPM5;
    // Set all GPIOs to neutral state
    //set_neutral_gpio();

    // Configure clock to 16MHz
    // Using DCO at 16MHz
    // We need the maximum speed for the handshakes
    FRCTL0 = FRCTLPW | NWAITS_1;
    CSCTL0_H = CSKEY_H;
    CSCTL1 = DCOFSEL_4 | DCORSEL;
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;

    // Configure all timers with ID__8 (divide by 8)
    // configure_timer((volatile uint16_t *)&TA1CTL, ID__8); // Configure Timer A1
    // configure_timer((volatile uint16_t *)&TA2CTL, ID__8); // Configure Timer A2
    // configure_timer((volatile uint16_t *)&TA4CTL, ID__8); // Configure Timer A4
    // configure_timer((volatile uint16_t *)&TB0CTL, ID__8); // Configure Timer B0

    init_seeds(); // Initialize LFSR seeds

    __enable_interrupt(); // Enable global interrupts
}
