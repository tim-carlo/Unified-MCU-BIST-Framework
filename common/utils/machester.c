/*
This code is based on the Atmel Corporation Manchester
Coding Basics Application Note.

http://www.atmel.com/dyn/resources/prod_documents/doc9164.pdf

Quotes from the application note:

"Manchester coding states that there will always be a transition of the message signal
at the mid-point of the data bit frame.
What occurs at the bit edges depends on the state of the previous bit frame and
does not always produce a transition. A logical '1' is defined as a mid-point transition
from low to high and a '0' is a mid-point transition from high to low.

We use Timing Based Manchester Decode.
In this approach we will capture the time between each transition coming from the demodulation
circuit."

Timer 2 is used with a ATMega328. Timer 1 is used for a ATtiny85.

This code gives a basic data rate as 1200 bauds. In manchester encoding we send 1 0 for a data bit 0.
We send 0 1 for a data bit 1. This ensures an average over time of a fixed DC level in the TX/RX.
This is required by the ASK RF link system to ensure its correct operation.
The data rate is then 600 bits/s.
*/

#include "manchester.h"
#include "printf.h"

#define BAUD_FROM_SPEEDFACTOR(sf) (BASE_BAUD_RATE << (sf))
#define LONG_BIT_US(sf) (1000000UL / BAUD_FROM_SPEEDFACTOR(sf)) // Full bit time
#define MID_BIT_US(sf) (LONG_BIT_US(sf) / 2)                    // Half bit time

#if defined(NRF52840_XXAA)
#include "nrf52840.h"
#include "nrf52840_helper.h"
#elif defined(__MSP430FR5994__)
#include <msp430fr5994.h>
#include "msp430fr5994_helper.h"
#endif

static int8_t RxPin = 255;
static int8_t TxPin = 255;
uint8_t applyWorkAround1Mhz = 0;
uint8_t speedFactor = MAN_1200;
uint16_t delay1 = 0;
uint16_t delay2 = 0;

volatile static int16_t rx_sample = 0;
volatile static int16_t rx_last_sample = 0;
volatile static uint8_t rx_count = 0;
volatile static uint8_t rx_sync_count = 0;
volatile static uint8_t rx_mode = RX_MODE_IDLE;

volatile uint16_t rx_manBits = 0;
volatile uint8_t rx_numMB = 0;
volatile uint8_t rx_curByte = 0;
volatile uint8_t rx_maxBytes = 2;
volatile uint8_t rx_default_data[2];
volatile uint8_t *rx_data = rx_default_data;

void sendZero(void)
{
    // Manchester 0: first half high, second half low (transition from high to low)
    release_gpio_open_drain(TxPin); // Set high (release to pull-up)
    delay_us(delay1);
    gpio_open_drain_drive(TxPin); // Drive low
    delay_us(delay2);
}

void sendOne(void)
{
    // Manchester 1: first half low, second half high (transition from low to high)
    gpio_open_drain_drive(TxPin); // Drive low
    delay_us(delay1);
    release_gpio_open_drain(TxPin); // Set high (release to pull-up)
    delay_us(delay2);
}

void manchester_init(uint8_t txPin, uint8_t rxPin, uint8_t sF)
{
    TxPin = txPin;
    RxPin = rxPin;

    // Configure both pins as open-drain
    gpio_open_drain(txPin); // Set Tx pin as open-drain output
    gpio_open_drain(rxPin); // Set Rx pin as open-drain input

    // Initialize variables
    rx_sample = 0;
    rx_last_sample = 0;
    rx_count = 0;
    rx_sync_count = 0;
    rx_mode = RX_MODE_IDLE;

    rx_manBits = 0;
    rx_numMB = 0;
    rx_curByte = 0;

    rx_maxBytes = 2;
    rx_data = rx_default_data;

    speedFactor = sF;
    // we don't use exact calculation of passed time spent outside of transmitter
    // because of high ovehead associated with it, instead we use this
    // emprirically determined values to compensate for the time loss

    // Calculate delays based on speedFactor
    // HALF_BIT_INTERVAL is for speed factor 0 (300 baud)
    // Each speed factor doubles the rate, so we right-shift the interval

    // uint16_t baseInterval = HALF_BIT_INTERVAL >> speedFactor;
    // uint16_t halfBit = MID_BIT_US(speedFactor);
    uint32_t baudrate_tbl[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400};
    uint32_t baud = baudrate_tbl[speedFactor];

    // µs-Werte berechnen
    uint32_t fullBit_us = 1000000UL / baud;
    uint32_t halfBit_us = fullBit_us / 2;

#if defined(NRF52840_XXAA)
    // 64 MHz core
    // delay1 = delay2 = baseInterval;
    delay1 = (uint16_t)(halfBit_us);
    delay2 = (uint16_t)(halfBit_us);

#elif defined(__MSP430FR5994__)
    // 16 MHz core
    uint16_t compensationFactor = 4;
    delay1 = (uint16_t)(halfBit_us - compensationFactor);
    delay2 = (uint16_t)(halfBit_us - 2);
#endif

    printf("Manchester initialized with TxPin: %d, RxPin: %d, SpeedFactor: %d\n", (int)TxPin, (int)RxPin, (int)speedFactor);
    printf("Delay1: %d, Delay2: %d\n", (int)delay1, (int)delay2);
}

/*
The 433.92 Mhz receivers have AGC, if no signal is present the gain will be set
to its highest level.

In this condition it will switch high to low at random intervals due to input noise.
A CRO connected to the data line looks like 433.92 is full of transmissions.

Any ASK transmission method must first sent a capture signal of 101010........
When the receiver has adjusted its AGC to the required level for the transmisssion
the actual data transmission can occur.

We send 14 0's 1010... It takes 1 to 3 10's for the receiver to adjust to
the transmit level.

The receiver waits until we have at least 10 10's and then a start pulse 01.
The receiver is then operating correctly and we have locked onto the transmission.
*/
void manchester_transmitArray(uint8_t numBytes, uint8_t *data)
{

#if SYNC_BIT_VALUE
    for (int8_t i = 0; i < SYNC_PULSE_DEF; i++) // send capture pulses
    {
        sendOne(); // end of capture pulses
    }
    sendZero(); // start data pulse
#else
    for (int8_t i = 0; i < SYNC_PULSE_DEF; i++) // send capture pulses
    {
        sendZero(); // end of capture pulses
    }
    sendOne(); // start data pulse
#endif

    // Send the user data
    for (uint8_t i = 0; i < numBytes; i++)
    {
        uint16_t mask = 0x01; // mask to send bits
        uint8_t d = data[i] ^ DECOUPLING_MASK;
        for (uint8_t j = 0; j < 8; j++)
        {
            if ((d & mask) == 0)
                sendZero();
            else
                sendOne();
            mask <<= 1; // get next bit
        } // end of byte
    } // end of data

    // Send 3 terminatings 0's to correctly terminate the previous bit and to turn the transmitter off
#if SYNC_BIT_VALUE
    sendOne();
    sendOne();
    sendOne();
#else
    sendZero();
    sendZero();
    sendZero();
#endif
} // end of send the data

// TODO use repairing codes perhabs?
// http://en.wikipedia.org/wiki/Hamming_code

/*
    format of the message including checksum and ID

    [0][1][2][3][4][5][6][7][8][9][a][b][c][d][e][f]
    [    ID    ][ checksum ][         data         ]
                  checksum = ID xor data[7:4] xor data[3:0] xor 0b0011

*/

// decode 8 bit payload and 4 bit ID from the message, return true if checksum is correct, otherwise false
uint8_t manchester_decodeMessage(uint16_t m, uint8_t *id, uint8_t *data)
{
    *data = (m & 0xFF);
    *id = (m >> 12);
    uint8_t ch = (m >> 8) & 0b1111;
    uint8_t ech = (*id ^ *data ^ (*data >> 4) ^ 0b0011) & 0b1111;
    return ch == ech;
}

// encode 8 bit payload, 4 bit ID and 4 bit checksum into 16 bit
uint16_t manchester_encodeMessage(uint8_t id, uint8_t data)
{
    uint8_t chsum = (id ^ data ^ (data >> 4) ^ 0b0011) & 0b1111;
    return ((id) << 12) | (chsum << 8) | (data);
}

void manchester_beginReceiveArray(uint8_t maxBytes, uint8_t *data)
{
    MANRX_SetupReceive(speedFactor); // Initialize timer
// Platform-specific timer start
#if defined(NRF52840_XXAA)
    NRF_TIMER3->TASKS_START = 1;
#elif defined(__MSP430FR5994__)
    start_timer(TIMER_A1); // Start Timer A1
#endif
    MANRX_BeginReceive();
    MANRX_BeginReceiveBytes(maxBytes, data);
}

void manchester_beginReceive(void)
{
    MANRX_BeginReceive();
}

uint8_t manchester_receiveComplete(void)
{
    return MANRX_ReceiveComplete();
}

uint8_t manchester_getMessage(void)
{
    return MANRX_GetMessage();
}

void manchester_stopReceive(void)
{
    MANRX_StopReceive();
}


void MANRX_SetupReceive(uint8_t speedFactor)
{
#if defined(NRF52840_XXAA)
    // Beispiel: Samplingrate = 8x pro Bit, Bitdauer aus speedFactor ableiten
    // Hier: 2T = Bitdauer, SAMPLES_PER_BIT = 8
    uint32_t sample_interval_us = (512 >> speedFactor); // 512 us for speed factor 0 (300 baud)

    // Set up the receive pin
    gpio_open_drain(RxPin);

    // Configure timer 3 for Manchester RX
    NVIC_DisableIRQ(TIMER3_IRQn);
    NRF_TIMER3->TASKS_STOP = 1;
    NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER3->PRESCALER = 4; // 1 MHz
    NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
    NRF_TIMER3->TASKS_CLEAR = 1;
    NRF_TIMER3->CC[0] = sample_interval_us;
    NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NVIC_ClearPendingIRQ(TIMER3_IRQn);
    NVIC_EnableIRQ(TIMER3_IRQn);

    printf("NRF52840 RX Timer: %lu µs interval\n", sample_interval_us);

    // Set P0.11 as output (for debugging/toggling in ISR)
    NRF_P0->DIRSET = (1 << 11);
#elif defined(__MSP430FR5994__)
    // Use same sample interval as transmission delays for consistency
    uint32_t baudrate_tbl[] = {300, 600, 1200, 2400, 4800, 9600, 19200, 38400};
    uint32_t baud = baudrate_tbl[speedFactor];
    uint32_t bit_time_us = 1000000UL / baud;
    uint32_t sample_interval_us = bit_time_us / 6; // 6 samples per bit (like Arduino)

    // Set up the receive pin
    gpio_open_drain(RxPin);

    // Configure Timer A1 for Manchester RX
    stop_timer(TIMER_A1);
    volatile uint16_t *ctl = GET_TxxCTL(TIMER_A1);
    volatile uint16_t *ccr0 = GET_TxxCCR0(TIMER_A1);

    // Calculate timer value for sample interval
    // SMCLK = 16MHz, we want sample_interval_us microseconds
    uint32_t timer_counts = (sample_interval_us * SMCLK_HZ) / 1000000UL;
    
    // Use appropriate divider to fit in 16-bit CCR0
    uint16_t divider = 1;
    uint16_t div_bits = 0; // ID__1
    
    if (timer_counts > 65535) {
        divider = 2;
        div_bits = ID__1; // /2
        timer_counts /= 2;
    }
    if (timer_counts > 65535) {
        divider = 4;
        div_bits = ID__2; // /4
        timer_counts /= 2;
    }
    if (timer_counts > 65535) {
        divider = 8;
        div_bits = ID__3; // /8
        timer_counts /= 2;
    }

    *ctl = TASSEL__SMCLK | MC__UP | TACLR | div_bits; // SMCLK, Up Mode, Clear
    *ccr0 = (uint16_t)timer_counts;

    // Enable CCR0 interrupt
    volatile uint16_t *cctl0 = GET_TxxCCTL0(TIMER_A1);
    *cctl0 |= CCIE; // Enable CCR0 interrupt

    printf("MSP430 RX Timer: interval=%lu µs, counts=%lu, divider=%u\n", 
           sample_interval_us, timer_counts, divider);
#endif
}

void MANRX_BeginReceive(void)
{
    rx_maxBytes = 2;
    rx_data = rx_default_data;
    rx_mode = RX_MODE_PRE;
}

void MANRX_BeginReceiveBytes(uint8_t maxBytes, uint8_t *data)
{
    rx_maxBytes = maxBytes;
    rx_data = data;
    rx_mode = RX_MODE_PRE;
}

void MANRX_StopReceive(void)
{
    rx_mode = RX_MODE_IDLE;
#if defined(NRF52840_XXAA)
    NRF_TIMER3->TASKS_STOP = 1; // Stop the timer
#elif defined(__MSP430FR5994__)
    stop_timer(TIMER_A1); // Stop Timer A1
#endif
}

uint8_t MANRX_ReceiveComplete(void)
{
    return (rx_mode == RX_MODE_MSG);
}

uint8_t MANRX_GetMessage(void)
{
    return (((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1];
}

static void AddManBit(volatile uint16_t *manBits, volatile uint8_t *numMB,
                      volatile uint8_t *curByte, volatile uint8_t *data,
                      uint8_t bit)
{
    *manBits <<= 1;
    *manBits |= bit;
    (*numMB)++;
    if (*numMB == 16)
    {
        uint8_t newData = 0;
        for (int8_t i = 0; i < 8; i++)
        {
            // ManBits holds 16 bits of manchester data
            // 1 = LO,HI
            // 0 = HI,LO
            // We can decode each bit by looking at the bottom bit of each pair.
            newData <<= 1;
            newData |= (*manBits & 1); // store the one
            *manBits = *manBits >> 2;  // get next data bit
        }
        data[*curByte] = newData ^ DECOUPLING_MASK;
        (*curByte)++;

        // added by caoxp @ https://github.com/caoxp
        // compatible with unfixed-length data, with the data length defined by the first byte.
        // at a maximum of 255 total data length.
        if ((*curByte) == 1)
        {
            rx_maxBytes = data[0];
        }

        *numMB = 0;
    }
}

void MANRX_ISR(void)
{
#if defined(NRF52840_XXAA)
    // Toggle pin P0.11 on NRF52840
    NRF_P0->OUT ^= (1 << 11);
#endif

    if (rx_mode < RX_MODE_MSG) // receiving something
    {
        // Increment counter
        rx_count += 8;

        // Check for value change
        // rx_sample = digitalRead(RxPin);
        // caoxp@github,
        // add filter.
        // sample twice, only the same means a change.
        static uint8_t rx_sample_0 = 0;
        static uint8_t rx_sample_1 = 0;
        rx_sample_1 = gpio_read(RxPin);
        if (rx_sample_1 == rx_sample_0)
        {
            rx_sample = rx_sample_1;
        }
        rx_sample_0 = rx_sample_1;

        // check sample transition
        uint8_t transition = (rx_sample != rx_last_sample);

        if (rx_mode == RX_MODE_PRE)
        {
            // Wait for first transition to HIGH
            if (transition && (rx_sample == 1))
            {
                rx_count = 0;
                rx_sync_count = 0;
                rx_mode = RX_MODE_SYNC;
            }
        }
        else if (rx_mode == RX_MODE_SYNC)
        {
            // Initial sync block
            if (transition)
            {
                if (((rx_sync_count < (SYNC_PULSE_MIN * 2)) || (rx_last_sample == 1)) &&
                    ((rx_count < MinCount) || (rx_count > MaxCount)))
                {
                    // First 20 bits and all 1 bits are expected to be regular
                    // Transition was too slow/fast
                    rx_mode = RX_MODE_PRE;
                }
                else if ((rx_last_sample == 0) &&
                         ((rx_count < MinCount) || (rx_count > MaxLongCount)))
                {
                    // 0 bits after the 20th bit are allowed to be a double bit
                    // Transition was too slow/fast
                    rx_mode = RX_MODE_PRE;
                }
                else
                {
                    rx_sync_count++;

                    if ((rx_last_sample == 0) &&
                        (rx_sync_count >= (SYNC_PULSE_MIN * 2)) &&
                        (rx_count >= MinLongCount))
                    {
                        // We have seen at least 10 regular transitions
                        // Lock sequence ends with unencoded bits 01
                        // This is encoded and TX as HI,LO,LO,HI
                        // We have seen a long low - we are now locked!
                        rx_mode = RX_MODE_DATA;
                        rx_manBits = 0;
                        rx_numMB = 0;
                        rx_curByte = 0;
                    }
                    else if (rx_sync_count >= (SYNC_PULSE_MAX * 2))
                    {
                        rx_mode = RX_MODE_PRE;
                    }
                    rx_count = 0;
                }
            }
        }
        else if (rx_mode == RX_MODE_DATA)
        {
            // Receive data
            if (transition)
            {
                if ((rx_count < MinCount) ||
                    (rx_count > MaxLongCount))
                {
                    // wrong signal lenght, discard the message
                    rx_mode = RX_MODE_PRE;
                    printf("Manchester RX: Invalid signal length %d\n", rx_count);
                }
                else
                {
                    if (rx_count >= MinLongCount) // was the previous bit a double bit?
                    {
                        AddManBit(&rx_manBits, &rx_numMB, &rx_curByte, rx_data, rx_last_sample);
                    }
                    if ((rx_sample == 1) &&
                        (rx_curByte >= rx_maxBytes))
                    {
                        rx_mode = RX_MODE_MSG;
                    }
                    else
                    {
                        // Add the current bit
                        AddManBit(&rx_manBits, &rx_numMB, &rx_curByte, rx_data, rx_sample);
                        rx_count = 0;
                    }
                }
            }
        }

        // Get ready for next loop
        rx_last_sample = rx_sample;
    }
}

#if defined(NRF52840_XXAA)
void TIMER3_IRQHandler(void)
{
    if (NRF_TIMER3->EVENTS_COMPARE[0])
    {
        NRF_TIMER3->EVENTS_COMPARE[0] = 0;
        MANRX_ISR();
    }
}
#elif defined(__MSP430FR5994__)
void __attribute__((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR(void)
{
    // CCR0 interrupt - automatically cleared
    MANRX_ISR();
}
#endif