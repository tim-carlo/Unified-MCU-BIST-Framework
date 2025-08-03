#include "manchester.h"
//#include "nrf52840_helper.h"
#include <string.h>

// The clock speed on the MSP is to inacurate to go higher than 2000 baud
#define BAUD_RATE_MANCHESTER 100
#define LONG_BIT_US (1000000UL / BAUD_RATE_MANCHESTER) // 500 us
#define MID_BIT_US (LONG_BIT_US / 2)        // 250 us
#define T_TIME_US MID_BIT_US                // 250 us
#define T2_TIME_US LONG_BIT_US              // 500 us

#define T_WINDOW_US (T_TIME_US / 2) // ±50 % Toleranz (125 us)
#define FRAME_TIMEOUT_US (64UL * LONG_BIT_US * 3UL)

#define PREAMBLE_LENGTH 1
#define PREAMBLE_BYTE 0xAA // Preamble byte for Manchester encoding
#define CHECKSUM_LENGTH 1 // 1 byte checksum

// Static variables to store pin configurations
static volatile uint8_t tx_pin_num = 0;
static volatile uint8_t rx_pin_num = 0;

// Manchester decode state variables
static volatile bool edge_detected = false;
static volatile bool last_edge_was_rising = false;
static volatile uint32_t edge_timestamp = 0;
static volatile uint32_t last_edge_timestamp = 0;
static volatile bool decode_active = false;
static volatile bool current_bit_value = false;
static volatile bool synchronized = false;
static uint8_t *decode_buffer = NULL;
static volatile uint16_t decode_bit_count = 0;
static volatile uint16_t target_bit_count = 0;
static volatile bool decode_error = false;

// Helper function to check if timing is within acceptable window
static bool is_timing_within_window(uint32_t measured_time, uint32_t expected_time, uint32_t window)
{
    return (measured_time >= (expected_time - window)) && (measured_time <= (expected_time + window));
}

// Use existing get_elapsed_time function for time difference calculation
static uint32_t get_time_diff(uint32_t start, uint32_t end)
{
    return get_elapsed_time(start, end);
}

// Rising edge handler
static void manchester_rising_handler(uint32_t pin)
{
    if (pin != rx_pin_num || !decode_active)
        return;
    edge_detected = true;
    last_edge_was_rising = true;
    edge_timestamp = get_timer_ticks(TIMER_A);
}

// Falling edge handler
static void manchester_falling_handler(uint32_t pin)
{
    if (pin != rx_pin_num || !decode_active)
        return;
    edge_detected = true;
    last_edge_was_rising = false;
    edge_timestamp = get_timer_ticks(TIMER_A);
}

static uint8_t calculate_checksum(uint8_t *data, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

static void mid_bit_duration(void)
{
    delay_us(MID_BIT_US);
}

static void send_zero(void)
{
    gpio_open_drain_drive(tx_pin_num);
    mid_bit_duration();
    release_gpio_open_drain(tx_pin_num);
    mid_bit_duration();
}

static void send_one(void)
{
    release_gpio_open_drain(tx_pin_num);
    mid_bit_duration();
    gpio_open_drain_drive(tx_pin_num);
    mid_bit_duration();
}

void manchester_init(uint8_t tx_pin, uint8_t rx_pin)
{
    tx_pin_num = tx_pin;
    rx_pin_num = rx_pin;
    release_gpio_open_drain(tx_pin);
}

void manchester_transmit_byte(uint8_t byte)
{
    release_gpio_open_drain(tx_pin_num);
    for (int i = 7; i >= 0; i--)
    {
        if (byte & (1 << i))
            send_one();
        else
            send_zero();
    }
    release_gpio_open_drain(tx_pin_num);
}

void manchester_transmit_array(uint8_t length, const uint8_t *data)
{
    if (data == NULL || length == 0)
        return;

    uint8_t total_length = PREAMBLE_LENGTH + length + CHECKSUM_LENGTH;
    uint8_t transmit_buffer[PREAMBLE_LENGTH + 255 + CHECKSUM_LENGTH]; // Max 255 bytes payload

    // Set preamble
    transmit_buffer[0] = PREAMBLE_BYTE;
    memcpy(&transmit_buffer[PREAMBLE_LENGTH], data, length);

    // Calculate and append checksum
    transmit_buffer[PREAMBLE_LENGTH + length] = calculate_checksum(&transmit_buffer[PREAMBLE_LENGTH], length);

    // Transmit all bytes
    for (uint8_t i = 0; i < total_length; i++)
    {
        manchester_transmit_byte(transmit_buffer[i]);
    }
}

bool manchester_receive_array(uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0)
        return false;

    printf("Starting Manchester decode for %d bytes\n", length);
    printf("Timing params: T=%luus, 2T=%luus, Window=%luus\n",
           T_TIME_US, T2_TIME_US, T_WINDOW_US);

    decode_buffer = data;
    target_bit_count = length * 8;
    decode_bit_count = 0;
    decode_error = false;
    synchronized = false;
    decode_active = true;

    memset(data, 0, length);
    start_timer(TIMER_A);

    uint64_t blacklist = ~(1ULL << rx_pin_num);
    gpio_listen_on_all_pins_interrupt(blacklist, manchester_rising_handler, manchester_falling_handler);

    // 3. Start timer, capture first edge and discard this
    edge_detected = false;
    uint32_t timeout_start = get_timer_ticks(TIMER_A);
    const uint32_t TIMEOUT_US = 1000000; // 1 second timeout

    // Wait for first edge (discard)
    while (!edge_detected)
    {
        if (get_elapsed_time(timeout_start, get_timer_ticks(TIMER_A)) > TIMEOUT_US)
        {
            printf("Timeout waiting for first edge\n");
            decode_active = false;
            return false;
        }
    }
    edge_detected = false;
    last_edge_timestamp = edge_timestamp;

    // 4-5. Synchronization phase: wait for 2T period
    while (!synchronized && !decode_error)
    {
        timeout_start = get_timer_ticks(TIMER_A);
        while (!edge_detected)
        {
            if (get_elapsed_time(timeout_start, get_timer_ticks(TIMER_A)) > TIMEOUT_US)
            {
                printf("Timeout waiting for sync edge\n");
                decode_active = false;
                return false;
            }
        }

        uint32_t time_diff = get_elapsed_time(last_edge_timestamp, edge_timestamp);
        //    printf("Sync diff: %luus (expected ~%luus)\n", time_diff, T2_TIME_US);

        // Check if this is a 2T period (synchronization)
        if (is_timing_within_window(time_diff, T2_TIME_US, T_WINDOW_US))
        {
            synchronized = true;
            // 6. Read current logic level and save as current bit value
            current_bit_value = gpio_read(rx_pin_num);
            //  printf("Synchronized! Current level: %d\n", current_bit_value);
        }

        last_edge_timestamp = edge_timestamp;
        edge_detected = false;
    }

    if (!synchronized)
    {
        printf("Synchronization failed\n");
        decode_active = false;
        return false;
    }

    // Main decode loop
    while (decode_bit_count < target_bit_count && !decode_error)
    {
        edge_detected = false;
        timeout_start = get_timer_ticks(TIMER_A);

        while (!edge_detected)
        {
            if (get_elapsed_time(timeout_start, get_timer_ticks(TIMER_A)) > T2_TIME_US * 3)
            {
                printf("Timeout waiting for data edge\n");
                decode_error = true;
                break;
            }
        }
        if (decode_error)
            break;

        uint32_t time_diff = get_elapsed_time(last_edge_timestamp, edge_timestamp);
        bool next_bit;
        bool valid_edge = false;

        // printf("Edge diff: %luus (T=%lu, 2T=%lu)\n", time_diff, T_TIME_US, T2_TIME_US);

        // 7a-c. Compare stored count value with T and 2T
        if (is_timing_within_window(time_diff, T_TIME_US, T_WINDOW_US))
        {
            // 7b. Value = T: need to capture next edge to confirm
            last_edge_timestamp = edge_timestamp;
            edge_detected = false;
            timeout_start = get_timer_ticks(TIMER_A);

            while (!edge_detected)
            {
                if (get_elapsed_time(timeout_start, get_timer_ticks(TIMER_A)) > T_TIME_US * 3)
                {
                    printf("Timeout waiting for confirm edge\n");
                    decode_error = true;
                    break;
                }
            }
            if (decode_error)
                break;

            uint32_t second_time_diff = get_elapsed_time(last_edge_timestamp, edge_timestamp);
            // printf("Confirm diff: %luus (expected ~%luus)\n", second_time_diff, T_TIME_US);

            if (is_timing_within_window(second_time_diff, T_TIME_US, T_WINDOW_US))
            {
                next_bit = current_bit_value;
                valid_edge = true;
            }
            else
            {
                printf("Invalid confirm timing\n");
                decode_error = true;
            }
        }
        else if (is_timing_within_window(time_diff, T2_TIME_US, T_WINDOW_US))
        {
            // 7c. Value = 2T: transition in middle
            next_bit = !current_bit_value;
            valid_edge = true;
        }
        else
        {
            printf("Invalid timing\n");
            decode_error = true;
        }

        if (valid_edge && !decode_error)
        {
            // 8. Store next bit in buffer
            uint8_t byte_index = decode_bit_count / 8;
            uint8_t bit_index = 7 - (decode_bit_count % 8);

            if (next_bit)
            {
                data[byte_index] |= (1 << bit_index);
            }

            decode_bit_count++;
            //  printf("Decoded bit %d: %d\n", decode_bit_count, next_bit);

            // 10. Set current bit to next bit for next iteration
            current_bit_value = next_bit;
            last_edge_timestamp = edge_timestamp;
        }

        edge_detected = false;
    }

    decode_active = false;

    bool success = (decode_bit_count == target_bit_count) && !decode_error;
    printf("Decode %s, bits: %d/%d\n", success ? "success" : "fail", decode_bit_count, target_bit_count);
    return success;
}