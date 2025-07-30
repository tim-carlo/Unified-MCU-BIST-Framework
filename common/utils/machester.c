#include "manchester.h"
#include <string.h>

#define LONG_BIT_DURATION_US 250 // Long bit duration in microseconds
#define MID_BIT_DURATION_US LONG_BIT_DURATION_US / 2 // Mid bit duration in microseconds

#define T_TIME_US MID_BIT_DURATION_US  
#define T2_TIME_US LONG_BIT_DURATION_US
#define T_WINDOW_US (T_TIME_US / 2)    // ±50% window for timing tolerance

// Static variables to store pin configurations
static volatile uint8_t tx_pin_num = 0;
static volatile uint8_t rx_pin_num = 0;

// Manchester decode state variables
static volatile bool edge_detected = false;
static volatile uint32_t edge_timestamp = 0;
static volatile uint32_t last_edge_timestamp = 0;
static volatile bool decode_active = false;
static volatile bool current_bit_value = false;
static volatile bool synchronized = false;
static uint8_t *decode_buffer = NULL;
static volatile uint8_t decode_bit_count = 0;
static volatile uint8_t target_bit_count = 0;
static volatile bool decode_error = false;

// Helper function to check if timing is within acceptable window
static bool is_timing_within_window(uint32_t measured_time, uint32_t expected_time, uint32_t window)
{
    return (measured_time >= (expected_time - window)) && (measured_time <= (expected_time + window));
}

// GPIO interrupt handler for Manchester decoding
static void manchester_edge_handler(uint32_t pin)
{
    if (pin != rx_pin_num || !decode_active)
        return;
    
    // Get current timestamp
    edge_timestamp = get_timer_ticks(TIMER_A);
    edge_detected = true;
}

static void send_zero(void)
{
    // Set the output signal low
    gpio_open_drain_drive(tx_pin_num);
    delay_us(MID_BIT_DURATION_US);

    // Set the output signal high
    release_gpio_open_drain(tx_pin_num);
    delay_us(MID_BIT_DURATION_US);
}

static void send_one(void)
{
    // Set the output signal high
    release_gpio_open_drain(tx_pin_num);
    delay_us(MID_BIT_DURATION_US);
    
    // Set the output signal low
    gpio_open_drain_drive(tx_pin_num);
    delay_us(MID_BIT_DURATION_US);
}

void manchester_init(uint8_t tx_pin, uint8_t rx_pin)
{
    // Store pin numbers for use in static functions
    tx_pin_num = tx_pin;
    rx_pin_num = rx_pin;


    // Configure TX pin in open-drain mode for transmission
    gpio_open_drain(tx_pin);
    
    // Configure RX pin in open-drain mode for reception
    gpio_open_drain(rx_pin);
}

void manchester_transmit_byte(uint8_t byte)
{
    release_gpio_open_drain(tx_pin_num);
    
    // 2-6. Loop through all bits (MSB first)
    for (int i = 7; i >= 0; i--) 
    {
        if (byte & (1 << i))
        {
            send_one();
        }
        else
        {
            send_zero();
        }
    }
    release_gpio_open_drain(tx_pin_num);
}

void manchester_transmit_array(uint8_t length, uint8_t *data)
{
    for (uint8_t i = 0; i < length; i++)
    {
        manchester_transmit_byte(data[i]);
    }
}

bool manchester_receive_array(uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0)
        return false;
    
    decode_buffer = data;
    target_bit_count = length * 8; // Total bits to receive
    decode_bit_count = 0;
    decode_error = false;
    synchronized = false;
    decode_active = true;
    
    // Clear the buffer
    memset(data, 0, length);
    
    // 1. Set up timer to interrupt on every edge
    start_timer(TIMER_A);
    
    // Set up GPIO interrupt for RX pin (both rising and falling edges)
    uint64_t blacklist = ~(1ULL << rx_pin_num); // Allow only RX pin
    gpio_listen_on_all_pins_interrupt(blacklist, manchester_edge_handler, manchester_edge_handler);
    
    // 3. Start timer, capture first edge and discard this
    edge_detected = false;
    uint32_t timeout_start = get_timer_ticks(TIMER_A);
    const uint32_t TIMEOUT_US = 10000; // 10ms timeout
    
    // Wait for first edge (discard)
    while (!edge_detected && (get_timer_ticks(TIMER_A) - timeout_start) < TIMEOUT_US) {}
    if (!edge_detected) {
        decode_active = false;
        return false; // Timeout
    }
    
    edge_detected = false;
    last_edge_timestamp = edge_timestamp;
    
    // 4-5. Synchronization phase: wait for 2T period
    while (!synchronized && !decode_error) {
        // Wait for next edge
        while (!edge_detected && (get_timer_ticks(TIMER_A) - timeout_start) < TIMEOUT_US) {}
        if (!edge_detected) {
            decode_active = false;
            return false; // Timeout
        }
        
        uint32_t time_diff = timer_diff_us(last_edge_timestamp, edge_timestamp);
        
        // Check if this is a 2T period (synchronization)
        if (is_timing_within_window(time_diff, T2_TIME_US, T_WINDOW_US)) {
            synchronized = true;
            // 6. Read current logic level and save as current bit value
            current_bit_value = gpio_read(rx_pin_num);
        }
        
        last_edge_timestamp = edge_timestamp;
        edge_detected = false;
    }
    
    if (!synchronized) {
        decode_active = false;
        return false;
    }
    
    // Main decode loop
    while (decode_bit_count < target_bit_count && !decode_error) {
        // 7. Capture next edge
        edge_detected = false;
        timeout_start = get_timer_ticks(TIMER_A);
        
        while (!edge_detected && (get_timer_ticks(TIMER_A) - timeout_start) < TIMEOUT_US) {}
        if (!edge_detected) {
            decode_error = true;
            break; // Timeout
        }
        
        uint32_t time_diff = timer_diff_us(last_edge_timestamp, edge_timestamp);
        bool next_bit;
        
        // 7a-c. Compare stored count value with T and 2T
        if (is_timing_within_window(time_diff, T_TIME_US, T_WINDOW_US)) {
            // 7b. Value = T: need to capture next edge to confirm
            last_edge_timestamp = edge_timestamp;
            edge_detected = false;
            
            // Wait for next edge
            while (!edge_detected && (get_timer_ticks(TIMER_A) - timeout_start) < TIMEOUT_US) {}
            if (!edge_detected) {
                decode_error = true;
                break;
            }
            
            uint32_t second_time_diff = timer_diff_us(last_edge_timestamp, edge_timestamp);
            if (!is_timing_within_window(second_time_diff, T_TIME_US, T_WINDOW_US)) {
                decode_error = true; // Error: second edge not at T
                break;
            }
            
            // Next bit = current bit (no transition in middle)
            next_bit = current_bit_value;
        }
        else if (is_timing_within_window(time_diff, T2_TIME_US, T_WINDOW_US)) {
            // 7c. Value = 2T: transition in middle
            next_bit = !current_bit_value;
        }
        else {
            // 7d. Invalid timing
            decode_error = true;
            break;
        }
        
        // 8. Store next bit in buffer
        uint8_t byte_index = decode_bit_count / 8;
        uint8_t bit_index = 7 - (decode_bit_count % 8); // MSB first
        
        if (next_bit) {
            data[byte_index] |= (1 << bit_index);
        }
        
        decode_bit_count++;
        
        // 10. Set current bit to next bit for next iteration
        current_bit_value = next_bit;
        last_edge_timestamp = edge_timestamp;
    }
    
    decode_active = false;
    
    // 9. Check if desired number of bits are decoded
    return (decode_bit_count == target_bit_count) && !decode_error;
}