#include "handshake.h"


static volatile uint64_t initial_state_mask = 0;    // Mask to store the initial state of pins
static TimingPinData *global_timing_pindata = NULL; // Global pointer to TimingPinData array

static void on_syn_complete(TimingPinData *data)
{
    // SYN task completed, prepare for SYN_ACK
    set_syn(data, true);           // Set SYN flag
    set_initiator_syn(data, true); // Mark as initiator for SYN
    data->sending_counter = 0;     // Reset sending counter for next task
}
static void on_syn_ack_complete(TimingPinData *data)
{
    // SYN_ACK task completed, prepare for ACK
    set_syn_ack(data, true);          // Set SYN_ACK flag
    set_initiator_synack(data, true); // Mark as initiator for SYN_ACK
    data->sending_counter = 0;        // Reset sending counter for next task
}
static void on_ack_complete(TimingPinData *data)
{
    // ACK task completed, reset task to none
    set_ack(data, true);
    set_initiator_ack(data, true); // Mark as initiator for ACK
    data->sending_counter = 0;     // Reset sending counter
    if (is_successful(data))
    {
        data->successful_handshakes++; // Increment successful handshakes counter
        if (data->successful_handshakes >= 5)
        {
            initial_state_mask |= (1ULL << data->pin);
        }
    }
}

const TimingTaskDef task_lut[] = {
    [TASK_NONE] = {0, NULL},
    [TASK_JOB_SYN] = {SYN_CYCLES, &on_syn_complete},
    [TASK_JOB_SYN_ACK] = {SYN_ACK_CYCLES, &on_syn_ack_complete},
    [TASK_JOB_ACK] = {ACK_CYCLES, &on_ack_complete}};

static void reader_isr(void)
{
    gpio_drive_high(DEBUG_PIN1); // Set debug pin high to indicate ISR entry
    uint64_t all_pins_mask = (NUMBER_OF_GPIO_PINS >= 64)
                                 ? ~0ULL
                                 : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);

    BitmapIterator it = bitmap_iterator_create(~initial_state_mask & all_pins_mask);

    uint8_t pin;
    while (bitmap_iterator_next(&it, &pin))
    {
        TimingPinData *data = &global_timing_pindata[pin];

        bool something_done = false; // Flag to track if any task was done

        if (data->waiting_counter >= MAXIMUM_WAITING_CYCLES)
        {
            set_task(data, TASK_JOB_SYN);
            data->waiting_counter = 0; // Reset waiting counter
        }

        // Check if a task is conducted on this pin
        PinDataTask task = get_task(data);
        bool pin_state = gpio_read(pin);

        if (task != TASK_NONE)
        {
            something_done = true; // At least one task is being done
            data->waiting_counter = 0;
            //  Collision detection:
            //  Before sending, check if the pin is high and we are sending
            if (data->sending_counter == 0 && !pin_state)
            {
                set_task(data, TASK_NONE); // Reset task
            }
            else
            {
                uint16_t required_cycles = task_lut[task].cycles;
                if (data->sending_counter >= required_cycles)
                {
                    // Reset when task is complete
                    gpio_od_release(pin); // Release the pin
                    set_task(data, TASK_NONE);
                    task_lut[task].on_complete(data);
                }
                else
                {
                    gpio_od_hold_low(pin);   // Drive pin low to send signal
                    data->sending_counter++; // Increment cycle counter
                }
            }
        }
        else
        {
            if (!pin_state) // if a signal is received
            {
                something_done = true;     // At least one task is being done
                data->receiving_counter++; // Increment receiving counter
            }
        }
        if (pin_state) // if no signal is received, reset the receiving counter
        {
            uint16_t receive_counter = data->receiving_counter;
            if (receive_counter >= MIN_SYN_CYCLES && receive_counter <= MAXIMUM_SYN_CYCLES)
            {
                // SYN signal detected
                set_syn(data, true);
                set_initiator_syn(data, false); // Mark as responder for SYN
                set_task(data, TASK_JOB_SYN_ACK);
                data->receiving_counter = 0; // Reset receiving counter after SYN
                something_done = true;       // Mark that something was done
            }
            else if (receive_counter >= MIN_SYN_ACK_CYCLES && receive_counter <= MAXIMUM_SYN_ACK_CYCLES)
            {
                // SYN_ACK signal detected
                set_syn_ack(data, true);
                set_initiator_synack(data, false); // Mark as responder for SYN_ACK
                set_task(data, TASK_JOB_ACK);
                data->receiving_counter = 0; // Reset receiving counter after SYN_ACK
                something_done = true;       // Mark that something was done
            }
            else if (receive_counter >= MIN_ACK_CYCLES && receive_counter <= MAXIMUM_ACK_CYCLES)
            {
                // ACK signal detected
                set_ack(data, true);
                set_initiator_ack(data, false); // Mark as responder for ACK
                data->receiving_counter = 0;    // Reset receiving counter after ACK
                something_done = true;          // Mark that something was done
                if (is_successful(data))
                {
                    data->successful_handshakes++; // Increment successful handshakes counter
                    if (data->successful_handshakes >= 5)
                    {
                        initial_state_mask |= (1ULL << pin);
                    }
                }
            }
            else if (receive_counter > MAXIMUM_ACK_CYCLES)
            {
                // Signal too long, reset counters
                data->receiving_counter = 0;
            }
        }

        if (!something_done)
        {
            data->waiting_counter++; // Increment waiting counter if no task was done
        }
    }
    gpio_drive_low(DEBUG_PIN1); // Set debug pin low to indicate ISR exit
}

static void start_handshake_timer(void)
{
#if defined(NRF52840_XXAA)
    configure_timer(NRF_TIMER4, 4, TIMER_BITMODE_BITMODE_32Bit); // 1MHz (1µs per tick)
    set_timer_compare(NRF_TIMER4, 0, READER_INTERVAL_US, true, true);  // Set compare for 1ms intervals and enable interrupt
    // Set callback and start
    set_timer_event_callback(NRF_TIMER4, reader_isr);
    start_timer(NRF_TIMER4);

#elif defined(__MSP430FR5994__)
    // Configure Timer A1 for periodic ISR calls
    configure_timer(TIMER_A1, 8, MC__STOP); // SMCLK/8, continuous mode
    set_timer_compare(TIMER_A1, 0, READER_TICKS);

    // Set callback and start
    set_timer_compare_callback(TIMER_A1, reader_isr);
    start_timer_with_interrupt(TIMER_A1);
#endif
}

static void stop_handshake_timer(void)
{
#if defined(NRF52840_XXAA)
    clear_timer_event_callback(NRF_TIMER4);
    stop_timer(NRF_TIMER4);

#elif defined(__MSP430FR5994__)
    clear_timer_callbacks(TIMER_A1);
    stop_timer(TIMER_A1);
#endif
}

static inline uint64_t mask_all_pins(void)
{
    return (NUMBER_OF_GPIO_PINS >= 64) ? ~0ULL : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);
}

static inline uint64_t mask_valid_pins(uint64_t blacklist)
{
    return ~blacklist & mask_all_pins();
}

void perform_handshake(PinData *pin_data_array, const uint64_t initial_blacklist_mask)
{
    initial_state_mask = initial_blacklist_mask;

    uint64_t valid_pins_mask = mask_valid_pins(initial_blacklist_mask);
    uint8_t active_pins = __builtin_popcountll(valid_pins_mask);

    if (active_pins == 0) {
        return; // Nothing to process
    }

    // Allocate or reallocate global_timing_pindata
    if (global_timing_pindata) {
        free(global_timing_pindata);
        global_timing_pindata = NULL;
    }

    global_timing_pindata = calloc(active_pins, sizeof(TimingPinData));
    if (!global_timing_pindata) {
        return; // Allocation failed
    }

    // Run handshake period
    start_handshake_timer();
    delay_ms(DURATION_OF_HANDSHAKE_MS);
    stop_handshake_timer();

    // Copy data
    // BitmapIterator it = bitmap_iterator_create(valid_pins_mask);
    // uint8_t pin, idx = 0;
    // while (bitmap_iterator_next(&it, &pin)) {
    //     if (idx < active_pins) {
    //         pin_data_array[pin]. =
    //             global_timing_pindata[idx].successful_handshakes;
    //     }
    //     idx++;
    // }

    // Cleanup
    free(global_timing_pindata);
    global_timing_pindata = NULL;
}
