#include "handshake.h"

static volatile uint64_t initial_state_mask = 0;    // Mask to store the initial state of pins
static TimingPinData *global_timing_pindata = NULL; // Global pointer to TimingPinData array
static volatile uint8_t number_of_active_pins = 0;    // Number of active pins participating in handshake

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
    for(uint8_t pin_index = 0; pin_index < number_of_active_pins; pin_index++)
    {
        TimingPinData *data = &global_timing_pindata[pin_index];
        const uint8_t physical_pin = data->pin;

        bool something_done = false; // Flag to track if any task was done

        if (data->waiting_counter >= MAXIMUM_WAITING_CYCLES)
        {
            data->status = 0;               // Reset status flags
            set_task(data, TASK_JOB_SYN);
            data->waiting_counter = 0; // Reset waiting counter
        }

        // Check if a task is conducted on this pin
        PinDataTask task = get_task(data);
        bool pin_state = gpio_read(physical_pin);

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
                if (data->sending_counter >= task_lut[task].cycles)
                {
                    // Reset when task is complete
                    gpio_od_release(physical_pin); // Release the pin
                    set_task(data, TASK_NONE);
                    task_lut[task].on_complete(data);
                }
                else
                {
                    gpio_od_hold_low(physical_pin); // Drive pin low to send signal
                    data->sending_counter++;        // Increment cycle counter
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
    configure_timer(NRF_TIMER4, 4, TIMER_BITMODE_BITMODE_32Bit);      // 1MHz (1µs per tick)
    set_timer_compare(NRF_TIMER4, 0, READER_INTERVAL_US, true, true); // Set compare for 1ms intervals and enable interrupt
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
    clear_timer_event_callback(TIMER_A1);
    stop_timer(TIMER_A1);
#endif
}

void perform_handshake(PinData *pin_data_array, const uint64_t initial_blacklist_mask)
{
    // Create bitmap iterator for non-blacklisted pins
    uint64_t all_pins_mask = (NUMBER_OF_GPIO_PINS >= 64)
                                 ? ~0ULL
                                 : ((1ULL << NUMBER_OF_GPIO_PINS) - 1);

    BitmapIterator it = bitmap_iterator_create(~initial_blacklist_mask & all_pins_mask);

    uint64_t valid_pins_mask = ~initial_blacklist_mask & all_pins_mask;
    printf("Valid pins mask: 0x%016llX\n", valid_pins_mask);

    number_of_active_pins = __builtin_popcountll(valid_pins_mask);
    printf("Number of active pins: %u\n", number_of_active_pins);
    if (number_of_active_pins == 0)
    {
        return; // Nothing to process
    }

    // Allocate or reallocate global TimingPinData array
    free(global_timing_pindata);
    global_timing_pindata = calloc(number_of_active_pins, sizeof(TimingPinData));
    if (!global_timing_pindata)
    {
        return; // Allocation failed
    }

    // Initialize TimingPinData for each valid pin
    it = bitmap_iterator_create(valid_pins_mask);
    uint8_t pin_index, idx = 0;
    while (bitmap_iterator_next(&it, &pin_index))
    {
        uint8_t physical_pin = pin_data_array[pin_index].pin;
        global_timing_pindata[idx++] = (TimingPinData){
            .pin = physical_pin,
            .status = 0,
            .current_job = TASK_JOB_SYN,
            .number_of_unsuccessful_syns = 1, // Start mit 1 → erlaubt initial SYN
        };
    }

    // Debug: Print global_timing_pindata initialization
    printf("Initialized TimingPinData array:\n");
    for (uint8_t i = 0; i < number_of_active_pins; ++i)
    {
        printf("  [%u] pin=%u, status=0x%02X, current_job=%d, unsuccessful_syns=%u\n",
               i,
               global_timing_pindata[i].pin,
               global_timing_pindata[i].status,
               global_timing_pindata[i].current_job,
               global_timing_pindata[i].number_of_unsuccessful_syns);
    }

    // start handshake process
    start_handshake_timer();
    delay_ms(DURATION_OF_HANDSHAKE_MS);
    stop_handshake_timer();


    // Copy results back to PinData and add events
    it = bitmap_iterator_create(valid_pins_mask);
    idx = 0;
    while (bitmap_iterator_next(&it, &pin_index))
    {
        TimingPinData *timing_data = &global_timing_pindata[idx++];

        // Print binary status of timing_data
        printf("  timing_data[%u] status (binary): 0b", idx - 1);
        for (int bit = 7; bit >= 0; --bit) {
            printf("%d", (timing_data->status >> bit) & 1);
        }
        printf("\n");

        // The physical pin number is also the index in pin_data_array
        uint8_t physical_pin = timing_data->pin;

        if (timing_data->successful_handshakes > 0)
        {
            RoleType role = get_role(timing_data);
            switch (role)
            {
            case ROLE_INITIATOR:
                add_pin_event(pin_data_array, physical_pin, HANDSHAKE_OK_INITIATOR);
                break;

            case ROLE_RESPONDER:
                add_pin_event(pin_data_array, physical_pin, HANDSHAKE_OK_RESPONDER);
                break;

            case ROLE_UNCLEAR:
                add_pin_event(pin_data_array, physical_pin, HANDSHAKE_FAILURE);
                break;

            default:
                add_pin_event(pin_data_array, physical_pin, HANDSHAKE_FAILURE);
                break;
            }
        }
        else
        {
            add_pin_event(pin_data_array, physical_pin, HANDSHAKE_FAILURE);
        }
    }

    // Cleanup
    free(global_timing_pindata);
    global_timing_pindata = NULL;
}