#include "PinData.h"
#include "printf.h"



/**
 * @brief Reset the PinData structure to its initial state
 *
 * This function sets all fields of the PinData structure to their default values.
 *
 * @param data Pointer to the PinData structure to reset
 */
void reset_pin_data(PinData *data)
{
    data->pin = 0;
    data->steps = 0;
    data->num_false_responses = 0;
    data->error_reason = 0;
    data->num_tries = 0;
    data->last_falling_edge = 0;
}

/**
 * @brief Reset the last falling edge timestamp in the PinData structure
 *
 * @param data Pointer to the PinData structure
 */
void reset_last_falling_edge(PinData *data)
{
    data->last_falling_edge = 0;
}

/**
 * @brief Initialize an array of PinData structures
 *
 * This function initializes each PinData structure in the array with default values.
 *
 * @param array Pointer to the array of PinData structures
 * @param length Number of elements in the array
 */
void initialize_pin_data_array(PinData *array, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i)
    {
        array[i].pin = (uint8_t)i;
        array[i].steps = 0;
        array[i].num_false_responses = 0;
        array[i].num_tries = 0;
        array[i].error_reason = 0;
        array[i].last_falling_edge = INVALID_TIMESTAMP;
    }
}

/**
 * @brief Print the contents of a PinData array for debugging
 *
 * @param array Pointer to the array of PinData structures
 * @param length Number of elements in the array
 */
void print_pin_data_array(PinData *array, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i)
    {
        printf("PinData[%u]: pin=%u, steps=0x%X, num_false_responses=%u, num_tries=%u, error_reason=%u, last_falling_edge=%lld\n",
               i,
               array[i].pin,
               array[i].steps,
               array[i].num_false_responses,
               array[i].num_tries,
               array[i].error_reason,
               (long long)array[i].last_falling_edge);
    }
}

/**
 * @brief Set or clear the ACK signal flag (bit 0)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag, false to clear it
 *
 */
void set_ack(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 0);
    else
        data->steps &= ~(1 << 0);
}

/**
 * @brief Set or clear the SYN-ACK signal flag (bit 1)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag, false to clear it
 */
void set_syn_ack(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 1);
    else
        data->steps &= ~(1 << 1);
}

/**
 * @brief Set or clear the SYN signal flag (bit 2)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag, false to clear it
 */
void set_syn(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 2);
    else
        data->steps &= ~(1 << 2);
}

/**
 * @brief Set or clear the role flag (bit 3)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag (responder), false to clear it (initiator)
 */
void set_role(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 3);
    else
        data->steps &= ~(1 << 3);
}

/**
 * @brief Set or clear the blacklisted status flag (bit 4)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag (blacklisted), false to clear it
 */
void set_blacklisted(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 4);
    else
        data->steps &= ~(1 << 4);
}

/**
 * @brief Set or clear the successful status flag (bit 5)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag (successful), false to clear it
 */
void set_successful(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 5);
    else
        data->steps &= ~(1 << 5);
}

/**
 * @brief Check if the ACK signal flag is set (bit 0)
 *
 * @param data Pointer to the PinData structure
 * @return true if ACK signal is set, false otherwise
 */
bool is_ack(PinData *data)
{
    return (data->steps & (1 << 0)) != 0;
}

/**
 * @brief Check if the SYN-ACK signal flag is set (bit 1)
 *
 * @param data Pointer to the PinData structure
 * @return true if SYN-ACK signal is set, false otherwise
 */
bool is_syn_ack(PinData *data)
{
    return (data->steps & (1 << 1)) != 0;
}

/**
 * @brief Check if the SYN signal flag is set (bit 2)
 *
 * @param data Pointer to the PinData structure
 * @return true if SYN signal is set, false otherwise
 */
bool is_syn(PinData *data)
{
    return (data->steps & (1 << 2)) != 0;
}

/**
 * @brief Check if the role is responder (bit 3)
 *
 * @param data Pointer to the PinData structure
 * @return true if role is responder, false if initiator
 */
bool is_role_responder(PinData *data)
{
    return (data->steps & (1 << 3)) != 0;
}

/**
 * @brief Check if the pin is blacklisted (bit 4)
 *
 * @param data Pointer to the PinData structure
 * @return true if pin is blacklisted, false otherwise
 */
bool is_blacklisted(PinData *data)
{
    return (data->steps & (1 << 4)) != 0;
}

/**
 * @brief Check if the pin is successful (bit 5)
 *
 * @param data Pointer to the PinData structure
 * @return true if pin is successful, false otherwise
 */
bool is_successful(PinData *data)
{
    return (data->steps & (1 << 5)) != 0;
}

/**
 * @brief Set the blacklisted status of a pin in a bitmask
 *
 * This function sets the corresponding bit in the blacklist mask to indicate that the pin is blacklisted.
 *
 * @param mask Pointer to the blacklist mask
 * @param pin Pin number to be blacklisted (0-63)
 */
void set_blacklisted_in_mask(volatile uint64_t *mask, uint32_t pin)
{
    if (pin < 64)
    {
        *mask |= (1ULL << pin);
    }
}

/**
 * @brief Debug function to decode and print all step flags
 *
 * @param data Pointer to the PinData structure
 */
void debug_pin_steps(PinData *data)
{
    printf("=== PIN %u STEPS DEBUG ===\n", data->pin);
    printf("Raw steps value: 0x%02X (binary: ", data->steps);
    
    // Binary representation
    for (int i = 7; i >= 0; i--) {
        printf("%d", (data->steps >> i) & 1);
    }
    printf(")\n");
    
    // Individual flags
    printf("Step flags breakdown:\n");
    printf("  Bit 0 - ACK signal:      %s\n", is_ack(data) ? "SET" : "CLEAR");
    printf("  Bit 1 - SYN-ACK signal: %s\n", is_syn_ack(data) ? "SET" : "CLEAR");
    printf("  Bit 2 - SYN signal:     %s\n", is_syn(data) ? "SET" : "CLEAR");
    printf("  Bit 3 - Role:           %s\n", is_role_responder(data) ? "RESPONDER" : "INITIATOR");
    printf("  Bit 4 - Blacklisted:    %s\n", is_blacklisted(data) ? "YES" : "NO");
    printf("  Bit 5 - Successful:     %s\n", is_successful(data) ? "YES" : "NO");
    printf("  Bit 6 - Reserved:       %s\n", (data->steps & (1 << 6)) ? "SET" : "CLEAR");
    printf("  Bit 7 - Reserved:       %s\n", (data->steps & (1 << 7)) ? "SET" : "CLEAR");
    
    // Signal sequence analysis
    printf("Signal sequence: ");
    if (is_syn(data) && is_syn_ack(data) && is_ack(data)) {
        printf("COMPLETE (SYN -> SYN-ACK -> ACK)\n");
    } else if (is_syn(data) && is_syn_ack(data)) {
        printf("PARTIAL (SYN -> SYN-ACK, waiting for ACK)\n");
    } else if (is_syn(data)) {
        printf("INITIATED (SYN sent, waiting for SYN-ACK)\n");
    } else {
        printf("NONE or INVALID\n");
    }
    
    printf("========================\n");
}

/**
 * @brief Debug function to print complete PinData with detailed analysis
 *
 * @param data Pointer to the PinData structure
 */
void debug_pin_data_complete(PinData *data)
{
    printf("\n=== COMPLETE PIN DATA DEBUG ===\n");
    printf("Pin Number: %u\n", data->pin);
    printf("Error Reason: %u (", data->error_reason);
    
    switch (data->error_reason) {
        case ERROR_REASON_NONE:
            printf("NONE");
            break;
        case ERROR_REASON_TIMEOUT:
            printf("TIMEOUT");
            break;
        case ERROR_REASON_BLACKLISTED:
            printf("BLACKLISTED");
            break;
        case ERROR_REASON_DISTURBED:
            printf("DISTURBED");
            break;
        case ERROR_REASON_TRIES_EXCEEDED:
            printf("TRIES_EXCEEDED");
            break;
        default:
            printf("UNKNOWN");
            break;
    }
    printf(")\n");
    
    printf("Number of tries: %u\n", data->num_tries);
    printf("False responses: %u\n", data->num_false_responses);
    printf("Last falling edge: %lu (", (unsigned long)data->last_falling_edge);
    
    if (data->last_falling_edge == INVALID_TIMESTAMP) {
        printf("INVALID");
    } else {
        printf("valid timestamp");
    }
    printf(")\n");
    
    debug_pin_steps(data);
    printf("==============================\n\n");
}

/**
 * @brief Debug function to analyze the entire pin data array with statistics
 *
 * @param array Pointer to the array of PinData structures
 * @param length Number of elements in the array
 */
void debug_pin_data_array_analysis(PinData *array, uint32_t length)
{
    printf("\n=== PIN DATA ARRAY ANALYSIS ===\n");
    printf("Total pins: %u\n", length);
    
    // Statistics
    uint32_t active_pins = 0;
    uint32_t blacklisted_pins = 0;
    uint32_t successful_pins = 0;
    uint32_t syn_pins = 0;
    uint32_t syn_ack_pins = 0;
    uint32_t ack_pins = 0;
    uint32_t complete_handshakes = 0;
    uint32_t initiator_pins = 0;
    uint32_t responder_pins = 0;
    uint32_t error_pins = 0;
    
    for (uint32_t i = 0; i < length; i++) {
        PinData *pin = &array[i];
        
        if (pin->steps != 0 || pin->num_tries > 0 || pin->error_reason != ERROR_REASON_NONE) {
            active_pins++;
        }
        
        if (is_blacklisted(pin)) blacklisted_pins++;
        if (is_successful(pin)) successful_pins++;
        if (is_syn(pin)) syn_pins++;
        if (is_syn_ack(pin)) syn_ack_pins++;
        if (is_ack(pin)) ack_pins++;
        if (is_syn(pin) && is_syn_ack(pin) && is_ack(pin)) complete_handshakes++;
        if (is_role_responder(pin)) responder_pins++;
        else if (pin->steps != 0) initiator_pins++;
        if (pin->error_reason != ERROR_REASON_NONE) error_pins++;
    }
    
    printf("\n--- STATISTICS ---\n");
    printf("Active pins:         %u / %u (%.1f%%)\n", active_pins, length, 
           (float)active_pins * 100.0f / length);
    printf("Blacklisted pins:    %u (%.1f%%)\n", blacklisted_pins, 
           (float)blacklisted_pins * 100.0f / length);
    printf("Successful pins:     %u (%.1f%%)\n", successful_pins, 
           (float)successful_pins * 100.0f / length);
    printf("Complete handshakes: %u\n", complete_handshakes);
    printf("SYN signals:         %u\n", syn_pins);
    printf("SYN-ACK signals:     %u\n", syn_ack_pins);
    printf("ACK signals:         %u\n", ack_pins);
    printf("Initiator pins:      %u\n", initiator_pins);
    printf("Responder pins:      %u\n", responder_pins);
    printf("Pins with errors:    %u\n", error_pins);
    
    printf("\n--- DETAILED PIN STATUS ---\n");
    for (uint32_t i = 0; i < length; i++) {
        PinData *pin = &array[i];
        
        // Nur aktive Pins anzeigen
        if (pin->steps != 0 || pin->num_tries > 0 || pin->error_reason != ERROR_REASON_NONE) {
            printf("Pin %2u: ", pin->pin);
            
            if (is_blacklisted(pin)) printf("[BLACKLISTED] ");
            if (is_successful(pin)) printf("[SUCCESS] ");
            
            printf("Steps=0x%02X ", pin->steps);
            printf("(");
            if (is_syn(pin)) printf("S");
            if (is_syn_ack(pin)) printf("A");
            if (is_ack(pin)) printf("K");
            printf(") ");
            
            printf("%s ", is_role_responder(pin) ? "RESP" : "INIT");
            printf("Tries=%u ", pin->num_tries);
            printf("Errors=%u ", pin->num_false_responses);
            
            if (pin->error_reason != ERROR_REASON_NONE) {
                printf("ERR=%u ", pin->error_reason);
            }
            
            if (pin->last_falling_edge != INVALID_TIMESTAMP) {
                printf("LastEdge=%lu", (unsigned long)pin->last_falling_edge);
            }
            
            printf("\n");
        }
    }
    printf("===============================\n\n");
}

/**
 * @brief Real-time monitor function for pin data changes
 *
 * @param array Pointer to the array of PinData structures
 * @param length Number of elements in the array
 * @param pin_index Specific pin to monitor (or -1 for all active pins)
 */
void monitor_pin_data_changes(PinData *array, uint32_t length, int32_t pin_index)
{
    static PinData previous_state[64];  // Assuming max 64 pins
    static bool first_call = true;
    
    if (first_call) {
        // Initialize previous state
        for (uint32_t i = 0; i < length && i < 64; i++) {
            previous_state[i] = array[i];
        }
        first_call = false;
        printf("=== PIN DATA MONITOR STARTED ===\n");
        return;
    }
    
    printf("\n=== PIN DATA CHANGES ===\n");
    bool changes_detected = false;
    
    for (uint32_t i = 0; i < length && i < 64; i++) {
        if (pin_index >= 0 && (uint32_t)pin_index != i) continue;
        
        PinData *current = &array[i];
        PinData *previous = &previous_state[i];
        
        if (current->steps != previous->steps ||
            current->num_tries != previous->num_tries ||
            current->num_false_responses != previous->num_false_responses ||
            current->error_reason != previous->error_reason ||
            current->last_falling_edge != previous->last_falling_edge) {
            
            changes_detected = true;
            printf("Pin %u changed:\n", i);
            
            if (current->steps != previous->steps) {
                printf("  Steps: 0x%02X -> 0x%02X\n", previous->steps, current->steps);
                printf("    Old flags: ");
                if (previous->steps & (1<<0)) printf("ACK ");
                if (previous->steps & (1<<1)) printf("SYN-ACK ");
                if (previous->steps & (1<<2)) printf("SYN ");
                if (previous->steps & (1<<3)) printf("RESP ");
                if (previous->steps & (1<<4)) printf("BLACK ");
                if (previous->steps & (1<<5)) printf("SUCCESS ");
                printf("\n");
                printf("    New flags: ");
                if (current->steps & (1<<0)) printf("ACK ");
                if (current->steps & (1<<1)) printf("SYN-ACK ");
                if (current->steps & (1<<2)) printf("SYN ");
                if (current->steps & (1<<3)) printf("RESP ");
                if (current->steps & (1<<4)) printf("BLACK ");
                if (current->steps & (1<<5)) printf("SUCCESS ");
                printf("\n");
            }
            
            if (current->num_tries != previous->num_tries) {
                printf("  Tries: %u -> %u\n", previous->num_tries, current->num_tries);
            }
            
            if (current->num_false_responses != previous->num_false_responses) {
                printf("  False responses: %u -> %u\n", previous->num_false_responses, current->num_false_responses);
            }
            
            if (current->error_reason != previous->error_reason) {
                printf("  Error reason: %u -> %u\n", previous->error_reason, current->error_reason);
            }
            
            if (current->last_falling_edge != previous->last_falling_edge) {
                printf("  Last falling edge: %lu -> %lu\n", 
                       (unsigned long)previous->last_falling_edge, 
                       (unsigned long)current->last_falling_edge);
            }
            
            // Update previous state
            previous_state[i] = *current;
        }
    }
    
    if (!changes_detected) {
        printf("No changes detected.\n");
    }
    printf("=======================\n\n");
}


