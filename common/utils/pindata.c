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
 * @brief Debug function to analyze the entire pin data array with statistics
 *
 * @param array Pointer to the array of PinData structures
 * @param length Number of elements in the array
 */
void debug_pin_data_array_analysis(PinData *array, uint32_t length)
{
    uint32_t active = 0;
    uint32_t blacklisted = 0;
    uint32_t success = 0;
    uint32_t syn = 0, syn_ack = 0, ack = 0;
    uint32_t handshakes = 0;
    uint32_t initiators = 0, responders = 0;
    uint32_t errors = 0;

    for (uint32_t i = 0; i < length; i++) {
        PinData *p = &array[i];

        if (p->steps || p->num_tries > 0 || p->error_reason != ERROR_REASON_NONE)
            active++;

        if (is_blacklisted(p)) blacklisted++;
        if (is_successful(p))  success++;
        if (is_syn(p))         syn++;
        if (is_syn_ack(p))     syn_ack++;
        if (is_ack(p))         ack++;

        if (is_syn(p) && is_syn_ack(p) && is_ack(p))
            handshakes++;

        if (is_role_responder(p))
            responders++;
        else if (p->steps)
            initiators++;
    }

    printf("Active pins:     %u / %u\n", active, length);
    printf("Blacklisted:     %u\n", blacklisted);
    printf("Successful:      %u\n", success);
    printf("Handshakes:      %u\n", handshakes);
    printf("SYN:             %u\n", syn);
    printf("SYN-ACK:         %u\n", syn_ack);
    printf("ACK:             %u\n", ack);
    printf("Initiators:      %u\n", initiators);
    printf("Responders:      %u\n", responders);
    printf("Errors:          %u\n", errors);

    for (uint32_t i = 0; i < length; i++) {
        PinData *p = &array[i];

        // show only active pins with some data
        if (!(p->steps || p->num_tries > 0 || p->error_reason != ERROR_REASON_NONE))
            continue;

        printf("Pin %2u: ", p->pin);

        if (is_blacklisted(p)) printf("[BL] ");
        if (is_successful(p))  printf("[OK] ");

        if (is_syn(p))     printf("S");
        if (is_syn_ack(p)) printf("A");
        if (is_ack(p))     printf("K");
        printf(") ");

        printf("%s ", is_role_responder(p) ? "RESP" : "INIT");
        printf("Tries=%u ", p->num_tries);
        printf("ErrCnt=%u ", p->num_false_responses);

        if (p->error_reason != ERROR_REASON_NONE)
            printf("Err=%u ", p->error_reason);

        if (p->last_falling_edge != INVALID_TIMESTAMP)
            printf("LastEdge=%lu", (unsigned long)p->last_falling_edge);

        printf("\n");
    }
}
