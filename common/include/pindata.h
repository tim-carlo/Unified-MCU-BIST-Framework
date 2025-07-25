#ifndef PINDATAHELPER_H
#define PINDATAHELPER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint32_t pin; // Pin number
    // The 'steps' field uses individual bits to represent various flags:
    // Bit 0: ACK signal
    // Bit 1: SYN-ACK signal
    // Bit 2: SYN signal
    // Bit 3: Role (0 = initiator, 1 = responder)
    // Bit 4: Blacklisted status (1 = blacklisted)
    // Bit 5: Success status (1 = successful)
    uint8_t steps;
    uint8_t num_false_responses; // Number of false responses received
    uint8_t num_tries;          // Number of tries made with this pin
    uint8_t error_reason;        // Error reason code
    uint32_t last_falling_edge; // Timestamp of the last falling edge
} PinData;

/**
 * @brief Reset the PinData structure to its initial state
 *
 * This function sets all fields of the PinData structure to their default values.
 *
 * @param data Pointer to the PinData structure to reset
 */
inline void reset_pin_data(PinData *data)
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
inline void reset_last_falling_edge(PinData *data)
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
inline void initialize_pin_data_array(PinData *array, uint32_t length)
{
    for (uint32_t i = 0; i < length; ++i) {
        array[i].pin = i;
        array[i].steps = 0;
        array[i].num_false_responses = 0;
        array[i].num_tries = 0;
        array[i].error_reason = 0;
        array[i].last_falling_edge = -1;
    }
}

/** 
 * @brief Set or clear the ACK signal flag (bit 0)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the flag, false to clear it
 *
*/
inline void set_ack(PinData *data, bool value)
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
inline void set_syn_ack(PinData *data, bool value)
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
inline void set_syn(PinData *data, bool value)
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
 * @param value True for responder, false for initiator
 */
inline void set_role(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 3); // Responder
    else
        data->steps &= ~(1 << 3); // Initiator
}

/**
 * @brief Set or clear the blacklisted flag (bit 4)
 *
 * @param data Pointer to the PinData structure
 * @param value True to set the pin as blacklisted, false to clear it
 */
inline void set_blacklisted(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 4);
    else
        data->steps &= ~(1 << 4);
}

/**
 * @brief Set or clear the successful flag (bit 5)
 *
 * @param data Pointer to the PinData structure
 * @param value True if the pin was successful, false otherwise
 */
inline void set_successful(PinData *data, bool value)
{
    if (value)
        data->steps |= (1 << 5);
    else
        data->steps &= ~(1 << 5);
}

/**
 * @brief Check if the ACK signal flag (bit 0) is set
 * 
 * @param data Pointer to the PinData structure
 * @return true 
 * @return false 
 */
inline bool is_ack(PinData *data)
{
    return (data->steps & (1 << 0)) != 0;
}

/**
 * @brief Check if the SYN-ACK signal flag (bit 1) is set
 *
 * @param data Pointer to the PinData structure
 * @return true 
 * @return false 
 */
inline bool is_syn_ack(PinData *data)
{
    return (data->steps & (1 << 1)) != 0;
}

/**
 * @brief Check if the SYN signal flag (bit 2) is set
 *
 * @param data Pointer to the PinData structure
 * @return true 
 * @return false 
 */
inline bool is_syn(PinData *data)
{
    return (data->steps & (1 << 2)) != 0;
}

/**
 * @brief Check if the role is responder (bit 3)
 *
 * @param data Pointer to the PinData structure
 * @return true if the role is responder, false if initiator
 */
inline bool is_role_responder(PinData *data)
{
    return (data->steps & (1 << 3)) != 0;
}

/**
 * @brief Check if the pin is blacklisted (bit 4)
 *
 * @param data Pointer to the PinData structure
 * @return true if the pin is blacklisted, false otherwise
 */
inline bool is_blacklisted(PinData *data)
{
    return (data->steps & (1 << 4)) != 0;
}

/**
 * @brief Check if the pin was successful (bit 5)
 *
 * @param data Pointer to the PinData structure
 * @return true if the pin was successful, false otherwise
 */
inline bool is_successful(PinData *data)
{
    return (data->steps & (1 << 5)) != 0;
}

/**
 * @brief Set blacklisted status in a mask
 *
 * @param data Pointer to the PinData structure
 * @param pin Pin number (0-47)
 */
inline void set_blacklisted_in_mask(uint64_t *mask, uint32_t pin)
{
    if (pin < 64) {
        *mask |= (1ULL << pin);
    }
}

#endif // PINDATAHELPER_H