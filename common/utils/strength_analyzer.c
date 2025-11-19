#include "strength_analyzer.h"
#include <stddef.h>

/**
 * @brief Helper to check if a specific step matches the expected state.
 * 
 * @param pindata Pointer to PinData array
 * @param pin Pin index
 * @param high_event Event enum for HIGH state
 * @param low_event Event enum for LOW state
 * @param expected_state 1 for HIGH, 0 for LOW
 * @return true if the actual event matches the expected state
 */
static bool check_step(PinData *pindata, uint8_t pin, PinEventType high_event, PinEventType low_event, uint8_t expected_state)
{
    if (expected_state == 1)
    {
        return check_if_pinevent_exists(pindata, pin, high_event);
    }
    else
    {
        return check_if_pinevent_exists(pindata, pin, low_event);
    }
}

/**
 * @brief Analyzes the events of a specific pin to determine its drive strength.
 * 
 * @param pindata Pointer to the PinData array
 * @param pin The index of the pin to analyze
 * @return int8_t The calculated strength (3 to -3), or STRENGTH_UNDEFINED
 */
int8_t strength_analyzer_get_strength(PinData *pindata, uint8_t pin)
{
    if (pindata == NULL)
    {
        return STRENGTH_UNDEFINED;
    }

    // Definition of the 6 checks in order: 1A, 1B, 2A, 2B, 3A, 3B
    // Stored in flash (const)
    const struct {
        PinEventType high;
        PinEventType low;
    } steps[6] = {
        {STEP_1_A_HIGH, STEP_1_A_LOW},
        {STEP_1_B_HIGH, STEP_1_B_LOW},
        {STEP_2_A_HIGH, STEP_2_A_LOW},
        {STEP_2_B_HIGH, STEP_2_B_LOW},
        {STEP_3_A_HIGH, STEP_3_A_LOW},
        {STEP_3_B_HIGH, STEP_3_B_LOW}
    };

    // Patterns definition
    // 1=HIGH, 0=LOW
    // Order: 1A, 1B, 2A, 2B, 3A, 3B
    const struct {
        int8_t strength;
        uint8_t bits[6];
    } patterns[] = {
        // Positive Forces (High-Side)
        { 3, {1, 1, 1, 1, 1, 1}},
        { 2, {1, 1, 1, 1, 0, 1}},
        { 1, {1, 1, 0, 1, 0, 1}},
        { 0, {0, 1, 0, 1, 0, 1}},
        // Negative Forces (Low-Side)
        {-1, {0, 0, 0, 1, 0, 1}},
        {-2, {0, 0, 0, 0, 0, 1}},
        {-3, {0, 0, 0, 0, 0, 0}},
    };

    const uint8_t num_patterns = sizeof(patterns) / sizeof(patterns[0]);

    // Iterate through all defined patterns
    for (uint8_t i = 0; i < num_patterns; i++)
    {
        bool match = true;

        // Check all 6 bits for the current pattern
        for (uint8_t step = 0; step < 6; step++)
        {
            if (!check_step(pindata, pin, steps[step].high, steps[step].low, patterns[i].bits[step]))
            {
                match = false;
                break;
            }
        }

        if (match)
        {
            return patterns[i].strength;
        }
    }

    return STRENGTH_UNDEFINED;
}

/**
 * @brief Checks if a pin should be blacklisted based on its strength.
 * 
 * Returns true if strength is 1 or -1.
 * 
 * @param pindata Pointer to the PinData array
 * @param pin The index of the pin to analyze
 * @return true If pin should be blacklisted
 * @return false Otherwise
 */
bool strength_analyzer_should_blacklist(PinData *pindata, uint8_t pin)
{
    if (pindata == NULL)
    {
        return false;
    }

    int8_t strength = strength_analyzer_get_strength(pindata, pin);

    // Blacklist if strength is 1 or -1
    if (strength == 1 || strength == -1)
    {
        return true;
    }

    return false;
}