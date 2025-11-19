#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pindata.h"

// Special value for undefined strength
#define STRENGTH_UNDEFINED -127

/**
 * @brief Analyzes the events of a specific pin to determine its drive strength.
 * 
 * @param pindata Pointer to the PinData array
 * @param pin The index of the pin to analyze
 * @return int8_t The calculated strength (3 to -3), or STRENGTH_UNDEFINED
 */
int8_t strength_analyzer_get_strength(PinData *pindata, uint8_t pin);

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
bool strength_analyzer_should_blacklist(PinData *pindata, uint8_t pin);