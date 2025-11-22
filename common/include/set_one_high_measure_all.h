#ifndef SET_ONE_HIGH_MEASURE_ALL_H
#define SET_ONE_HIGH_MEASURE_ALL_H

#include <stdint.h>
#include <stdlib.h>
#include "pin_config.h"
#include "pindata.h"

#if defined(__MSP430FR5994__)
#include "msp430fr5994_gpio.h"
#include "msp430fr5994_time.h"
#elif defined(NRF52840_XXAA)
#include "nrf52840_gpio.h"
#include "nrf52840_time.h"
#endif

#include "strength_analyzer.h"



void step_1(uint64_t blacklist_mask, PinData *pindata);
void step_2(uint64_t blacklist_mask, PinData *pindata);
void step_3(uint64_t blacklist_mask, PinData *pindata);
void phase_0_one_set_pulldown(uint64_t blacklist_mask, PinData *pindata);
void phase_1_one_set_pullup(uint64_t blacklist_mask, PinData *pindata);
void phase_2_drive_low(uint64_t blacklist_mask, PinData *pindata);
void phase_3_drive_high(uint64_t blacklist_mask, PinData *pindata);
void phase_4_pullup_all_drive_low(uint64_t blacklist_mask, PinData *pindata);
void phase_5_pulldown_all_drive_high(uint64_t blacklist_mask, PinData *pindata);



/**
 * @brief Main function to run all 4 phases of set-one-high-measure-all pin connectivity testing 
 * This function implements a comprehensive pin connectivity test using 4 different
 * electrical configurations to detect connections between pins:

 * 
 * @param blacklist_mask 64-bit mask where 1 = test this pin, 0 = skip this pin
 * @param pindata Array to store discovered connections and events
 * @param pindata_size Size of the pindata array
 */
void run_selfexploration_tests(uint64_t blacklist_mask, PinData *pindata);



#endif // SET_ONE_HIGH_MEASURE_ALL_H
