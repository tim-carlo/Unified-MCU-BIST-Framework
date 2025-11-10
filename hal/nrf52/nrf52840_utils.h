#ifndef NRF52840_UTILS_H
#define NRF52840_UTILS_H

#include <stdint.h>
#include "nrf.h"
#include "nrf52840.h"


uint32_t random32_lfsr(void);
uint32_t random32(void);
uint64_t get_unique_id(void);
const uint8_t* get_unique_id_str(void);
const char* get_chip_family_name(void);


#endif // NRF52840_UTILS_H