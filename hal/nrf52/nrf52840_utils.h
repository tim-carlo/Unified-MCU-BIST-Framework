#ifndef NRF52840_UTILS_H
#define NRF52840_UTILS_H

#include <stdint.h>
#include "nrf.h"
#include "nrf52840.h"

#ifdef __cplusplus
extern "C" {
#endif

uint32_t random32_lfsr(void);
uint32_t random32(void);
uint64_t get_unique_id(void);
const uint8_t* get_unique_id_str(void);
const uint8_t* get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // NRF52840_UTILS_H