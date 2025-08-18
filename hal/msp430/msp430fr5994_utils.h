#ifndef MSP430FR5994_UTILS_H
#define MSP430FR5994_UTILS_H

#include <stdint.h>
#include <msp430.h>
#include <msp430fr5994.h>

#ifdef __cplusplus
extern "C" {
#endif

uint64_t get_unique_id(void);
const char *get_unique_id_str(void);
uint32_t random32_lfsr(void);
uint32_t random32(void);
const char *get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_UTILS_H