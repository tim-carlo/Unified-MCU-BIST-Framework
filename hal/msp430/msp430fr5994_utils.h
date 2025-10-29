#ifndef MSP430FR5994_UTILS_H
#define MSP430FR5994_UTILS_H

#include <stdint.h>
#include <msp430.h>
#include <msp430fr5994.h>
#include "printf.h"
#include "msp430fr5994_helper.h"
#define POLYMASK_32 0xb4bcd35c
#define POLYMASK_31 0x7a5bc2e3


#ifdef __cplusplus
extern "C" {
#endif

uint64_t get_unique_id(void);
const uint8_t *get_unique_id_str(void);
uint32_t random32_lfsr(void);
uint32_t random32(void);
const uint8_t *get_chip_family_name(void);

#ifdef __cplusplus
}
#endif

#endif // MSP430FR5994_UTILS_H