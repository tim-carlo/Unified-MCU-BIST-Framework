#ifndef BYTESWAP_H
#define BYTESWAP_H

#include <stdint.h>
#include <msp430.h>  // für __swap_bytes()

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* MSP430-optimierte Byte-Swaps mit __swap_bytes() */

/* 16-Bit Swap: direkt __swap_bytes() verwenden */
#define __bswap_16(x) (__extension__ \
  ({ \
    uint16_t __v = (x); \
    __swap_bytes(__v); \
  }))

/* 32-Bit Swap: zwei 16-Bit Halbwörter swapen + tauschen */
#define __bswap_32(x) (__extension__ \
  ({ \
    uint32_t __x = (x); \
    uint16_t __hi = __swap_bytes((uint16_t)(__x >> 16)); \
    uint16_t __lo = __swap_bytes((uint16_t)(__x & 0xFFFF)); \
    ((__lo << 16) | __hi); \
  }))

/* 64-Bit Swap: vier 16-Bit Halbwörter swapen + Reihenfolge umdrehen */
#define __bswap_64(x) (__extension__ \
  ({ \
    uint64_t __x = (x); \
    uint16_t w0 = __swap_bytes((uint16_t)(__x >> 48)); \
    uint16_t w1 = __swap_bytes((uint16_t)(__x >> 32)); \
    uint16_t w2 = __swap_bytes((uint16_t)(__x >> 16)); \
    uint16_t w3 = __swap_bytes((uint16_t)(__x & 0xFFFF)); \
    ((uint64_t)w3 << 48) | ((uint64_t)w2 << 32) | ((uint64_t)w1 << 16) | w0; \
  }))

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // BYTESWAP_H