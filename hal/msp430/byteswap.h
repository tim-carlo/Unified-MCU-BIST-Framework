#ifndef BYTESWAP_H
#define BYTESWAP_H

#include <stdint.h>
#include <msp430.h>  // für __swap_bytes()

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define __bswap_16(x) (__extension__ \
  ({ \
    uint16_t __v = (x); \
    __swap_bytes(__v); \
  }))

#define __bswap_32(x) (__extension__                   \
  ({                                                   \
    uint32_t __x = (uint32_t)(x);                      \
    uint16_t __hi = (uint16_t)(__x >> 16);             \
    uint16_t __lo = (uint16_t)(__x & 0xFFFF);          \
    uint32_t __new_hi = (uint32_t)__swap_bytes(__lo);  \
    uint32_t __new_lo = (uint32_t)__swap_bytes(__hi);  \
    ((__new_hi << 16) | __new_lo);                     \
  }))

#define __bswap_64(x) (__extension__                                   \
  ({                                                                   \
    uint64_t __x = (uint64_t)(x);                                      \
    uint16_t w0 = (uint16_t)(__x >> 48);                               \
    uint16_t w1 = (uint16_t)(__x >> 32);                               \
    uint16_t w2 = (uint16_t)(__x >> 16);                               \
    uint16_t w3 = (uint16_t)(__x & 0xFFFF);                            \
    uint64_t nw0 = (uint64_t)__swap_bytes(w3);                         \
    uint64_t nw1 = (uint64_t)__swap_bytes(w2);                         \
    uint64_t nw2 = (uint64_t)__swap_bytes(w1);                         \
    uint64_t nw3 = (uint64_t)__swap_bytes(w0);                         \
    ((nw0 << 48) | (nw1 << 32) | (nw2 << 16) | nw3);                   \
  }))

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // BYTESWAP_H