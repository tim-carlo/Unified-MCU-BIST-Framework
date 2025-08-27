#include <stdint.h>

// 16-bit swap
static inline uint16_t htobe16(uint16_t x) {
    return __builtin_bswap16(x);
}

// 32-bit swap
static inline uint32_t htobe32(uint32_t x) {
    return __builtin_bswap32(x);
}

// 64-bit swap
static inline uint64_t htobe64(uint64_t x) {
    return __builtin_bswap64(x);
}