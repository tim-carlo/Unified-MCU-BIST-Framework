#include "msp430fr5994_utils.h"
/**
 * @brief Get the unique device ID from TLV memory
 *
 * @return uint32_t Unique device ID
 */
uint64_t get_unique_id(void)
{
    // Not available on MSP430 devices.
    // Maybe return the random number.
    return 0;
}

/**
 * @brief Get the unique device ID as a string
 *
 * @return const char* Pointer to a static buffer containing the hex string
 */
const char *get_unique_id_str(void)
{
    static char buf[9];
    uint32_t id = get_unique_id();
    // Forat as 8-digit hexadecimal string
    snprintf(buf, sizeof(buf), "%08lX", (unsigned long)id);
    return buf;
}

/**
 * @brief Generate a random 32-bit number using the LFSR algorithm
 * from Wikipedia: https://de.wikipedia.org/wiki/Linear_r%C3%BCckgekoppeltes_Schieberegister
 *
 */
uint32_t random32_lfsr(void)
{
    static unsigned r = 1;
    unsigned b = r & 1;
    r = (r >> 1) ^ (-b & 0xc3308398);
    return b;
}

/**
 * @brief Get a random 32-bit number using the RNG peripheral
 *
 * @return uint32_t
 */
uint32_t random32(void)
{
    return random32_lfsr(); // Use LFSR for now, as RNG is not available on MSP430
}

/**
 * @brief Get the family name of the chip
 *
 * @return const char* Chin family name
 */
const char *get_chip_family_name(void)
{
    return "MSP430FR5994";
}