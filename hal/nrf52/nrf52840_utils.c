#include "nrf52840_utils.h"
/**
 * @brief Generate a random 32-bit number using the LFSR algorithm
 * from Wikipedia: https://de.wikipedia.org/wiki/Linear_r%C3%BCckgekoppeltes_Schieberegister
 *
 */
#include "nrf.h" // Standard CMSIS header for nRF52840

uint32_t random32_lfsr(void)
{
    static uint32_t r = 0; 
    if (r == 0) 
    {
        // Access the Factory Information Configuration Registers (FICR)
        uint32_t id_low  = NRF_FICR->DEVICEID[0];
        uint32_t id_high = NRF_FICR->DEVICEID[1];

        // XOR the lower and upper 32 bits to create a unique 32-bit seed
        r = id_low ^ id_high;

        // Safety check: An LFSR must never be seeded with 0, otherwise it stays 0 forever.
        if (r == 0) {
            r = 0xC3308398; 
        }
    }

    // 3. The LFSR Algorithm
    unsigned b = r & 1;
    r = (r >> 1) ^ (-b & 0xc3308398);
    return r; 
}

/**
 * @brief Get a random 32-bit number using the RNG peripheral
 *
 * @return uint32_t
 */
uint32_t hardware_random32(void)
{
    uint32_t rnd = 0;
    for (int i = 0; i < 4; i++)
    {
        if (NRF_RNG->TASKS_START == 0)
            NRF_RNG->TASKS_START = 1;

        while (!NRF_RNG->EVENTS_VALRDY)
        {
        }

        rnd |= ((uint32_t)NRF_RNG->VALUE) << (8 * i);
        NRF_RNG->EVENTS_VALRDY = 0;
    }
    return rnd;
}

uint16_t hardware_random16(void)
{
    uint16_t rnd = 0;
    for (int i = 0; i < 2; i++)
    {
        if (NRF_RNG->TASKS_START == 0)
            NRF_RNG->TASKS_START = 1;

        while (!NRF_RNG->EVENTS_VALRDY)
        {
        }

        rnd |= ((uint16_t)NRF_RNG->VALUE) << (8 * i);
        NRF_RNG->EVENTS_VALRDY = 0;
    }
    return rnd;
}

uint32_t random32(void)
{
    // Prefer lfsr method instead of hardware RNG for performance and simplicity
    return random32_lfsr();
}

uint16_t random16(void)
{
    // Prefer lfsr method instead of hardware RNG for performance and simplicity
    return (uint16_t)(random32_lfsr() & 0xFFFF);
}

/**
 * @brief Get the unique id object
 *
 * @return uint64_t
 */
uint64_t get_unique_id(void)
{
    return ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
}

/**
 * @brief Get the unique id as a string
 *
 * @return const uint8_t*
 */
const uint8_t *get_unique_id_str(void)
{
    static uint8_t unique_id_str[17];
    uint64_t unique_id = get_unique_id();
    const uint8_t hex[] = "0123456789ABCDEF";
    for (int i = 0; i < 8; i++)
    {
        uint8_t byte = (unique_id >> (56 - i * 8)) & 0xFF;
        unique_id_str[i * 2] = hex[byte >> 4];
        unique_id_str[i * 2 + 1] = hex[byte & 0x0F];
    }
    unique_id_str[16] = '\0';
    return unique_id_str;
}

/**
 * @brief Get the chip family name
 *
 * @return const uint8_t*
 */
const char *get_chip_family_name(void)
{
    return "NRF52840";
}
