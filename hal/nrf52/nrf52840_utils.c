#include "nrf52840_utils.h"
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

uint16_t random16(void)
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
 * @return const char*
 */
const char *get_unique_id_str(void)
{
    static char unique_id_str[17];
    uint64_t unique_id = get_unique_id();
    const char hex[] = "0123456789ABCDEF";
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
 * @return const char*
 */
const char *get_chip_family_name(void)
{
    return "NRF52840";
}
