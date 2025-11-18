#include "msp430fr5994_utils.h"

/**
 * @brief Get the unique device ID from TLV memory
 *
 * @return uint32_t Unique device ID
 */

uint64_t get_unique_id(void)
{
    const uint8_t *tlv = (const uint8_t *)TLV_START;
    const uint8_t *tlv_end = (const uint8_t *)TLV_END;

    while (tlv < tlv_end)
    {
        uint8_t tag = *tlv++;
        uint8_t length = *tlv++;

        // Correct constant name:
        if (tag == TLV_DIERECORD)
        {
            // TLV_DIERECORD layout (SLAU367):
            // +0: Wafer ID (2 bytes)
            // +2: Die X position (2 bytes)
            // +4: Die Y position (2 bytes)
            // +6: Lot ID (6 bytes)
            const uint16_t wafer_id  = *(const uint16_t *)(tlv + 0);
            const uint16_t die_x_pos = *(const uint16_t *)(tlv + 2);
            const uint16_t die_y_pos = *(const uint16_t *)(tlv + 4);
            const uint8_t *lot_id    = tlv + 6;

            uint64_t unique_id = 0;

            // Combine into 64 bits: [lot_id(6 bytes)][wafer_id(2 bytes)]
            for (int i = 0; i < 6; ++i)
            {
                unique_id = (unique_id << 8) | lot_id[i];
            }
            unique_id = (unique_id << 16) | wafer_id;

            // Mix in die coordinates for extra entropy
            unique_id ^= ((uint64_t)die_x_pos << 32) | die_y_pos;

            return unique_id;
        }

        tlv += length;  // move to next TLV entry
    }

    return 0;  // fallback only if no TLV record found
}

/**
 * @brief Get the unique device ID as a string
 *
 * @return const uint8_t* Pointer to a static buffer containing the hex string
 */
const uint8_t *get_unique_id_str(void)
{
    static char buf[9];
    uint32_t id = (uint32_t)get_unique_id();
    // Format as 8-digit hexadecimal string
    snprintf(buf, sizeof(buf), "%08lX", (unsigned long)id);
    return (const uint8_t *)buf;
}

/**
 * @brief Generate a random 32-bit number using the LFSR algorithm
 * from: https://gist.github.com/nurpax/d32529b017f71fd0e77c89a9b5a5e328
 *
 */
uint32_t shift_lsfr(uint32_t* lfsr, uint32_t polynomial_mask)
{
    int feedback = *lfsr & 1;
    *lfsr >>= 1;
    if (feedback) {
        *lfsr ^= polynomial_mask;
    }
    return *lfsr;
}
uint32_t get_random()
{
    shift_lsfr(&lfsr32, POLYMASK_32);
    uint32_t a = shift_lsfr(&lfsr32, POLYMASK_32);
    uint32_t b = shift_lsfr(&lfsr31, POLYMASK_31);
    return (a ^ b) & 0xffff;
}

/**
 * @brief Get a random 32-bit number using the RNG peripheral
 *
 * @return uint32_t
 */
uint32_t random32(void)
{
    return get_random(); // Use LFSR for now, as RNG is not available on MSP430
}

/**
 * @brief Get the family name of the chip
 *
 * @return const uint8_t* Chip family name
 */
const char* get_chip_family_name(void)
{
    return "MSP430FR5994";
}