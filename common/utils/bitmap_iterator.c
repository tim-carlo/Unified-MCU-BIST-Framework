#include "bitmap_iterator.h"


/**
 * Create a bitmap iterator with the given mask
 * @param mask The bitmask to iterate over
 * @return BitmapIterator Initialized iterator
 */
BitmapIterator bitmap_iterator_create(uint64_t mask)
{
    BitmapIterator it;
    it.mask = mask;
    return it;
}

/**
 * Reset the iterator with a new mask
 * @param it Pointer to the BitmapIterator to reset
 * @param mask New bitmask to set 
 */
void bitmap_iterator_reset(BitmapIterator *it, uint64_t mask)
{
    it->mask = mask;
}

/**
 * Check if more bits are available
 * @param it Pointer to the BitmapIterator
 * @return true if more bits are available, false otherwise
 */
bool bitmap_iterator_hasnext(const BitmapIterator *it)
{
    return it->mask != 0;
}

/**
 * Get the next set bit LSB -> MSB order.
 * Returns true if a bit was found, false otherwise.
 * @param it Pointer to the BitmapIterator
 * @param out_bit Pointer to store the found bit index (0-63)
 * @return true if a bit was found, false otherwise
 */
bool bitmap_iterator_next(BitmapIterator *it, uint8_t *out_bit)
{
    if (it->mask == 0)
        return false;

    uint8_t bit = __builtin_ctzll(it->mask); // find lowest set bit
    it->mask &= it->mask - 1;                // clear that bit
    *out_bit = bit;
    return true;
}


bool bitmap_iterator_next_mask_as_param(uint64_t *mask, uint8_t *out_bit)
{
    if (mask == NULL || *mask == 0)
        return false;

    uint64_t m = *mask;
    uint8_t bit = __builtin_ctzll(m);  // find lowest set bit
    *mask &= m - 1;                     // clear that bit
    *out_bit = bit;
    return true;
}