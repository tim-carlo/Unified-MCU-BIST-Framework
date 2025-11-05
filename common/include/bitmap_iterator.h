#ifndef BITMAP_ITERATOR_H
#define BITMAP_ITERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   // for NULL
#include <stdbool.h>
#include <stdint.h>

typedef struct BitmapIterator
{
    uint64_t mask; // Remaining bitmask to iterate
} BitmapIterator;

BitmapIterator bitmap_iterator_create(uint64_t mask);
void bitmap_iterator_reset(BitmapIterator *it, uint64_t mask);
bool bitmap_iterator_hasnext(const BitmapIterator *it);
bool bitmap_iterator_next(BitmapIterator *it, uint8_t *out_bit);
bool bitmap_iterator_next_mask_as_param(uint64_t *mask, uint8_t *out_bit);

#endif // BITMAP_ITERATOR_H