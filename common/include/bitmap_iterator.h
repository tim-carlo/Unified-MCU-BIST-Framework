#ifndef BITMAP_ITERATOR_H
#define BITMAP_ITERATOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct BitmapIterator
{
    uint64_t mask; // Remaining bitmask to iterate
} BitmapIterator;

BitmapIterator bitmap_iterator_create(uint64_t mask);
void bitmap_iterator_reset(BitmapIterator *it, uint64_t mask);
bool bitmap_iterator_hasnext(const BitmapIterator *it);
bool bitmap_iterator_next(BitmapIterator *it, uint8_t *out_bit);

#endif // BITMAP_ITERATOR_H