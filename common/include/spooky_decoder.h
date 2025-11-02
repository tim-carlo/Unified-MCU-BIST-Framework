#ifndef SPOOKY_DECODE_H
#define SPOOKY_DECODE_H

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/* The smallest a buffer can be and still have space for clock recovery. */
#define SPOOKY_DECODER_MIN_BUFFER_SIZE 16
#define SPOOKY_DECODER_MAX_BUFFER_SIZE 255


struct spooky_decoder {
    uint16_t index;             /* current index in buffer */
    uint8_t buffer_size;        /* buffer size, in bytes */
    uint8_t mode;               /* current state */
    uint8_t ticks;              /* ticks since last logic level change */
    uint8_t bit_index;          /* index of current bit in bit_accum */
    uint8_t bit_accum;          /* accumulator for signal bits */
    uint8_t last;               /* last bit received */
    uint8_t interval;           /* avg. interval between single edges */
    uint8_t payload_length;     /* bytes in payload */
    uint8_t chksum;             /* sum-and-invert checksum for payload */
    uint8_t pre_ticks;          /* tick count during setup part of bit frame */

    /* internal buffer, used for clock recovery and to accumulate payload */
    uint8_t *buffer;
};
// Structs packed

enum spooky_decoder_init_res {
    SPOOKY_DECODER_INIT_OK = 0,
    SPOOKY_DECODER_INIT_ERROR_NULL = -1,
    SPOOKY_DECODER_INIT_ERROR_BAD_ARGUMENT = -2,
};

enum spooky_decoder_step_res {
    SPOOKY_DECODER_STEP_OK = 0,
    SPOOKY_DECODER_STEP_DONE = 1,
    SPOOKY_DECODER_STEP_ERROR_NULL = -1,
};

/* Initialize a spooky decoder. */
enum spooky_decoder_init_res
spooky_decoder_init(struct spooky_decoder *dec,
    uint8_t *output_buffer, size_t buffer_size);

/* Step the decoder, with a new bit of input.
 * If a complete message has been received, the callback
 * passed to spooky_decoder_init will be called with it. */
enum spooky_decoder_step_res
spooky_decoder_step(struct spooky_decoder *dec, bool bit);

/* Reset the decoder to initial state. */
void reset_decoder(struct spooky_decoder *dec);

#endif
