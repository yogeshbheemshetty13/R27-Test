#include "en_dc.h"
#include <stdint.h>
#include <stddef.h>

/*
 * COBS (Consistent Overhead Byte Stuffing)
 *
 * Encode:
 *   Converts arbitrary bytes, including 0x00, into a frame
 *   containing no zero bytes.
 *
 * Decode:
 *   Converts a valid COBS frame back into the original data.
 */

/* ------------------------------------------------------------------------- */
/* Encode                                                                    */
/* ------------------------------------------------------------------------- */

encode_result frame_encode(void *dst_buf_ptr,
                           size_t dst_buf_len,
                           const void *src_ptr,
                           size_t src_len)
{
    encode_result result = {0u, ENCODE_OK};

    if (dst_buf_ptr == NULL || src_ptr == NULL) {
        result.status = ENCODE_NULL_POINTER;
        return result;
    }

    size_t required_len = ENCODE_DST_BUF_LEN_MAX(src_len);

    if (dst_buf_len < required_len) {
        result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    size_t code_index = 0u;
    size_t write_index = 1u;
    uint8_t code = 1u;

    for (size_t i = 0u; i < src_len; ++i) {

        if (src[i] == 0u) {
            dst[code_index] = code;

            code_index = write_index;
            write_index++;

            code = 1u;
        } else {
            dst[write_index] = src[i];
            write_index++;
            code++;

            if (code == 0xFFu) {
                dst[code_index] = code;

                code_index = write_index;
                write_index++;

                code = 1u;
            }
        }
    }

    dst[code_index] = code;

    result.out_len = write_index;

    return result;
}


/* ------------------------------------------------------------------------- */
/* Decode                                                                    */
/* ------------------------------------------------------------------------- */

decode_result frame_decode(void *dst_buf_ptr,
                           size_t dst_buf_len,
                           const void *src_ptr,
                           size_t src_len)
{
    decode_result result = {0u, DECODE_OK};

    if (dst_buf_ptr == NULL || src_ptr == NULL) {
        result.status = DECODE_NULL_POINTER;
        return result;
    }

    if (src_len == 0u) {
        result.status = DECODE_INPUT_TOO_SHORT;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    size_t src_index = 0u;
    size_t dst_index = 0u;

    while (src_index < src_len) {

        uint8_t code = src[src_index];

        /*
         * A COBS code byte can never be zero.
         */
        if (code == 0u) {
            result.status |= DECODE_ZERO_BYTE_IN_INPUT;
            break;
        }

        src_index++;

        size_t data_len = (size_t)code - 1u;

        /*
         * Check that the encoded frame contains
         * all bytes described by the code byte.
         */
        if (data_len > src_len - src_index) {
            result.status |= DECODE_INPUT_TOO_SHORT;
            break;
        }

        /*
         * Check destination capacity before copying.
         */
        if (data_len > dst_buf_len - dst_index) {
            result.status |= DECODE_OUT_BUFFER_OVERFLOW;
            break;
        }

        for (size_t i = 0u; i < data_len; ++i) {
            dst[dst_index++] = src[src_index++];
        }

        /*
         * A code value below 0xFF indicates that a zero byte
         * follows this block, unless this is the final block.
         */
        if (code < 0xFFu && src_index < src_len) {

            if (dst_index >= dst_buf_len) {
                result.status |= DECODE_OUT_BUFFER_OVERFLOW;
                break;
            }

            dst[dst_index++] = 0u;
        }
    }

    result.out_len = dst_index;

    return result;
}
