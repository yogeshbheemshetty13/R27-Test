#include "en_dc.h"
#include <stdint.h>
#include <stddef.h>

/*
 * COBS (Consistent Overhead Byte Stuffing)
 *
 * This implementation:
 *   - encodes zero bytes safely
 *   - decodes encoded frames
 *   - checks NULL pointers
 *   - checks destination buffer size
 *   - checks malformed input
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

    /*
     * Even an empty input needs one byte in COBS encoding.
     */
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

            /*
             * Finish the current COBS block.
             */
            dst[code_index] = code;

            code_index = write_index;
            write_index++;

            code = 1u;

        } else {

            dst[write_index] = src[i];
            write_index++;
            code++;

            /*
             * COBS block can contain at most 254 non-zero bytes.
             */
            if (code == 0xFFu) {

                dst[code_index] = code;

                code_index = write_index;
                write_index++;

                code = 1u;
            }
        }
    }

    /*
     * Write the final block length.
     */
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

    /*
     * Empty encoded input is invalid for this decoder.
     */
    if (src_len == 0u) {
        result.status = DECODE_INPUT_TOO_SHORT;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    size_t src_index = 0u;
    size_t dst_index = 0u;

    while (src_index < src_len) {

        /*
         * The first byte of every COBS block is its code.
         */
        uint8_t code = src[src_index];

        if (code == 0u) {
            result.status |= DECODE_ZERO_BYTE_IN_INPUT;
            break;
        }

        src_index++;

        /*
         * code - 1 is the number of data bytes in this block.
         */
        size_t data_len = (size_t)code - 1u;

        /*
         * Make sure the encoded input actually contains
         * all bytes promised by the code byte.
         */
        if (data_len > src_len - src_index) {
            result.status |= DECODE_INPUT_TOO_SHORT;
            break;
        }

        /*
         * Make sure the destination has enough space.
         */
        if (data_len > dst_buf_len - dst_index) {
            result.status |= DECODE_OUT_BUFFER_OVERFLOW;
            break;
        }

        /*
         * Copy the non-zero bytes.
         */
        for (size_t i = 0u; i < data_len; ++i) {
            dst[dst_index++] = src[src_index++];
        }

        /*
         * If the code is less than 0xFF and this is NOT
         * the final block, COBS represents a zero byte.
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
