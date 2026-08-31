#include "en_dc.h"
#include <stdint.h>
#include <stddef.h>

/*****************************************************************************
 * Functions
 ****************************************************************************/

/*
 * Encode data using a COBS-style zero-byte-safe format.
 *
 * Every code byte tells how many bytes follow before the next
 * zero byte in the original data.
 */
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
     * Calculate the maximum encoded size required.
     */
    size_t required_len = ENCODE_DST_BUF_LEN_MAX(src_len);

    if (dst_buf_len < required_len) {
        result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    /*
     * Empty input is represented by one code byte.
     */
    if (src_len == 0u) {
        dst[0] = 1u;
        result.out_len = 1u;
        return result;
    }

    size_t code_index = 0u;
    size_t write_index = 1u;
    uint8_t code = 1u;

    for (size_t i = 0u; i < src_len; i++) {

        /*
         * A zero byte ends the current block.
         */
        if (src[i] == 0u) {
            dst[code_index] = code;

            code_index = write_index;
            write_index++;

            code = 1u;
        } else {
            dst[write_index++] = src[i];
            code++;

            /*
             * A code byte can represent at most 254 data bytes.
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
     * Write the final code byte.
     */
    dst[code_index] = code;

    result.out_len = write_index;

    return result;
}


/*
 * Decode data produced by frame_encode().
 */
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
     * There must be at least one code byte.
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

        uint8_t code = src[src_index++];

        /*
         * Code zero is invalid.
         */
        if (code == 0u) {
            result.status |= DECODE_ZERO_BYTE_IN_INPUT;
            return result;
        }

        size_t copy_len = (size_t)code - 1u;

        /*
         * Make sure the encoded input actually contains
         * all bytes described by the code.
         */
        if (copy_len > src_len - src_index) {
            result.status |= DECODE_INPUT_TOO_SHORT;
            return result;
        }

        /*
         * Make sure the destination buffer is large enough.
         */
        if (copy_len > dst_buf_len - dst_index) {
            result.status |= DECODE_OUT_BUFFER_OVERFLOW;
            return result;
        }

        /*
         * Copy the non-zero data bytes.
         */
        for (size_t i = 0u; i < copy_len; i++) {
            dst[dst_index++] = src[src_index++];
        }

        /*
         * If this is not the final encoded block and the code
         * is less than 255, the original data contained a zero.
         */
        if (src_index < src_len && code < 0xFFu) {

            if (dst_index >= dst_buf_len) {
                result.status |= DECODE_OUT_BUFFER_OVERFLOW;
                return result;
            }

            dst[dst_index++] = 0u;
        }
    }

    result.out_len = dst_index;

    return result;
}
