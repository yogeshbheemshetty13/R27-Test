#include "en_dc.h"
#include <stdint.h>

/*****************************************************************************
 * Functions
 ****************************************************************************/

/*
 * COBS (Consistent Overhead Byte Stuffing) encoder.
 *
 * The encoded output contains no zero bytes.
 */
encode_result frame_encode(void *dst_buf_ptr, size_t dst_buf_len,
                           const void *src_ptr, size_t src_len)
{
    encode_result result = {0u, ENCODE_OK};

    /* Check for NULL pointers */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
        result.status = ENCODE_NULL_POINTER;
        return result;
    }

    /* Check output buffer size */
    size_t required_len = ENCODE_DST_BUF_LEN_MAX(src_len);

    if (dst_buf_len < required_len) {
        result.status = ENCODE_OUT_BUFFER_OVERFLOW;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    /* Empty input */
    if (src_len == 0u) {
        dst[0] = 1u;
        result.out_len = 1u;
        return result;
    }

    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;

    while (read_index < src_len) {

        if (src[read_index] == 0u) {

            /* Finish current block */
            dst[code_index] = code;

            /* Start a new block */
            code_index = write_index;
            write_index++;
            code = 1u;

            read_index++;
        }
        else {

            /* Copy non-zero byte */
            dst[write_index] = src[read_index];

            write_index++;
            read_index++;
            code++;

            /*
             * Maximum COBS block size is 254 non-zero bytes.
             */
            if (code == 255u) {

                dst[code_index] = code;

                code_index = write_index;
                write_index++;
                code = 1u;
            }
        }
    }

    /* Finish final block */
    dst[code_index] = code;

    result.out_len = write_index;
    result.status = ENCODE_OK;

    return result;
}


/*
 * COBS decoder.
 */
decode_result frame_decode(void *dst_buf_ptr, size_t dst_buf_len,
                           const void *src_ptr, size_t src_len)
{
    decode_result result = {0u, DECODE_OK};

    /* Check for NULL pointers */
    if ((dst_buf_ptr == NULL) || (src_ptr == NULL)) {
        result.status = DECODE_NULL_POINTER;
        return result;
    }

    uint8_t *dst = (uint8_t *)dst_buf_ptr;
    const uint8_t *src = (const uint8_t *)src_ptr;

    /* Empty input is too short to contain a COBS frame */
    if (src_len == 0u) {
        result.status = DECODE_INPUT_TOO_SHORT;
        return result;
    }

    size_t read_index = 0u;
    size_t write_index = 0u;

    while (read_index < src_len) {

        uint8_t code = src[read_index];

        /* Zero is not allowed in a COBS encoded frame */
        if (code == 0u) {
            result.status = DECODE_ZERO_BYTE_IN_INPUT;
            return result;
        }

        /*
         * The code byte must point to bytes that actually
         * exist in the input.
         */
        if ((size_t)code > (src_len - read_index)) {
            result.status = DECODE_INPUT_TOO_SHORT;
            return result;
        }

        /*
         * Copy the bytes belonging to this block.
         */
        for (size_t i = 1u; i < (size_t)code; i++) {

            if (write_index >= dst_buf_len) {
                result.status = DECODE_OUT_BUFFER_OVERFLOW;
                return result;
            }

            dst[write_index] = src[read_index + i];
            write_index++;
        }

        read_index += (size_t)code;

        /*
         * If another block follows, the original data
         * contained a zero byte between the blocks.
         */
        if (read_index < src_len) {

            if (write_index >= dst_buf_len) {
                result.status = DECODE_OUT_BUFFER_OVERFLOW;
                return result;
            }

            dst[write_index] = 0u;
            write_index++;
        }
    }

    result.out_len = write_index;
    result.status = DECODE_OK;

    return result;
}
