#pragma once
#include <stdint.h>

/**
 * Source of image. */
enum {
    LV_IMG_SRC_VARIABLE, /** Binary/C variable */
    LV_IMG_SRC_FILE, /** File in filesystem */
    LV_IMG_SRC_SYMBOL, /** Symbol (@ref lv_symbol_def.h) */
    LV_IMG_SRC_UNKNOWN, /** Unknown source */
};

typedef uint8_t lv_img_src_t;
/**
 * LittlevGL image header
 */
typedef struct
{

    /* The first 8 bit is very important to distinguish the different source types.
     * For more info see `lv_img_get_src_type()` in lv_img.c */
    uint32_t cf : 5;          /* Color format: See `lv_img_color_format_t`*/
    uint32_t always_zero : 3; /*It the upper bits of the first byte. Always zero to look like a
                                 non-printable character*/

    uint32_t reserved : 2; /*Reserved to be used later*/

    uint32_t w : 11; /*Width of the image map*/
    uint32_t h : 11; /*Height of the image map*/
} lv_img_header_t;

/*Image color format*/
enum {
    LV_IMG_CF_UNKNOWN = 0,

    LV_IMG_CF_RAW,              /**< Contains the file as it is. Needs custom decoder function*/
    LV_IMG_CF_RAW_ALPHA,        /**< Contains the file as it is. The image has alpha. Needs custom decoder
                                   function*/
    LV_IMG_CF_RAW_CHROMA_KEYED, /**< Contains the file as it is. The image is chroma keyed. Needs
                                   custom decoder function*/

    LV_IMG_CF_TRUE_COLOR,              /**< Color format and depth should match with LV_COLOR settings*/
    LV_IMG_CF_TRUE_COLOR_ALPHA,        /**< Same as `LV_IMG_CF_TRUE_COLOR` but every pixel has an alpha byte*/
    LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED, /**< Same as `LV_IMG_CF_TRUE_COLOR` but LV_COLOR_TRANSP pixels
                                          will be transparent*/

    LV_IMG_CF_INDEXED_1BIT, /**< Can have 2 different colors in a palette (always chroma keyed)*/
    LV_IMG_CF_INDEXED_2BIT, /**< Can have 4 different colors in a palette (always chroma keyed)*/
    LV_IMG_CF_INDEXED_4BIT, /**< Can have 16 different colors in a palette (always chroma keyed)*/
    LV_IMG_CF_INDEXED_8BIT, /**< Can have 256 different colors in a palette (always chroma keyed)*/

    LV_IMG_CF_ALPHA_1BIT, /**< Can have one color and it can be drawn or not*/
    LV_IMG_CF_ALPHA_2BIT, /**< Can have one color but 4 different alpha value*/
    LV_IMG_CF_ALPHA_4BIT, /**< Can have one color but 16 different alpha value*/
    LV_IMG_CF_ALPHA_8BIT, /**< Can have one color but 256 different alpha value*/

    LV_IMG_CF_RESERVED_15,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_16,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_17,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_18,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_19,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_20,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_21,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_22,              /**< Reserved for further use. */
    LV_IMG_CF_RESERVED_23,              /**< Reserved for further use. */

    LV_IMG_CF_USER_ENCODED_0,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_1,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_2,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_3,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_4,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_5,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_6,          /**< User holder encoding format. */
    LV_IMG_CF_USER_ENCODED_7,          /**< User holder encoding format. */
};
typedef uint8_t lv_img_cf_t;

/** Image header it is compatible with
 * the result from image converter utility*/
typedef struct
{
    lv_img_header_t header;
    uint32_t data_size;
    const uint8_t * data;
} lv_img_dsc_t;