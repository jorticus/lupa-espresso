#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "lv_color.h"

#define LV_ATTRIBUTE_LARGE_CONST 

/** Magic number for lvgl image, 9 means lvgl version 9
 *  It must be neither a valid ASCII character nor larger than 0x80. See `lv_image_src_get_type`.
 */
#define LV_IMAGE_HEADER_MAGIC (0x19)
//LV_EXPORT_CONST_INT(LV_IMAGE_HEADER_MAGIC);

typedef enum _lv_image_flags_t {
    /**
     * For RGB map of the image data, mark if it's pre-multiplied with alpha.
     * For indexed image, this bit indicated palette data is pre-multiplied with alpha.
     */
    LV_IMAGE_FLAGS_PREMULTIPLIED    = 0x0001,
    /**
     * The image data is compressed, so decoder needs to decode image firstly.
     * If this flag is set, the whole image will be decompressed upon decode, and
     * `get_area_cb` won't be necessary.
     */
    LV_IMAGE_FLAGS_COMPRESSED       = 0x0008,

    /*Below flags are applicable only for draw buffer header.*/

    /**
     * The image is allocated from heap, thus should be freed after use.
     */
    LV_IMAGE_FLAGS_ALLOCATED        = 0x0010,

    /**
     * If the image data is malloced and can be processed in place.
     * In image decoder post processing, this flag means we modify it in-place.
     */
    LV_IMAGE_FLAGS_MODIFIABLE       = 0x0020,

    /**
     * Flags reserved for user, lvgl won't use these bits.
     */
    LV_IMAGE_FLAGS_USER1            = 0x0100,
    LV_IMAGE_FLAGS_USER2            = 0x0200,
    LV_IMAGE_FLAGS_USER3            = 0x0400,
    LV_IMAGE_FLAGS_USER4            = 0x0800,
    LV_IMAGE_FLAGS_USER5            = 0x1000,
    LV_IMAGE_FLAGS_USER6            = 0x2000,
    LV_IMAGE_FLAGS_USER7            = 0x4000,
    LV_IMAGE_FLAGS_USER8            = 0x8000,
} lv_image_flags_t;

typedef enum {
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
} lv_img_cf_t;

typedef struct {
    uint32_t magic: 8;          /*Magic number. Must be LV_IMAGE_HEADER_MAGIC*/
    uint32_t cf : 8;            /*Color format: See `lv_color_format_t`*/
    uint32_t flags: 16;         /*Image flags, see `lv_image_flags_t`*/

    uint32_t w: 16;
    uint32_t h: 16;
    uint32_t stride: 16;        /*Number of bytes in a row*/
    uint32_t reserved_2: 16;    /*Reserved to be used later*/

    // For backwards compatability
    uint8_t always_zero;
    uint8_t reserved;
} lv_image_header_t;

typedef struct {
    lv_image_header_t header;   /**< A header describing the basics of the image*/
    uint32_t data_size;         /**< Size of the image in bytes*/
    const uint8_t * data;       /**< Pointer to the data of the image*/
    const void * reserved;      /**< A reserved field to make it has same size as lv_draw_buf_t*/
} lv_image_dsc_t;

// Backwards compatability
// (Layout is different, but member names are the same)
typedef lv_image_dsc_t lv_img_dsc_t;

#ifdef __cplusplus
} /*extern "C"*/
#endif
