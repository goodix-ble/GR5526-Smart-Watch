/*******************************************************************************
 * Base Size: 8 px
 * Bpp: 1
 * Cmd-Opts: --no-compress --no-prefilter --bpp 1 --size 8 --font unscii-8.ttf -r 0x20-0x7F --format lvgl -o lv_font_unscii_8.c --force-fast-kern-format
 ******************************************************************************/

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "../../lvgl.h"
#endif

#ifndef LV_FONT_UNSCII_8
#if GR552X_GPU_FONT_SUPPORT
#define LV_FONT_UNSCII_8 1
#else
#define LV_FONT_UNSCII_8 0
#endif
#endif

#if LV_FONT_UNSCII_8

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] __attribute__((aligned(32))) = {
    /* U+0020 " " */
    0x0, 0x0, 0x0, 0x0,

    /* U+0021 "!" */
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0xc0, 0x0,

    /* U+0022 "\"" */
    0xcc, 0xcc, 0xcc, 0x0,

    /* U+0023 "#" */
    0x6c, 0x6c, 0xfe, 0x6c, 0xfe, 0x6c, 0x6c, 0x0,

    /* U+0024 "$" */
    0x30, 0x7c, 0xc0, 0x78, 0xc, 0xf8, 0x30, 0x0,

    /* U+0025 "%" */
    0xc6, 0xcc, 0x18, 0x30, 0x66, 0xc6, 0x0, 0x0,

    /* U+0026 "&" */
    0x38, 0x6c, 0x38, 0x76, 0xdc, 0xcc, 0x76, 0x0,

    /* U+0027 "'" */
    0x60, 0x60, 0xc0, 0x0,

    /* U+0028 "(" */
    0x30, 0x60, 0xc0, 0xc0, 0xc0, 0x60, 0x30, 0x0,

    /* U+0029 ")" */
    0xc0, 0x60, 0x30, 0x30, 0x30, 0x60, 0xc0, 0x0,

    /* U+002A "*" */
    0x66, 0x3c, 0xff, 0x3c, 0x66, 0x0, 0x0, 0x0,

    /* U+002B "+" */
    0x30, 0x30, 0xfc, 0x30, 0x30, 0x0, 0x0, 0x0,

    /* U+002C "," */
    0x60, 0x60, 0xc0, 0x0,

    /* U+002D "-" */
    0xfc, 0x0, 0x0, 0x0,

    /* U+002E "." */
    0xc0, 0xc0, 0x0, 0x0,

    /* U+002F "/" */
    0x3, 0x6, 0xc, 0x18, 0x30, 0x60, 0xc0, 0x0,

    /* U+0030 "0" */
    0x78, 0xcc, 0xdc, 0xec, 0xcc, 0xcc, 0x78, 0x0,

    /* U+0031 "1" */
    0x30, 0x70, 0x30, 0x30, 0x30, 0x30, 0xfc, 0x0,

    /* U+0032 "2" */
    0x78, 0xcc, 0x18, 0x30, 0x60, 0xc0, 0xfc, 0x0,

    /* U+0033 "3" */
    0x78, 0xcc, 0xc, 0x38, 0xc, 0xcc, 0x78, 0x0,

    /* U+0034 "4" */
    0x1c, 0x3c, 0x6c, 0xcc, 0xfe, 0xc, 0xc, 0x0,

    /* U+0035 "5" */
    0xfc, 0xc0, 0xf8, 0xc, 0xc, 0xcc, 0x78, 0x0,

    /* U+0036 "6" */
    0x38, 0x60, 0xc0, 0xf8, 0xcc, 0xcc, 0x78, 0x0,

    /* U+0037 "7" */
    0xfc, 0xc, 0xc, 0x18, 0x30, 0x30, 0x30, 0x0,

    /* U+0038 "8" */
    0x78, 0xcc, 0xcc, 0x78, 0xcc, 0xcc, 0x78, 0x0,

    /* U+0039 "9" */
    0x78, 0xcc, 0xcc, 0x7c, 0xc, 0x18, 0x70, 0x0,

    /* U+003A ":" */
    0xc0, 0xc0, 0x0, 0x0, 0xc0, 0xc0, 0x0, 0x0,

    /* U+003B ";" */
    0x60, 0x60, 0x0, 0x0, 0x60, 0x60, 0xc0, 0x0,

    /* U+003C "<" */
    0x18, 0x30, 0x60, 0xc0, 0x60, 0x30, 0x18, 0x0,

    /* U+003D "=" */
    0xfc, 0x0, 0xfc, 0x0,

    /* U+003E ">" */
    0xc0, 0x60, 0x30, 0x18, 0x30, 0x60, 0xc0, 0x0,

    /* U+003F "?" */
    0x78, 0xcc, 0xc, 0x18, 0x30, 0x0, 0x30, 0x0,

    /* U+0040 "@" */
    0x7c, 0xc6, 0xde, 0xde, 0xde, 0xc0, 0x7c, 0x0,

    /* U+0041 "A" */
    0x30, 0x78, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0x0,

    /* U+0042 "B" */
    0xf8, 0xcc, 0xcc, 0xf8, 0xcc, 0xcc, 0xf8, 0x0,

    /* U+0043 "C" */
    0x78, 0xcc, 0xc0, 0xc0, 0xc0, 0xcc, 0x78, 0x0,

    /* U+0044 "D" */
    0xf0, 0xd8, 0xcc, 0xcc, 0xcc, 0xd8, 0xf0, 0x0,

    /* U+0045 "E" */
    0xfc, 0xc0, 0xc0, 0xf8, 0xc0, 0xc0, 0xfc, 0x0,

    /* U+0046 "F" */
    0xfc, 0xc0, 0xc0, 0xf8, 0xc0, 0xc0, 0xc0, 0x0,

    /* U+0047 "G" */
    0x78, 0xcc, 0xc0, 0xdc, 0xcc, 0xcc, 0x7c, 0x0,

    /* U+0048 "H" */
    0xcc, 0xcc, 0xcc, 0xfc, 0xcc, 0xcc, 0xcc, 0x0,

    /* U+0049 "I" */
    0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0xfc, 0x0,

    /* U+004A "J" */
    0xc, 0xc, 0xc, 0xc, 0xc, 0xcc, 0x78, 0x0,

    /* U+004B "K" */
    0xc6, 0xcc, 0xd8, 0xf0, 0xd8, 0xcc, 0xc6, 0x0,

    /* U+004C "L" */
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xfc, 0x0,

    /* U+004D "M" */
    0xc6, 0xee, 0xfe, 0xd6, 0xc6, 0xc6, 0xc6, 0x0,

    /* U+004E "N" */
    0xc6, 0xe6, 0xf6, 0xde, 0xce, 0xc6, 0xc6, 0x0,

    /* U+004F "O" */
    0x78, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x0,

    /* U+0050 "P" */
    0xf8, 0xcc, 0xcc, 0xf8, 0xc0, 0xc0, 0xc0, 0x0,

    /* U+0051 "Q" */
    0x78, 0xcc, 0xcc, 0xcc, 0xcc, 0xd8, 0x6c, 0x0,

    /* U+0052 "R" */
    0xf8, 0xcc, 0xcc, 0xf8, 0xd8, 0xcc, 0xcc, 0x0,

    /* U+0053 "S" */
    0x78, 0xcc, 0xc0, 0x78, 0xc, 0xcc, 0x78, 0x0,

    /* U+0054 "T" */
    0xfc, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x0,

    /* U+0055 "U" */
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x0,

    /* U+0056 "V" */
    0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x0,

    /* U+0057 "W" */
    0xc6, 0xc6, 0xc6, 0xd6, 0xfe, 0xee, 0xc6, 0x0,

    /* U+0058 "X" */
    0xc3, 0x66, 0x3c, 0x18, 0x3c, 0x66, 0xc3, 0x0,

    /* U+0059 "Y" */
    0xc3, 0x66, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x0,

    /* U+005A "Z" */
    0xfc, 0xc, 0x18, 0x30, 0x60, 0xc0, 0xfc, 0x0,

    /* U+005B "[" */
    0xf0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xf0, 0x0,

    /* U+005C "\\" */
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x6, 0x3, 0x0,

    /* U+005D "]" */
    0xf0, 0x30, 0x30, 0x30, 0x30, 0x30, 0xf0, 0x0,

    /* U+005E "^" */
    0x10, 0x38, 0x6c, 0xc6,

    /* U+005F "_" */
    0xff, 0x0, 0x0, 0x0,

    /* U+0060 "`" */
    0xc0, 0x60, 0x30, 0x0,

    /* U+0061 "a" */
    0x78, 0xc, 0x7c, 0xcc, 0x7c, 0x0, 0x0, 0x0,

    /* U+0062 "b" */
    0xc0, 0xc0, 0xf8, 0xcc, 0xcc, 0xcc, 0xf8, 0x0,

    /* U+0063 "c" */
    0x78, 0xc0, 0xc0, 0xc0, 0x78, 0x0, 0x0, 0x0,

    /* U+0064 "d" */
    0xc, 0xc, 0x7c, 0xcc, 0xcc, 0xcc, 0x7c, 0x0,

    /* U+0065 "e" */
    0x78, 0xcc, 0xfc, 0xc0, 0x78, 0x0, 0x0, 0x0,

    /* U+0066 "f" */
    0x38, 0x60, 0xf8, 0x60, 0x60, 0x60, 0x60, 0x0,

    /* U+0067 "g" */
    0x7c, 0xcc, 0xcc, 0x7c, 0xc, 0xf8, 0x0, 0x0,

    /* U+0068 "h" */
    0xc0, 0xc0, 0xf8, 0xcc, 0xcc, 0xcc, 0xcc, 0x0,

    /* U+0069 "i" */
    0x60, 0x0, 0xe0, 0x60, 0x60, 0x60, 0x78, 0x0,

    /* U+006A "j" */
    0x18, 0x0, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf0,

    /* U+006B "k" */
    0xc0, 0xc0, 0xcc, 0xd8, 0xf0, 0xd8, 0xcc, 0x0,

    /* U+006C "l" */
    0xe0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x78, 0x0,

    /* U+006D "m" */
    0xcc, 0xfe, 0xd6, 0xd6, 0xc6, 0x0, 0x0, 0x0,

    /* U+006E "n" */
    0xf8, 0xcc, 0xcc, 0xcc, 0xcc, 0x0, 0x0, 0x0,

    /* U+006F "o" */
    0x78, 0xcc, 0xcc, 0xcc, 0x78, 0x0, 0x0, 0x0,

    /* U+0070 "p" */
    0xf8, 0xcc, 0xcc, 0xf8, 0xc0, 0xc0, 0x0, 0x0,

    /* U+0071 "q" */
    0x7c, 0xcc, 0xcc, 0x7c, 0xc, 0xc, 0x0, 0x0,

    /* U+0072 "r" */
    0xf8, 0xcc, 0xc0, 0xc0, 0xc0, 0x0, 0x0, 0x0,

    /* U+0073 "s" */
    0x7c, 0xc0, 0x78, 0xc, 0xf8, 0x0, 0x0, 0x0,

    /* U+0074 "t" */
    0x60, 0x60, 0xfc, 0x60, 0x60, 0x60, 0x3c, 0x0,

    /* U+0075 "u" */
    0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0, 0x0, 0x0,

    /* U+0076 "v" */
    0xcc, 0xcc, 0xcc, 0x78, 0x30, 0x0, 0x0, 0x0,

    /* U+0077 "w" */
    0xc6, 0xc6, 0xd6, 0x7c, 0x6c, 0x0, 0x0, 0x0,

    /* U+0078 "x" */
    0xc6, 0x6c, 0x38, 0x6c, 0xc6, 0x0, 0x0, 0x0,

    /* U+0079 "y" */
    0xcc, 0xcc, 0xcc, 0x7c, 0xc, 0x78, 0x0, 0x0,

    /* U+007A "z" */
    0xfc, 0x18, 0x30, 0x60, 0xfc, 0x0, 0x0, 0x0,

    /* U+007B "{" */
    0x1c, 0x30, 0x30, 0xe0, 0x30, 0x30, 0x1c, 0x0,

    /* U+007C "|" */
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0,

    /* U+007D "}" */
    0xe0, 0x30, 0x30, 0x1c, 0x30, 0x30, 0xe0, 0x0,

    /* U+007E "~" */
    0x76, 0xdc, 0x0, 0x0,

    /* U+007F "" */
    0xc0, 0xa0, 0xae, 0xa4, 0xc4, 0x4, 0x4, 0x0
};

/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 128, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 8},
    {.bitmap_index = 4, .adv_w = 128, .box_w = 2, .box_h = 7, .ofs_x = 3, .ofs_y = 1},
    {.bitmap_index = 12, .adv_w = 128, .box_w = 6, .box_h = 3, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 16, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 24, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 32, .adv_w = 128, .box_w = 7, .box_h = 6, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 40, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 48, .adv_w = 128, .box_w = 3, .box_h = 3, .ofs_x = 2, .ofs_y = 5},
    {.bitmap_index = 52, .adv_w = 128, .box_w = 4, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 60, .adv_w = 128, .box_w = 4, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 68, .adv_w = 128, .box_w = 8, .box_h = 5, .ofs_x = 0, .ofs_y = 2},
    {.bitmap_index = 76, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 84, .adv_w = 128, .box_w = 3, .box_h = 3, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 88, .adv_w = 128, .box_w = 6, .box_h = 1, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 92, .adv_w = 128, .box_w = 2, .box_h = 2, .ofs_x = 3, .ofs_y = 1},
    {.bitmap_index = 96, .adv_w = 128, .box_w = 8, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 104, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 112, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 120, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 128, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 136, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 144, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 152, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 160, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 168, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 176, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 184, .adv_w = 128, .box_w = 2, .box_h = 6, .ofs_x = 3, .ofs_y = 1},
    {.bitmap_index = 192, .adv_w = 128, .box_w = 3, .box_h = 7, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 200, .adv_w = 128, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 208, .adv_w = 128, .box_w = 6, .box_h = 3, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 212, .adv_w = 128, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 220, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 228, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 236, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 244, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 252, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 260, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 268, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 276, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 284, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 292, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 300, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 308, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 316, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 324, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 332, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 340, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 348, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 356, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 364, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 372, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 380, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 388, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 396, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 404, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 412, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 420, .adv_w = 128, .box_w = 8, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 428, .adv_w = 128, .box_w = 8, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 436, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 444, .adv_w = 128, .box_w = 4, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 452, .adv_w = 128, .box_w = 8, .box_h = 7, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 460, .adv_w = 128, .box_w = 4, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 468, .adv_w = 128, .box_w = 7, .box_h = 4, .ofs_x = 0, .ofs_y = 4},
    {.bitmap_index = 472, .adv_w = 128, .box_w = 8, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 476, .adv_w = 128, .box_w = 4, .box_h = 3, .ofs_x = 3, .ofs_y = 5},
    {.bitmap_index = 480, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 488, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 496, .adv_w = 128, .box_w = 5, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 504, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 512, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 520, .adv_w = 128, .box_w = 5, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 528, .adv_w = 128, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 536, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 544, .adv_w = 128, .box_w = 5, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 552, .adv_w = 128, .box_w = 5, .box_h = 8, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 560, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 568, .adv_w = 128, .box_w = 5, .box_h = 7, .ofs_x = 2, .ofs_y = 1},
    {.bitmap_index = 576, .adv_w = 128, .box_w = 7, .box_h = 5, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 584, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 592, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 600, .adv_w = 128, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 608, .adv_w = 128, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 616, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 624, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 632, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 640, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 648, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 656, .adv_w = 128, .box_w = 7, .box_h = 5, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 664, .adv_w = 128, .box_w = 7, .box_h = 5, .ofs_x = 0, .ofs_y = 1},
    {.bitmap_index = 672, .adv_w = 128, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 680, .adv_w = 128, .box_w = 6, .box_h = 5, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 688, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 696, .adv_w = 128, .box_w = 2, .box_h = 7, .ofs_x = 3, .ofs_y = 1},
    {.bitmap_index = 704, .adv_w = 128, .box_w = 6, .box_h = 7, .ofs_x = 1, .ofs_y = 1},
    {.bitmap_index = 712, .adv_w = 128, .box_w = 7, .box_h = 2, .ofs_x = 0, .ofs_y = 6},
    {.bitmap_index = 716, .adv_w = 128, .box_w = 7, .box_h = 7, .ofs_x = 0, .ofs_y = 1}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 96, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
lv_font_t lv_font_unscii_8 = {
#else
lv_font_t lv_font_unscii_8 = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 9,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = 0,
    .underline_thickness = 0,
#endif
    .dsc = &font_dsc,      /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
    .origin_dsc = &font_dsc,
    .origin_line_height = 9,
    .origin_base_line = 0,
};



#endif /*#if LV_FONT_UNSCII_8*/

