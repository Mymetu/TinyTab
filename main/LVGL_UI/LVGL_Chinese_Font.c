#include "LVGL_Chinese_Font.h"

#include <string.h>

#include "FontUTF8.h"
#include "utf8_16x16.h"

#define UTF8_FONT_GLYPH_BYTES  ((utf8_16x16_WIDTH * utf8_16x16_HEIGHT) / 8U)

static int32_t utf8_font_find_glyph(uint32_t codepoint)
{
    if (codepoint > UINT16_MAX) return -1;

    int32_t low = 0;
    int32_t high = utf8_16x16_font.count - 1;
    while (low <= high) {
        const int32_t middle = low + (high - low) / 2;
        const uint16_t candidate = utf8_16x16_font.map[middle];
        if (candidate == codepoint) return middle;
        if (candidate < codepoint) low = middle + 1;
        else high = middle - 1;
    }
    return -1;
}

static bool utf8_font_get_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc_out,
                                    uint32_t letter, uint32_t letter_next)
{
    LV_UNUSED(font);
    LV_UNUSED(letter_next);
    const int32_t glyph_index = utf8_font_find_glyph(letter);
    if (glyph_index < 0) return false;

    memset(dsc_out, 0, sizeof(*dsc_out));
    dsc_out->adv_w = utf8_16x16_WIDTH * 16U;
    dsc_out->box_w = utf8_16x16_WIDTH;
    dsc_out->box_h = utf8_16x16_HEIGHT;
    dsc_out->stride = utf8_16x16_WIDTH;
    dsc_out->format = LV_FONT_GLYPH_FORMAT_A8;
    dsc_out->gid.index = (uint32_t)glyph_index;
    return true;
}

static const void *utf8_font_get_glyph_bitmap(lv_font_glyph_dsc_t *dsc,
                                               lv_draw_buf_t *draw_buf)
{
    if (draw_buf == NULL || draw_buf->data == NULL ||
        dsc->gid.index >= utf8_16x16_font.count) {
        return NULL;
    }

    const uint8_t *glyph = utf8_16x16_font.data +
                           dsc->gid.index * UTF8_FONT_GLYPH_BYTES;
    uint8_t *output = draw_buf->data;
    const uint32_t output_stride =
        lv_draw_buf_width_to_stride(utf8_16x16_WIDTH, LV_COLOR_FORMAT_A8);

    /* The reference font stores each x-column as two 8-pixel vertical pages. */
    for (uint32_t y = 0; y < utf8_16x16_HEIGHT; ++y) {
        for (uint32_t x = 0; x < utf8_16x16_WIDTH; ++x) {
            const uint8_t column_page = glyph[x * 2U + y / 8U];
            output[y * output_stride + x] =
                (column_page & (1U << (y & 7U))) ? 0xFF : 0x00;
        }
    }
    /* LVGL v9 expects the draw-buffer descriptor here, not the raw A8
     * pointer. Returning `output` makes the renderer interpret the first
     * glyph pixels as a lv_draw_buf_t and causes a Load access fault when a
     * Chinese glyph is drawn. */
    return draw_buf;
}

const lv_font_t lv_font_utf8_16x16 = {
    .get_glyph_dsc = utf8_font_get_glyph_dsc,
    .get_glyph_bitmap = utf8_font_get_glyph_bitmap,
    .line_height = 20,
    .base_line = 4,
    .subpx = LV_FONT_SUBPX_NONE,
    .kerning = LV_FONT_KERNING_NONE,
    .static_bitmap = 0,
    .underline_position = -2,
    .underline_thickness = 1,
    .fallback = &lv_font_montserrat_16,
};
