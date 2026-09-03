#pragma once
#include "font.hpp"
#include "sprite.hpp"



// bundles every rendering variety on top of a font's baked SDF atlas. size,
// weight and italic are synthetic (the atlas is baked once, at one size,
// with no bold/oblique flags read from the file) - for a genuinely distinct
// face, load a separate font_data from a different TTF and use that instead
struct text_style {
    float size;            // pixels; 0 = font's natural baked size (ascent - descent)
    float weight;          // sdf fill-edge bias: 0 = normal, + bolder, - thinner (sane range ~-0.25..0.25)
    float italic;           // shear factor: 0 = upright, ~0.2 = typical italic slant
    float letter_spacing;   // extra pixels of advance after each glyph
    vec4 color;
    vec4 outline_color;
    float outline_width;    // 0 = no outline; normalized sdf units, ~0.05..0.3 is visible
};

static text_style pix_default_style(vec4 color = { 1.0f, 1.0f, 1.0f, 1.0f }) {
    text_style s = {};
    s.color = color;
    s.outline_color = { 0.0f, 0.0f, 0.0f, 1.0f };
    return s;
}

struct pixi_text {
    font_data* font;
    sprite_batch batch;
    text_style style;
    vec2 position; // baseline of the first character; pix_update_text re-lays-out from here
};

pixi_text pix_create_text(
    font_data* font,
    const char* text,
    vec2 position,
    text_style style = pix_default_style(),
    size_t capacity = 0); // glyph slots to bake room for; 0 = exactly fit `text`

void pix_destroy_text(pixi_text& text);

// re-lays-out the same baked batch for new content (reuses the GPU buffer, no realloc);
// glyphs beyond the batch's capacity are silently dropped, same as push_sprite
void pix_update_text(pixi_text& text, const char* new_text);

// pixel width `text` would occupy, widest line if it contains '\n' - handy for right/centre alignment
static float pix_text_width(font_data* font, const char* text, const text_style& style = pix_default_style());

static void draw_text(const pixi_text& text, idx shader, const mat4& view_proj);

//----------------------------implementation----------------------------

// decodes one UTF-8 codepoint so callers can embed the icon glyphs (font_icon_entries
// keys them by real unicode codepoint, e.g. U+2191) directly in a source literal
static unsigned int text__utf8_next(const char** p) {
    unsigned char c = (unsigned char)**p;
    if (c < 0x80) { (*p)++; return c; }
    int extra = (c & 0xE0) == 0xC0 ? 1 : (c & 0xF0) == 0xE0 ? 2 : (c & 0xF8) == 0xF0 ? 3 : 0;
    unsigned int cp = extra == 1 ? c & 0x1F : extra == 2 ? c & 0x0F : extra == 3 ? c & 0x07 : c;
    (*p)++;
    for (int i = 0; i < extra && ((unsigned char)**p & 0xC0) == 0x80; i++) {
        cp = (cp << 6) | ((unsigned char)**p & 0x3F);
        (*p)++;
    }
    return cp;
}

// expands the 1-channel sdf bitmap to RGBA8 so it can go through the shared texture helper
static idx text__upload_atlas(font_data& font) {
    size_t n = font.width * font.height;
    unsigned char* rgba = (unsigned char*)malloc(n * 4);
    for (size_t i = 0; i < n; i++) {
        unsigned char v = (unsigned char)font.bitmap[i];
        rgba[i * 4 + 0] = rgba[i * 4 + 1] = rgba[i * 4 + 2] = rgba[i * 4 + 3] = v;
    }
    idx tex = opengl_create_texture2d((int)font.width, (int)font.height, 4, rgba, TEXTURE_LINEAR);
    free(rgba);
    return tex;
}

// size / the font's own em span; the sdf atlas stays sharp at any scale, so
// resizing text is just this multiplier applied to every layout quantity
static float text__scale(const font_data* font, const text_style& style) {
    float natural = font->ascent - font->descent;
    return (style.size > 0.0f && natural > 0.0f) ? style.size / natural : 1.0f;
}

static void text__layout(pixi_text& t, const char* text) {
    clear_sprites(t.batch);
    idx atlas_slot = pix_batch_texture(t.batch, t.font->atlas_texture);

    float scale = text__scale(t.font, t.style);
    float line_height = t.font->line_height * scale;

    float pen_x = t.position.x, pen_y = t.position.y;
    const char* p = text;
    while (*p) {
        unsigned int cp = text__utf8_next(&p);
        if (cp == '\n') { pen_x = t.position.x; pen_y += line_height; continue; }

        const glyph& g = t.font->glyphs[pix_font_slot(cp)];
        if (g.crop.z > 0.0f) {
            sprite s = {};
            s.box = { pen_x + g.offset.x * scale, pen_y + g.offset.y * scale, g.crop.z * scale, g.crop.w * scale };
            s.crop = g.crop;
            s.texture = atlas_slot;
            push_sprite(t.batch, s);
        }
        pen_x += g.advance * scale + t.style.letter_spacing;
    }
}

pixi_text pix_create_text(font_data* font, const char* text, vec2 position, text_style style, size_t capacity) {
    pixi_text t = {};
    t.font = font;
    t.style = style;
    t.position = position;

    size_t len = strlen(text);
    t.batch = pix_create_sprite_batch(capacity ? capacity : (len ? len : 1));
    if (!font->atlas_texture) font->atlas_texture = text__upload_atlas(*font);

    text__layout(t, text);
    return t;
}

void pix_destroy_text(pixi_text& text) {
    pix_destroy_sprite_batch(text.batch);
}

void pix_update_text(pixi_text& text, const char* new_text) {
    text__layout(text, new_text);
}

static float pix_text_width(font_data* font, const char* text, const text_style& style) {
    float scale = text__scale(font, style);
    float width = 0.0f, max_width = 0.0f;
    const char* p = text;
    while (*p) {
        unsigned int cp = text__utf8_next(&p);
        if (cp == '\n') { if (width > max_width) max_width = width; width = 0.0f; continue; }
        width += font->glyphs[pix_font_slot(cp)].advance * scale + style.letter_spacing;
    }
    return width > max_width ? width : max_width;
}

static void draw_text(const pixi_text& text, idx shader, const mat4& view_proj) {
    glUseProgram(shader);
    const text_style& st = text.style;
    glUniform4f(glGetUniformLocation(shader, "uColor"), st.color.x, st.color.y, st.color.z, st.color.w);
    glUniform4f(glGetUniformLocation(shader, "uOutlineColor"),
        st.outline_color.x, st.outline_color.y, st.outline_color.z, st.outline_color.w);
    glUniform1f(glGetUniformLocation(shader, "uWeight"), st.weight);
    glUniform1f(glGetUniformLocation(shader, "uOutlineWidth"), st.outline_width);
    glUniform1f(glGetUniformLocation(shader, "uItalic"), st.italic);
    draw_sprites(text.batch, shader, view_proj);
}
