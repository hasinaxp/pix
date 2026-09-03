#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "dtype.hpp"

const char * FONT_CHAR_SET = " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~";

#define FONT_ATLAS_WIDTH 1024
#define FONT_ATLAS_HEIGHT 1024

#define FONT_ATLAS_PADDING 2
#define FONT_ATLAS_GLYPH_WIDTH 64

// signed distance stored around each outline, in atlas pixels
#define FONT_SDF_SPREAD 6


struct font_icon_entry {
    idx charcode;
    idx glyph_index;
};


#define PIX_CHAR_ARROW_UP 1
#define PIX_CHAR_ARROW_DOWN 2
#define PIX_CHAR_ARROW_LEFT 3
#define PIX_CHAR_ARROW_RIGHT 4
#define PIX_CHAR_CHECK 5
#define PIX_CHAR_CROSS 6
#define PIX_CHAR_CIRCLE 7
#define PIX_CHAR_SQUARE 8
#define PIX_CHAR_TRIANGLE_UP 9
#define PIX_CHAR_TRIANGLE_DOWN 10




const font_icon_entry font_icon_entries[] = {
    { 0x2191 , PIX_CHAR_ARROW_UP},
    { 0x2193 , PIX_CHAR_ARROW_DOWN},
    { 0x2190 , PIX_CHAR_ARROW_LEFT},
    { 0x2192 , PIX_CHAR_ARROW_RIGHT},
    { 0x2713 , PIX_CHAR_CHECK},
    { 0x2717 , PIX_CHAR_CROSS},
    { 0x25CB , PIX_CHAR_CIRCLE},
    { 0x25A1 , PIX_CHAR_SQUARE},
    { 0x25B2 , PIX_CHAR_TRIANGLE_UP},
    { 0x25BC , PIX_CHAR_TRIANGLE_DOWN}
};


struct glyph {
    vec4 crop;    // atlas pixel rect { x, y, w, h }, already grown by the sdf spread
    vec2 offset;  // pen (baseline origin) -> crop top-left, y grows down
    float advance;

};

// bitmap is a signed distance field: 0.5 is the outline, > 0.5 inside,
// one texel = one atlas pixel, saturating at +/- sdf_spread pixels.
// alpha = smoothstep(0.5 - w, 0.5 + w, texel), w = 0.5 / (sdf_spread * scale)
struct font_data {
    size_t width;
    size_t height;
    char * bitmap; // 1 channel, 8-bit

    float ascent;      // pixels above the baseline
    float descent;     // pixels below the baseline (negative)
    float line_height; // baseline to baseline
    float sdf_spread;  // atlas pixels covered by the field

    idx atlas_texture; // 0 until text.hpp lazily uploads the bitmap on first use

    glyph glyphs[128];
};

font_data pix_load_font_ttf(const char* path);
void pix_free_font(font_data& font);

// unicode codepoint -> font_data::glyphs index (0 when unsupported)
static idx pix_font_slot(idx codepoint);

//----------------------------implementation----------------------------

// Minimal TrueType loader: 'glyf' outlines (simple + composite), cmap formats
// 0/4/6/12, flattened to line segments once and used twice - a non-zero
// winding scanline pass for the sign, an exact point-to-segment pass for the
// distance. No third-party code.

#define TTF_MAX_POINTS   1024
#define TTF_MAX_CONTOURS 64
#define TTF_MAX_EDGES    4096
#define TTF_MAX_CROSS    256
#define TTF_SUBSAMPLES   4   // vertical samples per pixel row, sign only
#define TTF_CELL         FONT_ATLAS_GLYPH_WIDTH
#define TTF_COLS         (FONT_ATLAS_WIDTH / FONT_ATLAS_GLYPH_WIDTH)
#define TTF_PAD          (FONT_ATLAS_PADDING + FONT_SDF_SPREAD)

static uint16_t ttf__u16(const unsigned char* p) { return (uint16_t)((p[0] << 8) | p[1]); }
static int16_t  ttf__i16(const unsigned char* p) { return (int16_t)ttf__u16(p); }
static uint32_t ttf__u32(const unsigned char* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static float ttf__f2dot14(const unsigned char* p) { return (float)ttf__i16(p) / 16384.0f; }

struct ttf__reader {
    const unsigned char* loca;
    const unsigned char* glyf;
    const unsigned char* hmtx;
    const unsigned char* cmap; // the chosen subtable, not the table header
    int loc_long;              // indexToLocFormat
    int num_glyphs;
    int num_hmetrics;
};

static const unsigned char* ttf__table(const unsigned char* d, size_t size, const char* tag) {
    if (size < 12) return 0;
    unsigned int base = 0;
    if (memcmp(d, "ttcf", 4) == 0) {          // collection: take the first font
        if (size < 16) return 0;
        base = ttf__u32(d + 12);
        if ((size_t)base + 12 > size) return 0;
    }
    int n = ttf__u16(d + base + 4);
    for (int i = 0; i < n; i++) {
        const unsigned char* rec = d + base + 12 + 16 * i;
        if ((size_t)(rec + 16 - d) > size) break;
        if (memcmp(rec, tag, 4) == 0) {
            unsigned int off = ttf__u32(rec + 8);
            return (size_t)off < size ? d + off : 0;
        }
    }
    return 0;
}

// most capable unicode subtable available
static const unsigned char* ttf__pick_cmap(const unsigned char* cmap) {
    if (!cmap) return 0;
    int n = ttf__u16(cmap + 2);
    const unsigned char* best = 0;
    int best_score = -1;
    for (int i = 0; i < n; i++) {
        const unsigned char* rec = cmap + 4 + 8 * i;
        int plat = ttf__u16(rec), enc = ttf__u16(rec + 2);
        const unsigned char* sub = cmap + ttf__u32(rec + 4);
        int fmt = ttf__u16(sub), score = -1;
        if (plat == 3 && enc == 10 && fmt == 12) score = 4; // full unicode
        else if (plat == 0 && fmt == 12)         score = 3;
        else if (plat == 3 && enc == 1)          score = 2; // BMP
        else if (plat == 0)                      score = 1;
        else if (plat == 3 && enc == 0)          score = 0; // symbol
        else if (plat == 1 && enc == 0)          score = 0; // mac roman
        if (score > best_score) { best_score = score; best = sub; }
    }
    return best;
}

static int ttf__glyph_id(const ttf__reader* f, unsigned int cp) {
    const unsigned char* c = f->cmap;
    if (!c) return 0;
    switch (ttf__u16(c)) {
    case 0:
        return cp < 256 ? c[6 + cp] : 0;
    case 4: {
        if (cp > 0xFFFF) return 0;
        int segs = ttf__u16(c + 6) >> 1;
        const unsigned char* endc  = c + 14;
        const unsigned char* start = endc + segs * 2 + 2;
        const unsigned char* delta = start + segs * 2;
        const unsigned char* range = delta + segs * 2;
        for (int i = 0; i < segs; i++) {
            if (cp > (unsigned int)ttf__u16(endc + i * 2)) continue;
            unsigned int lo = ttf__u16(start + i * 2);
            if (cp < lo) return 0;
            int ro = ttf__u16(range + i * 2);
            if (!ro) return lo == 0xFFFF ? 0 : (int)((cp + ttf__u16(delta + i * 2)) & 0xFFFF);
            int gid = ttf__u16(range + i * 2 + ro + (cp - lo) * 2);
            return gid ? (int)((gid + ttf__u16(delta + i * 2)) & 0xFFFF) : 0;
        }
        return 0;
    }
    case 6: {
        unsigned int first = ttf__u16(c + 6), count = ttf__u16(c + 8);
        return (cp >= first && cp < first + count) ? ttf__u16(c + 10 + (cp - first) * 2) : 0;
    }
    case 12: {
        unsigned int groups = ttf__u32(c + 12);
        for (unsigned int i = 0; i < groups; i++) {
            const unsigned char* g = c + 16 + i * 12;
            unsigned int lo = ttf__u32(g), hi = ttf__u32(g + 4);
            if (cp < lo) return 0;
            if (cp <= hi) return (int)(ttf__u32(g + 8) + (cp - lo));
        }
        return 0;
    }
    }
    return 0;
}

static float ttf__advance(const ttf__reader* f, int gid) {
    if (!f->hmtx || f->num_hmetrics <= 0) return 0.0f;
    int i = gid < f->num_hmetrics ? gid : f->num_hmetrics - 1;
    return (float)ttf__u16(f->hmtx + i * 4);
}

// ---------------- outlines ----------------

struct ttf__pt { float x, y; unsigned char on; };

struct ttf__outline {
    ttf__pt pts[TTF_MAX_POINTS];
    int ends[TTF_MAX_CONTOURS];
    int np, nc;
};

static const unsigned char* ttf__glyph_data(const ttf__reader* f, int gid, unsigned int* len) {
    if (gid < 0 || gid >= f->num_glyphs) return 0;
    unsigned int a, b;
    if (f->loc_long) { a = ttf__u32(f->loca + gid * 4);     b = ttf__u32(f->loca + gid * 4 + 4); }
    else             { a = ttf__u16(f->loca + gid * 2) * 2u; b = ttf__u16(f->loca + gid * 2 + 2) * 2u; }
    if (b <= a) return 0;
    *len = b - a;
    return f->glyf + a;
}

// appends the transformed outline of gid; x' = a*x + c*y + dx, y' = b*x + d*y + dy
static void ttf__decode(const ttf__reader* f, int gid, ttf__outline* o,
                        float a, float b, float c, float d, float dx, float dy, int depth) {
    unsigned int len = 0;
    const unsigned char* g = ttf__glyph_data(f, gid, &len);
    if (!g || len < 10 || depth > 4) return;

    int ncont = ttf__i16(g);

    if (ncont < 0) { // composite
        const unsigned char* p = g + 10;
        for (;;) {
            if ((size_t)(p + 4 - g) > len) break;
            int flags = ttf__u16(p), sub = ttf__u16(p + 2);
            p += 4;
            float ox = 0.0f, oy = 0.0f;
            if (flags & 0x0001) { // ARG_1_AND_2_ARE_WORDS
                if (flags & 0x0002) { ox = (float)ttf__i16(p); oy = (float)ttf__i16(p + 2); }
                p += 4;
            } else {
                if (flags & 0x0002) { ox = (float)(signed char)p[0]; oy = (float)(signed char)p[1]; }
                p += 2;
            }
            float sa = 1.0f, sb = 0.0f, sc = 0.0f, sd = 1.0f;
            if (flags & 0x0008)      { sa = sd = ttf__f2dot14(p); p += 2; }
            else if (flags & 0x0040) { sa = ttf__f2dot14(p); sd = ttf__f2dot14(p + 2); p += 4; }
            else if (flags & 0x0080) { sa = ttf__f2dot14(p);     sb = ttf__f2dot14(p + 2);
                                       sc = ttf__f2dot14(p + 4); sd = ttf__f2dot14(p + 6); p += 8; }
            ttf__decode(f, sub, o,
                        sa * a + sb * c, sa * b + sb * d,
                        sc * a + sd * c, sc * b + sd * d,
                        ox * a + oy * c + dx, ox * b + oy * d + dy, depth + 1);
            if (!(flags & 0x0020)) break; // MORE_COMPONENTS
        }
        return;
    }

    if (ncont == 0 || o->nc + ncont > TTF_MAX_CONTOURS) return;

    const unsigned char* p = g + 10;
    int base = o->np;
    int npts = ttf__u16(p + (ncont - 1) * 2) + 1;
    if (npts <= 0 || base + npts > TTF_MAX_POINTS) return;
    for (int i = 0; i < ncont; i++) o->ends[o->nc + i] = base + ttf__u16(p + i * 2);
    p += ncont * 2;
    p += 2 + ttf__u16(p); // skip hinting instructions

    unsigned char flags[TTF_MAX_POINTS];
    for (int i = 0; i < npts; ) {
        unsigned char fl = *p++;
        flags[i++] = fl;
        if (fl & 8) { int r = *p++; while (r-- > 0 && i < npts) flags[i++] = fl; } // REPEAT
    }

    int v = 0;
    for (int i = 0; i < npts; i++) {
        unsigned char fl = flags[i];
        if (fl & 2)          { int s = *p++; v += (fl & 16) ? s : -s; }
        else if (!(fl & 16)) { v += ttf__i16(p); p += 2; }
        o->pts[base + i].x = (float)v;
    }
    v = 0;
    for (int i = 0; i < npts; i++) {
        unsigned char fl = flags[i];
        if (fl & 4)          { int s = *p++; v += (fl & 32) ? s : -s; }
        else if (!(fl & 32)) { v += ttf__i16(p); p += 2; }
        ttf__pt* pt = &o->pts[base + i];
        float px = pt->x, py = (float)v;
        pt->x = px * a + py * c + dx;
        pt->y = px * b + py * d + dy;
        pt->on = (unsigned char)(fl & 1);
    }

    o->np += npts;
    o->nc += ncont;
}

// ---------------- flattening ----------------

// y0 <= y1 always; dir is the original winding direction, 0 for horizontal
// (skipped by the scanline pass, still used by the distance pass)
struct ttf__edge { float x0, y0, x1, y1, xlo, xhi; int dir; };

static void ttf__line(ttf__edge* e, int* ne, float x0, float y0, float x1, float y1) {
    if (*ne >= TTF_MAX_EDGES) return;
    if (x0 == x1 && y0 == y1) return;
    ttf__edge* t = &e[(*ne)++];
    if (y0 <= y1) { t->x0 = x0; t->y0 = y0; t->x1 = x1; t->y1 = y1; t->dir = y0 < y1 ? 1 : 0; }
    else          { t->x0 = x1; t->y0 = y1; t->x1 = x0; t->y1 = y0; t->dir = -1; }
    t->xlo = t->x0 < t->x1 ? t->x0 : t->x1;
    t->xhi = t->x0 < t->x1 ? t->x1 : t->x0;
}

static void ttf__quad(ttf__edge* e, int* ne, float x0, float y0, float cx, float cy, float x1, float y1) {
    float fx = fabsf(x0 - 2.0f * cx + x1), fy = fabsf(y0 - 2.0f * cy + y1);
    int n = (int)(sqrtf((fx + fy) * 2.0f) + 1.0f);
    if (n > 16) n = 16;
    float px = x0, py = y0, inv = 1.0f / (float)n;
    for (int i = 1; i <= n; i++) {
        float t = (float)i * inv, s = 1.0f - t;
        float qx = s * s * x0 + 2.0f * s * t * cx + t * t * x1;
        float qy = s * s * y0 + 2.0f * s * t * cy + t * t * y1;
        ttf__line(e, ne, px, py, qx, qy);
        px = qx; py = qy;
    }
}

static int ttf__flatten(const ttf__outline* o, ttf__edge* e) {
    int ne = 0, s = 0;
    for (int ci = 0; ci < o->nc; ci++) {
        int en = o->ends[ci];
        int n = en - s + 1;
        if (n < 2) { s = en + 1; continue; }

        // start on an on-curve point, synthesising one if the contour has none
        float sx, sy;
        int i0;
        if (o->pts[s].on)       { sx = o->pts[s].x;  sy = o->pts[s].y;  i0 = 1; }
        else if (o->pts[en].on) { sx = o->pts[en].x; sy = o->pts[en].y; i0 = 0; }
        else { sx = 0.5f * (o->pts[s].x + o->pts[en].x);
               sy = 0.5f * (o->pts[s].y + o->pts[en].y); i0 = 0; }

        float cx = sx, cy = sy;
        for (int k = 0; k < n; ) {
            const ttf__pt* p = &o->pts[s + (i0 + k) % n];
            if (p->on) {
                ttf__line(e, &ne, cx, cy, p->x, p->y);
                cx = p->x; cy = p->y;
                k++;
            } else {
                const ttf__pt* q = &o->pts[s + (i0 + k + 1) % n];
                float ex, ey;
                if (q->on) { ex = q->x; ey = q->y; k += 2; }
                else { ex = 0.5f * (p->x + q->x); ey = 0.5f * (p->y + q->y); k += 1; } // implied on-curve
                ttf__quad(e, &ne, cx, cy, p->x, p->y, ex, ey);
                cx = ex; cy = ey;
            }
        }
        ttf__line(e, &ne, cx, cy, sx, sy);
        s = en + 1;
    }
    return ne;
}

// ---------------- coverage (sign source) ----------------

static void ttf__span(float* cov, int w, float x0, float x1, float weight) {
    if (x0 < 0.0f) x0 = 0.0f;
    if (x1 > (float)w) x1 = (float)w;
    if (x1 <= x0) return;
    int i0 = (int)x0, i1 = (int)x1;
    if (i1 >= w) i1 = w - 1;
    if (i0 == i1) { cov[i0] += (x1 - x0) * weight; return; }
    cov[i0] += ((float)(i0 + 1) - x0) * weight;
    for (int i = i0 + 1; i < i1; i++) cov[i] += weight;
    cov[i1] += (x1 - (float)i1) * weight;
}

// non-zero winding, TTF_SUBSAMPLES sub-scanlines with exact horizontal coverage
static void ttf__raster(const ttf__edge* e, int ne, unsigned char* dst, int stride,
                        int w, int y_begin, int y_end) {
    float cov[TTF_CELL];
    float xs[TTF_MAX_CROSS];
    int   ds[TTF_MAX_CROSS];
    const float sub = 1.0f / (float)TTF_SUBSAMPLES;

    for (int py = y_begin; py < y_end; py++) {
        memset(cov, 0, sizeof(float) * (size_t)w);
        for (int s = 0; s < TTF_SUBSAMPLES; s++) {
            float y = (float)py + ((float)s + 0.5f) * sub;
            int nx = 0;
            for (int i = 0; i < ne && nx < TTF_MAX_CROSS; i++) {
                if (!e[i].dir || y < e[i].y0 || y >= e[i].y1) continue;
                xs[nx] = e[i].x0 + (y - e[i].y0) * (e[i].x1 - e[i].x0) / (e[i].y1 - e[i].y0);
                ds[nx] = e[i].dir;
                nx++;
            }
            for (int i = 1; i < nx; i++) { // crossings come in nearly sorted
                float xv = xs[i];
                int dv = ds[i], j = i - 1;
                while (j >= 0 && xs[j] > xv) { xs[j + 1] = xs[j]; ds[j + 1] = ds[j]; j--; }
                xs[j + 1] = xv; ds[j + 1] = dv;
            }
            int wind = 0;
            for (int i = 0; i + 1 < nx; i++) {
                wind += ds[i];
                if (wind) ttf__span(cov, w, xs[i], xs[i + 1], sub);
            }
        }
        unsigned char* row = dst + (size_t)py * stride;
        for (int x = 0; x < w; x++) {
            float v = cov[x];
            if (v <= 0.0f) continue;
            if (v > 1.0f) v = 1.0f;
            row[x] = (unsigned char)(v * 255.0f + 0.5f);
        }
    }
}

// ---------------- distance field ----------------

static float ttf__seg_dist2(float px, float py, const ttf__edge* e) {
    float vx = e->x1 - e->x0, vy = e->y1 - e->y0;
    float wx = px - e->x0,    wy = py - e->y0;
    float len = vx * vx + vy * vy;
    float t = len > 0.0f ? (wx * vx + wy * vy) / len : 0.0f;
    if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
    float dx = wx - t * vx, dy = wy - t * vy;
    return dx * dx + dy * dy;
}

// signed distance over [x0,x1) x [y0,y1), sign taken from the coverage mask
static void ttf__sdf(const ttf__edge* e, int ne, const unsigned char* cov,
                     unsigned char* dst, int stride, int x0, int y0, int x1, int y1) {
    const float spread = (float)FONT_SDF_SPREAD;
    const float far2 = spread * spread;
    int active[TTF_MAX_EDGES];

    for (int py = y0; py < y1; py++) {
        float fy = (float)py + 0.5f;
        int na = 0;
        for (int i = 0; i < ne; i++)                 // rows share the same y band
            if (fy >= e[i].y0 - spread && fy <= e[i].y1 + spread) active[na++] = i;

        unsigned char* row = dst + (size_t)py * stride;
        const unsigned char* crow = cov + (size_t)py * stride;
        for (int px = x0; px < x1; px++) {
            float fx = (float)px + 0.5f;
            float best = far2;
            for (int a = 0; a < na; a++) {
                const ttf__edge* ed = &e[active[a]];
                float gx = fx < ed->xlo ? ed->xlo - fx : (fx > ed->xhi ? fx - ed->xhi : 0.0f);
                if (gx * gx >= best) continue;       // bbox lower bound
                float gy = fy < ed->y0 ? ed->y0 - fy : (fy > ed->y1 ? fy - ed->y1 : 0.0f);
                if (gx * gx + gy * gy >= best) continue;
                float d2 = ttf__seg_dist2(fx, fy, ed);
                if (d2 < best) best = d2;
            }
            float d = sqrtf(best);
            if (crow[px] < 128) d = -d;              // outside
            float v = 0.5f + 0.5f * d / spread;
            if (v <= 0.0f) continue;
            if (v > 1.0f) v = 1.0f;
            row[px] = (unsigned char)(v * 255.0f + 0.5f);
        }
    }
}

// ---------------- atlas ----------------

static idx pix_font_slot(idx codepoint) {
    if (codepoint >= 32 && codepoint < 127) return codepoint;
    for (idx i = 0; i < sizeof(font_icon_entries) / sizeof(font_icon_entries[0]); i++)
        if (font_icon_entries[i].charcode == codepoint) return font_icon_entries[i].glyph_index;
    return 0;
}

void pix_free_font(font_data& font) {
    free(font.bitmap);
    font.bitmap = 0;
}

font_data pix_load_font_ttf(const char* path) {
    font_data font = {};
    font.width      = FONT_ATLAS_WIDTH;
    font.height     = FONT_ATLAS_HEIGHT;
    font.sdf_spread = (float)FONT_SDF_SPREAD;
    font.bitmap     = (char*)calloc(FONT_ATLAS_WIDTH * FONT_ATLAS_HEIGHT, 1);
    if (!font.bitmap) return font;

    FILE* fp = fopen(path, "rb");
    if (!fp) return font;
    fseek(fp, 0, SEEK_END);
    long bytes = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    unsigned char* src = (unsigned char*)malloc((size_t)bytes);
    if (!src) { fclose(fp); return font; }
    size_t got = fread(src, 1, (size_t)bytes, fp);
    fclose(fp);

    const unsigned char* head = ttf__table(src, got, "head");
    const unsigned char* hhea = ttf__table(src, got, "hhea");
    const unsigned char* maxp = ttf__table(src, got, "maxp");

    ttf__reader f = {};
    f.loca = ttf__table(src, got, "loca");
    f.glyf = ttf__table(src, got, "glyf");
    f.hmtx = ttf__table(src, got, "hmtx");
    if (!head || !hhea || !maxp || !f.loca || !f.glyf) { free(src); return font; }

    f.cmap         = ttf__pick_cmap(ttf__table(src, got, "cmap"));
    f.loc_long     = ttf__u16(head + 50);
    f.num_glyphs   = ttf__u16(maxp + 4);
    f.num_hmetrics = ttf__u16(hhea + 34);

    float ascent  = (float)ttf__i16(hhea + 4);
    float descent = (float)ttf__i16(hhea + 6);
    float gap     = (float)ttf__i16(hhea + 8);
    float span    = ascent - descent;
    if (span <= 0.0f) span = (float)ttf__u16(head + 18);
    if (span <= 0.0f) { free(src); return font; }

    // ascender..descender is scaled to fill one cell minus padding and spread
    float scale  = (float)(TTF_CELL - 2 * TTF_PAD) / span;
    float base_y = (float)TTF_PAD + ascent * scale; // baseline, cell local
    font.ascent      = ascent * scale;
    font.descent     = descent * scale;
    font.line_height = (span + gap) * scale;

    unsigned int cps[128] = {};
    for (int i = 0; FONT_CHAR_SET[i]; i++) {
        unsigned int c = (unsigned char)FONT_CHAR_SET[i];
        if (c < 128) cps[c] = c;
    }
    for (idx i = 0; i < sizeof(font_icon_entries) / sizeof(font_icon_entries[0]); i++)
        cps[font_icon_entries[i].glyph_index & 127] = font_icon_entries[i].charcode;

    ttf__outline* out   = (ttf__outline*)malloc(sizeof(ttf__outline));
    ttf__edge*    edges = (ttf__edge*)malloc(sizeof(ttf__edge) * TTF_MAX_EDGES);
    unsigned char cov[TTF_CELL * TTF_CELL];
    unsigned char cell[TTF_CELL * TTF_CELL];
    if (!out || !edges) { free(out); free(edges); free(src); return font; }

    for (int slot = 0; slot < 128; slot++) {
        if (!cps[slot]) continue;
        int gid = ttf__glyph_id(&f, cps[slot]);
        glyph* gl = &font.glyphs[slot];
        gl->advance = ttf__advance(&f, gid) * scale;

        out->np = out->nc = 0; // gid 0 is .notdef, so unmapped codepoints show a box
        ttf__decode(&f, gid, out, scale, 0.0f, 0.0f, -scale, (float)TTF_PAD, base_y, 0);
        if (out->np < 2) continue; // blank glyph, advance only

        float minx = out->pts[0].x, maxx = minx;
        float miny = out->pts[0].y, maxy = miny;
        for (int i = 1; i < out->np; i++) {
            const ttf__pt* p = &out->pts[i];
            if (p->x < minx) minx = p->x;
            if (p->x > maxx) maxx = p->x;
            if (p->y < miny) miny = p->y;
            if (p->y > maxy) maxy = p->y;
        }
        int bx0 = (int)floorf(minx), by0 = (int)floorf(miny);
        int bx1 = (int)ceilf(maxx) + 1, by1 = (int)ceilf(maxy) + 1;
        if (bx0 < 0) bx0 = 0;
        if (by0 < 0) by0 = 0;
        if (bx1 > TTF_CELL) bx1 = TTF_CELL;
        if (by1 > TTF_CELL) by1 = TTF_CELL;
        if (bx1 <= bx0 || by1 <= by0) continue;

        int ne = ttf__flatten(out, edges);
        if (!ne) continue;

        // the field extends one spread past the outline, so the crop does too
        int gx0 = bx0 - FONT_SDF_SPREAD, gy0 = by0 - FONT_SDF_SPREAD;
        int gx1 = bx1 + FONT_SDF_SPREAD, gy1 = by1 + FONT_SDF_SPREAD;
        if (gx0 < 0) gx0 = 0;
        if (gy0 < 0) gy0 = 0;
        if (gx1 > TTF_CELL) gx1 = TTF_CELL;
        if (gy1 > TTF_CELL) gy1 = TTF_CELL;

        memset(cov, 0, sizeof(cov));
        memset(cell, 0, sizeof(cell));
        ttf__raster(edges, ne, cov, TTF_CELL, TTF_CELL, by0, by1);
        ttf__sdf(edges, ne, cov, cell, TTF_CELL, gx0, gy0, gx1, gy1);

        int cx = (slot % TTF_COLS) * TTF_CELL;
        int cy = (slot / TTF_COLS) * TTF_CELL;
        for (int y = gy0; y < gy1; y++)
            memcpy(font.bitmap + (size_t)(cy + y) * FONT_ATLAS_WIDTH + cx + gx0,
                   cell + y * TTF_CELL + gx0, (size_t)(gx1 - gx0));

        gl->crop.x   = (float)(cx + gx0);
        gl->crop.y   = (float)(cy + gy0);
        gl->crop.z   = (float)(gx1 - gx0);
        gl->crop.w   = (float)(gy1 - gy0);
        gl->offset.x = (float)gx0 - (float)TTF_PAD;
        gl->offset.y = (float)gy0 - base_y;
    }

    free(out);
    free(edges);
    free(src);
    return font;
}
