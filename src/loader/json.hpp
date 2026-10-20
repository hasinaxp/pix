// pix - a small JSON reader: tokenises a document in one pass into a flat
// array of tokens, then answers lookups by walking it. No allocation of its
// own and no copies of the text - strings and numbers are read in place.
//
//   size_t size;
//   char* text = (char*)pix_read_file("level.json", &size);
//   js_doc doc;
//   int root = js_parse(doc, text, size);              // tokens on the heap; js_free when done
//   int enemies = js_get(doc, root, "enemies");
//   for (int i = 0; i < js_len(doc, enemies); i++) {
//       int e = js_at(doc, enemies, i);
//       float x = js_float(doc, js_get(doc, e, "x"), 0.0f);
//   }
//   js_free(doc);
//   pix_free(text);
//
// Every lookup takes a token index and returns one, -1 for "not there"; a -1
// passed on is harmless, so chains need no checks until the end. js_get and
// js_at walk siblings, so loop over an array with js_at only for short ones -
// for long arrays step with js_next, which is O(1).
//
// Used by the glTF loader and the sprite atlas loader. No third-party code.
#ifndef PIX_JSON_HPP
#define PIX_JSON_HPP
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "../core/dtype.hpp"

enum { JS_OBJ = 1, JS_ARR, JS_STR, JS_NUM, JS_BOOL, JS_NULL };

struct js_tok {
    uint8_t type;
    int32_t start;  // for strings, the first char inside the quotes
    int32_t end;    // exclusive
    int32_t count;  // OBJ: member pairs, ARR: elements
    int32_t next;   // token index just past this whole subtree
};

struct js_doc {
    const char* text;  // NUL terminated
    js_tok* toks;
    int count;
    int cap;
    bool owns;         // toks came from js_parse and js_free releases them
};

static inline void js__ws(const char* s, int* p) {
    while (s[*p] == ' ' || s[*p] == '\t' || s[*p] == '\n' || s[*p] == '\r') (*p)++;
}

static inline int js__push(js_doc* d, uint8_t type, int start) {
    if (d->count >= d->cap) return -1;
    int i = d->count++;
    js_tok* t = &d->toks[i];
    t->type = type; t->start = start; t->end = start; t->count = 0; t->next = 0;
    return i;
}

// recursive descent; tokens land in document order so `next` can be filled on the way out
static inline int js__value(js_doc* d, int* p) {
    const char* s = d->text;
    js__ws(s, p);
    char c = s[*p];
    int me;

    if (c == '{') {
        me = js__push(d, JS_OBJ, *p);
        if (me < 0) return -1;
        (*p)++;
        js__ws(s, p);
        if (s[*p] == '}') (*p)++;
        else for (;;) {
            js__ws(s, p);
            if (s[*p] != '"' || js__value(d, p) < 0) return -1; // key
            js__ws(s, p);
            if (s[*p] != ':') return -1;
            (*p)++;
            if (js__value(d, p) < 0) return -1;                 // value
            d->toks[me].count++;
            js__ws(s, p);
            if (s[*p] == ',') { (*p)++; continue; }
            if (s[*p] == '}') { (*p)++; break; }
            return -1;
        }
    } else if (c == '[') {
        me = js__push(d, JS_ARR, *p);
        if (me < 0) return -1;
        (*p)++;
        js__ws(s, p);
        if (s[*p] == ']') (*p)++;
        else for (;;) {
            if (js__value(d, p) < 0) return -1;
            d->toks[me].count++;
            js__ws(s, p);
            if (s[*p] == ',') { (*p)++; continue; }
            if (s[*p] == ']') { (*p)++; break; }
            return -1;
        }
    } else if (c == '"') {
        (*p)++;
        int st = *p;
        while (s[*p] && s[*p] != '"') { if (s[*p] == '\\') (*p)++; (*p)++; }
        me = js__push(d, JS_STR, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
        if (s[*p] == '"') (*p)++;
    } else if (c == 't' || c == 'f' || c == 'n') {
        int st = *p;
        while (s[*p] >= 'a' && s[*p] <= 'z') (*p)++;
        me = js__push(d, c == 'n' ? JS_NULL : JS_BOOL, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
    } else {
        int st = *p;
        while (s[*p] && (s[*p] == '-' || s[*p] == '+' || s[*p] == '.' ||
                         s[*p] == 'e' || s[*p] == 'E' || (s[*p] >= '0' && s[*p] <= '9'))) (*p)++;
        if (*p == st) return -1;
        me = js__push(d, JS_NUM, st);
        if (me < 0) return -1;
        d->toks[me].end = *p;
    }

    d->toks[me].next = d->count;
    return me;
}

// Tokenises `text` (NUL terminated, `len` bytes) and returns the root token,
// or -1 if the document is malformed. Tokens go to the heap (or scoped
// memory with PIX_MEM_SCOPED); release them with js_free.
static inline int js_parse(js_doc& d, const char* text, size_t len, unsigned mem = PIX_MEM_GLOBAL) {
    d.text = text;
    d.count = 0;
    d.cap = (int)(len / 3) + 64;                  // ~3.5 characters per token in practice
    d.toks = pix_new<js_tok>((size_t)d.cap, mem & PIX_MEM_SCOPED);
    d.owns = true;
    if (!d.toks) return -1;
    int pos = 0;
    return js__value(&d, &pos);
}
static inline void js_free(js_doc& d) {
    if (d.owns) pix_free(d.toks);
    d.toks = 0;
    d.count = d.cap = 0;
}

static inline int js_type(const js_doc& d, int tok) { return tok < 0 ? 0 : d.toks[tok].type; }

static inline bool js_is(const js_doc& d, int tok, const char* lit) {
    if (tok < 0 || d.toks[tok].type != JS_STR) return false;
    size_t n = (size_t)(d.toks[tok].end - d.toks[tok].start);
    return strlen(lit) == n && memcmp(d.text + d.toks[tok].start, lit, n) == 0;
}

// the value under `key` in an object
static inline int js_get(const js_doc& d, int obj, const char* key) {
    if (obj < 0 || d.toks[obj].type != JS_OBJ) return -1;
    int k = obj + 1;
    for (int i = 0; i < d.toks[obj].count; i++) {
        int v = d.toks[k].next;
        if (js_is(d, k, key)) return v;
        k = d.toks[v].next;
    }
    return -1;
}

static inline int js_len(const js_doc& d, int arr) {
    return (arr >= 0 && d.toks[arr].type == JS_ARR) ? d.toks[arr].count : 0;
}

// element i of an array
static inline int js_at(const js_doc& d, int arr, int i) {
    if (arr < 0 || d.toks[arr].type != JS_ARR || i < 0 || i >= d.toks[arr].count) return -1;
    int e = arr + 1;
    while (i-- > 0) e = d.toks[e].next;
    return e;
}

// Walking siblings in O(1) each: the first element of an array, or the first
// key of an object (its value is js_value_of(key)); js_next steps from an
// element (or a key's value) to the one after it. -1 past the end.
static inline int js_first(const js_doc& d, int container) {
    return (container >= 0 && d.toks[container].count > 0) ? container + 1 : -1;
}
static inline int js_value_of(const js_doc& d, int key) { return key >= 0 ? d.toks[key].next : -1; }
static inline int js_next(const js_doc& d, int container, int tok) {
    if (tok < 0) return -1;
    int n = d.toks[tok].next;
    return n < d.toks[container].next ? n : -1;
}

static inline float js_float(const js_doc& d, int tok, float def) {
    if (tok < 0 || d.toks[tok].type != JS_NUM) return def;
    return strtof(d.text + d.toks[tok].start, 0);
}

static inline int js_int(const js_doc& d, int tok, int def) {
    if (tok < 0 || d.toks[tok].type != JS_NUM) return def;
    return (int)strtol(d.text + d.toks[tok].start, 0, 10);
}

static inline bool js_bool(const js_doc& d, int tok, bool def) {
    if (tok < 0 || d.toks[tok].type != JS_BOOL) return def;
    return d.text[d.toks[tok].start] == 't';
}

// The raw characters of a string (escapes are not decoded), cut to fit.
static inline void js_str(const js_doc& d, int tok, char* out, size_t cap) {
    if (cap == 0) return;
    out[0] = 0;
    if (tok < 0 || d.toks[tok].type != JS_STR) return;
    size_t n = (size_t)(d.toks[tok].end - d.toks[tok].start);
    if (n >= cap) n = cap - 1;
    memcpy(out, d.text + d.toks[tok].start, n);
    out[n] = 0;
}

#endif // PIX_JSON_HPP
