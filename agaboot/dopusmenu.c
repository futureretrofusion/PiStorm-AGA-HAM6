/* dopusmenu.c - add/refresh a "Chipset" title in a Directory Opus 5 Magellan
 * user-menu button bank (IFF FORM OPUS, chunks BTNW/BUTN/FUNC).
 *
 * Layout (from the Dopus 5 sources, Library/config_save.c):
 *   BUTN: CFG_BUTN {fpen,bpen,flags(4),count(2),pad(2),pad(4)} followed by
 *         count x (label\0 name\0), name "\1" meaning "same as label"
 *   FUNC: CFG_FUNC {flags(4),flags2(4),pad(4),code(2),qual(2),type(2),
 *         qual_mask(2),qual_same(2)} followed by (string\0 type(2))*
 * A menu title is a BUTN with BUTNF_TITLE, items are plain BUTNs until the
 * next title. Plain C so the host can unit-test it.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUTNF_NEW_FORMAT 0x10
#define BUTNF_TITLE      0x40
#define FUNCF_RUN_ASYNC  0x10
#define FUNCF2_VALID_IX  0x04
#define INST_AMIGADOS    1

static unsigned be32(const unsigned char *p) { return ((unsigned)p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3]; }
static void put32(unsigned char *p, unsigned v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void put16(unsigned char *p, unsigned v) { p[0] = v >> 8; p[1] = v; }

static unsigned char *chunk(unsigned char *o, const char *id, const unsigned char *body, unsigned len)
{
    memcpy(o, id, 4); put32(o + 4, len); memcpy(o + 8, body, len);
    o += 8 + len;
    if (len & 1) *o++ = 0;
    return o;
}

/* BUTN with one function label */
static unsigned char *emit_button(unsigned char *o, unsigned flags, const char *label)
{
    unsigned char b[14 + 64];
    memset(b, 0, sizeof b);
    b[0] = 1; b[1] = 0; put32(b + 2, flags); put16(b + 6, 1);
    unsigned n = 14;
    strcpy((char *)b + n, label); n += strlen(label) + 1;
    b[n++] = 1; b[n++] = 0;                        /* name = label */
    return chunk(o, "BUTN", b, n);
}

/* FUNC: empty (title) or one AmigaDOS instruction */
static unsigned char *emit_func(unsigned char *o, const char *cmd)
{
    unsigned char f[22 + 128];
    memset(f, 0, sizeof f);
    put32(f, cmd ? FUNCF_RUN_ASYNC : 0); put32(f + 4, FUNCF2_VALID_IX);
    put16(f + 12, 0xFFFF); put16(f + 16, 0); put16(f + 18, cmd ? 0 : 0xFFFF);
    unsigned n = 22;
    if (cmd) {
        strcpy((char *)f + n, cmd); n += strlen(cmd) + 1;
        put16(f + n, INST_AMIGADOS); n += 2;
    }
    return chunk(o, "FUNC", f, n);
}

static int butn_is_title(const unsigned char *body, unsigned len, const char *label)
{
    if (len < 14) return 0;
    if (!(be32(body + 2) & BUTNF_TITLE)) return 0;
    return label ? strncmp((const char *)body + 14, label, len - 14) == 0 : 1;
}

/* Returns 0 on success, -1 file error, -2 not an Opus bank. */
int dopus_menu_update(const char *path, const char *title, const char *label_ecs, const char *cmd_ecs,
                      const char *label_aga, const char *cmd_aga,
                      const char *const *extra, int n_extra)   /* extra: label, command pairs */
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    fseek(f, 0, SEEK_END); long size = ftell(f); fseek(f, 0, SEEK_SET);
    unsigned char *in = malloc(size + 16), *out = malloc(size + 1024);
    if (!in || !out || fread(in, 1, size, f) != (size_t)size) { fclose(f); free(in); free(out); return -1; }
    fclose(f);
    if (size < 12 || memcmp(in, "FORM", 4) || memcmp(in + 8, "OPUS", 4)) { free(in); free(out); return -2; }

    unsigned char *o = out + 12;
    long p = 12; int skipping = 0;
    while (p + 8 <= size) {
        const char *id = (const char *)in + p;
        unsigned len = be32(in + p + 4);
        const unsigned char *body = in + p + 8;
        if (p + 8 + len > size) break;
        if (!memcmp(id, "BUTN", 4) && butn_is_title(body, len, NULL))
            skipping = butn_is_title(body, len, title);          /* drop my old block */
        if (!skipping) {
            memcpy(o, in + p, 8 + len); o += 8 + len;
            if (len & 1) *o++ = 0;
        }
        p += 8 + len + (len & 1);
    }
    if (label_ecs) {                               /* NULL: just take my title out */
        o = emit_button(o, BUTNF_NEW_FORMAT | BUTNF_TITLE, title);
        o = emit_func(o, NULL);
        o = emit_button(o, BUTNF_NEW_FORMAT, label_ecs);
        o = emit_func(o, cmd_ecs);
        if (label_aga) {                           /* NULL: a one-item menu */
            o = emit_button(o, BUTNF_NEW_FORMAT, label_aga);
            o = emit_func(o, cmd_aga);
        }
        for (int i = 0; i < n_extra; i++) {
            o = emit_button(o, BUTNF_NEW_FORMAT, extra[2 * i]);
            o = emit_func(o, extra[2 * i + 1]);
        }
    }

    memcpy(out, "FORM", 4); put32(out + 4, (unsigned)(o - out - 8)); memcpy(out + 8, "OPUS", 4);
    f = fopen(path, "wb");
    if (!f) { free(in); free(out); return -1; }
    fwrite(out, 1, o - out, f);
    fclose(f);
    free(in); free(out);
    return 0;
}

#ifdef HOST_TEST
int main(int argc, char **argv)
{
    int remove = argc > 2 && !strcmp(argv[2], "remove");
    int r = dopus_menu_update(argv[1], "Chipset", remove ? NULL : "AGA toggle", "C:agaboot AGAGAMES ASK",
                              NULL, NULL, NULL, 0);
    printf("result %d\n", r);
    return r ? 1 : 0;
}
#endif
