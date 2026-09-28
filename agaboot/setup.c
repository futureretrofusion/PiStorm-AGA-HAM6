/* setup.c - install / uninstall the AGA-PISTORM hooks and config.txt line.
 *
 * Every change the installer makes is a text edit, and all of them happen
 * here, not in the Installer script - that damn language has no way to edit a
 * file in place. Plain C, stdio only, so the exact code that runs on the Amiga
 * gets unit-tested on the host against copies of real CaffeineOS files - see
 * HOST_TEST at the bottom.
 *
 * What changes, and how uninstall undoes it:
 *
 *   S:WHDLoad-Startup   my ;BEGIN/;END AGA-PISTORM block appended
 *                       (agaboot SANDBOX AUTO: enter the AGA sandbox if the
 *                       toggle is on, do nothing if it is off)
 *   S:WHDLoad-Cleanup   the same kind of block, after the header comments
 *                       (agaboot SANDBOX OFF: back to the real chipset)
 *   S:User-Startup      a block that rebuilds the Chipset menu at boot
 *   EMU68:config.txt    in each board's section - [gpio17=0] for the classic
 *                       PiStorm, [gpio24=1] for the PiStorm16 - the kernel=
 *                       line points at that board's AGA kernel (both at once,
 *                       see setup_config). The old line is kept as a comment
 *                       marked #AGA-PISTORM-WAS:, and uninstall puts it back,
 *                       so undoing it needs no bloody backup file.
 *
 * A block that is already there is REPLACED, so re-running the installer
 * updates it. Each file gets a <name>.pre-aga copy the first time it is
 * changed and never again - that copy stays the original.
 *
 * AmigaDOS scripts must never contain CRLF (return code 20, and a User-Startup
 * that no longer boots), so the blocks are LF and the scripts keep whatever
 * they had. config.txt keeps its own line endings - CaffeineOS ships it CRLF.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAG_BEGIN ";BEGIN AGA-PISTORM"
#define TAG_END   ";END AGA-PISTORM"
#define CFG_MARK  "# AGA-PISTORM: kernel with the emulated AGA chipset. Uninstalling it restores the #AGA-PISTORM-WAS: line(s)."
#define CFG_WAS   "#AGA-PISTORM-WAS: "

/* Each board: the config.txt section whose kernel= line it boots, and the
   kernel it gets - one per board, the protocols differ and a classic kernel
   stops on the logo on a PiStorm16. CaffeineOS pulls every GPIO up and picks
   the kernel by pin level: [gpio17=0] is the classic PiStorm, [gpio24=1] the
   PiStorm16. When a board matches more than one section the later kernel=
   line wins, and [gpio17=0] comes last - so the classic board's line is
   always that one. */
static const char *const board_sect[2]   = { "[gpio17=0]", "[gpio24=1]" };
static const char *const board_kernel[2] = { "kernel=kernel/Emu68-classic-aga.gz",
                                             "kernel=kernel/Emu68-pistorm16-aga.gz" };
static int cfg_board = 0;                     /* the board the installer runs on */
void setup_board(int ps16) { cfg_board = !!ps16; }
const char *setup_section(void) { return board_sect[cfg_board]; }

const char *const setup_whd_on =
    TAG_BEGIN "\n"
    "; AGA-PISTORM: run WHDLoad games on the emulated AGA chipset when the\n"
    "; AGA toggle in the Chipset menu is on. Does nothing when it is off.\n"
    "FAILAT 21\n"
    "If Exists C:agaboot\n"
    "  C:agaboot SANDBOX AUTO >NIL:\n"
    "EndIf\n"
    "FAILAT 10\n"
    TAG_END "\n";

const char *const setup_whd_off =
    TAG_BEGIN "\n"
    "; AGA-PISTORM: back to the real chipset once WHDLoad has restored the OS.\n"
    "FAILAT 21\n"
    "If Exists C:agaboot\n"
    "  C:agaboot SANDBOX OFF >NIL:\n"
    "EndIf\n"
    "FAILAT 10\n"
    TAG_END "\n";

const char *const setup_user_startup =
    TAG_BEGIN "\n"
    "   If Exists C:agaboot\n"
    "      C:agaboot MENU >NIL:\n"
    "   EndIf\n"
    TAG_END "\n";

/* ---- small string helpers ------------------------------------------------ */

/* a line that starts with `tag` at column 0 */
static const char *find_line(const char *text, const char *tag)
{
    size_t n = strlen(tag);
    for (const char *p = text; p && *p; ) {
        if (!strncmp(p, tag, n)) return p;
        p = strchr(p, '\n');
        if (p) p++;
    }
    return NULL;
}

static const char *line_end(const char *p)          /* just past the '\n' */
{
    const char *nl = strchr(p, '\n');
    return nl ? nl + 1 : p + strlen(p);
}

/* my block in `text`, [*b, *e) - returns 0 when there is none */
static int block_span(const char *text, const char **b, const char **e)
{
    const char *s = find_line(text, TAG_BEGIN);
    if (!s) return 0;
    const char *t = find_line(s, TAG_END);
    if (!t) return 0;                               /* half a block: leave it alone */
    *b = s;
    *e = line_end(t);
    return 1;
}

static char *join3(const char *a, size_t na, const char *b, const char *c)
{
    size_t nb = strlen(b), nc = strlen(c);
    char *r = malloc(na + nb + nc + 2);
    if (!r) return NULL;
    memcpy(r, a, na);
    memcpy(r + na, b, nb);
    memcpy(r + na + nb, c, nc + 1);
    return r;
}

/* ---- AmigaDOS scripts ------------------------------------------------------ */

/* Put `block` into `text`. where: 0 = at the end, 1 = after the leading ';'
   header comment lines. Returns a new string, or NULL when `text` already
   holds this block byte for byte (nothing to write). */
char *setup_script_set(const char *text, const char *block, int where)
{
    const char *b, *e;
    if (block_span(text, &b, &e)) {
        if ((size_t)(e - b) == strlen(block) && !strncmp(b, block, e - b)) return NULL;
        return join3(text, b - text, block, e);
    }
    if (where == 0) {
        size_t n = strlen(text);
        if (n && text[n - 1] != '\n') {
            char *t = join3(text, n, "\n", "");
            if (!t) return NULL;
            char *r = join3(t, strlen(t), block, "");
            free(t);
            return r;
        }
        return join3(text, n, block, "");
    }
    const char *p = text;
    while (*p == ';') p = line_end(p);
    return join3(text, p - text, block, p);
}

/* Take my block out. NULL when there is none. */
char *setup_script_remove(const char *text)
{
    const char *b, *e;
    if (!block_span(text, &b, &e)) return NULL;
    return join3(text, b - text, "", e);
}

/* ---- config.txt ------------------------------------------------------------ */

/* ---- config.txt: the overlay block --------------------------------------------
 *
 * Right under the global cmdline= line, one marked block with what this
 * installation needs and the card does not already have:
 *
 *   dtoverlay=pal                 NTSC Agnus: run the system as PAL. First, so
 *                                 the mode is set before unicam sizes its capture
 *   dtoverlay=unicam,boot,smooth  a Framethrower is fitted
 *   gpu_mem=128                   see below
 *   [pi3] dtoverlay=sdhc,unit0=rw   the Amiga may write this partition. Each
 *   [pi4] dtoverlay=emmc,unit0=rw   overlay also switches its SD driver ON, so
 *         force_turbo=1             each Pi gets only its own
 *         core_freq=500
 *   [all]
 *
 * Emu68 1.0.x took sd.unit0=rw on the command line; 1.1 ignores that and knows
 * only the overlay parameter - its SD drivers read their settings from the
 * /emu68 nodes the overlay adds, and without the emmc one it cannot even find
 * the Pi 4's card. Stock CaffeineOS 9317 ships Emu68 1.0.7, my kernel is 1.1:
 * on a fresh 9317 card (2026-09-10) the 1.1 kernel stopped right after its
 * splash until gpu_mem=128 and the [pi4] clock lines went in as well (9317 has
 * gpu_mem=32 and a dynamic core clock, the one timing the classic PiStorm's
 * GPIO bus). With them it boots on 9317's own firmware. Each line goes in only
 * when the card lacks it; lines further down the file win, so a user's own
 * settings still apply. Uninstall removes the block.
 *
 * gpu_mem is the one line outside the block that gets touched: any gpu_mem
 * below 128 that a Pi 4 on either board reads - CaffeineOS has gpu_mem=32 at
 * the top - is commented out as #AGA-PISTORM-WAS:, like an old kernel= line,
 * and uninstall puts it back. Adding mine alone left two gpu_mem lines, where
 * only their order decided which one counted. */
#define BLK_BEGIN "# AGA-PISTORM { added by the AGA-PISTORM installer - its uninstaller removes this block"
#define BLK_END   "# AGA-PISTORM }"

static int line_is(const char *q, size_t qn, const char *s)
{
    size_t n = strlen(s);
    return qn >= n && !strncmp(q, s, n);
}

/* an active (uncommented) line starting with `s`, outside my block */
static int has_active(const char *text, const char *s, const char *also)
{
    int in_blk = 0;
    for (const char *p = text; *p; p = line_end(p)) {
        const char *q = p;
        while (*q == ' ' || *q == '\t') q++;
        size_t qn = line_end(p) - q;
        if (line_is(q, qn, BLK_BEGIN)) { in_blk = 1; continue; }
        if (line_is(q, qn, BLK_END))   { in_blk = 0; continue; }
        if (in_blk || !line_is(q, qn, s)) continue;
        if (!also) return 1;
        const char *e = line_end(p);
        for (const char *r = q; r + strlen(also) <= e; r++)
            if (!strncmp(r, also, strlen(also))) return 1;
    }
    return 0;
}

/* an active line starting with `s`, outside my block, in a part of the file
   a Pi 4 on this board reads: before any filter, [all], [pi4] or the board's
   own section. `min` > 0: the value after `s` must also be at least that. */
static int has_active_pi4(const char *text, const char *s, long min)
{
    int in_blk = 0, applies = 1, found = 0;
    for (const char *p = text; *p; p = line_end(p)) {
        const char *q = p;
        while (*q == ' ' || *q == '\t') q++;
        size_t qn = line_end(p) - q;
        if (line_is(q, qn, BLK_BEGIN)) { in_blk = 1; continue; }
        if (line_is(q, qn, BLK_END))   { in_blk = 0; continue; }
        if (in_blk) continue;
        if (*q == '[') {
            applies = line_is(q, qn, "[all]") || line_is(q, qn, "[pi4]") || line_is(q, qn, board_sect[cfg_board]);
            continue;
        }
        if (!applies || !line_is(q, qn, s)) continue;
        /* the last one wins, as in the firmware */
        found = min <= 0 || strtol(q + strlen(s), NULL, 10) >= min;
    }
    return found;
}

#define GPU_KEY  "gpu_mem="
#define GPU_MIN  128

/* Comment out, as #AGA-PISTORM-WAS:, every active gpu_mem below GPU_MIN that a
   Pi 4 on either board reads: before any filter, [all], [pi4], [cm4] or a
   board section. *kept: an active one of GPU_MIN or more stays where EVERY
   board reads it (before any filter, [all], [pi4]) - then mine is not needed.
   *changed: a line was commented out. Returns a new string. */
static char *gpu_mem_low_out(const char *text, int *kept, int *changed)
{
    size_t n = 0;
    for (const char *s = text; (s = strstr(s, GPU_KEY)); s++) n++;
    char *out = malloc(strlen(text) + n * strlen(CFG_WAS) + 1), *o = out;
    if (!out) return NULL;
    int any = 1, every = 1;                  /* before any filter: both */
    *kept = *changed = 0;
    for (const char *p = text; *p; ) {
        const char *e = line_end(p), *q = p;
        while (*q == ' ' || *q == '\t') q++;
        size_t qn = e - q;
        if (*q == '[') {
            every = line_is(q, qn, "[all]") || line_is(q, qn, "[pi4]");
            any = every || line_is(q, qn, "[cm4]") || line_is(q, qn, board_sect[0]) || line_is(q, qn, board_sect[1]);
        } else if (any && line_is(q, qn, GPU_KEY)) {
            if (strtol(q + strlen(GPU_KEY), NULL, 10) < GPU_MIN) {
                memcpy(o, p, q - p); o += q - p;                /* indentation */
                o += sprintf(o, "%s", CFG_WAS);
                memcpy(o, q, e - q); o += e - q;
                *changed = 1;
                p = e;
                continue;
            }
            if (every) *kept = 1;
        }
        memcpy(o, p, e - p); o += e - p;
        p = e;
    }
    *o = 0;
    return out;
}

/* opts: bit 0 = Framethrower (unicam), bit 1 = NTSC Agnus (pal) */
char *setup_config_block(const char *text, unsigned opts, int install)
{
    const char *eol = strstr(text, "\r\n") ? "\r\n" : "\n";
    /* 1. pull any block from an earlier install, and put back the gpu_mem
          lines it commented out - so what follows starts from the file as the
          user left it, and a second install changes nothing */
    size_t cap = strlen(text) + 1024;
    char *base = malloc(cap), *o = base;
    if (!base) return NULL;
    int in_blk = 0, had = 0;
    for (const char *p = text; *p; ) {
        const char *e = line_end(p);
        const char *q = p;
        while (*q == ' ' || *q == '\t') q++;
        size_t qn = e - q;
        if (line_is(q, qn, BLK_BEGIN)) { in_blk = 1; had = 1; p = e; continue; }
        if (in_blk) { if (line_is(q, qn, BLK_END)) in_blk = 0; p = e; continue; }
        if (line_is(q, qn, CFG_WAS GPU_KEY)) {
            size_t k = strlen(CFG_WAS);
            memcpy(o, p, q - p); o += q - p;                    /* indentation */
            memcpy(o, q + k, e - q - k); o += e - q - k;
            had = 1;
            p = e;
            continue;
        }
        memcpy(o, p, e - p); o += e - p;
        p = e;
    }
    *o = 0;
    if (!install) {
        if (!had) { free(base); return NULL; }
        return base;
    }
    /* 2. what is missing - gpu_mem first, it also edits the rest of the file */
    int gpu_kept, gpu_changed;
    char *g = gpu_mem_low_out(base, &gpu_kept, &gpu_changed);
    free(base);
    if (!g) return NULL;
    base = g;
    char blk[1024];
    int n = sprintf(blk, "%s%s", BLK_BEGIN, eol);
    int lines = 0, filt = 0;
    if ((opts & 2) && !has_active(base, "dtoverlay=pal", NULL))    { n += sprintf(blk + n, "dtoverlay=pal%s", eol); lines++; }
    if ((opts & 1) && !has_active(base, "dtoverlay=unicam", NULL)) { n += sprintf(blk + n, "dtoverlay=unicam,boot,smooth%s", eol); lines++; }
    if (!gpu_kept)                                                 { n += sprintf(blk + n, GPU_KEY "%d%s", GPU_MIN, eol); lines++; }
    if (!has_active(base, "dtoverlay=sdhc", "unit0=rw")) { n += sprintf(blk + n, "[pi3]%sdtoverlay=sdhc,unit0=rw%s", eol, eol); lines++; filt = 1; }
    int emmc = !has_active(base, "dtoverlay=emmc", "unit0=rw");
    int turbo = !has_active_pi4(base, "force_turbo=1", 0);
    int core = !has_active_pi4(base, "core_freq=500", 0);
    if (emmc || turbo || core) {
        n += sprintf(blk + n, "[pi4]%s", eol);
        if (emmc)  { n += sprintf(blk + n, "dtoverlay=emmc,unit0=rw%s", eol); lines++; }
        if (turbo) { n += sprintf(blk + n, "force_turbo=1%s", eol); lines++; }
        if (core)  { n += sprintf(blk + n, "core_freq=500%s", eol); lines++; }
        filt = 1;
    }
    if (filt) n += sprintf(blk + n, "[all]%s", eol);
    sprintf(blk + n, "%s%s", BLK_END, eol);
    if (!lines) {                                    /* the card already has it all */
        if (!strcmp(base, text)) { free(base); return NULL; }
        return base;
    }
    /* 3. after the first cmdline= line of the global section; else before the
          first [filter]; else at the end */
    const char *at = NULL, *first_sect = NULL;
    for (const char *p = base; *p; p = line_end(p)) {
        const char *q = p;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == '[') { first_sect = p; break; }
        if (!strncmp(q, "cmdline=", 8)) { at = line_end(p); break; }
    }
    if (!at) at = first_sect ? first_sect : base + strlen(base);
    char *tail = "";
    size_t head = at - base;
    if (head && base[head - 1] != '\n') tail = (char *)eol;   /* last line had no ending */
    char *out = malloc(strlen(base) + strlen(blk) + 4);
    if (!out) { free(base); return NULL; }
    memcpy(out, base, head);
    strcpy(out + head, tail);
    strcat(out, blk);
    strcat(out, at);
    free(base);
    if (!strcmp(out, text)) { free(out); return NULL; }
    return out;
}

/* Line-by-line rewrite of one section (`sect`, e.g. "[gpio17=0]"). `install`
   1 points kernel= at `kernel`, 0 restores what was there. Returns a new
   string, NULL when nothing changes, and sets *err to 1 when the file has no
   such section at all (not a CaffeineOS config I understand: touch nothing). */
char *setup_config_kernel(const char *text, const char *sect, const char *kernel, int install, int *err)
{
    *err = 0;
    const char *eol = strstr(text, "\r\n") ? "\r\n" : "\n";
    size_t cap = strlen(text) + 4 * (strlen(CFG_MARK) + strlen(kernel) + strlen(CFG_WAS)) + 64;
    char *out = malloc(cap), *o = out;
    if (!out) { *err = 2; return NULL; }
    int in_sect = 0, seen = 0, changed = 0;
    const char *p = text;
    while (*p) {
        const char *e = line_end(p);
        size_t n = e - p, body = n;                   /* body: without the line ending */
        while (body && (p[body - 1] == '\n' || p[body - 1] == '\r')) body--;
        const char *q = p;
        while (q < p + body && (*q == ' ' || *q == '\t')) q++;
        size_t qn = body - (q - p);

        if (qn && *q == '[') {
            in_sect = qn >= strlen(sect) && !strncmp(q, sect, strlen(sect));
            if (in_sect) seen = 1;
            memcpy(o, p, n); o += n;
            if (in_sect && install) {
                /* my two lines go straight under the header */
                o += sprintf(o, "%s%s%s%s", CFG_MARK, eol, kernel, eol);
                changed = 1;
            }
            p = e;
            continue;
        }
        if (in_sect) {
            /* drop my own lines from any earlier install: re-inserted above */
            if (qn == strlen(CFG_MARK) && !strncmp(q, CFG_MARK, qn)) {
                const char *e2 = line_end(e);          /* and the kernel= under it */
                p = e2; changed = 1;
                continue;
            }
            if (install && qn >= 7 && !strncmp(q, "kernel=", 7)) {
                o += sprintf(o, "%s", CFG_WAS);
                memcpy(o, p, n); o += n;
                changed = 1;
                p = e;
                continue;
            }
            if (!install && qn > strlen(CFG_WAS) && !strncmp(q, CFG_WAS, strlen(CFG_WAS))) {
                size_t k = strlen(CFG_WAS);
                memcpy(o, q + k, n - (q - p) - k); o += n - (q - p) - k;
                changed = 1;
                p = e;
                continue;
            }
        }
        memcpy(o, p, n); o += n;
        p = e;
    }
    *o = 0;
    if (!seen) { free(out); *err = 1; return NULL; }
    /* re-installing over my own install produces the identical file */
    if (!changed || !strcmp(out, text)) { free(out); return NULL; }
    return out;
}

/* 1 when `text` has the board's section at all */
int setup_has_section(const char *text, int ps16)
{
    const char *s = board_sect[!!ps16];
    for (const char *p = text; *p; p = line_end(p)) {
        const char *q = p;
        while (*q == ' ' || *q == '\t') q++;
        if (!strncmp(q, s, strlen(s))) return 1;
    }
    return 0;
}

/* Both passes: the kernel lines, then the overlay block. NULL when nothing at
   all changes.

   Install points BOTH boards' sections at their AGA kernels, not only the
   board it runs on. The overlay block is global, and its emmc overlay stops
   Emu68 1.0.x right after the splash: a section left on its stock 1.0.x
   kernel hangs the card on the other board - where the installer can then
   never run to fix it. With both lines, one install serves a card that moves
   between a classic PiStorm and a PiStorm16; on a card that never moves, the
   other section simply never matches. *err is 1 when the running
   board's own section is missing (touch nothing); the other board's is set
   when it is there. Uninstall restores both, whichever board runs it. */
char *setup_config(const char *text, int install, unsigned opts, int *err)
{
    char *a = NULL;
    int found = 0;
    *err = 0;
    for (int i = 0; i < 2; i++) {
        /* install: the running board's section first; uninstall: in order */
        int bd = !install ? i : i == 0 ? cfg_board : !cfg_board;
        int e;                            /* 0 done, 1 no such section, 2 out of memory */
        char *k = setup_config_kernel(a ? a : text, board_sect[bd], install ? board_kernel[bd] : "", install, &e);
        if (e == 2) { free(a); *err = 2; return NULL; }
        if (e == 1 && install && i == 0) { free(a); *err = 1; return NULL; }
        if (!e) found++;
        if (k) { free(a); a = k; }
    }
    if (!found) { free(a); *err = 1; return NULL; }   /* neither section: not a config I understand */
    char *b = setup_config_block(a ? a : text, opts, install);
    if (b) { free(a); return b; }
    return a;
}

/* ---- files ----------------------------------------------------------------- */

char *setup_read(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *b = malloc(n + 1);
    if (b && fread(b, 1, n, f) == (size_t)n) b[n] = 0;
    else { free(b); b = NULL; }
    fclose(f);
    return b;
}

int setup_write(const char *path, const char *text)
{
    FILE *f = fopen(path, "wb");
    if (!f) return 0;
    size_t n = strlen(text);
    int ok = fwrite(text, 1, n, f) == n;
    return fclose(f) == 0 && ok;
}

/* <path>.pre-aga, the first time only: it is always the untouched original */
int setup_backup(const char *path, const char *text)
{
    char bak[256];
    snprintf(bak, sizeof bak, "%s.pre-aga", path);
    FILE *f = fopen(bak, "rb");
    if (f) { fclose(f); return 1; }
    return setup_write(bak, text);
}

#ifdef HOST_TEST
/* gcc -DHOST_TEST -o setuptest setup.c && ./setuptest DIR
   DIR holds copies of WHDLoad-Startup, WHDLoad-Cleanup, User-Startup and
   config.txt; each is installed, re-installed (must not change), and
   uninstalled (must give back the original byte for byte). */
/* equal once trailing whitespace is ignored: a script that ended without a
   newline gets one before my block, and keeps it after uninstall */
static int same_trimmed(const char *a, const char *b)
{
    size_t na = strlen(a), nb = strlen(b);
    while (na && (a[na - 1] == '\n' || a[na - 1] == ' ' || a[na - 1] == '\r')) na--;
    while (nb && (b[nb - 1] == '\n' || b[nb - 1] == ' ' || b[nb - 1] == '\r')) nb--;
    return na == nb && !strncmp(a, b, na);
}

static int check(const char *name, const char *orig, const char *once, const char *twice, const char *back)
{
    int exact = back && !strcmp(back, orig);
    int ok = once && !twice && back && (exact || same_trimmed(back, orig));
    printf("%-18s install %s, reinstall %s, uninstall %s\n", name,
           once ? "changed" : "NO CHANGE", twice ? "CHANGED AGAIN" : "idempotent",
           !back ? "NOTHING" : exact ? "identical" : same_trimmed(back, orig) ? "identical but for the final newline" : "DIFFERS");
    return ok;
}

int main(int argc, char **argv)
{
    char p[512];
    int all = 1;
    const char *files[3] = { "WHDLoad-Startup", "WHDLoad-Cleanup", "User-Startup" };
    const char *blocks[3] = { setup_whd_on, setup_whd_off, setup_user_startup };
    int where[3] = { 0, 1, 0 };
    for (int i = 0; i < 3; i++) {
        snprintf(p, sizeof p, "%s/%s", argv[1], files[i]);
        char *t = setup_read(p);
        if (!t) { printf("%s: missing\n", p); all = 0; continue; }
        char *a = setup_script_set(t, blocks[i], where[i]);
        char *b = a ? setup_script_set(a, blocks[i], where[i]) : NULL;
        char *c = a ? setup_script_remove(a) : NULL;
        all &= check(files[i], t, a, b, c);
        if (a) { snprintf(p, sizeof p, "%s/%s.installed", argv[1], files[i]); setup_write(p, a); }
        if (a && strstr(a, "\r")) { printf("  CR IN SCRIPT\n"); all = 0; }
    }
    static const char *const kern[2] = { "kernel=kernel/Emu68-classic-aga.gz", "kernel=kernel/Emu68-pistorm16-aga.gz" };
    for (int k = 2; k < argc; k++) {
        char *t = setup_read(argv[k]);
        if (!t) { printf("%s: missing\n", argv[k]); all = 0; continue; }
        const char *base = strrchr(argv[k], '/') ? strrchr(argv[k], '/') + 1 : argv[k];
        int err;
        /* What uninstall must give back: the card as it was before any
           install. For a card already carrying a mark of mine - installed,
           or given the overlay block by SD-Setup, which uses the same block
           so that Uninstall removes it too - that is uninstall(t), which
           must itself carry no mark. */
        char *orig = strstr(t, CFG_MARK) || strstr(t, BLK_BEGIN) ? setup_config(t, 0, 0, &err) : NULL;
        const char *ref = orig ? orig : t;
        if (orig && (strstr(orig, CFG_MARK) || strstr(orig, CFG_WAS) || strstr(orig, BLK_BEGIN))) { printf("%s: uninstall left a mark\n", base); all = 0; }
        /* run on each board, with every combination of the two questions */
        for (int ps16 = 0; ps16 < 2; ps16++) {
            setup_board(ps16);
            for (unsigned opts = 0; opts < 4; opts++) {
                char *a = setup_config(t, 1, opts, &err);
                if (err) { printf("%s: no %s (err %d)\n", base, setup_section(), err); all = 0; break; }
                char *b = a ? setup_config(a, 1, opts, &err) : NULL;
                char *c = a ? setup_config(a, 0, 0, &err) : NULL;
                char name[600];
                snprintf(name, sizeof name, "%s [%s ft=%u pal=%u]", base, ps16 ? "ps16" : "classic", opts & 1, (opts >> 1) & 1);
                if (!a && strstr(t, CFG_MARK)) {
                    /* a card this installer already set up, with every line it
                       would add: installing again must change nothing at all */
                    printf("%-18s already installed: install changes nothing (correct)\n", name);
                    continue;
                }
                all &= check(name, ref, a, b, c);
                /* whichever board it ran on, every board section the card has
                   now boots that board's AGA kernel */
                for (int j = 0; j < 2; j++)
                    if (a && setup_has_section(t, j) && !strstr(a, kern[j])) { printf("  %s NOT SET\n", kern[j]); all = 0; }
                /* and no gpu_mem below 128 is left where a Pi 4 reads it */
                if (a) {
                    int kept, low;
                    char *g = gpu_mem_low_out(a, &kept, &low);
                    if (low) { printf("  A gpu_mem BELOW 128 STILL COUNTS\n"); all = 0; }
                    free(g);
                }
                if (a && opts == 3) { snprintf(p, sizeof p, "%s.%s.installed", argv[k], ps16 ? "ps16" : "classic"); setup_write(p, a); }
                if (a && (strstr(a, "\r\n") != NULL) != (strstr(t, "\r\n") != NULL)) { printf("  LINE ENDINGS CHANGED\n"); all = 0; }
                free(a); free(b); free(c);
            }
        }
        /* the card moves: installed on a classic PiStorm, the installer is run
           again on a PiStorm16. The second run must find nothing left to do,
           and ONE uninstall - on either board - must take everything out. */
        setup_board(0);
        char *x = setup_config(t, 1, 3, &err);
        setup_board(1);
        char *y = setup_config(x ? x : t, 1, 3, &err);
        const char *both = y ? y : x ? x : t;
        char *z = setup_config(both, 0, 0, &err);
        int ok = !y && z && strstr(both, kern[0]) && strstr(both, kern[1]) && !strstr(z, CFG_MARK) &&
                 (!strcmp(z, ref) || same_trimmed(z, ref));
        printf("%-18s moved to the other board: %s\n", base,
               ok ? "nothing left to do, one uninstall undoes it all" : y ? "SECOND RUN CHANGED IT" : "FAILED");
        all &= ok;
        setup_board(0);
        free(x); free(y); free(z); free(orig); free(t);
    }
    printf(all ? "ALL OK\n" : "FAILURES\n");
    return all ? 0 : 1;
}
#endif
