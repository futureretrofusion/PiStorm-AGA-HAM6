/* AGA-PISTORM — video output through the VideoCore HVS (VC6: Pi 4 / CM4).
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * The AGA frame goes out as an HVS plane next to the P96 VideoCore driver's
 * display. Native (chipset) screen active: build my own display list (one
 * scaled RGBA8888 plane) in display-list RAM above the driver's allocator and
 * point DISPLIST1 at it. RTG in front: point DISPLIST1 back at whatever list
 * was live (P96's, or the firmware's). Same trick as vc4gpu.c in the PI
 * Mailbox GPU driver, just moved into Emu68.
 *
 * List format copied from VideoCore.card VC6_SetPanning (scaled plane path).
 */
#include <stdint.h>
#include "aga/chipset.h"
#include "aga_glue.h"

extern void kprintf(const char *fmt, ...);
extern uintptr_t mmu_virt2phys(uintptr_t virt);
extern void arm_flush_cache(uintptr_t addr, uint32_t len);
extern void arm_dcache_invalidate(uintptr_t addr, uint32_t len);

#define LE32(x) __builtin_bswap32(x)            /* Emu68 runs the ARM big-endian */

/* ---- HVS ------------------------------------------------------------------ */
#define ARM_PERIIOBASE      0xF2000000UL
#define HVS_BASE            (ARM_PERIIOBASE + 0x400000)
/* Display-list RAM at +0x4000 on the VC VI (Pi 4/CM4), +0x2000 on the VC IV
   (Pi 3); the plane words differ too. Generation picked once at init off
   CNTFRQ (same test Emu68 uses) and every builder below asks. Layouts
   straight out of VideoCore.card's own vc4.c / vc6.c. */
#define DLIST_BASE_VC6      (HVS_BASE + 0x4000)
#define DLIST_BASE_VC4      (HVS_BASE + 0x2000)
static int       hvs_gen = 6;
static uintptr_t dlist_base = DLIST_BASE_VC6;
#define REG_DISPLIST1       (HVS_BASE + 0x24)
#define REG_DISPSTAT1       (HVS_BASE + 0x58)
#define REG_DISPLACT1       (HVS_BASE + 0x34)   /* the list the HVS is scanning NOW (latched from DISPLIST1 at frame start) */

/* Where my two display lists and the scaling kernel sit in display-list RAM.
   These used to be hardcoded constants somebody measured as free once. They
   were not free, the lying bastards - the machine reported 331 of 331 words
   in use. VideoCore.card puts its video overlay's CLUT at 0x400 (overlay.c
   OVL_CLUT_WORD). An earlier note here claimed the mouse pointer lived there
   too; the driver's source says otherwise - see ptr_probe().

   So now they get picked from a survey of what is actually free with the
   desktop up, and re-picked on every sandbox entry so the answer cannot go
   stale like the old constants did. The values below are only the fallback
   for a machine where no free run is big enough. */
#define LIST_STRIDE         0xA0UL              /* words per list */
#define DL_SPAN             (LIST_STRIDE * 2 + 11)
static uint32_t LIST_A      = 0x400UL;
static uint32_t LIST_B      = 0x4A0UL;
static uint32_t KERNEL_WORD = 0x540UL;

#define CTL_VALID           (1UL << 30)
#define CTL_WORDS(n)        (((n) & 0x3f) << 24)
#define CTL_ALPHA_EXPAND    (1UL << 12)
#define CTL_RGB_EXPAND      (1UL << 11)
#define CTL_FORMAT(n)       ((n) & 0x1f)
#define CTL_PIXEL_ORDER(n)  (((n) & 3) << 13)
#define FMT_RGBA8888        7
#define ORDER_ARGB          2                   /* memory bytes B,G,R,A (little-endian 0xAARRGGBB) */
/* The core writes pixels as native words on a big-endian kernel: memory
   bytes A,R,G,B. Read little-endian by the HVS that is 0xBBGGRRAA, which its
   pixel-order field calls BGRA. Telling the HVS so costs nothing per pixel;
   swapping 230k words a frame on core 3 did. */
#define ORDER_BGRA          1
/* VC IV: no separate alpha word - it rides in POS0 - and the alpha mode sits
   in POS2's top bits. FIXED_NONZERO is what both lists here lean on: opaque
   where the pixel's alpha byte is set (the core writes 0xFF), not drawn
   where the word is zero (the idle plane). */
#define VC4_POS0(x, y)      (((x) & 0xfff) | (((y) & 0xfff) << 12) | (0xffUL << 24))
#define VC4_POS1(w, h)      (((w) & 0xffff) | (((h) & 0xffff) << 16))
#define VC4_POS2(w, h)      (((w) & 0xfff) | (((h) & 0xfff) << 16) | (2UL << 30))
#define VC4_CTL_UNITY       (1UL << 4)
/* VC IV names its pixel orders the other way round from the VI:
   VideoCore.card's vc4.c maps Picasso96's A8R8G8B8 - memory bytes A,R,G,B,
   exactly the core's native words - to HVS_PIXEL_ORDER_RGBA, where the VI
   wants BGRA for the same bytes. Measured 2026-09-14: BGRA on a Pi 3 swaps
   red and blue, Banshee's brown sand came out blue. */
#define ORDER_VC4_ARGB_BYTES 0
#define POS0_X(n)           ((n) & 0x2fff)
#define POS0_Y(n)           (((n) & 0x2fff) << 16)
#define POS1_W(n)           ((n) & 0xffff)
#define POS1_H(n)           (((n) & 0xffff) << 16)
#define POS2_W(n)           ((n) & 0x3fff)
#define POS2_H(n)           (((n) & 0x3fff) << 16)
#define POS2_ALPHA_FIXED    (1UL << 30)
#define POS2_ALPHA(n)       (((n) << 4) & 0xfff0)
#define LIST_END            0x80000000UL
#define PPF_SCALER          0xc0000000UL        /* VideoCore.card default (AGC + bilinear) */
#define PPF_PHASE           128

/* Mitchell-Netravali (B = C = 1/3) kernel, VideoCore.card compute_scaling_kernel layout */
static const uint32_t kernel_words[11] = {
    0x07f40000, 0x07e7f1fa, 0x00600ffd, 0x01fcb035, 0x036988a4, 0x0001c6e3,
    0x036988a4, 0x01fcb035, 0x00600ffd, 0x07e7f1fa, 0x07f40000
};

static inline uint32_t reg_rd(uintptr_t a)          { return LE32(*(volatile uint32_t *)a); }
static inline void     reg_wr(uintptr_t a, uint32_t v) { *(volatile uint32_t *)a = LE32(v); }
static inline uint32_t dl_rd(uint32_t w)            { return reg_rd(dlist_base + w * 4); }
static inline void     dl_wr(uint32_t w, uint32_t v) { reg_wr(dlist_base + w * 4, v); }

/* ---- borrowing display-list memory ----------------------------------------
   LIST_A, LIST_B and KERNEL_WORD are addresses in HVS display-list RAM that
   happened to be free when somebody measured them. Not mine to keep - other
   code writes that RAM too (the video overlay's CLUT sits at 0x400). So
   save a copy before the first write and put it back when I let the screen
   go, wherever the survey below put me.

   First version of this assumed the RTG driver's mouse pointer plane lived
   in these words. VideoCore.card's source (vc6.c VC6_SetPanning) builds the
   pointer plane and its palette inside the driver's own buddy block below
   0x300, image in Amiga fast RAM - and saving/restoring these words never
   brought the pointer back. No shit it didn't, the pointer was never in
   there. */
#define DL_BORROW_WORDS  DL_SPAN
static uint32_t dl_backup[DL_BORROW_WORDS];
static uint32_t dl_borrow_first;
static int      dl_borrowed;
uint32_t aga_dl_inuse;      /* words that were non-zero when I took them */
uint32_t aga_dl_free_at;    /* start of the longest free run in the dlist RAM */
uint32_t aga_dl_free_len;   /* and how long it is - both published to agastat */

static void dl_survey(void);    /* defined below; called from dl_borrow */

static void dl_borrow(void)
{
    if (dl_borrowed) return;

    /* What is free NOW, desktop up, driver settled - not what was free when
       somebody measured it once. */
    dl_survey();
    if (aga_dl_free_len >= DL_SPAN) {
        LIST_A      = aga_dl_free_at;
        LIST_B      = LIST_A + LIST_STRIDE;
        KERNEL_WORD = LIST_B + LIST_STRIDE;
        /* the scaling kernel is written at init to the old address, so put a
           copy at the new one before any list points at it */
        for (int i = 0; i < 11; i++) dl_wr(KERNEL_WORD + i, kernel_words[i]);
    }

    dl_borrow_first = LIST_A;
    uint32_t used = 0;
    for (uint32_t i = 0; i < DL_BORROW_WORDS; i++) {
        dl_backup[i] = dl_rd(dl_borrow_first + i);
        if (dl_backup[i]) used++;
    }
    aga_dl_inuse = used;        /* 0 = found somewhere genuinely spare */
    dl_borrowed = 1;
}

/* Where in display-list RAM is there actually room?

   dlist_words_inuse came back 331 of 331 on the machine: not one word at
   LIST_A/LIST_B/KERNEL_WORD was free. They sit on overlay.c's OVL_CLUT_WORD,
   the video overlay's palette. That is NOT where the mouse pointer lives, so
   moving off it stops a real trespass but is not the pointer fix; the PTR_*
   counters exist to find what is.

   This hunts the longest run of zero words in the 4096-word (16 KB) RAM and
   reports it, so the next build can put my two lists and the scaling kernel
   somewhere the driver is not - measured, not guessed-and-gone-stale. Crap
   way to find room, but it works. */
#define DL_RAM_WORDS  4096

/* Never below 0x500. Per VideoCore.card's own source everything under it is
   spoken for even where it reads as zero: buddyalloc.c owns words 0-0x2FF
   (live lists, pointer plane and palette), vc6.c keeps Picasso96's palette
   and the Unicam list at 0x300, overlay.c puts the video overlay's
   256-entry CLUT at 0x400. A mostly black palette is one long run of zeros
   and the survey must not fall for that shit. */
#define DL_SURVEY_FLOOR 0x500

static void dl_survey(void)
{
    uint32_t best_at = 0, best_len = 0, run_at = 0, run = 0;
    for (uint32_t i = DL_SURVEY_FLOOR; i < DL_RAM_WORDS; i++) {
        if (dl_rd(i) == 0) {
            if (!run) run_at = i;
            run++;
            if (run > best_len) { best_len = run; best_at = run_at; }
        } else {
            run = 0;
        }
    }
    /* dl_return() zeroes my lists before the next survey, so a region I used
       last time is free to be picked again. */
    aga_dl_free_at = best_at;
    aga_dl_free_len = best_len;
}

static void dl_return(void)
{
    if (!dl_borrowed) return;
    for (uint32_t i = 0; i < DL_BORROW_WORDS; i++)
        dl_wr(dl_borrow_first + i, dl_backup[i]);
    dl_borrowed = 0;
}

/* ---- the RTG mouse pointer, read out of the driver's live list -------------
   Diagnostic only, nothing here writes. Walks a display list the way
   VideoCore.card vc6.c VC6_SetPanning lays it out, unity and scaled path
   alike: screen plane, then pointer plane (CTL, POS0 at +1, PTR0 at +6, 18
   words), optional video overlay plane, then 0x80000000, one zero word
   (colour 0, transparent) and the pointer's three colours. Fills six
   counters from `base`: LIST PLANES CTL POS0 IMAGE PAL. */
static void ptr_probe(uint32_t list, int base)
{
    if (hvs_gen != 6) return;            /* VC6 layout only; diagnostic */
    uint32_t planes = 0, ctl2 = 0, pos0 = 0, image = 0, pal = 0;
    uint32_t o = list;
    for (int n = 0; n < 8 && o + 6 < DL_RAM_WORDS; n++) {
        uint32_t ctl = dl_rd(o);
        if (ctl == LIST_END) {
            pal = dl_rd(o + 2);             /* terminator, colour 0, colour 1 */
            break;
        }
        uint32_t words = (ctl >> 24) & 0x3f;
        if (!(ctl & CTL_VALID) || words == 0) break;
        if (++planes == 2) {                /* plane 2 is the pointer */
            ctl2  = ctl;
            pos0  = dl_rd(o + 1);
            image = dl_rd(o + 6);
        }
        o += words;
    }
    aga_diag_set(base + 0, list);
    aga_diag_set(base + 1, planes);
    aga_diag_set(base + 2, ctl2);
    aga_diag_set(base + 3, pos0);
    aga_diag_set(base + 4, image);
    aga_diag_set(base + 5, pal);
}

/* ---- mailbox (only used at init, on core 0) ------------------------------- */
#define MBOX_BASE           (ARM_PERIIOBASE + 0xB880)
#define MBOX_READ           (*(volatile uint32_t *)(MBOX_BASE + 0x00))
#define MBOX_STATUS         (*(volatile uint32_t *)(MBOX_BASE + 0x18))
#define MBOX_WRITE          (*(volatile uint32_t *)(MBOX_BASE + 0x20))
#define MBOX_FULL           0x80000000u
#define MBOX_EMPTY          0x40000000u
static uint32_t req[32] __attribute__((aligned(16)));

static void mbox_call(void)
{
    uintptr_t phys = mmu_virt2phys((uintptr_t)req);
    arm_flush_cache((uintptr_t)req, sizeof req);
    while (LE32(MBOX_STATUS) & MBOX_FULL) ;
    MBOX_WRITE = LE32((uint32_t)((phys & ~0xFu) | 8));
    for (;;) {
        while (LE32(MBOX_STATUS) & MBOX_EMPTY) ;
        uint32_t r = LE32(MBOX_READ);
        if ((r & 0xF) == 8) break;
    }
    arm_dcache_invalidate((uintptr_t)req, sizeof req);
}

/* Firmware's view of the SoC: temperature (millidegrees) and the
   GET_THROTTLED bits - 0 under-voltage now, 1 frequency capped now, 2
   throttled now, 3 soft temperature limit now; bits 16-19 the same since
   boot. Nothing to do with the video path, so it works on a Pi 3 too.
   Small mercy. */
int aga_soc_status(uint32_t *temp_mc, uint32_t *throttled)
{
    int c = 1;
    req[c++] = 0;
    req[c++] = LE32(0x30006); req[c++] = LE32(8); req[c++] = 0;   /* GET_TEMPERATURE id 0 */
    int tpos = c; req[c++] = 0; req[c++] = 0;
    req[c++] = LE32(0x30046); req[c++] = LE32(4); req[c++] = 0;   /* GET_THROTTLED, 0 = do not clear */
    int fpos = c; req[c++] = 0;
    req[c++] = 0;
    req[0] = LE32(c << 2);
    mbox_call();
    if (LE32(req[1]) != 0x80000000u) return -1;
    *temp_mc = LE32(req[tpos + 1]);
    *throttled = LE32(req[fpos]);
    return 0;
}

/* Firmware display blanking (tag 0x40003), the call VideoCore.card makes for
   Picasso96's SetSwitch. Pi 4 firmware seems to ignore the damn thing; Pi 3
   honours it, and if the unblank never comes the desktop gets drawn into a
   dark output. Sent on the way out on a VC IV. */
static int fw_blank(int blank)
{
    int c = 1;
    req[c++] = 0;
    req[c++] = LE32(0x40003); req[c++] = LE32(4); req[c++] = 0;
    int vpos = c; req[c++] = LE32((uint32_t)(blank ? 1 : 0));
    req[c++] = 0;
    req[0] = LE32(c << 2);
    mbox_call();
    return (LE32(req[1]) == 0x80000000u) ? (int)(LE32(req[vpos]) & 1) : -1;
}

/* ---- state ---------------------------------------------------------------- */
static uint8_t  *fb_va;             /* two frames of AGA_OUT_W x AGA_OUT_H x 4, uncached */
static uintptr_t fb_phys;
static int       fb_page;
static uint32_t  disp_w, disp_h;
static uint32_t  dst_w, dst_h, dst_x, dst_y;
static uint32_t  scale_x, scale_y;  /* 16.16, src/dst */
static uint32_t  saved_list;        /* DISPLIST1 as found before I took over */

/* Dumps display-list RAM into the diagnostic log: every run of non-zero
   words as start:length (hex words), plus DISPLIST1. On the Pi 3 the survey
   found no free run at all (2026-09-14) and my lists fell back to the
   compile-time words; this says what the RAM really holds - at boot, at the
   first frame and after the hand-back, no serial console needed. */
static void hex_put(char *d, uint32_t v, int digits)
{
    for (int i = digits - 1; i >= 0; i--) { d[i] = "0123456789abcdef"[v & 15]; v >>= 4; }
}
static void dl_map_log(const char *tag)
{
    char line[120];
    int n = 0;
    while (*tag && n < 24) line[n++] = *tag++;
    line[n++] = ' '; line[n++] = 'L';
    hex_put(line + n, reg_rd(REG_DISPLIST1), 4); n += 4; line[n++] = ' ';
    uint32_t run_at = 0, run = 0; int runs = 0;
    for (uint32_t i = 0; i <= DL_RAM_WORDS && runs < 12; i++) {
        uint32_t v = (i < DL_RAM_WORDS) ? dl_rd(i) : 0;
        if (v) { if (!run) run_at = i; run++; }
        else if (run) {
            if (n + 9 >= (int)sizeof line) break;
            hex_put(line + n, run_at, 3); n += 3; line[n++] = ':';
            hex_put(line + n, run, 3); n += 3; line[n++] = ' ';
            run = 0; runs++;
        }
    }
    line[n] = 0;
    aga_diag_log(line);
}
/* eight words of a list, for the same log */
static void dl_words_log(const char *tag, uint32_t at)
{
    char line[120];
    int n = 0;
    while (*tag && n < 24) line[n++] = *tag++;
    line[n++] = ' '; hex_put(line + n, at, 4); n += 4; line[n++] = ':';
    for (int i = 0; i < 8; i++) { line[n++] = ' '; hex_put(line + n, dl_rd(at + i), 8); n += 8; }
    line[n] = 0;
    aga_diag_log(line);
}

/* The watch. hide() arms it; the housekeeper core calls aga_hvs_service()
   every loop and it samples once per 20 ms for three seconds: does DISPLIST1
   still hold what I wrote, what does DISPLACT1 say the HVS is really
   scanning, does the line counter move between samples. Last sample dumps
   the map and the live list's first words. */
static volatile uint64_t watch_next, watch_end;
static uint32_t watch_list_prev, watch_stat_prev;
static int      watch_samples;
static inline uint64_t cnt_now(void) { uint64_t t; __asm__ volatile("mrs %0, CNTVCT_EL0" : "=r"(t)); return t; }
static void watch_arm(void)
{
    uint64_t f; __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(f));
    uint64_t now = cnt_now();
    watch_samples = 0;
    watch_list_prev = reg_rd(REG_DISPLIST1);
    watch_stat_prev = reg_rd(REG_DISPSTAT1);
    aga_diag_set(AGA_D_WATCH_SAMPLES, 0);
    aga_diag_set(AGA_D_WATCH_LIST_CHANGES, 0);
    aga_diag_set(AGA_D_WATCH_STAT_MOVES, 0);
    aga_diag_set(AGA_D_WATCH_LACT_FIRST, reg_rd(REG_DISPLACT1));
    watch_end = now + f * 3;
    __atomic_store_n(&watch_next, now + f / 50, __ATOMIC_RELEASE);
}
/* Wait for the HVS to latch DISPLIST1 into DISPLACT1 - frame start only - so
   the words of the list it was scanning can be rewritten. Bounded, because a
   stalled channel must not stall me too. Returns 1 when DISPLACT1 reads the
   list asked for. */
static int wait_latched(uint32_t list, uint32_t timeout_us)
{
    uint64_t f; __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(f));
    uint64_t end = cnt_now() + (f * timeout_us) / 1000000u;
    do {
        if (reg_rd(REG_DISPLACT1) == list) return 1;
    } while (cnt_now() < end);
    return reg_rd(REG_DISPLACT1) == list;
}
void aga_hvs_service(void)
{
    if (!(AGA_PI3_BUILD && AGA_PROBES_ON)) return;   /* Pi 3 test kernels only */
    uint64_t nx = __atomic_load_n(&watch_next, __ATOMIC_ACQUIRE);
    if (!nx) return;
    uint64_t now = cnt_now();
    if (now < nx) return;
    uint64_t f; __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(f));
    uint32_t list = reg_rd(REG_DISPLIST1);
    uint32_t lact = reg_rd(REG_DISPLACT1);
    uint32_t stat = reg_rd(REG_DISPSTAT1);
    if (list != watch_list_prev) aga_diag_inc(AGA_D_WATCH_LIST_CHANGES);
    if ((stat & 0xfff) != (watch_stat_prev & 0xfff)) aga_diag_inc(AGA_D_WATCH_STAT_MOVES);
    watch_list_prev = list; watch_stat_prev = stat;
    watch_samples++;
    aga_diag_set(AGA_D_WATCH_SAMPLES, (uint32_t)watch_samples);
    aga_diag_set(AGA_D_WATCH_LIST_LAST, list);
    aga_diag_set(AGA_D_WATCH_LACT_LAST, lact);
    aga_diag_set(AGA_D_WATCH_STAT_LAST, stat);
    aga_diag_set(AGA_D_WATCH_CTL_LAST, dl_rd(lact));
    aga_diag_set(AGA_D_WATCH_PTR0_LAST, dl_rd(lact + 4));
    if (now >= watch_end) {
        __atomic_store_n(&watch_next, (uint64_t)0, __ATOMIC_RELEASE);
        dl_map_log("[AGA] dlist@+3s");
        dl_words_log("[AGA] DISPLACT1 words", lact);
        if (list != lact) dl_words_log("[AGA] DISPLIST1 words", list);
    } else {
        __atomic_store_n(&watch_next, now + f / 50, __ATOMIC_RELEASE);
    }
}
static uint32_t  cur_list;          /* LIST_A/LIST_B when my list is live, else 0 */
static uint32_t  ptr0_word;         /* dlist word index of my plane's PTR0 */
static int       video_ok;

#define FB_BYTES   (AGA_OUT_W * AGA_OUT_H * 4)

int aga_video_init_at(uint8_t *fbmem_va, uintptr_t fbmem_phys)
{
    fb_va = fbmem_va;
    fb_phys = fbmem_phys;

    /* Which HVS this is. Emu68 tells a Pi 3 from a Pi 4 by generic timer
       frequency - 19.2 MHz against 54 MHz - and so do I. A VC6 list written
       into a VC4 scaler corrupts it (the first Pi 3 boot of this kernel froze
       on the damn Emu68 logo that way), so generation decides the RAM offset
       and every plane word from here on. */
    {
        uint64_t cntfrq;
        __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(cntfrq));
        if (cntfrq <= 20000000) { hvs_gen = 4; dlist_base = DLIST_BASE_VC4; }
        else                    { hvs_gen = 6; dlist_base = DLIST_BASE_VC6; }
        kprintf("[AGA] HVS: VideoCore %s, display lists at %p\n", hvs_gen == 4 ? "IV (Pi 3)" : "VI (Pi 4)", (void *)dlist_base);
    }
    if (hvs_gen == 4 && !AGA_PI3_BUILD) {
        /* Pi 4 kernel on a Pi 3. Its lists are VC6 lists, which corrupt a
           VC4 scaler, so no AGA picture: games still run, the AGA toggle
           tells the user to reinstall for the Pi 3 kernel. */
        aga_diag_log("[AGA] HVS: Pi 4 kernel on a Pi 3 - no AGA picture, reinstall for the Pi 3");
        return -1;
    }
    if (AGA_PI3_BUILD && AGA_PROBES_ON) dl_map_log("[AGA] dlist@boot");

    /* physical display size */
    int c = 1;
    req[c++] = 0;
    req[c++] = LE32(0x40003); req[c++] = LE32(8); req[c++] = 0;
    int pos = c;
    req[c++] = 0; req[c++] = 0;
    req[c++] = 0;
    req[0] = LE32(c << 2);
    mbox_call();
    disp_w = LE32(req[pos]);
    disp_h = LE32(req[pos + 1]);
    if (disp_w < 320 || disp_h < 200 || disp_w > 4096 || disp_h > 4096) {
        kprintf("[AGA] HVS: implausible display size %dx%d, video disabled\n", disp_w, disp_h);
        return -1;
    }

    /* fit 800 x 286 (line-doubled height 572) into the display, keep aspect */
    uint32_t src_w = AGA_OUT_W, src_h2 = AGA_OUT_H * 2;
    uint32_t sx = (0x10000u * src_w) / disp_w;       /* src/dst */
    uint32_t sy = (0x10000u * src_h2) / disp_h;
    uint32_t s = sx > sy ? sx : sy;                  /* larger ratio = smaller picture, fits */
    dst_w = (0x10000u * src_w) / s;
    dst_h = (0x10000u * src_h2) / s;
    if (dst_w > disp_w) dst_w = disp_w;
    if (dst_h > disp_h) dst_h = disp_h;
    dst_x = (disp_w - dst_w) / 2;
    dst_y = (disp_h - dst_h) / 2;
    scale_x = (0x10000u * src_w) / dst_w;
    scale_y = (0x10000u * AGA_OUT_H) / dst_h;         /* source is not line-doubled */

    for (int i = 0; i < 11; i++) dl_wr(KERNEL_WORD + i, kernel_words[i]);

    saved_list = reg_rd(REG_DISPLIST1);
    cur_list = 0;
    video_ok = 1;
    kprintf("[AGA] HVS: display %dx%d, AGA plane %dx%d at %d,%d (scale %08x/%08x), fb %p\n",
            disp_w, disp_h, dst_w, dst_h, dst_x, dst_y, scale_x, scale_y, (void *)fb_phys);
    return 0;
}

int aga_video_init(void)
{
    return video_ok ? 0 : -1;
}

/* Build my display list in the block that is not live: one scaled plane. */
static void build_list(uint32_t block, uintptr_t frame_phys)
{
    uint32_t o = block + 1;
    if (hvs_gen == 4) {
        dl_wr(o++, VC4_POS0(dst_x, dst_y));                           /* POS0: x, y, alpha */
        dl_wr(o++, VC4_POS1(dst_w, dst_h));                           /* scaled destination size */
        dl_wr(o++, VC4_POS2(AGA_OUT_W, AGA_OUT_H));                   /* source size, alpha mode */
    } else {
        dl_wr(o++, POS0_X(dst_x) | POS0_Y(dst_y));                       /* POS0 */
        dl_wr(o++, POS2_ALPHA_FIXED | POS2_ALPHA(0xfff));                /* POS1 (alpha, VC6) */
        dl_wr(o++, POS1_H(dst_h) | POS1_W(dst_w));                       /* scaled destination size */
        dl_wr(o++, POS2_H(AGA_OUT_H) | POS2_W(AGA_OUT_W));               /* source size */
    }
    dl_wr(o++, 0xdeadbeefUL);                                        /* POS3 ctx (HVS) */
    ptr0_word = o;
    dl_wr(o++, 0xc0000000UL | (uint32_t)frame_phys);                 /* PTR0 */
    dl_wr(o++, 0xdeadbeefUL);                                        /* PTRCTX0 (HVS) */
    dl_wr(o++, AGA_OUT_W * 4);                                       /* pitch */
    dl_wr(o++, 0);                                                   /* LBM base: only plane here */
    dl_wr(o++, (scale_x << 8) | PPF_SCALER | PPF_PHASE);             /* PPF0 horizontal */
    dl_wr(o++, (scale_y << 8) | PPF_SCALER | PPF_PHASE);             /* PPF1 vertical */
    dl_wr(o++, 0);                                                   /* scratch (HVS) */
    dl_wr(o++, KERNEL_WORD);                                         /* kernel pointers */
    dl_wr(o++, KERNEL_WORD);
    dl_wr(o++, KERNEL_WORD);
    dl_wr(o++, KERNEL_WORD);
    dl_wr(block, CTL_VALID | CTL_WORDS(o - block) | CTL_ALPHA_EXPAND | CTL_RGB_EXPAND
               | CTL_FORMAT(FMT_RGBA8888) | CTL_PIXEL_ORDER(hvs_gen == 4 ? ORDER_VC4_ARGB_BYTES : ORDER_BGRA));
    dl_wr(o++, LIST_END);
}

/* Length of a plane list in words (0 if it does not look like one). */
static uint32_t list_len(uint32_t start)
{
    uint32_t n = 0;
    for (;;) {
        uint32_t ctl = dl_rd(start + n);
        if (ctl == LIST_END) return n;
        if (!(ctl & CTL_VALID)) return 0;
        uint32_t words = (ctl >> 24) & 0x3f;
        if (words == 0 || n + words > 0x70) return 0;
        n += words;
    }
}

/* Geometry of a list's first plane (the RTG screen): where the screen's
   pixel (0,0) lands on the display, and its scale (16.16 display px per
   screen px). */
static void base_plane_geometry(uint32_t start, int *ox, int *oy, uint32_t *sx, uint32_t *sy, uint32_t *modew)
{
    uint32_t ctl = dl_rd(start);
    uint32_t pos0 = dl_rd(start + 1);
    *sx = *sy = 0x10000; *modew = disp_w;
    if (hvs_gen == 4) {
        /* VC4: POS0 = x | y<<12 | alpha; unity lists go POS0, POS2, scaled ones
           POS0, POS1(dst), POS2(src), and the unity flag is bit 4 */
        *ox = pos0 & 0xfff; *oy = (pos0 >> 12) & 0xfff;
        if (ctl & VC4_CTL_UNITY) {
            *modew = dl_rd(start + 2) & 0xfff;
        } else {
            uint32_t dst = dl_rd(start + 2), src = dl_rd(start + 3);
            uint32_t dw = dst & 0xffff, dh = (dst >> 16) & 0xffff;
            uint32_t sw = src & 0xfff, sh = (src >> 16) & 0xfff;
            if (sw && sh && dw && dh) { *sx = (dw << 16) / sw; *sy = (dh << 16) / sh; *modew = sw; }
        }
        return;
    }
    *ox = pos0 & 0x2fff; *oy = (pos0 >> 16) & 0x2fff;
    if (ctl & (1UL << 15)) {                         /* UNITY: POS0, POS1(alpha), POS2(size) */
        uint32_t pos2 = dl_rd(start + 3);
        *modew = pos2 & 0x3fff;
    } else {                                         /* scaled: POS0, POS1(alpha), POS1(dst), POS2(src) */
        uint32_t dst = dl_rd(start + 3), src = dl_rd(start + 4);
        uint32_t dw = dst & 0xffff, dh = (dst >> 16) & 0xffff;
        uint32_t sw = src & 0x3fff, sh = (src >> 16) & 0x3fff;
        if (sw && sh && dw && dh) { *sx = (dw << 16) / sw; *sy = (dh << 16) / sh; *modew = sw; }
    }
}

/* Windowed: copy the live RTG list, insert my plane after its first plane
   (so the mouse pointer stays on top), scaled into the window rectangle. */
static void build_windowed_list(uint32_t block, uint32_t base, uintptr_t frame_phys, int wx, int wy, int ww, int wh)
{
    uint32_t n = list_len(base);
    int ox, oy; uint32_t sx, sy, modew;
    base_plane_geometry(base, &ox, &oy, &sx, &sy, &modew);
    uint32_t first = (dl_rd(base) >> 24) & 0x3f;
    /* window rectangle in display pixels */
    int dx = ox + (int)(((int64_t)wx * sx) >> 16);
    int dy = oy + (int)(((int64_t)wy * sy) >> 16);
    int dw = (int)(((int64_t)ww * sx) >> 16);
    int dh = (int)(((int64_t)wh * sy) >> 16);
    if (dw < 16) dw = 16;
    if (dh < 16) dh = 16;
    if (dx < 0) dx = 0;
    if (dy < 0) dy = 0;
    if (dx + dw > (int)disp_w) dw = (int)disp_w - dx;
    if (dy + dh > (int)disp_h) dh = (int)disp_h - dy;
    if (dw < 16 || dh < 16) { build_list(block, frame_phys); return; }
    uint32_t scx = (0x10000u * AGA_OUT_W) / (uint32_t)dw;
    uint32_t scy = (0x10000u * AGA_OUT_H) / (uint32_t)dh;

    uint32_t o = block;
    for (uint32_t i = 0; i < first; i++) dl_wr(o++, dl_rd(base + i));      /* RTG screen plane */
    uint32_t plane = o++;
    if (hvs_gen == 4) {
        dl_wr(o++, VC4_POS0(dx, dy));
        dl_wr(o++, VC4_POS1(dw, dh));
        dl_wr(o++, VC4_POS2(AGA_OUT_W, AGA_OUT_H));
    } else {
        dl_wr(o++, POS0_X(dx) | POS0_Y(dy));
        dl_wr(o++, POS2_ALPHA_FIXED | POS2_ALPHA(0xfff));
        dl_wr(o++, POS1_H(dh) | POS1_W(dw));
        dl_wr(o++, POS2_H(AGA_OUT_H) | POS2_W(AGA_OUT_W));
    }
    dl_wr(o++, 0xdeadbeefUL);
    ptr0_word = o;
    dl_wr(o++, 0xc0000000UL | (uint32_t)frame_phys);
    dl_wr(o++, 0xdeadbeefUL);
    dl_wr(o++, AGA_OUT_W * 4);
    dl_wr(o++, 8 * modew + 1024);                       /* LBM: after the RTG plane and its mouse pointer */
    dl_wr(o++, (scx << 8) | PPF_SCALER | PPF_PHASE);
    dl_wr(o++, (scy << 8) | PPF_SCALER | PPF_PHASE);
    dl_wr(o++, 0);
    dl_wr(o++, KERNEL_WORD); dl_wr(o++, KERNEL_WORD); dl_wr(o++, KERNEL_WORD); dl_wr(o++, KERNEL_WORD);
    dl_wr(plane, CTL_VALID | CTL_WORDS(o - plane) | CTL_ALPHA_EXPAND | CTL_RGB_EXPAND
               | CTL_FORMAT(FMT_RGBA8888) | CTL_PIXEL_ORDER(hvs_gen == 4 ? ORDER_VC4_ARGB_BYTES : ORDER_BGRA));
    for (uint32_t i = first; i < n; i++) dl_wr(o++, dl_rd(base + i));     /* mouse pointer, overlays */
    dl_wr(o++, LIST_END);
}

static int win_mode, win_x, win_y, win_w, win_h;
static int cur_is_windowed, cur_x, cur_y, cur_w, cur_h;

static void show(uintptr_t frame_phys)
{
    uint32_t live = reg_rd(REG_DISPLIST1);
    int same_geometry = cur_is_windowed == win_mode && cur_x == win_x && cur_y == win_y && cur_w == win_w && cur_h == win_h;
    if (cur_list && live == cur_list && same_geometry) {
        dl_wr(ptr0_word, 0xc0000000UL | (uint32_t)frame_phys);       /* page flip only */
        return;
    }
    /* someone else's list: remember it so I can hand it back / build on */
    if (live != LIST_A && live != LIST_B) saved_list = live;
    if (hvs_gen == 4) {                       /* the VC6 pointer probe's counters are free here */
        aga_diag_set(AGA_D_PTR_IN_LIST, saved_list);
        aga_diag_set(AGA_D_PTR_IN_PLANES, list_len(saved_list));
        aga_diag_set(AGA_D_PTR_IN_CTL, dl_rd(saved_list));
        aga_diag_set(AGA_D_PTR_IN_POS0, dl_rd(saved_list + 1));
        aga_diag_set(AGA_D_PTR_IN_IMAGE, dl_rd(saved_list + 4));
    }
    if (AGA_PROBES_ON && !cur_list) ptr_probe(saved_list, AGA_D_PTR_IN_LIST);   /* the pointer as I found it */
    if (AGA_PI3_BUILD && AGA_PROBES_ON && !dl_borrowed) dl_map_log("[AGA] dlist@show");
    dl_borrow();            /* before the first write into words I do not own */
    uint32_t block = (cur_list == LIST_A) ? LIST_B : LIST_A;
    /* two rebuilds inside one display frame would write into the list the
       HVS is still scanning; wait for it to move to the current one first */
    if (AGA_PI3_BUILD && cur_list && reg_rd(REG_DISPLACT1) == block) wait_latched(cur_list, 25000);
    if (win_mode && list_len(saved_list) > 0)
        build_windowed_list(block, saved_list, frame_phys, win_x, win_y, win_w, win_h);
    else
        build_list(block, frame_phys);
    reg_wr(REG_DISPLIST1, block);
    cur_list = block;
    cur_is_windowed = win_mode; cur_x = win_x; cur_y = win_y; cur_w = win_w; cur_h = win_h;
}

static void hide(void)
{
    if (!cur_list) return;
    uint32_t live = reg_rd(REG_DISPLIST1);
    /* Hand the driver's list back whenever mine was the one shown. This used
       to be conditional on the register reading back my address, and a Pi 3
       stayed black as hell after the first game it ever displayed
       (2026-09-14) - a read that does not match must not leave the HVS on a
       plane whose frame is about to stop. Only a live list that is neither
       of mine (the driver re-panned in the meantime) gets left alone. */
    if (live == cur_list || live == LIST_A || live == LIST_B) reg_wr(REG_DISPLIST1, saved_list);
    aga_diag_log(live == cur_list ? "[AGA] HVS: handed the driver's list back" : "[AGA] HVS: live list was not ours at hide");
    if (hvs_gen == 4) {
        aga_diag_set(AGA_D_PTR_OUT_LIST, live);
        aga_diag_set(AGA_D_PTR_OUT_PLANES, list_len(saved_list));
        aga_diag_set(AGA_D_PTR_OUT_CTL, dl_rd(saved_list));
        aga_diag_set(AGA_D_PTR_OUT_POS0, dl_rd(saved_list + 1));
        aga_diag_set(AGA_D_PTR_OUT_IMAGE, dl_rd(saved_list + 4));
        int r = fw_blank(0);
        aga_diag_set(AGA_D_PTR_NOW_LIST, reg_rd(REG_DISPLIST1));
        aga_diag_set(AGA_D_PTR_NOW_PLANES, (uint32_t)r);
        aga_diag_log(r < 0 ? "[AGA] HVS: firmware unblank failed" : "[AGA] HVS: firmware unblank sent");
    }
    cur_list = 0;
    /* Only after the HVS is scanning the driver's list again, never while it
       is still walking mine. This used to be a comment and no wait: on the
       Pi 3 the restored words are garbage, the scaler chewed on them for the
       rest of the frame, hung, and never reached the frame start where it
       latches the new address - black desktop, line counter frozen. */
    if (AGA_PI3_BUILD) {
        uint64_t t0 = cnt_now();
        int ok = wait_latched(saved_list, 60000);
        uint64_t f; __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(f));
        aga_diag_set(AGA_D_WATCH_LATCH_US, (uint32_t)(((cnt_now() - t0) * 1000000u) / f));
        aga_diag_log(ok ? "[AGA] HVS: driver's list latched, ours released" : "[AGA] HVS: driver's list NOT latched in 60 ms");
    }
    dl_return();
    if (AGA_PROBES_ON) ptr_probe(reg_rd(REG_DISPLIST1), AGA_D_PTR_OUT_LIST);      /* the pointer as I left it */
    if (AGA_PI3_BUILD && AGA_PROBES_ON) {
        dl_map_log("[AGA] dlist@hide");
        dl_words_log("[AGA] handed-back words", saved_list);
        watch_arm();
    }
}

/* Sandbox leave / watchdog park. The loop stops presenting, but nothing told
   the HVS to stop SHOWING - so the last frame sat on top of the desktop.
   Invisible while the last frame was always a hidden one; the moment the
   overlay made it a black frame with digits on it, a machine that left the
   sandbox cleanly looked wedged, counters frozen at whatever the hell they
   read on the way out. */
void aga_video_hide(void) { if (video_ok) hide(); }

/* Live read, from the diagnostic dump path: whatever the HVS is scanning
   right now, sandbox or not. Run agastat while the pointer is invisible and
   these counters are the state on screen. Reads only. */
void aga_video_probe_pointer(void)
{
    if (video_ok) ptr_probe(reg_rd(REG_DISPLIST1), AGA_D_PTR_NOW_LIST);
}
volatile int aga_overlay_idle = 0;   /* show the overlay on a blank plane while the sandbox idles */

/* Called from the chipset loop (core 3) once per completed frame. */
void aga_video_set_window(int mode, int x, int y, int w, int h)
{
    win_mode = mode; win_x = x; win_y = y; win_w = w; win_h = h;
}

/* ---- debug overlay --------------------------------------------------------
 * Game wedges, AmigaOS is gone, caffed cannot answer - but the chipset loop
 * keeps presenting frames, so the screen is the only channel left. And every
 * way out of a wedge (Ctrl-Amiga-Amiga included, measured 2026-09-07) resets
 * the Pi and wipes the counters, which is why two Banshee runs in a row told
 * me sweet fuck-all. So: draw the numbers into the frame and photograph
 * them.
 *
 * Seven-segment, not a font: sixteen glyphs out of a sixteen-byte table and
 * four rectangles, no font data to carry around.
 */
#define OVL_ROWS 12
static volatile uint32_t ovl_val[OVL_ROWS];
#ifdef AGA_DIAG
static volatile int      ovl_on = 1;      /* diag builds: on, to photograph a wedge */
#else
static volatile int      ovl_on = 0;      /* the build people play: a clean picture */
#endif

void aga_video_overlay(int row, uint32_t v) { if ((unsigned)row < OVL_ROWS) ovl_val[row] = v; }
void aga_video_overlay_enable(int on)       { ovl_on = on; }

/* bit 0..6 = segments a,b,c,d,e,f,g */
static const uint8_t seg7[16] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
    0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71,
};

#define DW 10          /* digit cell width  */
#define DH 18          /* digit cell height */
#define DT 2           /* segment thickness */

static void ovl_fill(uint32_t *dst, int x, int y, int w, int h, uint32_t col)
{
    if (x < 0 || y < 0 || x + w > AGA_OUT_W || y + h > AGA_OUT_H) return;
    for (int j = 0; j < h; j++)
        for (int i = 0; i < w; i++)
            dst[(y + j) * AGA_OUT_W + (x + i)] = col;      /* native, like the core's pixels */
}

static void ovl_digit(uint32_t *dst, int x, int y, int d, uint32_t col)
{
    int s = seg7[d & 15], half = (DH - 3 * DT) / 2;
    if (s & 0x01) ovl_fill(dst, x + DT,      y,                     DW - 2 * DT, DT,   col); /* a */
    if (s & 0x02) ovl_fill(dst, x + DW - DT, y + DT,                DT,          half, col); /* b */
    if (s & 0x04) ovl_fill(dst, x + DW - DT, y + 2 * DT + half,     DT,          half, col); /* c */
    if (s & 0x08) ovl_fill(dst, x + DT,      y + 2 * DT + 2 * half, DW - 2 * DT, DT,   col); /* d */
    if (s & 0x10) ovl_fill(dst, x,           y + 2 * DT + half,     DT,          half, col); /* e */
    if (s & 0x20) ovl_fill(dst, x,           y + DT,                DT,          half, col); /* f */
    if (s & 0x40) ovl_fill(dst, x + DT,      y + DT + half,         DW - 2 * DT, DT,   col); /* g */
}

static void ovl_draw(uint32_t *dst)
{
    const int x0 = 8, y0 = 8, pitch = DH + 6;
    ovl_fill(dst, x0 - 4, y0 - 4, 8 * (DW + 2) + 8, OVL_ROWS * pitch + 8, 0xFF000000u);
    for (int r = 0; r < OVL_ROWS; r++) {
        uint32_t v = ovl_val[r];
        /* row 0 is the one that matters most, so make it stand out */
        uint32_t col = 0xFF000000u | (r == 0 ? 0x00FF6060u : 0x0000FF00u);
        for (int d = 0; d < 8; d++)
            ovl_digit(dst, x0 + d * (DW + 2), y0 + r * pitch, (v >> ((7 - d) * 4)) & 0xF, col);
    }
}

void aga_video_present(const uint32_t *frame, int native_active)
{
    if (!video_ok) return;
    /* No native display. Normally hide the plane and let the RTG desktop show
       through - and that is what the "grey screen" has been all along: not my
       output at all, just P96's screen sitting blank after WHDLoad took the
       machine off AmigaOS. Which also means the overlay never drew, because
       this function returned here first.

       When the sandbox is up that is the case worth looking at, so show a
       black plane with the numbers on it instead. Outside the sandbox
       nothing changes, desktop untouched. */
    if (!native_active) {
        /* Default: hide. My display list REPLACES the HVS list, so while the
           plane is up the RTG desktop is not drawn at all - and that hid
           WHDLoad's own splash menu, Start button nobody could click. Idle
           overlay is opt-in (agaboot OVERLAY ON, $5F01) for a game that
           hangs without ever drawing. */
        if (!(ovl_on && aga_sandbox_active && aga_overlay_idle)) { hide(); return; }
        /* TRANSPARENT, not black. The plane runs alpha mode FIXED_NONZERO
           (POS2_ALPHA_FIXED): a pixel whose alpha byte is zero is not drawn
           at all and the RTG desktop shows through, while the overlay box
           stays opaque and readable. A black plane hid a WHDLoad requester
           behind the digits and cost me a damn power-cut to find. */
        int p = fb_page ^ 1;
        uint32_t *d = (uint32_t *)(fb_va + (uintptr_t)p * FB_BYTES);
        for (int i = 0; i < AGA_OUT_W * AGA_OUT_H; i++) d[i] = 0;
        ovl_draw(d);
        show(fb_phys + (uintptr_t)p * FB_BYTES);
        fb_page = p;
        return;
    }

    /* Core rendered this frame straight into one of my two pages (see
       aga_set_framebuffers): no copy, and the pages are uncached, so no
       flush either. This used to be 900 KB read, 900 KB written, 900 KB of
       cache cleaned every fucking frame - 1.2 ms on a Pi 4 and, on a Pi 3,
       the working set the game's chip RAM traffic evicted. What a waste of a
       core. */
    int page = (frame == (const uint32_t *)(fb_va + FB_BYTES)) ? 1 : 0;
    uint32_t *dst = (uint32_t *)(fb_va + (uintptr_t)page * FB_BYTES);
    if (ovl_on) ovl_draw(dst);
    show(fb_phys + (uintptr_t)page * FB_BYTES);
    fb_page = page;
}

/* ---- mode switch: reboot the Pi, optionally with the firmware's one-shot
   tryboot flag so tryboot.txt (the AGA configuration) is used for the next
   boot only ----------------------------------------------------------------- */
#ifdef PISTORM_CLASSIC
/* The classic protocol keeps its reset inline in the housekeeper and exports
   no pi_reset(), so I copied the same damn watchdog sequence it uses
   (ps_classic_protocol.c, the PIN_RESET branch). */
#define PM_RSTC         ((volatile unsigned int *)(0xf2000000 + 0x0010001c))
#define PM_RSTS         ((volatile unsigned int *)(0xf2000000 + 0x00100020))
#define PM_WDOG         ((volatile unsigned int *)(0xf2000000 + 0x00100024))
#define PM_WDOG_MAGIC   0x5a000000
#define PM_RSTC_FULLRST 0x00000020
static void pi_reset(void)
{
    unsigned int r = LE32(*PM_RSTS);
    r &= ~0xfffffaaau;                       /* boot from partition 0 */
    *PM_RSTS = LE32(PM_WDOG_MAGIC | r);
    *PM_WDOG = LE32(PM_WDOG_MAGIC | 10);
    *PM_RSTC = LE32(PM_WDOG_MAGIC | PM_RSTC_FULLRST);
    for (;;) { }
}
#else
extern void pi_reset(void);
#endif

void aga_reboot(int tryboot)
{
    aga_diag_inc(AGA_D_REBOOTS);
    if (tryboot) {
        int c = 1;
        req[c++] = 0;
        req[c++] = LE32(0x38064); req[c++] = LE32(4); req[c++] = 0;   /* SET_REBOOT_FLAGS */
        req[c++] = LE32(1);                                           /* bit 0 = tryboot */
        req[c++] = 0;
        req[0] = LE32(c << 2);
        mbox_call();
        kprintf("[AGA] tryboot flag set, rebooting into AGA mode\n");
    } else {
        kprintf("[AGA] rebooting into ECS mode\n");
    }
    pi_reset();
}
