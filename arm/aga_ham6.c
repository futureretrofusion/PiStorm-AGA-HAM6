/* PiStorm AGA/ECS -> physical HAM6 presentation engine
 *
 * Provenance:
 *   The HAM6 display lineage originates in E-UAE's AmigaOS graphics backend,
 *   principally src/gfx-amigaos/ami-win.c. That Amiga-side HAM/display path
 *   was adapted here into a reusable RGB -> HAM6 output engine for the
 *   PiStorm software AGA/ECS renderer, then accelerated by moving the physical
 *   conversion/presentation workload onto a spare Raspberry Pi CPU core.
 *
 * Architecture:
 *   CPU3 software AGA -> completed 800x286 RGB framebuffer
 *      -> CPU3 copies the descriptor-sized lowres presentation window into A/B Fast-RAM slots
 *      -> CPU1 realtime-service hook consumes newest slot asynchronously
 *      -> CPU1 native lores sampling / hires 2:1 reduction -> HAM6 encode -> dirty runs
 *      -> CPU1 direct physical Chip-RAM publication -> real RGB
 *
 * CPU3 NEVER waits for CPU1. If CPU1 is behind, an obsolete pending slot is
 * overwritten by the newest frame, using a newest-frame-wins non-blocking producer/consumer policy.
 *
 * The sandbox's Copper, bitplanes and virtual Chip RAM NEVER become physical.
 * The only real-Chip resource is one or two descriptor-sized lowres HAM bitmaps (up to the 400x286 software envelope) plus
 * the graphics.library Copper list and a tiny descriptor. HAMBUFFER OFF preserves
 * the V0.14.1 direct-visible path; HAMBUFFER ON writes the hidden physical bitmap
 * and changes only the HAM screen's Copper BPLxPT words during physical VBlank.
 *
 * Physical sink layout supports the Amiga bitmap layouts encountered by the
 * hardware-tested HAM6 presenter:
 *   canonical AGA/V39 interleaved: BitMap.BytesPerRow=240, planeRow=40, stride=240
 *   legacy interleaved:            BitMap.BytesPerRow=40,  planeRow=40, stride=240
 *   separate planar:               BitMap.BytesPerRow=40,  planeRow=40, stride=40
 *
 * Geometry policy:
 *   The AGA core framebuffer is stored in HIRES coordinates:
 *   800 framebuffer pixels == 400 lores pixels of overscan.
 *   The physical HAM descriptor now supplies runtime width/height/stride.  The
 *   source window expands around the proven standard-PAL centre: 320x256 maps
 *   exactly to the old X=114,Y=18 crop, while a 368x280 sink clamps at the framebuffer edge to X=64,Y=6.
 *   LORES takes one of each duplicated pair: native sampling, no averaging.
 *   HIRES/SHRES reduce the hires-coordinate framebuffer horizontally 2:1.
 *   LACE keeps each field vertically 1:1;
 *   true 320x512 physical lowres-interlace timing is a later sink extension.
 *
 * CPU1 coexistence:
 *   CPU1 already owns FRF realtime/I/O services. This backend therefore does
 *   only 8 HAM scanlines per frf_services loop pass, then returns immediately
 *   so USB/input/RDB/ADF service latency remains bounded.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <stdint.h>
#include "aga_ham6.h"
#include "aga_glue.h"
#include "ps_protocol.h"

extern void kprintf(const char *fmt, ...);

/* FRF_AGA_ECS_DENISE_BRIDGE_V0_1: logically separate presenter compiled in
 * this translation unit so it can reuse the proven descriptor/staging/service
 * ownership without a second CMake or CPU1 dispatch path. */
static int  ecs_denise_descriptor_selected(void);
static int  ecs_denise_is_active(void);
static void ecs_denise_present(const uint32_t *frame, int native_active);
static void ecs_denise_cpu1_service(void);
static void ecs_denise_request_stop(void);


#define FRF_AGA_HAM_MAGIC       0x48414D36u
#define FRF_AGA_HAM_VERSION_V9   9u  /* exact V0.14.1 single-buffer descriptor */
#define FRF_AGA_HAM_VERSION_V10 10u  /* V10 adds optional physical back-buffer extension */
#define FRF_AGA_HAM_PLANES      6u
#define FRF_AGA_CHIP_LIMIT      0x00080000u

/* Software AGA output envelope.  The HAM sink is lowres, therefore one HAM
 * pixel consumes two framebuffer samples regardless of guest lores/hires: a
 * lores source is pair-duplicated by the renderer; a hires source is averaged
 * 2:1 here. */
#define FRF_AGA_SRC_W           800u
#define FRF_AGA_SRC_H           286u
#define FRF_AGA_HAM_MAX_W       (FRF_AGA_SRC_W / 2u)   /* 400 lores pixels */
#define FRF_AGA_HAM_MAX_H       FRF_AGA_SRC_H           /* 286 lines */
#define FRF_AGA_HAM_MAX_WORDS   ((FRF_AGA_HAM_MAX_W + 15u) / 16u)
#define FRF_AGA_STAGE_MAX_W     FRF_AGA_SRC_W

/* Preserve the old proven 320x256 crop exactly, then grow outwards around its
 * centre when the physical descriptor asks for overscan. */
#define FRF_AGA_PAL_CENTER_X    (114u + 320u)  /* hires framebuffer samples */
#define FRF_AGA_PAL_CENTER_Y    (18u + 128u)

#define FRF_BPLCON0_HIRES        0x8000u
#define FRF_BPLCON0_SHRES        0x0040u
#define FRF_BPLCON0_LACE         0x0004u

extern uint16_t aga_glue_peek_reg(uint32_t reg);

enum {
    HD_MAGIC = 0,
    HD_WIDTH,
    HD_HEIGHT,
    HD_BYTESPERROW,
    HD_ROWSTRIDE,
    HD_FLAGS,
    HD_PLANE0,
    HD_PLANE1,
    HD_PLANE2,
    HD_PLANE3,
    HD_PLANE4,
    HD_PLANE5,
    HD_COPPER_LOF,
    HD_VERSION,
    HD_LONGS
};

#define FRF_AGA_HAMBUFFER_EXT_MAGIC 0x48424631u /* HBF1 */
enum {
    HBE_MAGIC = 0,
    HBE_COPPER_LOF,
    HBE_FLAGS,
    HBE_BACK_PLANE0,
    HBE_BACK_PLANE1,
    HBE_BACK_PLANE2,
    HBE_BACK_PLANE3,
    HBE_BACK_PLANE4,
    HBE_BACK_PLANE5,
    HBE_LONGS
};

static volatile uint32_t g_desc_addr;
static volatile uint32_t g_start_deferred; /* RUN: hold physical HAM until target AGA display is stable */
static volatile uint32_t g_scoped_start;   /* RUN: suppress diagnostic first-light at the delayed handover */
static uint32_t g_plane[FRF_AGA_HAM_PLANES];
static uint32_t g_phys_plane[2][FRF_AGA_HAM_PLANES];
static uint8_t g_double_buffer;
static uint8_t g_front_phys;
static uint8_t g_write_phys;
static uint8_t g_phys_valid[2];
static uint8_t g_flip_pending;
static uint32_t g_page_flips;
static uint32_t g_flip_waits;

#define FRF_AGA_COPPTR_MAX_OCCURRENCES 16u
static uint32_t g_cop_value_slot[FRF_AGA_HAM_PLANES][2][FRF_AGA_COPPTR_MAX_OCCURRENCES];
static uint8_t g_cop_value_count[FRF_AGA_HAM_PLANES][2];
static uint32_t g_row_stride;
static uint32_t g_copper_lof;
/* Geometry is written by CPU1 before g_active is release-published. CPU3 only
 * snapshots after observing g_active, so these plain words are stable for the
 * lifetime of an armed descriptor. */
static uint32_t g_ham_w, g_ham_h, g_ham_words, g_ham_row_bytes;
static uint32_t g_src_x, g_src_y, g_stage_w;
static volatile uint32_t g_active;
static uint8_t g_force_full;
static uint8_t g_lut_ready;
static uint8_t g_direct_pen[65536];
static uint16_t g_shadow[2][FRF_AGA_HAM_PLANES][FRF_AGA_HAM_MAX_H][FRF_AGA_HAM_MAX_WORDS];
static uint32_t g_frames, g_changed_lines, g_reused_lines, g_chip_bytes, g_runs;
static uint32_t g_begin_attempts, g_begin_failures;
static uint16_t g_src_cache[FRF_AGA_HAM_MAX_H][FRF_AGA_HAM_MAX_W];
static uint8_t g_src_valid[FRF_AGA_HAM_MAX_H];
static uint16_t g_last_bplcon0;
static uint32_t g_source_reused_lines, g_source_changed_lines;
static uint32_t g_input_frames, g_skipped_frames;
static volatile uint8_t g_ham_fps = 25u;

#define FRF_AGA_CPU1_LINES_PER_SERVICE    8u

enum {
    FRF_HAM_SLOT_FREE    = 0u,
    FRF_HAM_SLOT_WRITING = 1u,
    FRF_HAM_SLOT_PENDING = 2u,
    FRF_HAM_SLOT_BUSY    = 3u
};

static uint32_t g_stage[2][FRF_AGA_HAM_MAX_H][FRF_AGA_STAGE_MAX_W]
    __attribute__((aligned(64)));

static volatile uint32_t g_slot_state[2];
static volatile uint32_t g_slot_seq[2];
static uint16_t g_slot_bplcon0[2];

static volatile int32_t g_worker_slot = -1;
static uint32_t g_worker_seq;
static unsigned g_worker_y;
static uint16_t g_worker_bplcon0;

static volatile uint32_t g_submit_seq;
static volatile uint32_t g_completed_seq;
static uint32_t g_submit_input_frames;
static uint32_t g_submitted_frames;
static uint32_t g_async_completed_frames;
static uint32_t g_async_dropped_frames;
static uint16_t g_submit_last_mode = 0xffffu;

static volatile uint32_t g_stop_requested;
static volatile uint32_t g_rearm_requested;
static uint32_t g_cpu1_service_calls;
static uint32_t g_cpu1_no_job_calls;

static inline uint32_t frf_load_u32(volatile uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void frf_store_u32(volatile uint32_t *p, uint32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static inline int32_t frf_load_i32(volatile int32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}

static inline void frf_store_i32(volatile int32_t *p, int32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}


static const uint8_t ham_base_rgb[16][3] = {
    {  0,  0,  0}, {255,255,255}, { 85, 85, 85}, {170,170,170},
    {255,  0,  0}, {  0,255,  0}, {  0,  0,255}, {255,255,  0},
    {  0,255,255}, {255,  0,255}, {128,  0,  0}, {  0,128,  0},
    {  0,  0,128}, {128,128,  0}, {  0,128,128}, {128,  0,128}
};

static const uint8_t ham_q5_to_4[32] = {
     0, 0, 1, 1, 2, 2, 3, 3,
     4, 4, 5, 5, 6, 6, 7, 7,
     8, 8, 9, 9,10,10,11,11,
    12,12,13,13,14,14,15,15
};

static const uint8_t ham_q6_to_4[64] = {
     0, 0, 0, 1, 1, 1, 1, 2,
     2, 2, 2, 3, 3, 3, 3, 4,
     4, 4, 4, 5, 5, 5, 5, 6,
     6, 6, 6, 7, 7, 7, 7, 8,
     8, 8, 8, 8, 9, 9, 9, 9,
    10,10,10,10,11,11,11,11,
    12,12,12,12,13,13,13,13,
    14,14,14,14,15,15,15,15
};

static void build_direct_pen(void)
{
    if (g_lut_ready) return;

    for (uint32_t pix = 0; pix < 65536u; ++pix) {
        unsigned tr = ham_q5_to_4[(pix >> 11) & 31u];
        unsigned tg = ham_q6_to_4[(pix >> 5) & 63u];
        unsigned tb = ham_q5_to_4[pix & 31u];
        uint32_t best = 0xffffffffu;
        uint8_t best_code = 0;

        for (unsigned i = 0; i < 16u; ++i) {
            int dr = (int)tr - (int)(ham_base_rgb[i][0] >> 4);
            int dg = (int)tg - (int)(ham_base_rgb[i][1] >> 4);
            int db = (int)tb - (int)(ham_base_rgb[i][2] >> 4);
            uint32_t e = (uint32_t)(3 * dr * dr + 4 * dg * dg + 2 * db * db);
            if (e < best) {
                best = e;
                best_code = (uint8_t)i;
            }
        }
        g_direct_pen[pix] = best_code;
    }

    g_lut_ready = 1;
    kprintf("[AGA-HAM6] V0.12 fast 4-bit direct-pen LUT ready\n");
}

static inline uint16_t rgb32_to_565(uint32_t rgb)
{
    return (uint16_t)(((rgb >> 8) & 0xF800u) |
                      ((rgb >> 5) & 0x07E0u) |
                      ((rgb >> 3) & 0x001Fu));
}

static inline uint32_t rgb32_avg2(uint32_t a, uint32_t b)
{
    unsigned ar = (a >> 16) & 0xffu, ag = (a >> 8) & 0xffu, ab = a & 0xffu;
    unsigned br = (b >> 16) & 0xffu, bg = (b >> 8) & 0xffu, bb = b & 0xffu;
    return (((ar + br + 1u) >> 1) << 16) |
           (((ag + bg + 1u) >> 1) << 8) |
           ((ab + bb + 1u) >> 1);
}

static __attribute__((noinline)) void frf_words_zero(uint16_t *dst, unsigned words)
{
    volatile uint16_t *d = (volatile uint16_t *)dst;
    for (unsigned i = 0; i < words; ++i) d[i] = 0;
}

static int prepare_source_line(const uint32_t active[FRF_AGA_STAGE_MAX_W],
                               unsigned y, uint16_t bplcon0,
                               uint16_t src565[FRF_AGA_HAM_MAX_W])
{
    const int hires =
        (bplcon0 & (FRF_BPLCON0_HIRES | FRF_BPLCON0_SHRES)) != 0;

    for (unsigned x = 0; x < g_ham_w; ++x) {
        unsigned sx = x * 2u;
        uint32_t rgb =
            hires ? rgb32_avg2(active[sx], active[sx + 1u]) : active[sx];
        src565[x] = rgb32_to_565(rgb);
    }

    if (g_src_valid[y]) {
        int same = 1;
        for (unsigned x = 0; x < g_ham_w; ++x) {
            if (g_src_cache[y][x] != src565[x]) {
                same = 0;
                break;
            }
        }
        if (same) {
            ++g_source_reused_lines;
            return 0;
        }
    }

    for (unsigned x = 0; x < g_ham_w; ++x)
        g_src_cache[y][x] = src565[x];

    g_src_valid[y] = 1;
    ++g_source_changed_lines;
    return 1;
}

static void encode_line_565(const uint16_t src565[FRF_AGA_HAM_MAX_W],
                            uint16_t packed[FRF_AGA_HAM_PLANES][FRF_AGA_HAM_MAX_WORDS])
{
    unsigned pr = 0u, pg = 0u, pb = 0u;

    for (unsigned word = 0; word < g_ham_words; ++word) {
        uint16_t pw0 = 0, pw1 = 0, pw2 = 0, pw3 = 0, pw4 = 0, pw5 = 0;

        for (unsigned bit = 0; bit < 16u; ++bit) {
            unsigned x = word * 16u + bit;
            /* Padding bits in a non-16-pixel final word remain black.  Normal
             * PAL profiles (320/352/368/384/400) are word aligned. */
            uint16_t p = x < g_ham_w ? src565[x] : 0u;
            unsigned tr = ham_q5_to_4[(p >> 11) & 31u];
            unsigned tg = ham_q6_to_4[(p >> 5) & 63u];
            unsigned tb = ham_q5_to_4[p & 31u];

            uint8_t direct = g_direct_pen[p];
            unsigned dr4 = (unsigned)(ham_base_rgb[direct][0] >> 4);
            unsigned dg4 = (unsigned)(ham_base_rgb[direct][1] >> 4);
            unsigned db4 = (unsigned)(ham_base_rgb[direct][2] >> 4);

            int er = (int)tr - (int)dr4;
            int eg = (int)tg - (int)dg4;
            int eb = (int)tb - (int)db4;
            uint32_t best = (uint32_t)(3 * er * er + 4 * eg * eg + 2 * eb * eb);
            uint8_t code = direct;
            unsigned br = dr4, bg = dg4, bb = db4;

            {
                int rr = (int)tr - (int)pr;
                int gg = (int)tg - (int)pg;
                uint32_t e = (uint32_t)(3 * rr * rr + 4 * gg * gg);
                if (e < best) {
                    best = e;
                    code = (uint8_t)(0x10u | tb);
                    br = pr; bg = pg; bb = tb;
                }
            }
            {
                int gg = (int)tg - (int)pg;
                int db = (int)tb - (int)pb;
                uint32_t e = (uint32_t)(4 * gg * gg + 2 * db * db);
                if (e < best) {
                    best = e;
                    code = (uint8_t)(0x20u | tr);
                    br = tr; bg = pg; bb = pb;
                }
            }
            {
                int rr = (int)tr - (int)pr;
                int db = (int)tb - (int)pb;
                uint32_t e = (uint32_t)(3 * rr * rr + 2 * db * db);
                if (e < best) {
                    code = (uint8_t)(0x30u | tg);
                    br = pr; bg = tg; bb = pb;
                }
            }

            uint16_t mask = (uint16_t)(0x8000u >> bit);
            if (code & 0x01u) pw0 |= mask;
            if (code & 0x02u) pw1 |= mask;
            if (code & 0x04u) pw2 |= mask;
            if (code & 0x08u) pw3 |= mask;
            if (code & 0x10u) pw4 |= mask;
            if (code & 0x20u) pw5 |= mask;

            pr = br; pg = bg; pb = bb;
        }

        packed[0][word] = pw0;
        packed[1][word] = pw1;
        packed[2][word] = pw2;
        packed[3][word] = pw3;
        packed[4][word] = pw4;
        packed[5][word] = pw5;
    }
}


#define FRF_AGA_HAM_FLAG_INTERLEAVED  0x00000001u
#define FRF_AGA_HAM_FLAG_DOUBLEBUFFER 0x00000002u

static unsigned physical_vpos(void)
{
    uint16_t vposr = ps_read_16(0xDFF004u);
    uint16_t vhposr = ps_read_16(0xDFF006u);
    return (((unsigned)vposr & 7u) << 8) | ((unsigned)vhposr >> 8);
}

static int physical_vblank_window(void)
{
    unsigned v = physical_vpos();
    /* PAL physical sink: use only the quiet top/bottom blanking margins.
     * Never busy-wait on CPU1; if we miss the window the service loop tries
     * again on its next pass. */
    return v < 20u || v > 300u;
}

static void copper_slots_reset(void)
{
    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
        g_cop_value_count[p][0] = 0u;
        g_cop_value_count[p][1] = 0u;
        for (unsigned hl = 0; hl < 2u; ++hl)
            for (unsigned i = 0; i < FRF_AGA_COPPTR_MAX_OCCURRENCES; ++i)
                g_cop_value_slot[p][hl][i] = 0u;
    }
}

static int copper_find_ham_bitplane_slots(void)
{
    uint32_t hi_slot[FRF_AGA_HAM_PLANES] = {0};
    uint16_t hi_value[FRF_AGA_HAM_PLANES] = {0};
    copper_slots_reset();

    /* The active Intuition View can contain other screen transitions. Pair a
     * BPLxPTH MOVE with the following BPLxPTL for that same plane, then accept
     * it only when the combined 32-bit value equals THIS HAM front plane.
     * Matching only the high 16 bits would be unsafe because unrelated screens
     * can share the same Chip-RAM high word. */
    for (unsigned ins = 0; ins < 4096u; ++ins) {
        uint32_t ia = g_copper_lof + ins * 4u;
        if (ia + 4u > FRF_AGA_CHIP_LIMIT) break;
        uint16_t w0 = ps_read_16(ia);
        uint16_t w1 = ps_read_16(ia + 2u);
        if (w0 == 0xffffu && w1 == 0xfffeu) break;
        if (w0 & 1u) continue; /* WAIT/SKIP */
        unsigned reg = (unsigned)w0 & 0x01feu;
        for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
            unsigned rh = 0x00e0u + p * 4u;
            unsigned rl = rh + 2u;
            if (reg == rh) {
                hi_slot[p] = ia + 2u;
                hi_value[p] = w1;
                continue;
            }
            if (reg == rl && hi_slot[p]) {
                uint32_t ptr = ((uint32_t)hi_value[p] << 16) | w1;
                if (ptr == g_phys_plane[g_front_phys][p]) {
                    uint8_t n = g_cop_value_count[p][0];
                    if (n >= FRF_AGA_COPPTR_MAX_OCCURRENCES) {
                        kprintf("[AGA-HAM6] V0.15.2 HAMBUFFER too many Copper BPL%uPT occurrences; refusing pageflip\n",
                                p + 1u);
                        return 0;
                    }
                    g_cop_value_slot[p][0][n] = hi_slot[p];
                    g_cop_value_slot[p][1][n] = ia + 2u;
                    g_cop_value_count[p][0] = (uint8_t)(n + 1u);
                    g_cop_value_count[p][1] = (uint8_t)(n + 1u);
                }
                hi_slot[p] = 0u;
            }
        }
    }

    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
        if (!g_cop_value_count[p][0] ||
            g_cop_value_count[p][0] != g_cop_value_count[p][1]) {
            kprintf("[AGA-HAM6] V0.15.2 HAMBUFFER cannot locate paired Copper BPL%uPT slots for front=%08x\n",
                    p + 1u, g_phys_plane[g_front_phys][p]);
            return 0;
        }
    }
    return 1;
}

static void copper_publish_front(unsigned phys)
{
    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
        uint32_t pa = g_phys_plane[phys][p];
        uint16_t hi = (uint16_t)(pa >> 16);
        uint16_t lo = (uint16_t)pa;
        for (unsigned i = 0; i < g_cop_value_count[p][0]; ++i)
            ps_write_16(g_cop_value_slot[p][0][i], hi);
        for (unsigned i = 0; i < g_cop_value_count[p][1]; ++i)
            ps_write_16(g_cop_value_slot[p][1][i], lo);
    }
}

static void physical_select_write_buffer(unsigned phys)
{
    g_write_phys = (uint8_t)phys;
    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p)
        g_plane[p] = g_phys_plane[phys][p];
}

static int cpu1_try_page_flip(void)
{
    if (!g_double_buffer || !g_flip_pending) return 1;
    if (!physical_vblank_window()) {
        ++g_flip_waits;
        return 0;
    }

    unsigned new_front = g_write_phys;
    unsigned old_front = g_front_phys;
    copper_publish_front(new_front);
    __asm__ volatile("dmb ish" ::: "memory");
    g_front_phys = (uint8_t)new_front;
    physical_select_write_buffer(old_front);
    g_flip_pending = 0u;
    ++g_page_flips;

    /* The old front is now hidden and safe for CPU1. If this buffer has never
     * held an encoded RGB frame, force one complete refresh before its first
     * future presentation. */
    if (!g_phys_valid[g_write_phys])
        g_force_full = 1u;

    if (g_page_flips <= 3u)
        kprintf("[AGA-HAM6] V0.15.2 HAMBUFFER pageflip=%u front=%u back=%u vpos=%u\n",
                g_page_flips, (unsigned)g_front_phys, (unsigned)g_write_phys,
                physical_vpos());
    return 1;
}

static int descriptor_read(void)
{
    uint32_t a = frf_load_u32(&g_desc_addr);
    if (!a || a >= FRF_AGA_CHIP_LIMIT || (a & 3u)) return 0;

    uint32_t magic   = ps_read_32(a + HD_MAGIC * 4u);
    uint32_t width   = ps_read_32(a + HD_WIDTH * 4u);
    uint32_t height  = ps_read_32(a + HD_HEIGHT * 4u);
    uint32_t bpr     = ps_read_32(a + HD_BYTESPERROW * 4u);
    uint32_t stride  = ps_read_32(a + HD_ROWSTRIDE * 4u);
    uint32_t flags   = ps_read_32(a + HD_FLAGS * 4u);
    uint32_t version = ps_read_32(a + HD_VERSION * 4u);
    uint32_t fps     = (flags >> 8) & 0xffu;
    uint32_t words, row_bytes;

    if (fps != 25u && fps != 50u) fps = 25u;
    if (magic != FRF_AGA_HAM_MAGIC ||
        (version != FRF_AGA_HAM_VERSION_V9 && version != FRF_AGA_HAM_VERSION_V10))
        return 0;
    if (width < 16u || width > FRF_AGA_HAM_MAX_W ||
        (width & 15u) != 0u ||
        height < 1u || height > FRF_AGA_HAM_MAX_H)
        return 0;

    /* Native Amiga bitplane rows are word-addressed.  Requiring a 16-pixel
     * lowres width keeps descriptor geometry, Copper fetch width and CPU1
     * dirty-word publication in one exact unit; every offered HAMAREA profile
     * already satisfies this. */
    words = width >> 4;
    row_bytes = words * 2u;
    if (bpr != row_bytes)
        return 0;

    if (flags & FRF_AGA_HAM_FLAG_INTERLEAVED) {
        if (stride != row_bytes * FRF_AGA_HAM_PLANES) return 0;
    } else {
        if (stride != row_bytes) return 0;
    }

    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
        uint32_t pa = ps_read_32(a + (HD_PLANE0 + p) * 4u);
        uint32_t last = pa + (height - 1u) * stride + row_bytes;
        if (!pa || (pa & 1u) || pa >= FRF_AGA_CHIP_LIMIT ||
            last > FRF_AGA_CHIP_LIMIT || last < pa)
            return 0;
        g_phys_plane[0][p] = pa;
        g_plane[p] = pa;
    }

    if (version == FRF_AGA_HAM_VERSION_V9) {
        /* HAMBUFFER OFF is deliberately the exact V0.14.1 descriptor contract. */
        g_copper_lof = ps_read_32(a + HD_COPPER_LOF * 4u);
        if (!g_copper_lof || (g_copper_lof & 1u) ||
            g_copper_lof >= FRF_AGA_CHIP_LIMIT)
            return 0;
        g_double_buffer = 0u;
        for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p)
            g_phys_plane[1][p] = 0u;
    } else {
        /* Descriptor V10 keeps the original 14-long ABI size. HD_COPPER_LOF
         * points at a tiny real-Chip HAMBUFFER extension containing the actual
         * Copper list address plus the hidden bitmap planes. */
        uint32_t ext = ps_read_32(a + HD_COPPER_LOF * 4u);
        if (!ext || (ext & 3u) || ext >= FRF_AGA_CHIP_LIMIT ||
            ext + HBE_LONGS * 4u > FRF_AGA_CHIP_LIMIT)
            return 0;
        if (ps_read_32(ext + HBE_MAGIC * 4u) != FRF_AGA_HAMBUFFER_EXT_MAGIC)
            return 0;
        g_copper_lof = ps_read_32(ext + HBE_COPPER_LOF * 4u);
        if (!g_copper_lof || (g_copper_lof & 1u) ||
            g_copper_lof >= FRF_AGA_CHIP_LIMIT)
            return 0;
        if (!(ps_read_32(ext + HBE_FLAGS * 4u) & FRF_AGA_HAM_FLAG_DOUBLEBUFFER))
            return 0;
        g_double_buffer = 1u;
        for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
            uint32_t pa = ps_read_32(ext + (HBE_BACK_PLANE0 + p) * 4u);
            uint32_t last = pa + (height - 1u) * stride + row_bytes;
            if (!pa || (pa & 1u) || pa >= FRF_AGA_CHIP_LIMIT ||
                last > FRF_AGA_CHIP_LIMIT || last < pa)
                return 0;
            g_phys_plane[1][p] = pa;
            if (pa == g_phys_plane[0][p]) return 0;
        }
        if (flags & FRF_AGA_HAM_FLAG_INTERLEAVED) {
            for (unsigned p = 1; p < FRF_AGA_HAM_PLANES; ++p)
                if (g_phys_plane[1][p] != g_phys_plane[1][0] + p * row_bytes)
                    return 0;
        }
    }

    if (flags & FRF_AGA_HAM_FLAG_INTERLEAVED) {
        for (unsigned p = 1; p < FRF_AGA_HAM_PLANES; ++p)
            if (g_plane[p] != g_plane[0] + p * row_bytes)
                return 0;
    }

    g_ham_w = width;
    g_ham_h = height;
    g_ham_words = words;
    g_ham_row_bytes = row_bytes;
    g_row_stride = stride;
    g_ham_fps = (uint8_t)fps;

    /* Descriptor geometry chooses the physical canvas. Expand symmetrically
     * around the proven standard-PAL centre and clamp only at the software
     * framebuffer's real 800x286 envelope. */
    g_stage_w = width * 2u;
    g_src_x = FRF_AGA_PAL_CENTER_X > width ?
        FRF_AGA_PAL_CENTER_X - width : 0u;
    if (g_src_x + g_stage_w > FRF_AGA_SRC_W)
        g_src_x = FRF_AGA_SRC_W - g_stage_w;

    g_src_y = FRF_AGA_PAL_CENTER_Y > (height / 2u) ?
        FRF_AGA_PAL_CENTER_Y - (height / 2u) : 0u;
    if (g_src_y + height > FRF_AGA_SRC_H)
        g_src_y = FRF_AGA_SRC_H - height;

    return 1;
}

static void refresh_ham_fps(void)
{
    uint32_t desc = frf_load_u32(&g_desc_addr);
    if (!desc) return;

    uint32_t flags = ps_read_32(desc + HD_FLAGS * 4u);
    uint32_t fps = (flags >> 8) & 0xffu;
    if (fps != 25u && fps != 50u) fps = 25u;

    if ((uint8_t)fps != g_ham_fps) {
        g_ham_fps = (uint8_t)fps;
        g_input_frames = 0;
        kprintf("[AGA-HAM6] V0.12 HAMFPS changed live -> %u fps (PAL scanout stays 50Hz)\n",
                (unsigned)g_ham_fps);
    }
}

static void publish_run(unsigned plane, unsigned y, unsigned first, unsigned last,
                        const uint16_t row[FRF_AGA_HAM_MAX_WORDS])
{
    uint32_t addr = g_plane[plane] + y * g_row_stride + first * 2u;
    unsigned i = first;
    if ((addr & 3u) && i < last) {
        ps_write_16(addr, row[i]);
        addr += 2u; ++i; g_chip_bytes += 2u;
    }
    while (i + 1u < last) {
        uint32_t v = ((uint32_t)row[i] << 16) | row[i + 1u];
        ps_write_32(addr, v);
        addr += 4u; i += 2u; g_chip_bytes += 4u;
    }
    if (i < last) { ps_write_16(addr, row[i]); g_chip_bytes += 2u; }
    ++g_runs;
}

/* V0.15.2 first-light is now a CPU1-owned physical-CHIP operation. */
static void first_light_cpu1(void)
{
    uint16_t packed[FRF_AGA_HAM_PLANES][FRF_AGA_HAM_MAX_WORDS];

    for (unsigned y = 0; y < g_ham_h; ++y) {
        uint8_t code = (uint8_t)(1u + ((y >> 4) % 15u));
        for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
            uint16_t v = (code & (1u << p)) ? 0xffffu : 0x0000u;
            for (unsigned w = 0; w < g_ham_words; ++w)
                packed[p][w] = v;
            publish_run(p, y, 0u, g_ham_words, packed[p]);
        }
    }
}

static void async_slots_reset(void)
{
    frf_store_u32(&g_slot_state[0], FRF_HAM_SLOT_FREE);
    frf_store_u32(&g_slot_state[1], FRF_HAM_SLOT_FREE);
    frf_store_u32(&g_slot_seq[0], 0u);
    frf_store_u32(&g_slot_seq[1], 0u);
    frf_store_i32(&g_worker_slot, -1);
    g_worker_seq = 0u;
    g_worker_y = 0u;
    g_worker_bplcon0 = 0u;
}

void aga_ham6_set_descriptor(uint32_t addr)
{
    uint32_t old = frf_load_u32(&g_desc_addr);
    frf_store_u32(&g_desc_addr, addr);

    if (!addr) {
        frf_store_u32(&g_stop_requested, 1u);
    } else if (addr != old) {
        frf_store_u32(&g_rearm_requested, 1u);
        frf_store_u32(&g_stop_requested, 0u);
    }

    __asm__ volatile("dmb ish; sev" ::: "memory");
}

void aga_ham6_defer_start(int on)
{
    frf_store_u32(&g_start_deferred, on ? 1u : 0u);
    if (on) frf_store_u32(&g_scoped_start, 1u);
    if (!on && frf_load_u32(&g_desc_addr)) {
        /* Wake CPU1 and let it consume the already-prepared descriptor. */
        frf_store_u32(&g_rearm_requested, 1u);
        frf_store_u32(&g_stop_requested, 0u);
    }
    __asm__ volatile("dmb ish; sev" ::: "memory");
}

uint32_t aga_ham6_descriptor(void)
{
    return frf_load_u32(&g_desc_addr);
}

int aga_ham6_active(void)
{
    return frf_load_u32(&g_active) != 0u;
}

/* CPU1 ONLY. aga_ham6_present() never calls this in V0.14.1. */

/* FRF_HAM6_WHD_REARM_DIAG_V0_5
 * Observation only. Presenter behavior is not changed.
 */

/* FRF_HAM6_WHD_REARM_FIX_V0_8_INTENA_HANDOFF */
void aga_ham6_request_whd_rearm(void)
{
    if (g_desc_addr) {
        g_rearm_requested = 1u;
        __asm__ volatile("dmb ishst; sev" ::: "memory");
    }
}

static volatile uint32_t g_hdiag5_begin_calls;
static volatile uint32_t g_hdiag5_present_calls;
static volatile uint32_t g_hdiag5_cpu1_calls;
static volatile uint32_t g_hdiag6_takeover_calls;
static volatile uint32_t g_hdiag5_last_active = 0xffffffffu;
static volatile uint32_t g_hdiag5_last_desc   = 0xffffffffu;
static volatile uint32_t g_hdiag5_last_stop   = 0xffffffffu;
static volatile uint32_t g_hdiag5_last_rearm  = 0xffffffffu;

/* FRF_HAM6_WHD_REARM_DIAG_V0_7_HOOKTRACE */
/* FRF_HAM6_WHD_REARM_DIAG_V0_6_SHELL
 *
 * Shell-readable snapshot protocol.
 *
 * Existing control path:
 *   $DFF1F2 <- $5E00 / $5E01
 *
 * A 32-bit preamble is followed by:
 *   32 bits destination MEMF_CHIP address
 *    8 bits command
 *
 * Command 1: snapshot counters/state to destination.
 * Command 2: reset diagnostic counters.
 *
 * Normal isolated $5E00/$5E01 chipset writes cannot accidentally trigger this
 * protocol; they would have to reproduce the full 32-bit preamble bit-for-bit.
 */
#define FRF_HDIAG6_PREAMBLE 0xD15EA5C3u
#define FRF_HDIAG6_MAGIC    0x48443631u /* "HD61" */

static uint32_t g_hdiag6_shift;
static uint32_t g_hdiag6_ptr;
static uint32_t g_hdiag6_cmd;
static unsigned g_hdiag6_collect;
static unsigned g_hdiag6_bits;

static void frf_hdiag6_store32(uint32_t addr, uint32_t value)
{
    if (addr >= 0x00000100u && addr < 0x00200000u)
        ps_write_32(addr, value);
}

static void frf_hdiag6_snapshot(uint32_t addr)
{
    if (addr < 0x00000100u || addr > 0x001FFE00u)
        return;

    frf_hdiag6_store32(addr +  0u, FRF_HDIAG6_MAGIC);
    frf_hdiag6_store32(addr +  4u, 7u);
    frf_hdiag6_store32(addr +  8u, (uint32_t)g_hdiag5_begin_calls);
    frf_hdiag6_store32(addr + 12u, (uint32_t)g_hdiag6_takeover_calls);
    frf_hdiag6_store32(addr + 16u, (uint32_t)g_hdiag5_present_calls);
    frf_hdiag6_store32(addr + 20u, (uint32_t)g_hdiag5_cpu1_calls);
    frf_hdiag6_store32(addr + 24u, (uint32_t)(g_active));
    frf_hdiag6_store32(addr + 28u, (uint32_t)(g_desc_addr));
    frf_hdiag6_store32(addr + 32u, (uint32_t)(g_stop_requested));
    frf_hdiag6_store32(addr + 36u, (uint32_t)(g_rearm_requested));
    {
        extern uint32_t aga_hdiag7_hook_calls(void);
        extern uint32_t aga_hdiag7_trace_count(void);
        extern void aga_hdiag7_get_first(uint32_t idx, uint32_t *reg, uint32_t *val);
        extern void aga_hdiag7_get_last(uint32_t idx, uint32_t *reg, uint32_t *val);
        uint32_t i, n=aga_hdiag7_trace_count();
        uint32_t r, v;

        frf_hdiag6_store32(addr + 40u, aga_hdiag7_hook_calls());
        frf_hdiag6_store32(addr + 44u, n);
        frf_hdiag6_store32(addr + 48u, n);

        for (i=0u;i<16u;++i) {
            aga_hdiag7_get_first(i,&r,&v);
            frf_hdiag6_store32(addr + 52u + i*8u, r);
            frf_hdiag6_store32(addr + 56u + i*8u, v);
        }
        for (i=0u;i<16u;++i) {
            aga_hdiag7_get_last(i,&r,&v);
            frf_hdiag6_store32(addr + 180u + i*8u, r);
            frf_hdiag6_store32(addr + 184u + i*8u, v);
        }
    }

}

static void frf_hdiag6_reset(void)
{
    g_hdiag5_begin_calls = 0u;
    g_hdiag6_takeover_calls = 0u;
    g_hdiag5_present_calls = 0u;
    g_hdiag5_cpu1_calls = 0u;
    { extern void aga_hdiag7_reset_hook(void); aga_hdiag7_reset_hook(); }
}

/* Return 1 when this bit belongs to an active diagnostic transaction and
 * therefore must not alter the normal AGA/ECS preference. */
int aga_ham6_diag_control_bit(int bit)
{
    bit &= 1;

    if (!g_hdiag6_collect) {
        g_hdiag6_shift = (g_hdiag6_shift << 1) | (uint32_t)bit;
        if (g_hdiag6_shift == FRF_HDIAG6_PREAMBLE) {
            g_hdiag6_collect = 1u;
            g_hdiag6_bits = 0u;
            g_hdiag6_ptr = 0u;
            g_hdiag6_cmd = 0u;
            return 1;
        }
        return 0;
    }

    if (g_hdiag6_bits < 32u)
        g_hdiag6_ptr = (g_hdiag6_ptr << 1) | (uint32_t)bit;
    else
        g_hdiag6_cmd = (g_hdiag6_cmd << 1) | (uint32_t)bit;

    ++g_hdiag6_bits;

    if (g_hdiag6_bits == 40u) {
        if (g_hdiag6_cmd == 1u)
            frf_hdiag6_snapshot(g_hdiag6_ptr);
        else if (g_hdiag6_cmd == 2u)
            frf_hdiag6_reset();

        g_hdiag6_collect = 0u;
        g_hdiag6_bits = 0u;
        g_hdiag6_shift = 0u;
    }
    return 1;
}

void aga_ham6_diag_takeover(void)
{
    ++g_hdiag6_takeover_calls;
}


int aga_ham6_begin(void)
{
    ++g_hdiag5_begin_calls;
    kprintf("[HAMDIAG:BEGIN] call=%u active=%u desc=%08x stop=%u rearm=%u\n",
            (unsigned)g_hdiag5_begin_calls,
            (unsigned)(uint32_t)(g_active),
            (unsigned)(uint32_t)(g_desc_addr),
            (unsigned)(uint32_t)(g_stop_requested),
            (unsigned)(uint32_t)(g_rearm_requested));

    ++g_begin_attempts;

    if (!frf_load_u32(&g_desc_addr)) {
        ++g_begin_failures;
        return 0;
    }

    if (!descriptor_read()) {
        ++g_begin_failures;
        if (g_begin_failures <= 3u || (g_begin_failures % 250u) == 0u)
            kprintf("[AGA-HAM6] V0.15.2 CPU1 arm attempt=%u descriptor invalid/not ready\n",
                    g_begin_attempts);
        return 0;
    }

    build_direct_pen();

    for (unsigned b = 0; b < 2u; ++b)
        for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p)
            for (unsigned y = 0; y < g_ham_h; ++y)
                frf_words_zero(g_shadow[b][p][y], g_ham_words);

    for (unsigned y = 0; y < FRF_AGA_HAM_MAX_H; ++y)
        g_src_valid[y] = 0;

    g_frames = g_changed_lines = g_reused_lines = g_chip_bytes = g_runs = 0;
    g_source_reused_lines = g_source_changed_lines = 0;
    g_input_frames = g_skipped_frames = 0;
    g_submit_input_frames = 0;
    g_submitted_frames = 0;
    g_async_completed_frames = 0;
    g_async_dropped_frames = 0;
    g_last_bplcon0 = 0xffffu;
    g_submit_last_mode = 0xffffu;
    async_slots_reset();
    g_front_phys = 0u;
    g_write_phys = g_double_buffer ? 1u : 0u;
    g_phys_valid[0] = 0u;
    g_phys_valid[1] = g_double_buffer ? 1u : 0u; /* AllocBitMap(BMF_CLEAR) back starts black. */
    g_flip_pending = 0u;
    g_page_flips = g_flip_waits = 0u;

    /* Ordinary sandbox sessions keep the proven diagnostic first-light.
       Program-scoped RUN deliberately skips it: when its delayed handover
       occurs, the first visible physical content should be the child's RGB
       frame rather than test bars. */
    physical_select_write_buffer(0u);
    if (!frf_load_u32(&g_scoped_start))
        first_light_cpu1();
    physical_select_write_buffer(g_write_phys);

    if (g_double_buffer && !copper_find_ham_bitplane_slots()) {
        ++g_begin_failures;
        return 0;
    }

    ps_write_16(0xDFF096u, 0x0180u);
    ps_write_16(0xDFF080u, (uint16_t)(g_copper_lof >> 16));
    ps_write_16(0xDFF082u, (uint16_t)g_copper_lof);
    ps_write_16(0xDFF088u, 0x0000u);
    ps_write_16(0xDFF096u, 0x8380u);

    g_force_full = 1;
    frf_store_u32(&g_active, 1u);
    frf_store_u32(&g_scoped_start, 0u);
    frf_store_u32(&g_rearm_requested, 0u);
    frf_store_u32(&g_stop_requested, 0u);

    kprintf("[AGA-HAM6] V0.15.2 CPU1 OVERSCAN HAM online %ux%u words=%u row=%u stride=%u src=%u,%u %ux%u copper=%08x desc=%08x fps=%u budget=%u HAMBUFFER=%s\n",
            g_ham_w, g_ham_h, g_ham_words, g_ham_row_bytes,
            g_row_stride, g_src_x, g_src_y, g_stage_w, g_ham_h,
            g_copper_lof, frf_load_u32(&g_desc_addr),
            (unsigned)g_ham_fps, FRF_AGA_CPU1_LINES_PER_SERVICE,
            g_double_buffer ? "ON" : "OFF");
    kprintf("[AGA-HAM6] V0.15.2 runtime geometry; 320x256 old crop preserved exactly; hires always 2:1 to lowres HAM\n");
    return 1;
}

static int claim_write_slot(unsigned *slot_out)
{
    uint32_t seq_hint = frf_load_u32(&g_submit_seq) + 1u;
    unsigned first = seq_hint & 1u;

    for (unsigned pass = 0; pass < 2u; ++pass) {
        unsigned slot = (first + pass) & 1u;
        uint32_t state = frf_load_u32(&g_slot_state[slot]);

        if (state == FRF_HAM_SLOT_BUSY || state == FRF_HAM_SLOT_WRITING)
            continue;

        uint32_t expected = state;
        if (__atomic_compare_exchange_n(
                &g_slot_state[slot], &expected, FRF_HAM_SLOT_WRITING, 0,
                __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            if (state == FRF_HAM_SLOT_PENDING)
                ++g_async_dropped_frames;
            *slot_out = slot;
            return 1;
        }
    }

    ++g_async_dropped_frames;
    return 0;
}

/* CPU3 producer: no physical ps_write_* calls. */
void aga_ham6_present(const uint32_t *frame, int native_active)
{
    {
        uint32_t _hc = ++g_hdiag5_present_calls;
        if (_hc <= 16u || (_hc & 63u) == 0u) {
            kprintf("[HAMDIAG:PRESENT] call=%u active=%u desc=%08x stop=%u rearm=%u\n",
                    (unsigned)_hc,
                    (unsigned)(uint32_t)(g_active),
                    (unsigned)(uint32_t)(g_desc_addr),
                    (unsigned)(uint32_t)(g_stop_requested),
                    (unsigned)(uint32_t)(g_rearm_requested));
        }
    }

    (void)native_active;
    if (!frame || !frf_load_u32(&g_desc_addr))
        return;
    if (ecs_denise_is_active()) {
        ecs_denise_present(frame, native_active);
        return;
    }
    if (!frf_load_u32(&g_active))
        return;

    uint16_t bplcon0 = aga_glue_peek_reg(0x100u);
    uint16_t mode_mask =
        FRF_BPLCON0_HIRES | FRF_BPLCON0_SHRES | FRF_BPLCON0_LACE;
    uint16_t mode = bplcon0 & mode_mask;
    uint8_t fps = __atomic_load_n(&g_ham_fps, __ATOMIC_ACQUIRE);

    ++g_submit_input_frames;

    if (fps == 25u &&
        mode == g_submit_last_mode &&
        ((g_submit_input_frames & 1u) == 0u)) {
        ++g_skipped_frames;
        return;
    }

    g_submit_last_mode = mode;

    unsigned slot;
    if (!claim_write_slot(&slot))
        return;

    for (unsigned y = 0; y < g_ham_h; ++y) {
        const uint32_t *src =
            frame + (g_src_y + y) * FRF_AGA_SRC_W + g_src_x;
        uint32_t *dst = g_stage[slot][y];
        const uint64_t *s64 = (const uint64_t *)(const void *)src;
        uint64_t *d64 = (uint64_t *)(void *)dst;

        for (unsigned q = 0; q < g_stage_w / 2u; ++q)
            d64[q] = s64[q];
    }

    uint32_t seq = __atomic_add_fetch(&g_submit_seq, 1u, __ATOMIC_RELAXED);
    g_slot_bplcon0[slot] = bplcon0;
    frf_store_u32(&g_slot_seq[slot], seq);
    frf_store_u32(&g_slot_state[slot], FRF_HAM_SLOT_PENDING);
    ++g_submitted_frames;

    if (g_submitted_frames <= 2u)
        kprintf("[AGA-HAM6] V0.15.2 CPU3 submit seq=%u slot=%u mode=%04x sink=%ux%u fps=%u\n",
                seq, slot, bplcon0, g_ham_w, g_ham_h, (unsigned)fps);

    __asm__ volatile("dmb ish; sev" ::: "memory");
}

static int cpu1_claim_newest(void)
{
    int best = -1;
    uint32_t best_seq = 0u;

    for (unsigned slot = 0; slot < 2u; ++slot) {
        if (frf_load_u32(&g_slot_state[slot]) != FRF_HAM_SLOT_PENDING)
            continue;

        uint32_t seq = frf_load_u32(&g_slot_seq[slot]);
        if (best < 0 || seq > best_seq) {
            best = (int)slot;
            best_seq = seq;
        }
    }

    if (best < 0)
        return 0;

    uint32_t expected = FRF_HAM_SLOT_PENDING;
    if (!__atomic_compare_exchange_n(
            &g_slot_state[best], &expected, FRF_HAM_SLOT_BUSY, 0,
            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
        return 0;

    for (unsigned slot = 0; slot < 2u; ++slot) {
        if ((int)slot == best)
            continue;

        if (frf_load_u32(&g_slot_state[slot]) == FRF_HAM_SLOT_PENDING) {
            uint32_t seq = frf_load_u32(&g_slot_seq[slot]);
            if (seq < best_seq) {
                uint32_t old = FRF_HAM_SLOT_PENDING;
                if (__atomic_compare_exchange_n(
                        &g_slot_state[slot], &old, FRF_HAM_SLOT_FREE, 0,
                        __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
                    ++g_async_dropped_frames;
            }
        }
    }

    frf_store_i32(&g_worker_slot, best);
    g_worker_seq = best_seq;
    g_worker_y = 0u;
    g_worker_bplcon0 = g_slot_bplcon0[best];

    {
        uint16_t mode_mask =
            FRF_BPLCON0_HIRES | FRF_BPLCON0_SHRES | FRF_BPLCON0_LACE;
        if ((g_worker_bplcon0 & mode_mask) !=
            (g_last_bplcon0 & mode_mask)) {
            g_force_full = 1;
            if (g_double_buffer) {
                g_phys_valid[0] = 0u;
                g_phys_valid[1] = 0u;
            }
            for (unsigned y = 0; y < g_ham_h; ++y)
                g_src_valid[y] = 0;
            g_last_bplcon0 = g_worker_bplcon0;

            kprintf("[AGA-HAM6] V0.15.2 CPU1 mode=%04x %s %s sink=%ux%u\n",
                    g_worker_bplcon0,
                    (g_worker_bplcon0 &
                     (FRF_BPLCON0_HIRES | FRF_BPLCON0_SHRES))
                        ? "HIRES->LOWRES-2:1" : "LORES-NATIVE",
                    (g_worker_bplcon0 & FRF_BPLCON0_LACE)
                        ? "LACE-FIELD" : "PROGRESSIVE",
                    g_ham_w, g_ham_h);
        }
    }

    return 1;
}

static void cpu1_process_line(unsigned slot, unsigned y)
{
    uint16_t src565[FRF_AGA_HAM_MAX_W];
    uint16_t packed[FRF_AGA_HAM_PLANES][FRF_AGA_HAM_MAX_WORDS];
    int any = 0;

    int source_changed = prepare_source_line(
        g_stage[slot][y], y, g_worker_bplcon0, src565);

    /* Single-buffer mode keeps the V0.14.1 source-line fast path. In physical
     * double-buffer mode the hidden bitmap is two frames old, so an unchanged
     * source line can still differ from that bitmap. Encode it and compare
     * against that buffer's own shadow instead of skipping it globally. */
    if (!g_double_buffer && !g_force_full && !source_changed) {
        ++g_reused_lines;
        return;
    }

    encode_line_565(src565, packed);

    for (unsigned p = 0; p < FRF_AGA_HAM_PLANES; ++p) {
        if (g_force_full) {
            publish_run(p, y, 0u, g_ham_words, packed[p]);
            for (unsigned w = 0; w < g_ham_words; ++w)
                g_shadow[g_write_phys][p][y][w] = packed[p][w];
            any = 1;
            continue;
        }

        unsigned i = 0;
        while (i < g_ham_words) {
            while (i < g_ham_words &&
                   packed[p][i] == g_shadow[g_write_phys][p][y][i])
                ++i;

            if (i == g_ham_words)
                break;

            unsigned first = i;
            while (i < g_ham_words &&
                   packed[p][i] != g_shadow[g_write_phys][p][y][i]) {
                g_shadow[g_write_phys][p][y][i] = packed[p][i];
                ++i;
            }

            publish_run(p, y, first, i, packed[p]);
            any = 1;
        }
    }

    if (any)
        ++g_changed_lines;
    else
        ++g_reused_lines;
}

static void cpu1_finish_frame(unsigned slot)
{
    if (g_force_full) {
        g_force_full = 0;
        kprintf("[AGA-HAM6] V0.15.2 CPU1 FIRST RGB FRAME full %u-byte refresh %ux%u\n",
                g_ham_row_bytes * g_ham_h * FRF_AGA_HAM_PLANES,
                g_ham_w, g_ham_h);
    }

    ++g_frames;
    ++g_async_completed_frames;
    g_phys_valid[g_write_phys] = 1u;
    if (g_double_buffer)
        g_flip_pending = 1u;
    frf_store_u32(&g_completed_seq, g_worker_seq);
    frf_store_u32(&g_slot_state[slot], FRF_HAM_SLOT_FREE);
    frf_store_i32(&g_worker_slot, -1);

    if (g_frames <= 2u)
        kprintf("[AGA-HAM6] V0.15.2 CPU1 completed seq=%u frame=%u chipbytes=%u runs=%u dropped=%u\n",
                g_worker_seq, g_frames, g_chip_bytes, g_runs,
                g_async_dropped_frames);

    if ((g_frames % 250u) == 0u)
        kprintf("[AGA-HAM6] V0.15.2 ASYNC submitted=%u completed=%u dropped=%u chipbytes=%u avg=%u src-change=%u src-reuse=%u\n",
                g_submitted_frames, g_async_completed_frames,
                g_async_dropped_frames, g_chip_bytes,
                g_frames ? g_chip_bytes / g_frames : 0u,
                g_source_changed_lines, g_source_reused_lines);
}

void aga_ham6_cpu1_service(void)
{
    {
        uint32_t _hc = ++g_hdiag5_cpu1_calls;
        uint32_t _ha = (uint32_t)(g_active);
        uint32_t _hd = (uint32_t)(g_desc_addr);
        uint32_t _hs = (uint32_t)(g_stop_requested);
        uint32_t _hr = (uint32_t)(g_rearm_requested);
        if (_hc <= 16u || (_hc & 65535u) == 0u ||
            _ha != g_hdiag5_last_active ||
            _hd != g_hdiag5_last_desc ||
            _hs != g_hdiag5_last_stop ||
            _hr != g_hdiag5_last_rearm) {
            kprintf("[HAMDIAG:CPU1] call=%u active=%u desc=%08x stop=%u rearm=%u\n",
                    (unsigned)_hc,(unsigned)_ha,(unsigned)_hd,
                    (unsigned)_hs,(unsigned)_hr);
            g_hdiag5_last_active=_ha;
            g_hdiag5_last_desc=_hd;
            g_hdiag5_last_stop=_hs;
            g_hdiag5_last_rearm=_hr;
        }
    }

    if (ecs_denise_is_active() ||
        (!frf_load_u32(&g_active) && ecs_denise_descriptor_selected())) {
        ecs_denise_cpu1_service();
        return;
    }

    ++g_cpu1_service_calls;

    if (frf_load_u32(&g_stop_requested)) {
        if (frf_load_u32(&g_active))
            ps_write_16(0xDFF096u, 0x0180u);

        frf_store_u32(&g_active, 0u);
        g_flip_pending = 0u;
        async_slots_reset();
        frf_store_u32(&g_stop_requested, 0u);
        kprintf("[AGA-HAM6] V0.15.2 CPU1 physical output stopped\n");
        return;
    }

    if (!frf_load_u32(&g_start_deferred) &&
        (frf_load_u32(&g_rearm_requested) ||
         (!frf_load_u32(&g_active) && frf_load_u32(&g_desc_addr)))) {
        frf_store_u32(&g_active, 0u);
        if (!aga_ham6_begin())
            return;
    }

    if (!frf_load_u32(&g_active)) {
        ++g_cpu1_no_job_calls;
        return;
    }

    refresh_ham_fps();

    /* A completed hidden HAM frame is immutable until a physical VBlank page
     * flip. Do not claim another RGB frame and overwrite it while waiting. */
    if (g_double_buffer && g_flip_pending && !cpu1_try_page_flip())
        return;

    int32_t worker = frf_load_i32(&g_worker_slot);
    if (worker < 0) {
        if (!cpu1_claim_newest()) {
            ++g_cpu1_no_job_calls;
            return;
        }

        worker = frf_load_i32(&g_worker_slot);
        if (worker < 0)
            return;
    }

    unsigned budget = FRF_AGA_CPU1_LINES_PER_SERVICE;
    while (budget-- && g_worker_y < g_ham_h) {
        cpu1_process_line((unsigned)worker, g_worker_y);
        ++g_worker_y;
    }

    if (g_worker_y >= g_ham_h)
        cpu1_finish_frame((unsigned)worker);
}

void aga_ham6_end(void)
{
    ecs_denise_request_stop();
    frf_store_u32(&g_start_deferred, 0u);
    frf_store_u32(&g_scoped_start, 0u);
    frf_store_u32(&g_desc_addr, 0u);
    frf_store_u32(&g_stop_requested, 1u);
    __asm__ volatile("dmb ish; sev" ::: "memory");
}

/* Standalone Emu68 CPU1 owner.
 *
 * CPU1 owns physical HAM/ECS presentation, including the real VBlank/Copper
 * publish point.  Keep servicing continuously while active so a pending page
 * flip is observed at the physical VBlank boundary; when idle, WFE until the
 * descriptor/control side issues SEV.
 */
void aga_ham6_cpu1_main(void)
{
    for (;;) {
        aga_ham6_cpu1_service();
        if (aga_ham6_active() || ecs_denise_is_active())
            __asm__ volatile("yield" ::: "memory");
        else
            __asm__ volatile("wfe" ::: "memory");
    }
}

/* ECS/OCS physical presenter. Kept as an include to preserve the established
 * aga_ham6.c build target and CPU1 service owner. */
#include "aga_ecs_denise.inc"
