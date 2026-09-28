/* AGA-PISTORM — bitplane DMA fetch and line rendering (Alice/Lisa playfield).
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Fetch model follows UAE (fetch units, fetch start, plane sequence, delay
 * offset). One render pass per line, from the fetched planar data plus the
 * register changes recorded during the line.
 *
 * FRF_AGA_UAE_ALIGNMENT_CORE_V0_2_2: retain the fast line renderer, but
 * align three concrete AGA behaviours with E-UAE/PUAE: delayed Agnus side of
 * BPLCON0, BPLCON1 SHRES sub-pixel delays, and FMODE.BSCAN2 modulo selection.
 */
#include <string.h>
#include "internal.h"

#define HARD_DDF_START 0x18
#define HARD_DDF_STOP  0xD8
/* Bitplane DMA starts DDF_OFFSET colour clocks after the DDFSTRT match (WinUAE
   DMA records show the first fetch block at DDFSTRT+4). A block that completes
   at hpos h is displayed from hardware lores pixel h*2 + DISP_OFFSET. DIW register
   values are compared against hardware coordinates minus DIW_HW_OFFSET (UAE:
   DIW_DDF_OFFSET 9 with fetch at DDFSTRT gives the same picture). */
#define DDF_OFFSET     4
#define DISP_OFFSET    1
#define DIW_HW_OFFSET  8

static const int fetchunits[]  = { 8, 8, 8, 0, 16, 8, 8, 0, 32, 16, 8, 0 };
static const int fetchstarts[] = { 3, 2, 1, 0,  4, 3, 2, 0,  5,  4, 3, 0 };
static const int fm_maxplanes[]= { 3, 2, 1, 0,  3, 3, 2, 0,  3,  3, 3, 0 };
static const uint8_t seq8[8] = { 8, 4, 6, 2, 7, 3, 5, 1 };
static const uint8_t seq4[4] = { 4, 2, 3, 1 };
static const uint8_t seq2[2] = { 2, 1 };

/* per-line snapshot of state at line start, used by the renderer */
static uint32_t line_color[256];
static uint16_t line_bplcon0, line_bplcon1, line_bplcon2, line_bplcon3, line_bplcon4;
/* FRF_AGA_COLOR_DISPLAY_CORE_V0_1_6: DIW raster replay must start from the
   line-start window, not the final window after all Copper writes. */
static int line_diw_hstart, line_diw_hstop;
/* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_PROBE: one-shot live DMA-gate proof. */
static int jotd_bpl_gate_probe_done;
/* FRF_AGA_JOTD_BITPLANE_COHERENCY_V0_1: one-shot JOTD FMODE3 hardware proof. */
static int jotd_fmode3_probe_done;

void pf_reset(aga_t *a)
{
    /* The chipset personality outlives a reset: it is a property of the game
       about to run, chosen before the sandbox is entered, and aga_reset() runs
       as part of entering. */
    int keep_ecs = a->pf.ecs_mode;
    playfield_t *p = &a->pf;
    memset(p, 0, sizeof *p);
    p->bplcon4 = 0x0011;      /* sprite colour bases 16 (OCS compatible) */
    p->bplcon3 = 0x0C00;      /* PF2OF = 3 (offset 8), like OCS dual playfield */
    p->diwstrt = 0x2C81;
    p->diwstop = 0x2CC1;
    p->ddfstrt = 0x0038;
    p->ddfstop = 0x00D0;
    p->fetchunit = 8; p->fetchstart = 8; p->fetchstart_shift = 3; p->maxplanes = 8;
    p->line_plfleft = -1;
    p->ecs_mode = keep_ecs;
    jotd_fmode3_probe_done = 0;
    jotd_bpl_gate_probe_done = 0;
}

void aga_set_ecs(aga_t *a, int on) { a->pf.ecs_mode = !!on; }
int  aga_get_ecs(const aga_t *a)   { return a->pf.ecs_mode; }

static void expand_fmodes(playfield_t *p)
{
    int i = p->fm * 4 + p->res;
    p->fetchunit = fetchunits[i];
    p->fetchstart_shift = fetchstarts[i];
    p->fetchstart = 1 << p->fetchstart_shift;
    p->maxplanes = 1 << fm_maxplanes[i];
}

/* E-UAE separates BPLCON0's Denise-visible change from the later Agnus DMA
   setup. The display register is recorded immediately by pf_write(); only
   these DMA-derived fields are delayed. */
static void apply_dma_bplcon0(playfield_t *p, uint16_t v)
{
    p->res = GET_RES(v);
    p->planes = GET_PLANES(v);
    expand_fmodes(p);
}

static void service_dma_bplcon0(playfield_t *p, int hpos)
{
    if (!p->dma_bplcon0_pending || hpos < p->dma_bplcon0_apply_hpos) return;
    apply_dma_bplcon0(p, p->dma_bplcon0_pending_value);
    p->dma_bplcon0_pending = 0;
}

static void calcdiw(playfield_t *p)
{
    /* Horizontal edges are kept in superhires units (4 per lores pixel) so the
       AGA sub-lores bits of DIWHIGH can be applied; see WinUAE calchdiw(). */
    int hstrt = (p->diwstrt & 0xFF) << 2, hstop = (p->diwstop & 0xFF) << 2;
    int vstrt = p->diwstrt >> 8,   vstop = p->diwstop >> 8;
    if (p->diwhigh_written) {
        /* a written DIWHIGH is masked before use (WinUAE DIWHIGH()) */
        int dh = p->diwhigh & ~(0x8000 | 0x4000 | 0x0080 | 0x0040);
        hstrt |= ((dh >> 5) & 1) << (8 + 2);
        hstop |= ((dh >> 13) & 1) << (8 + 2);
        /* AGA quarter-lores window edges: measurably wrong in this coordinate
           model (see docs/VALIDATION-FINDINGS.md), left off until calibrated
           hstrt |= (dh >> 3) & 3;
           hstop |= (dh >> 11) & 3; */
        vstrt |= (dh & 15) << 8;             /* AGA has 4 vertical high bits, ECS 3 */
        vstop |= ((dh >> 8) & 15) << 8;
    } else {
        hstop |= 0x100 << 2;
        if ((vstop & 0x80) == 0) vstop |= 0x100;
    }
    /* the output is hires, so window edges land on whole hires pixels
       (WinUAE masks with hbmask for the current output resolution) */
    hstrt &= ~1;
    hstop &= ~1;
    p->diw_hstart = hstrt;
    p->diw_hstop = hstop;
    p->diw_vstart = vstrt;
    p->diw_vstop = vstop;

    p->plfstrt = p->ddfstrt;
    if (p->plfstrt < HARD_DDF_START) p->plfstrt = HARD_DDF_START;
    p->plfstop = p->ddfstop;
    if (p->plfstop > HARD_DDF_STOP) p->plfstop = HARD_DDF_STOP;
}

/* Scroll delays in native pixels for a BPLCON1 value, using the line's fetch
   parameters. UAE: an undocumented offset applies for fetch starts that are
   not aligned to the fetch unit. */
static void delays_for(const playfield_t *p, uint16_t con1, int *d1out, int *d2out)
{
    int fetchwidth = 16 << p->line_fm;
    int d1 = (con1 & 0x0F) | ((con1 & 0x0C00) >> 6);
    int d2 = ((con1 >> 4) & 0x0F) | (((con1 >> 4) & 0x0C00) >> 6);
    int sh1 = (con1 >> 12) & 3;
    int sh2 = (con1 >> 8) & 3;
    int mask = (fetchwidth - 1) >> p->line_res;
    int subshift = RES_SHRES - p->line_res;
    int sub1 = subshift > 0 ? (sh1 >> subshift) : sh1;
    int sub2 = subshift > 0 ? (sh2 >> subshift) : sh2;

    /* E-UAE compute_toscr_delay_1(): the SHRES delay bits occupy the
       otherwise-zero low bits beneath the coarse scroll delay. Omitting them
       shifts odd/even plane groups relative to each other and can turn a
       correct palette into the wrong colour indices. */
    *d1out = (((d1 + p->line_delayoffset) & mask) << p->line_res) | sub1;
    *d2out = (((d2 + p->line_delayoffset) & mask) << p->line_res) | sub2;
}

static void compute_delays(playfield_t *p)
{
    int fetchwidth = 16 << p->fm;
    p->line_delayoffset = fetchwidth - (((p->plfstrt - HARD_DDF_START) & (p->fetchstart - 1)) << 1);
    delays_for(p, p->bplcon1, &p->line_delay1, &p->line_delay2);
}

/* An ECS game drives an ECS Denise, which has no LOCT bit, no colour bank and
   no sprite colour offset: a COLORxx write is one whole 12-bit colour and
   sprites take colours 16-31. This chipset is AGA and obeys all three, so an
   ECS game whose copper happens to set BPLCON3 bit 9 - Turrican II does,
   harmlessly on real ECS hardware, the lucky bastard - gets its palette
   written into the low nibbles and comes out at a sixteenth brightness. (Who
   sets LOCT in the sandbox is still open: the AGA here matches WinUAE's, and
   no OCS/ECS snapshot writes LOCT at all.) In ECS mode those registers are
   presented as their reset values instead, so a COLORxx write is one whole
   12-bit colour, there is no colour bank, and sprites take colours 16-31 - an
   ECS Denise.

   The mode is the player's choice (agaboot's ECS toggle, sent to $DFF1F2 as
   $5E00/$5E01 at sandbox entry), because nothing in a WHDLoad slave reliably
   says which chipset a game draws for. Nothing changes for AGA games: with
   this off the renderer is byte-identical, which tools/regress.py checks. */
#define BPLCON3_RESET 0x0C00
#define BPLCON4_RESET 0x0011

uint16_t pf_con3(const playfield_t *p) { return p->ecs_mode ? BPLCON3_RESET : p->bplcon3; }
uint16_t pf_con4(const playfield_t *p) { return p->ecs_mode ? BPLCON4_RESET : p->bplcon4; }

void pf_color_write(aga_t *a, int num, uint16_t v, int hpos)
{
    playfield_t *p = &a->pf;
    if (p->bplcon2 & BPLCON2_RDRAM) return;
    uint16_t con3 = pf_con3(p);
    int idx = ((con3 >> 13) & 7) * 32 + num;
    uint32_t r = (v >> 8) & 15, g = (v >> 4) & 15, b = v & 15;
    uint32_t cur = a->color[idx];
    uint32_t cr = cur >> 16, cg = (cur >> 8) & 0xFF, cb = cur & 0xFF;
    if (con3 & BPLCON3_LOCT) {
        cr = (cr & 0xF0) | r; cg = (cg & 0xF0) | g; cb = (cb & 0xF0) | b;
        /* LOCT writes replace only the low RGB nibbles. The AGA genlock
           sideband belongs to the MSB colour write and is not changed here. */
    } else {
        cr = r | (r << 4); cg = g | (g << 4); cb = b | (b << 4);
        a->color_genlock[idx] = (v & 0x8000u) ? 1u : 0u;
    }
    uint32_t val = (cr << 16) | (cg << 8) | cb;
    if (val == cur) return;
    a->color[idx] = val;
    aga_record_change(a, AGA_LINECHANGE_COLOR(idx), val);
    (void)hpos;
}

void pf_write(aga_t *a, uint32_t reg, uint16_t v, int hpos)
{
    playfield_t *p = &a->pf;
    if (reg >= BPL1PTH && reg < BPL1PTH + 32) {
        int n = (reg - BPL1PTH) >> 2;
        pf_sync(a, hpos);
        if ((reg & 2) == 0) p->bplpt[n] = (p->bplpt[n] & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16);
        else                p->bplpt[n] = (p->bplpt[n] & 0xFFFF0000u) | (v & 0xFFFE);
        return;
    }
    if (reg >= BPL1DAT && reg < BPL1DAT + 16) {
        p->bpldat[(reg - BPL1DAT) >> 1] = v;
        return;
    }
    switch (reg) {
    case BPLCON0:
        if (p->bplcon0 == v) return;
        pf_sync(a, hpos);
        {
            int old_dma_planes = p->planes;
            int bpl_dma_live = aga_dma_enabled(a, DMAF_BPLEN);
            p->bplcon0 = v;
            /* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_BPLCON0
             * E-UAE's Denise/Agnus split matters while a fetch stream is live.
             * JOTD's arcade ports program BPLCON0 while all DMA is disabled,
             * then enable DMA much later. With BPL DMA stopped, converge the
             * state now instead of carrying a synthetic 4-cck delay until a
             * future DMA enable. */
            if (!bpl_dma_live) {
                apply_dma_bplcon0(p, v);
                p->dma_bplcon0_pending = 0;
            } else {
                p->dma_bplcon0_pending_value = v;
                p->dma_bplcon0_apply_hpos = hpos + 4 + (old_dma_planes == 8 ? 1 : 0);
                p->dma_bplcon0_pending = 1;
            }
        }
        spr_expand_res(a);
        aga_record_change(a, BPLCON0, v);
        break;
    case BPLCON1:
        if (p->bplcon1 == v) return;
        pf_sync(a, hpos);
        p->bplcon1 = v;
        if (p->fetch_active) compute_delays(p);   /* takes effect for following blocks */
        aga_record_change(a, BPLCON1, v);
        break;
    case BPLCON2:
        if (p->bplcon2 == v) return;
        p->bplcon2 = v;
        aga_record_change(a, BPLCON2, v);
        break;
    case BPLCON3:
        if (p->bplcon3 == v) return;
        p->bplcon3 = v;
        spr_expand_res(a);
        aga_record_change(a, BPLCON3, v);
        break;
    case BPLCON4:
        if (p->bplcon4 == v) return;
        p->bplcon4 = v;
        aga_record_change(a, BPLCON4, v);
        break;
    case BPL1MOD:
        pf_sync(a, hpos);
        p->bpl1mod = (int16_t)(v & 0xFFFE);
        break;
    case BPL2MOD:
        pf_sync(a, hpos);
        p->bpl2mod = (int16_t)(v & 0xFFFE);
        break;
    case DIWSTRT:
        p->diwstrt = v; p->diwhigh_written = 0; calcdiw(p);
        aga_record_change(a, DIWSTRT, ((uint32_t)(p->diw_hstart & 0xFFFF) << 16) | (uint32_t)(p->diw_hstop & 0xFFFF));
        break;
    case DIWSTOP:
        p->diwstop = v; p->diwhigh_written = 0; calcdiw(p);
        aga_record_change(a, DIWSTOP, ((uint32_t)(p->diw_hstart & 0xFFFF) << 16) | (uint32_t)(p->diw_hstop & 0xFFFF));
        break;
    case DIWHIGH:
        p->diwhigh = v; p->diwhigh_written = 1; calcdiw(p);
        aga_record_change(a, DIWHIGH, ((uint32_t)(p->diw_hstart & 0xFFFF) << 16) | (uint32_t)(p->diw_hstop & 0xFFFF));
        break;
    case DDFSTRT:
        pf_sync(a, hpos);
        p->ddfstrt = v & 0xFE; calcdiw(p);
        break;
    case DDFSTOP:
        pf_sync(a, hpos);
        p->ddfstop = v & 0xFE; calcdiw(p);
        break;
    case FMODE:
        pf_sync(a, hpos);
        p->fmode = v;
        p->fm = (v & 3) == 3 ? 2 : (v & 3) ? 1 : 0;
        expand_fmodes(p);
        spr_expand_res(a);
        break;
    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */
void pf_vsync(aga_t *a)
{
    /* FRF_AGA_UP104_SAFE_VDIW:
       frame start resets both the raw DIW latch and visible gate. */
    a->pf.vdiw = 0;
    a->pf.vdiw_ff = 0;
}

/* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_SYNC
 * DMACON can be enabled long after display registers were programmed.
 * Before the first BPL DMA opportunity, make Agnus-side derived fields agree
 * with the live BPLCON0 state. Mid-line delayed-write semantics stay intact. */
void pf_dma_enable_sync(aga_t *a)
{
    playfield_t *p = &a->pf;
    apply_dma_bplcon0(p, p->bplcon0);
    p->dma_bplcon0_pending = 0;
    calcdiw(p);
}

void pf_start_line(aga_t *a)
{
    playfield_t *p = &a->pf;
    /* A BPLCON0 write in the final Agnus-delay cycles of the previous line
       completes at the corresponding early cycle of this line. */
    if (p->dma_bplcon0_pending && p->dma_bplcon0_apply_hpos >= a->maxhpos)
        p->dma_bplcon0_apply_hpos -= a->maxhpos;
    /* FRF_AGA_UP104_SAFE_VDIW:
       DIWSTRT may occur while VBLANK is active. Hardware keeps that SET
       latched; VBLANK suppresses visibility only. */
    if (a->vpos == p->diw_vstop)  p->vdiw_ff = 0;
    if (a->vpos == p->diw_vstart) p->vdiw_ff = 1;
    p->vdiw = p->vdiw_ff && a->vpos >= a->vblank_end;

    p->fetch_active = 0;
    p->fetch_done = 0;
    p->fetch_cycle = 0;
    p->fetch_last_hpos = 0;
    p->line_plfleft = -1;
    p->line_blocks = 0;
    p->line_planes = 0;
    p->fetch_start_h = -1;
    memset(p->blockdata, 0, sizeof p->blockdata);

    memcpy(line_color, a->color, sizeof line_color);
    line_bplcon0 = p->bplcon0; line_bplcon1 = p->bplcon1; line_bplcon2 = p->bplcon2;
    line_bplcon3 = pf_con3(p); line_bplcon4 = pf_con4(p);
    line_diw_hstart = p->diw_hstart; line_diw_hstop = p->diw_hstop;
}

/* OR `nbits` bits of `data` (MSB first, data right-aligned) into plane `plane` at bit position `pos`. */
static inline void put_bits(playfield_t *p, int plane, int pos, uint64_t data, int nbits)
{
    if (pos < 0 || pos + nbits > AGA_PLANE_WORDS * 64) return;
    int w = pos >> 6, b = pos & 63;
    /* place the nbits field so that its MSB lands at bit position `pos` counting from the MSB of word w */
    int shift = 64 - b - nbits;   /* left shift within word w */
    if (shift >= 0) {
        p->bits[plane][w] |= data << shift;
    } else {
        p->bits[plane][w] |= data >> (-shift);
        if (w + 1 < AGA_PLANE_WORDS) p->bits[plane][w + 1] |= data << (64 + shift);
    }
}

/* FRF_AGA_FETCH_CORE_V0_1_7
 * AGA 32/64-bit bitplane DMA is NOT an ordinary unaligned CPU long/quad read.
 * Lisa/Alice preserve word alignment inside the wide fetch.  WinUAE models
 * this in fetch32_bpl()/fetch64():
 *   - any 32-bit fetch from address ...+2 duplicates the selected 16-bit word;
 *   - FMODE=2 (BPAGEM without BPL32) duplicates the aligned high 16-bit word;
 *   - a 64-bit fetch starting at +4 repeats the same longword;
 *   - a 64-bit fetch starting at +2 duplicates each selected word.
 * Treating these as chip_rd32/chip_rd64 made legal word-aligned AGA screens
 * turn into repeated vertical blocks/false colours even though their palette
 * registers were correct.
 */
static uint32_t fetch_bpl32_aga(const aga_t *a, uint32_t addr, unsigned fmode_bpl)
{
    uint32_t pm = addr & ~3u;
    uint32_t v = chip_rd32(a, pm);
    if (addr & 2u) {
        uint32_t w = v & 0x0000FFFFu;
        return w | (w << 16);
    }
    if (fmode_bpl & 2u) { /* FMODE=2 page mode */
        uint32_t w = v & 0xFFFF0000u;
        return w | (w >> 16);
    }
    return v;
}

static uint64_t fetch_bpl64_aga(const aga_t *a, uint32_t addr)
{
    uint32_t pm = addr & ~7u;
    uint32_t pm1 = (addr & 4u) ? pm + 4u : pm;
    uint32_t pm2 = pm + 4u;
    if (addr & 4u) pm2 = pm + 4u; /* hardware repeats the second long */

    if (addr & 2u) {
        uint32_t v1 = chip_rd32(a, pm1) & 0x0000FFFFu;
        uint32_t v2 = chip_rd32(a, pm2) & 0x0000FFFFu;
        v1 |= v1 << 16;
        v2 |= v2 << 16;
        return ((uint64_t)v1 << 32) | v2;
    }
    return ((uint64_t)chip_rd32(a, pm1) << 32) | chip_rd32(a, pm2);
}

/* FRF_AGA_FETCH_POINTER_CORE_V0_1_9
 * AGA wide-fetch pointer advancement has the same alignment semantics as the
 * fetch itself.  WinUAE's addmodfm() aligns the current address down to the
 * active 32/64-bit fetch boundary before adding the consumed fetch width and
 * (on the final access) the modulo.  Keeping the low +2/+4 offset forever
 * (plain += 4/8) makes the first corrected wide fetch look right but every
 * following block walk the wrong words, producing repeated columns / false
 * palette indices.
 */
static uint32_t advance_bpl_ptr_aga(uint32_t pt, int add, int mod, unsigned fmode_bpl)
{
    if (fmode_bpl == 1u || fmode_bpl == 2u)
        return (pt & ~3u) + (uint32_t)(add + mod);
    if (fmode_bpl == 3u)
        return (pt & ~7u) + (uint32_t)(add + mod);
    return pt + (uint32_t)(add + mod);
}

static void fetch_plane(aga_t *a, int plane, int block, int hpos)
{
    playfield_t *p = &a->pf;
    uint32_t addr = p->bplpt[plane];
    uint64_t data;
    int add;
    aga_trace_at(a, hpos, AGA_DMA_BITPLANE, (uint16_t)(BPL1DAT + 2 * plane), addr);
    switch (p->line_fm) {
    case 0:
        data = chip_rd16(a, addr);
        add = 2;
        break;
    case 1:
        data = fetch_bpl32_aga(a, addr, (unsigned)p->line_fmode_bpl);
        add = 4;
        break;
    default:
        data = fetch_bpl64_aga(a, addr);
        add = 8;
        break;
    }
    if (!jotd_fmode3_probe_done && plane == 0 && block == 0 &&
        p->line_fm == 2 && p->line_planes == 7 &&
        p->ddfstrt == 0x0038 && p->ddfstop == 0x00C0 &&
        p->bpl1mod == -8 && p->bpl2mod == -8) {
        jotd_fmode3_probe_done = 1;
        aga_logf(a,
            "AGA JOTD FMODE3 fetch: v=%d BPL1=%08x data=%08x%08x chipmask=%08x DMACON=%04x BPLCON0=%04x BPLCON1=%04x",
            a->vpos, addr, (uint32_t)(data >> 32), (uint32_t)data,
            a->chipmask, a->dmacon, p->bplcon0, p->bplcon1);
        if (addr > a->chipmask)
            aga_logf(a, "AGA JOTD FMODE3 ERROR: BPL1 pointer is outside virtual Chip RAM");
        else if (!data)
            aga_logf(a, "AGA JOTD FMODE3 WARNING: first BPL1 64-bit DMA fetch is ZERO");
        else
            aga_logf(a, "AGA JOTD FMODE3 OK: first BPL1 64-bit DMA fetch is NONZERO");
    }
    p->bplpt[plane] = advance_bpl_ptr_aga(addr, add, 0, (unsigned)p->line_fmode_bpl);
    if (data) a->pf_data_nonzero = 1;   /* diagnostics: did anything ever fill the bitplanes */
    if (block < AGA_MAX_BLOCKS) p->blockdata[block][plane] = data;
}

/* A block is complete once its last slot (plane 1) has passed: the scroll delay
   in effect at that moment applies to the whole block (UAE beginning_of_plane_block). */
static void complete_block(playfield_t *p, int block)
{
    if (block < AGA_MAX_BLOCKS) {
        p->blockdelay1[block] = (int16_t)p->line_delay1;
        p->blockdelay2[block] = (int16_t)p->line_delay2;
    }
}

static void build_bits(playfield_t *p)
{
    int W = 16 << p->line_fm;
    int nb = p->line_blocks < AGA_MAX_BLOCKS ? p->line_blocks : AGA_MAX_BLOCKS;
    memset(p->bits, 0, sizeof p->bits);
    for (int b = 0; b < nb; b++)
        for (int pl = 0; pl < p->line_planes; pl++)
#ifdef SCROLL_MODEL_BLOCK
            put_bits(p, pl, b * W + ((pl & 1) ? p->blockdelay2[b] : p->blockdelay1[b]), p->blockdata[b][pl], W);
#else
            put_bits(p, pl, b * W, p->blockdata[b][pl], W);
#endif
}

static void add_modulos(aga_t *a)
{
    playfield_t *p = &a->pf;
    int scan2 = (p->line_fmode & FMODE_BSCAN2) != 0;
    int scan2_mod = 0;
    if (scan2)
        scan2_mod = ((((p->diwstrt >> 8) ^ a->vpos) & 1) != 0) ? p->bpl2mod : p->bpl1mod;

    for (int i = 0; i < p->line_planes; i++) {
        /* E-UAE add_modulos()/one_fetch_cycle(): BSCAN2 selects one modulo for
           every plane from display-start/beam parity; without BSCAN2 the
           normal odd/even plane BPL1MOD/BPL2MOD split applies. */
        int mod = scan2 ? scan2_mod : ((i & 1) ? p->bpl2mod : p->bpl1mod);
        p->bplpt[i] = advance_bpl_ptr_aga(p->bplpt[i], 0, mod, (unsigned)p->line_fmode_bpl);
    }
}

void pf_sync(aga_t *a, int hpos)
{
    playfield_t *p = &a->pf;
    if (hpos > a->maxhpos) hpos = a->maxhpos;
    int h = p->fetch_last_hpos;
    if (h >= hpos) return;

    for (; h < hpos; h++) {
        service_dma_bplcon0(p, h);
        if (!p->fetch_active && !p->fetch_done) {
            if (!jotd_bpl_gate_probe_done && h == p->plfstrt + DDF_OFFSET &&
                GET_PLANES(p->bplcon0) == 7 && (p->fmode & 3) == 3 &&
                p->ddfstrt == 0x0038 && p->ddfstop == 0x00C0) {
                jotd_bpl_gate_probe_done = 1;
                aga_logf(a,
                    "AGA JOTD BPLDMA gate: v=%d vdiw=%d regplanes=%d dmaplanes=%d maxplanes=%d DMACON=%04x BPLCON0=%04x BPLCON1=%04x FMODE=%04x DDF=%04x/%04x pending=%d",
                    a->vpos, p->vdiw, GET_PLANES(p->bplcon0), p->planes, p->maxplanes,
                    a->dmacon, p->bplcon0, p->bplcon1, p->fmode, p->ddfstrt, p->ddfstop,
                    p->dma_bplcon0_pending);
            }
            if (h == p->plfstrt + DDF_OFFSET && p->vdiw && aga_dma_enabled(a, DMAF_BPLEN) && p->planes > 0) {
                if (p->planes > p->maxplanes) { p->fetch_done = 1; continue; }
                /* FRF_AGA_JOTD_BITPLANE_COHERENCY_V0_1
                 * Pair the CPU0 producer with CPU3 DMA consumption.  The ARM
                 * mappings are inner-shareable/cached; this fence orders the
                 * following DMA loads after all coherence traffic visible at
                 * the start of this scanline. */
                __atomic_thread_fence(__ATOMIC_ACQUIRE);
                p->fetch_active = 1;
                p->fetch_cycle = 0;
                p->fetch_start_h = h;
                p->line_res = p->res;
                p->line_fm = p->fm;
                p->line_fmode_bpl = p->fmode & 3;
                p->line_fmode = p->fmode;
                p->line_planes = p->planes;
                p->line_bplcon0 = p->bplcon0;
                compute_delays(p);
                int stop = p->plfstop < p->plfstrt ? p->plfstrt : p->plfstop;
                int len = stop - p->plfstrt;
                len = (len + p->fetchunit - 1) & ~(p->fetchunit - 1);
                p->fetch_total = len + p->fetchunit;
                /* fetching may continue into the first cycles of the next line (the
                   hardware keeps the DMA slots going); those fetches only advance the
                   pointers, the data lands in horizontal blanking */
                /* the first block becomes visible once plane 1 (fetched last in the
                   block, at cycle maxplanes-1) has been fetched */
                p->line_plfleft = h + p->maxplanes;
            }
        }
        if (p->fetch_active) {
            int cyc = p->fetch_cycle & (p->fetchstart - 1);
            int block = p->fetch_cycle >> p->fetchstart_shift;
            if (cyc < p->maxplanes) {
                int plane;
                if (p->maxplanes == 8) plane = seq8[cyc];
                else if (p->maxplanes == 4) plane = seq4[cyc];
                else plane = seq2[cyc];
                if (plane <= p->line_planes)
                    fetch_plane(a, plane - 1, block, h);
                if (cyc == p->maxplanes - 1) complete_block(p, block);
            }
            p->fetch_cycle++;
            if (p->fetch_cycle < p->fetch_total && h + 1 >= a->maxhpos) {
                /* end of line: complete the remaining slots of the last block now */
                while (p->fetch_cycle < p->fetch_total) {
                    int cyc2 = p->fetch_cycle & (p->fetchstart - 1);
                    if (cyc2 < p->maxplanes) {
                        int plane2 = (p->maxplanes == 8) ? seq8[cyc2] : (p->maxplanes == 4) ? seq4[cyc2] : seq2[cyc2];
                        if (plane2 <= p->line_planes) fetch_plane(a, plane2 - 1, p->fetch_cycle >> p->fetchstart_shift, h);
                        if (cyc2 == p->maxplanes - 1) complete_block(p, p->fetch_cycle >> p->fetchstart_shift);
                    }
                    p->fetch_cycle++;
                }
            }
            if (p->fetch_cycle >= p->fetch_total) {
                p->fetch_active = 0;
                p->fetch_done = 1;
                p->line_blocks = (p->fetch_cycle + p->fetchstart - 1) >> p->fetchstart_shift;
                add_modulos(a);
            }
        }
    }
    p->fetch_last_hpos = hpos;
}

int pf_is_bpl_slot(aga_t *a, int hpos)
{
    playfield_t *p = &a->pf;
    pf_sync(a, hpos + 1);
    if (p->fetch_start_h < 0) return 0;
    int rel = hpos - p->fetch_start_h;
    if (rel < 0 || rel >= p->fetch_total) return 0;
    int cyc = rel & (p->fetchstart - 1);
    if (cyc >= p->maxplanes) return 0;
    int plane = (p->maxplanes == 8) ? seq8[cyc] : (p->maxplanes == 4) ? seq4[cyc] : seq2[cyc];
    return plane <= p->line_planes;
}

/* -------------------------------------------------------------------------- */
/* planar -> chunky for the whole line, odd and even planes separately (both keep
   their original bit positions: odd planes in bits 0,2,4,6, even in 1,3,5,7) */
static void planar_to_chunky(playfield_t *p, int nbits)
{
    int planes = p->line_planes;
    int words = (nbits + 63) >> 6;
    for (int w = 0; w < words; w++) {
        uint64_t b[8];
        for (int pl = 0; pl < 8; pl++) b[pl] = (pl < planes) ? p->bits[pl][w] : 0;
        for (int j = 0; j < 64; j++) {
            int sh = 63 - j;
            unsigned vo = 0, ve = 0;
            for (int pl = 0; pl < planes; pl += 2) vo |= (unsigned)((b[pl] >> sh) & 1) << pl;
            for (int pl = 1; pl < planes; pl += 2) ve |= (unsigned)((b[pl] >> sh) & 1) << pl;
            p->idx_odd[w * 64 + j] = (uint8_t)vo;
            p->idx_even[w * 64 + j] = (uint8_t)ve;
        }
    }
}

static inline uint32_t ehb_half(uint32_t c) { return (c >> 1) & 0x7F7F7F; }

typedef struct segstate {
    uint16_t con0, con1, con2, con3, con4;
    int diw_hstart, diw_hstop;
    uint32_t ham_last;
} segstate_t;

static void apply_change(segstate_t *s, const linechange_t *c)
{
    if (AGA_LINECHANGE_IS_COLOR(c->reg)) {
        line_color[AGA_LINECHANGE_COLOR_INDEX(c->reg)] = c->value;
        return;
    }
    switch (c->reg) {
    case BPLCON0: s->con0 = (uint16_t)c->value; break;
    case BPLCON1: s->con1 = (uint16_t)c->value; break;
    case BPLCON2: s->con2 = (uint16_t)c->value; break;
    case BPLCON3: s->con3 = (uint16_t)c->value; break;
    case BPLCON4: s->con4 = (uint16_t)c->value; break;
    case DIWSTRT: case DIWSTOP: case DIWHIGH:
        /* The event carries the window calculated at the moment of the write.
           Using a->pf here used the end-of-line/final DIW state for every
           segment and could blank/crop the part of the line before the write. */
        s->diw_hstart = (int)((c->value >> 16) & 0xFFFFu);
        s->diw_hstop  = (int)(c->value & 0xFFFFu);
        break;
    default: break;
    }
}

static const int pf2of_table[8] = { 0, 2, 4, 8, 16, 32, 64, 128 };

/* Collision detection, after WinUAE (do_sprite_collisions, do_playfield_
   collisions): worked out on the pixels as they are drawn, inside the display
   window, so it sees the same scroll and window as the picture. Sprite groups:
   group g is sprite 2g, plus sprite 2g+1 if CLXCON enables it. */

/* groups present at a pixel (bit g) -> sprite-to-sprite bits 9..14 of CLXDAT */
static const uint16_t clx_sprspr[16] = {
    0, 0, 0, 0x0200, 0, 0x0400, 0x1000, 0x1600,
    0, 0x0800, 0x2000, 0x2A00, 0x4000, 0x4C00, 0x7000, 0x7E00
};

static void render_segment(aga_t *a, segstate_t *s, uint32_t *out, int hx0, int hx1, int have_data, int x0, int nbits, int clx)
{
    playfield_t *p = &a->pf;
    int res = p->line_res;
    int planes = p->line_planes;
    uint16_t con0 = s->con0;
    int ecsena = con0 & BPLCON0_ECSENA;
    int ham = (con0 & BPLCON0_HAM) && (planes == 6 || planes == 8);
    int dpf = (con0 & BPLCON0_DPF) != 0;
    int ehb = (planes == 6) && !ham && !dpf && !(s->con2 & BPLCON2_KILLEHB);
    int brdrblnk = ecsena && (s->con3 & BPLCON3_BRDRBLNK);
    int brdsprt = ecsena && (s->con3 & BPLCON3_BRDSPRT);
    int pf1p = s->con2 & 7, pf2p = (s->con2 >> 3) & 7;
    int pf2pri = s->con2 & BPLCON2_PF2PRI;
    int pf2of = pf2of_table[(s->con3 >> 10) & 7];
    unsigned xorv = (s->con4 >> 8) & 0xFF;
    /* BPLCON4 sprite colour bases, as WinUAE applies them (drawing.cpp): the
       high nibble is the base for a pixel of the lower-numbered (even) sprite of
       a pair, the low nibble for the higher-numbered (odd) one and for an
       attached pair. With the reset value $11 both are 16, which is why only
       games that program BPLCON4 (Turrican 2: $65) tell the two apart. */
    int base_even = ((s->con4 >> 4) & 15) << 4, base_odd = (s->con4 & 15) << 4;
    int vdiw = p->vdiw;
    const uint8_t *spr = a->sprline_valid ? a->sprline : NULL;
    int sprlen = (int)sizeof a->sprline;
    int d1 = 0, d2 = 0;
#ifndef SCROLL_MODEL_BLOCK
    if (have_data) delays_for(p, s->con1, &d1, &d2);
#endif
    /* collisions: enabled planes, the value they must match, sprites taking part */
    const uint8_t *sprm = (clx && a->sprclx_valid) ? a->sprclx : NULL;
    unsigned c_ena = 0, c_match = 0, c_emask = 0;
    uint16_t cx = a->clxdat & 1;                      /* bit 0 already found this frame: skip that search */
    if (clx) {
        c_ena   = ((a->clxcon >> 6) & 0x3F) | (a->clxcon2 & 0xC0);
        c_match = ((a->clxcon & 0x3F) | ((a->clxcon2 & 3) << 6)) & c_ena;
        c_emask = 0x55 | ((a->clxcon >> 11) & 0x02) | ((a->clxcon >> 10) & 0x08)   /* ENSP1, ENSP3 */
                       | ((a->clxcon >> 9) & 0x20) | ((a->clxcon >> 8) & 0x80);    /* ENSP5, ENSP7 */
    }

    for (int hx = hx0; hx < hx1; hx++) {
        int hires = AGA_OUT_LEFT_LORES * 2 + hx;      /* hires hardware coordinate (hpos*4 based) */
        /* DIW register values are offset by DIW_DDF_OFFSET-1 from hardware coordinates (UAE) */
        int sx = hires << 1;                          /* superhires hardware coordinate */
        int in_window = vdiw && sx >= s->diw_hstart + DIW_HW_OFFSET * 4
                             && sx <  s->diw_hstop  + DIW_HW_OFFSET * 4;
        uint32_t col;
        int pfv = 0;             /* playfield pixel value for sprite priority */
        int front_pf = 2;        /* which playfield is in front for priority (1 or 2) */
        unsigned v = 0;          /* raw bitplane value, bit n = plane n+1 */
        int fetched = 0;         /* the pixel lies in fetched bitplane data */

        if (in_window) {
            if (have_data) {
                int nx = (res == 0) ? (hires >> 1) : (res == 1) ? hires : (hires << 1);
                int i1 = nx - x0 - d1, i2 = nx - x0 - d2;
                if (i1 >= 0 && i1 < nbits) { v = p->idx_odd[i1]; fetched = 1; }
                if (i2 >= 0 && i2 < nbits) { v |= p->idx_even[i2]; fetched = 1; }
            }
            if (dpf) {
                unsigned v1 = (v & 1) | ((v >> 1) & 2) | ((v >> 2) & 4) | ((v >> 3) & 8);
                unsigned v2 = ((v >> 1) & 1) | ((v >> 2) & 2) | ((v >> 3) & 4) | ((v >> 4) & 8);
                /* AGA BPLCON4.BPLAM is an 8-bit XOR on the final bitplane
                   colour-table address.  It is not limited to PF1's low
                   nibble: upper mask bits deliberately let Copper swap whole
                   16/32-colour banks, and PF2 is affected after PF2OF forms
                   its address too. */
                unsigned pf1idx = aga_bplam_index(v1, xorv);
                unsigned pf2idx = aga_bplam_index(v2 + pf2of, xorv);
                if (pf2pri) {
                    if (v2)      { col = line_color[pf2idx]; pfv = v2; front_pf = 2; }
                    else if (v1) { col = line_color[pf1idx]; pfv = v1; front_pf = 1; }
                    else         { col = line_color[0]; }
                } else {
                    if (v1)      { col = line_color[pf1idx]; pfv = v1; front_pf = 1; }
                    else if (v2) { col = line_color[pf2idx]; pfv = v2; front_pf = 2; }
                    else         { col = line_color[0]; }
                }
            } else if (ham) {
                pfv = v;
                if (planes == 8) {
                    switch (v & 3) {
                    case 0: s->ham_last = line_color[(v >> 2) ^ (xorv >> 2)]; break;
                    case 1: s->ham_last = (s->ham_last & 0xFFFF03) | (v & 0xFC); break;
                    case 2: s->ham_last = (s->ham_last & 0x03FFFF) | ((v & 0xFC) << 16); break;
                    default: s->ham_last = (s->ham_last & 0xFF03FF) | ((v & 0xFC) << 8); break;
                    }
                } else {
                    unsigned n = v & 15;
                    unsigned n8 = n | (n << 4);
                    switch (v & 0x30) {
                    case 0x00: s->ham_last = line_color[n ^ (xorv & 15)]; break;
                    case 0x10: s->ham_last = (s->ham_last & 0xFFFF00) | n8; break;
                    case 0x20: s->ham_last = (s->ham_last & 0x00FFFF) | (n8 << 16); break;
                    default:   s->ham_last = (s->ham_last & 0xFF00FF) | (n8 << 8); break;
                    }
                }
                col = s->ham_last;
            } else if (ehb) {
                pfv = v;
                if (v & 0x20) col = ehb_half(line_color[(v & 31) ^ (xorv & 31)]);
                else          col = line_color[v ^ xorv];
            } else {
                pfv = v;
                col = line_color[(v ^ xorv) & 255];
            }
        } else {
            col = brdrblnk ? 0 : line_color[0];
        }

        /* sprites */
        if (spr && (in_window || brdsprt)) {
            int sx = hires << 1;
            uint8_t sv = (sx >= 0 && sx < sprlen) ? spr[sx] : 0;
            if (sv) {
                int pair = sv >> 6;
                int visible;
                if (!in_window || pfv == 0) visible = 1;
                else if (front_pf == 1)     visible = pair < pf1p;
                else                        visible = pair < pf2p;
                if (visible) {
                    int c = sv & 15;
                    int idx = (sv & 0x10) ? (base_odd + c) : (((sv & 0x20) ? base_odd : base_even) + pair * 4 + c);
                    col = line_color[idx & 255];
                }
            }
        }

        /* once bit 0 is found, only pixels with a sprite on them need looking at */
        unsigned m = (clx && sprm) ? (sprm[sx] & c_emask) : 0;
        if (clx && in_window && (m || !(cx & 1))) {
            unsigned g = 0;                           /* sprite groups with a pixel here */
            if (m) {
                m = (m | (m >> 1)) & 0x55;
                g = (m & 1) | ((m >> 1) & 2) | ((m >> 2) & 4) | ((m >> 3) & 8);
                cx |= clx_sprspr[g];
            }
            if (fetched) {
                /* a disabled plane always matches, so no enabled plane = always */
                unsigned d = (v ^ c_match) & c_ena;
                unsigned even_ok = !(d & 0xAA), odd_ok = !(d & 0x55);
                if (even_ok && odd_ok) cx |= 0x0001;  /* even to odd bitplanes */
                if (!dpf) odd_ok &= even_ok;          /* one playfield: the odd bits need all planes (WinUAE) */
                if (odd_ok)  cx |= (uint16_t)(g << 1);
                if (even_ok) cx |= (uint16_t)(g << 5);
            }
        }
        out[hx] = col;
    }
    a->clxdat |= cx;
}

void pf_finish_line(aga_t *a)
{
    playfield_t *p = &a->pf;
    int y = a->vpos - AGA_OUT_TOP;
    if (!a->cfg.render || y < 0 || y >= AGA_OUT_H) return;

    uint32_t *out = a->linebuf;
    int have_data = p->line_blocks > 0 && p->line_planes > 0;
    if (have_data && p->vdiw) a->display_lines++;
    /* a skipped frame draws nothing - unless collisions switched on during
       it, since CLXDAT is computed by the renderer below */
    if (a->frame_skip && !(a->cfg.collisions || a->clx_frames > 0)) return;
    int nbits = 0, x0 = 0;
    if (have_data) {
        int W = 16 << p->line_fm;
        nbits = p->line_blocks * W + 64;
        if (nbits > AGA_PLANE_WORDS * 64) nbits = AGA_PLANE_WORDS * 64;
        build_bits(p);
        planar_to_chunky(p, nbits);
        x0 = (p->line_plfleft * 2 + DISP_OFFSET) << p->line_res;
    }

    int clx = a->cfg.collisions || a->clx_frames > 0;
    int any_sprite = 0;
    for (int i = 0; i < AGA_MAX_SPRITES; i++) if (a->spr[i].armed) { any_sprite = 1; break; }
    if (any_sprite) spr_build_line(a, clx); else a->sprline_valid = a->sprclx_valid = 0;

    segstate_t s;
    s.con0 = line_bplcon0; s.con1 = line_bplcon1; s.con2 = line_bplcon2;
    s.con3 = line_bplcon3; s.con4 = line_bplcon4;
    s.diw_hstart = line_diw_hstart; s.diw_hstop = line_diw_hstop;
    s.ham_last = line_color[0];

    int hx = 0;
    for (int i = 0; i < a->nchanges; i++) {
        const linechange_t *c = &a->changes[i];
        /* change takes effect at lores pixel 2*hpos plus a small pipeline delay
           (BPLCON1 scroll changes are seen 3 lores pixels after the write, colours 1) */
        int chx = (int)c->hpos * 4 + (c->reg == BPLCON1 ? 6 : 2) - AGA_OUT_LEFT_LORES * 2;
        if (chx > AGA_OUT_W) chx = AGA_OUT_W;
        if (chx > hx) {
            render_segment(a, &s, out, hx, chx, have_data, x0, nbits, clx);
            hx = chx;
        }
        apply_change(&s, c);
    }
    if (hx < AGA_OUT_W) render_segment(a, &s, out, hx, AGA_OUT_W, have_data, x0, nbits, clx);
    /* the finished line, in one pass: an external page (uncached HVS memory)
       gets the alpha byte the plane's alpha mode needs, the core's own
       frames stay byte-identical to what the host tests compare */
    uint32_t *row = a->fb[a->fb_cur] + y * AGA_OUT_W;
    if (a->fb_external) { for (int i = 0; i < AGA_OUT_W; i++) row[i] = out[i] | 0xFF000000u; }
    else memcpy(row, out, sizeof a->linebuf);
}
