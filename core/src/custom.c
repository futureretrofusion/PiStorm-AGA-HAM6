/* AGA-PISTORM — custom chip register file, beam counters, DMA/interrupt control.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "internal.h"

/* --------------------------------------------------------------------------
 * logging
 * ------------------------------------------------------------------------ */
#ifdef AGA_PISTORM
extern void vkprintf(const char *format, va_list args);
void aga_logf(aga_t *a, const char *fmt, ...)
{
    va_list ap;
    (void)a;
    va_start(ap, fmt);
    vkprintf(fmt, ap);
    va_end(ap);
}
#else
void aga_logf(aga_t *a, const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    if (!a->log) return;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    a->log(a->log_user, buf);
}
#endif

void aga_set_log(aga_t *a, void (*log)(void *user, const char *msg), void *user)
{
    a->log = log;
    a->log_user = user;
}

/* --------------------------------------------------------------------------
 * lifecycle
 * ------------------------------------------------------------------------ */
aga_t *aga_create(uint8_t *chipram, uint32_t chipram_size, const aga_config_t *cfg)
{
    aga_t *a = (aga_t *)calloc(1, sizeof *a);
    if (!a) return NULL;
    dsk_create(a);
#ifdef AGA_TRACE
    a->trace[0] = (aga_slot_t *)calloc((size_t)AGA_MAX_VPOS * AGA_MAX_HPOS, sizeof(aga_slot_t));
    a->trace[1] = (aga_slot_t *)calloc((size_t)AGA_MAX_VPOS * AGA_MAX_HPOS, sizeof(aga_slot_t));
#endif
    if (cfg) a->cfg = *cfg;
    else { a->cfg.blit_immediate = 1; a->cfg.render = 1; }
    a->chip = chipram;
    a->fb[0] = a->fb_store[0];
    a->fb[1] = a->fb_store[1];
    /* chipram_size must be a power of two */
    a->chipmask = chipram_size - 1;
    a->ntsc = a->cfg.ntsc;
    aga_reset(a);
    return a;
}

void aga_destroy(aga_t *a)
{
    dsk_destroy(a);
#ifdef AGA_TRACE
    free(a->trace[0]);
    free(a->trace[1]);
#endif
    free(a);
}

void aga_set_hooks(aga_t *a, const aga_hooks_t *hooks)
{
    if (hooks) a->hooks = *hooks;
    else memset(&a->hooks, 0, sizeof a->hooks);
}

static void set_geometry(aga_t *a)
{
    if (a->ntsc) {
        a->maxhpos = AGA_NTSC_HPOS;
        a->maxvpos = AGA_NTSC_VPOS;
        a->vblank_end = AGA_VBLANK_END_NTSC;
        a->sprite_vblank = AGA_SPRITE_VBLANK_NTSC;
    } else {
        a->maxhpos = AGA_PAL_HPOS;
        a->maxvpos = AGA_PAL_VPOS;
        a->vblank_end = AGA_VBLANK_END_PAL;
        a->sprite_vblank = AGA_SPRITE_VBLANK_PAL;
    }
}

void aga_reset(aga_t *a)
{
    uint8_t *chip = a->chip;
    uint32_t mask = a->chipmask;
    aga_config_t cfg = a->cfg;
    aga_hooks_t hooks = a->hooks;
    void (*log)(void *, const char *) = a->log;
    void *log_user = a->log_user;
    disk_t *dsk = a->dsk;
    int ecs = a->pf.ecs_mode;    /* chipset personality: see below */
    uint32_t *fb0 = a->fb[0], *fb1 = a->fb[1]; int fbx = a->fb_external;
#ifdef AGA_TRACE
    aga_slot_t *tr0 = a->trace[0], *tr1 = a->trace[1];
#endif

    memset(a, 0, sizeof *a);
    a->dsk = dsk;
    a->fb[0] = fb0 ? fb0 : a->fb_store[0];
    a->fb[1] = fb1 ? fb1 : a->fb_store[1];
    a->fb_external = fbx;
#ifdef AGA_TRACE
    a->trace[0] = tr0;
    a->trace[1] = tr1;
#endif
    dsk_reset(a);
    a->chip = chip;
    a->chipmask = mask;
    a->cfg = cfg;
    a->hooks = hooks;
    a->log = log;
    a->log_user = log_user;
    a->ntsc = cfg.ntsc;
    set_geometry(a);

    a->beamcon0 = a->ntsc ? 0 : BEAMCON0_DISPLAYPAL;
    a->lof = 1;
    a->hpos = 0;
    a->vpos = 0;

    /* Which chipset I present is a property of the game about to run, not of
       the chipset state, so it has to outlive the memset above: the sandbox
       picks the mode and THEN enters, and entering resets everything. pf_reset
       keeps its own copy across its memset too, but only reads what is here -
       this line is the only reason the ECS toggle works at all. */
    a->pf.ecs_mode = ecs;

    cop_reset(a);
    blt_reset(a);
    pf_reset(a);
    spr_reset(a);
    aud_reset(a);
    for (int i = 0; i < 256; i++) a->color[i] = 0;
    aga_update_ipl(a);
    pf_start_line(a);
    cop_start_line(a);
}

/* --------------------------------------------------------------------------
 * helpers
 * ------------------------------------------------------------------------ */
int aga_dma_enabled(const aga_t *a, uint16_t mask)
{
    return (a->dmacon & DMAF_DMAEN) && (a->dmacon & mask);
}

/* FRF_AGA_STRICT_TIMING_V0_1
 * Timing personality survives aga_reset() through cfg, just like PAL/NTSC.
 * Palette guard is deliberately one-shot per WHDLoad takeover. */
void aga_set_timing_strict(aga_t *a, int on) { a->cfg.strict_timing = on ? 1 : 0; }
int  aga_get_timing_strict(const aga_t *a)   { return a->cfg.strict_timing ? 1 : 0; }

void aga_prepare_game_takeover(aga_t *a)
{
    a->palette_guard = 1;
}

static void consume_palette_guard(aga_t *a)
{
    if (!a->palette_guard) return;
    a->palette_guard = 0;
    if (a->pf.ecs_mode) return;

    /* FRF_AGA_PALETTE_BANK_HANDOVER_V0_2
     *
     * WHDLoad takes the machine without resetting Lisa.  A preceding AGA
     * Workbench/display can therefore leave BOTH palette-addressing pieces of
     * BPLCON3 live:
     *
     *   BANK  bits 15..13  - selects one of eight 32-colour banks
     *   LOCT  bit  9       - selects low-nibble writes
     *
     * The old guard reset LOCT only.  If BANK was, for example, 7, a game
     * which assumes reset semantics and writes COLOR00..COLOR15 would program
     * colours 224..239 while its 4-plane pixels still index colours 0..15.
     * Geometry remains right but the whole picture is palette-remapped.
     *
     * Reset ONLY the palette-addressing fields here.  Preserve PF2OF, border,
     * sprite-resolution and every unrelated BPLCON3 bit.  An explicit CPU
     * BPLCON3 write by the game already cancels palette_guard before this
     * function is reached, so deliberate modern AGA bank/LOCT use wins.
     */
    {
        uint16_t oldv = a->pf.bplcon3;
        uint16_t newv = (uint16_t)(oldv & ~(0xE000u | BPLCON3_LOCT));

        if (newv != oldv) {
            a->regs[BPLCON3 >> 1] = newv;
            pf_write(a, BPLCON3, newv, a->hpos);

            if ((oldv & 0xE000u) && (oldv & BPLCON3_LOCT))
                aga_logf(a, "AGA palette handover: cleared stale BPLCON3 BANK + LOCT");
            else if (oldv & 0xE000u)
                aga_logf(a, "AGA palette handover: cleared stale BPLCON3 BANK");
            else
                aga_logf(a, "AGA palette handover: cleared stale BPLCON3.LOCT");
        } else {
            aga_logf(a, "AGA palette handover: BPLCON3 already BANK0/high-nibble");
        }
    }
}
void aga_record_change(aga_t *a, uint32_t reg, uint32_t value)
{
    if (a->nchanges < AGA_LINE_CHANGES) {
        linechange_t *c = &a->changes[a->nchanges++];
        c->hpos = (uint16_t)a->hpos;
        c->reg = (uint16_t)reg;
        c->value = value;
    }
}

static const uint8_t intlevel[15] = {
    1, 1, 1, 2,        /* TBE DSKBLK SOFT PORTS */
    3, 3, 3,           /* COPER VERTB BLIT */
    4, 4, 4, 4,        /* AUD0..3 */
    5, 5,              /* RBF DSKSYN */
    6,                 /* EXTER */
    0
};

void aga_update_ipl(aga_t *a)
{
    int ipl = 0;
    if (a->intena & INTF_INTEN) {
        uint16_t pending = a->intreq & a->intena & 0x3FFF;
        for (int i = 13; i >= 0; i--) {
            if (pending & (1 << i)) { ipl = intlevel[i]; break; }
        }
    }
    if (ipl != a->ipl) {
        a->ipl = ipl;
        if (a->hooks.ipl_changed) a->hooks.ipl_changed(a->hooks.user, ipl);
    }
}

void aga_intreq_set(aga_t *a, uint16_t bits)
{
    a->intreq |= bits & 0x7FFF;
    aga_update_ipl(a);
}

static void setclr(uint16_t *reg, uint16_t v)
{
    if (v & 0x8000) *reg |= v & 0x7FFF;
    else            *reg &= ~(v & 0x7FFF);
}

/* --------------------------------------------------------------------------
 * register reads
 * ------------------------------------------------------------------------ */
static uint16_t ext_read(aga_t *a, uint32_t reg, uint16_t dflt)
{
    if (a->hooks.ext_read) return a->hooks.ext_read(a->hooks.user, reg);
    return dflt;
}

/* FRF_AGA_COLOR_CORE_V0_1_4
 * FRF_AGA_COLOR_DISPLAY_CORE_V0_1_6 extends this with the COLORxx genlock
 * sideband bit and exact raster-window replay in playfield.c.
 * AGA palette readback.  BPLCON2.RDRAM turns COLOR00..COLOR31 into reads of
 * the 256-entry AGA colour table selected by BPLCON3 BANK, with BPLCON3.LOCT
 * choosing the low or high RGB nibble.  The write side already honoured
 * RDRAM; the read side was missing and COLORxx therefore fell through to 0.
 *
 * Keep ordinary COLORxx reads on the existing write-only/open-bus path: this
 * helper is called only while RDRAM is asserted and while presenting AGA. */
static uint16_t aga_color_rdram_read(aga_t *a, unsigned num)
{
    const playfield_t *p = &a->pf;
    const uint16_t con3 = pf_con3(p);
    const unsigned bank = (con3 >> 13) & 7u;
    const unsigned idx = bank * 32u + (num & 31u);
    const uint32_t rgb = a->color[idx];
    const unsigned r = (rgb >> 16) & 0xFFu;
    const unsigned g = (rgb >> 8) & 0xFFu;
    const unsigned b = rgb & 0xFFu;
    uint16_t v;

    if (con3 & BPLCON3_LOCT) {
        v = (uint16_t)(((r & 0x0Fu) << 8) | ((g & 0x0Fu) << 4) | (b & 0x0Fu));
    } else {
        v = (uint16_t)(((r >> 4) << 8) | ((g >> 4) << 4) | (b >> 4));
        if (a->color_genlock[idx]) v |= 0x8000u;
    }

    a->color_reads++;
    if (a->color_reads <= 8)
        aga_logf(a, "[AGA] RDRAM COLOR%02u bank=%u loct=%u -> %04x\n",
                 num & 31u, bank, (con3 & BPLCON3_LOCT) ? 1u : 0u, v);
    return v;
}

uint16_t aga_custom_rget(aga_t *a, uint32_t reg)
{
    reg &= 0x1FE;
    if (reg >= COLOR00 && reg < COLOR00 + 64 &&
        !a->pf.ecs_mode && (a->pf.bplcon2 & BPLCON2_RDRAM))
        return aga_color_rdram_read(a, (unsigned)((reg - COLOR00) >> 1));
    switch (reg) {
    case BLTDDAT:  return a->blt.ddat;
    case DMACONR:  return (a->dmacon & 0x07FF) | blt_dmaconr_bits(a);
    case VPOSR: {
        uint16_t v = (a->ntsc ? VPOSR_ID_AGA_NTSC : VPOSR_ID_AGA_PAL);
        v |= (a->lof ? 0x8000 : 0);
        v |= (a->vpos >> 8) & 7;
        return v;
    }
    case VHPOSR:   return (uint16_t)(((a->vpos & 0xFF) << 8) | (a->hpos & 0xFF));
    case DSKDATR: { uint16_t v; if (dsk_read(a, reg, &v)) return v; return ext_read(a, reg, 0); }
    case JOY0DAT:  return ext_read(a, reg, 0);
    case JOY1DAT:  return ext_read(a, reg, 0);
    case CLXDAT: {
        /* read and clear; bit 15 is unused and reads as 1 (WinUAE). A program
           that reads it wants collisions: work them out for the next frames */
        uint16_t v = a->clxdat | 0x8000;
        a->clxdat = 0;
        a->clx_frames = CLX_FRAMES;
        return v;
    }
    case ADKCONR:  return a->adkcon;
    case POT0DAT:  return ext_read(a, reg, 0);
    case POT1DAT:  return ext_read(a, reg, 0);
    case POTGOR:   return ext_read(a, reg, 0xFF00);
    case SERDATR:  return ext_read(a, reg, 0x3000);   /* TBE | TSRE */
    case DSKBYTR: { uint16_t v; if (dsk_read(a, reg, &v)) return v; return ext_read(a, reg, 0); }
    case INTENAR:  return a->intena & 0x7FFF;
    case INTREQR:  return a->intreq & 0x7FFF;
    case DENISEID: return DENISEID_LISA;
    case HHPOSR:   return 0;
    case COPINS:   return 0;
    default:
        /* write-only registers read back as the last value on the data bus; 0 here */
        return 0;
    }
}

uint32_t aga_read(aga_t *a, uint32_t addr, int size)
{
    uint32_t reg = addr & 0x1FF;
    switch (size) {
    case 1: {
        uint16_t w = aga_custom_rget(a, reg & 0x1FE);
        return (reg & 1) ? (w & 0xFF) : (w >> 8);
    }
    case 2:
        return aga_custom_rget(a, reg);
    default:
        return ((uint32_t)aga_custom_rget(a, reg) << 16) | aga_custom_rget(a, reg + 2);
    }
}

/* --------------------------------------------------------------------------
 * register writes
 * ------------------------------------------------------------------------ */
static void DMACON_write(aga_t *a, uint16_t v)
{
    uint16_t old = a->dmacon;
    uint16_t nv = old;
    setclr(&nv, v & 0x87FF);
    nv &= 0x07FF;
    a->dmacon = nv;
    /* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_DMACON: current BPLCON0 becomes the starting Agnus state when
       master+bitplane DMA transitions from disabled to enabled. */
    if (!((old & DMAF_DMAEN) && (old & DMAF_BPLEN)) &&
         ((nv  & DMAF_DMAEN) && (nv  & DMAF_BPLEN)))
        pf_dma_enable_sync(a);
    if ((old ^ nv) & (DMAF_AUDEN | DMAF_DMAEN))
        aud_dmacon_changed(a, old, nv);

    /* FRF_AGA_STRICT_TIMING_V0_1_3: direct hardware takeover guard.
     * A manually-entered sandbox does not have WHDLoad game-mode ownership,
     * so the glue-level INTENA/EXTER transition never arms the palette guard.
     * Real hardware games conventionally begin by clearing master DMA.  If
     * the OS left BPLCON3.LOCT set, treat a CPU master-DMA clear as the
     * ownership boundary.  This does not touch CIA/Paula state; it only arms
     * the existing one-shot colour guard.  An explicit game BPLCON3 write
     * below remains authoritative and cancels the guard before any colour is
     * changed.  Ignore Copper-authored DMACON writes. */
    if (!(v & 0x8000) && (v & DMAF_DMAEN) && !a->cop.in_move &&
        !a->pf.ecs_mode && (a->pf.bplcon3 & BPLCON3_LOCT)) {
        a->palette_guard = 1;
        aga_logf(a, "AGA takeover: CPU cleared master DMA with LOCT set; palette guard armed");
    }
    /* audio DMA bits may be mirrored to a real Paula by the glue */
    if (a->hooks.ext_write && (v & (DMAF_AUDEN | DMAF_DMAEN)))
        a->hooks.ext_write(a->hooks.user, DMACON, v);
    if (!(old & DMAF_COPEN) && (nv & DMAF_COPEN) && (nv & DMAF_DMAEN)) {
        /* FRF_AGA_STRICT_TIMING_V0_1_2: if WHDLoad just handed us the machine
           and the game starts a CPU-programmed copper without first writing
           BPLCON3, consume the stale-LOCT guard before that list can paint. */
        if (a->palette_guard && !a->cop.in_move) consume_palette_guard(a);
        /* copper DMA switched on: it continues from where it is */
    }
    /* an immediate blitter does not give a shit about BLTPRI */
}

static void INTENA_write(aga_t *a, uint16_t v)
{
    setclr(&a->intena, v);
    /* the glue mirrors the real-hardware sources (PORTS/EXTER) to the real Paula */
    if (a->hooks.ext_write) a->hooks.ext_write(a->hooks.user, INTENA, v);
    aga_update_ipl(a);
}

static void INTREQ_write(aga_t *a, uint16_t v)
{
    setclr(&a->intreq, v & 0xBFFF);
    if (a->hooks.ext_write && !(v & 0x8000) && (v & (INTF_PORTS | INTF_EXTER))) {
        /* forward clears of the mirrored real-hardware interrupt bits */
        a->hooks.ext_write(a->hooks.user, INTREQ, v & (INTF_PORTS | INTF_EXTER));
    }
    aga_update_ipl(a);
}

void aga_custom_wput(aga_t *a, uint32_t reg, uint16_t v)
{
    reg &= 0x1FE;
    a->regs[reg >> 1] = v;

    if (reg >= COLOR00 && reg < COLOR00 + 64) {
        /* A CPU palette write without an explicit game BPLCON3 write means the
           title expects reset/high-nibble palette semantics. Copper writes do
           not consume the guard: an old OS list can still be draining here. */
        if (a->palette_guard && !a->cop.in_move) consume_palette_guard(a);
        pf_color_write(a, (reg - COLOR00) >> 1, v, a->hpos);
        return;
    }
    if (reg >= BLTCON0 && reg <= BLTADAT) {
        blt_write(a, reg, v);
        return;
    }
    if (reg >= AUD0LCH && reg < AUD0LCH + 4 * 16) {
        aud_write(a, reg, v);
        return;
    }
    if (reg >= SPR0PTH && reg < SPR0POS + 8 * 8) {
        spr_write(a, reg, v);
        return;
    }
    if ((reg >= BPL1PTH && reg < BPL1PTH + 8 * 4) || (reg >= BPLCON0 && reg <= CLXCON2 && reg != CLXCON2)
        || (reg >= BPL1DAT && reg < BPL1DAT + 16) || reg == DIWSTRT || reg == DIWSTOP
        || reg == DDFSTRT || reg == DDFSTOP || reg == DIWHIGH || reg == FMODE) {
        /* If the game explicitly programs BPLCON3, that is authoritative:
           never rewrite LOCT behind its back. */
        if (reg == BPLCON3 && a->palette_guard && !a->cop.in_move)
            a->palette_guard = 0;
        pf_write(a, reg, v, a->hpos);
        return;
    }

    switch (reg) {
    case DSKPTH: case DSKPTL: case DSKLEN: case DSKDAT: case DSKSYNC:
        if (dsk_write(a, reg, v)) break;
        /* fall through: real drives */
    case SERDAT: case SERPER: case POTGO: case JOYTEST:
        if (a->hooks.ext_write) a->hooks.ext_write(a->hooks.user, reg, v);
        break;
    case REFPTR: break;
    case VPOSW:
        a->lof = (v & 0x8000) ? 1 : 0;
        a->vpos = (a->vpos & 0xFF) | ((v & 7) << 8);
        break;
    case VHPOSW:
        a->vpos = (a->vpos & 0x700) | (v >> 8);
        break;
    case COPCON:
        a->copcon = v;
        a->cop.danger = (v & 2) ? 1 : 0;
        break;
    case STREQU: case STRVBL: case STRHOR: case STRLONG: break;
    case COP1LCH: cop_write_lc(a, 1, 1, v); break;
    case COP1LCL: cop_write_lc(a, 1, 0, v); break;
    case COP2LCH: cop_write_lc(a, 2, 1, v); break;
    case COP2LCL: cop_write_lc(a, 2, 0, v); break;
    case COPJMP1:
        if (a->palette_guard && !a->cop.in_move) consume_palette_guard(a);
        cop_strobe(a, 1);
        break;
    case COPJMP2:
        if (a->palette_guard && !a->cop.in_move) consume_palette_guard(a);
        cop_strobe(a, 2);
        break;
    case COPINS: break;
    case DMACON: DMACON_write(a, v); break;
    case CLXCON:
        a->clxcon = v;
        a->clxcon2 &= ~(0x40 | 0x80);  /* writing CLXCON clears CLXCON2 sprite bits */
        break;
    case CLXCON2: a->clxcon2 = v; break;
    case INTENA: INTENA_write(a, v); break;
    case INTREQ: INTREQ_write(a, v); break;
    case ADKCON:
        setclr(&a->adkcon, v);
        if (a->hooks.ext_write) a->hooks.ext_write(a->hooks.user, reg, v);
        break;
    case BEAMCON0:
        /* FRF_AGA_UP104_SAFE_BEAMCON0:
           latch requested PAL/NTSC geometry at the frame boundary. */
        a->beamcon0 = v;
        break;
    case HTOTAL: case HSSTOP: case HBSTRT: case HBSTOP: case VTOTAL: case VSSTOP:
    case VBSTRT: case VBSTOP: case HSSTRT: case VSSTRT: case HCENTER:
    case SPRHSTRT: case SPRHSTOP: case BPLHSTRT: case BPLHSTOP: case HHPOSW:
    case BPLHMOD: case SPRHPTH: case SPRHPTL: case BPLHPTH: case BPLHPTL:
    case NOOP:
        break;
    case 0x1F2:              /* AGA-PISTORM control register (not an Amiga register) */
        a->aga_ctrl = v;
        break;
    case 0x1F4: a->aga_win[0] = v; break;   /* window x (RTG screen pixels) */
    case 0x1F6: a->aga_win[1] = v; break;   /* window y */
    case 0x1F8: a->aga_win[2] = v; break;   /* window width */
    case 0x1FA: a->aga_win[3] = v; break;   /* window height */
    default:
        break;
    }
}

/* FRF_AGA_SYSTEMWIDE_HAM6_OS_HANDOVER_V0_1_1
 * Seed the current AmigaOS Copper list into the VIRTUAL chipset only.
 * Normal core register machinery is retained, but ext_write is suppressed
 * for this one seed so COP/DMACON cannot leak onto the physical custom chips.
 * The existing HAM6 Copper therefore remains physical RGB owner. */
void aga_seed_os_copper(aga_t *a, uint32_t cop1lc)
{
    if (!a || cop1lc < 0x400u) return;

    __typeof__(a->hooks.ext_write) saved_ext_write = a->hooks.ext_write;
    a->hooks.ext_write = 0;

    /* graphics.library copinit -> virtual COP1LC; strobe; enable
       master + bitplane + copper + sprite DMA in virtual AGA. */
    aga_custom_wput(a, 0x080u, (uint16_t)(cop1lc >> 16));
    aga_custom_wput(a, 0x082u, (uint16_t)cop1lc);
    aga_custom_wput(a, 0x088u, 0);
    aga_custom_wput(a, 0x096u, 0x83A0u);

    a->hooks.ext_write = saved_ext_write;
}


void aga_write(aga_t *a, uint32_t addr, uint32_t value, int size)
{
    uint32_t reg = addr & 0x1FF;
    switch (size) {
    case 1:
        /* byte writes to custom registers write the byte to both halves (bus behaviour) */
        aga_custom_wput(a, reg & 0x1FE, (uint16_t)((value & 0xFF) | ((value & 0xFF) << 8)));
        break;
    case 2:
        aga_custom_wput(a, reg, (uint16_t)value);
        break;
    default:
        aga_custom_wput(a, reg, (uint16_t)(value >> 16));
        aga_custom_wput(a, reg + 2, (uint16_t)value);
        break;
    }
}

/* --------------------------------------------------------------------------
 * beam
 * ------------------------------------------------------------------------ */
static void end_of_line(aga_t *a)
{
    cop_sync(a, a->maxhpos);
    pf_sync(a, a->maxhpos);
    pf_finish_line(a);
    aud_line_tick(a);
    dsk_line_tick(a);
    a->nchanges = 0;
}

const aga_slot_t *aga_dma_trace(const aga_t *a)
{
#ifdef AGA_TRACE
    return a->trace[a->fb_cur ^ 1];      /* the frame just completed */
#else
    (void)a;
    return 0;
#endif
}

/* At most this many frames skipped in a row, then one gets drawn: a machine
   that's seconds behind still shows 12 fps, not a dead fucking still. */
#define SKIP_RUN_MAX 3

static void end_of_frame(aga_t *a)
{
    /* FRF_AGA_UP104_SAFE_BEAMCON0:
       apply requested geometry only at the frame boundary. */
    {
        int want_ntsc = (a->beamcon0 & BEAMCON0_DISPLAYPAL) ? 0 : 1;
        if (want_ntsc != a->ntsc) {
            a->ntsc = want_ntsc;
            set_geometry(a);
        }
    }
    a->frame++;
    a->frame_done_count++;
    a->last_display_lines = a->display_lines;
    a->display_lines = 0;
    a->last_frame_data = a->pf_data_nonzero;
    a->pf_data_nonzero = 0;
    if (a->clx_frames > 0) a->clx_frames--;
    if (a->frame_skip) {
        /* nothing new was drawn: keep the finished page where it is */
        a->frames_skipped++;
    } else {
        a->fb_cur ^= 1;
#ifdef AGA_TRACE
        if (a->trace[a->fb_cur])
            memset(a->trace[a->fb_cur], 0, (size_t)AGA_MAX_VPOS * AGA_MAX_HPOS * sizeof(aga_slot_t));
#endif
    }
    if (a->skip_req && a->skip_run < SKIP_RUN_MAX && !(a->cfg.collisions || a->clx_frames > 0)) {
        a->frame_skip = 1;
        a->skip_run++;
    } else {
        a->frame_skip = 0;
        a->skip_run = 0;
    }
    if (a->hooks.frame_done) a->hooks.frame_done(a->hooks.user);
}

static void start_of_line(aga_t *a)
{
    if (a->vpos == 0) {
        /* vertical blank start */
        aga_intreq_set(a, INTF_VERTB);
        cop_vsync(a);
        spr_vsync(a);
        pf_vsync(a);
    }
    pf_start_line(a);
    cop_start_line(a);
    a->sprline_valid = 0;
    a->sprclx_valid = 0;
}

static inline void cck_step(aga_t *a)
{
    int h = a->hpos;
    /* FRF_AGA_UP104_COPPER_IDLE_FAST_V0_1:
       Upstream 1.104 observation: most even colour clocks cannot advance the
       Copper. Skip cop_sync() only when progress is provably impossible.

       IMPORTANT: this does NOT change the Copper state machine itself. It
       advances only last_hpos across time that cop_sync() would have consumed
       doing no useful work. */
    if (!(h & 1)) {
        copper_t *c = &a->cop;
        int idle =
            (a->dmacon & (DMAF_DMAEN | DMAF_COPEN))
                != (DMAF_DMAEN | DMAF_COPEN)
            || c->state == COP_stop
            || (c->state == COP_wait
                && (a->vpos & c->vmask) < c->vcmp)
            || (c->state == COP_bltwait && a->blt.busy);

        if (idle) {
            if (c->last_hpos < h + 1)
                c->last_hpos = h + 1;
        } else {
            cop_sync(a, h + 1);
        }
    }
    /* sprite DMA slots 0x15..0x34 */
    if (h >= 3 && h <= 9 && (h & 1)) aga_trace_at(a, h, AGA_DMA_REFRESH, REFPTR, 0);
    if (h >= 0x19 && h < 0x19 + 4 * AGA_MAX_SPRITES) spr_dma_slot(a, h);
    /* audio DMA slots 0x0D,0x0F,0x11,0x13 */
    if (h >= 0x0D && h <= 0x13 && (h & 1)) aud_dma_slot(a, h);
    /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: advance the DAC/state machine after this cck's DMA slot. */
    aud_cck_tick(a);
}

void aga_run_cycles(aga_t *a, int cck)
{
    while (cck-- > 0) {
        cck_step(a);
        a->hpos++;
        if (a->hpos >= a->maxhpos) {
            end_of_line(a);
            a->hpos = 0;
            a->vpos++;
            int lines = a->maxvpos + (a->lof ? 1 : 0);
            if (a->vpos >= lines) {
                a->vpos = 0;
                if (a->pf.bplcon0 & BPLCON0_LACE) a->lof ^= 1;
                else a->lof = 1;
                end_of_frame(a);
            }
            start_of_line(a);
        }
    }
}

void aga_run_line(aga_t *a)
{
    aga_run_cycles(a, a->maxhpos - a->hpos);
}

void aga_run_frame(aga_t *a)
{
    int target = a->frame + 1;
    while (a->frame < target) aga_run_line(a);
}

/* --------------------------------------------------------------------------
 * status
 * ------------------------------------------------------------------------ */
int aga_last_frame_display_lines(const aga_t *a) { return a->last_display_lines; }
int aga_last_frame_had_data(const aga_t *a)      { return a->last_frame_data; }
uint32_t aga_blit_count(const aga_t *a)          { return a->blit_count; }
int aga_native_active(const aga_t *a)
{
    if (a->aga_ctrl & 0x0002) return 1;               /* windowed: always present the frame */
    if (a->aga_ctrl & 0x8000) return !(a->aga_ctrl & 1);
    if (a->last_display_lines > 0) return 1;
    /* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_NATIVE: hardware sprites are native video even when the planar
       playfield reports zero completed display lines. HAM6 exposed this because
       it presents independently of the RTG/HVS native-active gate. */
    for (int n = 0; n < AGA_MAX_SPRITES; n++)
        if (a->spr[n].armed) return 1;
    return 0;
}
int aga_window_mode(const aga_t *a, int *x, int *y, int *w, int *h)
{
    if (x) *x = a->aga_win[0];
    if (y) *y = a->aga_win[1];
    if (w) *w = a->aga_win[2];
    if (h) *h = a->aga_win[3];
    return (a->aga_ctrl & 0x0002) ? 1 : 0;
}
int aga_get_ipl(const aga_t *a)          { return a->ipl; }
int aga_get_vpos(const aga_t *a)         { return a->vpos; }
int aga_get_hpos(const aga_t *a)         { return a->hpos; }
int aga_frame_count(const aga_t *a)      { return a->frame; }
int aga_lines_per_frame(const aga_t *a)  { return a->maxvpos + (a->lof ? 1 : 0); }
int aga_cycles_per_line(const aga_t *a)  { return a->maxhpos; }
const uint32_t *aga_framebuffer(const aga_t *a) { return a->fb[a->fb_cur ^ 1]; }
uint32_t *aga_framebuffer_live(aga_t *a) { return a->fb[a->fb_cur]; }
/* Lock-free beam read.
 *
 * VPOSR/VHPOSR are by far the hottest registers a real Amiga program reads -
 * every short delay is a beam wait. Measured on hardware with an AGA WHDLoad
 * game loading: 2,173,700 reads a second, 43,000 a frame, from a dozen
 * two-instruction loops in ROM and fast RAM at once. Taking the chipset mutex
 * for every fucking one of those kept the machine in the trap path full time
 * and fired two million cross-core SEVs a second at the core actually running
 * the chipset.
 *
 * Nothing here needs the lock. vpos, hpos and lof are plain ints the chipset
 * thread advances; the worst a racing reader gets is a value a few colour
 * clocks old, which is indistinguishable from reading the register a moment
 * earlier - and on real hardware the beam moves under you anyway. */
int aga_is_beam_reg(uint32_t reg) { reg &= 0x1FE; return reg == VPOSR || reg == VHPOSR; }

/* add_cck: colour clocks elapsed since the chipset thread last stepped.
 *
 * The loop advances the beam CHUNK_CCK at a time, so without this hpos only
 * ever takes values 8 apart. A beam wait tests for an EXACT horizontal
 * position - `move.b $dff007,d0 / cmp.b d1,d0 / bne` - and 7 targets in 8
 * never fucking appear. The phase drifts by maxhpos mod CHUNK_CCK each line,
 * so the loop does eventually hit its value, but only every few lines: every
 * short beam wait in the machine silently becomes several lines long.
 *
 * Stepping the chipset one clock at a time would fix it and cost core 3 eight
 * times the work. Interpolating at the point of the read costs a divide, and
 * the pacing is against the same clock, so the answer comes out right. */
uint32_t aga_peek_beam_at(const aga_t *a, uint32_t reg, int size, uint32_t add_cck)
{
    uint32_t vpos = (uint32_t)a->vpos, hpos = (uint32_t)a->hpos;
    if (add_cck) {
        uint32_t maxh = (uint32_t)a->maxhpos ? (uint32_t)a->maxhpos : 227;
        uint32_t maxv = (uint32_t)a->maxvpos + (a->lof ? 1 : 0);
        if (!maxv) maxv = 313;
        hpos += add_cck;
        vpos += hpos / maxh;
        hpos %= maxh;
        if (vpos >= maxv) vpos %= maxv;
    }
    uint16_t vposr  = (uint16_t)((a->ntsc ? VPOSR_ID_AGA_NTSC : VPOSR_ID_AGA_PAL)
                                 | (a->lof ? 0x8000 : 0) | ((vpos >> 8) & 7));
    uint16_t vhposr = (uint16_t)(((vpos & 0xFF) << 8) | (hpos & 0xFF));
    uint16_t w = ((reg & 0x1FE) == VPOSR) ? vposr : vhposr;
    switch (size) {
    case 1:  return (reg & 1) ? (w & 0xFF) : (w >> 8);
    case 4:  return ((uint32_t)vposr << 16) | vhposr;
    default: return w;
    }
}

uint32_t aga_peek_beam(const aga_t *a, uint32_t reg, int size)
{
    uint32_t vpos = (uint32_t)a->vpos, hpos = (uint32_t)a->hpos;
    uint16_t vposr  = (uint16_t)((a->ntsc ? VPOSR_ID_AGA_NTSC : VPOSR_ID_AGA_PAL)
                                 | (a->lof ? 0x8000 : 0) | ((vpos >> 8) & 7));
    uint16_t vhposr = (uint16_t)(((vpos & 0xFF) << 8) | (hpos & 0xFF));
    uint16_t w = ((reg & 0x1FE) == VPOSR) ? vposr : vhposr;
    switch (size) {
    case 1:  return (reg & 1) ? (w & 0xFF) : (w >> 8);
    case 4:  return ((uint32_t)vposr << 16) | vhposr;   /* a longword at $DFF004 */
    default: return w;
    }
}

uint16_t aga_peek_reg(const aga_t *a, uint32_t reg) { return a->regs[(reg & 0x1FE) >> 1]; }
uint32_t aga_peek_color(const aga_t *a, int index) { return a->color[index & 255]; }
uint32_t aga_peek_bplpt(const aga_t *a, int plane) { return a->pf.bplpt[plane & 7]; }
uint32_t aga_peek_copper_pc(const aga_t *a) { return a->cop.pc; }
uint32_t aga_peek_cop1lc(const aga_t *a) { return a->cop.cop1lc; }

void aga_set_render(aga_t *a, int on)
{
    a->cfg.render = on ? 1 : 0;
}

/* Render into these two pages instead of the core's own: the ARM side hands
   over the HVS pages, so a finished frame is shown and not copied. */
void aga_set_framebuffers(aga_t *a, uint32_t *f0, uint32_t *f1)
{
    a->fb[0] = f0; a->fb[1] = f1; a->fb_external = 1;
}

/* Ask for whole frames to be skipped while the host is behind real time.
   Takes effect at the next frame end. */
void aga_set_frame_skip(aga_t *a, int on) { a->skip_req = on ? 1 : 0; }
uint32_t aga_frames_skipped(const aga_t *a) { return a->frames_skipped; }
