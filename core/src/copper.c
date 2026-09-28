/* AGA-PISTORM — Copper coprocessor.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Model: one memory access per two colour clocks. MOVE = 2 accesses, WAIT and
 * SKIP = 2 accesses plus a 2-cycle compare delay. Beam comparisons use the
 * position of the *next* copper cycle, like the hardware. A cycle already
 * taken by bitplane DMA is not available to the copper.
 */
#include "internal.h"

void cop_reset(aga_t *a)
{
    memset(&a->cop, 0, sizeof a->cop);
    a->cop.state = COP_stop;
}

void cop_write_lc(aga_t *a, int which, int high, uint16_t v)
{
    uint32_t *lc = (which == 1) ? &a->cop.cop1lc : &a->cop.cop2lc;
    if (high) *lc = (*lc & 0x0000FFFF) | ((uint32_t)(v & 0x01FF) << 16);
    else      *lc = (*lc & 0xFFFF0000) | (v & 0xFFFE);
}

void cop_strobe(aga_t *a, int which)
{
    copper_t *c = &a->cop;
    c->ignore_next = 0;
    if (c->in_move) {
        /* Written by the copper itself: the hardware has already fetched the
           next instruction word, so exactly one more instruction from the old
           list runs before the jump takes effect (WinUAE records: Castlevania
           line 52 and Turrican 2 line 194 both show COPJMP2, then one more
           instruction read from the old list, then the new list). Running that
           bastard breaks every list that ends in a WAIT terminator after the
           jump, so it is fetched and thrown away. */
        c->strobe = which;
        c->ignore_next = 1;       /* that instruction is fetched but not executed */
        return;
    }
    /* Written by the CPU: takes effect at once, after one idle cycle. */
    c->pc = (which == 1) ? c->cop1lc : c->cop2lc;
    c->reload = which;  /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6: sample LC again on first real fetch */
    c->state = COP_strobe_delay;
    c->strobe = 0;
}

void cop_vsync(aga_t *a)
{
    /* Vertical blank restarts the copper at COP1LC. */
    a->cop.strobe = 0;
    a->cop.pc = a->cop.cop1lc;
    a->cop.reload = 1;  /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6: v1.105 - read CURRENT COP1LC at first fetch */
    a->cop.state = COP_read1;
    a->cop.ignore_next = 0;
    a->cop.delay = 2;
}

void cop_start_line(aga_t *a)
{
    /* The first cycle a line offers the copper is 2, not 0: WinUAE DMA records
       show copper accesses at 2,4,..,226 (113 per line), never at 0. Starting
       at 0 gave the copper one extra access per line, which showed up as a
       drift of one instruction word per line whenever a long copper run was
       not interrupted by a WAIT (see docs/VALIDATION-FINDINGS.md). */
    a->cop.last_hpos = 1;
}

void cop_blitter_done(aga_t *a)
{
    if (a->cop.state == COP_bltwait) a->cop.state = COP_read1;
}

static int cop_reg_allowed(const aga_t *a, uint32_t reg)
{
    if (a->cop.danger) return 1;              /* AGA + CDANG: everything */
    return reg >= 0x80;
}

static inline int beam_reached(const aga_t *a, int hpos)
{
    int vp = a->vpos & a->cop.vmask;
    int hp = hpos & a->cop.hmask;
    if (vp > a->cop.vcmp) return 1;
    if (vp == a->cop.vcmp && hp >= a->cop.hcmp) return 1;
    return 0;
}

static void decode_compare(aga_t *a)
{
    copper_t *c = &a->cop;
    c->vcmp = (c->i1 & (c->i2 | 0x8000)) >> 8;
    c->hcmp = c->i1 & c->i2 & 0xFE;
    c->vmask = ((c->i2 >> 8) & 0x7F) | 0x80;
    c->hmask = c->i2 & 0xFE;
    c->blitwait = (c->i2 & 0x8000) ? 0 : 1;
}

/* Advance the copper to colour clock `until` (exclusive).
 *
 * Timing model (calibrated against WinUAE DMA records): the copper uses even
 * cycles. A read planned for cycle c happens only if cycle c itself is free of
 * bitplane DMA, and a WAIT is satisfied when the beam position c-2 has been
 * reached; the first read then happens at the next free even cycle. */
void cop_sync(aga_t *a, int until)
{
    copper_t *c = &a->cop;
    int c_hpos = c->last_hpos;

    if (until > a->maxhpos) until = a->maxhpos;
    if (c_hpos >= until) return;

    if (!aga_dma_enabled(a, DMAF_COPEN) || c->state == COP_stop) {
        c->last_hpos = until;
        return;
    }
    if (c->state == COP_wait && (a->vpos & c->vmask) < c->vcmp) {
        c->last_hpos = until;
        return;
    }
    if (c->state == COP_bltwait) {
        if (a->blt.busy) { c->last_hpos = until; return; }
        c->state = COP_read1;
    }

    c_hpos = (c_hpos + 1) & ~1;
    while (c_hpos < until) {
        int prev = c_hpos - 2;
        if (c->delay > 0) {
            c->delay--;
            c_hpos += 2;
            continue;
        }
        switch (c->state) {
        case COP_strobe_delay:
            c->state = COP_read1;
            break;
        case COP_read1:
            if (pf_is_bpl_slot(a, c_hpos)) break;                 /* slot taken by bitplane DMA */

            /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6: a COPJMP/vblank strobe samples its LC pointer
               when the first Copper DMA fetch actually occurs, not when DMA was off. */
            if (c->reload) {
                c->pc = (c->reload == 1) ? c->cop1lc : c->cop2lc;
                c->reload = 0;
            }
            aga_trace_at(a, c_hpos, AGA_DMA_COPPER, 0x8C, c->pc);
            c->i1 = chip_rd16(a, c->pc);
            c->pc += 2;
            c->state = COP_read2;
            break;
        case COP_read2:
            if (pf_is_bpl_slot(a, c_hpos)) break;
            aga_trace_at(a, c_hpos, AGA_DMA_COPPER,
                         (uint16_t)((c->i1 & 1) ? 0x8C : (c->i1 & 0x1FE)), c->pc);
            c->i2 = chip_rd16(a, c->pc);
            c->pc += 2;
            if (c->ignore_next) {
                c->ignore_next = 0;
                if (c->strobe) {
                    /* the discarded instruction after a copper-written COPJMP */
                    c->pc = (c->strobe == 1) ? c->cop1lc : c->cop2lc;
                    c->strobe = 0;
                    c->reload = 0;  /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6 */
                }
                c->state = COP_read1;
                break;
            }
            if (c->i1 & 1) {
                c->state = (c->i2 & 1) ? COP_skip_in2 : COP_wait_in2;
            } else {
                uint32_t reg = c->i1 & 0x1FE;
                if (cop_reg_allowed(a, reg)) {
                    c->in_move = 1;
                    aga_custom_wput(a, reg, c->i2);
                    c->in_move = 0;
                    c->state = COP_read1;
                } else {
                    c->state = COP_stop;   /* illegal access: copper halts until next strobe/vblank */
                    c->last_hpos = until;
                    return;
                }
            }
            break;
        case COP_wait_in2:
            decode_compare(a);
            if (c->i1 == 0xFFFF && c->i2 == 0xFFFE) {
                c->state = COP_stop;
                c->last_hpos = until;
                return;
            }
            c->state = COP_wait;
            if ((a->vpos & c->vmask) < c->vcmp) {
                c->last_hpos = until;
                return;
            }
            /* FALLTHROUGH */
        case COP_wait:
            if (prev >= 0 && beam_reached(a, prev)) {
                if (c->blitwait && a->blt.busy) {
                    c->state = COP_bltwait;
                    c->last_hpos = until;
                    return;
                }
                c->state = COP_read1;
                /* A WAIT that is already satisfied when the line begins wakes the
                   copper one slot later than a mid-line match: WinUAE's first
                   read is then at cycle 6 (Castlevania lines 52/57, Raptor line 34,
                   agatest line 10), where a mid-line wake-up reads at the next
                   free even cycle. */
                if (c_hpos <= 2) c->delay = 1;
            }
            break;
        case COP_skip_in2:
            decode_compare(a);
            if (prev >= 0 && beam_reached(a, prev) && !(c->blitwait && a->blt.busy))
                c->ignore_next = 1;
            c->state = COP_read1;
            break;
        default:
            c->state = COP_stop;
            break;
        }
        c_hpos += 2;
    }
    c->last_hpos = until;
}
