/* AGA-PISTORM — sprite DMA and per-line sprite rendering.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include "internal.h"

/* Sprite DMA slots: WinUAE records put SPRxPOS at 0x19+4x and SPRxCTL at
   0x1B+4x (sprite 0 at cycles 25 and 27), so sprite 7 ends at cycle 55. */
#define SPR0_HPOS 0x19

void spr_reset(aga_t *a)
{
    memset(a->spr, 0, sizeof a->spr);
    a->sprite_width = 16;
    a->sprres = RES_LORES;
}

void spr_expand_res(aga_t *a)
{
    switch ((pf_con3(&a->pf) >> 6) & 3) {
    case 0:  a->sprres = (a->pf.res == RES_SHRES) ? RES_HIRES : RES_LORES; break;
    case 1:  a->sprres = RES_LORES; break;
    case 2:  a->sprres = RES_HIRES; break;
    default: a->sprres = RES_SHRES; break;
    }
    switch ((a->pf.fmode >> 2) & 3) {
    case 0:  a->sprite_width = 16; break;
    case 3:  a->sprite_width = 64; break;
    default: a->sprite_width = 32; break;
    }
}

static void spr_update_posctl(aga_t *a, int n)
{
    sprite_t *s = &a->spr[n];
    /* xpos in shres pixels (4 per lores pixel), DIW coordinates */
    s->xpos = (((s->pos & 0xFF) << 1) | (s->ctl & 1)) << 2;
    s->xpos |= (s->ctl >> 3) & 3;
    s->vstart = (s->pos >> 8) | ((s->ctl & 0x04) ? 0x100 : 0) | ((s->ctl & 0x40) ? 0x200 : 0);
    s->vstop  = (s->ctl >> 8) | ((s->ctl & 0x02) ? 0x100 : 0) | ((s->ctl & 0x20) ? 0x200 : 0);
    s->attach = (s->ctl & 0x80) ? 1 : 0;
}

static inline uint64_t widen16(uint16_t v)
{
    /* a CPU write of 16 bits fills all fetch-width words (UAE behaviour) */
    uint64_t x = v;
    return x | (x << 16) | (x << 32) | (x << 48);
}

void spr_write(aga_t *a, uint32_t reg, uint16_t v)
{
    if (reg >= SPR0PTH && reg < SPR0PTH + 8 * 4) {
        int n = (reg - SPR0PTH) >> 2;
        sprite_t *s = &a->spr[n];
        if ((reg & 2) == 0) s->pt = (s->pt & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16);
        else                s->pt = (s->pt & 0xFFFF0000u) | (v & 0xFFFE);
        return;
    }
    if (reg >= SPR0POS && reg < SPR0POS + 8 * 8) {
        int n = (reg - SPR0POS) >> 3;
        sprite_t *s = &a->spr[n];
        switch (reg & 6) {
        case 0: s->pos = v; spr_update_posctl(a, n); break;
        case 2: s->ctl = v; s->armed = 0; spr_update_posctl(a, n); break;
        case 4: s->data = widen16(v); s->armed = 1; break;
        case 6: s->datb = widen16(v); break;
        }
    }
}

void spr_vsync(aga_t *a)
{
    for (int i = 0; i < AGA_MAX_SPRITES; i++) a->spr[i].dmastate = 0;
}

static uint64_t spr_fetch(aga_t *a, uint32_t addr)
{
    switch (a->sprite_width) {
    case 16: return (uint64_t)chip_rd16(a, addr) << 48;
    case 32: return (uint64_t)chip_rd32(a, addr) << 32;
    default: return chip_rd64(a, addr);
    }
}

/* Called at colour clocks 0x15.. for each sprite slot; two accesses per sprite:
   (hpos - 0x15) & 3 == 0 -> first word (POS or DATA), == 2 -> second (CTL or DATB). */
void spr_dma_slot(aga_t *a, int hpos)
{
    int off = hpos - SPR0_HPOS;
    int n = off >> 2;
    int cycle = off & 3;
    if (n < 0 || n >= AGA_MAX_SPRITES || (cycle & 1)) return;
    cycle >>= 1;
    sprite_t *s = &a->spr[n];
    int vblank_end = a->sprite_vblank;   /* sprites restart on their own line, not the display's */

    if (a->vpos < vblank_end) return;
    /* no sprite DMA on the last line of the frame (WinUAE: Super Stardust
       fetches nothing on line 312 where this dumb shit fetched eight data
       words) */
    if (a->vpos >= a->maxvpos + (a->lof ? 1 : 0) - 1) return;

    int posctl = 0;
    if (a->vpos == s->vstart) s->dmastate = 1;
    if (a->vpos == s->vstop || a->vpos == vblank_end) { s->dmastate = 0; posctl = 1; }
    if (!aga_dma_enabled(a, DMAF_SPREN)) return;
    if (!s->dmastate && !posctl) return;     /* idle: waiting for vstart, no DMA */

    uint32_t bytes = a->sprite_width >> 3;
    if (posctl) {
        aga_trace_at(a, hpos, AGA_DMA_SPRITE,
                     (uint16_t)(SPR0POS + 8 * n + 2 * cycle), s->pt);
        uint16_t w = chip_rd16(a, s->pt);
        s->pt += bytes;
        if (cycle == 0) { s->pos = w; spr_update_posctl(a, n); }
        else            { s->ctl = w; s->armed = 0; spr_update_posctl(a, n); }
    } else {
        aga_trace_at(a, hpos, AGA_DMA_SPRITE,
                     (uint16_t)(SPR0DATA + 8 * n + 2 * cycle), s->pt);
        uint64_t d = spr_fetch(a, s->pt);
        s->pt += bytes;
        if (cycle == 0) { s->data = d; s->armed = 1; }
        else            { s->datb = d; }
    }
}

/* Render all armed sprites for the current line into a->sprline (shres pixels,
   DIW coordinate origin). Encoding: bits0-3 colour, bit4 attached, bit5 odd base,
   bits6-7 pair, 0 = transparent. With clx, a->sprclx also gets one bit per
   sprite: collisions need every sprite, also the ones another sprite hides. */
void spr_build_line(aga_t *a, int clx)
{
    uint8_t *line = a->sprline;
    int linelen = (int)sizeof a->sprline;
    memset(line, 0, linelen);
    a->sprline_valid = 1;

    int w = a->sprite_width;
    int pixw = 4 >> a->sprres;     /* shres pixels per sprite pixel */
    /* sprite positions map to hardware coordinates with a +9 lores pixel offset (UAE DIW_DDF_OFFSET) */
    const int hwoff = 9 * 4;

    a->sprclx_valid = clx;
    if (clx) {
        memset(a->sprclx, 0, sizeof a->sprclx);
        for (int n = 0; n < AGA_MAX_SPRITES; n++) {
            const sprite_t *s = &a->spr[n];
            if (!s->armed) continue;
            uint64_t solid = (s->data | s->datb) >> (64 - w);   /* 1 = a non-transparent pixel */
            if (!solid) continue;
            for (int k = 0; k < w; k++) {
                if (!((solid >> (w - 1 - k)) & 1)) continue;
                int x = s->xpos + hwoff + k * pixw;
                for (int p = 0; p < pixw; p++)
                    if (x + p >= 0 && x + p < linelen) a->sprclx[x + p] |= (uint8_t)(1u << n);
            }
        }
    }

    /* lower-numbered sprites have priority: draw 7..0 so lower overwrite */
    for (int pair = 3; pair >= 0; pair--) {
        sprite_t *se = &a->spr[pair * 2];
        sprite_t *so = &a->spr[pair * 2 + 1];
        int attached = so->attach;
        if (attached && se->armed && so->armed && se->xpos == so->xpos) {
            for (int k = 0; k < w; k++) {
                int sh = w - 1 - k;
                int c = (int)((se->data >> (64 - w + sh)) & 1) | ((int)((se->datb >> (64 - w + sh)) & 1) << 1)
                      | ((int)((so->data >> (64 - w + sh)) & 1) << 2) | ((int)((so->datb >> (64 - w + sh)) & 1) << 3);
                if (!c) continue;
                int x = se->xpos + hwoff + k * pixw;
                uint8_t v = (uint8_t)(c | 0x10 | 0x20 | (pair << 6));
                for (int p = 0; p < pixw; p++) if (x + p >= 0 && x + p < linelen) line[x + p] = v;
            }
            continue;
        }
        for (int which = 1; which >= 0; which--) {
            sprite_t *s = which ? so : se;
            if (!s->armed) continue;
            for (int k = 0; k < w; k++) {
                int sh = 64 - w + (w - 1 - k);
                int c = (int)((s->data >> sh) & 1) | ((int)((s->datb >> sh) & 1) << 1);
                if (!c) continue;
                int x = s->xpos + hwoff + k * pixw;
                uint8_t v = (uint8_t)(c | (which ? 0x20 : 0) | (pair << 6));
                for (int p = 0; p < pixw; p++) if (x + p >= 0 && x + p < linelen) line[x + p] = v;
            }
        }
    }
}
