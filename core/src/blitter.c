/* AGA-PISTORM — Blitter (immediate mode).
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Blit maths lifted from UAE's blitter_dofast / blitter_dofast_desc and the
 * line mode state machine (GPL, Bernd Schmidt / Toni Wilen).
 */
#include "internal.h"

static uint8_t fill_table[256][4][2];   /* [byte][ife*2 + carry_in] -> {out, carry_out} */
static int fill_table_ready;

static void build_fill_table(void)
{
    for (int d = 0; d < 256; d++) {
        for (int i = 0; i < 4; i++) {
            int fc = i & 1;
            int ife = i >> 1;
            int out = 0;
            for (int bit = 0; bit < 8; bit++) {
                int b = (d >> bit) & 1;
                int nfc = fc ^ b;
                if (ife) {
                    /* inclusive fill: bit set if fc or b or new fc */
                    if (fc || b || nfc) out |= 1 << bit;
                } else {
                    /* exclusive fill: set if (fc xor b) */
                    if (fc ^ b) out |= 1 << bit;
                }
                fc = nfc;
            }
            fill_table[d][i][0] = (uint8_t)out;
            fill_table[d][i][1] = (uint8_t)fc;
        }
    }
    fill_table_ready = 1;
}

void blt_reset(aga_t *a)
{
    memset(&a->blt, 0, sizeof a->blt);
    if (!fill_table_ready) build_fill_table();
}

uint16_t blt_dmaconr_bits(const aga_t *a)
{
    return (uint16_t)((a->blt.busy ? DMAF_BBUSY : 0) | (a->blt.zero ? DMAF_BZERO : 0));
}

static inline uint32_t blit_func(uint32_t sa, uint32_t sb, uint32_t sc, uint8_t mt)
{
    uint32_t d = 0;
    if (mt & 0x80) d |=  sa &  sb &  sc;
    if (mt & 0x40) d |=  sa &  sb & ~sc;
    if (mt & 0x20) d |=  sa & ~sb &  sc;
    if (mt & 0x10) d |=  sa & ~sb & ~sc;
    if (mt & 0x08) d |= ~sa &  sb &  sc;
    if (mt & 0x04) d |= ~sa &  sb & ~sc;
    if (mt & 0x02) d |= ~sa & ~sb &  sc;
    if (mt & 0x01) d |= ~sa & ~sb & ~sc;
    return d & 0xFFFF;
}

static inline uint16_t do_fill(const blitter_t *b, uint16_t d, int *fc)
{
    int ife = (b->con1 & BLTCON1_IFE) ? 2 : 0;
    int fc1 = fill_table[d & 255][ife + *fc][1];
    uint16_t out = fill_table[d & 255][ife + *fc][0] | (fill_table[d >> 8][ife + fc1][0] << 8);
    *fc = fill_table[d >> 8][ife + fc1][1];
    return out;
}

static void blit_normal(aga_t *a, int hsize, int vsize)
{
    blitter_t *b = &a->blt;
    uint8_t mt = (uint8_t)(b->con0 & 0xFF);
    int usea = b->con0 & BLTCON0_USEA, useb = b->con0 & BLTCON0_USEB;
    int usec = b->con0 & BLTCON0_USEC, used = b->con0 & BLTCON0_USED;
    int ash = (b->con0 >> 12) & 15, bsh = (b->con1 >> 12) & 15;
    int desc = b->con1 & BLTCON1_DESC;
    int fill = (b->con1 & (BLTCON1_IFE | BLTCON1_EFE)) && desc;  /* fill only works descending */
    int fill_any = b->con1 & (BLTCON1_IFE | BLTCON1_EFE);
    uint32_t apt = b->apt, bpt = b->bpt, cpt = b->cpt, dpt = b->dpt;
    uint32_t preva = 0, prevb = 0;
    uint32_t bhold = b->bhold;
    uint16_t cdat = b->cdat, ddat = b->ddat;
    int zero = 1;
    int dodst = 0;
    uint32_t dstp = 0;

    (void)fill_any;
    for (int y = 0; y < vsize; y++) {
        int fc = (b->con1 & BLTCON1_FCI) ? 1 : 0;
        for (int x = 0; x < hsize; x++) {
            uint32_t adat, ahold;
            uint16_t mask = 0xFFFF;
            if (x == 0) mask &= b->afwm;
            if (x == hsize - 1) mask &= b->alwm;

            if (usea) {
                b->adat = chip_rd16(a, apt);
                apt += desc ? -2 : 2;
            }
            adat = b->adat & mask;
            if (desc) {
                ahold = ((adat << 16) | preva) >> (16 - ash);
            } else {
                ahold = ((preva << 16) | adat) >> ash;
            }
            preva = adat;

            if (useb) {
                uint32_t bdat = b->bdat = chip_rd16(a, bpt);
                bpt += desc ? -2 : 2;
                if (desc) bhold = ((bdat << 16) | prevb) >> (16 - bsh);
                else      bhold = ((prevb << 16) | bdat) >> bsh;
                prevb = bdat;
            }
            if (usec) {
                cdat = b->cdat = chip_rd16(a, cpt);
                cpt += desc ? -2 : 2;
            }
            if (dodst) chip_wr16(a, dstp, ddat);
            ddat = (uint16_t)blit_func(ahold, bhold, cdat, mt);
            if (fill) ddat = do_fill(b, ddat, &fc);
            if (ddat) zero = 0;
            if (used) {
                dodst = 1;
                dstp = dpt;
                dpt += desc ? -2 : 2;
            }
        }
        if (usea) apt += desc ? -b->amod : b->amod;
        if (useb) bpt += desc ? -b->bmod : b->bmod;
        if (usec) cpt += desc ? -b->cmod : b->cmod;
        if (used) dpt += desc ? -b->dmod : b->dmod;
    }
    if (dodst) chip_wr16(a, dstp, ddat);

    b->apt = apt; b->bpt = bpt; b->cpt = cpt; b->dpt = dpt;
    b->bhold = (uint16_t)bhold;
    b->ddat = ddat;
    b->zero = zero;
}

/* ---- line mode ----------------------------------------------------------- */
static void blit_line(aga_t *a, int count)
{
    blitter_t *b = &a->blt;
    uint8_t mt = (uint8_t)(b->con0 & 0xFF);
    int shift = (b->con0 >> 12) & 15;
    int bsh = (b->con1 >> 12) & 15;
    int sign = (b->con1 & BLTCON1_SIGN) ? 1 : 0;
    int sing = b->con1 & BLTCON1_SING;
    int usea = b->con0 & BLTCON0_USEA;
    int usec = b->con0 & BLTCON0_USEC;
    int used = b->con0 & BLTCON0_USED;
    uint32_t cpt = b->cpt;
    int16_t apt = (int16_t)b->apt;   /* error accumulator lives in BLTAPTL */
    uint16_t blineb = (uint16_t)((b->bdat >> bsh) | (b->bdat << (16 - bsh)));
    int zero = 1;
    int drawn_this_row = 0;

    for (int i = 0; i < count; i++) {
        uint16_t ahold = (uint16_t)((b->adat & b->afwm & 0x8000) >> shift);
        uint16_t bhold = (blineb & 0x8000) ? 0xFFFF : 0;
        uint16_t cdat = usec ? chip_rd16(a, cpt) : b->cdat;
        uint16_t ddat = (uint16_t)blit_func(ahold, bhold, cdat, mt);
        if (ddat) zero = 0;
        if (used && !(sing && drawn_this_row)) {
            chip_wr16(a, cpt, ddat);
            drawn_this_row = 1;
        }
        b->ddat = ddat;

        /* step */
        if (usea) apt += sign ? b->bmod : b->amod;
        int moved_y = 0;
        if (!sign) {
            if (b->con1 & BLTCON1_SUD) {
                if (b->con1 & BLTCON1_SUL) { cpt -= b->cmod; moved_y = 1; }
                else                       { cpt += b->cmod; moved_y = 1; }
            } else {
                if (b->con1 & BLTCON1_SUL) { if (shift-- == 0) { shift = 15; cpt -= 2; } }
                else                       { if (++shift == 16) { shift = 0; cpt += 2; } }
            }
        }
        if (b->con1 & BLTCON1_SUD) {
            if (b->con1 & BLTCON1_AUL) { if (shift-- == 0) { shift = 15; cpt -= 2; } }
            else                       { if (++shift == 16) { shift = 0; cpt += 2; } }
        } else {
            if (b->con1 & BLTCON1_AUL) cpt -= b->cmod;
            else                       cpt += b->cmod;
            moved_y = 1;
        }
        if (moved_y) drawn_this_row = 0;
        sign = apt < 0;
        blineb = (uint16_t)((blineb << 1) | (blineb >> 15));
    }
    b->apt = (b->apt & 0xFFFF0000u) | (uint16_t)apt;
    b->cpt = cpt;
    b->dpt = cpt;
    b->con0 = (uint16_t)((b->con0 & 0x0FFF) | (shift << 12));
    b->con1 = (uint16_t)((b->con1 & ~BLTCON1_SIGN) | (sign ? BLTCON1_SIGN : 0));
    b->zero = zero;
}

/* Blit's done: drop BBUSY, raise the interrupt, wake a copper stuck waiting
   on BLTDONE. */
static void blit_finish(aga_t *a)
{
    blitter_t *b = &a->blt;
    b->busy = 0;
    b->irq_cck = 0;
    aga_intreq_set(a, INTF_BLIT);
    cop_blitter_done(a);
}

/* Delay belongs to the GLUE now, not me. A blit must never finish inside its
   own BLTSIZE write - games arm their wait AFTER the write and would then wait
   forever. The old code timed it in colour clocks off the chipset loop, which
   is fucking useless once the loop catches up: presenting a frame costs
   ~900us, the loop replays thousands of ccks in microseconds and the delay
   expires before the game sets its flag. That coupling also kept the loop from
   catching up and ran the audio off the same clock 4.5% slow. Core just flags
   "blit outstanding"; the glue has a real timer and decides when the poor
   thing is fucking done. Two clocks, independent again, and the blit still
   cannot finish inside its own write. */

/* FRF_AGA_STRICT_TIMING_V0_1
 * FAST preserves the old ~9 us compatibility delay. STRICT derives BBUSY from
 * the Hardware Reference Manual blitter-speed equation. The Amiga blitter
 * clock is the ~7.09/7.16 MHz system clock, i.e. two blitter ticks per colour
 * clock. Normal-mode clocks/word are 4 + (B?2:0) + (C&&D?2:0); line mode is
 * always 8 clocks/pixel. DMA contention is not yet modelled, so this is the
 * documented uncontended duration rather than a claim of cycle-exact bus use. */
#define BLIT_FAST_DELAY_CCK 32u

static uint32_t blit_duration_cck(const blitter_t *b, int hsize, int vsize)
{
    uint64_t units;
    unsigned clocks;
    if (b->con1 & BLTCON1_LINE) {
        units = (uint64_t)(vsize > 0 ? vsize : 1);
        clocks = 8;
    } else {
        int useb = (b->con0 & BLTCON0_USEB) != 0;
        int usec = (b->con0 & BLTCON0_USEC) != 0;
        int used = (b->con0 & BLTCON0_USED) != 0;
        units = (uint64_t)(hsize > 0 ? hsize : 1) * (uint64_t)(vsize > 0 ? vsize : 1);
        clocks = 4u + (useb ? 2u : 0u) + ((usec && used) ? 2u : 0u);
    }
    /* Two blitter/system clocks per colour clock; round up. */
    uint64_t cck = (units * clocks + 1u) >> 1;
    if (cck == 0) cck = 1;
    if (cck > 0xFFFFFFFFu) cck = 0xFFFFFFFFu;
    return (uint32_t)cck;
}

static void blit_start(aga_t *a, int hsize, int vsize)
{
    blitter_t *b = &a->blt;
    if (!aga_dma_enabled(a, DMAF_BLTEN)) {
        /* DMA off? Real hardware would just latch the blit. I run it anyway -
           every game enables DMA before blitting. */
    }
    if (b->busy) blit_finish(a);       /* new blit while the last one is still going */
    b->busy = 1;
    a->blit_count++;
    if (b->con1 & BLTCON1_LINE) blit_line(a, vsize);
    else                        blit_normal(a, hsize, vsize);

    /* The DATA is done now, but the blit must not report finished yet.
     *
     * Completing inside the BLTSIZE write raises INTF_BLIT before the write
     * instruction has even retired, and a game that starts the blit and only
     * then arms its wait - sets its "blit running" flag AFTER writing BLTSIZE -
     * gets the interrupt first. Its handler clears a flag that has not been set,
     * the game sets it a moment later, and then waits for an interrupt that
     * already happened. Banshee sits in exactly that loop for ever, the poor
     * bastard:
     *
     *   3618cd9e: move.w $3618b926(pc),d7
     *   3618cda2: bne.w  $3618cd9e            ; wait for the handler to zero it
     *   3618cda6: lea    $dff000,a6
     *   3618cdac: move.w #$0040,$9a(a6)       ; INTENA: done with INTF_BLIT
     *
     * so its main loop never reaches the copper-list writes below it, nothing
     * is ever drawn, and the screen stays blank while the game runs happy as
     * hell.
     *
     * Deferring by the time the real blitter would have taken fixes the
     * handshake and is what the hardware does anyway. It also makes BBUSY in
     * DMACONR and the copper's BLTDONE wait behave, both of which already read
     * b->busy. */
    if (a->cfg.blit_immediate) {
        blit_finish(a);
    } else {
        b->irq_cck = a->cfg.strict_timing
                   ? blit_duration_cck(b, hsize, vsize)
                   : BLIT_FAST_DELAY_CCK;
    }
}

/* Is a blit waiting to be reported finished? */
int aga_blit_pending(aga_t *a)
{
    return a->blt.busy && a->blt.irq_cck;
}

uint32_t aga_blit_delay_cck(const aga_t *a)
{
    return (a->blt.busy && a->blt.irq_cck) ? a->blt.irq_cck : 0;
}

/* Enough wall time gone by: report the bastard. */
void aga_blit_complete(aga_t *a)
{
    if (a->blt.busy && a->blt.irq_cck) blit_finish(a);
}

void blt_write(aga_t *a, uint32_t reg, uint16_t v)
{
    blitter_t *b = &a->blt;
    switch (reg) {
    case BLTCON0:  b->con0 = v; break;
    case BLTCON0L: b->con0 = (uint16_t)((b->con0 & 0xFF00) | (v & 0xFF)); break;
    case BLTCON1:  b->con1 = v; break;
    case BLTAFWM:  b->afwm = v; break;
    case BLTALWM:  b->alwm = v; break;
    case BLTCPTH:  b->cpt = (b->cpt & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16); break;
    case BLTCPTL:  b->cpt = (b->cpt & 0xFFFF0000u) | (v & 0xFFFE); break;
    case BLTBPTH:  b->bpt = (b->bpt & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16); break;
    case BLTBPTL:  b->bpt = (b->bpt & 0xFFFF0000u) | (v & 0xFFFE); break;
    case BLTAPTH:  b->apt = (b->apt & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16); break;
    case BLTAPTL:  b->apt = (b->apt & 0xFFFF0000u) | (v & 0xFFFE); break;
    case BLTDPTH:  b->dpt = (b->dpt & 0xFFFF) | ((uint32_t)(v & 0x01FF) << 16); break;
    case BLTDPTL:  b->dpt = (b->dpt & 0xFFFF0000u) | (v & 0xFFFE); break;
    case BLTSIZE: {
        int h = v & 0x3F; if (h == 0) h = 64;
        int vv = v >> 6;  if (vv == 0) vv = 1024;
        blit_start(a, h, vv);
        break;
    }
    case BLTSIZV:  b->sizev = v & 0x7FFF; break;
    case BLTSIZH: {
        int h = v & 0x7FF; if (h == 0) h = 2048;
        int vv = b->sizev; if (vv == 0) vv = 32768;
        blit_start(a, h, vv);
        break;
    }
    case BLTCMOD:  b->cmod = (int16_t)(v & 0xFFFE); break;
    case BLTBMOD:  b->bmod = (int16_t)(v & 0xFFFE); break;
    case BLTAMOD:  b->amod = (int16_t)(v & 0xFFFE); break;
    case BLTDMOD:  b->dmod = (int16_t)(v & 0xFFFE); break;
    case BLTCDAT:  b->cdat = v; break;
    case BLTBDAT:  b->bdat = v; b->bhold = v; break;
    case BLTADAT:  b->adat = v; break;
    default: break;
    }
}
