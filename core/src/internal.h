/* AGA-PISTORM — chipset core private state.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef AGA_INTERNAL_H
#define AGA_INTERNAL_H

#include <stdint.h>
#include <string.h>
#include "aga/chipset.h"

/* ---- chip RAM access (big-endian) ---------------------------------------- */
static inline uint16_t chip_rd16(const aga_t *a, uint32_t addr);
static inline uint32_t chip_rd32(const aga_t *a, uint32_t addr);
static inline uint64_t chip_rd64(const aga_t *a, uint32_t addr);
static inline void     chip_wr16(aga_t *a, uint32_t addr, uint16_t v);

/* a CLXDAT read keeps collision detection on for this many frames (2 s PAL) */
#define CLX_FRAMES 100

#define RES_LORES 0
#define RES_HIRES 1
#define RES_SHRES 2

/* copper states */
enum {
    COP_stop = 0,
    COP_read1,          /* fetch first word */
    COP_read2,          /* fetch second word */
    COP_wait_in2,       /* WAIT: 2 cycles delay before compare */
    COP_wait,           /* waiting for beam position */
    COP_skip_in2,
    COP_bltwait,        /* waiting for blitter */
    COP_strobe_delay,   /* COPJMP written: restart after a fetch cycle */
};

typedef struct copper {
    uint32_t cop1lc, cop2lc;
    uint32_t pc;
    uint16_t i1, i2;
    int      state;
    int      vcmp, hcmp, hmask, vmask, blitwait;
    int      danger;            /* COPCON CDANG */
    int      enabled_line;      /* copper DMA active on this line */
    int      strobe;            /* pending COPJMP1/2 written by the copper itself (1/2), 0 = none */
    int      strobe_count;      /* instructions still to complete before it takes effect */
    int      in_move;           /* cop_sync is executing a MOVE (aga_custom_wput in progress) */
    int      ignore_next;
    int      delay;             /* idle steps before the next access */
    int      last_hpos;         /* copper synced up to this hpos (even) */

    /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6: v1.105 pending Copper pointer reload.
       1=COP1LC, 2=COP2LC; sampled at the first actual DMA fetch. */
    uint8_t reload;
} copper_t;

typedef struct blitter {
    uint16_t con0, con1, afwm, alwm;
    uint32_t apt, bpt, cpt, dpt;
    int16_t  amod, bmod, cmod, dmod;
    uint16_t adat, bdat, cdat, ddat;
    uint16_t sizev, sizeh;        /* pending BLTSIZV value / lines */
    int      busy, zero;
    uint32_t irq_cck;       /* FRF_AGA_STRICT_TIMING_V0_1_2: requested BBUSY lifetime in colour clocks */
    int      line_mode;
    uint16_t bhold;               /* preloaded B */
} blitter_t;

typedef struct sprite {
    uint32_t pt;
    uint16_t pos, ctl;
    uint64_t data, datb;          /* up to 64 bits (FMODE) */
    int      armed;               /* DATA written since CTL */
    int      dmastate;            /* 1 = fetching data lines */
    int      vstart, vstop, xpos; /* xpos in shres pixels (4 per lores pixel) */
    int      attach;
} sprite_t;

typedef struct audio_channel {
    uint32_t lc, pt;              /* location (latched) and current pointer */
    uint16_t len, lencnt;
    uint16_t per, percnt;
    uint16_t vol;
    uint16_t dat;                 /* AUDxDAT last written */
    uint16_t dat2;                /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: DMA/CPU secondary word latch */
    int      dmaen;
    int      state;               /* Paula state machine 0..5 */
    int      dat_written;
    int      request_word;         /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: E-UAE DMA request state (-1/0/1/2/3) */
    int      request_word_skip;    /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: defer startup fetch at end of line */
    int8_t   cur_sample;          /* sample currently output */
    int8_t   last_sample;         /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: previous DAC sample */
    int      intreq2;             /* pending interrupt state (per HRM) */
    /* resampling accumulator */
    int32_t  acc;
} audio_channel_t;

/* One register change recorded within a line for the renderer.
 *
 * Real custom-register offsets occupy only $000..$1FE.  AGA palette writes,
 * however, can target 256 logical colours through 32 banked COLORxx
 * registers.  Older code encoded logical colour N as COLOR00 + N*2; that
 * synthetic range ($180..$37E) overlaps real AGA registers such as DIWHIGH
 * ($1E4), so a DIWHIGH raster write could be replayed as palette entry 50.
 * Keep palette events in a disjoint namespace instead. */
#define AGA_LINECHANGE_COLOR_FLAG      0x8000u
#define AGA_LINECHANGE_COLOR(index)    (AGA_LINECHANGE_COLOR_FLAG | ((uint16_t)(index) & 0x00FFu))
#define AGA_LINECHANGE_IS_COLOR(code)  (((uint16_t)(code) & AGA_LINECHANGE_COLOR_FLAG) != 0)
#define AGA_LINECHANGE_COLOR_INDEX(code) ((uint16_t)(code) & 0x00FFu)

/* AGA BPLCON4.BPLAM XORs the complete 8-bit bitplane colour address. */
static inline unsigned aga_bplam_index(unsigned address, unsigned mask)
{
    return (address ^ mask) & 0xFFu;
}

typedef struct linechange {
    uint16_t hpos;                /* colour clock */
    uint16_t reg;                 /* real register offset, or AGA_LINECHANGE_COLOR(index) */
    uint32_t value;               /* palette event: expanded 24-bit RGB; register event: raw */
} linechange_t;

/* virtual floppy drives (disk.c) */
#define DSK_WORDS_PER_LINE 2          /* DD: 32 us per MFM word, ~2 words per scan line */
typedef struct disk_drive {
    uint8_t  *image;                  /* ADF image (copy) */
    uint32_t  size;
    int       inserted, wprot, dirty;
    int       sectors;                /* 11 (DD) or 22 (HD) per track */
    int       cyl, side, motor, changed;
    uint32_t  id_shift;
    uint16_t *mfm;                    /* current track, MFM words */
    int       mfm_words, mfm_track, mfm_valid;
    int       pos;                    /* word under the head */
} disk_drive_t;

typedef struct disk {
    disk_drive_t drv[AGA_MAX_DRIVES];
    int      selected;                /* virtual drive selected via CIA-B PRB, -1 none */
    uint8_t  prb;                     /* last CIA-B PRB value */
    uint16_t dsklen, dsksync, dskdat, dskbytr;
    int      dsklen_armed;            /* first of the two DMAEN writes seen */
    uint32_t dskpt;
    int      dma_active, dma_write, dma_left, synced, wordequal;
} disk_t;

typedef struct playfield {
    /* register values */
    uint32_t bplpt[AGA_MAX_PLANES];
    uint16_t bplcon0, bplcon1, bplcon2, bplcon3, bplcon4;
    int ecs_mode;                  /* ECS Denise semantics: no LOCT, no colour bank */
    int16_t  bpl1mod, bpl2mod;
    uint16_t diwstrt, diwstop, diwhigh;
    int      diwhigh_written;
    uint16_t ddfstrt, ddfstop;
    uint16_t fmode;
    uint16_t bpldat[AGA_MAX_PLANES];
    /* derived */
    int      res;                 /* 0 lores, 1 hires, 2 shres */
    int      planes;              /* requested planes from BPLCON0 */
    int      fm;                  /* fetch mode 0,1,2 */
    int      fetchunit, fetchstart, fetchstart_shift, maxplanes;
    int      plfstrt, plfstop;    /* effective DDF start/stop (cck) */
    int      diw_hstart, diw_hstop;   /* superhires coordinates (DIW units, 4 per lores) */
    int      diw_vstart, diw_vstop;
    /* per-line fetch state */
    int      vdiw;                /* vertical DIW flip-flop */
    int      vdiw_ff;             /* FRF_AGA_UP104_SAFE_VDIW_FF: raw DIW set/clear latch */
    int      fetch_active;        /* fetching on this line */
    int      fetch_cycle;         /* cck since plfstrt */
    int      fetch_last_hpos;     /* fetch synced up to this hpos */
    int      fetch_total;         /* total fetch cycles for this line */
    int      fetch_start_h;       /* hpos of the first fetch cycle this line (-1 = not started) */
    int      fetch_done;          /* fetch finished this line (modulos added) */
    int      line_plfleft;        /* hpos where first block became visible (-1 none) */
    int      line_blocks;         /* number of blocks fetched */
    int      line_res, line_fm, line_planes;  /* latched at fetch start */
    int      line_fmode_bpl;              /* FRF V0.1.7: original FMODE[1:0], alignment semantics */
    uint16_t line_fmode;                  /* FRF V0.2.2: full FMODE for BSCAN2 modulo semantics */
    int      line_delay1, line_delay2;         /* native-pixel delays (odd/even planes) */
    int      line_bplcon0;

    /* E-UAE/AGA split timing: Denise observes a BPLCON0 write before Agnus
       changes the bitplane DMA schedule. Keep the visible register in bplcon0
       and defer only the DMA-derived mode (res/planes/fetch diagram). */
    uint16_t dma_bplcon0_pending_value;
    int      dma_bplcon0_pending;
    int      dma_bplcon0_apply_hpos;
    /* planar line data: bit i of plane p = native pixel (block origin) + i, MSB first */
    uint64_t bits[AGA_MAX_PLANES][AGA_PLANE_WORDS];
    /* raw fetched data per block and the scroll delays valid when the block completed */
    uint64_t blockdata[AGA_MAX_BLOCKS][AGA_MAX_PLANES];
    int16_t  blockdelay1[AGA_MAX_BLOCKS], blockdelay2[AGA_MAX_BLOCKS];
    /* chunky index buffers for the current line (native resolution), odd and even planes
       separately because they are shifted by different BPLCON1 delays at display time */
    uint8_t  idx_odd[AGA_PLANE_WORDS * 64 + 64];
    uint8_t  idx_even[AGA_PLANE_WORDS * 64 + 64];
    int      line_delayoffset;    /* UAE delay offset for unaligned fetch starts (lores px) */
} playfield_t;

struct aga {
    aga_config_t cfg;
    aga_hooks_t  hooks;
    uint8_t     *chip;
    uint32_t     chipmask;
    void       (*log)(void *, const char *);
    void        *log_user;

    /* beam */
    int hpos, vpos, lof, maxhpos, maxvpos, ntsc;
    int vblank_end;
    int sprite_vblank;          /* line the sprite DMA restarts on (UAE sprite_vblank_endline) */
    int frame;
    int lof_toggle;

    /* generic registers */
    uint16_t regs[256];
    uint16_t dmacon, intena, intreq, adkcon;
    uint16_t clxcon, clxcon2, clxdat;
    uint16_t beamcon0;
    uint16_t copcon;
    int      ipl;
    int      palette_guard;          /* FRF_AGA_STRICT_TIMING_V0_1_2: one-shot stale-LOCT handover guard */

    copper_t   cop;
    blitter_t  blt;
    sprite_t   spr[AGA_MAX_SPRITES];
    int        sprite_width;       /* 16/32/64 */
    int        sprres;             /* sprite resolution 0..2 */
    audio_channel_t aud[4];
    playfield_t pf;

    /* palette: 256 x 24-bit, plus the AGA COLORxx genlock sideband bit.
       The genlock bit is written with the MSB colour half and must survive
       LOCT low-nibble writes so RDRAM can read back the real register value. */
    uint32_t color[256];
    uint8_t  color_genlock[256];   /* FRF_AGA_COLOR_DISPLAY_CORE_V0_1_6 */

    /* per-line change list */
    linechange_t changes[AGA_LINE_CHANGES];
    int          nchanges;

    /* sprite line buffer, in shres pixels relative to lores pixel 0 (DIW coords):
       bits 0-3 colour (within 16), bit 4 attached, bits 5-6 pair, bit 7 present */
    uint8_t  sprline[AGA_OUT_W * 2 + 512];
    int      sprline_valid;

    /* Collision detection (CLXDAT). Worked out only while the program reads
       CLXDAT - most never do, and it costs time on every pixel - or always
       with cfg.collisions. sprclx has the same pixels as sprline but keeps
       every sprite apart: bit n = sprite n has a pixel there. */
    int      clx_frames;           /* frames left, re-armed by every CLXDAT read */
    uint8_t  sprclx[AGA_OUT_W * 2 + 512];
    int      sprclx_valid;

    /* output */
    /* Where finished lines go. The two frames below by default; the ARM side
       points them at the HVS's own pages instead (aga_set_framebuffers) and
       marks them external, which adds the alpha byte the HVS wants. Lines land
       in linebuf first: the renderer writes a pixel at a time, and an external
       page is uncached memory where one streaming copy per line is the only
       sane way to do it. */
    uint32_t *fb[2];
    int      fb_external;
    uint32_t linebuf[AGA_OUT_W];
    uint32_t fb_store[2][AGA_OUT_W * AGA_OUT_H];
    int      fb_cur;               /* index of frame being drawn */
    /* Frame skip. skip_req is the host's wish, latched at each frame end into
       frame_skip for the whole next frame: its lines are not drawn and the
       pages are not swapped after it, so what is on screen stays on screen and
       the next drawn frame goes to the other page. Never while collisions are
       being computed (they come out of the renderer), never more than
       SKIP_RUN_MAX frames in a row. */
    int      skip_req;
    int      frame_skip;
    int      skip_run;
    uint32_t frames_skipped;
    int      frame_done_count;
    uint32_t blit_count;           /* blits started since reset (diagnostics) */
    uint32_t color_reads;          /* V0.1.4 AGA RDRAM COLORxx reads (diagnostics) */
    uint32_t aud_on[4];            /* audio DMA switched on, per channel (diagnostics) */
    uint32_t aud_words[4];         /* sample words fetched, per channel (diagnostics) */
    uint32_t aud_nz[4];            /* |sample * vol| accumulated: what actually reaches the mix */
    uint32_t aud_volmax[4];        /* loudest AUDxVOL ever set on this channel */
    uint32_t aud_volwr[4];         /* AUDxVOL writes seen, per channel */
    uint32_t aud_volnz[4];         /* ...of which asked for a NON-zero volume */
    uint32_t aud_dmaoff[4];        /* times the game switched this channel's DMA OFF */
    uint32_t aud_wrap[4];          /* times the sample looped back to AUDxLC */
    uint32_t aud_irq[4];           /* times INTF_AUDx was raised for it */
    /* What the channel loops BACK to, sampled every 512th wrap - the loops run
       tens of thousands of times, so every one of them cannot be probed. */
    uint32_t aud_wrapinfo[32][4];  /* chan, lc, len, peak of the loop buffer */
    uint32_t aud_wrapinfo_head;
    uint32_t aud_audible[4];       /* steps with a non-zero sample AND non-zero volume */
    /* The last 32 times a channel's DMA was switched on, with everything that
       decides whether it makes a sound: 9 of every 10 starts on the effects
       channel produce silence, and one of these four has to fucking say why. */
    uint32_t aud_trig[32][6];      /* chan, lc, len, per, vol, first sample word */
    uint32_t aud_trig_head;
    int      aud_mask;             /* bit n: mix channel n. 0 = not yet set = all */
    int      pf_data_nonzero;      /* a bitplane fetch in this frame returned non-zero data */
    int      last_frame_data;      /* same, for the last completed frame */
    int      display_lines;        /* lines with bitplane data in the frame being drawn */
    int      last_display_lines;   /* same for the last completed frame */
    uint16_t aga_ctrl;             /* AGA control register 0xDFF1F2 (native/RTG switch, windowed) */
    uint16_t aga_win[4];           /* 0xDFF1F4..0xDFF1FA: window x, y, w, h on the RTG screen */

    /* audio output */
    int      audio_rate;
    int32_t  audio_frac;

    /* virtual floppy (allocated separately: survives aga_reset) */
    disk_t  *dsk;

#ifdef AGA_TRACE
    aga_slot_t *trace[2];       /* [AGA_MAX_VPOS][AGA_MAX_HPOS], double buffered like fb */
#endif
};

/* Record a DMA slot. Compiled out entirely unless AGA_TRACE is defined. */
static inline void aga_trace_at(aga_t *a, int hpos, uint8_t type, uint16_t reg, uint32_t addr)
{
#ifdef AGA_TRACE
    aga_slot_t *buf = a->trace[a->fb_cur];
    if (buf && a->vpos >= 0 && a->vpos < AGA_MAX_VPOS && hpos >= 0 && hpos < AGA_MAX_HPOS) {
        aga_slot_t *s = &buf[a->vpos * AGA_MAX_HPOS + hpos];
        s->type = type; s->reg = reg; s->addr = addr;
    }
#else
    (void)a; (void)hpos; (void)type; (void)reg; (void)addr;
#endif
}

/* custom.c */
void aga_intreq_set(aga_t *a, uint16_t bits);
void aga_update_ipl(aga_t *a);
void aga_record_change(aga_t *a, uint32_t reg, uint32_t value);
int  aga_dma_enabled(const aga_t *a, uint16_t mask);
void aga_logf(aga_t *a, const char *fmt, ...);

/* copper.c */
void cop_reset(aga_t *a);
void cop_start_line(aga_t *a);
void cop_sync(aga_t *a, int hpos);
void cop_vsync(aga_t *a);
void cop_write_lc(aga_t *a, int which, int high, uint16_t v);
void cop_strobe(aga_t *a, int which);
void cop_blitter_done(aga_t *a);

/* blitter.c */
void blt_reset(aga_t *a);
void blt_write(aga_t *a, uint32_t reg, uint16_t v);
int  aga_blit_pending(aga_t *a);
void aga_blit_complete(aga_t *a);
uint16_t blt_dmaconr_bits(const aga_t *a);

/* playfield.c */
void pf_reset(aga_t *a);
void pf_write(aga_t *a, uint32_t reg, uint16_t v, int hpos);
void pf_start_line(aga_t *a);
void pf_sync(aga_t *a, int hpos);
void pf_finish_line(aga_t *a);     /* fetch remainder, render line into framebuffer */
void pf_vsync(aga_t *a);
void pf_dma_enable_sync(aga_t *a); /* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_3_DECL */ /* FRF_AGA_JOTD_BITPLANE_DMA_GATE_V0_1: converge Agnus BPL state before BPL DMA starts */
void pf_color_write(aga_t *a, int num, uint16_t v, int hpos);
uint16_t pf_con3(const playfield_t *p);
uint16_t pf_con4(const playfield_t *p);
int  pf_is_bpl_slot(aga_t *a, int hpos);   /* 1 if a bitplane fetch occupies this cycle */

/* sprites.c */
void spr_reset(aga_t *a);
void spr_write(aga_t *a, uint32_t reg, uint16_t v);
void spr_dma_slot(aga_t *a, int hpos);
void spr_build_line(aga_t *a, int clx);   /* fill a->sprline (and a->sprclx if clx) for the current line */
void spr_vsync(aga_t *a);
void spr_expand_res(aga_t *a);

/* disk.c */
void dsk_create(aga_t *a);
void dsk_destroy(aga_t *a);
void dsk_reset(aga_t *a);
int  dsk_read(aga_t *a, uint32_t reg, uint16_t *v);     /* 1 = handled (virtual disk present) */
int  dsk_write(aga_t *a, uint32_t reg, uint16_t v);
void dsk_line_tick(aga_t *a);

/* audio.c */
void aud_reset(aga_t *a);
void aud_write(aga_t *a, uint32_t reg, uint16_t v);
void aud_dma_slot(aga_t *a, int hpos);
void aud_cck_tick(aga_t *a);              /* FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1: one Paula colour clock */
void aud_dmacon_changed(aga_t *a, uint16_t old, uint16_t new_);
void aud_line_tick(aga_t *a);

/* ---- inline helpers ------------------------------------------------------- */
static inline uint16_t chip_rd16(const aga_t *a, uint32_t addr)
{
    /* FRF_AGA_JOTD_BITPLANE_COHERENCY_V0_1
     * The chipset runs on CPU3 while the 68k/JIT writes virtual Chip RAM on
     * CPU0.  E-UAE is effectively single-owner here; our backing store is
     * genuinely shared.  Volatile byte loads prevent the compiler from
     * retaining DMA data across accesses and make each DMA fetch observe the
     * coherent ARM backing memory. */
    const volatile uint8_t *chip = (const volatile uint8_t *)a->chip;
    addr &= a->chipmask;
    return (uint16_t)((chip[addr] << 8) | chip[(addr + 1) & a->chipmask]);
}
static inline uint32_t chip_rd32(const aga_t *a, uint32_t addr)
{
    return ((uint32_t)chip_rd16(a, addr) << 16) | chip_rd16(a, addr + 2);
}
static inline uint64_t chip_rd64(const aga_t *a, uint32_t addr)
{
    return ((uint64_t)chip_rd32(a, addr) << 32) | chip_rd32(a, addr + 4);
}
static inline void chip_wr16(aga_t *a, uint32_t addr, uint16_t v)
{
    addr &= a->chipmask;
    a->chip[addr] = (uint8_t)(v >> 8);
    a->chip[(addr + 1) & a->chipmask] = (uint8_t)v;
}

#define GET_RES(con0)     (((con0) & BPLCON0_SHRES) ? RES_SHRES : ((con0) & BPLCON0_HIRES) ? RES_HIRES : RES_LORES)
#define GET_PLANES(con0)  (((((con0) >> 12) & 7) | (((con0) & BPLCON0_BPU3) >> 1)))

#endif /* AGA_INTERNAL_H */
