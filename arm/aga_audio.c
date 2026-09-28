/* AGA-PISTORM — Paula audio through the real Paula, used as a DAC.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * The games and AHI's paula.audio both talk to the virtual Paula; the
 * emulated mix (stereo, 8 bit, ~27.9 kHz) gets streamed into small loop
 * buffers in the REAL chip RAM (dead space in AGA mode) which real Paula
 * channels 0 (left) and 1 (right) play back forever. The stream is written a
 * little ahead of the DMA read position estimated off the Pi timer; when the
 * two clocks drift apart the loop is restarted - an inaudible hiccup every
 * few minutes, or so the first version of this file claimed.
 *
 * All bus writes here come from the chipset core (core 3) and go through the
 * bus mutex added to Emu68's ps_protocol (see emu68-aga.patch), so they never
 * interleave with the 68k's own bus transactions on core 0.
 */
#include <stdint.h>
#include "aga/chipset.h"
#include "aga_glue.h"

extern void kprintf(const char *fmt, ...);
#include "ps_protocol.h"    /* the bus API differs per board; do not re-declare it */

#define PAULA_PERIOD     127                       /* 3546895 / 127 = 27928 Hz */
#define RING_SAMPLES     2048                      /* per channel, bytes */
/* Real chip RAM addresses for the Paula DMA rings. Paula can only fetch from
   chip RAM, so these have to live there - fast RAM is not an option however
   tempting.

   The ring is OWNED, not assumed: agaboot does AllocMem(MEMF_CHIP) before
   sandbox entry and passes the address in through the control register (see
   aga_audio_set_ring). Two earlier hard-coded addresses each corrupted memory
   that belonged to somebody else:

     0x00000/0x01000  straight over the 68k exception vector table. Harmless
       in MACHINE mode, where the real chip RAM is dead space, fatal in
       SANDBOX mode where AmigaOS is live and its vectors are down there.
       Measured 2026-09-07: rungs 3, 4 and 5 all sustain 50 Hz because none
       of them call aga_audio_stream; rung 0 calls it and the machine dies at
       once.

     0x0FE000/0x0FF000  the last 8 KB of the first megabyte. Survives a bare
       Workbench, which is why a 90-second rung-0 run looked clean, and dies
       the moment anything actually allocates chip RAM - `WHDLoad ...
       PRELOAD` takes what it can get, is handed that region, and then I
       write 8 KB of audio over the game 437 times a second. My own damn
       fault.

   No fixed address can be safe in sandbox mode, because the OS owns all of
   chip RAM. AGA_RING_FALLBACK is used only when agaboot is too old to pass
   one, and carries the second bug knowingly; it stays inside the first
   megabyte so it behaves the same on an A500 with 1 MB of chip as on a 2 MB
   machine. */
static volatile int running;

#define AGA_RING_FALLBACK 0x0FE000
/* Four loops, not two: left high, left low, right high, right low.
 *
 * One 8-bit Paula channel per side threw away most of the goddamn mix. A
 * channel at sample 128 and volume 44 gives 5632, which my mixer turned into
 * 44 out of 127, where real hardware produces 127*44/64 = 87 - I was 6 dB
 * down, because hardware sums two channels per side in the ANALOG domain and
 * can swing to twice one DAC's range while I had to fit that sum into a
 * single byte. And everything was then quantised to 8 bits, so a channel at
 * volume 10 came out with about THREE bits of resolution. That is what "some
 * channels are really quiet" was, and why low-volume sound effects vanished
 * entirely.
 *
 * All four Paula channels are mine - the game's audio is fully emulated and
 * none of its AUDxx writes reach the real chip - so pair them. Channels 0 and
 * 3 are both LEFT on Amiga hardware, 1 and 2 both RIGHT, so each side gets a
 * high byte at volume 64 and a low byte at volume 1. Paula sums them itself:
 *
 *     out = hi * (64/64) + lo * (1/64) = (hi * 64 + lo) / 64
 *
 * a 14-bit value played through 8-bit DACs, and twice the level I had. 64
 * times the resolution for the quiet channels that were disappearing. */
#define RING_BYTES        (4u * RING_SAMPLES)      /* lhi, llo, rhi, rlo */

/* Who writes the real chip RAM: NOT core 3 any more.

   Every group of four samples used to go out as four ps_write_32 calls from
   the chipset loop, and on a Pi 3 one such push stalled the loop for 5 ms
   where it normally costs 0.15 ms (2026-09-14) - two thirds of that board's
   resyncs, and 1.2 ms of every frame on a Pi 4. All that shit for four
   samples. So core 3 now only fills the local byte rings and publishes how
   far it got (pub_wr, released after the bytes); the housekeeper core, which
   already owns bus traffic for the IPL line, drains them from its own loop,
   one group per visit so the interrupt sampling never waits behind more than
   four bus writes. Restarts and the once-a-second register refresh go the
   same way: core 3 asks, the housekeeper does, and sets start_tick when the
   DMA really started so the read-position model is not a guess. flush_pos,
   the ring clear and start_tick's write are the housekeeper's; core 3 reads
   start_tick only while no restart is pending. */
static uint32_t flush_pos;          /* housekeeper: first sample not yet pushed to chip RAM */
static volatile uint32_t pub_wr;    /* core 3 -> housekeeper: samples up to here are in the local rings */
static volatile int restart_req;    /* core 3 -> housekeeper: (re)start the DMA loops */
static volatile int reinit_req;     /* core 3 -> housekeeper: refresh the Paula registers */
static volatile int svc_busy;       /* housekeeper is inside the service right now */
static uint32_t clear_pos;          /* housekeeper: ring clear progress on a fresh start */
static int      svc_state;          /* 1 = clearing the rings, 2 = program Paula, 3 = running */
          /* first sample not yet pushed to chip RAM */
static uint32_t left_ring  = AGA_RING_FALLBACK;                     /* ch0: left  high */
static uint32_t right_ring = AGA_RING_FALLBACK + RING_SAMPLES;      /* ch1: right high */
static uint32_t llo_ring   = AGA_RING_FALLBACK + 2 * RING_SAMPLES;  /* ch3: left  low  */
static uint32_t rlo_ring   = AGA_RING_FALLBACK + 3 * RING_SAMPLES;  /* ch2: right low  */
static int      ring_owned;                        /* agaboot allocated it */

/* FRF_AGA_NATIVE_PAULA_V0_1 -------------------------------------------------
 * Native mode uses the real Paula as Paula, not merely as a DAC. Virtual Paula
 * still advances inside the AGA core so game-visible DMA/IRQ timing stays
 * authoritative. The physical chip only receives translated sample blocks and
 * AUDx control state; its AUD interrupts are forcibly disabled.
 *
 * The allocation handed over by agaboot always reserves the first RING_BYTES
 * for the known software fallback. The rest is split into four fixed staging
 * slots, one per hardware channel. A too-large sample switches the session to
 * software instead of repacking live slots or touching unowned Chip RAM.
 */
#define AENG_SOFTWARE 0u
#define AENG_NATIVE   1u
#define AENG_AUTO     2u
#define REAL_CHIP_LIMIT 0x00080000u

typedef struct native_aud_chan {
    volatile uint32_t lc;
    volatile uint16_t len, per, vol, dat;
    volatile uint32_t src_bytes;
    volatile uint32_t staged_lc;
    volatile uint16_t staged_len;
    uint32_t phys;
    volatile uint32_t req_gen;
    uint32_t svc_gen;
    uint32_t copy_pos;
    volatile uint8_t desired_on;
    volatile uint8_t stop_pending;
    volatile uint8_t copy_pending;
    volatile uint8_t dat_pending;
    volatile uint8_t per_pending;
    volatile uint8_t vol_pending;
    uint8_t prog_step;
    uint8_t hw_on;
} native_aud_chan_t;

static volatile unsigned audio_engine_mode=AENG_AUTO;
static volatile uint32_t audio_buffer_bytes=RING_BYTES;
static const uint8_t *native_chipram;
static uint32_t native_chipram_size;
static uint32_t native_pool_base, native_slot_bytes;
static native_aud_chan_t native_ch[4];
static volatile uint16_t native_dmacon;
static volatile uint16_t native_adkcon_write;
static volatile uint8_t native_adkcon_pending;
static volatile uint8_t native_fallback;
static volatile uint8_t native_stop_all_pending;
static uint8_t native_init_step;
static unsigned native_rr;
static int native_fallback_moaned;

static int native_requested(void)
{
    return audio_engine_mode != AENG_SOFTWARE && !native_fallback &&
           ring_owned && native_slot_bytes >= 512u && native_chipram;
}

int aga_audio_software_needed(void)
{
    return !native_requested();
}

static uint32_t pool_bytes_from_code(unsigned code)
{
    switch (code & 15u) {
    case 1: return 32768u;
    case 2: return 49152u;
    case 3: return 65536u;
    case 4: return 98304u;
    default:return RING_BYTES;
    }
}

void aga_audio_set_buffer_size_code(unsigned code)
{
    audio_buffer_bytes=pool_bytes_from_code(code);
}

static void native_reset_channels(void)
{
    for (unsigned i=0;i<4;i++) {
        native_ch[i]=(native_aud_chan_t){0};
    }
    native_dmacon=0;
    native_adkcon_pending=0;
    native_stop_all_pending=0;
    native_init_step=0;
    native_rr=0;
    native_fallback=0;
    native_fallback_moaned=0;
}

void aga_audio_set_engine(unsigned mode)
{
    if (mode > AENG_AUTO) mode=AENG_AUTO;
    audio_engine_mode=mode;
    native_reset_channels();
    kprintf("[AGA] audio engine: %s\n",
            mode==AENG_SOFTWARE ? "SOFTWARE MIXER" :
            mode==AENG_NATIVE ? "NATIVE PAULA" : "AUTO (NATIVE -> SOFTWARE fallback)");
}

static void native_force_fallback(const char *why)
{
    if (native_fallback) return;
    native_fallback=1;
    native_stop_all_pending=1;
    if (!native_fallback_moaned) {
        native_fallback_moaned=1;
        kprintf("[AGA] native Paula fallback: %s\n",why);
    }
}

static int native_channel_shape(unsigned c, uint32_t *src, uint32_t *bytes)
{
    native_aud_chan_t *n=&native_ch[c];
    uint32_t words=n->len ? (uint32_t)n->len : 65536u;
    uint32_t b=words<<1;
    uint32_t a=n->lc & ~1u;
    if (!native_chipram || a >= native_chipram_size || b > native_chipram_size-a) return 0;
    if (!native_slot_bytes || b > native_slot_bytes) return 0;
    *src=a; *bytes=b; return 1;
}

static void native_stage(unsigned c)
{
    uint32_t src,bytes;
    if (!native_channel_shape(c,&src,&bytes)) {
        native_force_fallback("sample does not fit the owned physical staging slot");
        return;
    }
    native_aud_chan_t *n=&native_ch[c];
    n->src_bytes=bytes;
    n->staged_lc=n->lc;
    n->staged_len=n->len;
    n->copy_pos=0;
    n->svc_gen=0;
    n->prog_step=0;
    __atomic_add_fetch(&n->req_gen,1u,__ATOMIC_RELEASE);
    __atomic_store_n(&n->copy_pending,1u,__ATOMIC_RELEASE);
}


/* Called from the control-register trap before sandbox entry. base is a 68k
   chip RAM address of RING_BYTES that agaboot has allocated and will free. */
void aga_audio_set_ring(uint32_t base)
{
    uint32_t bytes=audio_buffer_bytes;
    if (bytes < RING_BYTES) bytes=RING_BYTES;
    /* This FRF physical-sink branch has 512K genuinely DMA-visible Chip RAM.
       Reject AGALEND's virtual MEMF_CHIP addresses above it. */
    if (base < 0x400 || (base & 3) || base + bytes > REAL_CHIP_LIMIT) {
        kprintf("[AGA] audio: rejecting buffer %08x + %u (not real low Chip RAM)\n",base,bytes);
        ring_owned=0; native_pool_base=0; native_slot_bytes=0;
        return;
    }
    left_ring  = base;
    right_ring = base + RING_SAMPLES;
    llo_ring   = base + 2 * RING_SAMPLES;
    rlo_ring   = base + 3 * RING_SAMPLES;
    ring_owned = 1;
    running    = 0;
    native_pool_base = base + RING_BYTES;
    native_slot_bytes = bytes > RING_BYTES ? ((bytes-RING_BYTES)/4u)&~3u : 0u;
    for (unsigned i=0;i<4;i++) native_ch[i].phys=native_pool_base+i*native_slot_bytes;
    kprintf("[AGA] audio: buffer %08x %u bytes; native slot=%u bytes\n",
            base,bytes,native_slot_bytes);
}

/* Sandbox leave: agaboot is about to FreeMem the buffer, so stop trusting it. */
static void ring_release(void)
{
    left_ring  = AGA_RING_FALLBACK;
    right_ring = AGA_RING_FALLBACK + RING_SAMPLES;
    llo_ring   = AGA_RING_FALLBACK + 2 * RING_SAMPLES;
    rlo_ring   = AGA_RING_FALLBACK + 3 * RING_SAMPLES;
    ring_owned = 0;
    native_pool_base = 0;
    native_slot_bytes = 0;
    audio_buffer_bytes = RING_BYTES;
}

/* How far ahead of the DMA I write. 512 samples is 18 ms on a Pi 4, whose
   frames never take longer than that. The Pi 3's worst frame with video on
   measured 31 ms (2026-09-14): Paula ate through an 18 ms lead on every such
   spike and the ring restarted, 70 times in 33 seconds - the wobble in the
   music. 1024 samples is 37 ms, which covers it, at the price of 19 ms more
   latency on that board only. */
#define LEAD_PI4         512
#define LEAD_PI3         1024
static uint32_t lead_target;                   /* set at the first stream call */
#define LEAD_SAMPLES     lead_target
/* How far the lead may wander before it is worth correcting.
 *
 * It used to correct whenever the lead was more than ONE sample from target,
 * and the lead is not a measurement - it is an ESTIMATE, off a timer, of
 * where Paula has got to, against a producer that runs in bursts around each
 * frame present. So the corrector never settled: it dithered, dropping a
 * sample then repeating one, about 92 times a second. Measured over one game:
 *   aud_drops 5652, aud_repeats 5551
 * - within 2% of each other, i.e. no net drift at all, just 92
 * discontinuities a second chasing noise. That is what "catching other tones,
 * some kind of lag" sounds like, and it fucks up short sound effects worst,
 * since a 60 ms effect spans several of them.
 *
 * A deadband of 64 samples is 2.3 ms - far larger than the burst jitter, far
 * smaller than the 512-sample lead or the resync window either side of it.
 * Real crystal drift is about 0.005%, or 1.4 samples a second, so genuine
 * drift still gets corrected, roughly once every 45 seconds instead of 92
 * times a second. */
#define LEAD_DEADBAND    64
/* Applied in the 14-bit domain, where it costs no resolution. 2 is safe against
   a measured gameplay peak of 45/127; raise it only with a fresh measurement. */
#define AUD_GAIN         2
/* The window outside which the lead is declared lost. 1024 + a frame's
   burst stays well under 1900; the ring itself is 2048 (73 ms). */
#define RESYNC_LOW       64
#define RESYNC_HIGH      1900
#define REINIT_INTERVAL  50                        /* frames between register refreshes (1 s) */

static uint32_t wr_pos;             /* next sample index to write (0..RING_SAMPLES-1) */
static uint64_t start_tick;         /* timer value when the DMA loop was (re)started */
#define SAMPLE_HZ (3546895u / PAULA_PERIOD)   /* 27928 Hz, the rate Paula plays the ring at */
static uint64_t ticks_per_sample_fp;/* 32.32 */
static int      reinit_countdown;
static int16_t  pcm[4096];          /* scratch: interleaved stereo from the core */
static uint8_t  lbuf[RING_SAMPLES], rbuf[RING_SAMPLES];     /* high bytes */
static uint8_t  llob[RING_SAMPLES], rlob[RING_SAMPLES];     /* low bytes  */
static int      have;               /* samples pending in lbuf/rbuf */

static inline uint64_t cntvct(void)
{
    uint64_t v;
    __asm__ volatile("mrs %0, CNTVCT_EL0" : "=r"(v));
    return v;
}
static inline uint64_t cntfrq(void)
{
    uint64_t v;
    __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(v));
    return v;
}

void aga_audio_init(const uint8_t *chipram, uint32_t chipram_size)
{
    native_chipram=chipram;
    native_chipram_size=chipram_size;
    ticks_per_sample_fp = (cntfrq() << 32) / (3546895u / PAULA_PERIOD);
    kprintf("[AGA] audio: real Paula DAC, %d Hz stereo, %d-sample loop in real chip RAM\n",
            3546895 / PAULA_PERIOD, RING_SAMPLES);
}

/* SOFTWARE swallows the game AUDx registers as before. NATIVE observes the
   same virtual-register stream and reproduces only its playback side on the
   real Paula. The virtual core remains authoritative for interrupts/timing. */
int aga_audio_reg_write(uint32_t reg, uint16_t v)
{
    if (reg == ADKCON && native_requested()) {
        native_adkcon_write=(uint16_t)((v & 0x8000u) | (v & 0x00ffu));
        __atomic_store_n(&native_adkcon_pending,1u,__ATOMIC_RELEASE);
        return 1;
    }

    if (reg == DMACON) {
        if (native_requested()) {
            uint16_t d=native_dmacon;
            uint16_t bits=v & 0x020fu;
            if (v & 0x8000u) d|=bits; else d&=(uint16_t)~bits;
            native_dmacon=d;
            for (unsigned c=0;c<4;c++) {
                int on=(d & 0x0200u) && (d & (1u<<c));
                native_aud_chan_t *n=&native_ch[c];
                int was=__atomic_load_n(&n->desired_on,__ATOMIC_ACQUIRE);
                if (on && !was) {
                    __atomic_store_n(&n->desired_on,1u,__ATOMIC_RELEASE);
                    if (n->staged_lc == n->lc && n->staged_len == n->len) {
                        if (!__atomic_load_n(&n->copy_pending,__ATOMIC_ACQUIRE))
                            n->prog_step=1u;
                    } else {
                        native_stage(c);
                    }
                } else if (!on && was) {
                    __atomic_store_n(&n->desired_on,0u,__ATOMIC_RELEASE);
                    __atomic_store_n(&n->stop_pending,1u,__ATOMIC_RELEASE);
                }
            }
        }
        return 1; /* audio DMA bits never hit the real chipset directly */
    }

    if (reg >= AUD0LCH && reg < AUD0LCH + 4u*16u) {
        unsigned c=(unsigned)((reg-AUD0LCH)>>4);
        unsigned off=(unsigned)((reg-AUD0LCH)&0x0fu);
        native_aud_chan_t *n=&native_ch[c];
        switch (off) {
        case 0x0: n->lc=(n->lc & 0x0000ffffu)|((uint32_t)v<<16); break;
        case 0x2: n->lc=(n->lc & 0xffff0000u)|v; break;
        case 0x4: n->len=v; break;
        case 0x6: n->per=v; __atomic_store_n(&n->per_pending,1u,__ATOMIC_RELEASE); break;
        case 0x8: n->vol=v; __atomic_store_n(&n->vol_pending,1u,__ATOMIC_RELEASE); break;
        case 0xA: n->dat=v; __atomic_store_n(&n->dat_pending,1u,__ATOMIC_RELEASE); break;
        default: break;
        }
        if (native_requested() && (off==0x0 || off==0x2 || off==0x4)) {
            if (__atomic_load_n(&n->desired_on,__ATOMIC_ACQUIRE)) {
                /* Unusual live source change: stage then restart this one
                   physical channel rather than overwrite bytes Paula reads. */
                native_stage(c);
            } else if (n->len) {
                /* Normal tracker/game sequence writes LC/LEN before DMACON.
                   Pre-stage while DMA is off so native playback can start at
                   the same ownership edge instead of several ms later. Bad
                   partial LC values are simply ignored until another write. */
                uint32_t _src,_bytes;
                if (native_channel_shape(c,&_src,&_bytes)) native_stage(c);
            }
        }
        return 1;
    }
    return 0;
}



static void program_paula(void)
{
    /* 0 = left high, 3 = left low, 1 = right high, 2 = right low.
       On Amiga hardware 0 and 3 feed LEFT, 1 and 2 feed RIGHT, so each pair is
       already summed on the right side of the output. Volume 64 against 1 makes
       the second channel the bottom six bits of a 14-bit sample. */
    ps_write_16(0xDFF096, 0x000F);                /* stop all four first */
    ps_write_16(0xDFF0A0, left_ring >> 16);  ps_write_16(0xDFF0A2, left_ring & 0xFFFF);
    ps_write_16(0xDFF0A4, RING_SAMPLES / 2); ps_write_16(0xDFF0A6, PAULA_PERIOD); ps_write_16(0xDFF0A8, 64);
    ps_write_16(0xDFF0B0, right_ring >> 16); ps_write_16(0xDFF0B2, right_ring & 0xFFFF);
    ps_write_16(0xDFF0B4, RING_SAMPLES / 2); ps_write_16(0xDFF0B6, PAULA_PERIOD); ps_write_16(0xDFF0B8, 64);
    ps_write_16(0xDFF0C0, rlo_ring >> 16);   ps_write_16(0xDFF0C2, rlo_ring & 0xFFFF);
    ps_write_16(0xDFF0C4, RING_SAMPLES / 2); ps_write_16(0xDFF0C6, PAULA_PERIOD); ps_write_16(0xDFF0C8, 1);
    ps_write_16(0xDFF0D0, llo_ring >> 16);   ps_write_16(0xDFF0D2, llo_ring & 0xFFFF);
    ps_write_16(0xDFF0D4, RING_SAMPLES / 2); ps_write_16(0xDFF0D6, PAULA_PERIOD); ps_write_16(0xDFF0D8, 1);
    ps_write_16(0xDFF09E, 0x00FF);                /* ADKCON: clear audio modulation bits */
    ps_write_16(0xDFF096, 0x820F);                /* DMAEN | AUD0..AUD3 */
    start_tick = cntvct();
    reinit_countdown = REINIT_INTERVAL;
}

/* Called from the chipset loop (core 3, outside the chipset lock) with the
   samples pulled from the core since the last call. */
void aga_audio_stream(const int16_t *lr, int frames)
{
    if (!aga_audio_software_needed()) return;
    /* During a native->software fallback let CPU2 silence native AUD DMA first. */
    if (__atomic_load_n(&native_stop_all_pending,__ATOMIC_ACQUIRE)) return;
    if (!lead_target) {
        lead_target = aga_pi3_mode() ? LEAD_PI3 : LEAD_PI4;
        /* says in the log which tolerances this board got, so a run on a
           kernel that was never loaded cannot pass for a test of it */
        aga_diag_log(lead_target == LEAD_PI3
            ? "[AGA] audio: Pi 3 - lead 1024 samples, loop catches up to two frames"
            : "[AGA] audio: lead 512 samples, loop catches up to one frame");
    }
    /* Sandbox mode: AmigaOS owns every byte of chip RAM, so without a ring
       agaboot allocated there is nowhere safe to put the samples. Silence is
       the correct failure - the alternative is writing 8 KB over whatever
       the running game was given, which is how this cost a day. In machine
       mode the real chip RAM is dead space and the fallback is fine. */
    if (!ring_owned && aga_sandbox_active) {
        static int moaned;
        if (!moaned) { moaned = 1; kprintf("[AGA] audio: no allocated ring in sandbox mode, staying silent\n"); }
        return;
    }
    if (!running) {
        /* start with a lead of silence so the DMA never overtakes the writer;
           the housekeeper clears the real rings and programs Paula */
        for (int i = 0; i < RING_SAMPLES; i++) { lbuf[i] = rbuf[i] = llob[i] = rlob[i] = 0; }
        wr_pos = LEAD_SAMPLES;
        start_tick = cntvct();              /* provisional; the housekeeper sets the real one */
        clear_pos = 0;
        svc_state = 1;
        __atomic_store_n(&pub_wr, wr_pos, __ATOMIC_RELEASE);
        __atomic_store_n(&restart_req, 1, __ATOMIC_RELEASE);
        __atomic_store_n(&running, 1, __ATOMIC_RELEASE);
    }

    /* Estimated DMA read position.
     *
     * NOT `(elapsed << 32) / ticks_per_sample_fp`: elapsed is a tick count, and
     * shifting it left by 32 overflows as soon as it passes 2^32 - about 165
     * seconds at the Pi's ~26 MHz. Multiplying by the sample rate first keeps
     * every intermediate inside 64 bits for hours. */
    uint64_t elapsed = cntvct() - start_tick;
    uint32_t rd_pos = (uint32_t)((elapsed * SAMPLE_HZ) / cntfrq()) % RING_SAMPLES;
    uint32_t lead = (wr_pos - rd_pos) & (RING_SAMPLES - 1);

    if (restart_req) {
        /* the housekeeper has not restarted the DMA yet: the model has no
           origin to regulate against, so just keep buffering */
    } else if (lead < RESYNC_LOW || lead > RESYNC_HIGH) {
        /* Genuinely lost: restart the loop. Audible, so it must be rare. */
        aga_diag_inc(AGA_D_AUDIO_RESYNCS);
        rd_pos = 0;
        wr_pos = LEAD_SAMPLES;
        svc_state = 2;
        __atomic_store_n(&restart_req, 1, __ATOMIC_RELEASE);
    } else if (lead > LEAD_SAMPLES + LEAD_DEADBAND) {
        /* Paula plays on the Amiga's crystal and I write on the Pi's, so the
         * two clocks drift apart for ever - that is not a fault to recover
         * from, it is the normal condition, and the only question is how it
         * gets absorbed.
         *
         * It used to be absorbed by program_paula(), which restarts the DMA
         * loop outright. That is a hard discontinuity in the middle of the
         * music, and the drift is far larger than the design assumed: 31
         * restarts in 110 seconds, one every three and a half seconds, where
         * the comment promises "an inaudible hiccup every few minutes". Beats
         * landing in the wrong place is exactly what that sounds like.
         *
         * Absorb it a single sample at a time instead. This runs about 437
         * times a second, so nudging by one sample per call corrects far more
         * drift than the clocks can produce, and one sample at 27.9 kHz is 36
         * microseconds - inaudible, where a loop restart is not. */
        wr_pos = (wr_pos - 1) & (RING_SAMPLES - 1);   /* running ahead: drop one */
        aga_diag_inc(AGA_D_AUD_NUDGE);
        aga_diag_inc(AGA_D_AUD_DROP);
    } else if (lead + LEAD_DEADBAND < LEAD_SAMPLES) {
        /* Running behind: repeat the last sample. The slot must be FILLED,
           not merely stepped over - advancing wr_pos alone leaves this slot
           holding whatever was written a full ring lap (73 ms) ago, and the
           packer below duly ships that stale byte to the DAC. A one-sample
           hold is inaudible; a 73 ms-old sample is a broadband click. */
        uint32_t prev = (wr_pos - 1) & (RING_SAMPLES - 1);
        lbuf[wr_pos] = lbuf[prev];  llob[wr_pos] = llob[prev];
        rbuf[wr_pos] = rbuf[prev];  rlob[wr_pos] = rlob[prev];
        wr_pos = (wr_pos + 1) & (RING_SAMPLES - 1);
        aga_diag_inc(AGA_D_AUD_NUDGE);
        aga_diag_inc(AGA_D_AUD_REPEAT);
    }

    aga_diag_inc(AGA_D_AUDIO_STREAMS);
    /* Convert to 8 bit, then push every COMPLETE four-sample group that has
       not been pushed yet - tracked by its own pointer.
       It used to flush on `(wr_pos & 3) == 0` alone, which assumes wr_pos
       only ever advances by one. The drift nudges above move it by +1 or -1,
       and that changes its ALIGNMENT: a group boundary can be stepped
       straight over, so those four samples never reach chip RAM and the DAC
       plays whatever the previous lap left there. Four stale samples is a
       click; land one on a short sound effect and the effect is gone. A
       separate flush pointer cannot skip a group however wr_pos moves. */
    for (int i = 0; i < frames; i++) {
        /* 16-bit mix -> a 14-bit value played as high byte (volume 64) plus
           low byte (volume 1), which Paula sums on each side.
           AUD_GAIN is applied here, where there is room for it: the measured
           peak of a whole gameplay run was 45 out of 127, barely a third of
           the range, so doubling cannot clip anything that was not already
           loud - and with 14 bits the gain costs no resolution, where at
           8 bits it would have cost a whole bit. */
        for (int ch = 0; ch < 2; ch++) {
            int32_t v14 = ((int32_t)lr[i * 2 + ch] * AUD_GAIN) >> 2;
            if (v14 >  8128) v14 =  8128;
            if (v14 < -8128) v14 = -8128;
            int32_t hi = v14 >> 6;                 /* arithmetic: -127..127 */
            int32_t lo = v14 - (hi << 6);          /* 0..63 */
            if (ch == 0) { lbuf[wr_pos] = (uint8_t)(int8_t)hi; llob[wr_pos] = (uint8_t)lo; }
            else         { rbuf[wr_pos] = (uint8_t)(int8_t)hi; rlob[wr_pos] = (uint8_t)lo; }
        }
        if (AGA_PROBES_ON) aga_diag_wave(lbuf[wr_pos], rbuf[wr_pos]);
        wr_pos = (wr_pos + 1) & (RING_SAMPLES - 1);
    }
    /* the bytes are in the local rings: tell the housekeeper how far */
    __atomic_store_n(&pub_wr, wr_pos, __ATOMIC_RELEASE);

    /* Refresh the real registers now and then, because a 68k reset also resets
       the real Paula. SET-ONLY: this must not restart the DMA.

       It used to call program_paula(), which writes DMACON=$0003 to stop the
       channels before writing $8203 to start them. That falling edge on
       AUD0EN/AUD1EN puts Paula's audio state machine back through its start
       state, which reloads the pointer from AUDxLC - the hardware read
       position snaps to ring offset 0. My own emulated Paula does the same on
       a 0->1 edge (aud_dmacon_changed), so the behaviour is not in doubt. The
       line after it then restored start_tick "to keep the phase", so the
       MODEL of the read position carried on from the old origin while the
       hardware's had been reset: every lead, every resync test and every
       nudge below was regulating a fiction from the first refresh onwards.

       At REINIT_INTERVAL that happened 8.66 times a second - a splice to an
       arbitrary point in the last 73 ms of ring history, continuously, under
       all the music and effects. It was never counted, unlike the 3.5-second
       drift restarts, which is why silencing those did not make the sound
       right. Drove me up the wall, that one.

       Writing $8203 when the bits are already set produces no edge and no
       restart, yet still re-enables the channels if a reset cleared them,
       which is the only reason this refresh exists. AUDxLC/AUDxLEN are
       latched at the loop wrap and AUDxPER/AUDxVOL take effect live, so
       rewriting them mid-stream is harmless. */
    if (--reinit_countdown <= 0) {
        __atomic_store_n(&reinit_req, 1, __ATOMIC_RELEASE);
        reinit_countdown = REINIT_INTERVAL;
    }
}

/* The housekeeper core's share, called from its loop on every visit (about a
   million times a second): at most one group of four samples per visit, so
   the IPL sampling it exists for never waits behind more than four bus
   writes. A fresh start clears the four real rings sixteen words per visit,
   then programs Paula and stamps the moment the DMA really began. */
extern void aga_hvs_service(void);      /* aga_hvs.c: the post-hand-back HVS watch */

/* One physical-bus operation per housekeeper visit. The housekeeper runs far
   faster than Paula sample events, so even a 20K instrument stages quickly
   without ever making CPU3 wait on PiStorm bus latency. */
static int native_bus_service(void)
{
    if (__atomic_load_n(&native_stop_all_pending,__ATOMIC_ACQUIRE)) {
        ps_write_16(0xDFF096,0x000f); /* audio bits only; do not touch master/video DMA */
        for (unsigned c=0;c<4;c++) { native_ch[c].hw_on=0; native_ch[c].prog_step=0; }
        native_init_step=0;
        __atomic_store_n(&native_stop_all_pending,0u,__ATOMIC_RELEASE);
        return 1;
    }
    if (!native_requested()) return 0;

    /* Real AUD interrupts must never enter the 68k interrupt path; the virtual
       Paula owns those semantics. Do this lazily on CPU2, not from a trap. */
    if (native_init_step < 3u) {
        if (native_init_step==0u) ps_write_16(0xDFF09A,0x0780);      /* clear AUD INTENA */
        else if (native_init_step==1u) ps_write_16(0xDFF09C,0x0780); /* clear pending AUD INTREQ */
        else ps_write_16(0xDFF096,0x000f);                           /* stop stale real audio */
        native_init_step++;
        return 1;
    }

    if (__atomic_exchange_n(&native_adkcon_pending,0u,__ATOMIC_ACQ_REL)) {
        ps_write_16(0xDFF09E,native_adkcon_write);
        return 1;
    }

    for (unsigned k=0;k<4;k++) {
        unsigned c=(native_rr+k)&3u;
        native_aud_chan_t *n=&native_ch[c];
        uint32_t bit=1u<<c;

        if (__atomic_exchange_n(&n->stop_pending,0u,__ATOMIC_ACQ_REL)) {
            ps_write_16(0xDFF096,(uint16_t)bit);
            n->hw_on=0; n->prog_step=0;
            native_rr=(c+1u)&3u;
            return 1;
        }

        if (__atomic_load_n(&n->copy_pending,__ATOMIC_ACQUIRE)) {
            uint32_t gen=__atomic_load_n(&n->req_gen,__ATOMIC_ACQUIRE);
            if (n->svc_gen!=gen) { n->svc_gen=gen; n->copy_pos=0; }
            uint32_t src,bytes;
            if (!native_channel_shape(c,&src,&bytes)) {
                native_force_fallback("sample changed outside native staging limits");
                return 1;
            }
            if (n->copy_pos < bytes) {
                uint32_t p=n->copy_pos;
                uint32_t left=bytes-p;
                if (left >= 4u) {
                    const uint8_t *q=native_chipram+src+p;
                    uint32_t w=((uint32_t)q[0]<<24)|((uint32_t)q[1]<<16)|((uint32_t)q[2]<<8)|q[3];
                    ps_write_32(n->phys+p,w);
                    n->copy_pos=p+4u;
                } else {
                    const uint8_t *q=native_chipram+src+p;
                    ps_write_16(n->phys+p,(uint16_t)(((uint16_t)q[0]<<8)|q[1]));
                    n->copy_pos=p+2u;
                }
                native_rr=(c+1u)&3u;
                return 1;
            }
            if (__atomic_load_n(&n->req_gen,__ATOMIC_ACQUIRE)!=n->svc_gen) {
                n->copy_pos=0;
                continue;
            }
            __atomic_store_n(&n->copy_pending,0u,__ATOMIC_RELEASE);
            n->prog_step=1u;
        }

        if (n->prog_step) {
            uint32_t r=0xDFF0A0u+c*0x10u;
            switch (n->prog_step++) {
            case 1: ps_write_16(0xDFF096,(uint16_t)bit); n->hw_on=0; break;
            case 2: ps_write_16(r+0,(uint16_t)(n->phys>>16)); break;
            case 3: ps_write_16(r+2,(uint16_t)n->phys); break;
            case 4: ps_write_16(r+4,n->len); break;
            case 5: ps_write_16(r+6,n->per); n->per_pending=0; break;
            case 6: ps_write_16(r+8,n->vol); n->vol_pending=0; break;
            case 7: ps_write_16(0xDFF09C,(uint16_t)(0x0080u<<c)); break;
            default:
                n->prog_step=0;
                if (__atomic_load_n(&n->desired_on,__ATOMIC_ACQUIRE)) {
                    ps_write_16(0xDFF096,(uint16_t)(0x8200u|bit));
                    n->hw_on=1;
                }
                break;
            }
            native_rr=(c+1u)&3u;
            return 1;
        }

        if (n->hw_on && __atomic_exchange_n(&n->per_pending,0u,__ATOMIC_ACQ_REL)) {
            ps_write_16(0xDFF0A6u+c*0x10u,n->per); native_rr=(c+1u)&3u; return 1;
        }
        if (n->hw_on && __atomic_exchange_n(&n->vol_pending,0u,__ATOMIC_ACQ_REL)) {
            ps_write_16(0xDFF0A8u+c*0x10u,n->vol); native_rr=(c+1u)&3u; return 1;
        }
        if (__atomic_exchange_n(&n->dat_pending,0u,__ATOMIC_ACQ_REL)) {
            ps_write_16(0xDFF0AAu+c*0x10u,n->dat); native_rr=(c+1u)&3u; return 1;
        }
    }
    return 1; /* native owns audio even when there was no bus work this visit */
}

void aga_audio_bus_service(void)
{
    aga_hvs_service();                  /* cheap: one timer read unless a watch is armed */
    if (native_bus_service()) return;
    if (!__atomic_load_n(&running, __ATOMIC_ACQUIRE)) return;
    svc_busy = 1;
    __asm__ volatile("dmb ish" ::: "memory");
    if (!running) { svc_busy = 0; return; }
    if (__atomic_load_n(&restart_req, __ATOMIC_ACQUIRE)) {
        if (svc_state == 1) {
            for (int i = 0; i < 4 && clear_pos < RING_SAMPLES; i++, clear_pos += 4) {
                ps_write_32(left_ring  + clear_pos, 0);
                ps_write_32(right_ring + clear_pos, 0);
                ps_write_32(llo_ring   + clear_pos, 0);
                ps_write_32(rlo_ring   + clear_pos, 0);
            }
            if (clear_pos >= RING_SAMPLES) svc_state = 2;
        }
        if (svc_state == 2) {
            program_paula();
            flush_pos = LEAD_SAMPLES & ~3u;
            start_tick = cntvct();
            svc_state = 3;
            __atomic_store_n(&restart_req, 0, __ATOMIC_RELEASE);
        }
        svc_busy = 0;
        return;
    }
    if (__atomic_load_n(&reinit_req, __ATOMIC_ACQUIRE)) {
        /* SET-ONLY, no DMACON clear: a falling edge on AUDxEN restarts the
           channel from AUDxLC and the read-position model would be regulating
           a fiction from then on (see the history in git for the full story) */
        ps_write_16(0xDFF0A0, left_ring >> 16);  ps_write_16(0xDFF0A2, left_ring & 0xFFFF);
        ps_write_16(0xDFF0A4, RING_SAMPLES / 2); ps_write_16(0xDFF0A6, PAULA_PERIOD); ps_write_16(0xDFF0A8, 64);
        ps_write_16(0xDFF0B0, right_ring >> 16); ps_write_16(0xDFF0B2, right_ring & 0xFFFF);
        ps_write_16(0xDFF0B4, RING_SAMPLES / 2); ps_write_16(0xDFF0B6, PAULA_PERIOD); ps_write_16(0xDFF0B8, 64);
        ps_write_16(0xDFF096, 0x8203);
        __atomic_store_n(&reinit_req, 0, __ATOMIC_RELEASE);
        svc_busy = 0;
        return;
    }
    /* Distance forward from flush_pos, modulo the ring. A -1 nudge on core 3
       can put the target BEHIND me; half a ring of "distance" means exactly
       that, and the answer is to give up the gap, not to walk the whole ring
       writing stale samples. */
    uint32_t target = __atomic_load_n(&pub_wr, __ATOMIC_ACQUIRE) & ~3u;
    uint32_t dist = (target - flush_pos) & (RING_SAMPLES - 1);
    if (dist > RING_SAMPLES / 2) { flush_pos = target; dist = 0; }
    if (dist >= 4) {
        uint32_t p = flush_pos;
        uint32_t lw  = ((uint32_t)lbuf[p] << 24) | ((uint32_t)lbuf[p + 1] << 16) | ((uint32_t)lbuf[p + 2] << 8) | lbuf[p + 3];
        uint32_t rw  = ((uint32_t)rbuf[p] << 24) | ((uint32_t)rbuf[p + 1] << 16) | ((uint32_t)rbuf[p + 2] << 8) | rbuf[p + 3];
        uint32_t llw = ((uint32_t)llob[p] << 24) | ((uint32_t)llob[p + 1] << 16) | ((uint32_t)llob[p + 2] << 8) | llob[p + 3];
        uint32_t rlw = ((uint32_t)rlob[p] << 24) | ((uint32_t)rlob[p + 1] << 16) | ((uint32_t)rlob[p + 2] << 8) | rlob[p + 3];
        ps_write_32(left_ring  + p, lw);
        ps_write_32(right_ring + p, rw);
        ps_write_32(llo_ring   + p, llw);
        ps_write_32(rlo_ring   + p, rlw);
        flush_pos = (flush_pos + 4) & (RING_SAMPLES - 1);
    }
    svc_busy = 0;
}

int16_t *aga_audio_scratch(int *max_frames)
{
    *max_frames = (int)(sizeof pcm / sizeof pcm[0]) / 2;
    return pcm;
}

/* Sandbox leave: stop the DAC loops and forget the stream state. */
void aga_audio_stop(void)
{
    int was = running;
    __atomic_store_n(&running, 0, __ATOMIC_RELEASE);
    while (svc_busy) __asm__ volatile("yield");     /* let the housekeeper finish its group */
    __atomic_store_n(&restart_req, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&reinit_req, 0, __ATOMIC_RELEASE);
    /* FRF_AGA_NATIVE_PAULA_V0_1: native mode never sets `running`, but it owns
       the same real AUD0..3 bits. Silence those before agaboot frees the pool. */
    if (was || audio_engine_mode != AENG_SOFTWARE || native_init_step)
        ps_write_16(0xDFF096, 0x000F);
    have = 0;
    native_reset_channels();
    ring_release();
}
