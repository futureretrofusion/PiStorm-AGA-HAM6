/* AGA-PISTORM — Paula audio.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1
 *
 * Hardware-facing Paula state is aligned with the mature E-UAE channel state
 * machine while keeping the FRF stereo ring / physical-output transport.
 *
 * The important distinction is intentional:
 *   - E-UAE contributes Paula register/DMA/state semantics.
 *   - FRF keeps its existing PCM ring and CPU3/physical-Paula transport.
 *
 * This is not a copy of E-UAE's host mixer, sinc renderer or AHI backend.
 */
#include "internal.h"

static const char paula_alignment_marker[] __attribute__((used)) =
    "FRF_AGA_PAULA_EUAE_ALIGNMENT_V0_1";

#define AUD_RING 32768      /* stereo frames */
static int16_t ring[AUD_RING * 2];
static uint32_t ring_wr, ring_rd;

/* Box-filter accumulators.  The core now feeds them one Paula colour clock at
 * a time, so DMA slots and period edges share the same time base. */
static int64_t acc_l, acc_r;
static int32_t acc_cck;          /* accumulated colour clocks, fixed 16.16 */
static int32_t cck_per_sample;   /* 16.16 */

static inline uint16_t paula_period(const audio_channel_t *c)
{
    /* Paula period zero is not a zero-time oscillator.  E-UAE models it as a
       very long/special period.  0xffff is the representable equivalent in
       this compact core and, unlike the old code, never creates a 1-cck loop. */
    return c->per ? c->per : 0xffffu;
}

static inline uint16_t paula_volume(uint16_t v)
{
    /* AUDxVOL is a six-bit DAC gain with bit 6 selecting maximum volume.
       E-UAE: v & 64 ? 63 : v & 63. */
    return (v & 0x0040u) ? 63u : (v & 0x003fu);
}

static inline int paula_irq_pending(const aga_t *a, int n)
{
    return (a->intreq & (uint16_t)(INTF_AUD0 << n)) != 0;
}

static void paula_setirq(aga_t *a, int n)
{
    a->aud_irq[n]++;
    aga_intreq_set(a, (uint16_t)(INTF_AUD0 << n));
}

static inline int paula_audav(const aga_t *a, int n)
{
    return (a->adkcon & (uint16_t)(0x0001u << n)) != 0;
}

static inline int paula_audap(const aga_t *a, int n)
{
    return (a->adkcon & (uint16_t)(0x0010u << n)) != 0;
}

static inline int paula_modulator(const aga_t *a, int n)
{
    return paula_audav(a, n) || paula_audap(a, n);
}

static void record_wrap(aga_t *a, int n, audio_channel_t *c)
{
    a->aud_wrap[n]++;

    /* Preserve the existing bounded forensic sampler. */
    if ((a->aud_wrap[n] & 511u) == 0) {
        uint32_t *w = a->aud_wrapinfo[a->aud_wrapinfo_head & 31u];
        uint32_t words = c->len < 64 ? c->len : 64;
        int peak = 0;
        for (uint32_t k = 0; k < words; k++) {
            uint16_t wv = chip_rd16(a, c->lc + k * 2u);
            int hi = (int8_t)(wv >> 8), lo = (int8_t)(wv & 0xff);
            if (hi < 0) hi = -hi;
            if (lo < 0) lo = -lo;
            if (hi > peak) peak = hi;
            if (lo > peak) peak = lo;
        }
        w[0] = (uint32_t)n;
        w[1] = c->lc;
        w[2] = c->len;
        w[3] = (uint32_t)peak;
        a->aud_wrapinfo_head++;
    }
}

/* E-UAE state23(): arm the next DMA word and handle LEN reload exactly at the
 * word boundary.  request_word >= 0 means a request is already in flight. */
static void paula_state23(aga_t *a, int n, audio_channel_t *c)
{
    if (!c->dmaen)
        return;
    if (c->request_word >= 0)
        return;

    c->request_word = 0;
    if (c->lencnt == 1) {
        c->lencnt = c->len;
        c->pt = c->lc;
        c->intreq2 = 1;
        record_wrap(a, n, c);
    } else {
        c->lencnt = (uint16_t)(c->lencnt - 1u);
    }
}

static void paula_newsample(audio_channel_t *c, int8_t sample)
{
    c->last_sample = c->cur_sample;
    c->cur_sample = sample;
}

/* Port of the hardware-significant part of E-UAE audio_handler().  Host sound
 * scheduling has deliberately been removed: percnt is our colour-clock event
 * timer and the FRF ring remains the output backend. */
static void paula_handler(aga_t *a, int n, int timed)
{
    audio_channel_t *c = &a->aud[n];
    const int audav = paula_audav(a, n);
    const int audap = paula_audap(a, n);
    const int napnav = (!audav && !audap) || audav;
    const uint16_t oldev = c->percnt;
    (void)timed;

    switch (c->state) {
    case 0:
        c->request_word = 0;
        c->request_word_skip = 0;
        c->intreq2 = 0;
        if (c->dmaen) {
            c->state = 1;
            c->lencnt = c->len;
            c->pt = c->lc;
            paula_handler(a, n, timed);
        }
        return;

    case 1:
        if (!c->dmaen) {
            c->state = 0;
            return;
        }
        c->state = 5;
        if (c->lencnt != 1)
            c->lencnt = (uint16_t)(c->lencnt - 1u);
        c->request_word = 2;
        /* Match E-UAE's end-of-line startup guard. */
        if (a->hpos > a->maxhpos - 20)
            c->request_word_skip = 1;
        return;

    case 5:
        if (!c->request_word) {
            c->request_word = 2;
            return;
        }
        paula_setirq(a, n);               /* first DMA word accepted */
        if (!c->dmaen) {
            c->state = 0;
            c->request_word = 0;
            return;
        }
        c->state = 2;
        c->request_word = napnav ? 2 : 3;
        c->dat = c->dat2;
        return;

    case 2:
        /* CPU-driven AUDxDAT stops after its word once its IRQ is pending. */
        if (!c->dmaen && paula_irq_pending(a, n) &&
            (c->per <= 30 || oldev == 0 || oldev == paula_period(c))) {
            c->state = 0;
            c->request_word = 0;
            c->cur_sample = 0;
            c->percnt = 0;
            return;
        }

        paula_state23(a, n, c);
        c->state = 3;
        c->percnt = paula_period(c);
        paula_newsample(c, (int8_t)(c->dat >> 8));
        c->dat <<= 8;

        /* Period attachment: channel N modulates N+1 and is not mixed. */
        if (audap) {
            if (c->intreq2 && c->dmaen)
                paula_setirq(a, n);
            c->intreq2 = 0;
            c->request_word = 1;
            c->dat = c->dat2;
            if (n < 3)
                a->aud[n + 1].per = c->dat;
        }
        return;

    case 3:
        paula_state23(a, n, c);
        c->state = 2;
        c->percnt = paula_period(c);
        paula_newsample(c, (int8_t)(c->dat >> 8));
        c->dat <<= 8;
        c->dat = c->dat2;

        if (c->dmaen) {
            if (napnav)
                c->request_word = 1;
            if (c->intreq2 && napnav)
                paula_setirq(a, n);
        } else {
            if (napnav)
                paula_setirq(a, n);
        }
        c->intreq2 = 0;

        /* Volume attachment.  Decode using the same six-bit/bit6 rule as a
           real AUDxVOL write so a modulation word cannot overdrive our mixer. */
        if (audav && n < 3)
            a->aud[n + 1].vol = paula_volume(c->dat);
        return;

    default:
        c->state = 0;
        c->request_word = 0;
        c->cur_sample = 0;
        c->percnt = 0;
        return;
    }
}

void aud_reset(aga_t *a)
{
    memset(a->aud, 0, sizeof a->aud);
    for (int n = 0; n < 4; n++) {
        a->aud[n].request_word = 0;
        a->aud[n].per = 0;
        a->aud[n].percnt = 0;
    }
    a->audio_rate = 48000;
    a->aud_mask = 15;
    ring_wr = ring_rd = 0;
    acc_l = acc_r = 0;
    acc_cck = 0;
    {
        double cck_hz = a->ntsc ? 3579545.0 : 3546895.0;
        cck_per_sample = (int32_t)(cck_hz * 65536.0 / 48000.0);
    }
}

static void set_rate(aga_t *a, int rate)
{
    if (rate <= 0) rate = 48000;
    if (a->audio_rate != rate) {
        a->audio_rate = rate;
        double cck_hz = a->ntsc ? 3579545.0 : 3546895.0;
        cck_per_sample = (int32_t)(cck_hz * 65536.0 / rate);
    }
}

void aud_write(aga_t *a, uint32_t reg, uint16_t v)
{
    int n = (int)((reg - AUD0LCH) >> 4);
    if (n < 0 || n > 3)
        return;
    audio_channel_t *c = &a->aud[n];

    /* Preserve the existing FRF external observation hook. */
    if (a->hooks.ext_write)
        a->hooks.ext_write(a->hooks.user, reg, v);

    switch (reg & 0x0e) {
    case 0x0:
        c->lc = (c->lc & 0x0000ffffu) | ((uint32_t)(v & 0x01ffu) << 16);
        break;
    case 0x2:
        c->lc = (c->lc & 0xffff0000u) | (uint32_t)(v & 0xfffeu);
        break;
    case 0x4:
        c->len = v;
        break;
    case 0x6:
        c->per = v;
        /* If a CPU-driven channel was waiting on period zero, changing PER is
           immediately meaningful on Paula. */
        if ((c->state == 2 || c->state == 3) && c->percnt == 0)
            c->percnt = paula_period(c);
        break;
    case 0x8:
        c->vol = paula_volume(v);
        if ((uint32_t)c->vol > a->aud_volmax[n]) a->aud_volmax[n] = (uint32_t)c->vol;
        a->aud_volwr[n]++;
        if (c->vol) a->aud_volnz[n]++;
        break;
    case 0xa:
        /* E-UAE keeps CPU data in the secondary latch.  Do not assert the IRQ
           here: manual mode raises it after the low byte has been presented. */
        c->dat2 = v;
        c->dat_written = 1;
        c->request_word = -1;
        c->request_word_skip = 0;
        if (c->state == 0) {
            c->state = 2;
            c->dat = c->dat2;
            c->percnt = paula_period(c);
            paula_handler(a, n, 0);
        }
        break;
    default:
        break;
    }
}

void aud_dmacon_changed(aga_t *a, uint16_t old, uint16_t new_)
{
    for (int n = 0; n < 4; n++) {
        audio_channel_t *c = &a->aud[n];
        int was = (old & DMAF_DMAEN) && (old & (DMAF_AUD0EN << n));
        int now = (new_ & DMAF_DMAEN) && (new_ & (DMAF_AUD0EN << n));

        if (!was && now) {
            /* Preserve the existing trigger forensic record. */
            uint32_t *t = a->aud_trig[a->aud_trig_head & 31u];
            t[0] = (uint32_t)n;
            t[1] = c->lc;
            t[2] = c->len;
            t[3] = c->per;
            t[4] = (uint32_t)c->vol;
            {
                uint32_t words = c->len < 256 ? c->len : 256;
                int peak = 0;
                for (uint32_t k = 0; k < words; k++) {
                    uint16_t wv = chip_rd16(a, c->lc + k * 2u);
                    int hi = (int8_t)(wv >> 8), lo = (int8_t)(wv & 0xff);
                    if (hi < 0) hi = -hi;
                    if (lo < 0) lo = -lo;
                    if (hi > peak) peak = hi;
                    if (lo > peak) peak = lo;
                }
                t[5] = (uint32_t)peak;
            }
            a->aud_trig_head++;
            a->aud_on[n]++;

            c->dmaen = 1;
            c->pt = c->lc;
            c->lencnt = c->len;
            c->state = 0;
            c->request_word = 0;
            c->request_word_skip = 0;
            c->intreq2 = 0;
            c->dat_written = 0;
            c->cur_sample = 0;
            c->last_sample = 0;
            c->percnt = 0;
            paula_handler(a, n, 0);       /* state 0 -> 1 -> 5, request first word */
        } else if (was && !now) {
            a->aud_dmaoff[n]++;
            c->dmaen = 0;
            /* E-UAE does not brutally zero the DAC on a DMA clear.  The word
               already in Paula is allowed to finish and the state machine
               decides when the channel becomes idle. */
        }
    }
}

/* Real Paula gives each voice one fixed DMA word slot per scanline.
 *  ch0 $0d, ch1 $0f, ch2 $11, ch3 $13.
 *
 * The old FRF core fetched on demand from chan_run(), which allowed multiple
 * words to be consumed in one line and bypassed the two-word Paula pipeline.
 */
void aud_dma_slot(aga_t *a, int hpos)
{
    if (hpos < 0x0d || hpos > 0x13 || !(hpos & 1))
        return;

    int n = (hpos - 0x0d) >> 1;
    audio_channel_t *c = &a->aud[n];
    if (!c->dmaen || c->request_word <= 0)
        return;

    if (c->request_word_skip) {
        c->request_word_skip = 0;
        return;
    }

    if (c->state == 5)
        c->pt = c->lc;

    uint32_t fetch_addr = c->pt;
    c->dat2 = chip_rd16(a, fetch_addr);
    a->aud_words[n]++;
    aga_trace_at(a, hpos, AGA_DMA_AUDIO,
                 (uint16_t)(AUD0DAT + (uint32_t)n * 0x10u), fetch_addr);

    int request = c->request_word;
    if (request == 1 || request == 2)
        c->pt += 2;
    c->request_word = -1;

    /* Requests 2/3 are pipeline/state transitions in E-UAE; request 1 is a
       plain refill and does not immediately run the playback state machine. */
    if (request >= 2)
        paula_handler(a, n, 0);
}

/* Emit one output sample when enough Paula colour clocks have accumulated. */
static inline void paula_mix_cck(aga_t *a)
{
    int32_t l = 0, r = 0;

    for (int n = 0; n < 4; n++) {
        audio_channel_t *c = &a->aud[n];
        int vol = paula_volume(c->vol);
        int32_t s = (int32_t)c->cur_sample * vol;

        a->aud_nz[n] += (uint32_t)(s < 0 ? -s : s) >> 12;
        if (c->cur_sample && vol && !paula_modulator(a, n))
            a->aud_audible[n]++;

        /* ADKCON attachment channels feed N+1 instead of the DAC. */
        if (!(a->aud_mask & (1 << n)) || paula_modulator(a, n))
            continue;

        if (n == 0 || n == 3) l += s;
        else                  r += s;
    }

    acc_l += (int64_t)l << 16;
    acc_r += (int64_t)r << 16;
    acc_cck += 1 << 16;

    if (acc_cck >= cck_per_sample) {
        int64_t sl = acc_l / cck_per_sample;
        int64_t sr = acc_r / cck_per_sample;
        sl *= 2;
        sr *= 2;
        if (sl > 32767) sl = 32767;
        if (sl < -32768) sl = -32768;
        if (sr > 32767) sr = 32767;
        if (sr < -32768) sr = -32768;

        uint32_t idx = ring_wr & (AUD_RING - 1u);
        ring[idx * 2u] = (int16_t)sl;
        ring[idx * 2u + 1u] = (int16_t)sr;
        ring_wr++;

        /* Keep the fractional timing remainder exactly as the previous FRF
           resampler did.  Audio energy is reset at an output-sample boundary. */
        acc_l = acc_r = 0;
        acc_cck -= cck_per_sample;
    }
}

/* One Paula colour clock.  custom.c invokes this after the DMA-slot action for
 * the current hpos, which means a fetched word is visible to the state machine
 * on the same hardware clock. */
void aud_cck_tick(aga_t *a)
{
    paula_mix_cck(a);

    for (int n = 0; n < 4; n++) {
        audio_channel_t *c = &a->aud[n];
        if (c->state != 2 && c->state != 3)
            continue;

        if (c->percnt > 1) {
            c->percnt--;
            continue;
        }

        c->percnt = 0;
        paula_handler(a, n, 1);
    }
}

/* Kept for source/API compatibility.  Audio timing is now advanced in
 * aud_cck_tick(), not reconstructed after the scanline has already happened. */
void aud_line_tick(aga_t *a)
{
    (void)a;
}

int aga_audio_pull(aga_t *a, int16_t *lr, int frames, int rate)
{
    set_rate(a, rate);
    int n = 0;
    while (n < frames && ring_rd != ring_wr) {
        uint32_t idx = ring_rd & (AUD_RING - 1u);
        lr[n * 2] = ring[idx * 2u];
        lr[n * 2 + 1] = ring[idx * 2u + 1u];
        ring_rd++;
        n++;
    }
    return n;
}

void aga_audio_set_mask(aga_t *a, int mask) { a->aud_mask = mask & 15; }

void aga_audio_triggers(const aga_t *a, uint32_t *out, int *count)
{
    uint32_t n = a->aud_trig_head < 32 ? a->aud_trig_head : 32;
    uint32_t start = a->aud_trig_head < 32 ? 0 : a->aud_trig_head;
    for (uint32_t i = 0; i < n; i++)
        for (int k = 0; k < 6; k++)
            out[i * 6 + (uint32_t)k] = a->aud_trig[(start + i) & 31u][k];
    *count = (int)n;
}

void aga_audio_vols(const aga_t *a, uint32_t *vol, uint32_t *volmax, uint32_t *audible)
{
    for (int n = 0; n < 4; n++) {
        vol[n] = (uint32_t)a->aud[n].vol;
        volmax[n] = a->aud_volmax[n];
        audible[n] = a->aud_audible[n];
    }
}

void aga_audio_volstats(const aga_t *a, uint32_t *wr, uint32_t *nz, uint32_t *dmaoff)
{
    for (int n = 0; n < 4; n++) {
        wr[n] = a->aud_volwr[n];
        nz[n] = a->aud_volnz[n];
        dmaoff[n] = a->aud_dmaoff[n];
    }
}

void aga_audio_wrapstats(const aga_t *a, uint32_t *wrap, uint32_t *irq)
{
    for (int n = 0; n < 4; n++) {
        wrap[n] = a->aud_wrap[n];
        irq[n] = a->aud_irq[n];
    }
}

void aga_audio_wrapinfo(const aga_t *a, uint32_t *out, int *count)
{
    uint32_t n = a->aud_wrapinfo_head < 32 ? a->aud_wrapinfo_head : 32;
    uint32_t start = a->aud_wrapinfo_head < 32 ? 0 : a->aud_wrapinfo_head;
    for (uint32_t i = 0; i < n; i++)
        for (int k = 0; k < 4; k++)
            out[i * 4 + (uint32_t)k] = a->aud_wrapinfo[(start + i) & 31u][k];
    *count = (int)n;
}

void aga_audio_stats(const aga_t *a, uint32_t *on, uint32_t *words, uint32_t *nz)
{
    for (int n = 0; n < 4; n++) {
        on[n] = a->aud_on[n];
        words[n] = a->aud_words[n];
        nz[n] = a->aud_nz[n];
    }
}
