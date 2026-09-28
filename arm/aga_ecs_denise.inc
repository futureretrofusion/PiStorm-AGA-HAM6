/* FRF_AGA_ECS_DENISE_BRIDGE_V0_1
 *
 * ECS/OCS sandbox -> real Denise/Agnus presentation backend.
 *
 * The game continues to run against the virtual chipset and virtual Chip RAM.
 * CPU3 snapshots the completed software-chipset RGB frame plus the guest's
 * ECS display registers. CPU1 quantises/encodes the visible image back into a
 * small real-Chip planar bank and presents that bank through the physical
 * chipset in the guest's lores/hires/LACE mode. The physical Copper is held
 * off while this presenter owns scanout and restored on teardown.
 *
 * This deliberately does NOT alias virtual bitplane addresses into physical
 * DMA. That would let physical Agnus fetch non-Chip memory and is invalid.
 * Only the small presentation bank is real Chip RAM.
 */

#define FRF_ECS_DENISE_MAGIC       0x45435344u /* ECSD */
#define FRF_ECS_DENISE_VERSION     1u
#define FRF_ECS_DENISE_PLANES      6u
#define FRF_ECS_DENISE_MAX_W       640u
#define FRF_ECS_DENISE_MAX_H       286u
#define FRF_ECS_DENISE_ROW_BYTES   (FRF_ECS_DENISE_MAX_W / 8u) /* 80 */
#define FRF_ECS_DENISE_MAX_WORDS   (FRF_ECS_DENISE_MAX_W / 16u)
#define FRF_ECS_CPU1_LINES_PER_SERVICE 8u

#define FRF_ECS_BPLCON0_HIRES 0x8000u
#define FRF_ECS_BPLCON0_DEPTH 0x7000u
#define FRF_ECS_BPLCON0_HAM   0x0800u
#define FRF_ECS_BPLCON0_LACE  0x0004u
#define FRF_ECS_BPLCON0_SHRES 0x0040u
#define FRF_ECS_BPLCON0_COLOR 0x0200u

/* Reuse the established 14-long physical descriptor transport. The magic and
 * version distinguish this record from HAM6 V9/V10; the existing agaboot ->
 * firmware pointer handoff therefore needs no new MMU/control opcode. */
enum {
    ED_MAGIC = 0,
    ED_WIDTH,
    ED_HEIGHT,
    ED_BYTESPERROW,
    ED_ROWSTRIDE,
    ED_FLAGS,
    ED_PLANE0,
    ED_PLANE1,
    ED_PLANE2,
    ED_PLANE3,
    ED_PLANE4,
    ED_PLANE5,
    ED_RESTORE_COPPER,
    ED_VERSION,
    ED_LONGS
};

enum {
    FRF_ECS_SLOT_FREE = 0u,
    FRF_ECS_SLOT_WRITING = 1u,
    FRF_ECS_SLOT_PENDING = 2u,
    FRF_ECS_SLOT_BUSY = 3u
};

typedef struct frf_ecs_meta {
    uint16_t bplcon0;
    uint16_t diwstrt, diwstop, ddfstrt, ddfstop;
    uint16_t color[32];
    uint16_t depth;
    uint16_t hires;
    uint16_t lace;
    uint16_t ham;
    uint16_t width;
    uint16_t height;
    uint16_t words;
    uint16_t src_x;
    uint16_t src_y;
} frf_ecs_meta_t;

static volatile uint32_t g_ecs_active;
static volatile uint32_t g_ecs_stop_requested;
static uint32_t g_ecs_plane[FRF_ECS_DENISE_PLANES];
static uint32_t g_ecs_restore_copper;
static uint32_t g_ecs_row_stride = FRF_ECS_DENISE_ROW_BYTES;
static volatile uint32_t g_ecs_slot_state[2];
static volatile uint32_t g_ecs_slot_seq[2];
static frf_ecs_meta_t g_ecs_meta[2];
static volatile int32_t g_ecs_worker_slot = -1;
static frf_ecs_meta_t g_ecs_worker_meta;
static uint32_t g_ecs_worker_seq;
static unsigned g_ecs_worker_y;
static volatile uint32_t g_ecs_submit_seq;
static uint32_t g_ecs_input_frames, g_ecs_submitted, g_ecs_completed, g_ecs_dropped;
static uint32_t g_ecs_chip_bytes, g_ecs_runs, g_ecs_frames;
static uint8_t g_ecs_force_full;
static uint8_t g_ecs_enable_pending;
static uint8_t g_ecs_mode_programmed;
static uint8_t g_ecs_fps = 25u;
static uint16_t g_ecs_shadow[FRF_ECS_DENISE_PLANES][FRF_ECS_DENISE_MAX_H][FRF_ECS_DENISE_MAX_WORDS];
static uint8_t g_ecs_lut[4096];
static uint16_t g_ecs_lut_palette[32];
static uint8_t g_ecs_lut_depth;
static uint8_t g_ecs_lut_ehb;
static uint8_t g_ecs_lut_valid;

static inline uint32_t frf_ecs_load_u32(volatile uint32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}
static inline void frf_ecs_store_u32(volatile uint32_t *p, uint32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}
static inline int32_t frf_ecs_load_i32(volatile int32_t *p)
{
    return __atomic_load_n(p, __ATOMIC_ACQUIRE);
}
static inline void frf_ecs_store_i32(volatile int32_t *p, int32_t v)
{
    __atomic_store_n(p, v, __ATOMIC_RELEASE);
}

static int ecs_denise_descriptor_selected(void)
{
    uint32_t a = frf_load_u32(&g_desc_addr);
    if (!a || a >= FRF_AGA_CHIP_LIMIT || (a & 3u)) return 0;
    return ps_read_32(a + ED_MAGIC * 4u) == FRF_ECS_DENISE_MAGIC;
}

static int ecs_denise_is_active(void)
{
    return frf_ecs_load_u32(&g_ecs_active) != 0u;
}

static void ecs_slots_reset(void)
{
    frf_ecs_store_u32(&g_ecs_slot_state[0], FRF_ECS_SLOT_FREE);
    frf_ecs_store_u32(&g_ecs_slot_state[1], FRF_ECS_SLOT_FREE);
    frf_ecs_store_u32(&g_ecs_slot_seq[0], 0u);
    frf_ecs_store_u32(&g_ecs_slot_seq[1], 0u);
    frf_ecs_store_i32(&g_ecs_worker_slot, -1);
    g_ecs_worker_seq = 0u;
    g_ecs_worker_y = 0u;
}

static int ecs_descriptor_read(void)
{
    uint32_t a = frf_load_u32(&g_desc_addr);
    if (!a || a >= FRF_AGA_CHIP_LIMIT || (a & 3u)) return 0;
    if (ps_read_32(a + ED_MAGIC * 4u) != FRF_ECS_DENISE_MAGIC) return 0;
    if (ps_read_32(a + ED_VERSION * 4u) != FRF_ECS_DENISE_VERSION) return 0;
    if (ps_read_32(a + ED_WIDTH * 4u) != FRF_ECS_DENISE_MAX_W) return 0;
    if (ps_read_32(a + ED_HEIGHT * 4u) != FRF_ECS_DENISE_MAX_H) return 0;
    if (ps_read_32(a + ED_BYTESPERROW * 4u) != FRF_ECS_DENISE_ROW_BYTES) return 0;
    if (ps_read_32(a + ED_ROWSTRIDE * 4u) != FRF_ECS_DENISE_ROW_BYTES) return 0;

    for (unsigned p = 0; p < FRF_ECS_DENISE_PLANES; ++p) {
        uint32_t pa = ps_read_32(a + (ED_PLANE0 + p) * 4u);
        uint32_t last = pa + FRF_ECS_DENISE_ROW_BYTES * FRF_ECS_DENISE_MAX_H;
        if (!pa || (pa & 1u) || pa >= FRF_AGA_CHIP_LIMIT ||
            last > FRF_AGA_CHIP_LIMIT || last < pa)
            return 0;
        g_ecs_plane[p] = pa;
    }

    g_ecs_restore_copper = ps_read_32(a + ED_RESTORE_COPPER * 4u);
    if (!g_ecs_restore_copper || (g_ecs_restore_copper & 1u) ||
        g_ecs_restore_copper >= FRF_AGA_CHIP_LIMIT)
        return 0;

    {
        uint32_t flags = ps_read_32(a + ED_FLAGS * 4u);
        uint32_t fps = (flags >> 8) & 0xffu;
        g_ecs_fps = (uint8_t)((fps == 50u) ? 50u : 25u);
    }
    return 1;
}

static unsigned ecs_depth_from_bplcon0(uint16_t b)
{
    unsigned d = (b >> 12) & 7u;
    if (d == 0u) d = 1u;
    if (d > 6u) d = 6u;
    return d;
}

static unsigned ecs_words_from_ddf(uint16_t bplcon0, uint16_t start, uint16_t stop)
{
    unsigned s = (unsigned)start & 0x00fcu;
    unsigned e = (unsigned)stop  & 0x00fcu;
    unsigned step = (bplcon0 & FRF_ECS_BPLCON0_HIRES) ? 4u : 8u;
    unsigned words;
    if (e < s) return (bplcon0 & FRF_ECS_BPLCON0_HIRES) ? 40u : 20u;
    words = ((e - s) / step) + 1u;
    if (words < 1u) words = 1u;
    if (words > FRF_ECS_DENISE_MAX_WORDS) words = FRF_ECS_DENISE_MAX_WORDS;
    return words;
}

static unsigned ecs_height_from_diw(uint16_t start, uint16_t stop)
{
    unsigned s = ((unsigned)start >> 8) & 0xffu;
    unsigned e = ((unsigned)stop >> 8) & 0xffu;
    if (e <= s) e += 256u;
    unsigned h = e - s;
    if (h < 1u) h = 256u;
    if (h > FRF_ECS_DENISE_MAX_H) h = FRF_ECS_DENISE_MAX_H;
    return h;
}

static void ecs_capture_meta(frf_ecs_meta_t *m)
{
    uint16_t b = aga_glue_peek_reg(0x100u);
    unsigned words, width, height, source_width;

    m->bplcon0 = b;
    m->diwstrt = aga_glue_peek_reg(0x08eu);
    m->diwstop = aga_glue_peek_reg(0x090u);
    m->ddfstrt = aga_glue_peek_reg(0x092u);
    m->ddfstop = aga_glue_peek_reg(0x094u);
    m->depth = (uint16_t)ecs_depth_from_bplcon0(b);
    m->hires = (uint16_t)((b & (FRF_ECS_BPLCON0_HIRES | FRF_ECS_BPLCON0_SHRES)) != 0u);
    m->lace = (uint16_t)((b & FRF_ECS_BPLCON0_LACE) != 0u);
    m->ham = (uint16_t)((b & FRF_ECS_BPLCON0_HAM) != 0u && m->depth == 6u);
    words = ecs_words_from_ddf(b, m->ddfstrt, m->ddfstop);
    width = words * 16u;
    if (width > FRF_ECS_DENISE_MAX_W) width = FRF_ECS_DENISE_MAX_W;
    height = ecs_height_from_diw(m->diwstrt, m->diwstop);
    m->words = (uint16_t)((width + 15u) >> 4);
    m->width = (uint16_t)width;
    m->height = (uint16_t)height;

    /* The software framebuffer uses hires-coordinate pixels. LORES renderer
     * pixels are duplicated horizontally, while HIRES pixels are 1:1. */
    source_width = m->hires ? width : width * 2u;
    if (source_width > FRF_AGA_SRC_W) source_width = FRF_AGA_SRC_W;
    m->src_x = (uint16_t)(FRF_AGA_PAL_CENTER_X > source_width / 2u ?
        FRF_AGA_PAL_CENTER_X - source_width / 2u : 0u);
    if ((unsigned)m->src_x + source_width > FRF_AGA_SRC_W)
        m->src_x = (uint16_t)(FRF_AGA_SRC_W - source_width);
    m->src_y = (uint16_t)(FRF_AGA_PAL_CENTER_Y > height / 2u ?
        FRF_AGA_PAL_CENTER_Y - height / 2u : 0u);
    if ((unsigned)m->src_y + height > FRF_AGA_SRC_H)
        m->src_y = (uint16_t)(FRF_AGA_SRC_H - height);

    for (unsigned i = 0; i < 32u; ++i)
        m->color[i] = (uint16_t)(aga_glue_peek_reg(0x180u + i * 2u) & 0x0fffu);
}

static int ecs_claim_write_slot(unsigned *slot_out)
{
    uint32_t hint = frf_ecs_load_u32(&g_ecs_submit_seq) + 1u;
    unsigned first = hint & 1u;
    for (unsigned pass = 0; pass < 2u; ++pass) {
        unsigned slot = (first + pass) & 1u;
        uint32_t state = frf_ecs_load_u32(&g_ecs_slot_state[slot]);
        if (state == FRF_ECS_SLOT_BUSY || state == FRF_ECS_SLOT_WRITING) continue;
        uint32_t expected = state;
        if (__atomic_compare_exchange_n(&g_ecs_slot_state[slot], &expected,
                FRF_ECS_SLOT_WRITING, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            if (state == FRF_ECS_SLOT_PENDING) ++g_ecs_dropped;
            *slot_out = slot;
            return 1;
        }
    }
    ++g_ecs_dropped;
    return 0;
}

static void ecs_denise_present(const uint32_t *frame, int native_active)
{
    (void)native_active;
    if (!frame || !ecs_denise_is_active()) return;

    ++g_ecs_input_frames;
    if (g_ecs_fps == 25u && ((g_ecs_input_frames & 1u) == 0u)) return;

    unsigned slot;
    if (!ecs_claim_write_slot(&slot)) return;
    ecs_capture_meta(&g_ecs_meta[slot]);

    /* Stage the complete renderer envelope. It is the same storage already
     * owned by the HAM backend; only one physical backend can be active. */
    const uint64_t *src64 = (const uint64_t *)(const void *)frame;
    uint64_t *dst64 = (uint64_t *)(void *)&g_stage[slot][0][0];
    const unsigned qwords = (FRF_AGA_SRC_W * FRF_AGA_SRC_H) / 2u;
    for (unsigned q = 0; q < qwords; ++q) dst64[q] = src64[q];

    uint32_t seq = __atomic_add_fetch(&g_ecs_submit_seq, 1u, __ATOMIC_RELAXED);
    frf_ecs_store_u32(&g_ecs_slot_seq[slot], seq);
    frf_ecs_store_u32(&g_ecs_slot_state[slot], FRF_ECS_SLOT_PENDING);
    ++g_ecs_submitted;
    if (g_ecs_submitted <= 2u)
        kprintf("[ECS-DENISE] V0.1 CPU3 submit seq=%u mode=%04x %ux%u depth=%u %s\n",
                seq, g_ecs_meta[slot].bplcon0, g_ecs_meta[slot].width,
                g_ecs_meta[slot].height, g_ecs_meta[slot].depth,
                g_ecs_meta[slot].ham ? "HAM6" : g_ecs_meta[slot].hires ? "HIRES" : "LORES");
    __asm__ volatile("dmb ish; sev" ::: "memory");
}

static inline uint8_t ecs_nib_r(uint16_t c) { return (uint8_t)((c >> 8) & 15u); }
static inline uint8_t ecs_nib_g(uint16_t c) { return (uint8_t)((c >> 4) & 15u); }
static inline uint8_t ecs_nib_b(uint16_t c) { return (uint8_t)(c & 15u); }

static void ecs_build_palette_lut(const frf_ecs_meta_t *m)
{
    int same = g_ecs_lut_valid && g_ecs_lut_depth == m->depth &&
               g_ecs_lut_ehb == (uint8_t)(m->depth == 6u && !m->ham);
    if (same) {
        for (unsigned i = 0; i < 32u; ++i)
            if (g_ecs_lut_palette[i] != m->color[i]) { same = 0; break; }
    }
    if (same || m->ham) return;

    for (unsigned i = 0; i < 32u; ++i) g_ecs_lut_palette[i] = m->color[i];
    g_ecs_lut_depth = (uint8_t)m->depth;
    g_ecs_lut_ehb = (uint8_t)(m->depth == 6u);

    unsigned count = m->depth == 6u ? 64u : (1u << m->depth);
    if (count > 32u && m->depth != 6u) count = 32u;
    for (unsigned key = 0; key < 4096u; ++key) {
        int tr = (int)((key >> 8) & 15u);
        int tg = (int)((key >> 4) & 15u);
        int tb = (int)(key & 15u);
        uint32_t best = 0xffffffffu;
        uint8_t bi = 0u;
        for (unsigned i = 0; i < count; ++i) {
            uint16_t c = m->color[i & 31u];
            int r = ecs_nib_r(c), g = ecs_nib_g(c), b = ecs_nib_b(c);
            if (i >= 32u) { r >>= 1; g >>= 1; b >>= 1; }
            int dr = tr-r, dg=tg-g, db=tb-b;
            uint32_t e = (uint32_t)(3*dr*dr + 4*dg*dg + 2*db*db);
            if (e < best) { best=e; bi=(uint8_t)i; }
        }
        g_ecs_lut[key] = bi;
    }
    g_ecs_lut_valid = 1u;
}

static inline uint16_t ecs_rgb12_key(uint32_t rgb)
{
    return (uint16_t)(((rgb >> 12) & 0x0f00u) |
                      ((rgb >> 8)  & 0x00f0u) |
                      ((rgb >> 4)  & 0x000fu));
}

static void ecs_pack_regular_line(const uint32_t *src, const frf_ecs_meta_t *m,
                                  uint16_t out[FRF_ECS_DENISE_PLANES][FRF_ECS_DENISE_MAX_WORDS])
{
    for (unsigned p=0;p<FRF_ECS_DENISE_PLANES;++p)
        for (unsigned w=0;w<m->words;++w) out[p][w]=0u;

    unsigned step = m->hires ? 1u : 2u;
    for (unsigned x=0; x<m->width; ++x) {
        uint32_t rgb = src[x*step];
        uint8_t idx = g_ecs_lut[ecs_rgb12_key(rgb)];
        uint16_t mask = (uint16_t)(0x8000u >> (x & 15u));
        unsigned w = x >> 4;
        for (unsigned p=0; p<m->depth; ++p)
            if (idx & (1u<<p)) out[p][w] |= mask;
    }
}

static uint8_t ecs_ham_direct_pen(uint16_t key, const frf_ecs_meta_t *m)
{
    int tr=(key>>8)&15, tg=(key>>4)&15, tb=key&15;
    uint32_t best=0xffffffffu; uint8_t bi=0;
    for (unsigned i=0;i<16u;++i) {
        int dr=tr-(int)ecs_nib_r(m->color[i]);
        int dg=tg-(int)ecs_nib_g(m->color[i]);
        int db=tb-(int)ecs_nib_b(m->color[i]);
        uint32_t e=(uint32_t)(3*dr*dr+4*dg*dg+2*db*db);
        if(e<best){best=e;bi=(uint8_t)i;}
    }
    return bi;
}

static void ecs_pack_ham6_line(const uint32_t *src, const frf_ecs_meta_t *m,
                               uint16_t out[FRF_ECS_DENISE_PLANES][FRF_ECS_DENISE_MAX_WORDS])
{
    for (unsigned p=0;p<6u;++p) for(unsigned w=0;w<m->words;++w) out[p][w]=0u;
    unsigned pr=ecs_nib_r(m->color[0]), pg=ecs_nib_g(m->color[0]), pb=ecs_nib_b(m->color[0]);
    unsigned step = m->hires ? 1u : 2u;
    for(unsigned x=0;x<m->width;++x){
        uint16_t key=ecs_rgb12_key(src[x*step]);
        unsigned tr=(key>>8)&15u,tg=(key>>4)&15u,tb=key&15u;
        uint8_t direct=ecs_ham_direct_pen(key,m);
        unsigned dr=ecs_nib_r(m->color[direct]),dg=ecs_nib_g(m->color[direct]),db=ecs_nib_b(m->color[direct]);
        int er=(int)tr-(int)dr,eg=(int)tg-(int)dg,eb=(int)tb-(int)db;
        uint32_t best=(uint32_t)(3*er*er+4*eg*eg+2*eb*eb); uint8_t code=direct;
        unsigned nr=dr,ng=dg,nb=db;
        { int a=(int)tr-(int)pr,b=(int)tg-(int)pg; uint32_t e=(uint32_t)(3*a*a+4*b*b); if(e<best){best=e;code=(uint8_t)(0x10u|tb);nr=pr;ng=pg;nb=tb;} }
        { int a=(int)tg-(int)pg,b=(int)tb-(int)pb; uint32_t e=(uint32_t)(4*a*a+2*b*b); if(e<best){best=e;code=(uint8_t)(0x20u|tr);nr=tr;ng=pg;nb=pb;} }
        { int a=(int)tr-(int)pr,b=(int)tb-(int)pb; uint32_t e=(uint32_t)(3*a*a+2*b*b); if(e<best){code=(uint8_t)(0x30u|tg);nr=pr;ng=tg;nb=pb;} }
        uint16_t mask=(uint16_t)(0x8000u>>(x&15u)); unsigned w=x>>4;
        for(unsigned p=0;p<6u;++p) if(code&(1u<<p)) out[p][w]|=mask;
        pr=nr;pg=ng;pb=nb;
    }
}

static void ecs_publish_run(unsigned plane, unsigned y, unsigned first, unsigned last,
                            const uint16_t row[FRF_ECS_DENISE_MAX_WORDS])
{
    uint32_t addr=g_ecs_plane[plane]+y*g_ecs_row_stride+first*2u;
    unsigned i=first;
    if((addr&3u)&&i<last){ps_write_16(addr,row[i++]);addr+=2u;g_ecs_chip_bytes+=2u;}
    while(i+1u<last){ps_write_32(addr,((uint32_t)row[i]<<16)|row[i+1u]);addr+=4u;i+=2u;g_ecs_chip_bytes+=4u;}
    if(i<last){ps_write_16(addr,row[i]);g_ecs_chip_bytes+=2u;}
    ++g_ecs_runs;
}

static int ecs_meta_mode_equal(const frf_ecs_meta_t *a,const frf_ecs_meta_t *b)
{
    return a->bplcon0==b->bplcon0 && a->diwstrt==b->diwstrt &&
           a->diwstop==b->diwstop && a->ddfstrt==b->ddfstrt &&
           a->ddfstop==b->ddfstop && a->width==b->width &&
           a->height==b->height && a->depth==b->depth && a->ham==b->ham;
}

static void ecs_program_physical_mode(const frf_ecs_meta_t *m)
{
    uint16_t phys=(uint16_t)(FRF_ECS_BPLCON0_COLOR |
        ((m->depth&7u)<<12) |
        (m->hires?FRF_ECS_BPLCON0_HIRES:0u) |
        (m->lace?FRF_ECS_BPLCON0_LACE:0u) |
        (m->ham?FRF_ECS_BPLCON0_HAM:0u));
    uint16_t mod=(uint16_t)(FRF_ECS_DENISE_ROW_BYTES - m->words*2u);

    ps_write_16(0xDFF096u,0x01A0u); /* COPEN+BPLEN+SPREN clear; keep audio/DMA peers untouched. */
    for(unsigned p=0;p<FRF_ECS_DENISE_PLANES;++p){
        uint32_t a=g_ecs_plane[p]; uint32_t r=0xDFF0E0u+p*4u;
        ps_write_16(r,(uint16_t)(a>>16)); ps_write_16(r+2u,(uint16_t)a);
    }
    ps_write_16(0xDFF08Eu,m->diwstrt); ps_write_16(0xDFF090u,m->diwstop);
    ps_write_16(0xDFF092u,m->ddfstrt); ps_write_16(0xDFF094u,m->ddfstop);
    ps_write_16(0xDFF100u,phys); ps_write_16(0xDFF102u,0u); ps_write_16(0xDFF104u,0u);
    ps_write_16(0xDFF108u,mod); ps_write_16(0xDFF10Au,mod);
    ps_write_16(0xDFF1FCu,0u); /* FMODE: ECS-safe 16-bit fetch on AGA physical chipsets. */
    for(unsigned i=0;i<32u;++i) ps_write_16(0xDFF180u+i*2u,m->color[i]);
    ps_write_16(0xDFF096u,0x8300u); /* SET DMAEN+BPLEN; Copper intentionally remains off. */
    g_ecs_mode_programmed=1u;
    kprintf("[ECS-DENISE] V0.1 physical %s %ux%u depth=%u %s LACE=%u mod=%u\n",
            m->hires?"HIRES":"LORES",m->width,m->height,m->depth,
            m->ham?"HAM6":(m->depth==6u?"EHB":"indexed"),m->lace,mod);
}

static void ecs_restore_physical(void)
{
    ps_write_16(0xDFF096u,0x01A0u);
    if(g_ecs_restore_copper){
        ps_write_16(0xDFF080u,(uint16_t)(g_ecs_restore_copper>>16));
        ps_write_16(0xDFF082u,(uint16_t)g_ecs_restore_copper);
        ps_write_16(0xDFF088u,0u);
        ps_write_16(0xDFF096u,0x83A0u);
    }
    g_ecs_mode_programmed=0u;
}

static int ecs_cpu1_claim_newest(void)
{
    int best=-1; uint32_t bestseq=0;
    for(unsigned s=0;s<2u;++s){
        if(frf_ecs_load_u32(&g_ecs_slot_state[s])!=FRF_ECS_SLOT_PENDING)continue;
        uint32_t q=frf_ecs_load_u32(&g_ecs_slot_seq[s]); if(best<0||q>bestseq){best=(int)s;bestseq=q;}
    }
    if(best<0)return 0;
    uint32_t expected=FRF_ECS_SLOT_PENDING;
    if(!__atomic_compare_exchange_n(&g_ecs_slot_state[best],&expected,FRF_ECS_SLOT_BUSY,0,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))return 0;
    for(unsigned s=0;s<2u;++s) if((int)s!=best && frf_ecs_load_u32(&g_ecs_slot_state[s])==FRF_ECS_SLOT_PENDING){
        uint32_t q=frf_ecs_load_u32(&g_ecs_slot_seq[s]); if(q<bestseq){uint32_t old=FRF_ECS_SLOT_PENDING; if(__atomic_compare_exchange_n(&g_ecs_slot_state[s],&old,FRF_ECS_SLOT_FREE,0,__ATOMIC_ACQ_REL,__ATOMIC_ACQUIRE))++g_ecs_dropped;}
    }
    frf_ecs_store_i32(&g_ecs_worker_slot,best); g_ecs_worker_seq=bestseq; g_ecs_worker_y=0u; g_ecs_worker_meta=g_ecs_meta[best];
    return 1;
}

static void ecs_process_line(unsigned slot,unsigned y)
{
    uint16_t packed[FRF_ECS_DENISE_PLANES][FRF_ECS_DENISE_MAX_WORDS];
    const frf_ecs_meta_t *m=&g_ecs_worker_meta;
    const uint32_t *src=&g_stage[slot][m->src_y+y][m->src_x];
    if(m->ham) ecs_pack_ham6_line(src,m,packed); else {ecs_build_palette_lut(m); ecs_pack_regular_line(src,m,packed);}
    for(unsigned p=0;p<FRF_ECS_DENISE_PLANES;++p){
        unsigned activep=p<m->depth?1u:0u;
        unsigned i=0;
        while(i<m->words){
            uint16_t v=activep?packed[p][i]:0u;
            while(i<m->words && !g_ecs_force_full && g_ecs_shadow[p][y][i]==v){++i;if(i<m->words)v=activep?packed[p][i]:0u;}
            if(i>=m->words)break;
            unsigned first=i;
            while(i<m->words){v=activep?packed[p][i]:0u;if(!g_ecs_force_full&&g_ecs_shadow[p][y][i]==v)break;g_ecs_shadow[p][y][i]=v;++i;}
            ecs_publish_run(p,y,first,i,packed[p]);
        }
    }
}

static void ecs_finish_frame(unsigned slot)
{
    ++g_ecs_frames; ++g_ecs_completed;
    frf_ecs_store_u32(&g_ecs_slot_state[slot],FRF_ECS_SLOT_FREE);
    frf_ecs_store_i32(&g_ecs_worker_slot,-1);
    if(g_ecs_force_full){g_ecs_force_full=0u;g_ecs_enable_pending=1u;}
    if(g_ecs_frames<=2u) kprintf("[ECS-DENISE] V0.1 CPU1 complete seq=%u frame=%u chipbytes=%u runs=%u dropped=%u\n",g_ecs_worker_seq,g_ecs_frames,g_ecs_chip_bytes,g_ecs_runs,g_ecs_dropped);
}

static int ecs_denise_begin(void)
{
    if(!ecs_descriptor_read())return 0;
    /* Stop stale HAM state without restoring its Copper: this presenter is
     * taking physical ownership immediately from the same descriptor channel. */
    frf_store_u32(&g_active,0u);
    async_slots_reset();
    ecs_slots_reset();
    for(unsigned p=0;p<6u;++p)for(unsigned y=0;y<FRF_ECS_DENISE_MAX_H;++y)for(unsigned w=0;w<FRF_ECS_DENISE_MAX_WORDS;++w)g_ecs_shadow[p][y][w]=0xffffu;
    g_ecs_input_frames=g_ecs_submitted=g_ecs_completed=g_ecs_dropped=0u;
    g_ecs_chip_bytes=g_ecs_runs=g_ecs_frames=0u; g_ecs_force_full=1u; g_ecs_enable_pending=0u; g_ecs_mode_programmed=0u; g_ecs_lut_valid=0u;
    ps_write_16(0xDFF096u,0x01A0u); /* blank physical bitplanes/Copper/sprites until first complete frame */
    frf_ecs_store_u32(&g_ecs_stop_requested,0u); frf_ecs_store_u32(&g_ecs_active,1u);
    kprintf("[ECS-DENISE] V0.1 CPU1 bridge online: virtual ECS -> real planar Denise; bank=%u bytes fps=%u restoreCopper=%08x\n",
            FRF_ECS_DENISE_ROW_BYTES*FRF_ECS_DENISE_MAX_H*6u,(unsigned)g_ecs_fps,g_ecs_restore_copper);
    return 1;
}

static void ecs_denise_request_stop(void)
{
    frf_ecs_store_u32(&g_ecs_stop_requested,1u);
}

static void ecs_denise_cpu1_service(void)
{
    int selected=ecs_denise_descriptor_selected();
    if(frf_ecs_load_u32(&g_ecs_stop_requested) || (ecs_denise_is_active()&&!selected)){
        if(ecs_denise_is_active()) ecs_restore_physical();
        frf_ecs_store_u32(&g_ecs_active,0u); ecs_slots_reset(); frf_ecs_store_u32(&g_ecs_stop_requested,0u);
        kprintf("[ECS-DENISE] V0.1 physical presenter stopped/restored\n");
        return;
    }
    if(selected&&!ecs_denise_is_active()){if(!ecs_denise_begin())return;}
    if(!ecs_denise_is_active())return;

    if(g_ecs_enable_pending){
        if(!physical_vblank_window())return;
        ecs_program_physical_mode(&g_ecs_worker_meta); g_ecs_enable_pending=0u;
    }

    int32_t worker=frf_ecs_load_i32(&g_ecs_worker_slot);
    if(worker<0){if(!ecs_cpu1_claim_newest())return;worker=frf_ecs_load_i32(&g_ecs_worker_slot);if(worker<0)return;
        static frf_ecs_meta_t lastm; static uint8_t lastvalid;
        if(!lastvalid||!ecs_meta_mode_equal(&lastm,&g_ecs_worker_meta)){
            ps_write_16(0xDFF096u,0x0100u); g_ecs_force_full=1u; g_ecs_mode_programmed=0u; lastm=g_ecs_worker_meta; lastvalid=1u;
        }
        for(unsigned i=0;i<32u;++i) {
            if(!g_ecs_mode_programmed || lastm.color[i]!=g_ecs_worker_meta.color[i])
                ps_write_16(0xDFF180u+i*2u,g_ecs_worker_meta.color[i]);
            lastm.color[i]=g_ecs_worker_meta.color[i];
        }
    }

    unsigned budget=FRF_ECS_CPU1_LINES_PER_SERVICE;
    while(budget--&&g_ecs_worker_y<g_ecs_worker_meta.height){ecs_process_line((unsigned)worker,g_ecs_worker_y);++g_ecs_worker_y;}
    if(g_ecs_worker_y>=g_ecs_worker_meta.height)ecs_finish_frame((unsigned)worker);

    if(!g_ecs_mode_programmed && !g_ecs_enable_pending && !g_ecs_force_full){g_ecs_enable_pending=1u;}
}
