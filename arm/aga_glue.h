/* AGA-PISTORM — Emu68 glue interface.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef AGA_GLUE_H
#define AGA_GLUE_H
#include <stdint.h>

extern volatile int aga_enabled;
extern int aga_enter_stop;
extern int aga_leave_stop;   /* bring-up: stop sandbox LEAVE after step N */
extern int aga_loop_stop;    /* bring-up: limit what the chipset loop does */   /* bring-up: stop sandbox entry after step N */

/* Bring-up watchdog: seconds the chipset may run without completing a frame
   before it parks itself. So a failed sandbox entry recovers instead of
   taking the whole damn machine down and leaving the counters unreadable. */
#define AGA_WATCHDOG_SEC 5

#define AGA_FB_RESERVE  (4u << 20)   /* bytes reserved after the chip RAM for two output frames */

/* start.c: after memory setup, before M68K_StartEmu. chipram_va is the ARM
   address of a cached, inner-shareable block that is also mapped at 68k
   0x000000; fb_va/fb_phys is a cached AGA_FB_RESERVE block for the HVS
   output frames. */
int  aga_glue_init(uint8_t *chipram_va, uint32_t size, uint8_t *fb_va, uintptr_t fb_phys, int ntsc, int enable);

/* Sandbox mode (ECS boot): switch the running machine onto the virtual chipset
   for a WHDLoad game and back. Called from the trap path on the control
   register ($DFF1F2 = $5A0B enter, $5A0F leave). Return 1 = done. */
int  aga_sandbox_enter(void);
int  aga_sandbox_leave(void);
extern volatile int aga_sandbox_active;

/* Which chipset the sandbox presents: $DFF1F2 = $5E00 AGA (default), $5E01 ECS
   Denise. Sent by agaboot at sandbox entry, from its ECS toggle. */
void aga_chipset_ecs(int on);
/* FRF_AGA_SYSTEMWIDE_HAM6_OS_HANDOVER_V0_1_1: consume OS Copper pointer only inside active sandbox. */
int aga_systemwide_seed_os_copper(uint32_t cop1lc);
/* Program-scoped AGA/HAM6: prepare before descriptor publication, then enter
   with $5A0D. Physical HAM stays off until a non-OS Copper list is stable. */
void aga_program_prepare(void);
void aga_program_cancel(void);
extern int aga_program_mode;


/* Virtual floppy (vectors.c): CIA-B PRB writes / CIA-A PRA and SDR reads pass
   through here when aga_enabled. Return 1 = handled (value delivered). */
int  aga_cia_write(uint64_t far, int size, uint64_t value);
int  aga_cia_read(uint64_t far, int size, uint64_t *value);
/* control register $DFF1F2 = $ADxx: disk image commands (see aga_disk_command) */
int  aga_disk_command(unsigned cmd);
/* M68k RESET instruction / ColdReboot: the machine restarts on the current
   chipset (virtual disks and window mode are kept) */
void aga_machine_reset(void);

/* vectors.c: first thing in SYSWriteValToAddr / SYSReadValFromAddr. Return 1 = handled. */
int  aga_trap_write(uint64_t far, int size, uint64_t value, uint64_t value2);
int  aga_trap_read(uint64_t far, int size, uint64_t *value, uint64_t *value2);

/* Chip RAM redirection WITHOUT the MMU. While the sandbox is up, every 68k
   access to 0..chip_size that reaches the abort handler gets served from the
   virtual chip RAM here instead of going out over the PiStorm bus. Code and
   data alike: the JIT's instruction-cache line fill is a plain 16-byte load
   from the 68k address, so it faults into the same door. Return 1 = handled.
   Cheap when off: one volatile load and a compare. */
int  aga_chip_read(uint64_t far, int size, uint64_t *value, uint64_t *value2);
int  aga_chip_write(uint64_t far, int size, uint64_t value, uint64_t value2);
extern volatile int aga_chip_redirect;
extern int aga_use_mmu;      /* aga.mmu on the cmdline: the old MMU-remap path, for A/B */
/* $5C0N on the control register: redirect a window of chip RAM through the
   abort handler with the real chipset left running (1 = top 64 KB, 2 = all,
   0 = off, copied back). Isolates "serving chip RAM from chip_va" from
   "switching the chipset" - sandbox entry does both of those at once, which
   is a crap way to debug either one. */
int  aga_redirect_test(unsigned n);

/* CPU3 renderer ownership. aga_chipset_main() is the standalone Emu68 owner;
 * aga_chipset_session() performs one enabled session and returns on leave. */
void aga_chipset_main(void);
void aga_chipset_session(void);

/* aga_hvs.c: HVS plane output, coexisting with the P96 VideoCore driver */
int  aga_video_init_at(uint8_t *fb_va, uintptr_t fb_phys);     /* core 0, at boot */
int  aga_video_init(void);
void aga_video_present(const uint32_t *frame, int native_active); /* AGA_OUT_W x AGA_OUT_H, 0x00RRGGBB */
void aga_video_set_window(int mode, int x, int y, int w, int h);  /* windowed presentation rectangle */
void aga_video_hide(void);        /* take the plane off the display: sandbox leave, watchdog park */
void aga_video_probe_pointer(void); /* re-read the RTG mouse pointer into the PTR_NOW counters */
/* Debug overlay drawn into the presented frame. It is MY output, not the
   game's, so it shows even when the game draws nothing at all - which is the
   only way to read a machine whose OS a wedged game has taken away. */
void aga_video_overlay(int row, uint32_t value);
void aga_video_overlay_enable(int on);
extern volatile int aga_overlay_idle;   /* $5F0N: overlay on the idle (no native display) plane */

/* aga_audio.c: Paula audio through the real Paula (samples staged in real chip RAM) */
void aga_audio_init(const uint8_t *chipram, uint32_t chipram_size);
int  aga_audio_reg_write(uint32_t reg, uint16_t v);     /* virtual Paula writes; native proxy observes them */
void aga_audio_stream(const int16_t *lr, int frames);   /* software fallback PCM path */
/* FRF_AGA_NATIVE_PAULA_V0_1 */
void aga_audio_set_engine(unsigned mode);               /* 0 software, 1 native, 2 auto */
void aga_audio_set_buffer_size_code(unsigned code);     /* pre-entry low-Chip allocation size */
int  aga_audio_software_needed(void);                   /* skip the mixer when native Paula owns playback */
/* 32-bit WRITE to $DFF1EA: agaboot hands over a chip RAM buffer it has
   AllocMem'd for the Paula DMA rings, before it enters the sandbox.

   The Pi-side register block is shared, so keep this map straight:
     write $DFF1E8, 16 bit  diag selector      (aga_diag_select)
     read  $DFF1EA, 16 bit  diag data, auto-advancing
     write $DFF1EA, 32 bit  audio ring address (here)
     write $DFF1EC/$1EE     ADF image address  (aga_disk_command, sandbox only)
     write $DFF1F2, 16 bit  control word       ($5A0B/$5A0F/$5E0N/$ADxx/$A6A1/$EC50)
   Write and read of $DFF1EA go through different paths at different widths,
   so they do not collide - but do NOT add a third fucking meaning to it. */
void aga_audio_set_ring(uint32_t base);
int16_t *aga_audio_scratch(int *max_frames);
void aga_audio_stop(void);
void aga_audio_bus_service(void);   /* the housekeeper core's share of the audio: bus writes */

/* aga_hvs.c: reboot the Pi into AGA (tryboot.txt, one shot) or ECS (config.txt) */
void aga_reboot(int tryboot);

/* ---- diagnostics (aga_diag.c) ---------------------------------------------
   Counters are at frame granularity or rarer, so they stay in the shipped
   build; the per-trap ones need AGA_DIAG and are compiled out otherwise. */
enum {
    AGA_D_FRAMES = 0, AGA_D_FRAME_US, AGA_D_FRAME_US_WORST, AGA_D_LAG_MAX,
    AGA_D_RESYNCS, AGA_D_AUDIO_STREAMS, AGA_D_AUDIO_RESYNCS,
    AGA_D_PRESENTS, AGA_D_VIDEO_ERRORS, AGA_D_SANDBOX_IN, AGA_D_SANDBOX_OUT,
    AGA_D_DISK_CMDS, AGA_D_REBOOTS, AGA_D_TRAPS_R, AGA_D_TRAPS_W, AGA_D_STATE,
    AGA_D_BPL_FRAMES,   /* frames where something actually had bitplanes on */
    /* interrupt delivery: does an IPL raised by the virtual chipset ever turn
       into a 68k exception? ipl_hook counts raises; irq_l1..7 count exceptions
       the execution loop actually took, per level, from ANY source;
       irq_from_virtual counts the ones where the virtual chipset's byte won. */
    AGA_D_IPL_HOOK, AGA_D_IRQ_VIRT,
    AGA_D_IRQ_L1, AGA_D_IRQ_L2, AGA_D_IRQ_L3, AGA_D_IRQ_L4, AGA_D_IRQ_L5, AGA_D_IRQ_L6, AGA_D_IRQ_L7,
    /* the REAL chipset's registers, sampled once per frame by the chipset core
       while the sandbox is up. The 68k cannot read them itself there (its reads
       are the virtual ones), and "level 2 stops after three seconds" needs to
       know whether the real Paula still has PORTS enabled and pending. */
    AGA_D_REAL_INTENA, AGA_D_REAL_INTREQ, AGA_D_REAL_DMACON,
    /* in-game blank screen: is anything filling the bitplanes at all? */
    AGA_D_BLITS,        /* blits started since reset */
    AGA_D_DATA_FRAMES,  /* frames in which a bitplane fetch returned non-zero data */
    /* 68k traffic through the chip RAM redirect, and WHERE it writes: eight
       256 KB buckets over the 2 MB. The display's BPLxPT (deepest snapshot)
       says where the chipset fetches from; these say where the game actually
       put bytes. A blank screen with writes in the wrong bucket is an address
       problem; no writes at all is a loading problem. */
    AGA_D_CHIP_R, AGA_D_CHIP_W,
    AGA_D_CHIPW_B0, AGA_D_CHIPW_B1, AGA_D_CHIPW_B2, AGA_D_CHIPW_B3,
    AGA_D_CHIPW_B4, AGA_D_CHIPW_B5, AGA_D_CHIPW_B6, AGA_D_CHIPW_B7,
    /* Per audio channel: how many times the game switched its DMA on, and how
       many sample words it actually fetched. Music plays and effects mostly do
       not, and no existing counter can tell the two apart - these say whether
       an effect channel is ever STARTED (a game that never starts it is a
       different bug from one whose sound is started and then lost). */
    AGA_D_AUD0_ON, AGA_D_AUD1_ON, AGA_D_AUD2_ON, AGA_D_AUD3_ON,
    AGA_D_AUD0_W,  AGA_D_AUD1_W,  AGA_D_AUD2_W,  AGA_D_AUD3_W,
    AGA_D_AUD_NUDGE,      /* one-sample drift corrections: never counted until now */
    /* Split by DIRECTION, which is the whole diagnosis: dropping samples makes
       the music play fast, repeating them makes it play slow. 92 nudges a
       second under gameplay against 1.5 on the desktop is a 0.33% rate error,
       far more than two crystals can differ by. */
    AGA_D_AUD_DROP, AGA_D_AUD_REPEAT,
    /* Colour clocks each channel spent playing a NON-ZERO sample. Words fetched
       proves nothing: the standard one-shot idiom plays a sound and then parks
       the channel on a two-word silent loop, so a channel can fetch millions of
       words that are all zeros. This says whether a channel makes any SOUND. */
    AGA_D_AUD0_NZ, AGA_D_AUD1_NZ, AGA_D_AUD2_NZ, AGA_D_AUD3_NZ,
    AGA_D_AUD0_VOL, AGA_D_AUD1_VOL, AGA_D_AUD2_VOL, AGA_D_AUD3_VOL,
    AGA_D_AUD0_VMAX, AGA_D_AUD1_VMAX, AGA_D_AUD2_VMAX, AGA_D_AUD3_VMAX,
    AGA_D_AUD0_AUD, AGA_D_AUD1_AUD, AGA_D_AUD2_AUD, AGA_D_AUD3_AUD,
    AGA_D_AUD0_VWR, AGA_D_AUD1_VWR, AGA_D_AUD2_VWR, AGA_D_AUD3_VWR,
    AGA_D_AUD0_VNZ, AGA_D_AUD1_VNZ, AGA_D_AUD2_VNZ, AGA_D_AUD3_VNZ,
    AGA_D_AUD0_OFF, AGA_D_AUD1_OFF, AGA_D_AUD2_OFF, AGA_D_AUD3_OFF,
    AGA_D_AUD0_WRAP, AGA_D_AUD1_WRAP, AGA_D_AUD2_WRAP, AGA_D_AUD3_WRAP,
    AGA_D_AUD_INTENA,     /* the virtual INTENA, so I can see if AUDx is even enabled */
    /* Display-list words that were already in use when I borrowed the region
       LIST_A..KERNEL_WORD. Non-zero means the RTG driver had allocated there -
       which is how the mouse pointer went invisible while still tracking. */
    AGA_D_DL_INUSE,
    /* longest run of zero words in the display-list RAM, and where it starts:
       where my lists could live without standing on the RTG driver. */
    AGA_D_DL_FREE_AT, AGA_D_DL_FREE_LEN,
    /* The mouse pointer as the RTG driver built it, read out of the driver's
       own live display list: plane 2 of that list is the pointer
       (VideoCore.card vc6.c VC6_SetPanning builds it straight after the
       screen plane). Captured at sandbox entry (IN), at exit in hide()
       (OUT), and live whenever a tool starts a diagnostic dump (NOW) - so
       agastat run while the pointer is invisible shows what is on screen.
         LIST   the DISPLIST1 value the list was read from
         PLANES planes before the terminator (screen, pointer, overlay)
         CTL    plane 2's control word (0 = there is no second plane)
         POS0   its position word; 0x2FFF2FFF = parked by VC6_SetSprite(FALSE)
         IMAGE  its PTR0: 0xC0000000 | Amiga address of vc4_SpriteShape
         PAL    first pointer colour, alpha in the top byte (0 = transparent) */
    AGA_D_PTR_IN_LIST,  AGA_D_PTR_IN_PLANES,  AGA_D_PTR_IN_CTL,
    AGA_D_PTR_IN_POS0,  AGA_D_PTR_IN_IMAGE,   AGA_D_PTR_IN_PAL,
    AGA_D_PTR_OUT_LIST, AGA_D_PTR_OUT_PLANES, AGA_D_PTR_OUT_CTL,
    AGA_D_PTR_OUT_POS0, AGA_D_PTR_OUT_IMAGE,  AGA_D_PTR_OUT_PAL,
    AGA_D_PTR_NOW_LIST, AGA_D_PTR_NOW_PLANES, AGA_D_PTR_NOW_CTL,
    AGA_D_PTR_NOW_POS0, AGA_D_PTR_NOW_IMAGE,  AGA_D_PTR_NOW_PAL,
    /* the frame's time budget on the chipset core (2026-09-13, for the
       Pi 3 question): busy = the frame minus the pacing wait, in us;
       audio = time inside aga_audio_stream, i.e. the PiStorm bus */
    AGA_D_LOOP_BUSY_US, AGA_D_LOOP_BUSY_WORST, AGA_D_LOOP_WAIT_US,
    AGA_D_AUDIO_US_FRAME, AGA_D_AUDIO_US_WORST,
    AGA_D_LOCK_US_FRAME, AGA_D_LOCK_US_WORST,   /* core 3 waiting for the chipset lock */
    AGA_D_SOC_TEMP_MC, AGA_D_THROTTLED,         /* firmware: temperature, throttle bits, at leave */
    AGA_D_BUS_US_FRAME, AGA_D_BUS_US_WORST,     /* the three real-register reads per frame */
    AGA_D_CYC_US_FRAME, AGA_D_CYC_US_WORST,     /* aga_run_cycles: per frame, worst single chunk */
    AGA_D_STAT_US_WORST, AGA_D_BODY_US_WORST,   /* the frame statistics block; the whole body between two waits */
    AGA_D_CORE3_DAIF,                           /* interrupt mask bits on core 3 */
    AGA_D_CYC_FRAME_WORST,                      /* worst per-frame sum of aga_run_cycles */
    AGA_D_CHIPW_FRAME, AGA_D_CHIPW_FRAME_WORST, /* 68k chip RAM writes per frame (AGA_DIAG builds) */
    AGA_D_MEM_US, AGA_D_MEM_US_WORST,           /* the DRAM latency probe, per frame */
    AGA_D_CHIP_REAL_KB,                         /* real chip RAM found at the last entry */
    AGA_D_ABORT_US_FRAME, AGA_D_ABORT_US_WORST, /* core 0 inside chip RAM aborts, per frame */
    AGA_D_ABORTS_FRAME, AGA_D_ABORTS_WORST,     /* how many of them per frame */
    AGA_D_L1_REFILL_FRAME, AGA_D_L1_REFILL_WORST, /* core 3 PMU: L1 data cache refills per frame */
    AGA_D_L2_REFILL_FRAME, AGA_D_L2_REFILL_WORST, /* core 3 PMU: L2 refills per frame */
    /* the HVS for three seconds after the hand-back, sampled every 20 ms
       by the housekeeper core (Pi 3 black screen, 2026-09-14) */
    AGA_D_WATCH_SAMPLES, AGA_D_WATCH_LIST_CHANGES, AGA_D_WATCH_STAT_MOVES,
    AGA_D_WATCH_LACT_FIRST, AGA_D_WATCH_LIST_LAST, AGA_D_WATCH_LACT_LAST,
    AGA_D_WATCH_STAT_LAST, AGA_D_WATCH_CTL_LAST, AGA_D_WATCH_PTR0_LAST,
    AGA_D_WATCH_LATCH_US,                       /* hide: how long until the HVS latched the driver's list */
    AGA_D_FRAMES_SKIPPED,                       /* Pi 3: frames not drawn because the loop was behind */
    /* 3 or 4: the Pi this kernel was built for, and the Pi it found. agaboot
       reads them BY INDEX (147, 148) for the AGA toggle's reinstall warning;
       aga_diag.c asserts the indices, so add new counters after these. */
    AGA_D_BUILD_BOARD, AGA_D_RUN_BOARD,
    /* presented frames by how far behind real time the loop was:
       <2, 2-5, 5-10, 10-20, 20-40, >=40 ms */
    AGA_D_LAG_HIST0, AGA_D_LAG_HIST1, AGA_D_LAG_HIST2,
    AGA_D_LAG_HIST3, AGA_D_LAG_HIST4, AGA_D_LAG_HIST5,
    AGA_DIAG_COUNT
};
extern uint32_t aga_diag_ctr[AGA_DIAG_COUNT];
void aga_diag_init(void);
void aga_diag_set(int idx, uint32_t v);
void aga_diag_inc(int idx);
void aga_diag_max(int idx, uint32_t v);
void aga_diag_irq(int level, int from_virtual);   /* ExecutionLoop.c: an exception was taken */
void aga_diag_log(const char *line);

/* A Pi 3 (VideoCore IV, 19.2 MHz generic timer) against a Pi 4 / CM4
   (54 MHz) - the same test Emu68 and aga_hvs.c use. The two boards get
   different tolerances where the Pi 3's smaller cluster needs them. */
static inline int aga_is_pi3(void)
{
    uint64_t f; __asm__ volatile("mrs %0, CNTFRQ_EL0" : "=r"(f));
    return f <= 20000000;
}

/* 1 in the Pi 3 kernel (cmake -DAGA_PI3=ON), 0 in the Pi 4 one. A compile-
   time constant, so `if (AGA_PI3_BUILD)` leaves nothing in the Pi 4 kernel. */
#ifdef AGA_PI3
#define AGA_PI3_BUILD 1
#else
#define AGA_PI3_BUILD 0
#endif
/* The Pi 3 tolerances apply only where both agree: the Pi 3 kernel on a
   Pi 3. The Pi 3 kernel on a Pi 4 behaves as the Pi 4 kernel. */
static inline int aga_pi3_mode(void) { return AGA_PI3_BUILD && aga_is_pi3(); }

/* 1 in test kernels (cmake -DAGA_PROBES=ON, or AGA_DIAG), 0 in published ones:
   the per-frame measurements that cost time while a game runs. */
#ifdef AGA_PROBES
#define AGA_PROBES_ON 1
#else
#define AGA_PROBES_ON 0
#endif
void aga_diag_select(uint16_t v);
uint16_t aga_diag_read_word(void);
const char *aga_diag_name(int idx);
void aga_diag_trap(uint32_t reg, uint16_t val, int write, uint32_t pc);  /* AGA_DIAG only */
void aga_diag_snapshot(int had_data);   /* per frame: record the display state, watch for a blank one */
int  aga_diag_is_frozen(void);  /* the game's display went away: state kept, recording stopped */
void aga_diag_reg(uint32_t reg, int write);   /* per-register access histogram */
void aga_diag_beam_value(uint16_t v);         /* what a beam read handed back */
void aga_diag_reg_reset(void);                /* zero both, when the blank display starts */
void aga_diag_cia(uint32_t far, int write);   /* CIA traffic, which bypasses every other counter */
void aga_diag_cia_value(uint32_t far, int write, uint8_t v);   /* the bytes on CIA-B port A */
void aga_diag_serial(uint8_t c);              /* a byte the 68k wrote to SERDAT */
void aga_diag_capture_code(uint32_t pc);      /* 68k code around the PC I froze at */
void aga_diag_wave(uint8_t l, uint8_t r);     /* the exact bytes handed to the DAC */
void aga_diag_wave_freeze(int on);            /* keep the game's last 15 s; reset on entry */
void aga_diag_triggers(const uint32_t *v, int n);   /* the last audio channel starts */
void aga_diag_audw(uint32_t reg, int size, uint32_t value, uint32_t vpos, uint32_t hpos);   /* sound-register writes, in order */
void aga_diag_env(const uint32_t *nrg_delta);       /* per-channel loudness, this frame */
void aga_diag_wrapinfo(const uint32_t *v, int n);   /* what each channel loops back to */
void aga_diag_env_freeze(int on);
void aga_diag_pc_sample(uint32_t pc);         /* real 68k PC, sampled at interrupt dispatch */
uint32_t aga_diag_hot_pc(void);               /* the bucket with the most samples */
void aga_audio_mask(unsigned m);              /* $5D0N: which channels reach the mix */
extern int aga_game_mode;                     /* armed by the WHDLoad hook at sandbox entry */
extern int aga_exter_locked;                  /* WHDLoad has taken over: never re-arm EXTER */

#ifdef AGA_DIAG
#define AGA_DIAG_INC(i) aga_diag_inc(i)
#define AGA_DIAG_TRAP(r, v, w, p) aga_diag_trap(r, v, w, p)
#else
#define AGA_DIAG_INC(i) ((void)0)
#define AGA_DIAG_TRAP(r, v, w, p) ((void)0)
#endif

#endif
