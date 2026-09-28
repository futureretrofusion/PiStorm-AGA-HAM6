/* FRF_HAM6_WHD_REARM_DIAG_V0_5 */
/* AGA-PISTORM — Emu68 glue: runs the AGA core on a spare ARM core and serves
 * the emulated 68k's chip RAM / custom register accesses.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Built inside the Emu68 tree (docs/EMU68-PATCH.md) and glued to these
 * internals - change them and this file shits itself:
 *   - __m68k_state (struct M68KState *), INTF.AGA byte added to the INTF union
 *   - ps_read_16 / ps_write_16 (PiStorm bus)
 *   - mmu_map(), tlsf/allocators for the virtual chip RAM block
 *   - kprintf()
 *   - the trap path: SYSWriteValToAddr / SYSReadValFromAddr in vectors.c call
 *     aga_trap_write / aga_trap_read first when aga_enabled.
 *
 * Threading: core 0 (JIT) enters aga_trap_* from the data abort handler; core 3
 * owns the AGA renderer.  aga_chipset_main() is the standalone Emu68 CPU3 owner
 * and calls aga_chipset_session() while the virtual chipset is enabled. Both
 * sides take `aga_lock` around any core call.
 */
#include <stdint.h>
#include <string.h>
#include "aga/chipset.h"
#include "aga_glue.h"
#include "aga_ham6.h"   /* FRF_AGA_DIRECTCHIP_HAM6_V0_1 */
#include "mmu.h"
#include "A64.h"          /* MMU_* page attributes */
#include "cache.h"        /* Emu68's software I/D caches: flushed/invalidated around redirection */

/* ---- Emu68 externs ------------------------------------------------------- */
struct M68KState;
extern struct M68KState *__m68k_state;
extern void kprintf(const char *fmt, ...);
/* The bus API differs per board: plain functions on the classic CPLD PiStorm,
   function pointers on PiStorm16/32. Include the header instead of
   re-declaring it, so it can't drift from whatever I'm building. */
#include "ps_protocol.h"

/* FRF_AGA_WHDLOAD_HAM6_HANDOVER_V0_3
 *
 * Physical HAM6 ownership must not start merely because the virtual AGA sandbox
 * exists.  Workbench / WHDLoad startup keeps the real RGB display until the
 * existing WHDLoad ownership detector observes INTENA.EXTER being cleared.
 *
 * The custom-register trap only requests the transition.  CPU3 consumes it next
 * to the existing HAM6 presentation path and is the only place that calls
 * aga_ham6_begin.
 */
static volatile unsigned aga_ham6_takeover_pending = 0;

static inline void aga_ham6_request_takeover(void)
{
    __asm__ volatile("dmb ishst" ::: "memory");
    aga_ham6_takeover_pending = 1;
    __asm__ volatile("dmb ishst\n\tsev" ::: "memory");
}

static inline int aga_ham6_defer_startup(void)
{
    aga_ham6_takeover_pending = 0;
    __asm__ volatile("dmb ish" ::: "memory");
    aga_diag_log("[AGA] HAM6 armed: physical RGB deferred until WHDLoad takeover");
    return 0; /* preserves an existing if (aga_ham6_begin) startup block safely */
}

static inline void aga_ham6_service_takeover(void)
{
    if (!aga_ham6_takeover_pending)
        return;

    __asm__ volatile("dmb ish" ::: "memory");
    aga_ham6_takeover_pending = 0;
    __asm__ volatile("dmb ish" ::: "memory");

    if (aga_ham6_begin())
        aga_diag_log("[AGA] WHDLoad takeover: physical HAM6 RGB enabled");
    else
        aga_diag_log("[AGA] WHDLoad takeover: HAM6 begin FAILED");
}


extern void mmu_map(uintptr_t phys, uintptr_t virt, uintptr_t length, uint32_t attr_low, uint32_t attr_high);
extern void mmu_unmap(uintptr_t virt, uintptr_t length);
/* provided by the patched Emu68: set/get the AGA IPL byte in INTF */
extern void emu68_set_aga_ipl(int ipl);
extern int  emu68_get_real_ipl(void);
extern uint32_t emu68_get_pc(void);   /* 68k PC, for the trap ring */

/* ---- state ---------------------------------------------------------------- */
/* volatile: core 0 writes these, core 3 polls them in a spin loop. Without it
   gcc -O3 hoists the load out of `while (!aga_enabled) yield;` and core 3
   spins on a stale register for ever - chipset loop never starts, frames stays
   0, hook_ipl never fires, and since sandbox entry just killed the real
   chipset's interrupts, AmigaOS is left with no interrupt source and freezes.
   Every other cross-core flag here was already volatile; these two damn
   things, the ones that gate everything, were not. */
volatile int aga_enabled = 0;
int aga_enter_stop = 0;      /* aga.enter_stop=N on the cmdline */
int aga_loop_stop  = 0;      /* aga.loop_stop=N  on the cmdline */
int aga_leave_stop = 0;      /* aga.leave_stop=N on the cmdline */
int aga_use_mmu    = 0;      /* aga.mmu: the old MMU-remap path, kept for A/B */
static volatile uint32_t last_chip_wr;   /* overlay: last chip RAM address the 68k wrote */
volatile int aga_chip_redirect = 0;   /* no-MMU: serve redirect_lo..redirect_hi from chip_va in the abort handler */
static uint32_t redirect_lo, redirect_hi;   /* the window; the sandbox uses all of chip RAM */
static volatile int redirect_test_mirror = 0; /* FRF_CHIP_REDIRECT_SAFE_MIRROR_V0_2: live abort test keeps real Chip authoritative */
volatile int aga_sandbox_active = 0;
static volatile int core_idle = 1;    /* chipset loop parked (aga_enabled == 0) */
static aga_t *aga;
static uint8_t *chip_va;              /* ARM virtual address of the virtual chip RAM */
static uint32_t chip_size;   /* the virtual chip RAM: what the game gets, 2 MB */
static uint32_t chip_real;   /* the real chip RAM: what is copied in and out, probed at entry */
static volatile char aga_lock_v __attribute__((aligned(64)));
static volatile int frames_done;
static volatile int real_ipl_seen;

/* WFE needs a matching SEV or the core sleeps until some unrelated event
   shows up. A plain store-release signals fuck all, and __atomic_test_and_set
   finishes its exclusive sequence before the WFE, so there's no monitor left
   to wake it either.

   That hung the fucking machine the first time core 0 and core 3 fought over
   this lock - i.e. the moment the chipset goes live - and the abort handler
   runs with interrupts masked, so nothing was coming to save it.

   ps_bus_lock in ps_classic_protocol.c spins on YIELD, which has no event
   dependency at all. SEV now pairs with the WFE properly: it sets the event
   register on every core, so an unlock racing a WFE cannot be missed. */
/* How long core 3 waits for this lock, per frame, worst case. Core 0 holds it
   inside the abort handler and some of that is PiStorm bus traffic (INTREQR
   reads fold in the real chip, the mirrored input registers are real reads) -
   on a Pi 3 the bus stalled audio pushes for 5 ms (2026-09-14), and the same
   damn stalls show up through the lock. Fast path untouched: the clock is
   only read after the first try fails. */
static inline uint64_t cntvct(void);
static volatile uint64_t lock_acc3, lock_max3;
static inline void lock(void)
{
    if (!__atomic_test_and_set(&aga_lock_v, __ATOMIC_ACQUIRE)) return;
#ifdef AGA_PROBES
    uint64_t t0 = cntvct();
#endif
    __asm__ volatile("sevl");
    while (__atomic_test_and_set(&aga_lock_v, __ATOMIC_ACQUIRE))
        __asm__ volatile("wfe");
#ifdef AGA_PROBES
    uint64_t dt = cntvct() - t0, mpidr;
    __asm__ volatile("mrs %0, MPIDR_EL1" : "=r"(mpidr));
    if ((mpidr & 0xFF) == 3) { lock_acc3 += dt; if (dt > lock_max3) lock_max3 = dt; }
#endif
}
static inline void unlock(void)
{
    __atomic_clear(&aga_lock_v, __ATOMIC_RELEASE);
    __asm__ volatile("sev");        /* wake anyone parked in the WFE above */
}

/* ---- hooks ---------------------------------------------------------------- */
static uint16_t hook_ext_read(void *u, uint32_t reg)
{
    (void)u;
    /* input, pots, serial and disk status live in the real Paula/Denise */
    return (uint16_t)ps_read_16(0xDFF000 + reg);
}

/* Is a WHDLoad game running, or the Workbench?
 *
 * It decides one thing: the real Paula's EXTER (level 6).
 *
 * With the OS up EXTER has to stay on or the CIAs go silent - timer.device
 * stops, the network dies, waiting tasks never wake. Sandbox entry keeps it on
 * and INTREQR reads fold the real bit back in, and that is the only reason the
 * desktop survives the sandbox.
 *
 * A WHDLoad game is the other case: it takes the machine off the OS and
 * installs the GAME's vector table, and a game only fills the levels it uses.
 * ClockwiserAGA installs level 3 and nothing else - its autovector at BaseMem
 * $78 is zero. The real CIA-B keeps counting through all of it (TOD alarm
 * pending in the crash dump), the forced EXTER eventually fires, WHDLoad
 * dispatches level 6 through that zero vector, and the CPU jumps to address 0:
 *
 *   Exception "Line 1111 Emulator" ($2C) at $4 occurred.
 *   SR=%0010011000000000            <- interrupt mask 6
 *   $361E4D7C 2008 3613D108 0078    <- stacked frame, vector offset $078
 *
 * So I was killing the game with an interrupt it never fucking asked for.
 * Game mode leaves EXTER exactly as WHDLoad set it. PORTS (level 2) stays -
 * WHDLoad handles the keyboard itself and the quit key needs it.
 *
 * WHEN it engages matters as much as what it does. `agaboot SANDBOX AUTO` runs
 * from WHDLoad's startup script, so the sandbox opens while AmigaOS is still
 * up - WHDLoad then loads the slave, the game and (KickEmu titles) a whole
 * Kickstart through the OS filesystem. Taking EXTER away at entry starves the
 * OS's CIAs during exactly that window: timer.device stops and the load never
 * finishes. Banshee froze on startup that way; the far smaller ClockwiserAGA
 * happened to get through.
 *
 * Game mode is only ARMED at entry. It engages on the one unambiguous sign
 * that WHDLoad has taken the machine away from the OS: an INTENA write that
 * CLEARS EXTER (bit 15 low with bit 13 set) - WHDLoad's "all interrupts off"
 * as it shuts the OS down. From then on EXTER never goes back on. Before that,
 * everything behaves as it does for the desktop. */
/* Which audio channels reach the mix. Set with $5D0N at $DFF1F2 so a channel
   can be isolated on a RUNNING game without a rebuild - the only way left to
   tell "the effects are not in the output" from "the effects are in the output
   and something is masking them", now that the counters say channel 2 makes as
   much noise as the music channels. */
static volatile int aud_mask_want = 15;
void aga_audio_mask(unsigned m)
{
    /* Kept HERE and re-applied every frame, not written once into the core.
       aga_reset() puts the mask back to "all four" and the sandbox resets on
       leave, so a mask set from Workbench before launching a game would be
       gone by the time the game made a sound - exactly when it's needed.
       There's no shell during a WHDLoad game to set the bloody thing again. */
    aud_mask_want = (int)(m & 15);
    aga_diag_log("[AGA] audio channel mask changed");
}

int aga_game_mode = 0;
int aga_program_mode = 0;
int aga_exter_locked = 0;

/* FRF_AGA_PROGRAM_SCOPE_V0_1
 * RUN sessions virtualise AGA before the child starts, but keep the real RGB
 * display untouched.  Once the child has installed a different COP1LC and it
 * has produced two useful frames, CPU1 is released to take physical HAM6.
 * This avoids the visible Workbench takeover caused by SYSTEMWIDE ON. */
static volatile uint32_t aga_program_seed_cop1lc;
static volatile uint32_t aga_program_candidate_cop1lc;
static volatile unsigned aga_program_candidate_frames;
static volatile unsigned aga_program_handover_done;

void aga_program_prepare(void)
{
    aga_program_mode = 1;
    aga_program_seed_cop1lc = 0;
    aga_program_candidate_cop1lc = 0;
    aga_program_candidate_frames = 0;
    aga_program_handover_done = 0;
    aga_ham6_defer_start(1);
    aga_diag_log("[AGA] PROGRAM: armed; physical HAM6 deferred");
}

void aga_program_cancel(void)
{
    aga_program_mode = 0;
    aga_program_seed_cop1lc = 0;
    aga_program_candidate_cop1lc = 0;
    aga_program_candidate_frames = 0;
    aga_program_handover_done = 0;
    aga_ham6_defer_start(0);
}

static void aga_program_frame_probe(void)
{
    if (!aga_program_mode || aga_program_handover_done ||
        !aga_program_seed_cop1lc || aga_ham6_active())
        return;

    uint32_t cop = aga_peek_cop1lc(aga);
    int lines = aga_last_frame_display_lines(aga);

    /* Ignore the seeded Workbench/graphics.library list and transient writes.
       Require two completed display frames owned by something other than the
       seeded OS list. Do not require the same COP1LC twice: a well-behaved
       double-buffered program may alternate two Copper lists every frame. */
    if (cop < 0x400u || cop == aga_program_seed_cop1lc || lines < 16) {
        aga_program_candidate_cop1lc = 0;
        aga_program_candidate_frames = 0;
        return;
    }

    aga_program_candidate_cop1lc = cop;
    if (++aga_program_candidate_frames < 2)
        return;

    aga_program_handover_done = 1;
    aga_ham6_defer_start(0);  /* CPU1 now owns the physical HAM Copper. */
    kprintf("[AGA] PROGRAM: target display stable COP1LC=%08x lines=%d; physical HAM6 enabled\n",
            (unsigned)cop, lines);
    aga_diag_log("[AGA] PROGRAM: target display stable; physical HAM6 released");
}

/* FRF_AGA_JOTD_VBLANK_IRQ_V0_2
 * One-shot proof points for the JOTD black-playfield failure. Reset when
 * WHDLoad actually takes ownership, so Workbench startup traffic cannot
 * consume them before the game's own level-3 path begins. */
static volatile unsigned frf_irq_seen_publish3;
static volatile unsigned frf_irq_seen_drop3;
static volatile unsigned frf_irq_seen_intreq_vertb;
static volatile unsigned frf_irq_seen_ack_vertb;
/* core 0's time inside chip RAM data aborts, accumulated in vectors.c;
   published per frame below as abort_us_frame / aborts_frame */
volatile uint64_t aga_abort_ticks, aga_abort_count;
/* FRF_AGA_STRICT_TIMING_V0_1_2: stale BPLCON3.LOCT is handled by the
   core-side game-takeover ownership guard, not by polling for Copper-off. */

/* Which chipset the next game gets. Set with $5E00 (AGA) or $5E01 (ECS) at
   $DFF1F2 before the sandbox is entered, from agaboot's ECS toggle.
 *
   Not chosen automatically, and the WHDLoad slave can't choose either: its
   only chipset declaration is ws_Flags bit 5 (WHDLF_ReqAGA), which two of the
   seven AGA titles I have leave clear (CastlevaniaAGA, GhostsNGoblinsAGADemo).
   Guessing ECS for an AGA game breaks it visibly, so the choice stays the
   player's until I find the real cause.
 *
   Unlike the audio mask above this IS written into the core, which keeps it
   across aga_reset() for exactly this reason: entering the sandbox resets the
   chipset, so the mode has to be chosen before entering. Sandbox leave puts it
   back to AGA, so a game that says nothing gets what the desktop gets. */
void aga_chipset_ecs(int on)
{
    /* FRF_HAM6_WHD_REARM_DIAG_V0_6_SHELL */
    extern int aga_ham6_diag_control_bit(int bit);
    if (aga_ham6_diag_control_bit(on))
        return;

    if (!aga) return;
    lock();
    aga_set_ecs(aga, on);
    /* FRF_AGA_STRICT_TIMING_V0_1_2:
       Reuse the already-proven $5E00/$5E01 personality control.
       AGA ($5E00 -> on=0) gets STRICT timing; ECS ($5E01 -> on=1)
       keeps the FAST compatibility path. No new abort-handler control word. */
    aga_set_timing_strict(aga, on ? 0 : 1);
    unlock();
    aga_diag_log(on ? "[AGA] chipset: ECS Denise; timing: FAST"
                    : "[AGA] chipset: AGA; timing: STRICT");
    kprintf("[AGA] personality: %s / timing: %s\n",
            on ? "ECS" : "AGA", on ? "FAST" : "STRICT");
}


/* FRF_HAM6_WHD_REARM_DIAG_V0_7_HOOKTRACE
 * Capture the exact raw hook_ext_write traffic surrounding WHDLoad takeover.
 */
#define FRF_HDIAG7_RING 16u
static volatile uint32_t g_hdiag7_hook_calls;
static volatile uint32_t g_hdiag7_first_reg[FRF_HDIAG7_RING];
static volatile uint32_t g_hdiag7_first_val[FRF_HDIAG7_RING];
static volatile uint32_t g_hdiag7_last_reg[FRF_HDIAG7_RING];
static volatile uint32_t g_hdiag7_last_val[FRF_HDIAG7_RING];
static volatile uint32_t g_hdiag7_last_head;

uint32_t aga_hdiag7_hook_calls(void)
{
    return g_hdiag7_hook_calls;
}

uint32_t aga_hdiag7_trace_count(void)
{
    uint32_t n=g_hdiag7_hook_calls;
    return n < FRF_HDIAG7_RING ? n : FRF_HDIAG7_RING;
}

void aga_hdiag7_get_first(uint32_t idx, uint32_t *reg, uint32_t *val)
{
    if (idx >= aga_hdiag7_trace_count()) {
        *reg=0xffffffffu; *val=0xffffffffu; return;
    }
    *reg=g_hdiag7_first_reg[idx];
    *val=g_hdiag7_first_val[idx];
}

void aga_hdiag7_get_last(uint32_t idx, uint32_t *reg, uint32_t *val)
{
    uint32_t n=aga_hdiag7_trace_count();
    uint32_t pos;
    if (idx >= n) {
        *reg=0xffffffffu; *val=0xffffffffu; return;
    }
    if (g_hdiag7_hook_calls <= FRF_HDIAG7_RING)
        pos=idx;
    else
        pos=(g_hdiag7_last_head + idx) & (FRF_HDIAG7_RING-1u);
    *reg=g_hdiag7_last_reg[pos];
    *val=g_hdiag7_last_val[pos];
}

void aga_hdiag7_reset_hook(void)
{
    uint32_t i;
    g_hdiag7_hook_calls=0u;
    g_hdiag7_last_head=0u;
    for (i=0u;i<FRF_HDIAG7_RING;++i) {
        g_hdiag7_first_reg[i]=0u;
        g_hdiag7_first_val[i]=0u;
        g_hdiag7_last_reg[i]=0u;
        g_hdiag7_last_val[i]=0u;
    }
}


/* FRF_HAM6_WHD_REARM_FIX_V0_8_INTENA_HANDOFF
 * Hardware-observed fallback takeover sequence:
 * INTENA A02C -> C000 -> 4000. One-shot only.
 */
static unsigned frf_whd8_seen_exter_setup;
static unsigned frf_whd8_seen_master_enable;
static unsigned frf_whd8_fired;

/* FRF_HAM6_WHD_REARM_FIX_V0_9_PER_SESSION
 *
 * V0.8 takeover recognition was accidentally one-shot for the lifetime
 * of Emu68. Reset it at each real sandbox entry.
 */
static void frf_whd9_reset_takeover_state(void)
{
    frf_whd8_seen_exter_setup = 0u;
    frf_whd8_seen_master_enable = 0u;
    frf_whd8_fired = 0u;
    aga_exter_locked = 0;
}
/* FRF_V1_105_EXTER_STORM_GUARD_V0_1_6
 * v1.105 Shadow Fighter fix: a level-6 handler that clears Paula EXTER
 * but never reads CIA-B ICR can leave CIA-B /INT asserted forever.
 * 1000 EXTER acknowledgements in one emulated frame is treated as the
 * pathological case; read ICR, disable only the abandoned sources,
 * then drop the Paula EXTER latch once more.
 */
static void frf_v1_105_exter_storm_check(void)
{
    static uint32_t frame, n;
    uint32_t f = aga_diag_ctr[AGA_D_FRAMES];
    if (f != frame) { frame = f; n = 0; }
    if (++n != 1000) return;
    uint8_t icr = (uint8_t)ps_read_8(0xBFDD00);
    ps_write_8(0xBFDD00, icr & 0x1F);
    ps_write_16(0xDFF09C, INTF_EXTER);
    aga_diag_log("[AGA] v1.105: cleared pathological CIA-B/EXTER interrupt storm");
}




/* FRF_AGA_SYSTEMWIDE_HAM6_OS_HANDOVER_V0_1_1 */
int aga_systemwide_seed_os_copper(uint32_t cop1lc)
{
    if (!aga || !aga_sandbox_active || cop1lc < 0x400u) return 0;
    lock();
    aga_seed_os_copper(aga, cop1lc);
    unlock();
    if (aga_program_mode) {
        aga_program_seed_cop1lc = cop1lc;
        aga_program_candidate_cop1lc = 0;
        aga_program_candidate_frames = 0;
        kprintf("[AGA] PROGRAM: virtual OS Copper seeded from %08x; waiting for child display\n",
                (unsigned)cop1lc);
    } else {
        kprintf("[AGA] SYSTEMWIDE: virtual OS Copper seeded from %08x; physical HAM Copper untouched\n",
                (unsigned)cop1lc);
    }
    return 1;
}

static void hook_ext_write(void *u, uint32_t reg, uint16_t v)
{
    /* FRF_HAM6_WHD_REARM_FIX_V0_8_INTENA_HANDOFF */
    if (!frf_whd8_fired && !aga_exter_locked && reg == 0x009Au) {
        if ((v & 0xA000u) == 0xA000u)
            frf_whd8_seen_exter_setup = 1u;
        if (frf_whd8_seen_exter_setup && (v & 0xC000u) == 0xC000u)
            frf_whd8_seen_master_enable = 1u;
        if (frf_whd8_seen_exter_setup && frf_whd8_seen_master_enable &&
            (v & 0xC000u) == 0x4000u) {
            extern void aga_ham6_request_whd_rearm(void);
            extern void aga_ham6_diag_takeover(void);
            frf_whd8_fired = 1u;
            ps_write_8(0xBFDD00, 0x7F);
            (void)ps_read_8(0xBFDD00);
            aga_exter_locked = 1;
            aga_prepare_game_takeover(aga);
            aga_ham6_diag_takeover();
            aga_ham6_request_whd_rearm();
            aga_diag_log("[AGA] WHDLoad fallback takeover: INTENA A02C/C000/4000; HAM6 rearm requested");
            kprintf("[HAMFIX8] takeover fallback fired; HAM6 rearm requested\n");
        }
    }

    {
        uint32_t _hc = g_hdiag7_hook_calls++;
        uint32_t _p;
        if (_hc < FRF_HDIAG7_RING) {
            g_hdiag7_first_reg[_hc]=(uint32_t)reg;
            g_hdiag7_first_val[_hc]=(uint32_t)v;
        }
        _p=g_hdiag7_last_head;
        g_hdiag7_last_reg[_p]=(uint32_t)reg;
        g_hdiag7_last_val[_p]=(uint32_t)v;
        g_hdiag7_last_head=(_p+1u) & (FRF_HDIAG7_RING-1u);
    }

    (void)u;
    if (aga_audio_reg_write(reg, v)) return;     /* AUDx*, DMACON audio bits -> real Paula */
    switch (reg) {
    case INTREQ:
        /* clearing PORTS/EXTER in the virtual INTREQ: also drop the real line */
        ps_write_16(0xDFF09C, v & (INTF_PORTS | INTF_EXTER));
        if (v & INTF_EXTER) frf_v1_105_exter_storm_check();
        break;
    case INTENA: {
        /* mirror only the real-hardware sources so the real Paula asserts IPL
           for the CIAs. A CLEAR of EXTER is always forwarded - that's how the
           real line gets switched off. A SET of it is dropped once the lock is
           on, so nothing re-arms the level 6 the game has no handler for. */
        int is_set = (v & 0x8000) != 0;
        ps_write_16(0xDFF09A, (v & 0x8000) | (v & (INTF_INTEN | INTF_PORTS | INTF_EXTER)));
        if (aga_game_mode && !aga_exter_locked && !is_set && (v & INTF_EXTER)) {
            /* WHDLoad has just switched every interrupt off: the OS is gone
               and the machine is the game's now. Silence the CIA-B interrupt
               sources AmigaOS left running.
               Without this they hold the bloody CIA /INT line asserted, so
               Paula re-latches EXTER the instant the handler clears it and the
               68k drowns in level 6, which outranks the keyboard (2) and the
               game's own vertical blank (3). That's what Banshee did: its AGA
               title screen rendered perfectly and then did fuck all, WHDLoad's
               F10 quit key dead, INTREQ=$2000 the last thing the CPU ever
               wrote.
               Killing the sources instead of Paula's EXTER enable serves both
               kinds of game. A plain slave installs no level 6 handler and
               never sees one. A KickEmu title boots a whole Kickstart whose
               timer.device runs on these very timers, free to enable exactly
               the ones it wants - suppressing EXTER at Paula denied it that,
               leaving every task that waits on a timer waiting for ever. */
            ps_write_8(0xBFDD00, 0x7F);        /* CIA-B ICR: clear every enable */
            (void)ps_read_8(0xBFDD00);         /* and the read drops a pending line */
            aga_exter_locked = 1;
            extern void aga_ham6_diag_takeover(void);
            aga_ham6_diag_takeover(); /* FRF_HAM6_WHD_REARM_DIAG_V0_6_SHELL */

            kprintf("[HAMDIAG:TAKEOVER] reg=%04x value=%04x\n", (unsigned)reg, (unsigned)v);
            /* FRF_AGA_JOTD_VBLANK_IRQ_V0_2: begin proof at game ownership, not Workbench. */
            frf_irq_seen_publish3 = 0;
            frf_irq_seen_drop3 = 0;
            frf_irq_seen_intreq_vertb = 0;
            frf_irq_seen_ack_vertb = 0;
            /* Arm the core-side one-shot. Unlike the retired copper-off poll,
               this survives an OS copper list that remains enabled until the
               game installs its own list: the stale LOCT bit is consumed at
               the first authoritative CPU palette/copper action. */
            aga_prepare_game_takeover(aga);

            /* FRF V0.3: CPU3 performs the physical HAM6 handover. */
            aga_ham6_request_takeover();
            aga_diag_log("[AGA] WHDLoad took the machine: stale CIA-B sources silenced; palette guard armed");
        }
        break;
    }
    case ADKCON:
        ps_write_16(0xDFF09E, v);      /* UART/disk bits are real */
        break;
    case SERDAT:
        /* Every character the 68k transmits. WHDLoad's crash handler dumps
           its register state down this line and then waits for a keypress
           from a terminal that isn't fucking there - which is the state a
           hung game leaves the machine in. Recording the bytes is how I read
           it. */
        aga_diag_serial((uint8_t)(v & 0xFF));
        ps_write_16(0xDFF030, v);
        break;
    default:
        ps_write_16(0xDFF000 + reg, v);   /* SERDAT, SERPER, POTGO, JOYTEST, DSK* */
        break;
    }
}

static void hook_ipl(void *u, int ipl)
{
    (void)u;
    if (ipl) aga_diag_inc(AGA_D_IPL_HOOK);
    if (aga_exter_locked && ipl == 3 && !frf_irq_seen_publish3) {
        frf_irq_seen_publish3 = 1;
        aga_diag_log("[AGA] JOTD IRQ: virtual level 3 published to Emu68");
    }
    if (aga_exter_locked && ipl != 3 && frf_irq_seen_publish3 && !frf_irq_seen_drop3) {
        frf_irq_seen_drop3 = 1;
        aga_diag_log("[AGA] JOTD IRQ: virtual level 3 lowered after acknowledge");
    }
    emu68_set_aga_ipl(ipl);
}

/* Beam interpolation base, published by the chipset thread after every step
   so the trap path can work out where the beam is NOW without the lock. */
static volatile uint64_t beam_tick;          /* cntvct() at the last step */
static volatile uint64_t ticks_per_cck;      /* whole ticks; 0 until the loop runs */

static volatile uint32_t last_trap_pc;   /* 68k PC at the last register write */
/* The PC is stable and the write count climbs steadily, so the 68k is looping
   over the chipset - but BPLCON0 and DMACON never change, so it's hitting
   something else. These say what the hell. */
static volatile uint32_t last_wr;        /* reg << 16 | value, last write */
static volatile uint32_t last_rd;        /* reg << 16 | value, last read  */

static volatile int frame_pending;

static void hook_frame(void *u)
{
    (void)u;
    frames_done++;
    frame_pending = 1;          /* presented outside the lock by the chipset loop */
}

/* ---- init ----------------------------------------------------------------- */
static int aga_is_ntsc;      /* what it was BUILT for; regs[] does not know */

int aga_glue_init(uint8_t *chipram_va, uint32_t size, uint8_t *fb_va, uintptr_t fb_phys, int ntsc, int enable)
{
    aga_is_ntsc = ntsc;
    aga_config_t cfg = { .ntsc = ntsc, .blit_immediate = 0,   /* a blit must not finish inside its own BLTSIZE write */ .collisions = 0, .render = 1 };
    chip_va = chipram_va;
    chip_size = size;
    aga = aga_create(chip_va, size, &cfg);
    if (!aga) return -1;
    aga_hooks_t h = { hook_ext_read, hook_ext_write, hook_ipl, hook_frame, NULL };
    aga_set_hooks(aga, &h);
    aga_lock_v = 0;
    aga_audio_init(chip_va, size);
    /* framebuffer is allocated and mapped here on core 0, before the 68k starts */
    if (aga_video_init_at(fb_va, fb_phys) != 0) {
        /* No display on this board (a Pi 3): then don't render either. The
           renderer streams a 900 KB frame through the shared L2 fifty times a
           second for fucking nobody, and on the A53 cluster that's the working
           set the game's own chip RAM traffic evicts: PMU counted 108k L2
           refills in one frame against 2k in a quiet one (2026-09-14). */
        aga_set_render(aga, 0);
        kprintf("[AGA] continuing without video output, renderer off\n");
    } else {
        /* the core renders into the HVS pages themselves - one line at a
           time, through its line buffer; present() then only points the HVS
           at it */
        aga_set_framebuffers(aga, (uint32_t *)fb_va, (uint32_t *)(fb_va + (size_t)AGA_OUT_W * AGA_OUT_H * 4));
    }
    aga_diag_init();
    aga_enabled = enable;
    kprintf("[AGA] chipset core ready, %d KB chip RAM at %p, %s\n", size >> 10, chip_va,
            enable ? "machine mode" : "parked (sandbox available)");
    return 0;
}

/* ---- diagnostics: let aga_diag.c see the virtual chipset ------------------
   Read without the lock. Single words the chipset thread writes; a torn value
   would at worst mislead one dump, and taking the mutex from the diag read
   path would just serialise against the chipset for nothing. */
uint16_t aga_glue_peek_reg(uint32_t reg)   { return aga ? aga_peek_reg(aga, reg) : 0; }
uint32_t aga_glue_peek_bplpt(int plane)    { return aga ? aga_peek_bplpt(aga, plane) : 0; }
uint16_t aga_glue_peek_state(int which)
{
    if (!aga) return 0;
    /* the accumulated state, not the last value written - see the note in
       aga_sandbox_leave on why those differ for the set/clear registers */
    return aga_custom_rget(aga, which == 0 ? DMACONR : (which == 1 ? INTENAR : INTREQR));
}

/* ---- sandbox mode --------------------------------------------------------- */
/* Registers replayed to the real (ECS) chips on sandbox leave: the last values
   written to the virtual chipset (copper included). */
static const uint16_t replay_regs[] = {
    0x100, 0x102, 0x104, 0x106,                 /* BPLCON0-3 */
    0x08E, 0x090, 0x1E4, 0x092, 0x094,          /* DIWSTRT/STOP, DIWHIGH, DDFSTRT/STOP */
    0x108, 0x10A,                               /* BPL1MOD/2MOD */
    0x0E0, 0x0E2, 0x0E4, 0x0E6, 0x0E8, 0x0EA, 0x0EC, 0x0EE, 0x0F0, 0x0F2, 0x0F4, 0x0F6, /* BPL1-6PT */
    0x120, 0x122, 0x124, 0x126, 0x128, 0x12A, 0x12C, 0x12E,                          /* SPR0-3PT */
    0x130, 0x132, 0x134, 0x136, 0x138, 0x13A, 0x13C, 0x13E,                          /* SPR4-7PT */
    0x0A0, 0x0A2, 0x0A4, 0x0A6, 0x0A8, 0x0B0, 0x0B2, 0x0B4, 0x0B6, 0x0B8,            /* AUD0/1 */
    0x0C0, 0x0C2, 0x0C4, 0x0C6, 0x0C8, 0x0D0, 0x0D2, 0x0D4, 0x0D6, 0x0D8,            /* AUD2/3 */
    0x080, 0x082, 0x084, 0x086,                 /* COP1LC, COP2LC */
};

static inline uint32_t rd_be32(const uint8_t *p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }
static inline void wr_be32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

/* Hand the real chipset its DMA and interrupts back. Only the aga.enter_stop
   bring-up path uses this, so a partial entry leaves a working machine
   instead of a deaf one. */
/* defined below, used by the entry handshake above them */
static inline uint64_t cntvct(void);
static inline uint64_t cntfrq(void);

static void restore_real(uint16_t dmacon, uint16_t intena, uint16_t adkcon)
{
    ps_write_16(0xDFF09E, 0x8000 | (adkcon & 0x7FFF));
    ps_write_16(0xDFF096, 0x8000 | (dmacon & 0x03FF));
    ps_write_16(0xDFF09A, 0x8000 | (intena & 0x7FFF));
    aga_diag_log("[AGA] enter: stopped early, real chipset restored");
}

/* How much real chip RAM the machine has, probed on the bus with the 68k
   stalled and the real chipset quiet. The virtual chip RAM is always 2 MB -
   the game never runs in the real one - so this only decides how much gets
   copied in at entry and back out at leave. Copying 2 MB out of a 1 MB
   machine reads the Agnus mirror, and copying it back would put the upper
   half over the OS. Two things separate real RAM from the alternatives: a
   pattern written at the top must survive a read of another address (an open
   bus hands back whatever was on it last), and must not show up at the same
   offset half a megabyte down (a mirror). Everything written gets put back. */
static uint32_t probe_real_chip(void)
{
    static const uint32_t sizes[] = { 2u << 20, 1u << 20 };
    for (unsigned i = 0; i < 2; i++) {
        uint32_t s = sizes[i], hi = s - 16, lo = s / 2 - 16;
        uint32_t save_hi = ps_read_32(hi), save_lo = ps_read_32(lo);
        uint32_t pat = save_hi ^ 0x5A5AA5A5u;
        ps_write_32(hi, pat);
        (void)ps_read_32(lo);                    /* disturb the bus before reading back */
        int held = ps_read_32(hi) == pat;
        int mirror = ps_read_32(lo) == pat;
        ps_write_32(hi, save_hi);
        ps_write_32(lo, save_lo);
        if (held && !mirror) return s;
    }
    return 512u << 10;
}

int aga_sandbox_enter(void)
{
    if (!aga || aga_enabled) return 0;

    /* FRF_CHIP_REDIRECT_SAFE_MIRROR_V0_2: never leak a manual live REDIRECT
       test into a real sandbox. No copy-back is needed because live writes
       were mirrored to physical Chip RAM as they occurred. */
    if (redirect_test_mirror) {
        aga_chip_redirect = 0;
        redirect_test_mirror = 0;
        redirect_lo = redirect_hi = 0;
        __asm__ volatile("dmb ish" ::: "memory");
        cache_invalidate_all(DCACHE);
        cache_invalidate_all(ICACHE);
        aga_diag_log("[AGA] sandbox entry: cancelled live redirect mirror test");
    }
    aga_diag_log("[AGA] enter 1: begin");
    /* Bring-up: stop after step N and return, machine untouched. Recovery
       from a failed entry needs a power cut, which resets the Pi and wipes
       the diag log and counters - nothing recorded in here survives to be
       read. The only observable left is whether the machine is still alive,
       so make that the signal: raise the stop point one step at a time and
       see which step kills the bastard.
       Steps 1-2 touch nothing, so the machine MUST survive them; if it
       doesn't, the fault is in the trap dispatch or the bus reads, before any
       of the chipset logic runs. */
    if (aga_enter_stop && aga_enter_stop < 2) return 0;
    kprintf("[AGA] sandbox: entering\n");
    /* 1. readable register state of the real chips, BEFORE anything is changed */
    uint16_t dmacon = ps_read_16(0xDFF002), adkcon = ps_read_16(0xDFF010);
    uint16_t intena = ps_read_16(0xDFF01C), intreq = ps_read_16(0xDFF01E);
    (void)intreq; /* FRF_AGA_JOTD_VBLANK_IRQ_V0_2: preserve snapshot read; value intentionally unused. */
    aga_diag_log("[AGA] enter 2: read real registers");
    if (aga_enter_stop && aga_enter_stop <= 2) return 0;

    /* 2. quiet the real chips FIRST: no display/copper/blitter/sprite/disk/
          audio DMA, no interrupts (PORTS/EXTER come back through the INTENA
          mirror below).

          This used to happen *after* the chip RAM copy below, and that is
          what made sandbox entry look like a hang: the copy is 512K of bus
          reads, and with Agnus still on the bus the PiStorm only gets the
          slots the chipset leaves over, so an already slow operation
          crawls like shit. It all runs in the 68k abort handler with
          interrupts masked - frozen long enough that nobody can tell it from
          a dead machine. */
    ps_write_16(0xDFF096, 0x01FF);
    /* Clear every interrupt enable EXCEPT PORTS (bit 3) and EXTER (bit 13).
       The chipset loop mirrors CIA interrupts into the virtual INTREQ by
       watching emu68_get_real_ipl(), but the real Paula only raises an IPL for
       sources INTENA still enables - clear them all and the mirror has
       nothing to mirror. timer.device and the CIAs go silent: task timeouts
       stop, the network stack dies, and the vertical blank keeps running.
       Exactly the failure I saw: the pointer still moves and fuck all else
       works.

       hook_ext_write (above) already forwards PORTS/EXTER to the real chipset,
       so keeping them enabled here is what the rest of the design assumes. */
    aga_exter_locked = 0;      /* armed, not engaged: the OS still needs its CIAs here */
    aga_diag_wave_freeze(0);   /* a fresh recording for this game */
    aga_diag_env_freeze(0);
    ps_write_16(0xDFF09A, 0x7FFF & ~(uint16_t)(INTF_PORTS | INTF_EXTER));
    ps_write_16(0xDFF09C, 0x7FFF);
    aga_diag_log("[AGA] enter 3: real chipset quiet");
    /* From here the real chipset is deaf: step 3 cleared DMACON and INTENA
       and AmigaOS can't survive without interrupts. A stop at step 3 or
       beyond MUST hand the chipset back before returning, or the machine
       freezes for reasons that have nothing to do with the step under test -
       which is how the earlier bisects fooled me. */
    if (aga_enter_stop && aga_enter_stop <= 3) { restore_real(dmacon, intena, adkcon); return 0; }

    /* 3. now copy the OS state out of real chip RAM, bus to myself. Drop
          Emu68's software I/D caches first so no stale line is served once
          the redirect is on. NOT flushed from here: cache_flush_all writes
          dirty lines back with plain stores to 68k addresses, and a plain
          store to a bus address from INSIDE the abort handler is a nested
          abort. agaboot does CacheClearU() before it sends the magic word,
          so the 68k has already flushed in its own context. */
    cache_invalidate_all(DCACHE);
    cache_invalidate_all(ICACHE);
    chip_real = probe_real_chip();
    if (chip_real > chip_size) chip_real = chip_size;
    aga_diag_set(AGA_D_CHIP_REAL_KB, chip_real >> 10);
    kprintf("[AGA] sandbox: %d KB real chip RAM, %d KB virtual\n", chip_real >> 10, chip_size >> 10);
    for (uint32_t a = 0; a < chip_real; a += 4)
        wr_be32(chip_va + a, ps_read_32(a));
    for (uint32_t a = chip_real; a < chip_size; a += 4)   /* the part the machine lacks: lent to the OS by agaboot, empty */
        wr_be32(chip_va + a, 0);
    aga_diag_log("[AGA] enter 4: 2MB chip RAM copied");
    if (aga_enter_stop && aga_enter_stop <= 4) { restore_real(dmacon, intena, adkcon); return 0; }
    /* 4. seed the virtual chipset with that state.
     *
     * DMACON is copied MINUS the channels whose source registers can't be
     * recovered. COP1LC, the bitplane pointers, the sprite pointers, BPLCON0
     * and DIWSTRT are all WRITE-ONLY on the real chipset: step 1 can read
     * DMACON, ADKCON, INTENA and INTREQ, nothing else. A faithful DMACON copy
     * then says "copper, bitplane and sprite DMA are all running" while every
     * pointer they need is zero.
     *
     * The copper is the one that fucks everything. Enabled with COP1LC = 0
     * it fetches chip RAM address 0 - the 68k exception vectors - and
     * executes them as a copper list: arbitrary MOVEs to arbitrary
     * registers, fifty times a second. It flattens BPLCON0 and the bitplane
     * pointers as fast as a game can set them, which is why EVERY WHDLoad
     * game showed a grey screen under the sandbox, ECS and AGA alike, while
     * `agastat REGS` reported BPLCON0 $0000 and all eight BPLxPT $00000000.
     *
     * Nothing is lost by starting them off. AmigaOS is on the RTG screen here
     * and isn't using the native display, and a WHDLoad game programs its own
     * copper list, bitplane pointers and DMACON before it draws anything.
     */
#define DMAF_UNRECOVERABLE (DMAF_AUDEN | DMAF_SPREN | DMAF_COPEN | DMAF_BPLEN)
    lock();
    aga_reset(aga);
    aga_custom_wput(aga, 0x09E, 0x8000 | (adkcon & 0x7FFF));
    aga_custom_wput(aga, 0x096, 0x8000 | (dmacon & 0x03FF & ~DMAF_UNRECOVERABLE));
    /* INTREQ: seed NOTHING pending.
     *
     * Same reasoning as DMAF_UNRECOVERABLE above. The real INTREQ's bits
     * belong to the OS's drivers - audio.device, the serial and disk handlers
     * - which are about to be replaced by a game that knows nothing about
     * them. Copy them in and flags stay set that nothing will ever clear:
     * measured on hardware, INTREQ read $0780 (AUD0-3 pending) for the entire
     * life of a sandboxed game, because the game clears only VERTB. WinUAE at
     * the same point in the same game reads $0040. A game that polls INTREQR
     * - Banshee reads it five times a frame - is being shown shit that could
     * not be set on a real machine. */
    aga_custom_wput(aga, 0x09C, 0x7FFF);            /* clear every pending bit */
    aga_custom_wput(aga, 0x09A, 0x8000 | (intena & 0x7FFF));
    unlock();
    aga_diag_log("[AGA] enter 5: virtual chipset seeded");
    if (aga_enter_stop && aga_enter_stop <= 5) { restore_real(dmacon, intena, adkcon); return 0; }
    /* 5. 68k address 0 reads the virtual chip RAM now. Default is NO MMU: the
          abort handler serves it (aga_chip_read/write). aga.mmu keeps the old
          page-table remap for A/B. */
    if (aga_use_mmu) {
        mmu_map((uintptr_t)chip_va, 0x000000, chip_size, MMU_ACCESS | MMU_ISHARE | MMU_ALLOW_EL0 | MMU_ATTR_CACHED, 0);
        __asm__ volatile("dsb ish; isb" ::: "memory");
        aga_diag_log("[AGA] enter 6: chip RAM mapped at 0 (MMU)");
    } else {
        redirect_lo = 0;
        redirect_hi = chip_size;
        aga_chip_redirect = 1;
        __asm__ volatile("dmb ish" ::: "memory");
        aga_diag_log("[AGA] enter 6: chip RAM redirected in the abort handler (no MMU)");
    }
    if (aga_enter_stop && aga_enter_stop <= 6) {
        /* Undo the map before handing the chipset back. Leave it and the 68k
           points at virtual chip RAM while Agnus still DMAs from the real
           thing - the machine breaks for a reason that has nothing to do with
           whether mmu_map itself is safe. */
        if (aga_use_mmu) { mmu_unmap(0x000000, chip_size); __asm__ volatile("dsb ish; isb" ::: "memory"); }
        else             { aga_chip_redirect = 0; __asm__ volatile("dmb ish" ::: "memory"); }
        aga_diag_log("[AGA] enter 6b: chip RAM redirect undone again");
        restore_real(dmacon, intena, adkcon);
        return 0;
    }
    /* 6. go */
    aga_sandbox_active = 1;
    aga_enabled = 1;
    __asm__ volatile("sev");   /* wake core 3 */

    /* Don't commit until core 3 confirms it's running. From here the OS has
       no interrupts of its own - step 3 took them away - so if the chipset
       loop never starts, nothing delivers VERTB and the machine freezes with
       no way to report why. That's what made this fault so expensive to
       chase: it destroys the evidence and needs a power cut, which wipes the
       fucking log.

       core_idle goes to 0 only when core 3 leaves its WFE park, so poll it
       briefly. If it never clears the SEV/WFE handshake failed: roll
       everything back and return an error the caller can actually read. */
    {
        uint64_t f = cntfrq(), deadline = cntvct() + f / 5;   /* 200 ms */
        while (core_idle && (int64_t)(cntvct() - deadline) < 0)
            __asm__ volatile("sev; yield");
        if (core_idle) {
            aga_enabled = 0;
            aga_sandbox_active = 0;
            if (aga_use_mmu) { mmu_unmap(0x000000, chip_size); __asm__ volatile("dsb ish; isb" ::: "memory"); }
            else             { aga_chip_redirect = 0; __asm__ volatile("dmb ish" ::: "memory"); }
            restore_real(dmacon, intena, adkcon);
            aga_diag_log("[AGA] ABORT: core 3 never woke - rolled back");
            kprintf("[AGA] sandbox: core 3 did not start, rolled back\n");
            return 0;
        }
        aga_diag_log("[AGA] core 3 confirmed running");
    }
    aga_diag_inc(AGA_D_SANDBOX_IN);
    aga_diag_log("[AGA] sandbox entered");
    kprintf("[AGA] sandbox: active (DMACON %04x INTENA %04x)\n", dmacon, intena);
    return 1;
}

/* Undo everything sandbox entry did, from core 3's own context: chip RAM back
   onto the real bus, the registers replayed to the real chips. Split out of
   aga_sandbox_leave so the WATCHDOG PARK can run it too.

   It didn't, and that cost a whole evening of confusion: the park cleared
   aga_enabled and aga_sandbox_active - which sends the register traps back to
   the real chipset - but left aga_chip_redirect set, so the 68k kept reading
   and writing the ARM-side copy of chip RAM while Agnus fetched from the real
   one. The machine survived and the OS answered the network, but every
   display was grey and the next game launched into that half-state and greyed
   too, until someone got off their arse and power-cycled. A recovery path
   that leaves the machine subtly broken is worse than no recovery path at
   all. */
static void sandbox_restore(void)
{
    emu68_set_aga_ipl(0);
    aga_audio_stop();
    /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_RESTORE_SAFETY: stop real HAM DMA before chip RAM is restored. */
    aga_ham6_takeover_pending = 0; /* FRF_AGA_WHDLOAD_HAM6_HANDOVER_V0_3: cancel stale arm */
    __asm__ volatile("dmb ish" ::: "memory");
    aga_ham6_end();              /* descriptor first: CPU1 cannot race a restore */
    aga_program_cancel();
    if (aga_leave_stop == 1) { aga_diag_log("[AGA] leave 1: chipset parked"); return; }
    /* 2. 68k address 0 is the real chip RAM again.

       The barrier is not fucking optional. Every other mmu_map/mmu_unmap in
       this file is followed by `dsb ish; isb` - entry's map, both of entry's
       rollback unmaps. This one wasn't, so the 68k could keep running on a
       stale TLB entry pointing at virtual chip RAM that is about to stop
       being the machine's memory. */
    if (aga_use_mmu) {
        mmu_unmap(0x000000, chip_size);
        __asm__ volatile("dsb ish; isb" ::: "memory");
    } else {
        /* Drop both software caches - they hold virtual-era lines - then stop
           redirecting. No flush from in here (nested-abort hazard, see entry);
           agaboot CacheClearU()s before sending MAGIC_SANDBOX_OFF, so dirty
           lines already landed in chip_va through the redirect. */
        cache_invalidate_all(DCACHE);
        cache_invalidate_all(ICACHE);
        aga_chip_redirect = 0;
        __asm__ volatile("dmb ish" ::: "memory");
    }
    if (aga_leave_stop == 2) { aga_diag_log("[AGA] leave 2: chip RAM redirect undone"); return; }
    /* 3. copy the (restored) OS state back into the real chip RAM */
    for (uint32_t a = 0; a < chip_real; a += 4)          /* only what the machine really has */
        ps_write_32(a, rd_be32(chip_va + a));
    if (aga_leave_stop == 3) { aga_diag_log("[AGA] leave 3: chip RAM copied back"); return; }
    /* 4. replay the register state to the real chips */
    ps_write_16(0xDFF096, 0x03FF);                       /* all DMA off while programming */
    ps_write_16(0xDFF09A, 0x7FFF);
    for (unsigned i = 0; i < sizeof replay_regs / sizeof replay_regs[0]; i++)
        ps_write_16(0xDFF000 + replay_regs[i], aga_peek_reg(aga, replay_regs[i]));
    for (int c = 0; c < 32; c++) {
        uint32_t rgb = aga_peek_color(aga, c);
        ps_write_16(0xDFF180 + 2 * c, ((rgb >> 12) & 0xF00) | ((rgb >> 8) & 0x0F0) | ((rgb >> 4) & 0x00F));
    }
    if (aga_leave_stop == 4) { aga_diag_log("[AGA] leave 4: registers replayed"); return; }
    ps_write_16(0xDFF088, 0);                            /* COPJMP1: copper restarts from COP1LC */
    /* ADKCON, INTENA and DMACON are set/clear registers: bit 15 says whether
       the other bits are set or cleared, so the STATE is an accumulation, not
       the last value written.

       aga_peek_reg returns the last value WRITTEN - chipset.h says so plainly.
       I read the wrong one anyway, which meant that if the OS's last INTENA
       write was a clear of one source (bit 15 low, say $0010), leave first
       cleared everything with $7FFF and then wrote $8010, re-enabling exactly
       one interrupt and leaving every other one off. The machine came back
       with no timer and no CIA interrupts: network dead, waiting tasks never
       woken, a pulled-down menu stuck on screen - while the Pi carried on
       drawing the RTG desktop, so it looked alive. Same shit for DMACON and
       ADKCON.

       The accumulated state is what the R registers return, so read those. */
    uint16_t adk_now  = aga_custom_rget(aga, ADKCONR);
    uint16_t inte_now = aga_custom_rget(aga, INTENAR);
    uint16_t dmac_now = aga_custom_rget(aga, DMACONR);
    ps_write_16(0xDFF09E, 0x7FFF);
    ps_write_16(0xDFF09E, 0x8000 | (adk_now  & 0x7FFF));
    ps_write_16(0xDFF09A, 0x8000 | (inte_now & 0x7FFF));
    ps_write_16(0xDFF096, 0x8000 | (dmac_now & 0x07FF));   /* incl. DMAEN and BLTPRI */
    aga_sandbox_active = 0;
    aga_game_mode = 0;
    aga_exter_locked = 0;
    frf_whd9_reset_takeover_state();
    aga_diag_wave_freeze(1);   /* keep the game's last fifteen seconds of audio */
    aga_diag_env_freeze(1);

    /* Hand the NEXT game a clean chipset, not this one's leftovers.
     *
     * The virtual chipset is created once and lives as long as the Pi is
     * powered, so without this everything a quitting game leaves behind is
     * still there when the next one starts: bitplane and copper pointers into
     * memory that has since been freed, DMACON with planes and the blitter
     * still enabled, a copper list mid-execution, sprite pointers, audio
     * channels mid-sample, a blit still counted as running. The next game
     * inherits the whole shitpile before it writes a single register, and
     * anything that reads back a register it hasn't set yet gets the previous
     * game's answer.
     *
     * Chip RAM is re-copied from the real machine on the next entry, so only
     * the register state needs clearing. aga_reset() zeroes the lot and
     * recomputes the IPL; emu68_set_aga_ipl(0) after it makes sure the virtual
     * Paula isn't still asserting an interrupt level it can't justify, since
     * aga_update_ipl only calls the hook when the level CHANGES and the reset
     * makes it 0 by memset without ever passing through the hook. */
    lock();
    aga_reset(aga);
    aga_set_ecs(aga, 0);     /* next game chooses again; the desktop is AGA */
    unlock();
    emu68_set_aga_ipl(0);

    aga_video_hide();          /* or the last presented frame stays on top of the desktop */
    {   /* the SoC's own account of the game: temperature now, plus the sticky
           "throttled / frequency capped / under-voltage since boot" bits. One
           mailbox call, here, where the 68k is stalled in agaboot's trap and
           the RTG driver can't be mid-call. */
        uint32_t tmc = 0, thr = 0;
        extern int aga_soc_status(uint32_t *temp_mc, uint32_t *throttled);
        if (aga_soc_status(&tmc, &thr) == 0) {
            aga_diag_set(AGA_D_SOC_TEMP_MC, tmc);
            aga_diag_set(AGA_D_THROTTLED, thr);
        }
    }
    aga_diag_inc(AGA_D_SANDBOX_OUT);
    aga_diag_log("[AGA] sandbox left");
    kprintf("[AGA] sandbox: left (DMACON %04x INTENA %04x)\n", dmac_now, inte_now);
    return;
}

int aga_sandbox_leave(void)
{
    if (!aga || !aga_sandbox_active) return 0;
    kprintf("[AGA] sandbox: leaving\n");
    /* 1. park the chipset loop and the virtual interrupt line.

       This used to be `aga_enabled = 0; sev; while (!core_idle) yield;` - one
       SEV, then an unbounded spin. Entry doesn't do that on purpose: it
       repeats the SEV inside the wait and gives up after 200 ms, because a
       single SEV can be missed and core 0 is spinning here inside the 68k's
       abort handler with interrupts masked. If core 3 doesn't answer, the
       machine dies outright - no console, no counters, no network, power cut
       the only fucking way out. That's exactly what the first ever SANDBOX
       OFF did.

       So: bounded, SEV repeated. If core 3 won't park, abandon the leave and
       stay sandboxed, which is ugly but alive and reportable. */
    aga_enabled = 0;
    {
        uint64_t f = cntfrq(), deadline = cntvct() + f / 2;   /* 500 ms */
        while (!core_idle && (int64_t)(cntvct() - deadline) < 0)
            __asm__ volatile("sev; yield");
        if (!core_idle) {
            aga_enabled = 1;                 /* it never parked: put it back */
            aga_diag_log("[AGA] ABORT: core 3 would not park, still sandboxed");
            kprintf("[AGA] sandbox: core 3 would not park, leave abandoned\n");
            return 0;
        }
    }
    sandbox_restore();
    return 1;
}

/* ---- chip RAM redirection, no MMU (core 0, abort handler) -----------------
 *
 * The old design remapped 68k address 0 onto the virtual chip RAM with the
 * ARM MMU, so JIT-emitted loads and stores stopped faulting and hit Pi memory
 * directly. Emu68 was never written with its memory map changing under it,
 * and every access that did NOT go through a plain load - or went through one
 * the map didn't cover - was a question I could never quite close.
 *
 * This is the other way round: leave the page tables alone, let every chip
 * RAM access fault into the abort handler exactly as it does for the real
 * bus, and answer it from chip_va instead of the PiStorm. One door. Code and
 * data alike - the JIT's I-cache line fill is a plain 16-byte load from the
 * 68k address, so it comes through here too. And it's faster than the real
 * thing: an abort plus a memory copy beats a bus transaction.
 *
 * Emu68 is built big-endian, so a plain load from chip_va already yields what
 * the 68k expects, and the 128-bit split (value = first 8 bytes, value2 =
 * next 8) matches SYSReadValFromAddr's own convention for unmapped space.
 */
int aga_chip_read(uint64_t far, int size, uint64_t *value, uint64_t *value2)
{
    if (!aga_chip_redirect || far < redirect_lo || far + (uint64_t)size > redirect_hi) return 0;
    aga_diag_ctr[AGA_D_CHIP_R]++;

    const uint8_t *p = chip_va + far;

    /* FRF_CHIP_REDIRECT_SAFE_MIRROR_V0_2
     * REDIRECT 1/2 are live abort-handler tests. The old code returned the
     * isolated chip_va shadow while real Agnus/Paula/Copper continued DMA
     * from physical Chip RAM. CPU and chipset therefore saw different memory.
     * Keep the real bus authoritative during the live test and mirror each
     * observed value into chip_va. Real sandbox operation remains virtual-only.
     */
    if (redirect_test_mirror && !aga_sandbox_active) {
        switch (size) {
        case 1:
            *value = ps_read_8((unsigned)far);
            *(uint8_t *)(chip_va + far) = (uint8_t)*value;
            break;
        case 2:
            *value = ps_read_16((unsigned)far);
            *(uint16_t *)(chip_va + far) = (uint16_t)*value;
            break;
        case 4:
            *value = ps_read_32((unsigned)far);
            *(uint32_t *)(chip_va + far) = (uint32_t)*value;
            break;
        case 8:
            *value = ps_read_64((unsigned)far);
            *(uint64_t *)(chip_va + far) = *value;
            break;
        case 16:
            *value = ps_read_64((unsigned)far);
            *value2 = ps_read_64((unsigned)far + 8u);
            *(uint64_t *)(chip_va + far) = *value;
            *(uint64_t *)(chip_va + far + 8u) = *value2;
            break;
        default:
            return 0;
        }
        return 1;
    }

    switch (size) {
    case 1:  *value = *(const uint8_t  *)p; break;
    case 2:  *value = *(const uint16_t *)p; break;
    case 4:  *value = *(const uint32_t *)p; break;
    case 8:  *value = *(const uint64_t *)p; break;
    case 16: *value = *(const uint64_t *)p; *value2 = *(const uint64_t *)(p + 8); break;
    default: return 0;
    }
    return 1;
}

int aga_chip_write(uint64_t far, int size, uint64_t value, uint64_t value2)
{
    if (!aga_chip_redirect || far < redirect_lo || far + (uint64_t)size > redirect_hi) return 0;
#ifdef AGA_PROBES
    aga_diag_ctr[AGA_D_CHIP_W]++;
    aga_diag_ctr[AGA_D_CHIPW_B0 + (unsigned)(far >> 18 & 7)]++;
#endif
    last_chip_wr = (uint32_t)far;
    uint8_t *p = chip_va + far;

    /* FRF_CHIP_REDIRECT_SAFE_MIRROR_V0_2
     * Live-test writes update both copies immediately so real DMA and the CPU
     * can never diverge. REDIRECT 0 therefore needs no destructive copy-back.
     */
    if (redirect_test_mirror && !aga_sandbox_active) {
        switch (size) {
        case 1:
            *(uint8_t *)p = (uint8_t)value;
            ps_write_8((unsigned)far, (uint8_t)value);
            break;
        case 2:
            *(uint16_t *)p = (uint16_t)value;
            ps_write_16((unsigned)far, (uint16_t)value);
            break;
        case 4:
            *(uint32_t *)p = (uint32_t)value;
            ps_write_32((unsigned)far, (uint32_t)value);
            break;
        case 8:
            *(uint64_t *)p = value;
            ps_write_64((unsigned)far, value);
            break;
        case 16:
            *(uint64_t *)p = value;
            *(uint64_t *)(p + 8) = value2;
            ps_write_64((unsigned)far, value);
            ps_write_64((unsigned)far + 8u, value2);
            break;
        default:
            return 0;
        }
        return 1;
    }

    switch (size) {
    case 1:  *(uint8_t  *)p = (uint8_t)value;  break;
    case 2:  *(uint16_t *)p = (uint16_t)value; break;
    case 4:  *(uint32_t *)p = (uint32_t)value; break;
    case 8:  *(uint64_t *)p = value;           break;
    case 16: *(uint64_t *)p = value; *(uint64_t *)(p + 8) = value2; break;
    default: return 0;
    }
    return 1;
}

/* Redirect test WITHOUT the sandbox: control word $5C0N.
 *
 * Both sandbox entries on the no-MMU path killed the machine before the first
 * counter could be read, and a dead machine says nothing about WHICH of the
 * two things entry does is at fault: switching the chipset (interrupts and
 * DMA away from the real chips) or serving chip RAM from the abort handler.
 * This does only the second. The real chipset keeps running untouched, so the
 * OS keeps its interrupts and caffed keeps talking: if the redirect is what
 * breaks, it breaks on a machine that can still report it.
 *
 *   N=1  the top 64 KB of chip RAM (agastat CHIPTEST's last two rows)
 *   N=2  all of chip RAM: every OS access, ExecBase, vectors, stacks, code
 *   N=0  off again; the window is copied back to the real chip RAM first,
 *        since the OS has been writing to the copy in the meantime.
 *
 * The window is copied from the bus first, so the 68k sees the same bytes
 * either side of the switch. agaboot sends this inside Disable() after a
 * CacheClearU(), exactly like the sandbox words. */
int aga_redirect_test(unsigned n)
{
    if (aga_sandbox_active || !chip_va || n > 2u) return 0;

    /* FRF_CHIP_REDIRECT_SAFE_MIRROR_V0_2
     * REDIRECT is a LIVE abort-handler probe, not a second sandbox. The real
     * chipset remains active, so physical Chip RAM must remain authoritative.
     */
    if (n == 0) {
        aga_chip_redirect = 0;
        redirect_test_mirror = 0;
        redirect_lo = redirect_hi = 0;
        __asm__ volatile("dmb ish" ::: "memory");
        cache_invalidate_all(DCACHE);
        cache_invalidate_all(ICACHE);
        aga_diag_log("[AGA] redirect test: OFF; physical Chip RAM stayed authoritative");
        return 1;
    }

    /* chip_real is learned during sandbox entry while the chipset is quiet.
     * Before the first safe probe, use this branch's guaranteed 512-KB floor.
     * The old N=1 incorrectly used the top of the 2-MB virtual allocation.
     */
    uint32_t real = chip_real ? chip_real : (512u << 10);
    if (real > chip_size) real = chip_size;
    if (real < 0x10000u) return 0;

    uint32_t lo = (n == 1u) ? real - 0x10000u : 0u;
    uint32_t hi = real;

    aga_chip_redirect = 0;
    redirect_test_mirror = 0;
    __asm__ volatile("dmb ish" ::: "memory");
    cache_invalidate_all(DCACHE);
    cache_invalidate_all(ICACHE);

    redirect_lo = lo;
    redirect_hi = hi;
    redirect_test_mirror = 1;
    __asm__ volatile("dmb ish" ::: "memory");
    aga_chip_redirect = 1;
    __asm__ volatile("dmb ish" ::: "memory");

    if (n == 1u)
        aga_diag_log("[AGA] redirect test: SAFE MIRROR top 64 KB of physical Chip RAM");
    else
        aga_diag_log("[AGA] redirect test: SAFE MIRROR all physical Chip RAM");
    return 1;
}

/* ---- CPU trap path (core 0) ----------------------------------------------- */
int aga_trap_write(uint64_t far, int size, uint64_t value, uint64_t value2)
{
    (void)value2;
    if (!aga_enabled || far < 0xDFF000 || far >= 0xE00000) return 0;
    uint32_t reg = (uint32_t)far & 0x1FF;
    last_trap_pc = emu68_get_pc();
    last_wr = (reg << 16) | (uint16_t)value;
    AGA_DIAG_INC(AGA_D_TRAPS_W);
    aga_diag_reg(reg, 1);
    AGA_DIAG_TRAP(reg, (uint16_t)value, 1, emu68_get_pc());
    lock();
    /* The sound driver's writes, in order, with where the beam was. A few
       thousand a second at most; the blitter's millions never get past the
       register test. */
#ifdef AGA_PROBES
    if ((reg >= 0x0A0 && reg < 0x0E0)
        || ((reg & 0x1FE) == 0x096 && (value & 0x020F))
        || (((reg & 0x1FE) == 0x09A || (reg & 0x1FE) == 0x09C) && (value & 0x0780))) {
        uint16_t vp = aga_custom_rget(aga, 0x004), vh = aga_custom_rget(aga, 0x006);
        aga_diag_audw(reg, size, (uint32_t)value, ((uint32_t)(vp & 7) << 8) | (vh >> 8), vh & 0xFF);
    }
#endif
    switch (size) {
    case 1: aga_write(aga, reg, (uint32_t)value, 1); break;
    case 2: aga_write(aga, reg, (uint32_t)value, 2); break;
    case 4: aga_write(aga, reg, (uint32_t)value, 4); break;
    case 8:
        aga_write(aga, reg, (uint32_t)(value >> 32), 4);
        aga_write(aga, reg + 4, (uint32_t)value, 4);
        break;
    case 16:
        /* FRF_V1_105_SAFE_COMPAT_MERGE_V0_1_6: AArch64 128-bit trap / 68k MOVEM.L write. */
        aga_write(aga, reg,      (uint32_t)(value  >> 32), 4);
        aga_write(aga, reg + 4,  (uint32_t)value,          4);
        aga_write(aga, reg + 8,  (uint32_t)(value2 >> 32), 4);
        aga_write(aga, reg + 12, (uint32_t)value2,         4);
        break;
    default: break;
    }
    /* FRF_AGA_JOTD_VBLANK_IRQ_V0_2: JOTD handlers clear VERTB with move.w #$20,INTREQ. */
    if (((reg & 0x1FE) == INTREQ) && !(value & INTF_SETCLR) &&
        (value & INTF_VERTB) && !frf_irq_seen_ack_vertb) {
        frf_irq_seen_ack_vertb = 1;
        aga_diag_log("[AGA] JOTD IRQ: 68k acknowledged VERTB through INTREQ");
    }
    unlock();
    return 1;
}

int aga_trap_read(uint64_t far, int size, uint64_t *value, uint64_t *value2)
{
    (void)value2;
    if (!aga_enabled || far < 0xDFF000 || far >= 0xE00000) return 0;
    uint32_t reg = (uint32_t)far & 0x1FF;
    AGA_DIAG_INC(AGA_D_TRAPS_R);
    aga_diag_reg(reg, 0);
    /* The beam registers bypass the mutex - see aga_peek_beam. They're 90% of
       all reads a game makes, and locking them saturated the machine. */
    if (aga_is_beam_reg(reg)) {
        uint64_t tpc = ticks_per_cck, base = beam_tick, add = 0;
        if (tpc) {
            uint64_t now;
            __asm__ volatile("mrs %0, CNTVCT_EL0" : "=r"(now));
            if (now > base) add = (now - base) / tpc;
            if (add > 0xFFFF) add = 0xFFFF;    /* loop stalled; do not run away */
            /* FRF_AGA_STRICT_TIMING_V0_1_2: the beam read must not claim the
               raster is thousands of ccks ahead while Copper/playfield state
               is frozen behind a host-side present. The core advances in
               8-cck chunks; interpolation is only valid inside that chunk. */
            if (aga_get_timing_strict(aga) && add > 7) add = 7;
        }
        *value = aga_peek_beam_at(aga, reg, size, (uint32_t)add);
        aga_diag_beam_value((uint16_t)*value);   /* the values a raster wait keeps seeing */
        /* NOT recorded in the trap ring. Beam reads are ~90% of all traps -
           2.7 million a second - so recording them turned the 256-entry ring
           into a 0.1 ms window full of noise. Everything else together runs
           at a few thousand a second, which makes the same ring about 100 ms
           of the machine actually doing things: what it programmed, and where
           it stopped. The rate itself is still counted in traps_read. */
        return 1;
    }
    lock();
    switch (size) {
    case 1: *value = aga_read(aga, reg, 1); break;
    case 2: *value = aga_read(aga, reg, 2); break;
    case 4: *value = aga_read(aga, reg, 4); break;
    case 8: *value = ((uint64_t)aga_read(aga, reg, 4) << 32) | aga_read(aga, reg + 4, 4); break;
    default: *value = 0; break;
    }
    unlock();
    if (((reg & 0x1FE) == INTREQR) && ((uint16_t)*value & INTF_VERTB) &&
        !frf_irq_seen_intreq_vertb) {
        frf_irq_seen_intreq_vertb = 1;
        aga_diag_log("[AGA] JOTD IRQ: 68k read INTREQR with VERTB pending");
    }
    /* after the read, so the ring records what the 68k actually got back -
       with a busy-wait the whole question is what value it keeps seeing */
    last_rd = (reg << 16) | (uint16_t)*value;
    AGA_DIAG_TRAP(reg, (uint16_t)*value, 0, emu68_get_pc());
    return 1;
}

/* ---- chipset loop (core 3) ------------------------------------------------ */
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

#define CHUNK_CCK 8      /* colour clocks per pacing step (~2.3 us) */

/* ---- libc shims for the core (Emu68 has no malloc/free) ------------------- */
extern void *tlsf;
extern void *tlsf_malloc(void *handle, uintptr_t size);
extern void  tlsf_free(void *handle, void *ptr);
extern void *memset(void *ptr, int fill, uintptr_t sz);

void *calloc(uintptr_t n, uintptr_t size)
{
    void *p = tlsf_malloc(tlsf, n * size);
    if (p) memset(p, 0, n * size);
    return p;
}

void free(void *p)
{
    if (p) tlsf_free(tlsf, p);
}

/* FRF_AGA_CPU3_V0_2
 * Cooperative CPU3 ownership: the existing FRF worker remains the permanent
 * CPU3 owner.  It calls this session only while AGA sandbox mode is active.
 * The session returns after aga_enabled is cleared, so FRF CPU3 services
 * resume after WHDLoad cleanup instead of being replaced forever. */
void aga_chipset_session(void)
{
    core_idle = 1;
    if (!aga_enabled)
        return;
    core_idle = 0;
    aga_diag_log("[AGA] CPU3 handoff: entering chipset session");
    /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_BEGIN_SESSION */
    if (aga_ham6_defer_startup())
        aga_diag_log("[AGA] RGB tap: physical HAM6 mirror + normal HVS/P96");
    /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_BEGIN_SESSION_END */
    const uint64_t freq = cntfrq();
    /* PAL colour clock 3546895 Hz; NTSC 3579545 Hz. Fixed point 32.32 ticks per cck. */
    /* PAL 3546895, NTSC 3579545 - from the flag the core was created with,
     * NOT from the register file.
     *
     * This used to ask aga_peek_reg(aga, BEAMCON0), and regs[] only ever holds
     * what the 68k has WRITTEN: aga_reset memsets it, and the machine's real
     * PAL/NTSC state lives in a->ntsc and a->beamcon0, not there. Almost no
     * game writes BEAMCON0, so the read returned 0, DISPLAYPAL was clear, and
     * the ternary picked NTSC - pacing a PAL machine, with PAL 227x313
     * geometry, at the NTSC colour clock. 3579545/3546895 is 1.0092, so
     * everything ran 0.92% fast.
     *
     * Two measurements already in docs/VALIDATION-FINDINGS.md match that to
     * better than 0.1%: 49.9203 x 1.0092 = 50.38 fps predicted against 50.4
     * sustained, and 436.9 audio streams/s predicted against 436.8 measured.
     *
     * It's also why the audio drifts. aga_audio_pull is asked for 27928
     * samples per second of EMULATED time, so at 0.92% fast the writer
     * produces 28185 a second while the real Paula, on the Amiga's own
     * crystal, consumes 27928 - an overrun of 257 samples every second. One
     * ring restart every 3.45 s, against the 3.55 s actually counted. Every
     * correction in aga_audio.c is compensating for this one damn line. */
    const uint64_t cck_hz = aga_is_ntsc ? 3579545 : 3546895;
    const uint64_t ticks_per_cck_fp = (freq << 32) / cck_hz;
    /* The pacing deadline: whole timer ticks plus a 32-bit fraction.
     *
     * NOT `cntvct() << 32`. cntvct() is a free-running 64-bit tick count, so
     * that shift throws away its top half and the time base keeps only the
     * low 32 bits - wrapping every 2^32 ticks. Every comparison against it
     * then means nothing, and the counters showed it plainly: 6163 resyncs in
     * 5973 frames against a threshold of a whole frame, while the largest lag
     * ever recorded was 7380 colour clocks. A resync needs 71051 to fire. It
     * could not have fired once, and it fired more often than there were
     * frames.
     *
     * Every one of those spurious resyncs DISCARDS the time the loop owes,
     * and the audio is generated from this clock - so this was the last thing
     * left corrupting it.
     *
     * Tried once before and it fucked the game, because the blitter's
     * completion delay was still counted in colour clocks: with the overflow
     * gone the loop started genuinely catching up, replaying thousands of
     * colour clocks in microseconds and expiring the delay before the game
     * had armed its wait. The glue now times the blit against the real clock
     * instead, so the two no longer interfere. */
    uint64_t next_ticks = cntvct();
    /* Where the frame's 20 ms go: waiting for the pacing clock is the loop's
       headroom, the rest is what the emulation costs. The Pi 3 fell a frame
       behind seven times a second (2026-09-13) and frame_us_worst, which only
       times the present, said nothing about why; these say it directly, on
       any board. */
    uint64_t frame_t0 = next_ticks, wait_acc = 0, audio_acc = 0;
    /* and where the busy time goes: the core itself per chunk, the once-a-
       frame statistics, and the whole loop body between two pacing waits -
       a body that's long while the parts are short is time taken out from
       under it (an interrupt, the firmware). DAIF says whether interrupts
       can even reach this core. */
    uint64_t cyc_acc = 0, cyc_max = 0, stat_max = 0, body_max = 0, stat_t0 = 0;
    /* The A53's own account of where this core's time goes: L1 data refills
       (event 0x03) and L2 refills (0x17) on core 3, per frame. A frame that
       runs the same emulation six times slower while another core streams
       through the shared L2 should show it here, and which of the two says
       whether the remedy is my working set or the other core's traffic. */
#ifdef AGA_PROBES
    uint64_t pmu_l1_prev = 0, pmu_l2_prev = 0;
    {
        uint64_t v;
        __asm__ volatile("msr PMEVTYPER0_EL0, %0" :: "r"((uint64_t)0x03));
        __asm__ volatile("msr PMEVTYPER1_EL0, %0" :: "r"((uint64_t)0x17));
        __asm__ volatile("msr PMEVCNTR0_EL0, %0" :: "r"((uint64_t)0));
        __asm__ volatile("msr PMEVCNTR1_EL0, %0" :: "r"((uint64_t)0));
        __asm__ volatile("msr PMCNTENSET_EL0, %0" :: "r"((uint64_t)0x3));
        __asm__ volatile("mrs %0, PMCR_EL0" : "=r"(v));
        __asm__ volatile("msr PMCR_EL0, %0" :: "r"(v | 1));           /* E: counters on */
        __asm__ volatile("isb");
    }
    /* A memory-latency probe, once a frame: 512 dependent loads chasing a
       random cycle through 1 MB - twice the Pi 3's L2 - so it measures DRAM
       under whatever the other cores are doing to it. If it spikes in the
       frames where the emulation runs slow, the core is being starved of
       memory, not of anything of its own. */
    static uint32_t mem_probe[262144];
    static uint32_t cw_prev;
    {
        uint32_t n = 262144, seed = 12345;
        for (uint32_t i = 0; i < n; i++) mem_probe[i] = i;
        for (uint32_t i = n - 1; i > 0; i--) {          /* Sattolo: one cycle through all of it */
            seed = seed * 1103515245u + 12345u;
            uint32_t j = (seed >> 8) % i;
            uint32_t t = mem_probe[i]; mem_probe[i] = mem_probe[j]; mem_probe[j] = t;
        }
    }
#endif
    { uint64_t daif; __asm__ volatile("mrs %0, DAIF" : "=r"(daif)); aga_diag_set(AGA_D_CORE3_DAIF, (uint32_t)daif); }
    uint32_t next_frac  = 0;
    uint64_t chunk_at   = cntvct();   /* when the last pacing chunk actually ran */
    const uint64_t ticks_per_frame = freq / 50;
    /* How far behind before the owed time is dropped instead of caught up.
       One frame on a Pi 4. The Pi 3's frames spike to 31 ms with video on
       (2026-09-14) and every spike past 20 ms became a resync that dropped
       the time and starved the audio ring; two frames lets it catch up, which
       the loop does in a few milliseconds of real time. */
    const uint64_t resync_ticks = ticks_per_frame * (aga_pi3_mode() ? 2 : 1);
    /* Frame skip, Pi 3 only: ask for it once 2.5 ms behind, drop the request
       once caught up. 5 ms was the first value and let the loop build
       catch-up bursts big enough to be heard: the game's music routine runs
       in real time off the emulated vertical blank, so a burst bunches its
       note starts against the emulated Paula (Banshee, 2026-09-14). The Pi 4
       never falls that far behind. */
    const int skip_ok = aga_pi3_mode();
    int skip_want = 0;
    int last_real_ipl = 0;
    uint32_t blit_prev = 0;   /* accumulate the delta, so zeroing `blits` sticks */
    uint32_t pace_frames = 0;
    int audio_countdown = 0;
    /* FRF_AGA_STRICT_TIMING_V0_1_2: the core now supplies the requested BBUSY
       lifetime in colour clocks. FAST returns 32 cck (~9 us), preserving the
       old compatibility behaviour; STRICT returns the HRM-derived duration. */
    uint64_t blit_due = 0;
    uint32_t blit_due_seq = 0;

    kprintf("[AGA] chipset loop on core 3, timer %d Hz, %d ticks/cck\n", (int)freq, (int)(ticks_per_cck_fp >> 32));
    /* whole ticks per colour clock for the interpolating beam read. The Pi's
       26 MHz counter gives ~7 ticks per cck, so integer division is good to
       about a seventh of a colour clock - far finer than the 8 the loop
       steps. */
    ticks_per_cck = (ticks_per_cck_fp >> 32) ? (ticks_per_cck_fp >> 32) : 1;
    beam_tick = cntvct();

    /* Bring-up watchdog. The first time the chipset ran on real hardware,
       entering the sandbox hung the machine: agaboot never returned, so caffed
       died with the 68k and neither the counters nor the log ring could be
       read. Debugging a hang you can't observe isn't debugging.
     *
     * If the chipset is enabled but no frame completes within the timeout,
     * give up and park: clear aga_enabled so this loop drops out, release the
     * sandbox, log it. That covers the likely case where core 0 is stuck in a
     * trap waiting on a lock this core holds - dropping out releases it and
     * the machine comes back with the counters intact and readable.
     *
     * It can't do a damn thing if THIS core is the stuck one. That needs a
     * serial console; see docs/HARDWARE-BRINGUP.md. */
    uint64_t wd_deadline = cntvct() + freq * AGA_WATCHDOG_SEC;

    while (aga_enabled) {
        /* Sandbox mode only. In machine mode the whole system runs on the
           virtual chipset from boot, so there's no real chipset to fall back
           to - parking would kill the machine outright instead of recovering
           it. The watchdog exists to make a failed sandbox ENTRY survivable. */
        if (aga_sandbox_active && (int64_t)(cntvct() - wd_deadline) > 0) {
            aga_diag_log("[AGA] watchdog: no frame completed, parking the chipset");
            kprintf("[AGA] watchdog: no frame in %d s, parking the chipset\n",
                    AGA_WATCHDOG_SEC);
            aga_enabled = 0;
            __asm__ volatile("sev");   /* wake core 3 */
            aga_sandbox_active = 0;
            sandbox_restore();         /* NOT optional - see the note on it */
            aga_video_hide();          /* same reason as in aga_sandbox_leave */
            break;
        }
        {
            uint64_t inc = ticks_per_cck_fp * CHUNK_CCK;   /* 32.32 ticks */
            uint32_t f = next_frac + (uint32_t)inc;
            next_ticks += (inc >> 32) + (f < next_frac ? 1u : 0u);
            next_frac = f;

            /* Catch up at no more than twice real time.
             *
             * Being right ON AVERAGE isn't worth shit. Presenting a frame
             * blocks this loop for about 900us, and an uncapped catch-up then
             * replays those 900us of emulated time in a few microseconds. The
             * average rate is perfect - loop_resyncs sits at 0 - but the
             * instantaneous
             * rate is a stall followed by a burst, and everything the chipset
             * drives comes out in that shape. A music player clocked by audio
             * or timer interrupts gets its tempo ticks bunched, so several
             * notes fire at once and then nothing: "you hear drums all at
             * once, so lag", with effects appearing only here and there.
             *
             * Requiring each chunk to occupy at least half its real duration
             * caps the loop at 2x, so a 900us present is repaid smoothly over
             * the next 900us instead of instantly - well inside one frame, and
             * the interrupts stay evenly spaced. */
            uint64_t real_chunk = (inc >> 32);
            /* FRF_AGA_STRICT_2X_CATCHUP_V0_1
               Preserve STRICT correctness, but let CPU3 repay presentation
               stalls at up to 2x real-time just like FAST. No chipset state
               is skipped: every normal aga_step chunk is still executed. */
            /* FRF_AGA_STRICT_CATCHUP_2X_V0_1_2: pacing-only update.
               STRICT and FAST both retain every chipset quantum, but bounded
               host catch-up may run at up to 2x when CPU3 is behind. */
            uint64_t floor_delta = (real_chunk >> 1);
            if (!floor_delta) floor_delta = 1;
            uint64_t floor_t = chunk_at + floor_delta;
            uint64_t target = next_ticks > floor_t ? next_ticks : floor_t;
            {
                uint64_t w0 = cntvct();
                if (chunk_at && w0 > chunk_at && w0 - chunk_at > body_max) body_max = w0 - chunk_at;
                while (cntvct() < target) {
                    __asm__ volatile("yield");
                }
                chunk_at = cntvct();
                if (chunk_at > w0) wait_acc += chunk_at - w0;
            }
        }
        /* aga.loop_stop=N: a ladder, one rung per boot. Each level adds one
           thing the loop does, so a level that boots and a level that dies
           name the culprit between them.

             1  pace only - keep time, count frames, call nothing
             2  + the bus mutex: lock()/unlock() around nothing
             3  + aga_run_cycles: the chipset core itself, no video, no audio
             4  + the frame present path
             5  + aga_audio_pull only - the audio maths, but nothing streamed
             0  everything, audio included (the normal build)

           MEASURED 2026-09-07 on a Pi 4 in sandbox mode (see
           docs/VALIDATION-FINDINGS.md): rung 3 and rung 4 both run at
           50.4-50.5 frames per second, sustained, presents tracking frames
           exactly, video_errors 0, machine reachable throughout. Rung 0 dies
           on its arse. So aga_run_cycles and aga_video_present are BOTH
           innocent, and the fault is in the audio block - which is what rung
           5 now splits in half.

           Rungs 1 and 2 prove fuck all, and can't. Sandbox entry installs the
           traps whatever this setting is, so at those rungs the 68k talks to
           a chipset that never ticks: no vblank, no interrupts, AmigaOS
           stalls. A hang there is the expected behaviour of correct code.
           They're kept only to check that the loop skeleton and the wake
           handshake run at all, and they now let the watchdog fire so the
           machine recovers and the counters can be read. */
        if (aga_loop_stop == 1 || aga_loop_stop == 2) {
            if (aga_loop_stop == 2) { lock(); unlock(); }
            if (++pace_frames >= 313 * 227 / CHUNK_CCK) {   /* one PAL frame */
                pace_frames = 0;
                aga_diag_inc(AGA_D_FRAMES);
                /* NOT rearming the watchdog here. Rungs 1 and 2 leave the
                   chipset frozen - the traps are installed but nothing ticks,
                   so the 68k waits forever for a vblank and AmigaOS stalls.
                   That's expected, not a bug, and it means a hang at these
                   rungs proves nothing. Letting the watchdog fire parks the
                   chipset, releases the machine and leaves the counters
                   readable, which is the whole point. */
            }
            continue;
        }
        int native = 0;
        lock();
        /* FRF_AGA_STRICT_TIMING_V0_1_2: no Copper-off LOCT poll here. The
           one-shot core guard resolves ownership at the first game palette,
           BPLCON3 or Copper action, so explicit modern AGA LOCT use wins. */
#ifdef AGA_PROBES
        {
            uint64_t c0 = cntvct();
            aga_run_cycles(aga, CHUNK_CCK);
            uint64_t dc = cntvct() - c0;
            cyc_acc += dc; if (dc > cyc_max) cyc_max = dc;
        }
#else
        aga_run_cycles(aga, CHUNK_CCK);
#endif
        /* Report an outstanding blit finished once enough WALL time has
           passed. The core no longer counts this down in colour clocks: those
           only mean anything while the loop runs them at real-time speed, and
           it doesn't - so a catch-up burst used to expire the delay before
           the game had armed the wait it was about to sleep on. Timing it
           here decouples the blitter from the pacing entirely. */
        if (aga_blit_pending(aga)) {
            uint64_t now = cntvct();
            uint32_t seq = aga_blit_count(aga);
            if (!blit_due || seq != blit_due_seq) {
                uint32_t delay_cck = aga_blit_delay_cck(aga);
                __uint128_t prod = (__uint128_t)ticks_per_cck_fp * (uint64_t)delay_cck;
                uint64_t delay_ticks = (uint64_t)(prod >> 32);
                if (!delay_ticks) delay_ticks = 1;
                blit_due = now + delay_ticks;
                blit_due_seq = seq;
            } else if (now >= blit_due) {
                aga_blit_complete(aga);
                blit_due = 0;
            }
        } else {
            blit_due = 0;
            blit_due_seq = 0;
        }
        /* publish the beam base for the lock-free interpolating read. Order
           matters: the timestamp goes last, so a reader that sees a fresh
           tick is guaranteed to be looking at the position that goes with
           it. */
        __asm__ volatile("dmb ish" ::: "memory");
        beam_tick = cntvct();
        int wmode = 0, wx = 0, wy = 0, ww = 0, wh = 0;
        if (frame_pending) {
            native = aga_native_active(aga);
            wmode = aga_window_mode(aga, &wx, &wy, &ww, &wh);
        }
        /* CIA interrupts are NOT mirrored here any more. This used to compare
           emu68_get_real_ipl() against 2 and 6, but on the classic PiStorm
           that byte is a 0/1 "IPL line active" flag, never the level, so it
           never matched and the virtual INTREQ never showed PORTS/EXTER. The
           real exception path delivers those levels; vectors.c folds the real
           INTREQR bits into INTREQR reads so the OS handler acknowledges the
           CIA. */
        (void)last_real_ipl;
        unlock();
        if (frame_pending && aga_loop_stop == 3) {
            /* chipset only: a frame completed, but nothing is presented. */
            frame_pending = 0;
            aga_diag_inc(AGA_D_FRAMES);
            {   /* the frame's time budget: busy = everything but the pacing wait */
                uint64_t fn = cntvct(), ft = fn - frame_t0;
                uint64_t busy = ft > wait_acc ? ft - wait_acc : 0;
                uint32_t bus_us = (uint32_t)((busy * 1000000u) / freq);
                aga_diag_set(AGA_D_LOOP_BUSY_US, bus_us);
                aga_diag_max(AGA_D_LOOP_BUSY_WORST, bus_us);
                aga_diag_set(AGA_D_LOOP_WAIT_US, (uint32_t)((wait_acc * 1000000u) / freq));
                aga_diag_set(AGA_D_AUDIO_US_FRAME, (uint32_t)((audio_acc * 1000000u) / freq));
                aga_diag_set(AGA_D_LOCK_US_FRAME, (uint32_t)((lock_acc3 * 1000000u) / freq));
                aga_diag_max(AGA_D_LOCK_US_WORST, (uint32_t)((lock_max3 * 1000000u) / freq));
                lock_acc3 = 0;
                aga_diag_set(AGA_D_CYC_US_FRAME, (uint32_t)((cyc_acc * 1000000u) / freq));
                aga_diag_max(AGA_D_CYC_US_WORST, (uint32_t)((cyc_max * 1000000u) / freq));
                aga_diag_max(AGA_D_BODY_US_WORST, (uint32_t)((body_max * 1000000u) / freq));
                aga_diag_max(AGA_D_CYC_FRAME_WORST, (uint32_t)((cyc_acc * 1000000u) / freq));
#ifdef AGA_PROBES
                { uint64_t l1, l2;
                  __asm__ volatile("mrs %0, PMEVCNTR0_EL0" : "=r"(l1));
                  __asm__ volatile("mrs %0, PMEVCNTR1_EL0" : "=r"(l2));
                  uint32_t d1 = (uint32_t)(l1 - pmu_l1_prev), d2 = (uint32_t)(l2 - pmu_l2_prev);
                  pmu_l1_prev = l1; pmu_l2_prev = l2;
                  aga_diag_set(AGA_D_L1_REFILL_FRAME, d1); aga_diag_max(AGA_D_L1_REFILL_WORST, d1);
                  aga_diag_set(AGA_D_L2_REFILL_FRAME, d2); aga_diag_max(AGA_D_L2_REFILL_WORST, d2); }
                { static uint64_t ab_prev, ac_prev;
                  uint64_t ab = aga_abort_ticks, ac = aga_abort_count;
                  uint32_t aus = (uint32_t)(((ab - ab_prev) * 1000000u) / freq);
                  aga_diag_set(AGA_D_ABORT_US_FRAME, aus); aga_diag_max(AGA_D_ABORT_US_WORST, aus);
                  aga_diag_set(AGA_D_ABORTS_FRAME, (uint32_t)(ac - ac_prev)); aga_diag_max(AGA_D_ABORTS_WORST, (uint32_t)(ac - ac_prev));
                  ab_prev = ab; ac_prev = ac; }
                { uint32_t cw = aga_diag_ctr[AGA_D_CHIP_W];
                  aga_diag_set(AGA_D_CHIPW_FRAME, cw - cw_prev);
                  aga_diag_max(AGA_D_CHIPW_FRAME_WORST, cw - cw_prev); cw_prev = cw; }
                { uint64_t m0 = cntvct(); uint32_t p = (uint32_t)(m0 & 0x3FFFF);
                  for (int i = 0; i < 512; i++) p = mem_probe[p];      /* ~80 us on a Pi 3, less on a Pi 4 */
                  __asm__ volatile("" :: "r"(p));
                  uint32_t mus = (uint32_t)(((cntvct() - m0) * 1000000u) / freq);
                  aga_diag_set(AGA_D_MEM_US, mus); aga_diag_max(AGA_D_MEM_US_WORST, mus); }
#endif
                cyc_acc = 0; cyc_max = 0; body_max = 0;
                frame_t0 = fn; wait_acc = 0; audio_acc = 0;
            }
            wd_deadline = cntvct() + freq * AGA_WATCHDOG_SEC;
        } else if (frame_pending) {
            frame_pending = 0;
            if (AGA_PROBES_ON && aga_sandbox_active) {
                /* three bus reads per frame: what the real Paula thinks */
                /* timed: on a Pi 3 these three bus reads are the last thing
                   core 3 does on the PiStorm bus in rung 5, and the bus is
                   the suspect for the 12 ms stalls (2026-09-14) */
                uint64_t b0 = cntvct();
                aga_diag_set(AGA_D_REAL_INTENA, ps_read_16(0xDFF01C));
                aga_diag_set(AGA_D_REAL_INTREQ, ps_read_16(0xDFF01E));
                aga_diag_set(AGA_D_REAL_DMACON, ps_read_16(0xDFF002));
                uint32_t bus_us = (uint32_t)(((cntvct() - b0) * 1000000u) / freq);
                aga_diag_set(AGA_D_BUS_US_FRAME, bus_us);
                aga_diag_max(AGA_D_BUS_US_WORST, bus_us);
            }
            uint64_t t0 = cntvct();
            /* FRF_AGA_RGB_TAP_HAM6_V0_6_0_FRAME_TAP
             * Tap the completed software-AGA RGB frame BEFORE Picasso/HVS.
             * HAM6 is a mirror output, never a replacement for the RTG path. */
            {
                const uint32_t *frf_rgb = aga_framebuffer(aga);
                /* FRF_AGA_RGB_TAP_HAM6_V0_11_0_EXCLUSIVE_PRESENT
                 * OUTPUT HAM6 is the AGA presenter, not a compulsory mirror.
                 * If HAM cannot arm, retain RTG/HVS as a safe fallback. */
                {
                    static int ham_hvs_hidden = 0;
                    aga_program_frame_probe();   /* FRF_AGA_PROGRAM_SCOPE_V0_1 */
                    aga_ham6_service_takeover(); /* FRF_AGA_WHDLOAD_HAM6_HANDOVER_V0_3 CPU3 */
                    aga_ham6_present(frf_rgb, native);
                    if (aga_ham6_active()) {
                        if (!ham_hvs_hidden) {
                            aga_video_hide();
                            ham_hvs_hidden = 1;
                        }
                    } else {
                        ham_hvs_hidden = 0;
                        aga_video_set_window(wmode, wx, wy, ww, wh);
                        aga_video_present(frf_rgb, native);
                    }
                }
            }
            uint32_t us = (uint32_t)(((cntvct() - t0) * 1000000u) / freq);
            aga_diag_set(AGA_D_FRAME_US, us);
            aga_diag_max(AGA_D_FRAME_US_WORST, us);
            aga_diag_inc(AGA_D_FRAMES);
            aga_diag_inc(AGA_D_PRESENTS);
            if (AGA_PROBES_ON) {   /* how far behind real time the loop is as this frame goes out */
                uint64_t nowt = cntvct();
                uint64_t lagt = nowt > next_ticks ? nowt - next_ticks : 0;
                uint32_t lms = (uint32_t)((lagt * 1000u) / freq);
                int b = lms < 2 ? 0 : lms < 5 ? 1 : lms < 10 ? 2 : lms < 20 ? 3 : lms < 40 ? 4 : 5;
                aga_diag_inc(AGA_D_LAG_HIST0 + b);
            }
            {   /* the frame's time budget: busy = everything but the pacing wait */
                uint64_t fn = cntvct(), ft = fn - frame_t0;
                uint64_t busy = ft > wait_acc ? ft - wait_acc : 0;
                uint32_t bus_us = (uint32_t)((busy * 1000000u) / freq);
                aga_diag_set(AGA_D_LOOP_BUSY_US, bus_us);
                aga_diag_max(AGA_D_LOOP_BUSY_WORST, bus_us);
                aga_diag_set(AGA_D_LOOP_WAIT_US, (uint32_t)((wait_acc * 1000000u) / freq));
                aga_diag_set(AGA_D_AUDIO_US_FRAME, (uint32_t)((audio_acc * 1000000u) / freq));
                aga_diag_set(AGA_D_LOCK_US_FRAME, (uint32_t)((lock_acc3 * 1000000u) / freq));
                aga_diag_max(AGA_D_LOCK_US_WORST, (uint32_t)((lock_max3 * 1000000u) / freq));
                lock_acc3 = 0;
                aga_diag_set(AGA_D_CYC_US_FRAME, (uint32_t)((cyc_acc * 1000000u) / freq));
                aga_diag_max(AGA_D_CYC_US_WORST, (uint32_t)((cyc_max * 1000000u) / freq));
                aga_diag_max(AGA_D_BODY_US_WORST, (uint32_t)((body_max * 1000000u) / freq));
                aga_diag_max(AGA_D_CYC_FRAME_WORST, (uint32_t)((cyc_acc * 1000000u) / freq));
#ifdef AGA_PROBES
                { uint64_t l1, l2;
                  __asm__ volatile("mrs %0, PMEVCNTR0_EL0" : "=r"(l1));
                  __asm__ volatile("mrs %0, PMEVCNTR1_EL0" : "=r"(l2));
                  uint32_t d1 = (uint32_t)(l1 - pmu_l1_prev), d2 = (uint32_t)(l2 - pmu_l2_prev);
                  pmu_l1_prev = l1; pmu_l2_prev = l2;
                  aga_diag_set(AGA_D_L1_REFILL_FRAME, d1); aga_diag_max(AGA_D_L1_REFILL_WORST, d1);
                  aga_diag_set(AGA_D_L2_REFILL_FRAME, d2); aga_diag_max(AGA_D_L2_REFILL_WORST, d2); }
                { static uint64_t ab_prev, ac_prev;
                  uint64_t ab = aga_abort_ticks, ac = aga_abort_count;
                  uint32_t aus = (uint32_t)(((ab - ab_prev) * 1000000u) / freq);
                  aga_diag_set(AGA_D_ABORT_US_FRAME, aus); aga_diag_max(AGA_D_ABORT_US_WORST, aus);
                  aga_diag_set(AGA_D_ABORTS_FRAME, (uint32_t)(ac - ac_prev)); aga_diag_max(AGA_D_ABORTS_WORST, (uint32_t)(ac - ac_prev));
                  ab_prev = ab; ac_prev = ac; }
                { uint32_t cw = aga_diag_ctr[AGA_D_CHIP_W];
                  aga_diag_set(AGA_D_CHIPW_FRAME, cw - cw_prev);
                  aga_diag_max(AGA_D_CHIPW_FRAME_WORST, cw - cw_prev); cw_prev = cw; }
                { uint64_t m0 = cntvct(); uint32_t p = (uint32_t)(m0 & 0x3FFFF);
                  for (int i = 0; i < 512; i++) p = mem_probe[p];      /* ~80 us on a Pi 3, less on a Pi 4 */
                  __asm__ volatile("" :: "r"(p));
                  uint32_t mus = (uint32_t)(((cntvct() - m0) * 1000000u) / freq);
                  aga_diag_set(AGA_D_MEM_US, mus); aga_diag_max(AGA_D_MEM_US_WORST, mus); }
#endif
                cyc_acc = 0; cyc_max = 0; body_max = 0;
                frame_t0 = fn; wait_acc = 0; audio_acc = 0;
            }
            stat_t0 = cntvct();
            aga_audio_set_mask(aga, aud_mask_want);   /* survives any reset */
#ifdef AGA_PROBES
            {   /* per-channel audio, once a frame: which channels the game
                   actually runs, and how much data each one pulls */
                uint32_t aon[4], awd[4], anz[4];
                aga_audio_stats(aga, aon, awd, anz);
                for (int n = 0; n < 4; n++) {
                    aga_diag_set(AGA_D_AUD0_ON + n, aon[n]);
                    aga_diag_set(AGA_D_AUD0_W  + n, awd[n]);
                    aga_diag_set(AGA_D_AUD0_NZ + n, anz[n]);
                }
                uint32_t atrig[32 * 6]; int ntrig = 0;
                aga_audio_triggers(aga, atrig, &ntrig);
                aga_diag_triggers(atrig, ntrig);
                uint32_t avol[4], avmax[4], aaud[4];
                aga_audio_vols(aga, avol, avmax, aaud);
                {   /* per-channel loudness for THIS frame, not since boot */
                    static uint32_t nrg_prev[4];
                    uint32_t d[4];
                    for (int n = 0; n < 4; n++) { d[n] = anz[n] - nrg_prev[n]; nrg_prev[n] = anz[n]; }
                    aga_diag_env(d);
                }

                uint32_t vwr[4], vnz[4], voff[4], awrap[4], airq[4];
                aga_audio_volstats(aga, vwr, vnz, voff);
                aga_audio_wrapstats(aga, awrap, airq);
                for (int n = 0; n < 4; n++) aga_diag_set(AGA_D_AUD0_WRAP + n, awrap[n]);
                aga_diag_set(AGA_D_AUD_INTENA, aga_custom_rget(aga, INTENAR));
                { uint32_t wi[32 * 4]; int nwi = 0;
                  aga_audio_wrapinfo(aga, wi, &nwi); aga_diag_wrapinfo(wi, nwi); }
                for (int n = 0; n < 4; n++) {
                    aga_diag_set(AGA_D_AUD0_VWR + n, vwr[n]);
                    aga_diag_set(AGA_D_AUD0_VNZ + n, vnz[n]);
                    aga_diag_set(AGA_D_AUD0_OFF + n, voff[n]);
                    aga_diag_set(AGA_D_AUD0_VOL  + n, avol[n]);
                    aga_diag_set(AGA_D_AUD0_VMAX + n, avmax[n]);
                    aga_diag_set(AGA_D_AUD0_AUD  + n, aaud[n]);
                }
            }
#endif
            int had_data = aga_last_frame_had_data(aga);
            uint32_t bc = aga_blit_count(aga);
            aga_diag_ctr[AGA_D_BLITS] += bc - blit_prev;
            blit_prev = bc;
            if (had_data) aga_diag_inc(AGA_D_DATA_FRAMES);
            aga_diag_snapshot(had_data);   /* survives the reset a wedged game forces */
            /* and put it on the screen, which is the only channel that
               survives a wedge: no OS, no caffed, no keyboard needed. */
            /* The display state, two registers a row. On a classic PiStorm
               every Ctrl-Amiga-Amiga reboots the whole Pi (Emu68's housekeeper
               does it on purpose), so after a game with no quit - Reshoot
               Proxima 3 - nothing the kernel recorded survives. A photograph
               of this does. Off unless `agaboot OVERLAY ON`; the diag build
               has it on.
                 0 BPLCON0:DMACONR  1 BPLCON1:BPLCON2  2 BPLCON3:BPLCON4
                 3 FMODE:DIWHIGH    4 DIWSTRT:DIWSTOP  5 DDFSTRT:DDFSTOP
                 6 BPL1MOD:BPL2MOD  7 BPL1PT   8 BPL2PT
                 9 frames (moving = alive)  10 blits  11 last write reg:value */
#define OVL2(r1, r2) (((uint32_t)aga_peek_reg(aga, r1) << 16) | aga_peek_reg(aga, r2))
            aga_video_overlay(0, ((uint32_t)aga_peek_reg(aga, 0x100) << 16) | aga_custom_rget(aga, DMACONR));
            aga_video_overlay(1, OVL2(0x102, 0x104));
            aga_video_overlay(2, OVL2(0x106, 0x10C));
            aga_video_overlay(3, OVL2(0x1FC, 0x1E4));
            aga_video_overlay(4, OVL2(0x08E, 0x090));
            aga_video_overlay(5, OVL2(0x092, 0x094));
            aga_video_overlay(6, OVL2(0x108, 0x10A));
            aga_video_overlay(7, aga_peek_bplpt(aga, 0));
            aga_video_overlay(8, aga_peek_bplpt(aga, 1));
            aga_video_overlay(9, aga_diag_ctr[AGA_D_FRAMES]);
            aga_video_overlay(10, aga_diag_ctr[AGA_D_BLITS]);
            aga_video_overlay(11, last_wr);
#undef OVL2
            wd_deadline = cntvct() + freq * AGA_WATCHDOG_SEC;   /* progress: rearm */
            { extern uint32_t aga_dl_inuse, aga_dl_free_at, aga_dl_free_len;
              aga_diag_set(AGA_D_DL_INUSE, aga_dl_inuse);
              aga_diag_set(AGA_D_DL_FREE_AT, aga_dl_free_at);
              aga_diag_set(AGA_D_DL_FREE_LEN, aga_dl_free_len); }
            aga_diag_set(AGA_D_STATE, (uint32_t)(1 | (aga_sandbox_active ? 2 : 0) | (native ? 4 : 0)));
            { uint64_t ds = cntvct() - stat_t0; if (ds > stat_max) stat_max = ds;
              aga_diag_max(AGA_D_STAT_US_WORST, (uint32_t)((stat_max * 1000000u) / freq)); }
        }
        /* audio: SOFTWARE pulls/mixes PCM. NATIVE keeps virtual Paula timing
           and IRQ state inside the chipset core but lets the real Paula perform
           the repetitive DMA/DAC work, so there is deliberately no mixer pull. */
        if ((aga_loop_stop == 0 || aga_loop_stop == 5) && ++audio_countdown >= 1024) {
            audio_countdown = 0;
            if (aga_audio_software_needed()) {
            int maxf;
            int16_t *buf = aga_audio_scratch(&maxf);
            lock();
            int n = aga_audio_pull(aga, buf, maxf, 3546895 / 127);
            unlock();
            /* rung 5 stops here: aga_audio_pull has run (pure computation
               over virtual state, under the lock) but nothing is streamed.
               Rung 0 goes on to aga_audio_stream, which issues 32-bit
               ps_write_32 PiStorm bus writes from core 3 and reprograms the
               real Paula. Rung 4 survives at 50 fps and rung 0 does not, so
               the fault is in this block; this splits it in half. */
            if (n > 0 && aga_loop_stop == 0) {
                uint64_t a0 = cntvct();
                aga_audio_stream(buf, n);
                uint64_t at = cntvct() - a0;
                audio_acc += at;
                aga_diag_max(AGA_D_AUDIO_US_WORST, (uint32_t)((at * 1000000u) / freq));
            }
            }
        }
        /* if the loop falls far behind (e.g. blocked on the lock),
           resynchronise rather than race */
        uint64_t now_ticks = cntvct();
        if (now_ticks > next_ticks) {
            uint64_t behind = ((now_ticks - next_ticks) * cck_hz) / freq;
            aga_diag_max(AGA_D_LAG_MAX, (uint32_t)behind);
        }
        /* Resync threshold: only give up and drop the owed time when
         * hopelessly behind, never after a normal frame.
         *
         * This has flip-flopped. Four scanlines (256us) was the first value,
         * and presenting a frame takes about 900us (frame_us_worst), so EVERY
         * frame overshot it and resynchronised: 4848 resyncs in 4990 frames,
         * or 1883 in 1993 in another run. Every resync DISCARDS the time the
         * loop owes, and the audio mix is generated from this very clock - the
         * stream ran short by ~900us a frame (4.5% of a 20 ms frame), the
         * Paula ring repeated what was already in it, and the music came out
         * with its beats in the wrong places. That's what every nudge and ring
         * restart in aga_audio.c was trying to paper over.
         *
         * Raising it to a whole frame looked right but put Banshee straight
         * back to a blank fucking screen with blits at zero: catching up
         * REPLAYS the owed time, and with a frame-sized budget the chipset ran
         * ~3200 colour clocks in a few microseconds after every present. The
         * blitter's completion delay was counted in colour clocks then
         * (BLIT_DELAY_CCK), so inside a catch-up burst those 64 colour clocks
         * expired in well under a microsecond - before the game had armed the
         * wait it was about to sleep on, the very race the delay exists to
         * prevent. So back to four scanlines it went, and the drift stayed in
         * aga_audio.c, one sample at a time - audio_resyncs went from 31 in
         * 110 seconds to 0.
         *
         * Now the glue times the blit against the real clock, so the two are
         * independent and the loop can pay back what it owes. Anything
         * measured in emulated colour clocks is only as trustworthy as the
         * loop's willingness to run them at real-time speed, and the ARM runs
         * far faster than the emulated chipset, so the honest answer is to
         * catch the cycles up - which the loop does by itself as soon as the
         * present returns. */
        if (skip_ok) {
            uint64_t behind_t = now_ticks > next_ticks ? now_ticks - next_ticks : 0;
            int want = skip_want;
            if (behind_t > ticks_per_frame / 8) want = 1;
            else if (behind_t < ticks_per_frame / 50) want = 0;
            if (want != skip_want) { skip_want = want; aga_set_frame_skip(aga, want); }
            aga_diag_set(AGA_D_FRAMES_SKIPPED, aga_frames_skipped(aga));
        }
        if (now_ticks > next_ticks + resync_ticks) {
            next_ticks = now_ticks;
            next_frac  = 0;
            aga_diag_inc(AGA_D_RESYNCS);
        }
    }
    /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_END_SESSION */
    aga_ham6_takeover_pending = 0; /* FRF_AGA_WHDLOAD_HAM6_HANDOVER_V0_3: cancel stale arm */
    __asm__ volatile("dmb ish" ::: "memory");
    aga_ham6_end();
    frame_pending = 0;
    core_idle = 1;
    __asm__ volatile("dmb ish; sev" ::: "memory");
    aga_diag_log("[AGA] CPU3 handoff: chipset session ended; standalone CPU3 owner resumes");
}

/* Standalone Emu68 CPU3 owner.
 *
 * Earlier development builds entered aga_chipset_session() through a generic
 * FRF coprocessor worker.  The public AGA/HAM6 tree deliberately has no such
 * dependency: CPU3 waits here and owns only the virtual chipset.
 */
void aga_chipset_main(void)
{
    for (;;) {
        if (aga_enabled) {
            aga_chipset_session();
            continue;
        }
        core_idle = 1;
        __asm__ volatile("wfe" ::: "memory");
    }
}

/* ---- virtual floppy: CIA lines, image commands, disk-swap hotkey ----------- */
#define DSK_SLOTS 4
static uint8_t *dsk_slot[DSK_SLOTS];      /* images kept for Ctrl+F1..F4 swapping */
static uint32_t dsk_slot_size[DSK_SLOTS];
static int      key_ctrl;

static void slot_insert(int n)
{
    if (n < 0 || n >= DSK_SLOTS || !dsk_slot[n]) return;
    lock();
    aga_disk_insert(aga, 0, dsk_slot[n], dsk_slot_size[n], 0);
    unlock();
    kprintf("[AGA] disk: DF0 <- slot %d (%d KB)\n", n + 1, dsk_slot_size[n] >> 10);
}

/* The CIA hooks exist only for the virtual drives: with no image inserted they
   do nothing and the normal Emu68 bus path handles the access unchanged, so a
   WHDLoad game without a floppy never sees them. */
int aga_cia_write(uint64_t far, int size, uint64_t value)
{
    if (!aga_enabled || !aga || !aga_disk_active(aga)) return 0;
    if (far == 0xBFD100 && (size == 1 || size == 2)) {
        /* CIA-B lives on the upper data byte: a word access carries PRB in bits 8-15 */
        uint8_t prb = (size == 1) ? (uint8_t)value : (uint8_t)(value >> 8);
        lock();
        aga_disk_cia_prb(aga, prb);
        uint8_t mask = aga_disk_cia_prb_mask(aga);
        unlock();
        prb |= mask;                               /* virtual drives stay deselected on the real bus */
        if (size == 1) ps_write_8(0xBFD100, prb);
        else           ps_write_16(0xBFD100, ((unsigned)prb << 8) | (value & 0xFF));
        return 1;
    }
    return 0;
}

int aga_cia_read(uint64_t far, int size, uint64_t *value)
{
    if (!aga_enabled || !aga || !aga_disk_active(aga)) return 0;
    if (size == 2 && far == 0xBFE000) {
        /* A WORD read spanning $BFE000-$BFE001. The CIA sits on the odd byte
           lane, so the low half is the register and the high half is whatever
           the bus floats. Kickstart and bootblocks do use this form, and until
           now it fell straight through to the real bus, where the virtual
           drive's READY, TRACK0, WPROT and CHANGE bits don't exist - so a
           machine booting from the virtual floppy polled a drive that never
           fucking answered. */
        uint16_t w = (uint16_t)ps_read_16(0xBFE000);
        uint8_t bits;
        lock();
        int virt = aga_disk_cia_pra(aga, &bits);
        unlock();
        if (virt) w = (uint16_t)((w & ~0x3C) | (bits & 0x3C));
        *value = w;
        return 1;
    }
    if (size != 1) return 0;
    if (far == 0xBFE001) {
        uint8_t v = (uint8_t)ps_read_8(0xBFE001), bits;
        lock();
        int virt = aga_disk_cia_pra(aga, &bits);
        unlock();
        if (virt) v = (v & ~0x3C) | (bits & 0x3C);
        *value = v;
        return 1;
    }
    if (far == 0xBFEC01) {
        /* keyboard data: watch for Ctrl+F1..F4 = insert swap slot 1..4 into DF0 */
        uint8_t v = (uint8_t)ps_read_8(0xBFEC01);
        uint8_t raw = (uint8_t)~((v >> 1) | (v << 7));
        int up = raw & 0x80, code = raw & 0x7F;
        if (code == 0x63) key_ctrl = !up;
        else if (!up && key_ctrl && code >= 0x50 && code <= 0x53) slot_insert(code - 0x50);
        *value = v;
        return 1;
    }
    return 0;
}

/* $DFF1F2 = $ADxx with the image address in $DFF1EC/$DFF1EE (68k address of a
   fast RAM buffer) and $DFF1F0 = size in KB | flags << 12 (bit 12: write protect):
     $ADF0/$ADF1  insert the image into DF0/DF1 (also kept as swap slot 1 for DF0)
     $ADC1..$ADC4 load the image into swap slot 1..4 (Ctrl+F1..F4 inserts it into DF0)
     $ADE0/$ADE1  eject DF0/DF1 */
int aga_disk_command(unsigned cmd)
{
    aga_diag_inc(AGA_D_DISK_CMDS);
    if (!aga_enabled || !aga) { kprintf("[AGA] disk command %02x ignored (chipset not active)\n", cmd); return 0; }
    unsigned op = (cmd >> 4) & 0xF, n = cmd & 0xF;
    if (op == 0xE) {
        lock(); aga_disk_eject(aga, n & 1); unlock();
        kprintf("[AGA] disk: DF%d ejected\n", n & 1);
        return 1;
    }
    uint32_t addr = ((uint32_t)aga_peek_reg(aga, 0x1EC) << 16) | aga_peek_reg(aga, 0x1EE);
    uint32_t szf = aga_peek_reg(aga, 0x1F0);
    uint32_t size = (szf & 0xFFF) << 10;
    int wprot = (szf >> 12) & 1;
    if (addr < 0x200000 || addr + size > 0xFFFFFFFFu || (size != 901120 && size != 1802240)) {
        kprintf("[AGA] disk command %02x: bad image %08x/%d\n", cmd, addr, size);
        return 0;
    }
    const uint8_t *src = (const uint8_t *)(uintptr_t)addr;   /* 68k fast RAM is Pi memory, identity mapped */
    if (op == 0xF && n < 2) {
        lock(); int r = aga_disk_insert(aga, n, src, size, wprot); unlock();
        if (n == 0 && r == 0) {
            if (!dsk_slot[0]) dsk_slot[0] = calloc(1, 1802240);
            if (dsk_slot[0]) { memcpy(dsk_slot[0], src, size); dsk_slot_size[0] = size; }
        }
        kprintf("[AGA] disk: DF%d insert %d KB from %08x -> %d\n", n, size >> 10, addr, r);
        return r == 0;
    }
    if (op == 0xC && n >= 1 && n <= DSK_SLOTS) {
        if (!dsk_slot[n - 1]) dsk_slot[n - 1] = calloc(1, 1802240);
        if (!dsk_slot[n - 1]) return 0;
        memcpy(dsk_slot[n - 1], src, size);
        dsk_slot_size[n - 1] = size;
        kprintf("[AGA] disk: slot %d loaded (%d KB)\n", n, size >> 10);
        return 1;
    }
    return 0;
}

void aga_machine_reset(void)
{
    if (!aga_enabled || !aga) return;
    lock();
    uint16_t ctrl = aga_peek_reg(aga, 0x1F2), win[4];
    for (int i = 0; i < 4; i++) win[i] = aga_peek_reg(aga, 0x1F4 + 2 * i);
    aga_reset(aga);                           /* registers; disks stay inserted */
    for (int i = 0; i < 4; i++) aga_custom_wput(aga, 0x1F4 + 2 * i, win[i]);
    aga_custom_wput(aga, 0x1F2, ctrl);        /* keep the fullscreen/windowed choice */
    unlock();
    key_ctrl = 0;
    kprintf("[AGA] machine reset on the virtual chipset (ctrl %04x)\n", ctrl);
}
