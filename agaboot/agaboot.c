/* agaboot - switch the PiStorm between AGA and ECS mode from the desktop.
 *
 *   agaboot AGA [ASK]   reboot the Pi with the firmware's one-shot "tryboot"
 *                       flag, so the next boot uses tryboot.txt (AGA) once
 *   agaboot ECS [ASK]   plain Pi reboot: config.txt (ECS) as usual
 *   agaboot MODE        print the active mode; rc 0 = ECS, rc 5 = AGA
 *   agaboot SANDBOX ON  (ECS boot) switch the running machine onto the virtual
 *                       AGA chipset for a WHDLoad game: Emu68 copies the real
 *                       chip RAM into the virtual one and routes the custom
 *                       registers to the emulated chipset; graphics.library is
 *                       told the machine is AGA (ChipRevBits0) so AGA slaves pass
 *   agaboot SANDBOX OFF back to the real chipset (WHDLoad has restored the OS)
 *   agaboot SANDBOX ?   print the sandbox state
 *   agaboot RUN program  launch one ordinary AGA program in a scoped sandbox;
 *                       physical HAM6 remains on the previous display until
 *                       the child installs a stable Copper list, then restores
 *                       automatically when the child exits
 *   agaboot ECSGAMES AUTO|ON|OFF|ASK
 *                       which chipset the sandbox presents. AUTO (the
 *                       default) reads the WHDLoad slave in the game's own
 *                       directory and gives an ECS/OCS game an ECS Denise,
 *                       which is what stops its palette coming out dark;
 *                       anything that wants AGA, or cannot be judged, gets
 *                       AGA. ON/OFF force one for every game
 *   agaboot MENU        add/refresh a "Chipset" title holding the "AGA toggle"
 *                       requester (Automatic / ECS mode / off) and "About", in the desktop
 *                       menus: the Dopus Magellan user menu
 *                       (DOpus5:Buttons/user menu) and S:ToolsDaemon.menu,
 *                       whichever exist. Run it at boot from S:User-Startup,
 *                       before Dopus/ToolsDaemon start
 *
 * ASK pops a requester first, and does nothing if the mode asked for is
 * already active. The switch request is a word written to the AGA-PISTORM
 * control register $DFF1F2, which the patched Emu68 recognises in both modes;
 * on a stock Emu68 the write is harmless, nothing happens.
 * The active mode is detected from DENISEID ($DFF07C): $F8 = Lisa (AGA).
 *   agaboot ABOUT       who made it, and the licences, in a small window
 * Build: m68k-amigaos-gcc -O2 -noixemul -o agaboot agaboot.c dopusmenu.c
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <exec/types.h>
#include <exec/memory.h>
#include <exec/execbase.h>
#include <intuition/intuition.h>
#include <proto/exec.h>
#include <proto/dos.h>
#include <dos/dostags.h>
#include <dos/dosextens.h>      /* FileInfoBlock, DOS_FIB: the slave scan */
#include <proto/intuition.h>
#include <graphics/gfx.h>
#include <graphics/view.h>       /* V0.3 exact hardware Copper list */
#include <graphics/copper.h>     /* struct cprlist::start */
#include <graphics/displayinfo.h>
#include <graphics/modeid.h>
#include <intuition/screens.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define AGA_CTRL  (*(volatile UWORD *)0xDFF1F2)
/* FRF_AGA_SYSTEMWIDE_HAM6_OS_HANDOVER_V0_1_1: one-shot OS Copper pointer; consumed only while sandbox is active. */
#define AGA_SYS_COPPER (*(volatile ULONG *)0xDFF1F0)

#define AGA_RING  (*(volatile ULONG *)0xDFF1EA)   /* hand the audio ring to Emu68 */
#define AGA_HAM_DESC (*(volatile ULONG *)0xDFF1F4)   /* FRF_AGA_DIRECTCHIP_HAM6_V0_1: physical HAM screen descriptor */
/* The kernel's diagnostic counters, the way agastat reads them: select, then
   two words. The two below say which Pi the kernel was built for and which
   one it found; arm/aga_diag.c pins their indices with a _Static_assert. */
#define DIAG_SEL_W    (*(volatile UWORD *)0xDFF1E8)
#define DIAG_DAT_W    (*(volatile UWORD *)0xDFF1EA)
#define D_BUILD_BOARD 147
#define D_RUN_BOARD   148
#define RING_BYTES 8192                            /* software fallback: four 2 KB Paula loops */
#define RING_VAR  "AGA_AUDIORING"                  /* ENV: the buffer, so OFF can free it */
/* FRF_AGA_NATIVE_PAULA_V0_1 */
#define AUDIOBUF_SIZE_VAR "AGA_AUDIOBUF_SIZE"
#define AUDIO_ENGINE_VAR  "AGA_AUDIO_ENGINE"       /* ENVARC: SOFTWARE/NATIVE/AUTO */
#define AUDIOBUF_MIN_NATIVE 32768UL
#define AUDIOBUF_PHYS_LIMIT 0x00080000UL            /* this branch has 512K DMA-visible Chip RAM */
#define DENISEID  (*(volatile UWORD *)0xDFF07C)
#define MAGIC_AGA 0xA6A1
#define MAGIC_ECS 0xEC50
#define MAGIC_SANDBOX_ON  0x5A0B
#define MAGIC_SANDBOX_GAME 0x5A0C   /* same entry, but for a WHDLoad game */
#define MAGIC_SANDBOX_PROGRAM 0x5A0D /* ordinary AGA child; physical HAM waits for child's Copper */
#define MAGIC_PROGRAM_PREP 0x5A1D    /* pre-entry: gate CPU1 before descriptor publication */
#define MAGIC_PROGRAM_CANCEL 0x5A1E  /* abort/failure cleanup */
#define MAGIC_SANDBOX_OFF 0x5A0F

/* Which entry word `sandbox("ON")` sends. The two differ in one respect: a game
   gets the real Paula's EXTER left alone, because WHDLoad installs the game's
   own vector table and a game fills in only the levels it uses. Forcing EXTER
   on - the desktop needs it or its CIAs go silent - sent ClockwiserAGA through
   a zero level-6 vector into address 0. */
static UWORD enter_magic = MAGIC_SANDBOX_ON;
#include <hardware/custom.h>
#include <hardware/dmabits.h>
#define CUSTOM (*(volatile struct Custom *)0xDFF000)

#define GAMEMODE_VAR "AGA_GAMEMODE"        /* ENV: 0 = leave the CIAs as the OS left them */
#define SCR_VAR     "AGA_SETCHIPREV"      /* ENV: graphics' SetChipRev vector while my stub is in */
#define STUB_VAR    "AGA_SCRSTUB"         /* ENV: the stub's memory, freed at OFF */
#define LVO_SetChipRev (-888)
#define CHIPREV_VAR "AGA_CHIPREV"          /* ENV: saved ChipRevBits0 while sandboxed */
/* ENVARC:, so it survives a reboot. The WHDLoad hook runs `SANDBOX AUTO`,
   which does nothing unless this is set: with it clear, every game runs on
   the real chipset as before this project existed - how the machine should
   sit when nobody is debugging it. */
#define ARM_VAR   "AGA_SANDBOX"
static int armed(void);   /* defined below, used by the menu writers */
#define GFXF_AA (0x04 | 0x08 | 0x10)       /* AA_ALICE | AA_LISA | AA_MLISA */

#define MENU_FILE  "S:ToolsDaemon.menu"
#define MENU_TAG   "TITLE Chipset"          /* the block I own in the menu file */
#define DOPUS_MENU "DOpus5:Buttons/user menu"
#define DOPUS_BAK  "DOpus5:Buttons/user menu.pre-aga"

int dopus_menu_update(const char *path, const char *title, const char *label_ecs, const char *cmd_ecs,
                      const char *label_aga, const char *cmd_aga,
                      const char *const *extra, int n_extra);               /* dopusmenu.c */

/* The Chipset menu is ONE item. The emulated AGA chipset only makes sense
   with the desktop on RTG, which is how CaffeineOS runs, so the machine-mode
   reboots and ADF boots were entries nobody should pick. The item opens a
   requester that says what games run on now and offers the three answers
   worth having: Automatic, ECS mode, and off. A second chipset item was tried
   and ripped the hell out - two switches side by side, where leaving one set
   silently broke the other, is how AGA games ended up with ECS colours. Every
   command is still there from the Shell (agaboot ECS|AGA, agadisk). */
#define MENU_LABEL "AGA toggle"
#define MENU_CMD   "C:agaboot AGAGAMES ASK"
/* The second item only shows who made it and the licences: it changes no
   state, so it cannot become the second switch the note above warns about. */
#define ABOUT_LABEL "About"
#define ABOUT_CMD   "C:agaboot ABOUT"
#define AGA_PISTORM_VERSION "1.1"
/* The one item is a mode picker, not an on/off switch: Automatic, ECS mode,
   Turn off. Which chipset the emulation presents matters because the AGA here
   is faithful - WinUAE's COLOR_WRITE does bit for bit what mine does - yet
   some ECS games come out at a sixteenth brightness through it, because
   something in the sandbox leaves BPLCON3's LOCT bit set and every COLORxx
   write then lands in the low nibbles. Games on a real A1200 do not do this
   (no OCS/ECS title in the snapshot corpus writes LOCT at all), so the writer
   is mine and is still unidentified. Until it is found, an ECS Denise - no
   LOCT, no colour bank, no sprite colour offset - delivers the palette whole.
   Normally the slave decides per game and this item just says so; it is here
   for the game the slave gets wrong, in either direction. */

static int file_exists(const char *path)
{
    BPTR l = Lock((STRPTR)path, ACCESS_READ);
    if (l) UnLock(l);
    return l != 0;
}

static void copy_file(const char *from, const char *to)
{
    static char buf[8192];
    BPTR i = Open((STRPTR)from, MODE_OLDFILE), o = i ? Open((STRPTR)to, MODE_NEWFILE) : 0;
    if (i && o) { long n; while ((n = Read(i, buf, sizeof buf)) > 0) Write(o, buf, n); }
    if (i) Close(i);
    if (o) Close(o);
}

struct IntuitionBase *IntuitionBase;


static int aga_active(void)
{
    int lisa = 0;
    for (int i = 0; i < 4; i++)                 /* the id is stable on Lisa, floating on OCS */
        if ((DENISEID & 0xFF) == 0xF8) lisa++;
    return lisa == 4;
}

static int ask(const char *text)
{
    struct EasyStruct es = { sizeof(struct EasyStruct), 0, (UBYTE *)"AGA-PISTORM",
                             (UBYTE *)text, (UBYTE *)"Reboot now|Cancel" };
    IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
    if (!IntuitionBase) return 1;
    int r = EasyRequestArgs(NULL, &es, NULL, NULL);
    CloseLibrary((struct Library *)IntuitionBase);
    return r == 1;
}

static int write_menu(const char *path, int remove)
{
    static char buf[16384];
    long len = 0;
    BPTR f = Open((STRPTR)path, MODE_OLDFILE);
    if (f) { len = Read(f, buf, sizeof buf - 1); Close(f); if (len < 0) len = 0; }
    buf[len] = 0;

    /* cut my previous block: from MENU_TAG up to the next TITLE or END */
    char *start = strstr(buf, MENU_TAG);
    if (start) {
        char *end = start + strlen(MENU_TAG);
        for (;;) {
            char *nl = strchr(end, '\n');
            if (!nl) { end = buf + strlen(buf); break; }
            end = nl + 1;
            if (!strncmp(end, "TITLE ", 6) || !strncmp(end, "END", 3)) break;
        }
        memmove(start, end, strlen(end) + 1);
    }
    /* insert before the final END (or append) */
    char *at = strstr(buf, "\nEND");
    at = at ? at + 1 : buf + strlen(buf);
    char block[512];
    if (remove) block[0] = 0;                    /* uninstall: the title just goes */
    else sprintf(block,
            MENU_TAG "\n"
            "\tITEM " MENU_LABEL "\n"
            "\t(CLI) 4096 " MENU_CMD "\n"
            "\tITEM " ABOUT_LABEL "\n"
            "\t(CLI) 4096 " ABOUT_CMD "\n");
    char *rest = at;
    static char out[16384 + 512];
    long head = at - buf;
    memcpy(out, buf, head);
    strcpy(out + head, block);
    strcat(out, rest);

    f = Open((STRPTR)path, MODE_NEWFILE);
    if (!f) return 0;
    Write(f, out, strlen(out));
    Close(f);
    return 1;
}

#include <graphics/gfxbase.h>
#include <proto/graphics.h>
struct GfxBase *GfxBase;
int ptr_probe(const char *tag);      /* pointer.c */

/* The virtual chipset is seeded with DMACON MINUS copper/bitplane/sprite
   (COP1LC is write-only, so a faithful copy runs the copper from address 0).
   graphics.library switched those on at boot on the real chip and never
   touches them again, so under the sandbox no OS copper list ever ran:
   DMACON $0250 forever, BPLCON0 never written, dmatest blank. The OS does
   keep its own copper pointer: copinit is the list it shows when there is no
   view. Point the virtual copper at it and turn the channels on. A WHDLoad
   game overwrites all of this anyway. */
static void display_on(struct GfxBase *gb)
{
    Disable();
    CUSTOM.cop1lc = (ULONG)gb->copinit;
    CUSTOM.copjmp1 = 0;
    CUSTOM.dmacon = DMAF_SETCLR | DMAF_MASTER | DMAF_COPPER | DMAF_RASTER | DMAF_SPRITE;
    Enable();
}

/* Give the audio ring back. Safe to call when there is none. Only ever call
   it after Emu68 has been told to leave the sandbox (or refused to enter):
   aga_audio_stop() on the ARM side drops its pointer first, so there is no
   window where the chipset loop still streams into freed memory. */
static void free_ring(void)
{
    char v[16];
    if (GetVar((STRPTR)RING_VAR, v, sizeof v, 0) <= 0) return;
    APTR ring = (APTR)strtoul(v, NULL, 10);
    ULONG bytes = RING_BYTES;
    char sv[16];
    if (GetVar((STRPTR)AUDIOBUF_SIZE_VAR, sv, sizeof sv, 0) > 0)
        bytes = strtoul(sv, NULL, 10);
    if (bytes < RING_BYTES || bytes > 131072UL) bytes = RING_BYTES;
    if (ring) FreeMem(ring, bytes);
    DeleteVar((STRPTR)RING_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)AUDIOBUF_SIZE_VAR, GVF_GLOBAL_ONLY);
}


/* FRF_AGA_NATIVE_PAULA_V0_1 -------------------------------------------------
 * SOFTWARE = existing mixed PCM ring.
 * NATIVE   = real Paula DMA from an owned physical-Chip staging pool.
 * AUTO     = try NATIVE, fall back to SOFTWARE if the pool/sample cannot fit.
 */
static unsigned audio_engine_pref(void)
{
    char v[24]; LONG n = GetVar((STRPTR)AUDIO_ENGINE_VAR, v, sizeof v, 0);
    if (n <= 0) return 2u; /* AUTO */
    v[(n < (LONG)sizeof(v)) ? n : (LONG)sizeof(v)-1] = 0;
    if (!stricmp(v, "SOFTWARE")) return 0u;
    if (!stricmp(v, "NATIVE")) return 1u;
    return 2u;
}

static const char *audio_engine_name(unsigned mode)
{
    return mode == 0u ? "SOFTWARE" : mode == 1u ? "NATIVE" : "AUTO";
}

static void audio_engine_pref_set(unsigned mode)
{
    STRPTR v=(STRPTR)audio_engine_name(mode);
    SetVar((STRPTR)AUDIO_ENGINE_VAR,v,-1,GVF_GLOBAL_ONLY);
    SetVar((STRPTR)AUDIO_ENGINE_VAR,v,-1,GVF_GLOBAL_ONLY|GVF_SAVE_VAR);
}

static unsigned audio_buf_size_code(ULONG bytes)
{
    if (bytes >= 98304UL) return 4u;
    if (bytes >= 65536UL) return 3u;
    if (bytes >= 49152UL) return 2u;
    if (bytes >= 32768UL) return 1u;
    return 0u; /* 8192 */
}

static APTR alloc_real_chip_audio(ULONG bytes)
{
    APTR p=AllocMem(bytes,MEMF_CHIP|MEMF_PUBLIC|MEMF_CLEAR);
    ULONG a=(ULONG)p;
    if (!p) return NULL;
    /* AGALEND makes Exec believe virtual Chip exists above 512K. Real Paula
       cannot DMA it. Reject it rather than silently feeding Paula nonsense. */
    if (a < 0x400UL || a + bytes > AUDIOBUF_PHYS_LIMIT || (a & 3UL)) {
        FreeMem(p,bytes);
        return NULL;
    }
    return p;
}

static APTR audio_buffer_prepare(unsigned requested, unsigned *session_mode, ULONG *bytes_out)
{
    APTR p=NULL; ULONG bytes=RING_BYTES;
    *session_mode=requested;

    if (requested != 0u) {
        static const ULONG candidates[] = { 98304UL, 65536UL, 49152UL, 32768UL };
        for (unsigned i=0;i<sizeof(candidates)/sizeof(candidates[0]);++i) {
            p=alloc_real_chip_audio(candidates[i]);
            if (p) { bytes=candidates[i]; break; }
        }
        if (!p) {
            PutStr("agaboot: no low physical CHIP pool for native Paula; using software audio\n");
            *session_mode=0u;
        }
    }
    if (!p) {
        bytes=RING_BYTES;
        p=alloc_real_chip_audio(bytes);
    }
    if (!p) return NULL;

    {
        char v[20];
        sprintf(v,"%lu",(unsigned long)p); SetVar((STRPTR)RING_VAR,v,-1,GVF_GLOBAL_ONLY);
        sprintf(v,"%lu",(unsigned long)bytes); SetVar((STRPTR)AUDIOBUF_SIZE_VAR,v,-1,GVF_GLOBAL_ONLY);
    }
    *bytes_out=bytes;
    return p;
}


/* FRF_AGA_DIRECTCHIP_HAM6_V0_1_AGABOOT_HELPERS_BEGIN */
#define OUTPUT_VAR      "AGA_OUTPUT"
#define HAM_SCREEN_VAR  "AGA_HAM_SCREEN"
#define HAM_DESC_VAR    "AGA_HAM_DESC"
#define ECSOUTPUT_VAR "AGA_ECSOUTPUT"
#define ECS_DENISE_MAGIC 0x45435344UL /* ECSD */
#define ECS_DENISE_VERSION 1UL
#define ECS_DENISE_W 640UL
#define ECS_DENISE_H 286UL
#define ECS_DENISE_BPR 80UL
#define ECS_DENISE_PLANE_BYTES (ECS_DENISE_BPR * ECS_DENISE_H)
#define ECS_DENISE_PLANES 6
/* FRF_AGA_ECS_DENISE_BRIDGE_V0_1 */
#define HAMAREA_VAR     "AGA_HAMAREA"
#define HAMBUFFER_VAR   "AGA_HAMBUFFER"
#define HAM_BACK_VAR    "AGA_HAM_BACKBITMAP"
#define HAM_EXT_VAR     "AGA_HAMBUFFER_EXT"
#define HAM_VIEW_VAR    "AGA_HAM_HIDDEN_VIEW" /* RUN-only private View/Copper owner */
#define HAMBUFFER_EXT_MAGIC 0x48424631UL /* HBF1 */
#define HAMBUFFER_EXT_LONGS 9UL
#define HAMBUFFER_EXT_BYTES (HAMBUFFER_EXT_LONGS * sizeof(ULONG))
/* FRF_AGA_RGB_TAP_HAM6_V0_15_2_HAMBUFFER */
/* FRF_AGA_RGB_TAP_HAM6_V0_14_1_HAMAREA_DYNAMIC_RC10 */
#define HAMFPS_VAR      "AGA_HAMFPS"
/* FRF_AGA_RGB_TAP_HAM6_V0_12_0_HAMFPS */
#define HAM_DESC_BYTES  56UL
#define HAM_DESC_MAGIC  0x48414D36UL
#define HAM_W           320UL
#define HAM_H           256UL

static const UBYTE ham_base_rgb[16][3] = {
    {  0,  0,  0}, {255,255,255}, { 85, 85, 85}, {170,170,170},
    {255,  0,  0}, {  0,255,  0}, {  0,  0,255}, {255,255,  0},
    {  0,255,255}, {255,  0,255}, {128,  0,  0}, {  0,128,  0},
    {  0,  0,128}, {128,128,  0}, {  0,128,128}, {128,  0,128}
};

static int output_ham6(void)
{
    char v[16];
    if (GetVar((STRPTR)OUTPUT_VAR, v, sizeof v, 0) <= 0) return 0;
    return !stricmp(v, "HAM6");
}

static void output_set(int ham)
{
    STRPTR v = (STRPTR)(ham ? "HAM6" : "HDMI");
    SetVar((STRPTR)OUTPUT_VAR, v, -1, GVF_GLOBAL_ONLY);
    SetVar((STRPTR)OUTPUT_VAR, v, -1, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
}

static void ham_env_ptr(const char *name, APTR p)
{
    char v[20];
    sprintf(v, "%lu", (unsigned long)p);
    SetVar((STRPTR)name, v, -1, GVF_GLOBAL_ONLY);
}

static APTR ham_get_ptr(const char *name)
{
    char v[20];
    if (GetVar((STRPTR)name, v, sizeof v, 0) <= 0) return NULL;
    return (APTR)strtoul(v, NULL, 10);
}

static ULONG ham_copper_bytes(const UWORD *cp);

/* FRF_AGA_PROGRAM_SCOPE_V0_2_HIDDEN_HAM_VIEW
 *
 * RUN must allocate the real, DMA-visible HAM bitmap before the virtual AGA
 * sandbox is entered, but opening/preparing that bitmap must NOT visibly
 * replace the user's current Workbench/Magellan display.  A normal Intuition
 * screen's final Copper list belongs to the global loaded View, so taking
 * GfxBase->ActiView->LOFCprList after ScreenToFront() necessarily causes the
 * premature takeover we are trying to avoid.
 *
 * Instead RUN opens the HAM screen behind all existing screens. MakeScreen()
 * is allowed because it only rebuilds that Screen's intermediate ViewPort
 * Copper lists; without RethinkDisplay()/LoadView() it does not install them.
 * We then shallow-copy that prepared ViewPort into a private one-ViewPort View
 * and call MrgCop() only.  The private View owns only its final LOF/SHF lists;
 * Intuition keeps ownership of the Screen's intermediate Dsp/Spr/Clr lists.
 * CPU1 can later install the private LOF directly at the exact moment the
 * child program's virtual AGA display becomes stable.
 */
struct ham_hidden_view {
    struct View view;
    struct ViewPort vp;
    struct RasInfo ri;
};

static void ham_hidden_view_free(struct ham_hidden_view *hv)
{
    if (!hv) return;
    if (hv->view.LOFCprList) {
        FreeCprList(hv->view.LOFCprList);
        hv->view.LOFCprList = NULL;
    }
    if (hv->view.SHFCprList) {
        FreeCprList(hv->view.SHFCprList);
        hv->view.SHFCprList = NULL;
    }
    /* Do NOT FreeVPortCopLists(&hv->vp): those are the Screen's Intuition-
       owned intermediate lists copied by pointer into this private ViewPort. */
    FreeMem(hv, sizeof(*hv));
}

static struct ham_hidden_view *ham_hidden_view_build(struct Screen *scr,
                                                      struct BitMap *bm,
                                                      ULONG *copper_out)
{
    struct ham_hidden_view *hv;
    ULONG copper, bytes;

    if (!scr || !bm || !scr->ViewPort.RasInfo || !copper_out) return NULL;

    hv = (struct ham_hidden_view *)AllocMem(sizeof(*hv), MEMF_PUBLIC | MEMF_CLEAR);
    if (!hv) return NULL;

    InitView(&hv->view);

    /* MakeScreen(scr) has already rebuilt these intermediate lists while the
       Screen remains behind the visible desktop.  Borrow them read-only for
       MrgCop(), but isolate the ViewPort/RasInfo chain itself. */
    hv->vp = scr->ViewPort;
    hv->vp.Next = NULL;
    hv->ri = *scr->ViewPort.RasInfo;
    hv->ri.BitMap = bm;
    hv->ri.Next = NULL;
    hv->vp.RasInfo = &hv->ri;
    hv->view.ViewPort = &hv->vp;

    if (!hv->vp.DspIns) {
        PutStr("agaboot: RUN hidden HAM screen has no prepared display Copper list\n");
        ham_hidden_view_free(hv);
        return NULL;
    }
    if (MrgCop(&hv->view) != 0 || !hv->view.LOFCprList ||
        !hv->view.LOFCprList->start) {
        PutStr("agaboot: RUN could not build hidden HAM Copper list\n");
        ham_hidden_view_free(hv);
        return NULL;
    }
    if (hv->view.SHFCprList && hv->view.SHFCprList->start) {
        PutStr("agaboot: RUN hidden HAM sink unexpectedly became interlaced\n");
        ham_hidden_view_free(hv);
        return NULL;
    }

    copper = (ULONG)hv->view.LOFCprList->start;
    bytes = ham_copper_bytes((const UWORD *)copper);
    if (!copper || (copper & 1UL) || copper >= 0x00080000UL || !bytes ||
        copper + bytes > 0x00080000UL || copper + bytes < copper) {
        Printf("agaboot: RUN hidden HAM Copper not wholly inside real 512K CHIP ($%08lx, %ld bytes)\n",
               copper, (LONG)bytes);
        ham_hidden_view_free(hv);
        return NULL;
    }

    *copper_out = copper;
    return hv;
}


static void ham_release(void)
{
    struct Screen *scr = (struct Screen *)ham_get_ptr(HAM_SCREEN_VAR);
    struct BitMap *back = (struct BitMap *)ham_get_ptr(HAM_BACK_VAR);
    struct ham_hidden_view *hidden =
        (struct ham_hidden_view *)ham_get_ptr(HAM_VIEW_VAR);
    APTR ext = ham_get_ptr(HAM_EXT_VAR);
    APTR desc = ham_get_ptr(HAM_DESC_VAR);

    /* The private View points at the screen's ColorMap/BitMap, so release its
       generated Copper resources before Intuition owns/frees the screen. */
    if (hidden) ham_hidden_view_free(hidden);
    if (back) FreeBitMap(back);
    if (scr) {
        struct IntuitionBase *ib = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
        if (ib) {
            IntuitionBase = ib;
            CloseScreen(scr);
            CloseLibrary((struct Library *)ib);
            IntuitionBase = NULL;
        } else {
            PutStr("agaboot: warning: could not reopen intuition.library to close HAM screen\n");
        }
    }
    if (ext) FreeMem(ext, HAMBUFFER_EXT_BYTES);
    if (desc && ((ULONG *)desc)[0] == ECS_DENISE_MAGIC) {
        for (int p = 0; p < ECS_DENISE_PLANES; ++p) {
            APTR plane = (APTR)((ULONG *)desc)[6+p];
            if (plane) FreeMem(plane, ECS_DENISE_PLANE_BYTES);
        }
    }
    if (desc) FreeMem(desc, HAM_DESC_BYTES);
    DeleteVar((STRPTR)HAM_SCREEN_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)HAM_DESC_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)HAM_BACK_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)HAM_EXT_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)HAM_VIEW_VAR, GVF_GLOBAL_ONLY);
    DeleteVar((STRPTR)"AGA_HAM_BITMAP", GVF_GLOBAL_ONLY);
}



/* Open the exact kind of physical display our proven native HAM renderers use.
 * Intuition/graphics.library owns the timing and Copper construction. CPU3 is
 * handed only the physical plane geometry, never an OS structure pointer. */
/* Count one raw graphics.library Copper list. We only accept a normal
 * non-interlaced list ending in FFFF/FFFE within 8 KB. */
static ULONG ham_copper_bytes(const UWORD *cp)
{
    ULONG words;
    if (!cp) return 0;
    for (words = 0; words + 1 < 4096UL; words += 2) {
        if (cp[words] == 0xFFFFU && cp[words + 1] == 0xFFFEU)
            return (words + 2UL) * 2UL;
    }
    return 0;
}

/* Clone keeps every graphics.library timing/palette instruction byte-for-byte.
 * Only the twelve BPL1..6 pointer MOVE data words are changed. */
static int ham_patch_copper_planes(UWORD *cp, ULONG bytes, const ULONG plane[6])
{
    UBYTE seen[12] = {0};
    ULONG words = bytes >> 1;
    for (ULONG i = 0; i + 1 < words; i += 2) {
        UWORD ins = cp[i];
        if (ins & 1U) continue;              /* WAIT/SKIP */
        UWORD reg = ins & 0x01FEU;
        if (reg >= 0x00E0U && reg <= 0x00F6U) {
            unsigned n = (unsigned)((reg - 0x00E0U) >> 1);
            unsigned p = n >> 1;
            unsigned lo = n & 1U;
            if (p < 6U) {
                cp[i + 1] = lo ? (UWORD)(plane[p] & 0xFFFFUL)
                               : (UWORD)(plane[p] >> 16);
                seen[n] = 1;
            }
        }
    }
    for (unsigned i = 0; i < 12U; ++i)
        if (!seen[i]) return 0;
    return 1;
}

/* V0.4: preserve the exact V0.3 front screen, but create one hidden physical
 * page and one exact Copper clone for it. CPU3 never updates the visible page. */
/* FRF_AGA_RGB_TAP_HAM6_V0_7_0_OWNED_SINK
 * Allocate the exact physical bitmap we want instead of accepting whatever
 * private layout OpenScreenTags happens to choose. This is the same ownership
 * strategy already proven by the native MAME HAM path.
 *
 * Interleaved contract:
 *   BytesPerRow = 40
 *   Plane[n] = Plane[0] + n*40
 *   effective next-row stride = 40*6 = 240
 */
/* FRF_AGA_RGB_TAP_HAM6_V0_8_0_RC10_SINK
 * Reuse the runtime-proven Mame106RGB RC10 bitmap-layout policy instead of
 * forcing an owned AllocBitMap/SA_BitMap representation.
 *
 * Accepted physical 320x256x6 layouts:
 *   AGA/V39 canonical interleaved: bm->BytesPerRow=240, planeRow=40, stride=240
 *   legacy interleaved:            bm->BytesPerRow=40,  Plane[n]=base+n*40,
 *                                  stride=40*depth=240
 *   separate planar:               bm->BytesPerRow=40, stride=40
 */
static void hamarea_pref(ULONG *w, ULONG *h)
{
    char v[24];
    LONG n;
    *w = 368; *h = 280;
    n = GetVar((STRPTR)HAMAREA_VAR, v, sizeof v, 0);
    if (n <= 0) return;
    v[(n < (LONG)sizeof(v)) ? n : (LONG)sizeof(v)-1] = 0;
    if (!stricmp(v, "320x256")) { *w=320; *h=256; return; }
    if (!stricmp(v, "352x272")) { *w=352; *h=272; return; }
    if (!stricmp(v, "368x280")) { *w=368; *h=280; return; }
    if (!stricmp(v, "384x280")) { *w=384; *h=280; return; }
}

static void hamarea_pref_set(STRPTR v)
{
    SetVar((STRPTR)HAMAREA_VAR, v, -1, GVF_GLOBAL_ONLY);
    SetVar((STRPTR)HAMAREA_VAR, v, -1, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
}

static int hambuffer_pref(void)
{
    char v[16]; LONG n;
    n=GetVar((STRPTR)HAMBUFFER_VAR,v,sizeof v,0);
    if(n<=0) return 0; /* OFF is the preserved V0.14.1 control case. */
    v[(n<(LONG)sizeof(v))?n:(LONG)sizeof(v)-1]=0;
    return (!stricmp(v,"ON") || !stricmp(v,"1")) ? 1 : 0;
}

static void hambuffer_pref_set(int on)
{
    STRPTR v=(STRPTR)(on?"ON":"OFF");
    SetVar((STRPTR)HAMBUFFER_VAR,v,-1,GVF_GLOBAL_ONLY);
    SetVar((STRPTR)HAMBUFFER_VAR,v,-1,GVF_GLOBAL_ONLY|GVF_SAVE_VAR);
}

static int ham_detect_layout_rc10(struct BitMap *bm,
                                  ULONG target_w,
                                  ULONG target_h,
                                  ULONG *plane_row,
                                  ULONG *row_stride,
                                  UBYTE *interleaved)
{
    ULONG bpr, flags, pr, span, expected_row;
    int p, q, canonical = 0, legacy = 1;

    if (!target_w || target_w > 400UL || (target_w & 15UL) ||
        !target_h || target_h > 286UL)
        return 0;

    expected_row = ((target_w + 15UL) >> 4) * 2UL;

    if (!bm || !plane_row || !row_stride || !interleaved ||
        bm->Depth < 6 || (ULONG)bm->Rows < target_h ||
        (ULONG)bm->BytesPerRow < expected_row)
        return 0;

    for (p = 0; p < 6; ++p)
        if (!bm->Planes[p]) return 0;

    bpr = (ULONG)bm->BytesPerRow;
    flags = GetBitMapAttr(bm, BMA_FLAGS);

    /* Canonical AGA/V39 interleaved: BPR is the whole six-plane row. */
    if ((flags & BMF_INTERLEAVED) && bm->Depth &&
        (bpr % (ULONG)bm->Depth) == 0) {
        pr = bpr / (ULONG)bm->Depth;
        if (pr == expected_row) {
            canonical = 1;
            for (p = 1; p < 6; ++p) {
                if ((UBYTE *)bm->Planes[p] !=
                    (UBYTE *)bm->Planes[0] + (ULONG)p * pr) {
                    canonical = 0;
                    break;
                }
            }
        }
        if (canonical) {
            *plane_row = pr;
            *row_stride = bpr;
            *interleaved = 1;
            return 1;
        }
    }

    /* Legacy A500-style interleaved bitmap: BPR is one plane row. */
    if (bpr == expected_row) {
        for (p = 1; p < 6; ++p) {
            if ((UBYTE *)bm->Planes[p] !=
                (UBYTE *)bm->Planes[0] + (ULONG)p * bpr) {
                legacy = 0;
                break;
            }
        }
        if (legacy) {
            *plane_row = bpr;
            *row_stride = bpr * (ULONG)bm->Depth;
            *interleaved = 1;
            return 1;
        }
    }

    /* Separate planar bitmap. Reject overlapping planes. */
    if (bpr != expected_row) return 0;
    span = bpr * (ULONG)bm->Rows;
    for (p = 0; p < 6; ++p) {
        ULONG pa = (ULONG)bm->Planes[p];
        for (q = p + 1; q < 6; ++q) {
            ULONG pb = (ULONG)bm->Planes[q];
            if (pa < pb + span && pb < pa + span)
                return 0;
        }
    }
    *plane_row = bpr;
    *row_stride = bpr;
    *interleaved = 0;
    return 1;
}

static int hamfps_pref(void)
{
    char v[16];
    if (GetVar((STRPTR)HAMFPS_VAR, v, sizeof v, 0) <= 0) return 25;
    if (v[0] == '5' && v[1] == '0') return 50;
    return 25;
}

static void hamfps_pref_set(int fps)
{
    STRPTR v = (STRPTR)(fps == 50 ? "50" : "25");
    SetVar((STRPTR)HAMFPS_VAR, v, -1, GVF_GLOBAL_ONLY);
    SetVar((STRPTR)HAMFPS_VAR, v, -1, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
}

static void hamfps_apply_live(int fps)
{
    ULONG *desc = (ULONG *)ham_get_ptr(HAM_DESC_VAR);
    if (!desc) return;
    desc[5] = (desc[5] & 0x000000ffUL) | ((ULONG)fps << 8);
    CacheClearU();
}

/* V0.1: AUTO deliberately prefers the emulated ECS + real Denise bridge on
 * this FRF branch. Exec AvailMem(MEMF_CHIP) is not a trustworthy physical-RAM
 * detector here because AGALEND/virtual Chip RAM can make it report >512K.
 * NATIVE is therefore an explicit override for machines known to have enough
 * genuinely DMA-visible Chip RAM. */
static int ecs_for_this_game(void);

static int ecsoutput_pref(void)
{
    char v[24]; LONG n=GetVar((STRPTR)ECSOUTPUT_VAR,v,sizeof v,0);
    if(n<=0) return 1; /* AUTO -> DENISE on the 512K physical-sink branch. */
    v[(n<(LONG)sizeof(v))?n:(LONG)sizeof(v)-1]=0;
    if(!stricmp(v,"NATIVE")) return 0;
    if(!stricmp(v,"DENISE")) return 1;
    return 1; /* AUTO */
}

static void ecsoutput_pref_set(STRPTR v)
{
    if(!stricmp(v,"AUTO")) {
        DeleteVar((STRPTR)ECSOUTPUT_VAR,GVF_GLOBAL_ONLY);
        DeleteVar((STRPTR)ECSOUTPUT_VAR,GVF_GLOBAL_ONLY|GVF_SAVE_VAR);
        return;
    }
    SetVar((STRPTR)ECSOUTPUT_VAR,v,-1,GVF_GLOBAL_ONLY);
    SetVar((STRPTR)ECSOUTPUT_VAR,v,-1,GVF_GLOBAL_ONLY|GVF_SAVE_VAR);
}

static int ecs_denise_for_this_game(void)
{
    return ecs_for_this_game() && ecsoutput_pref();
}

static APTR ecs_denise_prepare(void)
{
    ULONG *desc=NULL;
    ULONG copper=0;
    int allocated=0;

    if(!GfxBase || !GfxBase->ActiView || !GfxBase->ActiView->LOFCprList ||
       !GfxBase->ActiView->LOFCprList->start) {
        PutStr("agaboot: ECS-DENISE cannot capture the current physical Copper for restore\n");
        return NULL;
    }
    copper=(ULONG)GfxBase->ActiView->LOFCprList->start;
    if(!copper || (copper&1UL) || copper>=0x00080000UL) {
        Printf("agaboot: ECS-DENISE restore Copper is not in real 512K CHIP: $%08lx\n",copper);
        return NULL;
    }

    desc=(ULONG*)AllocMem(HAM_DESC_BYTES,MEMF_CHIP|MEMF_PUBLIC|MEMF_CLEAR);
    if(!desc || (ULONG)desc>=0x00080000UL ||
       (ULONG)desc+HAM_DESC_BYTES>0x00080000UL) {
        if(desc) FreeMem(desc,HAM_DESC_BYTES);
        PutStr("agaboot: ECS-DENISE has no real 512K CHIP for descriptor\n");
        return NULL;
    }

    desc[0]=ECS_DENISE_MAGIC;
    desc[1]=ECS_DENISE_W;
    desc[2]=ECS_DENISE_H;
    desc[3]=ECS_DENISE_BPR;
    desc[4]=ECS_DENISE_BPR;
    desc[5]=((ULONG)hamfps_pref()<<8);
    desc[12]=copper;
    desc[13]=ECS_DENISE_VERSION;

    for(int p=0;p<ECS_DENISE_PLANES;++p) {
        APTR plane=AllocMem(ECS_DENISE_PLANE_BYTES,MEMF_CHIP|MEMF_PUBLIC|MEMF_CLEAR);
        ULONG pa=(ULONG)plane;
        if(!plane || pa>=0x00080000UL || pa+ECS_DENISE_PLANE_BYTES>0x00080000UL || pa&1UL) {
            if(plane) FreeMem(plane,ECS_DENISE_PLANE_BYTES);
            PutStr("agaboot: ECS-DENISE cannot allocate six 640x286 planes inside real 512K CHIP\n");
            goto fail;
        }
        desc[6+p]=pa;
        ++allocated;
    }

    ham_env_ptr(HAM_DESC_VAR,(APTR)desc); /* reuse proven descriptor handoff */
    Printf("agaboot: ECS-DENISE READY: virtual ECS -> real Denise, 640x286 max, %ld bytes real CHIP, restoreCopper=$%08lx\n",
           (LONG)(ECS_DENISE_PLANE_BYTES*ECS_DENISE_PLANES),copper);
    PutStr("agaboot: native LORES/HIRES/LACE + indexed/EHB/HAM6 presenter; physical Copper held off while active\n");
    return desc;
fail:
    for(int p=0;p<allocated;++p) if(desc[6+p]) FreeMem((APTR)desc[6+p],ECS_DENISE_PLANE_BYTES);
    FreeMem(desc,HAM_DESC_BYTES);
    return NULL;
}

static APTR ham_prepare(void)
{
    if (ecs_denise_for_this_game())
        return ecs_denise_prepare();

    ULONG hamw, hamh;
    ULONG mode = DEFAULT_MONITOR_ID | HAM_KEY;
    struct Screen *scr = NULL;
    struct BitMap *bm = NULL;
    struct BitMap *back = NULL;
    struct ham_hidden_view *hidden = NULL;
    ULONG *desc = NULL;
    ULONG *ext = NULL;
    ULONG plane_row = 0, stride = 0, copper = 0;
    ULONG back_plane_row = 0, back_stride = 0;
    UBYTE interleaved = 0;
    UBYTE back_interleaved = 0;
    int dbuf = hambuffer_pref();
    int scoped = (enter_magic == MAGIC_SANDBOX_PROGRAM);
    hamarea_pref(&hamw, &hamh);

    if (ModeNotAvailable(mode)) {
        PutStr("agaboot: physical LORES HAM6 ModeID unavailable; RGB tap disabled\n");
        return NULL;
    }

    IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
    if (!IntuitionBase) {
        PutStr("agaboot: cannot open intuition.library for RGB HAM sink\n");
        return NULL;
    }

    /* RC10 policy: let graphics/Intuition create the native HAM screen, then
     * classify the actual runtime bitmap. Do NOT force AllocBitMap+SA_BitMap. */
    scr = OpenScreenTags(NULL,
        SA_DisplayID, mode,
        SA_Width, (ULONG)hamw,
        SA_Height, (ULONG)hamh,
        SA_Depth, 6,
        SA_Type, CUSTOMSCREEN,
        /* RUN prepares the real HAM resources without changing the visible
           screen.  Ordinary SANDBOX/WHDLoad keeps its proven front-screen path. */
        SA_Behind, scoped ? TRUE : FALSE,
        SA_Quiet, TRUE,
        SA_ShowTitle, FALSE,
        SA_Draggable, FALSE,
        SA_AutoScroll, FALSE,
        SA_Overscan, OSCAN_MAX,
        TAG_DONE);
    if (!scr) {
        PutStr("agaboot: RC10 OpenScreen failed for physical LORES HAM6 RGB sink\n");
        goto fail;
    }

    bm = scr->RastPort.BitMap ? scr->RastPort.BitMap : &scr->BitMap;
    if (!ham_detect_layout_rc10(bm, hamw, hamh, &plane_row, &stride, &interleaved)) {
        Printf("agaboot: RC10 HAM bitmap layout rejected depth=%ld rows=%ld bpr=%ld flags=$%08lx\n",
               (LONG)bm->Depth, (LONG)bm->Rows, (LONG)bm->BytesPerRow,
               (ULONG)GetBitMapAttr(bm, BMA_FLAGS));
        goto fail;
    }

    /* This particular machine has only 512 KB of REAL DMA-visible Chip RAM.
     * The sandbox may emulate more, but the physical HAM sink must not use it. */
    for (int p = 0; p < 6; ++p) {
        ULONG pa = (ULONG)bm->Planes[p];
        ULONG last = pa + (hamh - 1UL) * stride + plane_row;
        if (!pa || (pa & 1UL) || pa >= 0x00080000UL ||
            last > 0x00080000UL || last < pa) {
            Printf("agaboot: RC10 HAM plane %ld not in real 512K CHIP start=$%08lx end=$%08lx\n",
                   (LONG)p, pa, last);
            goto fail;
        }
    }

    {
        struct RastPort rp;
        InitRastPort(&rp); rp.BitMap = bm;
        SetRast(&rp, 0); WaitBlit();
    }
    for (int i = 0; i < 16; ++i)
        SetRGB4(&scr->ViewPort, i,
                ham_base_rgb[i][0] >> 4,
                ham_base_rgb[i][1] >> 4,
                ham_base_rgb[i][2] >> 4);

    if (dbuf) {
        ULONG bmf = BMF_DISPLAYABLE | BMF_CLEAR;
        if (interleaved) bmf |= BMF_INTERLEAVED;
        back = AllocBitMap(hamw, hamh, 6, bmf, bm);
        if (!back) {
            PutStr("agaboot: HAMBUFFER ON could not allocate hidden HAM bitmap\n");
            goto fail;
        }
        if (!ham_detect_layout_rc10(back, hamw, hamh,
                                    &back_plane_row, &back_stride,
                                    &back_interleaved) ||
            back_plane_row != plane_row || back_stride != stride ||
            back_interleaved != interleaved) {
            PutStr("agaboot: HAMBUFFER hidden bitmap layout does not match physical front\n");
            goto fail;
        }
        for (int p = 0; p < 6; ++p) {
            ULONG pa = (ULONG)back->Planes[p];
            ULONG last = pa + (hamh - 1UL) * back_stride + back_plane_row;
            if (!pa || (pa & 1UL) || pa >= 0x00080000UL ||
                last > 0x00080000UL || last < pa) {
                Printf("agaboot: HAMBUFFER back plane %ld outside real 512K CHIP start=$%08lx end=$%08lx\n",
                       (LONG)p,pa,last);
                goto fail;
            }
        }
    }

    if (scoped) {
        /* PROGRAM-SCOPED TAKEOVER: refresh this hidden Screen's intermediate
           lists, but deliberately do NOT call RethinkDisplay/LoadView/front.
           The current desktop remains untouched until ARM observes two stable
           frames from the child's virtual COP1LC and releases CPU1. */
        MakeScreen(scr);
        hidden = ham_hidden_view_build(scr, bm, &copper);
        if (!hidden) goto fail;
        /* Re-establish Intuition's own intermediate lists defensively after
           the private merge. The private final LOF is already independent. */
        MakeScreen(scr);
        PutStr("agaboot: RUN HAM6 sink prepared behind current display; physical takeover deferred\n");
    } else {
        MakeScreen(scr); RethinkDisplay(); ScreenToFront(scr); WaitTOF();

        /* FRF_AGA_RGB_TAP_HAM6_V0_9_0_OS_FIRST_LIGHT
         * Visible proof that the real lowres HAM screen itself is valid BEFORE
         * ordinary WHDLoad sandbox takeover. Pens 1..15 are direct HAM base
         * palette codes. RUN deliberately skips these visible bars. */
        for (int bar = 0; bar < 16; ++bar) {
            ULONG barw = hamw / 16UL;
            ULONG left = (ULONG)bar * barw;
            ULONG right = (bar == 15) ? (hamw - 1UL) : (left + barw - 1UL);
            SetAPen(&scr->RastPort, (ULONG)bar);
            RectFill(&scr->RastPort, (LONG)left, 0,
                     (LONG)right, (LONG)(hamh - 1UL));
        }
        WaitBlit();
        WaitTOF();
        PutStr("agaboot: V0.9 OS FIRST-LIGHT bars loaded into real HAM6 sink\n");

        if (!GfxBase || !GfxBase->ActiView || !GfxBase->ActiView->LOFCprList ||
            !GfxBase->ActiView->LOFCprList->start) {
            PutStr("agaboot: RC10 physical HAM Copper list unavailable\n");
            goto fail;
        }
        if (GfxBase->ActiView->SHFCprList && GfxBase->ActiView->SHFCprList->start) {
            PutStr("agaboot: RC10 physical HAM sink became interlaced; refusing it\n");
            goto fail;
        }

        copper = (ULONG)GfxBase->ActiView->LOFCprList->start;
        if (!copper || (copper & 1UL) || copper >= 0x00080000UL) {
            Printf("agaboot: RC10 HAM Copper outside real 512K CHIP ($%08lx)\n", copper);
            goto fail;
        }
    }
    if (dbuf) {
        ext = (ULONG *)AllocMem(HAMBUFFER_EXT_BYTES,
                                MEMF_CHIP | MEMF_PUBLIC | MEMF_CLEAR);
        if (!ext || (ULONG)ext >= 0x00080000UL ||
            (ULONG)ext + HAMBUFFER_EXT_BYTES > 0x00080000UL) {
            if (ext) FreeMem(ext,HAMBUFFER_EXT_BYTES);
            ext=NULL;
            PutStr("agaboot: no real 512K CHIP RAM for HAMBUFFER extension\n");
            goto fail;
        }
    }


    desc = (ULONG *)AllocMem(HAM_DESC_BYTES, MEMF_CHIP | MEMF_PUBLIC | MEMF_CLEAR);
    if (!desc || (ULONG)desc >= 0x00080000UL) {
        if (desc) FreeMem(desc, HAM_DESC_BYTES);
        desc = NULL;
        PutStr("agaboot: no real 512K CHIP RAM for HAM descriptor\n");
        goto fail;
    }

    desc[0] = HAM_DESC_MAGIC;
    desc[1] = (ULONG)hamw;
    desc[2] = (ULONG)hamh;
    desc[3] = plane_row;               /* logical bytes per 320-pixel plane row */
    desc[4] = stride;             /* 240 interleaved, 40 separate planar */
    desc[5] = (interleaved ? 1UL : 0UL) |
              ((ULONG)hamfps_pref() << 8);  /* V0.12 HAM publish fps */
    for (int i = 0; i < 6; ++i) desc[6+i] = (ULONG)bm->Planes[i];
    if (dbuf) {
        ext[0] = HAMBUFFER_EXT_MAGIC;
        ext[1] = copper;
        ext[2] = 2UL;
        for (int i=0;i<6;++i) ext[3+i] = (ULONG)back->Planes[i];
        desc[12] = (ULONG)ext;
        desc[13] = 10UL;
    } else {
        /* Exact V0.14.1 physical descriptor contract for the OFF control. */
        desc[12] = copper;
        desc[13] = 9UL;
    }
    CacheClearU();

    ham_env_ptr(HAM_SCREEN_VAR, (APTR)scr);
    ham_env_ptr(HAM_DESC_VAR, (APTR)desc);
    if (ext) ham_env_ptr(HAM_EXT_VAR, (APTR)ext);
    if (back) ham_env_ptr(HAM_BACK_VAR, (APTR)back);
    if (hidden) ham_env_ptr(HAM_VIEW_VAR, (APTR)hidden);
    CloseLibrary((struct Library *)IntuitionBase); IntuitionBase = NULL;

    if (interleaved) {
        if ((ULONG)bm->BytesPerRow == plane_row * (ULONG)bm->Depth)
            Printf("agaboot: HAM6 V0.15.2 sink READY: AGA canonical interleaved %ldx%ld BPR=%ld planeRow=%ld stride=%ld real-chip=512K\n",
                   (LONG)hamw,(LONG)hamh,(LONG)bm->BytesPerRow,(LONG)plane_row,(LONG)stride);
        else
            Printf("agaboot: HAM6 V0.15.2 sink READY: legacy interleaved %ldx%ld BPR=%ld planeRow=%ld stride=%ld real-chip=512K\n",
                   (LONG)hamw,(LONG)hamh,(LONG)bm->BytesPerRow,(LONG)plane_row,(LONG)stride);
    } else {
        Printf("agaboot: HAM6 V0.15.2 sink READY: separate planar %ldx%ld BPR=%ld planeRow=%ld stride=%ld real-chip=512K\n",
               (LONG)hamw,(LONG)hamh,(LONG)bm->BytesPerRow,(LONG)plane_row,(LONG)stride);
    }
    Printf("agaboot: HAM sink desc=$%08lx plane0=$%08lx copper=$%08lx\n",
           (ULONG)desc, (ULONG)bm->Planes[0], copper);
    PutStr("agaboot: hires AGA frames will be reduced 2:1 at the post-render RGB tap\n");
    Printf("agaboot: HAMFPS %ld fps; physical PAL scanout remains 50 Hz\n",
           (LONG)hamfps_pref());
    Printf("agaboot: HAMAREA physical canvas %ldx%ld planeRow=%ld stride=%ld; HIRES/SHRES -> LOWRES 2:1\n",
           (LONG)hamw,(LONG)hamh,(LONG)plane_row,(LONG)stride);
    Printf("agaboot: HAMBUFFER %s; physical HAM bitmaps=%ld; flip owner=CPU1 physical VBlank Copper BPLxPT\n",
           (STRPTR)(dbuf?"ON":"OFF"),(LONG)(dbuf?2:1));
    if (scoped)
        PutStr("agaboot: PROGRAM scope ARMED: current display retained until child Copper is stable\n");
    return desc;

fail:
    if (hidden) { ham_hidden_view_free(hidden); hidden=NULL; }
    if (back) { FreeBitMap(back); back=NULL; }
    if (ext) { FreeMem(ext,HAMBUFFER_EXT_BYTES); ext=NULL; }
    if (scr) CloseScreen(scr);
    if (IntuitionBase) {
        CloseLibrary((struct Library *)IntuitionBase);
        IntuitionBase = NULL;
    }
    return NULL;
}


/* FRF_AGA_DIRECTCHIP_HAM6_V0_1_AGABOOT_HELPERS_END */

static int armed(void)
{
    char v[16];
    return GetVar((STRPTR)ARM_VAR, v, sizeof v, 0) > 0 && v[0] == '1';
}

static void arm_set(int on)
{
    if (on) {
        SetVar((STRPTR)ARM_VAR, (STRPTR)"1", -1, GVF_GLOBAL_ONLY);
        SetVar((STRPTR)ARM_VAR, (STRPTR)"1", -1, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
    } else {
        DeleteVar((STRPTR)ARM_VAR, GVF_GLOBAL_ONLY);
        DeleteVar((STRPTR)ARM_VAR, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
    }
}

/* ENVARC: like AGA_SANDBOX, so the choice survives a reboot. Read at sandbox
   entry and sent to the kernel as $5E00 (AGA) or $5E01 (ECS Denise). */
#define ECS_VAR "AGA_ECSMODE"

/* -1 = decide per game (the default), 0 = always AGA, 1 = always ECS. */
static int ecs_pref(void)
{
    char v[16];
    if (GetVar((STRPTR)ECS_VAR, v, sizeof v, 0) <= 0) return -1;
    if (v[0] == '1') return 1;
    if (v[0] == '0') return 0;
    return -1;
}

static void ecs_pref_set(int pref)
{
    if (pref < 0) {
        DeleteVar((STRPTR)ECS_VAR, GVF_GLOBAL_ONLY);
        DeleteVar((STRPTR)ECS_VAR, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
        return;
    }
    STRPTR v = (STRPTR)(pref ? "1" : "0");
    SetVar((STRPTR)ECS_VAR, v, -1, GVF_GLOBAL_ONLY);
    SetVar((STRPTR)ECS_VAR, v, -1, GVF_GLOBAL_ONLY | GVF_SAVE_VAR);
}

/* ---- which chipset does the game in the current directory want? -----------
   WHDLoad runs S:WHDLoad-Startup with the game's own directory current, so
   the slave it is about to start is right here.

   A slave declares one chipset fact: ws_Flags bit 5, WHDLF_ReqAGA ("abort if
   no AGA chipset is available"), at offset 10 from the "WHDLOADS" id, which
   itself follows ws_Security = $70FF4E75. That flag is a dependable YES and
   an undependable NO: of the AGA titles to hand, CastlevaniaAGA and
   GhostsNGoblinsAGADemo both leave it clear. So the name gets read as a
   second opinion, and ECS is chosen only when NEITHER says AGA.

   Every uncertain case lands on AGA, which is what this program did before
   any of this existed: an AGA game given ECS semantics is visibly wrong,
   while an ECS game given AGA semantics is at worst still dark - the thing
   "ECS mode" in the Chipset menu then fixes by hand. */
#define WHDL_REQAGA 0x0020

static int ci_eq(const char *s, const char *upper, int n)
{
    for (int i = 0; i < n; i++) {
        char a = s[i];
        if (a >= 'a' && a <= 'z') a = (char)(a - 32);
        if (a != upper[i]) return 0;
    }
    return 1;
}

static int name_says_aga(const char *n)
{
    int len = (int)strlen(n);
    for (int i = 0; i + 3 <= len; i++) {
        if (ci_eq(n + i, "AGA", 3)) return 1;
        if (i + 4 <= len && ci_eq(n + i, "CD32", 4)) return 1;
    }
    return 0;
}

/* 1 and *flags set if this really is a WHDLoad slave. */
static int slave_flags(const char *name, UWORD *flags)
{
    UBYTE buf[512];
    BPTR f = Open((STRPTR)name, MODE_OLDFILE);
    if (!f) return 0;
    long n = Read(f, buf, sizeof buf);
    Close(f);
    for (long i = 4; i + 12 <= n; i += 2) {         /* the header is word aligned */
        if (buf[i] != (UBYTE)'W' || buf[i+1] != (UBYTE)'H') continue;
        if (!ci_eq((const char *)(buf + i), "WHDLOADS", 8)) continue;
        if (buf[i-4] != 0x70 || buf[i-3] != 0xFF ||
            buf[i-2] != 0x4E || buf[i-1] != 0x75) continue;   /* ws_Security */
        *flags = (UWORD)(((UWORD)buf[i+10] << 8) | buf[i+11]);
        return 1;
    }
    return 0;
}

/* 1 = every slave here is an ECS/OCS game. 0 = one of them wants AGA, or
   there is nothing here to judge by. */
static int game_is_ecs(void)
{
    struct FileInfoBlock *fib = (struct FileInfoBlock *)AllocDosObject(DOS_FIB, NULL);
    BPTR l = Lock((STRPTR)"", ACCESS_READ);
    int found = 0, aga = 0;
    if (fib && l && Examine(l, fib)) {
        while (ExNext(l, fib)) {
            if (fib->fib_DirEntryType > 0) continue;            /* a drawer */
            int len = (int)strlen(fib->fib_FileName);
            if (len < 6 || !ci_eq(fib->fib_FileName + len - 6, ".SLAVE", 6)) continue;
            UWORD fl = 0;
            if (!slave_flags(fib->fib_FileName, &fl)) continue;
            found++;
            if ((fl & WHDL_REQAGA) || name_says_aga(fib->fib_FileName)) aga = 1;
        }
    }
    if (l) UnLock(l);
    if (fib) FreeDosObject(DOS_FIB, fib);
    return found && !aga;
}

/* WHDLoad calls graphics/SetChipRev() for a slave flagged ReqAGA, even when
   ChipRevBits0 already says AGA. Inside the sandbox that call finds my Lisa
   and switches graphics.library into AA mode for real, and nothing puts it
   back: SetChipRev(BEST) on the real chips afterwards leaves MemType at 3,
   and restoring ChipRevBits0 and MemType by hand does not help either. The
   visible damage is Picasso96's software pointer (SoftSprite=Yes on the
   VideoCore icon) drawn at one fixed spot for good while the real one moves
   invisibly - only after ReqAGA slaves, never after the OCS slave of the
   same game (phoenix_aga vs phoenix_ocs, 2026-09-13; the probe lines in
   pointer.c found the MemType 0 -> 3 difference).
   So while the sandbox is on, SetChipRev is a stub that answers with the
   ChipRevBits0 I set: WHDLoad gets its "AGA present" and graphics.library
   never hears about it. Proven: the pointer survives phoenix_aga with the
   stub in. The stub lives in allocated memory because this program exits;
   OFF puts the vector back and frees it.
       moveq #0,d0 ; move.b ChipRevBits0(a6),d0 ; rts */
static void scr_stub_in(void)
{
    UWORD *code = AllocMem(16, MEMF_PUBLIC);
    char v[16];
    if (!code) return;
    code[0] = 0x7000;
    code[1] = 0x102E; code[2] = (UWORD)((ULONG)&((struct GfxBase *)0)->ChipRevBits0);
    code[3] = 0x4E75;
    CacheClearU();
    APTR old = SetFunction((struct Library *)GfxBase, LVO_SetChipRev, (APTR)code);
    sprintf(v, "%lu", (unsigned long)old);  SetVar((STRPTR)SCR_VAR, v, -1, GVF_GLOBAL_ONLY);
    sprintf(v, "%lu", (unsigned long)code); SetVar((STRPTR)STUB_VAR, v, -1, GVF_GLOBAL_ONLY);
}
static void scr_stub_out(void)
{
    char v[16];
    if (GetVar((STRPTR)SCR_VAR, v, sizeof v, 0) > 0) {
        APTR old = (APTR)strtoul(v, NULL, 10);
        if (old) SetFunction((struct Library *)GfxBase, LVO_SetChipRev, old);
        DeleteVar((STRPTR)SCR_VAR, GVF_GLOBAL_ONLY);
    }
    if (GetVar((STRPTR)STUB_VAR, v, sizeof v, 0) > 0) {
        APTR code = (APTR)strtoul(v, NULL, 10);
        if (code) FreeMem(code, 16);
        DeleteVar((STRPTR)STUB_VAR, GVF_GLOBAL_ONLY);
    }
}

/* ---- the second megabyte on a 1 MB machine -------------------------------
   The game never runs in real chip RAM: the sandbox maps 2 MB of the Pi's
   memory at address 0 whatever the machine has, and the kernel copies only
   as much real chip RAM in and out as there is. What stops a 2 MB game on a
   1 MB machine is Exec's memory list: WHDLoad reads it and refuses. So while
   the sandbox is on, lend Exec the range from the real top up to 2 MB as a
   second chip-memory header, at a lower priority than the real one so the OS
   only touches it when the real megabyte is full, and take it back before the
   mapping goes. MaxLocMem follows, because some code reads the chip size from
   there rather than from the list. */
#define LEND_NAME "AGA-PISTORM chip"
#define MAXLOC_VAR "AGA_MAXLOCMEM"
extern struct ExecBase *SysBase;
/* Found by what it is, not by its name: AddMemList keeps the NAME POINTER,
   and a string inside this program is gone the moment SANDBOX ON exits - the
   OFF run would compare against freed memory. So the name lives in allocated
   public memory, and the search goes by range: the one chip header starting
   at or above 512 KB and ending at 2 MB, at my priority. */
static struct MemHeader *lend_find(void)
{
    for (struct Node *n = SysBase->MemList.lh_Head; n->ln_Succ; n = n->ln_Succ) {
        struct MemHeader *mh = (struct MemHeader *)n;
        if ((mh->mh_Attributes & MEMF_CHIP) && (ULONG)mh->mh_Lower >= 0x80000 &&
            (ULONG)mh->mh_Upper == 0x200000 && n->ln_Pri == -10) return mh;
    }
    return NULL;
}
/* The after-game diagnostics - counter dump, pointer probe, chip RAM lending
   notes - go to AGA-diag.log only when ENV:AGA_DIAGLOG is 1. Someone who
   just plays needs none of it and would wait on it every F10; a test machine
   sets it with `SetEnv SAVE AGA_DIAGLOG 1`. */
int aga_diaglog(void)
{
    char v[4];
    return GetVar((STRPTR)"AGA_DIAGLOG", (STRPTR)v, sizeof v, 0) > 0 && v[0] == '1';
}

static void lend_log(const char *line)
{
    if (!aga_diaglog()) return;
    BPTR f = Open((STRPTR)(file_exists("Stuff:") ? "Stuff:AGA-diag.log" : "SYS:AGA-diag.log"), MODE_READWRITE);
    if (!f) return;
    Seek(f, 0, OFFSET_END);
    Write(f, (APTR)line, strlen(line));
    Close(f);
}
static void chip_lend(int on)
{
    char v[16];
    if (on) {
        if (lend_find()) return;
        ULONG top = 0;
        Forbid();
        for (struct Node *n = SysBase->MemList.lh_Head; n->ln_Succ; n = n->ln_Succ) {
            struct MemHeader *mh = (struct MemHeader *)n;
            if ((mh->mh_Attributes & MEMF_CHIP) && (ULONG)mh->mh_Upper > top) top = (ULONG)mh->mh_Upper;
        }
        Permit();
        if (top == 0 || top >= 0x200000) return;          /* a 2 MB machine: nothing to lend */
        sprintf(v, "%lu", (unsigned long)SysBase->MaxLocMem);
        SetVar((STRPTR)MAXLOC_VAR, v, -1, GVF_GLOBAL_ONLY);
        char *name = AllocMem(32, MEMF_PUBLIC | MEMF_CLEAR);    /* outlives this program; freed at OFF */
        if (name) strcpy(name, LEND_NAME);
        AddMemList(0x200000 - top, MEMF_CHIP | MEMF_PUBLIC | MEMF_LOCAL | MEMF_24BITDMA, -10,
                   (APTR)top, (STRPTR)name);
        SysBase->MaxLocMem = 0x200000;
        { char l[120];
          sprintf(l, "AGALEND on: %lu KB above $%06lx, chip avail now %lu KB\n",
                  (unsigned long)((0x200000 - top) >> 10), (unsigned long)top,
                  (unsigned long)(AvailMem(MEMF_CHIP) >> 10));
          lend_log(l); PutStr(l); }
    } else {
        struct MemHeader *mh = lend_find();
        if (!mh) { lend_log("AGALEND off: nothing lent\n"); return; }
        ULONG size = (ULONG)mh->mh_Upper - (ULONG)mh->mh_Lower;
        char *name = (char *)mh->mh_Node.ln_Name;
        int waited = 0;
        /* WHDLoad gives everything back before its cleanup script runs, but
           give a late FreeMem a moment before pulling the floor away. */
        while (mh->mh_Free + sizeof(struct MemHeader) < size && waited < 25) { Delay(10); waited++; }
        if (mh->mh_Free + sizeof(struct MemHeader) < size)
            Printf("agaboot: WARNING %ld bytes of the lent chip RAM still in use, unlinking anyway\n",
                   (long)(size - mh->mh_Free));
        Forbid();
        Remove((struct Node *)mh);
        Permit();
        if (name) FreeMem(name, 32);
        { char l[140];
          sprintf(l, "AGALEND off: %lu KB unlinked, %lu bytes were still in use, chip avail now %lu KB\n",
                  (unsigned long)(size >> 10), (unsigned long)(size - mh->mh_Free),
                  (unsigned long)(AvailMem(MEMF_CHIP) >> 10));
          lend_log(l); PutStr(l); }
        if (GetVar((STRPTR)MAXLOC_VAR, v, sizeof v, 0) > 0) {
            SysBase->MaxLocMem = strtoul(v, NULL, 10);
            DeleteVar((STRPTR)MAXLOC_VAR, GVF_GLOBAL_ONLY);
        }
    }
}

/* What to send at sandbox entry: the preference, or the game if it is AUTO. */
static int ecs_for_this_game(void)
{
    int pref = ecs_pref();
    return (pref < 0) ? game_is_ecs() : pref;
}

static int sandbox(const char *what)
{
    /* FRF_AGA_ECS_DENISE_BRIDGE_V0_1_AUTO
     * AUTO now has three distinct states:
     *   AGA       -> existing virtual AGA -> HAM6
     *   ECS native override -> bypass sandbox
     *   ECS AUTO/DENISE      -> enter virtual ECS and present through real Denise
     */
    if (!stricmp(what, "AUTO")) {
        int auto_ecs=ecs_for_this_game();
        if(auto_ecs && !ecsoutput_pref()) {
            if(aga_active()) {
                PutStr("agaboot: SANDBOX AUTO: ECS + ECSOUTPUT NATIVE; leaving stale sandbox\n");
                return sandbox("OFF");
            }
            PutStr("agaboot: SANDBOX AUTO: ECS + ECSOUTPUT NATIVE; sandbox bypassed\n");
            return 0;
        }
        if(auto_ecs)
            PutStr("agaboot: SANDBOX AUTO: ECS detected; entering virtual ECS + real Denise presenter\n");
        else
            PutStr("agaboot: SANDBOX AUTO: AGA detected; entering virtual AGA + HAM6 presenter\n");
    }


        if (!stricmp(what, "?")) {
        char v[16];
        int on = GetVar((STRPTR)CHIPREV_VAR, v, sizeof v, 0) > 0;
        Printf("sandbox %s (chipset reports %s), AGA for games: %s, emulated: %s\n",
               (ULONG)(on ? "ON" : "OFF"), (ULONG)(aga_active() ? "AGA" : "ECS"),
               (ULONG)(armed() ? "ARMED" : "off"),
               (ULONG)(ecs_pref() < 0 ? "per game" : ecs_pref() ? "ECS Denise" : "AGA"));
        return on ? 5 : 0;
    }
    if (!stricmp(what, "AUTO")) {
        /* what the WHDLoad hook calls. Never fails: WHDLoad aborts the whole
           game on a non-zero return from ExecuteStartup, and "not armed" is a
           normal state, not an error. */
        if (!armed()) return 0;
        /* Game mode silences the CIA-B interrupt sources AmigaOS leaves
           running, at the moment WHDLoad takes the machine. Left alone they
           hold the CIA's /INT line asserted and the 68k drowns in level 6.
           Kept switchable so the old behaviour is one SetEnv away:
             SetEnv AGA_GAMEMODE 0   change nothing at takeover
             SetEnv AGA_GAMEMODE 1   silence them (the default) */
        {
            char v[16];
            int gm = 1;
            if (GetVar((STRPTR)GAMEMODE_VAR, v, sizeof v, 0) > 0 && v[0] == '0') gm = 0;
            enter_magic = gm ? MAGIC_SANDBOX_GAME : MAGIC_SANDBOX_ON;
        }
        return sandbox("ON");
    }
    if (stricmp(what, "ON") && stricmp(what, "OFF")) return 5;
    int on = !stricmp(what, "ON");
    GfxBase = (struct GfxBase *)OpenLibrary("graphics.library", 39);
    if (!GfxBase) return 20;
    if (on) {
        if (aga_active()) { PutStr("agaboot: AGA chipset already active, nothing to do\n"); CloseLibrary((struct Library *)GfxBase); return 0; }
        /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_SANDBOX_PREP */
        if (output_ham6()) {
            APTR hd = ham_prepare();
            AGA_HAM_DESC = (ULONG)hd;  /* zero means HDMI fallback */
            if (!hd) PutStr("agaboot: HAM6 preparation failed; this game will use HDMI\n");
        } else {
            AGA_HAM_DESC = 0;
        }
        /* FRF_AGA_NATIVE_PAULA_V0_1
           Allocate only genuinely DMA-visible low Chip RAM. Native mode keeps
           the first 8K as a ready software fallback and uses the remainder as
           four fixed physical sample slots, so no live OS/game address is ever
           mirrored or overwritten. */
        {
            unsigned requested=audio_engine_pref(), session=requested;
            ULONG audio_bytes=0;
            APTR audio_buf=audio_buffer_prepare(requested,&session,&audio_bytes);
            Disable();
            AGA_CTRL=(UWORD)(0x6100u | (session & 3u));
            AGA_CTRL=(UWORD)(0x6110u | (audio_buf_size_code(audio_bytes) & 0x0fu));
            if (audio_buf) AGA_RING=(ULONG)audio_buf;
            Enable();
            if (audio_buf) {
                Printf("agaboot: audio engine %s, low-CHIP buffer %ld KB at $%08lx"
                       "\n",
                       (ULONG)audio_engine_name(session),(LONG)(audio_bytes>>10),(ULONG)audio_buf);
            } else {
                PutStr("agaboot: no real low Chip RAM for audio; continuing silent"
                       "\n");
            }
        }
        CacheClearU();   /* flush the 68k caches HERE, in my own context - not inside Emu68's abort handler */
        Disable();
        /* Which chipset this game gets, BEFORE the enter word: entering
           resets the virtual chipset, and the core keeps the mode across that
           reset on purpose. Ignored by a stock Emu68, like the rest. */
        AGA_CTRL = (UWORD)(0x5E00 | (ecs_for_this_game() ? 1 : 0));
        AGA_CTRL = enter_magic;
        Enable();
        if (!aga_active()) {
            /* Deliberately NOT an error. This runs from S:WHDLoad-Startup,
               and WHDLoad aborts the entire game on any non-zero return from
               ExecuteStartup. A missing sandbox is a no-op - the game just
               runs in ECS as it always did - so failing here breaks WHDLoad
               for nothing. Only genuine faults return non-zero. */
            free_ring();
            /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_ENTRY_FAIL_CLEANUP */
            AGA_HAM_DESC = 0;
            ham_release();
            PutStr("agaboot: sandbox not available (stock Emu68, or aga.nosandbox);"
                   " continuing in ECS\n");
            CloseLibrary((struct Library *)GfxBase);
            return 0;   /* see note above: must not fail */
        }
        chip_lend(1);
        if (aga_diaglog()) ptr_probe("entry-before-poke");
        char v[16];
        sprintf(v, "%lu", (unsigned long)GfxBase->ChipRevBits0);
        SetVar((STRPTR)CHIPREV_VAR, v, -1, GVF_GLOBAL_ONLY);
        /* the full set, HR_AGNUS and HR_DENISE included: WHDLoad tests for
           SETCHIPREV_AA, and with only GFXF_AA answered it refuses the slave
           ("needs AGA chipset", 2026-09-13, once SetChipRev was stubbed) */
        GfxBase->ChipRevBits0 |= SETCHIPREV_AA;
        scr_stub_in();
        /* FRF_AGA_RGB_TAP_HAM6_V0_8_0_KEEP_HAM_COPPER
         * CPU3 has already restored the RC10 physical HAM Copper from
         * the descriptor. display_on() would replace COP1LC with gb->copinit
         * and turn the RGB connector black. Only use that path for HDMI or
         * when HAM preparation failed. */
        if (!output_ham6() || !ham_get_ptr(HAM_DESC_VAR)) {
            display_on(GfxBase);
        } else {
            PutStr("agaboot: physical Copper retained by HAM6 RGB sink (copinit NOT installed)\n");
        }
        PutStr("agaboot: sandbox ON, virtual AGA chipset active\n");
    } else {
        char v[16];
        chip_lend(0);
        scr_stub_out();
        if (GetVar((STRPTR)CHIPREV_VAR, v, sizeof v, 0) > 0) {
            GfxBase->ChipRevBits0 = (UBYTE)atoi(v);
            DeleteVar((STRPTR)CHIPREV_VAR, GVF_GLOBAL_ONLY);
        }
        CacheClearU();   /* dirty lines reach chip_va through the redirect before the leave copies it back */
        Disable();
        AGA_CTRL = MAGIC_SANDBOX_OFF;
        Enable();
        free_ring();     /* after MAGIC_SANDBOX_OFF: the ARM has dropped its pointer */
        /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_OFF_CLEANUP: CPU3 has stopped real HAM DMA. */
        AGA_HAM_DESC = 0;
        ham_release();
        /* Put the OS back on the REAL chipset, the mirror of display_on()
           above. Leaving the sandbox replays the virtual chipset's last
           DMACON to the real chips, and that is the GAME's - a game that
           quits with sprite DMA off hands the desktop a chipset with no
           sprites, and the mouse pointer IS sprite 0. Reported from the
           machine as "no mouse cursor when i return to workbench". Cheap,
           idempotent, same call the entry path already trusts. */
        if (!aga_active()) display_on(GfxBase);
        if (aga_diaglog()) ptr_probe("exit-after-restore");
        /* A snapshot of the Pi's counters after every game, appended to
           SYS:AGA-diag.log. The fault being chased - no mouse pointer after
           an AGA game - is the one that makes reading them by hand hard,
           because opening a Shell without a pointer is the problem.
           Appending lets an ECS run and an AGA run be compared side by side,
           which is the comparison that matters: only AGA games do it. A few
           KB per game; delete the file to reset it. Stuff: when mounted,
           SYS: otherwise - never RAM:, the point is to survive the power
           cycle that fixes the pointer. */
        if (aga_diaglog())
            Execute((STRPTR)(file_exists("Stuff:")
                             ? "C:agastat >>Stuff:AGA-diag.log"
                             : "C:agastat >>SYS:AGA-diag.log"), 0, 0);
        PutStr(aga_active() ? "agaboot: sandbox still active?\n" : "agaboot: sandbox OFF, real chipset active\n");
    }
    CloseLibrary((struct Library *)GfxBase);
    return 0;
}

/* FRF_AGA_PROGRAM_SCOPE_V0_1
 * Launch one ordinary executable inside virtual AGA, but do not visibly take
 * over the physical RGB connector at sandbox entry.  CPU1 stays gated until
 * the child installs a stable Copper list of its own.  SystemTags is
 * synchronous, so when the child exits this process immediately restores the
 * real chipset/display state.
 */
static int program_run(int argc, char **argv)
{
    char cmd[1024];
    int pos = 0, rc = 0, run_rc = 0;
    char old_ecs[16];
    LONG old_n;
    UWORD saved_magic = enter_magic;

    if (argc < 3) {
        PutStr("usage: agaboot RUN <program> [arguments...]\n");
        return 5;
    }
    if (aga_active()) {
        PutStr("agaboot: RUN requires the real chipset at launch; leave the current sandbox first\n");
        return 5;
    }

    cmd[0] = 0;
    for (int i = 2; i < argc; ++i) {
        int n;
        if (i != 2 && pos < (int)sizeof(cmd)-1) cmd[pos++] = ' ';
        n = snprintf(cmd + pos, sizeof(cmd) - (size_t)pos, "%s", argv[i]);
        if (n < 0 || n >= (int)(sizeof(cmd) - (size_t)pos)) {
            PutStr("agaboot: RUN command line too long\n");
            return 5;
        }
        pos += n;
    }

    /* Gate physical HAM BEFORE sandbox() publishes the descriptor. */
    Disable(); AGA_CTRL = MAGIC_PROGRAM_PREP; Enable();

    /* RUN is an AGA application session regardless of the WHDLoad ECS policy. */
    old_n = GetVar((STRPTR)ECS_VAR, old_ecs, sizeof old_ecs, GVF_GLOBAL_ONLY);
    SetVar((STRPTR)ECS_VAR, (STRPTR)"0", -1, GVF_GLOBAL_ONLY);
    enter_magic = MAGIC_SANDBOX_PROGRAM;
    rc = sandbox("ON");
    enter_magic = saved_magic;

    if (old_n > 0) {
        if (old_n >= (LONG)sizeof old_ecs) old_n = (LONG)sizeof old_ecs - 1;
        old_ecs[old_n] = 0;
        SetVar((STRPTR)ECS_VAR, (STRPTR)old_ecs, -1, GVF_GLOBAL_ONLY);
    } else {
        DeleteVar((STRPTR)ECS_VAR, GVF_GLOBAL_ONLY);
    }

    if (rc || !aga_active()) {
        Disable(); AGA_CTRL = MAGIC_PROGRAM_CANCEL; Enable();
        PutStr("agaboot: RUN could not enter the virtual AGA session\n");
        return rc ? rc : 10;
    }

    /* Seed the current OS list as the do-not-present baseline. The ARM side
       waits for the child to replace COP1LC for two completed display frames. */
    {
        struct GfxBase *gb = (struct GfxBase *)OpenLibrary("graphics.library", 39);
        if (!gb) {
            sandbox("OFF");
            PutStr("agaboot: RUN could not open graphics.library\n");
            return 20;
        }
        CacheClearU();
        Disable(); AGA_SYS_COPPER = (ULONG)gb->copinit; Enable();
        Printf("agaboot: RUN armed; OS Copper $%08lx ignored until child owns display\n",
               (ULONG)gb->copinit);
        CloseLibrary((struct Library *)gb);
    }

    Printf("agaboot: launching %s through %s; HAM6 takeover is program-scoped\n",
           (ULONG)cmd, (ULONG)(output_ham6() ? "HAM6" : "HDMI/HVS"));

    /* Default SystemTags execution is synchronous. This is deliberate: this
       agaboot process is the lifetime owner of the scoped sandbox. */
    run_rc = SystemTags((STRPTR)cmd, TAG_END);

    PutStr("agaboot: program returned; restoring previous chipset/display\n");
    rc = sandbox("OFF");
    if (rc) return rc;
    return (int)run_rc;
}

/* ---- installer support (setup.c does the text edits) ------------------------
 *
 *   agaboot CHECK                   pre-flight for the Installer script: sets
 *                                   ENV:AGAPISTORM_CLASSIC / _PS16 / _PI4 /
 *                                   _WRITABLE / _CONFIG / _CONFIG16 to 1, 0 or
 *                                   ? and prints what it found
 *   agaboot INSTALL HOOKS           S:WHDLoad-Startup, S:WHDLoad-Cleanup and
 *                                   S:User-Startup get their blocks
 *   agaboot INSTALL CONFIG [PS16] [file]   config.txt boots the AGA kernels:
 *                                   both boards' sections are set, PS16 says
 *                                   this one is a PiStorm16 - its section
 *                                   must exist (default EMU68:config.txt)
 *   agaboot UNINSTALL HOOKS|CONFIG [file]|MENU   undo each of those (CONFIG
 *                                   puts back both boards' kernel lines)
 *
 * Every edit backs the file up to <name>.pre-aga the first time and is safe to
 * repeat: running the installer twice changes nothing the second time. */
extern const char *const setup_whd_on, *const setup_whd_off, *const setup_user_startup;
char *setup_script_set(const char *text, const char *block, int where);
char *setup_script_remove(const char *text);
char *setup_config(const char *text, int install, unsigned opts, int *err);
int setup_has_section(const char *text, int ps16);
char *setup_read(const char *path);
int setup_write(const char *path, const char *text);
int setup_backup(const char *path, const char *text);
void setup_board(int ps16);
const char *setup_section(void);

#include <dos/dostags.h>
#include <dos/dosextens.h>
#define SETUP_OK    "  %-24s %s\n"

static int exists_quiet(const char *path)
{
    BPTR l = Lock((STRPTR)path, ACCESS_READ);
    if (l) UnLock(l);
    return l != 0;
}

/* The Emu68 boot partition, as "<volume>:", or "" if none is mounted.
 *
 * On CaffeineOS it is called EMU68:, but that is only the FAT partition's
 * LABEL - a card labelled BOOT or NO NAME mounts under that name instead.
 * So look for what makes it the boot partition: config.txt, a kernel
 * directory and the Pi firmware. Only MOUNTED volumes get asked, so no
 * "insert volume" requester can appear; requesters are off anyway. */
static char boot_vol[40];
static const char *find_boot(void)
{
    if (boot_vol[0]) return boot_vol;
    char names[16][32];
    int n = 0;
    struct DosList *dl = LockDosList(LDF_VOLUMES | LDF_READ);
    while ((dl = NextDosEntry(dl, LDF_VOLUMES)) && n < 16) {
        UBYTE *b = (UBYTE *)BADDR(dl->dol_Name);
        int len = b[0] < 30 ? b[0] : 30;
        memcpy(names[n], b + 1, len);
        names[n][len] = ':';
        names[n][len + 1] = 0;
        n++;
    }
    UnLockDosList(LDF_VOLUMES | LDF_READ);
    struct Process *me = (struct Process *)FindTask(NULL);
    APTR win = me->pr_WindowPtr;
    me->pr_WindowPtr = (APTR)-1;
    for (int i = 0; i < n && !boot_vol[0]; i++) {
        char p1[64], p2[64], p3[64], p4[64];
        snprintf(p1, sizeof p1, "%sconfig.txt", names[i]);
        snprintf(p2, sizeof p2, "%skernel", names[i]);
        snprintf(p3, sizeof p3, "%sstart4.elf", names[i]);
        snprintf(p4, sizeof p4, "%sstart.elf", names[i]);
        if (exists_quiet(p1) && exists_quiet(p2) && (exists_quiet(p3) || exists_quiet(p4)))
            strcpy(boot_vol, names[i]);
    }
    me->pr_WindowPtr = win;
    return boot_vol;
}

static const char *config_txt(void)
{
    static char p[64];
    snprintf(p, sizeof p, "%sconfig.txt", find_boot()[0] ? find_boot() : "EMU68:");
    return p;
}

static int edit_script(const char *path, const char *block, int where, int install)
{
    char *t = setup_read(path);
    if (!t) { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT READ"); return 10; }
    char *n = install ? setup_script_set(t, block, where) : setup_script_remove(t);
    int rc = 0;
    if (!n) Printf(SETUP_OK, (ULONG)path, (ULONG)(install ? "already set up" : "nothing to remove"));
    else {
        if (install && !setup_backup(path, t)) { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT BACK UP - left alone"); rc = 10; }
        else if (!setup_write(path, n))        { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT WRITE"); rc = 10; }
        else Printf(SETUP_OK, (ULONG)path, (ULONG)(install ? "set up" : "restored"));
        free(n);
    }
    free(t);
    return rc;
}

static int edit_config(const char *path, int install, unsigned opts, int ps16)
{
    char *t = setup_read(path);
    if (!t) { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT READ"); return 10; }
    int err, rc = 0;
    setup_board(ps16);
    char *n = setup_config(t, install, opts, &err);
    if (err == 1) {
        Printf("  %-24s has no %s section - not a config this understands, left alone\n", (ULONG)path,
               (ULONG)(install ? setup_section() : "[gpio17=0] or [gpio24=1]"));
        rc = 10;
    } else if (!n) Printf(SETUP_OK, (ULONG)path, (ULONG)(install ? "already boots the AGA kernel" : "nothing to restore"));
    else {
        if (install && !setup_backup(path, t)) { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT BACK UP - left alone"); rc = 10; }
        else if (!setup_write(path, n))        { Printf(SETUP_OK, (ULONG)path, (ULONG)"CANNOT WRITE (read-only?)"); rc = 10; }
        else Printf(SETUP_OK, (ULONG)path, (ULONG)(install ? "boots the AGA kernel now" : "boots the previous kernel again"));
        free(n);
    }
    free(t);
    return rc;
}

/* Run a command and keep the first line it prints (empty on failure). */
static void run_capture(const char *cmd, char *line, int size)
{
    line[0] = 0;
    BPTR out = Open((STRPTR)"T:agaboot.tmp", MODE_NEWFILE);
    if (!out) return;
    LONG r = SystemTags((STRPTR)cmd, SYS_Output, out, SYS_Input, 0, TAG_END);
    Close(out);
    if (r == 0) {
        BPTR f = Open((STRPTR)"T:agaboot.tmp", MODE_OLDFILE);
        if (f) {
            LONG n = Read(f, line, size - 1);
            Close(f);
            line[n > 0 ? n : 0] = 0;
            char *nl = strpbrk(line, "\r\n");
            if (nl) *nl = 0;
        }
    }
    DeleteFile((STRPTR)"T:agaboot.tmp");
}

static void set_flag(const char *name, int v)     /* 1, 0, or -1 = "?" (could not tell) */
{
    SetVar((STRPTR)name, (STRPTR)(v < 0 ? "?" : v ? "1" : "0"), -1, GVF_GLOBAL_ONLY);
}

#include <graphics/text.h>

/* Chipset menu - About. A window, not a requester, because a requester has
   one font: the credit line goes in the screen's own font, bold, and the
   licence lines below it in topaz 8. Closes on its close gadget, a click
   inside or any key. */
static int about(void)
{
    static const char *const head[] = {
        "AGA-PISTORM " AGA_PISTORM_VERSION,
        "made by Astair86 for the Amiga community",
    };
    static const char *const body[] = {
        "An AGA chipset (Alice, Lisa and Paula) emulated on the Raspberry",
        "Pi, for WHDLoad games on the classic PiStorm and the PiStorm16.",
        "",
        "AGA-PISTORM and its Amiga tools are free software under the GNU",
        "General Public License, version 2 (Licenses/GPL-2.0.txt).",
        "",
        "The kernel is Emu68 by Michal Schulz, Mozilla Public License 2.0",
        "(Licenses/MPL-2.0.txt), with the AGA-PISTORM chipset built in.",
        "VideoCore.card is from Emu68-tools by Michal Schulz, MPL 2.0,",
        "with changes by Astair86. All the source is in Source/.",
        "",
        "Thanks to the Emu68, PiStorm and CaffeineOS projects.",
        "",
        "This program comes with ABSOLUTELY NO WARRANTY.",
        "Use it at your own risk - back up your SD card first.",
    };
    enum { NH = sizeof head / sizeof head[0], NB = sizeof body / sizeof body[0], MARGIN = 16 };
    int rc = 20;
    IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
    GfxBase = (struct GfxBase *)OpenLibrary("graphics.library", 37);
    struct Screen *scr = NULL;
    struct DrawInfo *dri = NULL;
    struct TextFont *big = NULL, *small = NULL;
    struct Window *win = NULL;
    if (!IntuitionBase || !GfxBase) goto out;
    scr = LockPubScreen(NULL);
    if (!scr) goto out;
    dri = GetScreenDrawInfo(scr);
    big = OpenFont(scr->Font);
    static struct TextAttr topaz8 = { (STRPTR)"topaz.font", 8, 0, 0 };
    small = OpenFont(&topaz8);
    if (!dri || !big || !small) goto out;

    /* measure with a scratch RastPort, then size the window to the text */
    struct RastPort mrp;
    InitRastPort(&mrp);
    int w = 0;
    SetFont(&mrp, big);
    SetSoftStyle(&mrp, FSF_BOLD, AskSoftStyle(&mrp));
    for (int i = 0; i < NH; i++) { int t = TextLength(&mrp, (STRPTR)head[i], strlen(head[i])); if (t > w) w = t; }
    SetFont(&mrp, small);
    SetSoftStyle(&mrp, 0, AskSoftStyle(&mrp));
    for (int i = 0; i < NB; i++) { int t = TextLength(&mrp, (STRPTR)body[i], strlen(body[i])); if (t > w) w = t; }
    int bh = big->tf_YSize + 2, sh = small->tf_YSize + 2;
    int iw = w + 2 * MARGIN;
    int ih = MARGIN + NH * bh + small->tf_YSize + NB * sh + MARGIN;

    win = OpenWindowTags(NULL,
        WA_PubScreen, (ULONG)scr,
        WA_Title, (ULONG)"About AGA-PISTORM",
        WA_InnerWidth, iw, WA_InnerHeight, ih,
        WA_Left, scr->Width > iw ? (scr->Width - iw) / 2 : 0,
        WA_Top, scr->Height > ih ? (scr->Height - ih) / 2 : 0,
        WA_CloseGadget, TRUE, WA_DragBar, TRUE, WA_DepthGadget, TRUE,
        WA_Activate, TRUE, WA_RMBTrap, TRUE, WA_AutoAdjust, TRUE,
        WA_IDCMP, IDCMP_CLOSEWINDOW | IDCMP_VANILLAKEY | IDCMP_MOUSEBUTTONS,
        TAG_DONE);
    if (!win) goto out;

    struct RastPort *rp = win->RPort;
    int x0 = win->BorderLeft + MARGIN, y = win->BorderTop + MARGIN;
    SetDrMd(rp, JAM1);
    SetFont(rp, big);
    SetSoftStyle(rp, FSF_BOLD, AskSoftStyle(rp));
    for (int i = 0; i < NH; i++) {
        SetAPen(rp, dri->dri_Pens[i == 0 ? HIGHLIGHTTEXTPEN : TEXTPEN]);
        int t = TextLength(rp, (STRPTR)head[i], strlen(head[i]));
        Move(rp, win->BorderLeft + (iw - t) / 2, y + big->tf_Baseline);
        Text(rp, (STRPTR)head[i], strlen(head[i]));
        y += bh;
    }
    y += small->tf_YSize;
    SetFont(rp, small);
    SetSoftStyle(rp, 0, AskSoftStyle(rp));
    SetAPen(rp, dri->dri_Pens[TEXTPEN]);
    for (int i = 0; i < NB; i++) {
        Move(rp, x0, y + small->tf_Baseline);
        Text(rp, (STRPTR)body[i], strlen(body[i]));
        y += sh;
    }

    for (int done = 0; !done; ) {
        WaitPort(win->UserPort);
        struct IntuiMessage *m;
        while ((m = (struct IntuiMessage *)GetMsg(win->UserPort))) {
            if (m->Class == IDCMP_CLOSEWINDOW || m->Class == IDCMP_VANILLAKEY ||
                (m->Class == IDCMP_MOUSEBUTTONS && m->Code == SELECTUP)) done = 1;
            ReplyMsg((struct Message *)m);
        }
    }
    rc = 0;
out:
    if (win) CloseWindow(win);
    if (small) CloseFont(small);
    if (big) CloseFont(big);
    if (dri) FreeScreenDrawInfo(scr, dri);
    if (scr) UnlockPubScreen(NULL, scr);
    if (GfxBase) CloseLibrary((struct Library *)GfxBase);
    if (IntuitionBase) CloseLibrary((struct Library *)IntuitionBase);
    GfxBase = NULL; IntuitionBase = NULL;
    return rc;
}

/* Which Pi the running AGA-PISTORM kernel was built for and which it runs on:
   3 or 4 each, or both 0 when the kernel does not say (an older one, or not
   mine - then the reads land on the real chipset and are ignored). */
static void kernel_boards(int *built, int *runs)
{
    static const UWORD sel[2] = { D_BUILD_BOARD, D_RUN_BOARD };
    ULONG v[2];
    for (int i = 0; i < 2; i++) {
        Disable();
        DIAG_SEL_W = sel[i];
        ULONG hi = DIAG_DAT_W;
        ULONG lo = DIAG_DAT_W;
        Enable();
        v[i] = (hi << 16) | lo;
    }
    *built = (v[0] == 3 || v[0] == 4) ? (int)v[0] : 0;
    *runs  = (v[1] == 3 || v[1] == 4) ? (int)v[1] : 0;
    if (!*built || !*runs) *built = *runs = 0;
}

static int check(void)
{
    char variant[64], board[96];
    run_capture("C:Emu68Info VARIANT", variant, sizeof variant);
    run_capture("C:Emu68Info BOARDNAME", board, sizeof board);
    /* No C:Emu68Info means "cannot tell", not "no": the Installer asks instead.
       Emu68 1.1 names the board at runtime - "pistorm" (classic), "pistorm16",
       "pistorm32lite". 1.0.x only had the build name, "pistorm" or
       "pistorm32lite", and a PiStorm16 kernel of that age may say the latter:
       so "pistorm32lite" is "cannot tell" too, and the Installer asks. */
    int classic = !variant[0] ? -1 : !strcmp(variant, "pistorm");
    int ps16 = (!variant[0] || !strcmp(variant, "pistorm32lite")) ? -1 : !strcmp(variant, "pistorm16");
    int pi4 = !board[0] ? -1 : strstr(board, "Pi 4") || strstr(board, "Pi 400") || strstr(board, "Compute Module 4");
    /* the Pi 3 has its own kernel since 1.1; the Installer picks it by this */
    int pi3 = !board[0] ? -1 : strstr(board, "Pi 3") || strstr(board, "Compute Module 3");
    /* The real Agnus, read before any sandbox: VPOSR bit 12 is its NTSC bit on
       OCS and ECS alike. Pre-answers the Installer's PAL/NTSC question. */
    int ntsc = aga_active() ? -1 : (*(volatile UWORD *)0xDFF004 & 0x1000) != 0;
    const char *boot = find_boot();
    int writable = 0;
    if (boot[0]) {
        char probe[64];
        snprintf(probe, sizeof probe, "%sagaboot.write-test", boot);
        BPTR f = Open((STRPTR)probe, MODE_NEWFILE);
        if (f) { Close(f); DeleteFile((STRPTR)probe); writable = 1; }
    }
    /* the Installer copies the kernel and overlays here */
    SetVar((STRPTR)"AGAPISTORM_BOOT", (STRPTR)boot, -1, GVF_GLOBAL_ONLY);
    int config = 0, config16 = 0, unicam = 0, palovl = 0;
    char *t = boot[0] ? setup_read(config_txt()) : NULL;
    int have_cfg = t != NULL;
    if (t) {
        config = setup_has_section(t, 0);      /* each board's section, whichever this is */
        config16 = setup_has_section(t, 1);
        /* Overlays already in config.txt settle their question: the installer
           only ever adds lines. dtoverlay=pal also explains a "PAL" Agnus on
           an NTSC machine - Emu68 writes the PAL bit into BEAMCON0 at boot
           and masks the NTSC bit out of VPOSR (aarch64/vectors.c). */
        for (char *p = t; p && *p; p = strchr(p, '\n') ? strchr(p, '\n') + 1 : NULL) {
            while (*p == ' ' || *p == '\t') p++;
            if (!strncmp(p, "dtoverlay=unicam", 16)) unicam = 1;
            if (!strncmp(p, "dtoverlay=pal", 13) && (p[13] == '\r' || p[13] == '\n' || p[13] == ',' || !p[13])) palovl = 1;
        }
        free(t);
    }
    /* 2 MB of chip RAM, as the OS counted it at boot. The game itself runs in
       the emulated 2 MB, but entry and exit copy 2 MB between the real chip
       RAM and the emulated one - on a 1 MB machine the upper half is a mirror
       of the lower, and the copy back would put stale data over the OS. And
       WHDLoad refuses a 2 MB game when the OS reports 1 MB anyway. */
    ULONG chip = AvailMem(MEMF_CHIP | MEMF_TOTAL);
    int chip2m = chip >= 0x1C0000;
    set_flag("AGAPISTORM_CLASSIC", classic);
    set_flag("AGAPISTORM_PS16", ps16);
    set_flag("AGAPISTORM_PI4", pi4);
    set_flag("AGAPISTORM_PI3", pi3);
    set_flag("AGAPISTORM_WRITABLE", writable);
    set_flag("AGAPISTORM_CONFIG", config);
    set_flag("AGAPISTORM_CONFIG16", config16);
    set_flag("AGAPISTORM_CHIP2M", chip2m);
    set_flag("AGAPISTORM_NTSC", ntsc);
    set_flag("AGAPISTORM_UNICAM", unicam);
    set_flag("AGAPISTORM_PALOVL", palovl);
    Printf("  PiStorm variant          %s\n", (ULONG)(variant[0] ? variant : "unknown (no C:Emu68Info?)"));
    Printf("  Raspberry Pi             %s\n", (ULONG)(board[0] ? board : "unknown"));
    {
        int kb, kr;
        kernel_boards(&kb, &kr);
        if (kb)
            Printf("  AGA-PISTORM kernel       for a Pi %ld, running on a Pi %ld%s\n", (ULONG)kb, (ULONG)kr,
                   (ULONG)(kb != kr ? " - run the installer again" : ""));
    }
    Printf("  Agnus                    %s\n", (ULONG)(ntsc < 0 ? "unknown (AGA active)" : ntsc ? "NTSC" : "PAL"));
    Printf("  Framethrower in config   %s\n", (ULONG)(unicam ? "yes" : "no"));
    Printf("  dtoverlay=pal in config  %s\n", (ULONG)(palovl ? "yes (the system already runs as PAL)" : "no"));
    Printf("  Chip RAM                 %ld KB%s\n", chip >> 10, (ULONG)(chip2m ? "" : " (games get 2 MB from the Pi)"));
    Printf("  Emu68 boot partition     %s %s\n", (ULONG)(boot[0] ? boot : "NOT FOUND"),
           (ULONG)(!boot[0] ? "" : writable ? "(writable)" : "(READ-ONLY)"));
    if (!have_cfg)
        PutStr("  config.txt               missing\n");
    else {
        Printf("  config.txt [gpio17=0]    %s\n", (ULONG)(config ? "found (classic PiStorm)" : "not found"));
        Printf("  config.txt [gpio24=1]    %s\n", (ULONG)(config16 ? "found (PiStorm16)" : "not found"));
    }
    /* the answer is in the ENV flags; a non-zero return would only give the
       Installer script an error to trip over */
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !stricmp(argv[1], "CHECK")) return check();
    if (argc >= 2 && !stricmp(argv[1], "ABOUT")) return about();
    if (argc >= 2 && !stricmp(argv[1], "RUN")) return program_run(argc, argv);
    if (argc >= 3 && (!stricmp(argv[1], "INSTALL") || !stricmp(argv[1], "UNINSTALL"))) {
        int install = !stricmp(argv[1], "INSTALL");
        if (!stricmp(argv[2], "HOOKS")) {
            int rc = 0;
            rc |= edit_script("S:WHDLoad-Startup", setup_whd_on, 0, install);
            rc |= edit_script("S:WHDLoad-Cleanup", setup_whd_off, 1, install);
            rc |= edit_script("S:User-Startup", setup_user_startup, 0, install);
            return rc ? 10 : 0;
        }
        if (!stricmp(argv[2], "CONFIG")) {
            /* INSTALL CONFIG [FT] [PAL] [PS16] [file]: FT = a Framethrower
               (unicam), PAL = an NTSC Agnus that should run the system as PAL,
               PS16 = a PiStorm16 (its own section and kernel) */
            const char *path = config_txt();
            unsigned opts = 0;
            int ps16 = 0;
            for (int i = 3; i < argc; i++) {
                if (!stricmp(argv[i], "FT")) opts |= 1;
                else if (!stricmp(argv[i], "PAL")) opts |= 2;
                else if (!stricmp(argv[i], "PS16")) ps16 = 1;
                else path = argv[i];
            }
            return edit_config(path, install, opts, ps16);
        }
        if (!install && !stricmp(argv[2], "MENU")) {
            if (file_exists(MENU_FILE)) write_menu(MENU_FILE, 1);
            if (file_exists(DOPUS_MENU))
                dopus_menu_update(DOPUS_MENU, "Chipset", NULL, NULL, NULL, NULL, NULL, 0);
            PutStr("  Chipset menu             removed (gone after the next reboot)\n");
            return 0;
        }
        PutStr("usage: agaboot INSTALL HOOKS|CONFIG [FT] [PAL] [PS16] [file] | UNINSTALL HOOKS|CONFIG [file]|MENU\n");
        return 5;
    }
    /* FRF_AGA_NATIVE_PAULA_V0_1 */
    if (argc >= 2 && !stricmp(argv[1], "AUDIO")) {
        if (argc >= 3 && !stricmp(argv[2], "ENGINE")) {
            if (argc < 4 || !stricmp(argv[3], "?")) {
                Printf("agaboot: AUDIO ENGINE %s (next sandbox entry)\n",
                       (ULONG)audio_engine_name(audio_engine_pref()));
                return 0;
            }
            unsigned mode;
            if (!stricmp(argv[3],"SOFTWARE")) mode=0u;
            else if (!stricmp(argv[3],"NATIVE")) mode=1u;
            else if (!stricmp(argv[3],"AUTO")) mode=2u;
            else { PutStr("usage: agaboot AUDIO ENGINE SOFTWARE|NATIVE|AUTO|?\n"); return 5; }
            audio_engine_pref_set(mode);
            Disable(); AGA_CTRL=(UWORD)(0x6100u | mode); Enable();
            Printf("agaboot: AUDIO ENGINE %s; takes effect on next sandbox entry\n",
                   (ULONG)audio_engine_name(mode));
            return 0;
        }
        PutStr("usage: agaboot AUDIO ENGINE SOFTWARE|NATIVE|AUTO|?\n");
        return 5;
    }

    /* AUDMASK 0-15: which of the four emulated Paula channels reach the mix.
       Diagnostic - isolate one channel on a running game to hear it alone. */
    if (argc >= 3 && !stricmp(argv[1], "AUDMASK")) {
        LONG m = atol((const char *)argv[2]);
        if (m < 0 || m > 15) { PutStr("agaboot: AUDMASK takes 0-15\n"); return 5; }
        Disable(); AGA_CTRL = (UWORD)(0x5D00 | (m & 15)); Enable();
        Printf("agaboot: audio mask now %ld (bit n = channel n)\n", m);
        return 0;
    }
    /* FRF_AGA_DIRECTCHIP_HAM6_V0_1_OUTPUT_CLI */
    if (argc >= 2 && !stricmp(argv[1], "OUTPUT")) {
        if (argc < 3 || !stricmp(argv[2], "?")) {
            Printf("agaboot: AGA output for next game = %s\n", (ULONG)(output_ham6() ? "HAM6 RGB TAP" : "HDMI/HVS"));
            return 0;
        }
        if (!stricmp(argv[2], "HAM6")) output_set(1);
        else if (!stricmp(argv[2], "HDMI") || !stricmp(argv[2], "HVS")) output_set(0);
        else { PutStr("agaboot: OUTPUT takes HDMI|HAM6|?\n"); return 5; }
        Printf("agaboot: AGA output set to %s (takes effect on next sandbox entry)\n",
               (ULONG)(output_ham6() ? "HAM6 RGB TAP" : "HDMI/HVS"));
        return 0;
    }
    if (argc >= 3 && !stricmp(argv[1], "SANDBOX")) {
        /* SANDBOX OFF <0-4>: bisect the leave. Each level runs one more step
           of aga_sandbox_leave and then returns, so a level that kills the
           machine names the step that did it. Settable at runtime on purpose -
           as a cmdline option this needed a boot, and a card out of the Amiga
           and into the PC, for every rung. */
        if (argc >= 4 && !stricmp(argv[2], "OFF")) {
            int lvl = atoi(argv[3]);
            if (lvl >= 0 && lvl <= 4) {
                Disable();
                AGA_CTRL = (UWORD)(0x5B00 | lvl);
                Enable();
                Printf("agaboot: leave bisect level %ld\n", (long)lvl);
            }
        }
        return sandbox(argv[2]);
    }
    if (argc >= 2 && !stricmp(argv[1], "PROBE")) return ptr_probe(argc >= 3 ? argv[2] : "manual");
    if (argc >= 3 && !stricmp(argv[1], "REDIRECT")) {
        /* REDIRECT 0|1|2: chip RAM through Emu68's abort handler with the real
           chipset left running (1 = top 64 KB, 2 = all of it, 0 = off). The
           mechanism sandbox entry relies on, tested on its own. */
        int n = atoi(argv[2]);
        if (n < 0 || n > 2) return 5;
        CacheClearU();
        Disable();
        AGA_CTRL = (UWORD)(0x5C00 | n);
        Enable();
        Printf("agaboot: chip RAM redirect %s\n",
               (ULONG)(n == 0 ? "OFF (window copied back)" : n == 1 ? "ON for the top 64 KB" : "ON for ALL chip RAM"));
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "AGAGAMES")) {
        /* AGAGAMES ON|OFF for scripts; no argument, or ASK, opens a requester.
           The menu entry uses ASK on purpose. Putting the state in the menu
           LABEL does not work: the label is written into the menu file, and
           ToolsDaemon and Dopus only read that file when they start. So a
           label saying "- Active" kept claiming the switch was on for the rest
           of the session after it had been turned off - the tool was right and
           the menu was lying through its teeth. A requester reads the state
           when it opens, so it cannot go stale, and it also gives the
           confirmation that clicking a menu item otherwise never showed. */
        if (argc >= 3 && !stricmp(argv[2], "ON"))  arm_set(1);
        else if (argc >= 3 && !stricmp(argv[2], "OFF")) arm_set(0);
        else if (argc < 3 || !stricmp(argv[2], "ASK")) {
            /* One item for the whole Chipset menu. Picking a mode also turns
               the emulation on, because that is plainly what picking one
               means - nobody chooses a chipset for games that are about to
               run on the real one. "Always AGA" is not here on purpose:
               Automatic already gives every AGA game AGA, so a fourth button
               would earn its space about once a year. It is still one
               `agaboot ECSGAMES OFF` away in a Shell. */
            {   /* The kernel on the card was installed for the other Pi: say so
                   before anything else. A Pi 3 kernel on a Pi 4 works as a Pi 4
                   one; a Pi 4 kernel on a Pi 3 runs games without the picture. */
                int kb, kr;
                kernel_boards(&kb, &kr);
                if (kb && kb != kr) {
                    struct EasyStruct ws = { sizeof(struct EasyStruct), 0, (UBYTE *)"AGA-PISTORM",
                        (UBYTE *)(kb == 3 ? "This AGA-PISTORM kernel was installed\n"
                                            "for a Raspberry Pi 3, but the PiStorm\n"
                                            "has a Pi 4 now.\n\n"
                                            "Games still run. Run the AGA-PISTORM\n"
                                            "installer again for the Pi 4 kernel."
                                          : "This AGA-PISTORM kernel was installed\n"
                                            "for a Raspberry Pi 4, but the PiStorm\n"
                                            "has a Pi 3 now.\n\n"
                                            "Games run without the AGA picture.\n"
                                            "Run the AGA-PISTORM installer again\n"
                                            "for the Pi 3 kernel."),
                        (UBYTE *)"OK" };
                    IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
                    if (IntuitionBase) {
                        EasyRequestArgs(NULL, &ws, NULL, NULL);
                        CloseLibrary((struct Library *)IntuitionBase);
                    }
                }
            }
            int on = armed();
            int pref = ecs_pref();
            struct EasyStruct es = { sizeof(struct EasyStruct), 0, (UBYTE *)"AGA-PISTORM",
                                     (UBYTE *)(!on ? "WHDLoad games run on the REAL chipset.\n\n"
                                                     "Automatic gives each game the chipset its\n"
                                                     "WHDLoad slave asks for. ECS forces an ECS\n"
                                                     "Denise on every game."
                                               : pref < 0 ? "Games run on the emulated chipset,\n"
                                                     "AUTOMATIC: each one gets the chipset its\n"
                                                     "WHDLoad slave asks for.\n\n"
                                                     "Pick ECS only for a game that still\n"
                                                     "looks dark."
                                               : pref ? "Games run on the emulated chipset,\n"
                                                     "forced to ECS DENISE.\n\n"
                                                     "AGA games look wrong like this - go back\n"
                                                     "to Automatic for them."
                                               : "Games run on the emulated chipset,\n"
                                                     "forced to AGA.\n\n"
                                                     "Some ECS games come out very dark."),
                                     (UBYTE *)(on ? "Automatic|ECS mode|Turn off|Cancel"
                                                  : "Automatic|ECS mode|Leave off") };
            IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
            if (!IntuitionBase) return 20;
            int r = EasyRequestArgs(NULL, &es, NULL, NULL);
            CloseLibrary((struct Library *)IntuitionBase);
            if (r == 1)      { ecs_pref_set(-1); arm_set(1); }
            else if (r == 2) { ecs_pref_set(1);  arm_set(1); }
            else if (r == 3 && on) arm_set(0);
        }
        {
            int pref = ecs_pref();
            Printf("agaboot: AGA for WHDLoad games is %s, chipset %s\n",
                   (ULONG)(armed() ? "ON" : "OFF"),
                   (ULONG)(pref < 0 ? "per game" : pref ? "forced to ECS" : "forced to AGA"));
        }
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "ECSGAMES")) {
        /* Which chipset the emulation presents. Same requester-not-label
           reasoning as AGAGAMES above. Takes effect on the NEXT game: the
           word is sent at sandbox entry, so a game already running keeps
           the chipset it started with. */
        if (argc >= 3 && !stricmp(argv[2], "ON"))  ecs_pref_set(1);
        else if (argc >= 3 && !stricmp(argv[2], "OFF")) ecs_pref_set(0);
        else if (argc >= 3 && !stricmp(argv[2], "AUTO")) ecs_pref_set(-1);
        else if (argc >= 3 && !stricmp(argv[2], "?")) {
            /* What AUTO would decide for the CURRENT directory. Run it inside a
               game's drawer to see what that game will get, without launching
               it - the only way to check the slave reading on a real install. */
            int pref = ecs_pref();
            Printf("agaboot: setting %s; this directory reads as %s\n",
                   (ULONG)(pref < 0 ? "AUTO (per game)" : pref ? "always ECS" : "always AGA"),
                   (ULONG)(game_is_ecs() ? "ECS/OCS" : "AGA, or no slave to judge by"));
            return 0;
        }
        else if (argc < 3 || !stricmp(argv[2], "ASK")) {
            int pref = ecs_pref();
            struct EasyStruct es = { sizeof(struct EasyStruct), 0, (UBYTE *)"AGA-PISTORM",
                                     (UBYTE *)(pref < 0
                                       ? "Each game gets the chipset its WHDLoad\n"
                                         "slave asks for. This is the normal\n"
                                         "setting.\n\n"
                                         "Override it only if a game looks wrong."
                                       : pref
                                       ? "Every game runs on an ECS DENISE.\n\n"
                                         "ECS games get their full palette,\n"
                                         "but AGA games look wrong."
                                       : "Every game runs on the AGA chipset.\n\n"
                                         "Some ECS games come out very dark."),
                                     (UBYTE *)"Automatic|Always ECS|Always AGA" };
            IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
            if (!IntuitionBase) return 20;
            int r = EasyRequestArgs(NULL, &es, NULL, NULL);
            CloseLibrary((struct Library *)IntuitionBase);
            if (r == 1) ecs_pref_set(-1);
            else if (r == 2) ecs_pref_set(1);
            else ecs_pref_set(0);
        }
        {
            int pref = ecs_pref();
            Printf("agaboot: emulated chipset is %s\n",
                   (ULONG)(pref < 0 ? "chosen per game" : pref ? "ECS Denise" : "AGA"));
        }
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "HAMFPS")) {
        int fps;
        if (argc >= 3 && !stricmp(argv[2], "?")) {
            Printf("agaboot: HAMFPS %ld fps; AGA/PAL timing remains 50 Hz\n",
                   (LONG)hamfps_pref());
            return 0;
        }
        if (argc >= 3 && !stricmp(argv[2], "25")) fps = 25;
        else if (argc >= 3 && !stricmp(argv[2], "50")) fps = 50;
        else {
            PutStr("usage: agaboot HAMFPS 25|50|?\n");
            return 5;
        }
        hamfps_pref_set(fps);
        hamfps_apply_live(fps);
        Printf("agaboot: HAMFPS set to %ld fps; AGA/PAL timing stays 50 Hz\n",
               (LONG)fps);
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "ECSOUTPUT")) {
        if (argc >= 3 && !stricmp(argv[2], "?")) {
            char v[24]; LONG n=GetVar((STRPTR)ECSOUTPUT_VAR,v,sizeof v,0);
            if(n<=0) PutStr("agaboot: ECSOUTPUT AUTO -> DENISE on 512K physical-sink branch\n");
            else { v[(n<(LONG)sizeof(v))?n:(LONG)sizeof(v)-1]=0; Printf("agaboot: ECSOUTPUT %s\n",(STRPTR)v); }
            return 0;
        }
        if(argc<3 || (stricmp(argv[2],"AUTO") && stricmp(argv[2],"NATIVE") && stricmp(argv[2],"DENISE"))) {
            PutStr("usage: agaboot ECSOUTPUT AUTO|NATIVE|DENISE|?\n");
            return 5;
        }
        ecsoutput_pref_set((STRPTR)argv[2]);
        Printf("agaboot: ECSOUTPUT set to %s; takes effect on next SANDBOX AUTO/ON\n",(STRPTR)argv[2]);
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "HAMBUFFER")) {
        if (argc >= 3 && !stricmp(argv[2], "?")) {
            Printf("agaboot: HAMBUFFER %s; change takes effect on next SANDBOX ON\n",
                   (STRPTR)(hambuffer_pref()?"ON":"OFF"));
            return 0;
        }
        if (argc < 3 || (stricmp(argv[2],"ON") && stricmp(argv[2],"OFF"))) {
            PutStr("usage: agaboot HAMBUFFER ON|OFF|?\n");
            return 5;
        }
        hambuffer_pref_set(!stricmp(argv[2],"ON"));
        Printf("agaboot: HAMBUFFER set to %s; restart SANDBOX to recreate physical HAM sink\n",
               (STRPTR)(hambuffer_pref()?"ON":"OFF"));
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "HAMAREA")) {
        ULONG w,h;
        if (argc >= 3 && !stricmp(argv[2], "?")) {
            hamarea_pref(&w,&h);
            Printf("agaboot: HAMAREA %ldx%ld lowres; change takes effect on next SANDBOX ON\n",
                   (LONG)w,(LONG)h);
            return 0;
        }
        if (argc < 3 ||
            (stricmp(argv[2], "320x256") && stricmp(argv[2], "352x272") &&
             stricmp(argv[2], "368x280") && stricmp(argv[2], "384x280"))) {
            PutStr("usage: agaboot HAMAREA 320x256|352x272|368x280|384x280|?\n");
            return 5;
        }
        hamarea_pref_set((STRPTR)argv[2]);
        hamarea_pref(&w,&h);
        Printf("agaboot: HAMAREA set to %ldx%ld; restart SANDBOX to recreate physical HAM sink\n",
               (LONG)w,(LONG)h);
        return 0;
    }
    if (argc >= 3 && !stricmp(argv[1], "OVERLAY")) {
        /* OVERLAY ON|IDLE|OFF: the display registers drawn over the AGA picture.
           ON  - over every AGA screen: photograph a game that has no quit,
                 since on a classic PiStorm the reset that ends it reboots the
                 Pi and takes the kernel's own records with it
           IDLE- ON, and also on a blank plane while the sandbox idles (for a
                 game that hangs before it draws; hides WHDLoad's splash menu)
           OFF - neither (the default). Until the next reboot either way. */
        UWORD w = !stricmp(argv[2], "ON") ? 2 : !stricmp(argv[2], "IDLE") ? 3 : 0;
        Disable(); AGA_CTRL = (UWORD)(0x5F00 | w); Enable();
        Printf("agaboot: overlay %s\n", (ULONG)(w == 2 ? "ON" : w == 3 ? "ON, idle plane too" : "OFF"));
        return 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "DISPLAY")) {
        GfxBase = (struct GfxBase *)OpenLibrary("graphics.library", 39);
        if (!GfxBase) return 20;
        if (!aga_active()) { PutStr("agaboot: sandbox not active\n"); CloseLibrary((struct Library *)GfxBase); return 5; }
        display_on(GfxBase);
        Printf("agaboot: copper/bitplane/sprite DMA on, COP1LC = copinit $%08lx\n", (ULONG)GfxBase->copinit);
        CloseLibrary((struct Library *)GfxBase);
        return 0;
    }
    /* FRF_AGA_MODE_SYSTEM_ALIAS_V0_1_1
     * MODE alone is the existing status query.
     * MODE SYSTEM is a compatibility spelling for the existing AGA
     * whole-system boot command. Rewrite argv before the MODE query so
     * execution falls through into the original AGA/ECS reboot code below. */
    if (argc >= 3 && !stricmp(argv[1], "MODE") && !stricmp(argv[2], "SYSTEM")) {
        argv[1] = (char *)"AGA";
        argc = 2;
    }
    /* FRF_AGA_SYSTEMWIDE_CLI_V0_1_1
     * Whole-session virtual AGA entry for ordinary CLI executables.
     * Reuse the proven sandbox entry and force AGA only for entry;
     * restore the user's ECSGAMES preference immediately afterwards. */
    if (argc >= 2 && !stricmp(argv[1], "SYSTEMWIDE")) {
        if (argc >= 3 && !stricmp(argv[2], "?")) return sandbox("?");
        if (argc >= 3 && !stricmp(argv[2], "OFF")) return sandbox("OFF");
        if (argc >= 3 && stricmp(argv[2], "ON")) {
            PutStr("usage: agaboot SYSTEMWIDE [ON|OFF|?]\n");
            return 5;
        }
        {
            char old_ecs[16];
            LONG old_n = GetVar((STRPTR)ECS_VAR, old_ecs, sizeof old_ecs, GVF_GLOBAL_ONLY);
            STRPTR aga_v = (STRPTR)"0";
            int rc;
            SetVar((STRPTR)ECS_VAR, aga_v, -1, GVF_GLOBAL_ONLY);
            rc = sandbox("ON");
            if (old_n > 0) {
                if (old_n >= (LONG)sizeof old_ecs) old_n = (LONG)sizeof old_ecs - 1;
                old_ecs[old_n] = 0;
                SetVar((STRPTR)ECS_VAR, (STRPTR)old_ecs, -1, GVF_GLOBAL_ONLY);
            } else {
                DeleteVar((STRPTR)ECS_VAR, GVF_GLOBAL_ONLY);
            }
            if (!rc && aga_active()) {
            struct GfxBase *sysgb = (struct GfxBase *)OpenLibrary("graphics.library", 39);
            if (!sysgb) {
                PutStr("agaboot: SYSTEMWIDE could not reopen graphics.library\n");
                return 20;
            }
            {
                ULONG os_copinit = (ULONG)sysgb->copinit;
                CacheClearU();
                Disable();
                AGA_SYS_COPPER = os_copinit;
                Enable();
                Printf("agaboot: SYSTEMWIDE virtual OS Copper seed requested: $%08lx\n",
                       os_copinit);
            }
            CloseLibrary((struct Library *)sysgb);
            PutStr("agaboot: SYSTEMWIDE AGA active through existing output path; run the executable now\n");
        }
            return rc;
        }
    }
    if (argc >= 2 && !stricmp(argv[1], "MODE")) {
        int aga = aga_active();
        PutStr(aga ? "AGA\n" : "ECS\n");
        return aga ? 5 : 0;
    }
    if (argc >= 2 && !stricmp(argv[1], "MENU")) {
        int rc = 0;
        if (file_exists(MENU_FILE) && !write_menu(MENU_FILE, 0)) {
            Printf("agaboot: cannot write %s\n", (ULONG)MENU_FILE);
            rc = 10;
        }
        if (file_exists(DOPUS_MENU)) {
            if (!file_exists(DOPUS_BAK)) copy_file(DOPUS_MENU, DOPUS_BAK);
            static const char *const about_item[2] = { ABOUT_LABEL, ABOUT_CMD };
            int r = dopus_menu_update(DOPUS_MENU, "Chipset", MENU_LABEL, MENU_CMD,
                                      NULL, NULL, about_item, 1);
            if (r) { Printf("agaboot: cannot update %s (%ld)\n", (ULONG)DOPUS_MENU, (long)r); rc = 10; }
        }
        return rc;
    }
    if (argc < 2 || (stricmp(argv[1], "AGA") && stricmp(argv[1], "ECS"))) {
        PutStr("usage: agaboot AGA|ECS [ASK] | MODE [SYSTEM] | MENU | ABOUT\n"
               "       agaboot RUN <program> [args...]  scoped AGA session; physical HAM starts with child's display\n"
               "       agaboot AGAGAMES ON|OFF|ASK   emulate AGA for WHDLoad games\n"
               "       agaboot ECSGAMES AUTO|ON|OFF|ASK  chipset per game, or forced\n"
               "       agaboot SANDBOX AUTO|ON|OFF|?   AUTO is the WHDLoad hook path; ordinary apps use RUN\n"
               "       agaboot OUTPUT HDMI|HAM6|? select AGA presentation backend\n"
               "       agaboot ECSOUTPUT AUTO|NATIVE|DENISE|?  ECS physical presentation policy\n"
               "       agaboot HAMBUFFER ON|OFF|?    physical HAM page flip test; OFF=V0.14.1 direct\n"
               "       agaboot HAMAREA 320x256|352x272|368x280|384x280|?  lowres HAM canvas\n"
               "       agaboot HAMFPS 25|50|?   physical HAM publish fps; PAL timing stays 50Hz\n"
               "       agaboot AUDIO ENGINE SOFTWARE|NATIVE|AUTO|?  physical Paula policy\n"
               "       agaboot REDIRECT 0|1|2     chip RAM via the abort handler, chipset untouched\n");
        return 5;
    }
    int aga = stricmp(argv[1], "AGA") == 0;
    int with_ask = argc >= 3 && !stricmp(argv[2], "ASK");
    if (with_ask) {
        if (aga_active() == aga) {
            struct EasyStruct es = { sizeof(struct EasyStruct), 0, (UBYTE *)"AGA-PISTORM",
                                     (UBYTE *)(aga ? "AGA mode is already active." : "ECS mode is already active."),
                                     (UBYTE *)"OK" };
            IntuitionBase = (struct IntuitionBase *)OpenLibrary("intuition.library", 37);
            if (IntuitionBase) { EasyRequestArgs(NULL, &es, NULL, NULL); CloseLibrary((struct Library *)IntuitionBase); }
            return 0;
        }
        if (!ask(aga ? "Reboot the PiStorm into AGA mode?\n(one boot; the next reboot returns to ECS)"
                     : "Reboot the PiStorm into ECS mode?"))
            return 0;
    } else {
        PutStr(aga ? "Rebooting into AGA mode (one-shot; a normal reboot returns to ECS)...\n"
                   : "Rebooting into ECS mode...\n");
        Delay(25);
    }
    Disable();
    AGA_CTRL = aga ? MAGIC_AGA : MAGIC_ECS;
    /* still here? the Emu68 build does not support the switch */
    Enable();
    Delay(50);
    PutStr("This Emu68 does not support the AGA/ECS switch (no AGA-PISTORM build?)\n");
    return 10;
}
