# PiStorm AGA HAM6

Native AGA/ECS chipset emulation and physical HAM6 presentation for PiStorm Classic + Emu68.

This repository contains the AGA chipset core, Emu68 integration glue, the Amiga-side `agaboot` control utility, and the current physical HAM6 presenter. It is intentionally kept separate from unrelated PiStorm services so the display path can be built, reviewed and debugged on its own.

## Architecture

```text
68K software / WHDLoad
        |
        v
Emu68 CPU0 + trapped Chip RAM/custom-register access
        |
        v
CPU3: software AGA/ECS chipset + RGB framebuffer
        |
        v
RGB -> HAM6 low-resolution presentation stage
        |
        v
CPU1: physical HAM6/ECS presenter
        |
        v
real Amiga Chip RAM + Copper/VBlank -> RGB output
```

The virtual chipset and the physical display sink are deliberately separate. The renderer can work at its internal source geometry while the physical HAM6 sink uses a low-resolution Amiga-compatible canvas. Hires/SHRES content is reduced at the post-render presentation stage rather than treating HAM6 as a hires mode.

The current HAM6 presenter supports runtime physical canvases of `320x256`, `352x272`, `368x280`, and `384x280`, with 25 or 50 frame-per-second publish policy while PAL scanout remains 50 Hz. Optional double buffering keeps the physical VBlank/Copper publish point owned by CPU1.


## HAM6 provenance

The HAM6 output path in this project originates from the AmigaOS graphics work in **E-UAE**, principally `src/gfx-amigaos/ami-win.c`. The original Amiga-side HAM/display approach was adapted into a reusable PiStorm presentation engine that accepts rendered RGB output from the software AGA/ECS chipset implementation.

The path then evolved further for PiStorm/Emu68: the physical conversion and presentation work was moved onto a spare Raspberry Pi CPU core so it can run independently of the main 68k emulation workload. The resulting lineage is:

```text
E-UAE AmigaOS HAM/display implementation
        -> reusable RGB-to-HAM6 output engine
        -> PiStorm software AGA/ECS integration
        -> explicit Hires/SHRES-to-lowres presentation scaling
        -> independent physical framebuffer/Copper presentation
        -> spare-core accelerated HAM6 presentation
```

The E-UAE-derived portions retain their GNU GPL v2 provenance and attribution. Emu68 integration changes are separate from the E-UAE-derived HAM6 work and remain subject to the upstream Emu68 MPL-2.0 licensing terms for modified Emu68 files.

## Emu68 base

The integration patch targets upstream Emu68 **1.1.0-beta.1** at:

```text
0ba3a899341dee958332a4182911861420b03bff
```

Upstream repository: `michalsc/Emu68`.

`arm/emu68-aga.patch` is generated against that exact upstream commit. `arm/emu68-program-run.patch` is applied immediately afterwards and adds the scoped-program control words plus the `+HAM6 Accelerator` HDMI splash line. The upstream Emu68 source itself is not vendored here.

## Repository layout

| Path | Purpose |
| --- | --- |
| `core/` | Portable AGA/ECS chipset implementation: playfield, sprites, Copper, blitter, audio, disk and state |
| `arm/aga_glue.c` | Emu68/virtual-chipset glue and CPU3 renderer ownership |
| `arm/aga_ham6.c` | CPU3 -> CPU1 mailbox, RGB-to-HAM6 conversion and physical presentation |
| `arm/aga_ecs_denise.inc` | ECS/OCS physical presenter used by the same CPU1 owner |
| `arm/emu68-aga.patch` | Main patch for the exact upstream Emu68 base listed above |
| `arm/emu68-program-run.patch` | Follow-on patch for program-scoped AGA/HAM6 handover and the `+HAM6 Accelerator` HDMI splash |
| `agaboot/` | Amiga-side mode, sandbox and physical-output control utility |
| `agastat/` | Amiga-side diagnostics utility |
| `scripts/apply-emu68-patch.sh` | Safe patch helper for a clean upstream Emu68 checkout |

## Build

See [BUILDING.md](BUILDING.md) for the complete build flow.

The short version is:

```bash
git clone https://github.com/michalsc/Emu68.git
cd Emu68
git checkout 0ba3a899341dee958332a4182911861420b03bff

/path/to/PiStorm-AGA-HAM6/scripts/apply-emu68-patch.sh "$PWD"

cmake -S . -B build-aga-ham6 \
  -DTARGET=raspi64 \
  -DVARIANT=pistorm-classic \
  -DAGA_PISTORM=/path/to/PiStorm-AGA-HAM6 \
  -DCMAKE_TOOLCHAIN_FILE=toolchains/aarch64-linux-gnu.cmake

cmake --build build-aga-ham6 -j2
```

Build the matching Amiga utility with:

```bash
cd /path/to/PiStorm-AGA-HAM6/agaboot
./build.sh
```

Do not mix a newly built firmware with an unrelated `agaboot` binary: the descriptor/control protocol is part of the integration contract.

## Runtime controls

`agaboot` currently exposes the main presentation controls, including:

```text
agaboot OUTPUT HDMI|HAM6|?
agaboot RUN <program> [args...]
agaboot SANDBOX AUTO|ON|OFF|?
agaboot ECSOUTPUT AUTO|NATIVE|DENISE|?
agaboot HAMAREA 320x256|352x272|368x280|384x280|?
agaboot HAMFPS 25|50|?
agaboot HAMBUFFER ON|OFF|?
```

For the physical HAM6 path, CPU1 is dedicated to presentation when AGA support is active. Do not enable Emu68 `async_log` at the same time; upstream uses CPU1 for the asynchronous serial writer in that mode.

### Program-scoped HAM6

`SYSTEMWIDE ON` deliberately takes over the current AmigaOS display immediately. `SANDBOX AUTO` is the WHDLoad-hook path and should not be used to launch an ordinary AmigaOS application manually. For ordinary AGA applications use `RUN` instead:

```text
agaboot OUTPUT HAM6
agaboot HAMAREA 320x256
agaboot RUN AGS:AGS2Menu
```

`RUN` first prepares the real HAM6 sink invisibly: the native HAM screen is opened behind the current display and a private hardware Copper list is built without loading or fronting that screen. It then enters virtual AGA before launching the child so an AGA-only executable can open its screen. CPU1 remains gated until the child installs a different stable Copper list; two completed display frames from that list release the HAM6 presenter. When the child process exits, `agaboot` leaves the sandbox and restores the previous display automatically.

For a 640x256 AGA Hires source such as AGS2Menu, the renderer produces RGB at the virtual-chipset stage and the HAM presenter performs the existing Hires-to-lowres 2:1 horizontal reduction before HAM6 encoding, giving a 320x256 physical canvas when `HAMAREA 320x256` is selected.

## Current lineage

The source in this repository is reconstructed from the current hardware-tested AGA/HAM6 development line, but its Emu68 integration is rebased onto the clean upstream commit above. Unrelated experimental services and historical backup files are deliberately not part of this repository.

## License and attribution

The AGA/ECS chipset and HAM6 renderer code are GPL-2.0-only. See [LICENSE](LICENSE) and [NOTICE.md](NOTICE.md).

Emu68 1.1.0-beta.1 is licensed under MPL-2.0. The Emu68-covered source and modifications distributed in `arm/emu68-aga.patch` and `arm/emu68-program-run.patch`, and the modified Emu68 files produced by applying them, remain subject to MPL-2.0. The GPL statement above does not replace their upstream licence or notices. See [LICENSE-MPL-2.0](LICENSE-MPL-2.0).

This repository distributes integration patches, not a vendored Emu68 source tree. The exact upstream source revision is recorded in [EMU68-SOURCE.txt](EMU68-SOURCE.txt).
