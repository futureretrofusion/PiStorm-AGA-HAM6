# Building PiStorm AGA HAM6

## Requirements

A Linux development host with Git, CMake, an AArch64 cross toolchain suitable for Emu68, and `m68k-amigaos-gcc` for the Amiga-side utilities.

The integration is pinned to upstream Emu68 commit:

```text
0ba3a899341dee958332a4182911861420b03bff
```

Using another Emu68 revision should be treated as a porting task rather than assuming the patch is compatible.

## 1. Prepare upstream Emu68

```bash
git clone https://github.com/michalsc/Emu68.git
cd Emu68
git checkout 0ba3a899341dee958332a4182911861420b03bff
git status --short
```

The status should be clean.

## 2. Apply the integration patch

From the Emu68 checkout:

```bash
/path/to/PiStorm-AGA-HAM6/scripts/apply-emu68-patch.sh "$PWD"
```

The helper refuses the wrong upstream commit or a dirty worktree.

The helper applies two ordered patches: the main AGA/HAM6 integration followed by the scoped-program/splash patch. To inspect manually, apply/check them in the same order; the second patch is intentionally based on the first patch's result.

## 3. Configure PiStorm Classic

```bash
cmake -S . -B build-aga-ham6 \
  -DTARGET=raspi64 \
  -DVARIANT=pistorm-classic \
  -DAGA_PISTORM=/absolute/path/to/PiStorm-AGA-HAM6 \
  -DCMAKE_TOOLCHAIN_FILE=toolchains/aarch64-linux-gnu.cmake
```

Useful integration options supplied by the patch:

```text
-DAGA_DIAG=ON              per-trap / diagnostic instrumentation
-DAGA_PROBES=ON            deeper test-kernel probes
-DAGA_PI3=ON               Pi 3-specific kernel path
-DEMU68_SKIP_PPC_ROM=ON    build without requiring the PPC ROM cross-build
-DEMU68_SKIP_FIRMWARE=ON   do not download Raspberry Pi firmware files
```

Diagnostic/probe options are for investigation and should normally remain off for a release build.

## 4. Build firmware

```bash
cmake --build build-aga-ham6 -j2
```

Use the resulting Emu68 firmware according to the normal PiStorm/Emu68 deployment procedure for your system. Keep a known-good firmware available before hardware testing a development build.

## 5. Build `agaboot`

```bash
cd /path/to/PiStorm-AGA-HAM6/agaboot
chmod +x build.sh
./build.sh
```

This produces `agaboot.new`. Install it as your matching `C:agaboot` only when you are also using the corresponding firmware source line.

`agastat` is built similarly:

```bash
cd /path/to/PiStorm-AGA-HAM6/agastat
chmod +x build.sh
./build.sh
```

## Core ownership

For this clean standalone integration:

```text
CPU0  Emu68 68K/JIT + trap side
CPU1  physical HAM6/ECS presenter
CPU2  normal PiStorm housekeeper / AGA audio bus service
CPU3  software AGA/ECS chipset renderer
```

The CPU1 physical presenter is incompatible with Emu68 `async_log`, because upstream assigns the same core to its asynchronous serial writer.
