# Changelog

## v0.1.2 — 2026-09-28

- Added `agaboot RUN <program> [args...]` for program-scoped virtual AGA/HAM6 sessions.
- Preserved the proven ordinary `SANDBOX ON/OFF` HAM6 path, including the visible V0.9 FIRST-LIGHT colour bars and the existing return-to-real-chipset cleanup; `RUN` uses a separate entry mode.
- Documented `SANDBOX AUTO` as the WHDLoad-hook path rather than the launcher for ordinary AmigaOS applications.
- Added a hidden physical HAM preparation path for `RUN`: the real HAM screen opens behind the current display, `MakeScreen()` prepares its Intuition ViewPort, and a private `MrgCop()` builds the hardware Copper list without `RethinkDisplay()`, `LoadView()` or `ScreenToFront()`.
- Added an explicit CPU1 start gate so publishing the prepared HAM descriptor cannot force physical takeover during a `RUN` session.
- Suppressed the diagnostic HAM first-light bars on delayed program-scoped handover, so the first intended visible content is the child program rather than a test pattern.
- Program-scoped sessions seed the current AmigaOS Copper as a baseline and release physical HAM6 only after a different Copper list has produced two completed display frames.
- `RUN` is synchronous: when the child exits, `agaboot` automatically leaves the sandbox and restores the prior chipset/display state.
- Preserved the existing Hires/SHRES post-render 2:1 horizontal reduction; `640x256` AGA Hires therefore presents as `320x256` with `HAMAREA 320x256`.
- Added `+HAM6 Accelerator` beneath the custom AGA logo on the Emu68 HDMI boot splash.
- Kept the upstream Emu68 base pinned to `v1.1.0-beta.1` commit `0ba3a899341dee958332a4182911861420b03bff`.

## Initial clean repository line — 2026-09-28

- Rebased the public integration onto upstream Emu68 1.1.0-beta.1 commit `0ba3a899341dee958332a4182911861420b03bff`.
- Preserved the current software AGA/ECS renderer and physical HAM6 presenter source.
- Preserved dynamic HAM area, HAM publish rate and physical page-flip controls.
- Preserved the post-render low-resolution HAM6 presentation stage for hires/SHRES input.
- Added standalone CPU3 chipset ownership and standalone CPU1 physical presentation so the repository has no dependency on unrelated FRF service workers.
- Carried forward current descriptor handoff, system-wide Copper seeding, native-audio control hooks, cross-core AGA interrupt publication and Classic PiStorm bus serialization.
- Removed stale backup/build artefacts and unrelated experimental subsystems from the publishable source tree.
