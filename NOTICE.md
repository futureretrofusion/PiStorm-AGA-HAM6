# Attribution and upstream notice

## AGA/ECS and HAM6 lineage

This project includes AGA/ECS emulation work from the UAE/E-UAE lineage. The HAM6 output path specifically originates from the AmigaOS graphics backend in E-UAE, principally `src/gfx-amigaos/ami-win.c`. That path was adapted into a reusable RGB-to-HAM6 output engine and later accelerated by moving physical HAM6 conversion/presentation work onto a spare Raspberry Pi CPU core.

The E-UAE `ami-win.c` file credits Samuel Devulder and Richard Drummond for the Amiga interface. UAE-family contributors include Bernd Schmidt, Toni Wilen and others. E-UAE is distributed under GNU GPL version 2; E-UAE-derived portions in this repository retain GPL-2.0 provenance.

## Emu68

This repository targets Emu68 by Michal Schulz:

```text
https://github.com/michalsc/Emu68
commit 0ba3a899341dee958332a4182911861420b03bff
Emu68 1.1.0-beta.1
license: MPL-2.0
```

Emu68 is not vendored here. `arm/emu68-aga.patch` describes the integration changes required by this project against that upstream revision.

## PiStorm

PiStorm hardware, firmware interfaces and related upstream projects remain the work of their respective authors and contributors. This repository is an experimental AGA/HAM6 integration project and is not an official upstream Emu68 or PiStorm release.
