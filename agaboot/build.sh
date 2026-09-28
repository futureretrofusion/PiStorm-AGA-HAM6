#!/bin/sh
# Amiga-side utility, built with the m68k cross compiler.
set -e
export PATH=/opt/amiga/bin:$PATH
cd "$(dirname "$0")"
m68k-amigaos-gcc -O2 -noixemul -o agaboot.new agaboot.c dopusmenu.c setup.c pointer.c
ls -l agaboot.new
