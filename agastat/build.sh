#!/bin/sh
set -e
export PATH=/opt/amiga/bin:$PATH
cd "$(dirname "$0")"
m68k-amigaos-gcc -O2 -noixemul -o agastat.new agastat.c
ls -l agastat.new
