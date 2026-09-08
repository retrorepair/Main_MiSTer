#!/bin/bash
# Cross-build MiSTer Main for the DE10-Nano from WSL, using ARM's official toolchain
# (the one setup_default_toolchain.sh fetches).  Run from Windows as:
#   powershell -NoProfile -Command "wsl -d Ubuntu -e bash /mnt/c/.../Main_MiSTer/wslbuild.sh"
set -e
SRC=/mnt/c/Users/joelw/Documents/MegaCD_MiSTer_New/Main_MiSTer
OUT=/mnt/c/Users/joelw/AppData/Local/Temp/claude/C--Users-joelw-Documents-MegaCD-MiSTer-New/2ec3372d-bd5a-402d-ac38-21cf2da5ce03/scratchpad/MiSTer_nukedmd
TC=$HOME/tc/gcc-arm-10.2-2020.11-x86_64-arm-none-linux-gnueabihf/bin
export PATH=$TC:$PATH
# Building on /mnt/c is slow, so keep a working copy in the WSL filesystem and sync sources in.
[ -d ~/main ] || cp -r "$SRC" ~/main
rsync -a --exclude bin --exclude obj --exclude .git "$SRC"/ ~/main/
cd ~/main
make 2>&1 | tail -20
cp bin/MiSTer "$OUT"
md5sum bin/MiSTer
