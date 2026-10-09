#!/bin/bash
# Host test of the MiSTer-side rig translator (physical_disc_rig.cpp) against a fake servo board on a
# pty, in real time. Needs g++ and python3; runs on Linux or WSL. The fake board logs every command it
# receives with a timestamp, so the sequencing and timing can be read off the end of the output.
#
#   bash run.sh
cd "$(dirname "$0")"
S=$(cd .. && pwd)/..
g++ -O1 -Wall -I"$S" -o /tmp/rigtest harness.cpp "$S/physical_disc_rig.cpp" "$S/acoustic_model.cpp" "$S/cd_geometry.cpp" 2>&1 | head -30
python3 fakeboard.py /tmp/fakeboard.log > /tmp/fakeboard.out 2>&1 &
FB=$!
sleep 1
PD_RIG_PORT=/tmp/fakeacm timeout 120 /tmp/rigtest
kill $FB 2>/dev/null
echo "=== what the board received ==="
cat /tmp/fakeboard.log
