#!/bin/sh
# Fetch openamr-platform-fw at the commit pinned in launch/sil.launch.py (SIMREF_FW_SHA) and
# build the SIMULATION ONLY reference process of tests/sim_reference for this Linux host.
# Usage: build_simref.sh [output directory]   (default ~/.cache/openamrobot_simref/<sha>)
# The binary is written with <binary>.sha, which sil.launch.py checks before it starts it.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
LAUNCH="$HERE/../launch/sil.launch.py"
SHA=$(sed -n "s/^SIMREF_FW_SHA = '\([0-9a-f]*\)'$/\1/p" "$LAUNCH")
REPO=$(sed -n "s/^SIMREF_FW_REPO = '\(.*\)'$/\1/p" "$LAUNCH")
[ -n "$SHA" ] || { echo "BLOCKED: no SIMREF_FW_SHA in $LAUNCH"; exit 1; }
OUT=${1:-$HOME/.cache/openamrobot_simref/$SHA}
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
git init -q "$WORK/fw"
git -C "$WORK/fw" fetch -q --depth 1 "$REPO" "$SHA"
git -C "$WORK/fw" checkout -q FETCH_HEAD
GOT=$(git -C "$WORK/fw" rev-parse HEAD)
[ "$GOT" = "$SHA" ] || { echo "fetched $GOT, expected $SHA"; exit 1; }
cmake -S "$WORK/fw/tests/sim_reference" -B "$WORK/build" -DSIMREF_SANITIZE=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build "$WORK/build" --target simref_process
mkdir -p "$OUT"
cp "$WORK/build/simref_process" "$OUT/simref_process"
echo "$SHA" > "$OUT/simref_process.sha"
echo "simref_process at $SHA: $OUT/simref_process"
