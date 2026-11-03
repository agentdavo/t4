#!/usr/bin/env bash
#
# run.sh - render one picture with the C ray tracer on t4 and keep it.
#
#   ./run.sh N [model]          # N nodes as threads of one t4 (-st), model 3
#   ./run.sh -sl N [model]      # one t4 process a node, shared-memory links
#
# Picks raytraceN.btl/.map from $BIN, answers the model menu, and leaves
# ray-nN-mM.ppm, .png and .log in $OUT.  At 256 x 256 the picture is
# compared with ../pic/rayM.ppm, which the occam build renders: it must be
# identical, whatever N.
#
# env:
#   BIN      boot files from build.sh (bin)
#   OUT      results ($BIN)
#   T4       the emulator (../../../t4)
#   TIMEOUT  seconds (7200)
#   WORK     scratch directory t4 runs in (a new one under $TMPDIR)
#
# Node 0 holds the whole picture, 12 MB at 2048 x 2048, so above 256 it
# gets t4 -sm 24 (16 MB); threaded nodes all get the same size.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SL=0
if [ "${1:-}" = "-sl" ]; then SL=1; shift; fi
[ $# -ge 1 ] || { sed -n '3,22p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }
N="$1"
MODEL="${2:-3}"
BIN="${BIN:-$HERE/bin}"
case "$BIN" in /*) ;; *) BIN="$PWD/$BIN" ;; esac
OUT="${OUT:-$BIN}"
T4="${T4:-$(cd "$HERE/../../.." && pwd)/t4}"
TIMEOUT="${TIMEOUT:-7200}"
BTL="$BIN/raytrace$N.btl"
MAP="$BIN/raytrace$N.map"
[ -f "$BTL" ] && [ -f "$MAP" ] || { echo "run.sh: no $BTL / $MAP" >&2; exit 1; }
RES=$(sed -n 's/^RES=\([0-9]*\).*/\1/p' "$BIN/BUILD" 2>/dev/null)
RES="${RES:-2048}"
SM=""
[ "$RES" -gt 256 ] && SM="-sm 24"

WORK="${WORK:-$(mktemp -d "${TMPDIR:-/tmp}/raytrace-n$N.XXXXXX")}"
mkdir -p "$WORK" "$OUT"
cp "$MAP" "$WORK/net.map"
export SPYNET="$WORK/net.map" T4_HEADLESS=1
NAME="ray-n$N-m$MODEL"
LOG="$OUT/$NAME.log"

PIDS=""
cleanup() {
    local p
    # disown first, or bash reports each SIGKILL as "Killed ..."
    for p in $PIDS; do disown "$p" 2>/dev/null; kill -9 "$p" 2>/dev/null; done
    for p in $PIDS; do while kill -0 "$p" 2>/dev/null; do sleep 0.1; done; done
    PIDS=""
    [ "$SL" = 1 ] && drop_shm
    return 0
}
trap 'cleanup; exit 130' INT TERM

# -sl: node 1 creates a SysV segment keyed ftok(map, 1984) that the rest
# attach to (see ../bin/testall.sh, which this follows).
shm_key() {
    local dev ino
    eval "$(stat -c 'dev=%d ino=%i' "$SPYNET")"
    printf '0x%08x' $(( (192 << 24) | ((dev & 255) << 16) | (ino & 65535) ))
}
shm_exists() { ipcs -m | awk -v k="$(shm_key)" '$1 == k { f = 1 } END { exit !f }'; }
drop_shm() {
    local id
    id=$(ipcs -m | awk -v k="$(shm_key)" '$1 == k { print $2 }')
    [ -n "$id" ] && ipcrm -m "$id" 2>/dev/null
    return 0
}

cd "$WORK" || exit 1
rm -f ray.ppm
start=$SECONDS
if [ "$SL" = 0 ]; then
    printf '%s\n' "$MODEL" |
        timeout -k 5 "$TIMEOUT" "$T4" -st -s8 $SM -sn "-$N" -sb "$BTL" 2>&1 |
        head -c 4000000 > "$LOG"
else
    drop_shm
    if [ "$N" -gt 1 ]; then
        "$T4" -s8 -sl -sn 1 </dev/null >/dev/null 2>&1 &
        PIDS="$PIDS $!"
        w=0
        until shm_exists || [ $w -ge 100 ]; do sleep 0.1; w=$((w + 1)); done
        for i in $(seq 2 $((N - 1))); do
            "$T4" -s8 -sl -sn "$i" </dev/null >/dev/null 2>&1 &
            PIDS="$PIDS $!"
        done
        sleep $(( 1 + N / 40 ))
    fi
    printf '%s\n' "$MODEL" |
        timeout -k 5 "$TIMEOUT" "$T4" -s8 $SM -sl -sn 0 -sb "$BTL" 2>&1 |
        grep -av '^-I-EMUBOOT\|^  [0-9A-F]\{4\}: ' | head -c 4000000 > "$LOG"
    cleanup
fi
wall=$((SECONDS - start))

took=$(tr '\r\0' '\n\n' < "$LOG" | sed -n 's/.*DONE, took *\([0-9.]*\) seconds.*/\1/p' | head -1)
if ! grep -aq "OK, all done" "$LOG" || [ ! -f ray.ppm ]; then
    echo "n=$N model=$MODEL: FAIL after ${wall}s, see $LOG"
    exit 1
fi
mv ray.ppm "$OUT/$NAME.ppm"
verdict=""
if [ "$RES" = 256 ]; then
    if cmp -s "$OUT/$NAME.ppm" "$HERE/../pic/ray$MODEL.ppm"; then
        verdict=", identical to pic/ray$MODEL.ppm"
    else
        echo "n=$N model=$MODEL: MISMATCH with pic/ray$MODEL.ppm" \
             "($(cmp -l "$OUT/$NAME.ppm" "$HERE/../pic/ray$MODEL.ppm" | wc -l) bytes)"
        exit 1
    fi
fi
python3 "$HERE/ppm2png.py" "$OUT/$NAME.ppm" "$OUT/$NAME.png" > /dev/null
rays=$(tr '\r\0' '\n\n' < "$LOG" | sed -n 's/.*Worker *[0-9]* DONE, *\([0-9]*\) rays.*/\1/p' |
       awk '{ s += $1; n++ } END { printf "%d workers, %d rays", n, s }')
echo "n=$N model=$MODEL: ${RES}x$RES in ${took:-?}s (wall ${wall}s), $rays$verdict"
[ -z "${KEEP_WORK:-}" ] && rm -rf "$WORK"
exit 0
