#!/usr/bin/env bash
#
# testall.sh - run every raytraceN arrangement and verify the rendered image.
#
# The point of this script is not timing, it is correctness.  The raytracer is
# deterministic: the same scene rendered on 1 transputer and on 80 must produce
# byte-identical output.  pic/rayM.ppm are the reference images shipped with
# the example, so every (arrangement, model) pair has an exact oracle - which
# makes this a real end-to-end test of the link fabric, the chain bootstrap and
# the task farm, not just a smoke test.
#
#   ./testall.sh                  # every arrangement, model 3, shared-memory links
#   ./testall.sh --max 8          # arrangements up to 8 processors
#   ./testall.sh --ipc            # nanomsg ipc:// instead of -sl (needed at N>=8
#                                 # under load; SHM idle-polling storms the CPU)
#   ./testall.sh --models 1,2,3,4 # all four scenes
#   ./testall.sh --sizes 1,2,4    # just these
#
# Exit status is 0 only if every run rendered and matched its reference.
#
# The 16-node boot failure is fixed (it was the Link0In read shim in p.c
# shadowing a worker's channel control word - see word_int()).  Idle nodes
# used to burn a full second of usleep(50) spinning; they now poll with a
# few-millisecond timeout and 1ms slices, so large arrangements are no
# longer pathologically slow.  Oversubscribed runs still take wall time,
# so give them a budget.
#
# Results are only meaningful on an idle machine.  This box also runs chip
# jobs (yosys, klayout, ngspice); at load > ~10 runs fail spuriously with
# rc=255 or time out at sizes that pass when it is quiet.  Check
# /proc/loadavg first.
#
# Note on process handling: this script kills only the emulator PIDs it
# started.  `pkill t4` (as bin/multi.sh does) would take out any other t4 on
# the machine - a kernel boot, another test run - and a stray emulator burns a
# core and makes unrelated runs look like hangs.
#
# Note on links.  Makefile.linux now enables the ipc:// transport when nanomsg
# is present (it used to be stubbed out, every nn_* returning EINVAL).  This
# script still uses shared-memory links (-sl), which impose two rules that
# bin/README and bin/multi.sh both used to get wrong:
#
#   1. Node 1 CREATES the segment (p.c: `if ((1 == nodeid) || (0 == maxnode))
#      shlink_alloc(...)`); every other node attaches and calls handler(-1) -
#      exits - if it is not there yet.  So node 1 must be started first and be
#      given time to win, before nodes 2..N-1 and before node 0.  Starting all
#      the workers at once is a race that kills node 2 at N>=3.
#
#   2. The segment is SysV shared memory keyed by ftok(<mapfile>, 1984), and
#      shlink_alloc REFUSES to run if it already exists ("SHMLinks already
#      exists").  A run killed before shlink_free leaves the segment behind
#      and poisons every later run of the same size.  So the key is computed
#      here and any stale segment is removed before and after each run.

set -u

BIN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXAMPLE_DIR="$(cd "$BIN_DIR/.." && pwd)"
PIC_DIR="$EXAMPLE_DIR/pic"
T4="${T4:-$(cd "$BIN_DIR/../../.." && pwd)/t4}"

MODELS="3"
SIZES=""
MAXN=""
BASE_TIMEOUT="${BASE_TIMEOUT:-120}"   # seconds for a 1-processor run
KEEP=0
VERBOSE=0
SL="-sl"   # shared-memory links; --ipc selects nanomsg ipc:// instead
THREADS=0

usage() {
    sed -n '2,25p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --models)  MODELS="${2//,/ }"; shift 2 ;;
        --sizes)   SIZES="${2//,/ }";  shift 2 ;;
        --max)     MAXN="$2";          shift 2 ;;
        --timeout) BASE_TIMEOUT="$2";  shift 2 ;;
        --ipc)     SL="";              shift ;;
        --threads) THREADS=1; SL="";   shift ;;
        --keep)    KEEP=1;             shift ;;
        -v|--verbose) VERBOSE=1;       shift ;;
        -h|--help) usage 0 ;;
        *) echo "unknown option: $1" >&2; usage 1 ;;
    esac
done

[ -x "$T4" ] || { echo "emulator not found or not executable: $T4" >&2; exit 1; }

# Arrangements are whatever boot images are actually present.
if [ -z "$SIZES" ]; then
    SIZES=$(ls "$BIN_DIR"/raytrace*.btl 2>/dev/null |
            sed 's|.*/raytrace||; s|\.btl$||' | sort -n | tr '\n' ' ')
fi
[ -n "${SIZES// /}" ] || { echo "no raytraceN.btl files in $BIN_DIR" >&2; exit 1; }
if [ -n "$MAXN" ]; then
    filtered=""
    for n in $SIZES; do [ "$n" -le "$MAXN" ] && filtered="$filtered $n"; done
    SIZES="$filtered"
fi

NCPU=$(nproc 2>/dev/null || echo 1)

# Every emulator process this script has started, so cleanup is targeted.
PIDS=""
cleanup_pids() {
    # disown first, or bash announces every SIGKILL as "Killed  ( cd ... )"
    # in the middle of the results table.
    local p waited=0 alive
    for p in $PIDS; do disown "$p" 2>/dev/null; kill -9 "$p" 2>/dev/null; done

    # SIGKILL is not instant, and an emulator that has not finished dying is
    # still attached to the shared segment - which keeps the segment alive
    # through the next run's ipcrm, so that run's node 1 cannot create it and
    # the whole network dies at once (rc=255, a few seconds in).  That was
    # the last intermittent failure.  Wait for them to actually go.
    while [ "$waited" -lt 100 ]; do
        alive=0
        for p in $PIDS; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ "$alive" = 0 ] && break
        sleep 0.1; waited=$((waited + 1))
    done

    PIDS=""
    wait 2>/dev/null
    return 0
}
trap 'echo; echo "interrupted - stopping emulators"; cleanup_pids; exit 130' INT TERM

# ftok(path, 1984) as glibc computes it: proj_id in the top byte, the low byte
# of st_dev next, then the low 16 bits of st_ino.
shm_key_for() {
    local f="$1" dev ino
    eval "$(stat -c 'dev=%d ino=%i' "$f")"
    printf '0x%08x' $(( (192 << 24) | ((dev & 255) << 16) | (ino & 65535) ))
}

shm_exists() {
    ipcs -m | awk -v k="$1" '$1 == k { found = 1 } END { exit !found }'
}

# Drop a leftover segment so node 1 can create a fresh one.
#
# ipcrm only *marks* a segment for destruction: it lives on until the last
# process detaches.  So a run started while the previous run's emulators are
# still dying finds the segment still present, node 1's shmget(IPC_CREAT)
# fails with "SHMLinks already exists", and the whole network collapses
# immediately (rc=255, zero seconds).  That was an intermittent failure at
# every size - whichever run happened to follow a slow one.  Wait it out.
drop_shm() {
    local key="$1" id waited=0
    id=$(ipcs -m | awk -v k="$key" '$1 == k { print $2 }')
    [ -n "$id" ] && ipcrm -m "$id" 2>/dev/null
    while shm_exists "$key" && [ "$waited" -lt 100 ]; do
        sleep 0.1; waited=$((waited + 1))
    done
    return 0
}

# t4 drops these into its working directory.
clean_artifacts() {
    rm -f "$BIN_DIR"/boot-*.uart "$BIN_DIR"/boot.uart "$BIN_DIR"/framebuffer.bmp
    [ "$KEEP" = 1 ] || rm -f "$BIN_DIR"/ray.ppm
}

RESULTS=""     # "N model verdict wall reported detail"
FAILURES=0
RUNS=0

run_one() {
    local n="$1" model="$2"
    local btl="$BIN_DIR/raytrace$n.btl" map="$BIN_DIR/raytrace$n.map"
    local ref="$PIC_DIR/ray$model.ppm"
    local log="$BIN_DIR/.testall-n$n-m$model.log"

    RUNS=$((RUNS + 1))

    if [ ! -f "$btl" ] || [ ! -f "$map" ]; then
        RESULTS="$RESULTS$n|$model|SKIP|-|-|no btl/map\n"; return
    fi
    if [ ! -f "$ref" ]; then
        RESULTS="$RESULTS$n|$model|SKIP|-|-|no reference $ref\n"; return
    fi

    # Boot is serialised down the chain and the workers share $NCPU cores, so
    # the budget has to grow with N on both counts.
    local timeout_s=$(( BASE_TIMEOUT + n * 8 ))
    if [ "$n" -gt "$NCPU" ]; then
        timeout_s=$(( timeout_s * (n + NCPU - 1) / NCPU ))
    fi

    rm -f "$BIN_DIR/ray.ppm"
    export SPYNET="$map"
    export T4_HEADLESS=1

    local key
    key=$(shm_key_for "$map")
    drop_shm "$key"          # a previous run may have died holding it

    PIDS=""
    local i rc wall start
    if [ "$THREADS" = 1 ]; then
        start=$SECONDS
        ( cd "$BIN_DIR" && printf '%s\n' "$model" |
          timeout -k 5 "$timeout_s" "$T4" -st -s8 -sn "-$n" -sb "$btl" ) >"$log" 2>&1
        rc=$? wall=$(( SECONDS - start ))
    elif [ "$n" -gt 1 ]; then
        # Shared-memory links: node 1 creates the segment; nobody else may
        # start until it exists.  ipc:// has no such constraint.
        ( cd "$BIN_DIR" && exec "$T4" -s8 $SL -sn 1 ) </dev/null >/dev/null 2>&1 &
        PIDS="$PIDS $!"
        if [ -n "$SL" ]; then
            local waited=0
            until shm_exists "$key" || [ "$waited" -ge 100 ]; do
                sleep 0.1; waited=$((waited + 1))
            done
            if ! shm_exists "$key"; then
                cleanup_pids
                RESULTS="$RESULTS$n|$model|FAIL|-|-|node 1 never created SHM $key\n"
                printf '  %-4s %-6s %-9s %-7s %s\n' "$n" "$model" FAIL - "node 1 never created SHM segment"
                FAILURES=$((FAILURES + 1))
                return
            fi
        else
            sleep 0.3
        fi
        for i in $(seq 2 $((n - 1))); do
            ( cd "$BIN_DIR" && exec "$T4" -s8 $SL -sn "$i" ) </dev/null >/dev/null 2>&1 &
            PIDS="$PIDS $!"
        done
        sleep $(( 1 + n / 40 ))
    fi

    if [ "$THREADS" != 1 ]; then
    start=$SECONDS
    # -k: t4 installs a SIGTERM handler, so plain timeout can leave it running.
    ( cd "$BIN_DIR" && printf '%s\n' "$model" |
      timeout -k 5 "$timeout_s" "$T4" -s8 $SL -sn 0 -sb "$btl" ) >"$log" 2>&1
    rc=$? wall=$(( SECONDS - start ))
    fi

    cleanup_pids
    drop_shm "$key"

    local reported detail verdict
    reported=$(sed -n 's/.*DONE, took *\([0-9.]*\) seconds.*/\1/p' "$log" | head -1)
    [ -n "$reported" ] || reported="-"

    if [ "$rc" -eq 124 ]; then
        verdict=TIMEOUT; detail="killed after ${timeout_s}s"
    elif ! grep -q "OK, all done" "$log"; then
        verdict=FAIL;    detail="no completion message (rc=$rc)"
    elif [ ! -f "$BIN_DIR/ray.ppm" ]; then
        verdict=FAIL;    detail="no ray.ppm written"
    elif ! cmp -s "$BIN_DIR/ray.ppm" "$ref"; then
        local nd
        nd=$(cmp -l "$BIN_DIR/ray.ppm" "$ref" 2>/dev/null | wc -l)
        verdict=MISMATCH; detail="$nd bytes differ from $(basename "$ref")"
    else
        verdict=PASS
        # Load balance across workers is the interesting number once it works.
        local rays
        rays=$(sed -n 's/.*Worker *[0-9]* DONE, *\([0-9]*\) rays.*/\1/p' "$log")
        if [ -n "$rays" ]; then
            detail=$(echo "$rays" | awk '
                {v[NR]=$1; s+=$1; if(NR==1||$1<mn)mn=$1; if($1>mx)mx=$1}
                END{ if(NR>1 && s>0) printf "%d workers, %.1f%% imbalance", NR, 100.0*(mx-mn)/(s/NR);
                     else printf "%d worker", NR }')
        else
            detail="rendered"
        fi
    fi

    [ "$verdict" = PASS ] && [ "$VERBOSE" = 0 ] && rm -f "$log"
    [ "$verdict" = PASS ] || FAILURES=$((FAILURES + 1))

    if [ "$KEEP" = 1 ] && [ "$verdict" = PASS ]; then
        cp "$BIN_DIR/ray.ppm" "$BIN_DIR/ray-n$n-m$model.ppm"
    fi

    RESULTS="$RESULTS$n|$model|$verdict|${wall}s|$reported|$detail\n"
    printf '  %-4s %-6s %-9s %-7s %s\n' "$n" "$model" "$verdict" "${wall}s" "$detail"
}

echo "raytracer arrangement test"
echo "  emulator   : $T4"
echo "  arrangements: $SIZES"
echo "  models     : $MODELS"
echo "  links      : $([ "$THREADS" = 1 ] && echo '-st threads' || echo "${SL:-ipc://}")"
echo "  host cores : $NCPU"
maxsize=$(echo $SIZES | tr ' ' '\n' | sort -n | tail -1)
if [ "${maxsize:-1}" -gt "$NCPU" ]; then
    echo "  note       : up to $maxsize emulators on $NCPU cores - runs are"
    echo "               oversubscribed, so wall time is not a speedup measure."
fi
echo
printf '  %-4s %-6s %-9s %-7s %s\n' N model verdict wall detail
printf '  %-4s %-6s %-9s %-7s %s\n' ---- ----- ------- ---- ------

for model in $MODELS; do
    for n in $SIZES; do
        run_one "$n" "$model"
    done
done

clean_artifacts

echo
echo "summary"
printf '%b' "$RESULTS" | awk -F'|' '
    NF>=3 { c[$3]++ }
    END { for (k in c) printf "  %-9s %d\n", k, c[k] }' | sort
echo "  total     $RUNS"

if [ "$FAILURES" -eq 0 ]; then
    echo
    echo "all arrangements produced byte-identical output to the reference images."
    exit 0
fi
echo
echo "$FAILURES run(s) did not match; logs kept as $BIN_DIR/.testall-n*-m*.log"
exit 1
