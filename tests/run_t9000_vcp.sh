#!/usr/bin/env bash
# T9000 channel tests (see ../T9000_CHANNELS.md):
#
#   test_t9000_vcp      one node: Link1 cabled to itself carries the virtual
#                       links, Link2/Link3 crossed in byte-stream mode, event
#                       pins looped (T4_LINK_LOOPBACK, T4_EVENT_LOOPBACK)
#   test_t9000_vcp_net  two threaded nodes: node 0 boots node 1 down Link1,
#                       then both use it as a virtual link
#   test_t9000_c104_net two threaded nodes and two IMS C104 routers: virtual
#                       links across the switches, with header deletion and
#                       with global header labelling
#   test_t9000_ctl_net  two threaded nodes: node 0 configures, boots, stops
#                       and resets node 1 over node 1's control link, and
#                       receives its error reports
#   test_t9000_ctl_chain three threaded nodes on a daisy-chained control
#                       network: node 2 is controlled through node 1
#   test_t9000_pstack   one node: P-process stack extension on demand and
#                       device-access-only regions (ISM 9.3, 9.5)
#   test_t9000_pmode    one node: P-process instruction fetch (execute
#                       permission, relocated code), move/devmove/move2dall
#                       through the regions, and ps.Eptr for each trap
#   test_fused_cj       one node: the eqc;cj and gt;cj pairs t4 fuses leave
#                       the stack as the two instructions would (ISM cj)
#   test_link_sync      two threaded nodes: a byte-stream link output
#                       completes only once the far process has input it
#   test_alt_guard      one node: an ALT guard of 2 is true, and a false
#                       guard's disc leaves the channel alone (ISM enbc, disc)
#   test_timer_alt      one node: a timer ALT a channel won leaves the timer
#                       list (ISM dist), one its time woke is queued once,
#                       one a channel woke is queued once when its time
#                       comes before it runs, one whose time has passed
#                       carries on (taltwt), and the list keeps time order
#   test_thread_timeslice two threaded nodes: low priority processes are
#                       timesliced, and the low priority clock keeps time
#                       (TCWG 6.3.1)
#   test_timeslice_*    one node and two threaded nodes: a low priority
#                       process is timesliced after two periods, 256us
#                       each on a T9000 (-s9, ISM 8.2.4) and 1024us on a
#                       T800 (TCWG 6.3.1)
#   test_idle_mailbox   two threaded nodes: an idle node sleeps while a
#                       packet no process is waiting for sits in one of its
#                       link mailboxes (checked by t4's host CPU time)
#   test_timer_sleep    two threaded nodes: a node waiting only on a timer
#                       sleeps, a link wakes it, and the timers keep time
#                       (checked by t4's host CPU time too)
#   test_boot_links     three threaded nodes booted through different links:
#                       each node keeps its own boot link
#   test_timer_sleep_one one node: waiting only on a timer, it sleeps, and
#                       the timers keep time (checked by host CPU time)
#   test_sl_wake        two processes with shared-memory links (-sl): a
#                       node waiting on a link is woken by the far process
#
# Builds with transputer-tcc and transputer-netload from the top of the
# tinycc_transputer tree.  T4_BIN picks the emulator (default ../t4).
# Exit status: 0 if every test printed exactly "S\nALL-PASS\nH".
set -euo pipefail

T4DIR="$(cd "$(dirname "$0")/.." && pwd)"
TOP="$(cd "$T4DIR/../.." && pwd)"
TESTS="$T4DIR/tests"
TCC="$TOP/transputer-tcc"
NETLOAD="$TOP/transputer-netload"
T4_BIN="${T4_BIN:-$T4DIR/t4}"

for tool in "$TCC" "$NETLOAD" "$T4_BIN"; do
    [[ -x "$tool" ]] || { echo "missing $tool"; exit 1; }
done

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
expected=$'S\nALL-PASS\nH'
failures=0

build() {       # build NAME SOURCE...
    local name="$1"
    shift
    "$TCC" -static -nostdlib -I"$TOP/include/transputer" -I"$TOP/include" \
        -o "$tmp/$name.elf" "$TESTS/test_boot.S" "$@" >"$tmp/$name.build" 2>&1 &&
        "$NETLOAD" "$tmp/$name.elf" "$tmp/$name.btl" >>"$tmp/$name.build" 2>&1
}

emulate() {     # emulate NAME TIMEOUT [VAR=value...] -- [t4 option...]
    local name="$1" limit="$2"
    local -a vars=() opts=()
    shift 2
    while [[ $# -gt 0 && "$1" != "--" ]]; do vars+=("$1"); shift; done
    [[ $# -gt 0 ]] && shift
    opts=("$@")
    # -k: t4 exits on SIGTERM only between instructions.  `times` leaves
    # the host CPU time of the run in NAME.times, for cpu_below.
    ( cd "$tmp" && { env -u DISPLAY T4_UART_CONSOLE=0 T4_UART_LOG="$tmp/$name.uart" \
        "${vars[@]}" timeout -k 5 "$limit" "$T4_BIN" -s806 -sm 26 -ss \
        "${opts[@]}" -sb "$tmp/$name.btl" >"$tmp/$name.log" 2>&1 || true; } &&
        times >"$tmp/$name.times" ) || true
}

cpu_below() {   # cpu_below NAME SECONDS: the run took less host CPU time
    local cpu
    # the second line of `times` is the children's, "0m1.234s 0m0.056s"
    cpu="$(sed -n 2p "$tmp/$1.times" 2>/dev/null |
           awk '{ n = 0; for (i = 1; i <= 2; i++) { split($i, t, /[ms]/)
                  n += t[1] * 60 + t[2] }; printf "%.2f", n }')"
    if [[ -n "$cpu" ]] && awk -v c="$cpu" -v l="$2" 'BEGIN { exit !(c < l) }'; then
        echo "PASS $1: host CPU ${cpu}s, under ${2}s"
    else
        echo "FAIL $1: host CPU ${cpu:-?}s, limit ${2}s"
        failures=$((failures + 1))
    fi
}

verdict() {     # verdict NAME
    local got
    got="$(cat "$tmp/$1.uart" 2>/dev/null || true)"
    if [[ "$got" == "$expected" ]]; then
        echo "PASS $1"
    else
        echo "FAIL $1: uart=$(printf '%s' "$got" | tr '\n' '|')"
        tail -5 "$tmp/$1.log" | sed 's/^/    /'
        failures=$((failures + 1))
    fi
}

run_test() {    # run_test NAME "SOURCES" TIMEOUT [VAR=value...] -- [t4 option...]
    local name="$1" sources="$2"
    shift 2
    # shellcheck disable=SC2086 # SOURCES is a word list
    if ! build "$name" $sources; then
        echo "FAIL $name: build"
        sed 's/^/    /' "$tmp/$name.build"
        failures=$((failures + 1))
        return
    fi
    emulate "$name" "$@"
    verdict "$name"
}

# A stand-alone node halts once nothing is left to run.
run_test test_t9000_vcp \
    "$TESTS/test_t9000_vcp.c $TESTS/test_t9000_vcp_fn.S $TOP/lib/transputer-conc.c" \
    30s T4_LINK_LOOPBACK=1,2-3 T4_EVENT_LOOPBACK=1

# Node 1 keeps serving, so this run ends at the timeout.
printf 'Connect HOST to processor 0 link 0\nConnect processor 1 link 1 to processor 0 link 1\n' \
    >"$tmp/net.map"
run_test test_t9000_vcp_net \
    "$TESTS/test_t9000_vcp_net.c $TESTS/test_t9000_vcp_fn.S" \
    4s SPYNET="$tmp/net.map" -- -st -sn -2

# Routers 2 and 3 (see the picture in the test).  Path A: [node][vl] headers,
# the node byte deleted on the way out to the node, either cable between the
# routers.  Path B: network-wide 2-byte labels, #000-#0FF node 0 and
# #100-#1FF node 1, read on router 2's Link1 and Link4 only.
cat >"$tmp/c104.map" <<'EOF'
Connect HOST to processor 0 link 0
Connect processor 0 link 3 to processor 1 link 3
router 2
router 3 links 4
Connect processor 0 link 1 to router 2 link 0
Connect processor 0 link 2 to router 2 link 1
Connect router 2 link 4 to processor 1 link 2
Connect router 2 link 2 to router 3 link 2
Connect router 2 link 3 to router 3 link 3
Connect router 3 link 0 to processor 1 link 1
router 2 interval 0 1 0 on 0,2-3
router 2 interval 1 2 2-3 on 0,2-3
router 2 delete 0
router 3 interval 0 1 2-3
router 3 interval 1 2 0
router 3 delete 0
router 2 header 2 on 1,4
router 2 interval #000 #100 1 on 1,4
router 2 interval #100 #200 4 on 1,4
EOF
run_test test_t9000_c104_net \
    "$TESTS/test_t9000_c104_net.c $TESTS/test_t9000_vcp_fn.S $TOP/lib/transputer-conc.c" \
    5s SPYNET="$tmp/c104.map" -- -st -sn -2

# Node 1 is configured, booted, stopped and reset entirely over its control
# link: node 0's Link2 is cabled to node 1's CLink0 (link 4).  Node 1's
# control system keeps serving, so this run ends at the timeout.
printf '%s\n' 'Connect HOST to processor 0 link 0' \
    'Connect processor 0 link 1 to processor 1 link 1' \
    'Connect processor 0 link 2 to processor 1 clink 0' >"$tmp/ctl.map"
run_test test_t9000_ctl_net \
    "$TESTS/test_t9000_ctl_net.c $TESTS/test_t9000_vcp_fn.S" \
    6s SPYNET="$tmp/ctl.map" -- -st -sn -2

# The control network daisy-chained on: node 1's CLink1 to node 2's CLink0.
printf '%s\n' 'Connect HOST to processor 0 link 0' \
    'Connect processor 0 link 2 to processor 1 clink 0' \
    'Connect processor 1 clink 1 to processor 2 clink 0' >"$tmp/chain.map"
run_test test_t9000_ctl_chain \
    "$TESTS/test_t9000_ctl_chain.c $TESTS/test_t9000_vcp_fn.S" \
    6s SPYNET="$tmp/chain.map" -- -st -sn -3

# The supervisor restarts trapped call/ajw/gajw itself; the node halts.
run_test test_t9000_pstack "$TESTS/test_t9000_pstack.c" 20s
run_test test_t9000_pmode "$TESTS/test_t9000_pmode.c" 20s

# Not T9000: the instruction pairs t4 fuses must not change the stack.
run_test test_fused_cj "$TESTS/test_fused_cj.c $TESTS/test_fused_cj_fn.S" 10s
# Without the fix the channel ALT waits for ever: 10s is the verdict.
run_test test_alt_guard "$TESTS/test_alt_guard.c $TOP/lib/transputer-conc.c" 10s
# A timer ALT leaves the timer list whichever guard won, and is queued
# once.  Without the fixes t4 spins or hangs: 10s is the verdict.
run_test test_timer_alt "$TESTS/test_timer_alt.c $TOP/lib/transputer-conc.c" 10s

# Byte-stream links between two threaded nodes, Link1 and Link2 both cabled.
# Node 1 stops once it has echoed, so this run ends at the timeout.
printf '%s\n' 'Connect HOST to processor 0 link 0' \
    'Connect processor 0 link 1 to processor 1 link 1' \
    'Connect processor 0 link 2 to processor 1 link 2' >"$tmp/sync.map"
run_test test_link_sync \
    "$TESTS/test_link_sync.c $TOP/lib/transputer-conc.c" \
    5s SPYNET="$tmp/sync.map" -- -st -sn -2
# The timer thread serves threaded nodes on a path of its own.  Without
# the fix the first part spins for ever; node 1 is never booted, so the
# run ends at the timeout either way.
run_test test_thread_timeslice \
    "$TESTS/test_thread_timeslice.c $TOP/lib/transputer-conc.c" \
    5s SPYNET="$tmp/sync.map" -- -st -sn -2
# The timeslice period follows the part: -s9 after the -s806 every test
# gets makes a T9000.  One node and a threaded network take separate paths
# through the timer thread; the threaded runs end at the timeout.
ts_src="$TESTS/test_timeslice_period.c $TOP/lib/transputer-conc.c"
run_test test_timeslice_t9000 "$ts_src -DEXPECT_T9000=1" 10s -- -s9
run_test test_timeslice_t800 "$ts_src -DEXPECT_T9000=0" 10s
run_test test_timeslice_t9000_net "$ts_src -DEXPECT_T9000=1" \
    5s SPYNET="$tmp/sync.map" -- -s9 -st -sn -2
run_test test_timeslice_t800_net "$ts_src -DEXPECT_T9000=0" \
    5s SPYNET="$tmp/sync.map" -- -st -sn -2
# Node 0 ends idle with one link mailbox full and nobody waiting on it.
# Without the fix it spins until the timeout, a core's worth of CPU time;
# with it the network sleeps.
run_test test_idle_mailbox \
    "$TESTS/test_idle_mailbox.c $TOP/lib/transputer-conc.c" \
    4s SPYNET="$tmp/sync.map" -- -st -sn -2
cpu_below test_idle_mailbox 1.5
# About 1.7 s of timer waits.  Without the fix node 0 spins through all of
# them; with it the network sleeps.  Node 1 stops once it has sent its
# word and node 0 halts, so the run ends at the timeout.
run_test test_timer_sleep \
    "$TESTS/test_timer_sleep.c $TOP/lib/transputer-conc.c" \
    5s SPYNET="$tmp/sync.map" -- -st -sn -2
cpu_below test_timer_sleep 0.8
# Node 1 booted on Link1, node 2 on Link2.  Without the fix node 2 listens
# on Link1 only and node 0 waits for ever: the timeout is the verdict.
printf '%s\n' 'Connect HOST to processor 0 link 0' \
    'Connect processor 0 link 1 to processor 1 link 1' \
    'Connect processor 0 link 2 to processor 2 link 2' >"$tmp/boot3.map"
run_test test_boot_links "$TESTS/test_boot_links.c $TOP/lib/transputer-conc.c" \
    6s SPYNET="$tmp/boot3.map" -- -st -sn -3
# About 1.8 s of timer waits on one node: without the fix it spins through
# them.  The node halts after its last test, so the run ends early.
run_test test_timer_sleep_one \
    "$TESTS/test_timer_sleep_one.c $TOP/lib/transputer-conc.c" 5s
cpu_below test_timer_sleep_one 0.6

# Two processes with shared-memory links (-sl), as testall.sh starts them:
# node 1 first, which creates the segment (keyed by ftok(map, 1984)), then
# node 0 with the program.  Node 1 is stopped and the segment removed after.
shm_key() {     # shm_key FILE: ftok(FILE, 1984) as glibc computes it
    local dev ino
    eval "$(stat -c 'dev=%d ino=%i' "$1")"
    printf '0x%08x' $(( (192 << 24) | ((dev & 255) << 16) | (ino & 65535) ))
}
shm_drop() {    # shm_drop KEY
    local id
    id=$(ipcs -m | awk -v k="$1" '$1 == k { print $2 }')
    [[ -n "$id" ]] && ipcrm -m "$id" 2>/dev/null
    return 0
}
run_sl_test() { # run_sl_test NAME "SOURCES" TIMEOUT MAP
    local name="$1" sources="$2" limit="$3" map="$4" key n1 waited=0
    # shellcheck disable=SC2086 # SOURCES is a word list
    if ! build "$name" $sources; then
        echo "FAIL $name: build"
        sed 's/^/    /' "$tmp/$name.build"
        failures=$((failures + 1))
        return
    fi
    key=$(shm_key "$map")
    shm_drop "$key"
    # exec: $! is then timeout itself, which passes the kill below to t4
    ( cd "$tmp" && exec env -u DISPLAY SPYNET="$map" T4_UART_CONSOLE=0 \
        T4_UART_LOG="$tmp/$name.n1.uart" timeout -k 2 "$limit" "$T4_BIN" \
        -s806 -sm 26 -sl -sn 1 </dev/null >"$tmp/$name.n1.log" 2>&1 ) &
    n1=$!
    until ipcs -m | awk -v k="$key" '$1 == k { f = 1 } END { exit !f }' ||
          [[ $waited -ge 50 ]]; do
        sleep 0.1; waited=$((waited + 1))
    done
    emulate "$name" "$limit" SPYNET="$map" -- -sl -sn 0
    kill "$n1" 2>/dev/null || true
    wait "$n1" 2>/dev/null || true
    shm_drop "$key"
    verdict "$name"
}
# 1000 round trips between the two processes must take < 0.4 s: an idle
# -sl node polled in 1 ms slices (1.1 s); now the far process wakes it
# (25-60 ms).
printf '%s\n' 'Connect HOST to processor 0 link 0' \
    'Connect processor 0 link 1 to processor 1 link 1' >"$tmp/sl.map"
run_sl_test test_sl_wake "$TESTS/test_sl_wake.c $TOP/lib/transputer-conc.c" \
    4s "$tmp/sl.map"

exit $(( failures > 0 ))
