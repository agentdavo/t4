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
    # -k: t4 exits on SIGTERM only between instructions
    ( cd "$tmp" && env -u DISPLAY T4_UART_CONSOLE=0 T4_UART_LOG="$tmp/$name.uart" \
        "${vars[@]}" timeout -k 5 "$limit" "$T4_BIN" -s806 -sm 26 -ss \
        "${opts[@]}" -sb "$tmp/$name.btl" >"$tmp/$name.log" 2>&1 ) || true
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

exit $(( failures > 0 ))
