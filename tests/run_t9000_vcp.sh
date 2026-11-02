#!/usr/bin/env bash
# T9000 channel tests (see ../T9000_CHANNELS.md):
#
#   test_t9000_vcp      one node: Link1 cabled to itself carries the virtual
#                       links, Link2/Link3 crossed in byte-stream mode, event
#                       pins looped (T4_LINK_LOOPBACK, T4_EVENT_LOOPBACK)
#   test_t9000_vcp_net  two threaded nodes: node 0 boots node 1 down Link1,
#                       then both use it as a virtual link
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

exit $(( failures > 0 ))
