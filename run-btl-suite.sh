#!/bin/bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
T4_BIN="$ROOT_DIR/transputer/temu/t4"

if [ ! -x "$T4_BIN" ]; then
    echo "error: temu t4 binary not found: $T4_BIN" >&2
    exit 1
fi

BTLS=(
    "$ROOT_DIR/transputer/temu/examples/hello/hello.btl"
    "$ROOT_DIR/transputer/temu/examples/sqrroots.btl"
    "$ROOT_DIR/transputer/temu/examples/raytracer/raytraceT8.btl"
)

RUN_SECS="${RUN_SECS:-2}"
BOOTDBG="${BOOTDBG:-0}"

echo "=== TEMU BTL Suite ==="
echo "Run secs: $RUN_SECS"
echo "BOOTDBG:  $BOOTDBG"

for btl in "${BTLS[@]}"; do
    name="$(basename "$btl")"
    log="/tmp/temu-${name}.log"
    if [ ! -f "$btl" ]; then
        echo "MISSING: $btl"
        continue
    fi
    echo "--- $name ---"
    if [ "$BOOTDBG" = "1" ]; then
        T4_BOOTDBG=1 "$T4_BIN" -s8 -sm 24 -ss -sx 0 -sb "$btl" > "$log" 2>&1 &
    else
        "$T4_BIN" -s8 -sm 24 -ss -sx 0 -sb "$btl" > "$log" 2>&1 &
    fi
    pid=$!
    sleep "$RUN_SECS"
    kill "$pid" >/dev/null 2>&1 || true

    if rg -q "bad Icode" "$log"; then
        echo "FAIL: bad Icode detected (log: $log)"
    else
        echo "OK: no bad Icode detected (log: $log)"
    fi
done
