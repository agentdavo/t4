#!/usr/bin/env bash

set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T4_BIN="${T4_BIN:-$ROOT_DIR/t4}"
SUITE="${SUITE:-smoke}"
KEEP_LOGS="${KEEP_LOGS:-0}"

case "$SUITE" in
    smoke)
        TEST_TIMEOUT="${TEST_TIMEOUT:-10}"
        ;;
    benchmark)
        TEST_TIMEOUT="${TEST_TIMEOUT:-120}"
        ;;
    coremark)
        TEST_TIMEOUT="${TEST_TIMEOUT:-600}"
        ;;
    *)
        echo "error: unknown suite '$SUITE' (expected smoke, benchmark, or coremark)" >&2
        exit 2
        ;;
esac

if [[ ! -x "$T4_BIN" ]]; then
    echo "error: t4 binary not found or not executable: $T4_BIN" >&2
    exit 2
fi

RUN_DIR="$(mktemp -d "${TMPDIR:-/tmp}/t4-smoke.XXXXXX")"
failures=0
passes=0

cleanup()
{
    if [[ "$KEEP_LOGS" == "1" || "$failures" -ne 0 ]]; then
        echo "Logs kept in: $RUN_DIR"
    else
        rm -rf -- "$RUN_DIR"
    fi
}
trap cleanup EXIT

# name|image|expected output
SMOKE_TESTS=(
    "hello|examples/hello/hello.btl|Hello world..."
    "savage|examples/savage/savage.b8h|Savage benchmark..."
)
BENCHMARK_TESTS=(
    "dhrystone|examples/dhrystone/dhrystone.b8h|dhrystones/second"
    "whetstone-single|examples/dhrystone/whetstoner.b8h|Single Precision Whetstones"
    "whetstone-double|examples/dhrystone/whetstoned.b8h|Double Precision Whetstones"
)
COREMARK_TESTS=(
    "coremark|examples/coremark.btl|Correct operation validated."
)

case "$SUITE" in
    smoke)
        TESTS=("${SMOKE_TESTS[@]}")
        ;;
    benchmark)
        TESTS=("${SMOKE_TESTS[@]}" "${BENCHMARK_TESTS[@]}")
        ;;
    coremark)
        TESTS=("${COREMARK_TESTS[@]}")
        ;;
esac

echo "T4 $SUITE suite"
echo "Binary:  $T4_BIN"
echo "Timeout: ${TEST_TIMEOUT}s per test"

for spec in "${TESTS[@]}"; do
    IFS='|' read -r name relative_image expected <<< "$spec"
    image="$ROOT_DIR/$relative_image"
    log="$RUN_DIR/$name.log"

    if [[ ! -f "$image" ]]; then
        echo "FAIL  $name (missing image: $relative_image)"
        failures=$((failures + 1))
        continue
    fi

    args=(-s8 -sm 24 -ss)
    args+=(-sb "$image")

    start_ns="$(date +%s%N)"
    (
        cd "$RUN_DIR" || exit 125
        timeout -k 5 "$TEST_TIMEOUT" env \
            T4_HEADLESS=1 \
            T4_PRESENT_ON_FLIP=1 \
            "$T4_BIN" "${args[@]}"
    ) >"$log" 2>&1
    status=$?
    end_ns="$(date +%s%N)"
    elapsed_ms=$(((end_ns - start_ns) / 1000000))

    if rg -a -q "bad Icode|Invalid WPtr|WILDJUMP|Segmentation fault" "$log"; then
        echo "FAIL  $name (${elapsed_ms}ms, emulator error)"
        failures=$((failures + 1))
    elif ! rg -a -Fq "$expected" "$log"; then
        if [[ "$status" -eq 124 ]]; then
            echo "FAIL  $name (${elapsed_ms}ms, timed out before expected output)"
        else
            echo "FAIL  $name (${elapsed_ms}ms, exit $status, expected output missing)"
        fi
        failures=$((failures + 1))
    elif [[ "$status" -ne 0 ]]; then
        echo "FAIL  $name (${elapsed_ms}ms, output matched but exit was $status)"
        failures=$((failures + 1))
    else
        metric="$(rg -a -m1 -F "$expected" "$log" || true)"
        echo "PASS  $name (${elapsed_ms}ms)${metric:+ — $metric}"
        passes=$((passes + 1))
    fi
done

echo "Summary: $passes passed, $failures failed"
if [[ "$failures" -ne 0 ]]; then
    exit 1
fi
