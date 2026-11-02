#!/usr/bin/env bash
# Phase 1/3 smoke: L goprot → P syscall → trap cause 3 stored at RESULT.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

python3 tests/mk_hello_p.py >/dev/null

export T4_PROTECT=1
export T4_PROTECT_DEBUG="${T4_PROTECT_DEBUG:-0}"
# RESULT @ 0x80000270 must become trap cause 3 (T9K_TRAP_SYSCALL)
# RESULT+4/+8 must hold the ISM resume pair: t.Scall (8), et.NoError (0)
export T4_WATCH=0x80000270,1,0x80000274,2,0x800001b0,2

LOG=$(mktemp)
# short run — guest spins after store; timeout dumps T4_WATCH via handler
set +e
env -u DISPLAY timeout 1 ./t4 -s806 -sm 24 -ss -se -sb tests/hello_p.btl \
  >"$LOG" 2>&1
set -e

if ! grep -q 'P-trap cause=3' "$LOG" && ! grep -q '\[WATCH\] 0x80000270: 00000003' "$LOG"; then
  # cause=3 may only appear with DEBUG; WATCH is authoritative
  if ! grep -q '\[WATCH\] 0x80000270: 00000003' "$LOG"; then
    echo "FAIL: expected RESULT word 0x00000003 (syscall trap)"
    tail -30 "$LOG"
    rm -f "$LOG"
    exit 1
  fi
fi

if ! grep -q '\[WATCH\] 0x80000270: 00000003' "$LOG"; then
  echo "FAIL: RESULT not written"
  tail -30 "$LOG"
  rm -f "$LOG"
  exit 1
fi

if ! grep -q '\[WATCH\] 0x80000274: 00000008 00000000' "$LOG"; then
  echo "FAIL: supervisor did not resume with A=t.Scall, B=et.NoError"
  grep WATCH "$LOG"
  rm -f "$LOG"
  exit 1
fi

echo "PASS: hello_p goprot→syscall→trap cause 3, RESULT=0x00000003, A=t.Scall B=et.NoError"
grep -E 'goprot|syscall|P-trap|WATCH|resume' "$LOG" || true
rm -f "$LOG"
