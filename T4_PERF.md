# T4 Emulator Performance Notes

Owned by the project’s ongoing T4 work. Goal: keep **one** instruction hot loop
fast and cache-resident; push host I/O off the critical path.

## Design rules

1. **Single executor** for guest state (IPtr/WPtr/A/B/C/mem). Never thread the ISA.
2. **Background threads OK** for: wall-clock timer (done), future SDL present, TAP, UART log.
3. **Hot path must not** call `printf`, SDL present, or `gettimeofday` every instruction.
4. **Icache correctness**: any shortcut that jumps to `FastDispatch`/`ExecuteInstr`
   must fill `Icache[islot].{NextIPtr,OReg,Icode}` first, or the next hit loads garbage.

## Environment variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `T4_HEADLESS=1` | on via `build.sh --boot` | No SDL window; UART only |
| `T4_PRESENT_ON_FLIP=1` | on | Present texture only on VGA FLIP |
| `T4_VSYNC=1` | off | SDL present vsync (slower, smoother) |
| `T4_FB_FPS=N` | 15 | Cap when not flip-only |
| `T4_DCACHE=1` | **off** | Prefix decode cache (experimental) |
| `T4_CPU_PIN=N` | unset | `sched_setaffinity` hot loop to CPU N |
| `T4_FAST_PRIMARY` | on (release) | Single-insn primary fast path |
| `T4_FAST_BLOCK` | on (release) | Multi-insn primary block cache |

## What is already in place

- Timer thread (`timer_integration.c`) updates `ClockReg` / timeslice.
- Fast primary + fast block caches for LDL/STL/LDC/… straight-line code.
- Headless display + present-on-FLIP (less host churn during kernel boot).
- Optional decode cache behind `T4_DCACHE=1` (fills Icache correctly now).

## Recommended next steps (priority)

1. Recover the measured Dhrystone regression before adding another dispatch path.
2. Extract primary ops into a small `cpu_run()` with **computed goto** (`cpu_dispatch.h`).
3. Grow fast-block with common **OPR** ops that do not touch MMIO (`sum`, `diff`, `rev`, `gt`, …).
4. Present worker thread: hot loop only sets dirty; worker does SDL upload.
5. PGO: instrument a full boot, rebuild with `-fprofile-use`.
6. Optional lightweight JIT for hot traces (long term).

## Reproducible baseline

The standalone repository has two test tiers:

```bash
make -f Makefile.linux smoke
make -f Makefile.linux benchmark
make -f Makefile.linux coremark
```

Set `T4_BIN=bin/t4lnx64` to run the same suite against the shipped reference
binary. Logs are kept under `/tmp/t4-smoke.*` on failure; set `KEEP_LOGS=1`
to retain successful logs as well.

Baseline measured on 2026-07-19 (same host, release builds):

| Binary | Dhrystone passes | Guest time | Throughput |
|--------|------------------:|-----------:|-----------:|
| shipped `bin/t4lnx64` | 500,000 | 5.1s | 47.1K/s |
| current rebuild | 500,000 | 63.9s | 3.5K/s |

The current rebuild is therefore about 13.5x slower on this workload. Polled
and threaded timer runs both measured about 64s, ruling out the timer thread as
the primary cause. The active investigation target is the enlarged legacy
`p.c` hot path (especially OPR-heavy workloads).

The isolated current-build Dhrystone run produced the number above. A complete
benchmark-tier run immediately after the smoke cases passed 2/5: `hello` and
`savage` passed, while Dhrystone and both Whetstone variants exceeded the
120-second per-test limit without an emulator error signature. The difference
from the isolated Dhrystone result is reproducible evidence of either run-order
or throughput variability and should be resolved as part of the regression.

### CoreMark image

`examples/coremark.btl` is a zero-bootstrap, chunked netload image. The current
loader decodes its 71,008-byte payload and starts it correctly; the shipped
`bin/t4lnx64` predates zero-bootstrap support and rejects the image. The first
current-build trial entered the guest and ran for 180 seconds without a crash,
but did not reach the CoreMark validation/result output before timeout.

Use the isolated target for profiling and before/after measurements:

```bash
make -f Makefile.linux coremark
TEST_TIMEOUT=1200 make -f Makefile.linux coremark
```

The default CoreMark ceiling is 600 seconds. A run only passes after printing
`Correct operation validated.`; the runner also reports wall-clock time and the
guest's CoreMark output line.

## Quick rebuild / test

```bash
make -f Makefile.linux
make -f Makefile.linux smoke
```
