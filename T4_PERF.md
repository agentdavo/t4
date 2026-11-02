# T4 Emulator Performance Notes

Owned by the project’s ongoing T4 work. Goal: keep **one** instruction hot loop
fast and cache-resident; push host I/O off the critical path.

## Design rules

1. **Single executor** for guest state (IPtr/WPtr/A/B/C/mem). Never thread the ISA.
2. **Background threads OK** for: wall-clock timer (done), future SDL present, TAP, UART log.
3. **Hot path must not** call `printf`, SDL present, or `gettimeofday` every instruction.
4. **Icache correctness**: any shortcut that jumps to `FastDispatch`/`ExecuteInstr`
   must fill `Icache[islot].{NextIPtr,OReg,Icode}` first, or the next hit loads garbage.
5. **Every per-instruction check is paid ~10^8 times a second.** A new optional
   hook goes in `run_insn_hooks()` (reached through one compare against
   `insn_hook_next`) or in the timer block at the loop top (every 64
   instructions), not inline in `mainloop()`. Anything optional that must stay
   inline - a trace inside an instruction case, a debug breakpoint - starts
   its test with `rare_hooks &&`: a loop-invariant local, set once before the
   loop whenever diagnostics, a stop breakpoint, the opt-in fast paths, the
   decode cache, the T9000 grouper or the instruction-count hooks are in use.
   Never read `volatile` or `_Atomic` state per instruction:
   `heartbeat_counter` was `volatile` and cost ~13% on its own.
6. **`t4_sig_pending` stays a per-instruction check.** `handler(-1)` is also the
   emulator's own stop (bad Icode, unimplemented instruction, end of run) and
   must take effect at the next instruction. Polling it every 64 instructions
   let a stopped program run on (`test_hfa_args` printed past its end).
7. **GCC will not inline into `mainloop()`** - it is far past the inliner's
   growth limits. A helper on every instruction's path needs `HOT_INLINE`
   (`IHASH`, `IsCached`, the `*_fast` memory accessors).
8. **Memory accessors test `t9k_prot.active`, not `t9k_prot.enabled`.**
   `T4_PROTECT` defaults on, but translation can only change an address in
   P-mode or with a live region descriptor. Anything that changes `enabled`,
   `is_p` or `region[]` must call `t9k_update_active()` (`t9000_protect.c`).
9. **`mainloop()` locals and `setjmp()`.** A T9000 P-trap `longjmp`s back into
   `mainloop()`, so any local it modifies after the `setjmp()` is indeterminate
   there. Set such a local before the `setjmp()` and never change it
   (`rare_hooks`, `mainloop_icache`), or reset it in the `setjmp() != 0`
   branch (`timer_tick`).

## Environment variables

| Variable | Default | Meaning |
|----------|---------|---------|
| `T4_HEADLESS=1` | on via `build.sh --boot` | No SDL window; UART only |
| `T4_PRESENT_ON_FLIP=1` | on | Present texture only on VGA FLIP |
| `T4_VSYNC=1` | off | SDL present vsync (slower, smoother) |
| `T4_FB_FPS=N` | 15 | Cap when not flip-only |
| `T4_DCACHE=1` | **off** | Prefix decode cache (experimental) |
| `T4_CPU_PIN=N` | unset | `sched_setaffinity` hot loop to CPU N |
| `T4_FAST_PRIMARY=1` | **off** | Single-insn primary fast path. Opt-in, and **broken**: Dhrystone takes a wrong branch on its second run (2026-11-02) |
| `T4_FAST_BLOCK=1` | **off** | Multi-insn primary block cache. Opt-in, tested together with the above |
| `T4_IPTR_SAMPLE=N` | off | Record IPtr every N instructions (via `run_insn_hooks()`) |
| `T4_FB_RECORD=N` | off | Headless framebuffer capture every N instructions (same) |

## What is already in place

- Timer thread (`timer_integration.c`) updates `ClockReg` / timeslice; with it
  running, `mainloop()` polls the timer queues every 64 instructions.
- Inline RAM fast paths (`word_fast`/`writeword_fast`/`byte_fast`/
  `writebyte_fast`) for external memory **and** the on-chip core, in front of
  the out-of-line `*_int` accessors that handle MMIO, translation, the
  framebuffer windows and `T4_WATCH_ADDR`.
- Core<->external-RAM `move` is one `memcpy`, not a per-byte accessor loop.
- Optional per-instruction machinery and every trace inside the instruction
  cases sit behind `rare_hooks` (rule 5). The debug prints that used to run
  unconditionally - `[LDNL]`, `[GCALL]` and the bootstrap-range lines,
  `[VGA_STNL]`, `[SETUP_STNL]`, `[FB_STNL]` - now appear only when a T4
  diagnostic is enabled (any variable `runtime_diagnostics_requested()` checks,
  e.g. `T4_NOISY_BOOT_TRACE=1`).
- The `Icache`/`Acache` pointers are read once into `mainloop()` locals, and
  the wild-IPtr guard (`[WILDEXEC]`) runs on an Icache miss rather than on every
  instruction: a hit is an address that already passed it.
- Headless display + present-on-FLIP (less host churn during kernel boot).
- Optional decode cache behind `T4_DCACHE=1` (fills Icache correctly now).

## Recommended next steps (priority)

1. Close the remaining ~1.45x Dhrystone gap to the 2023 source (~1.8x to the
   shipped 2023 binary; see below). What is left is the interpreter's own
   machinery, spread thin. In the last `gprof -l` run, largest first:
   - `switch (Icode)` and `switch (OReg)` dispatch;
   - the RAM fast path (~14%): `ram_fast_base`'s translation/watchpoint and
     framebuffer-window tests, and the `IsCached()` test on every store;
   - the per-instruction core: Icache tag compare and slot loads, the error
     check after each instruction, `ReadGotoSNP`, `ResetRounding`,
     `heartbeat_counter`, and the timer tick against `delayCount1`.
   IPtr, AReg and the rest live in the `cpu_t` and are reloaded after every
   guest store, since a store through `u_char *` may alias them. Keeping them
   in locals is the structural fix, which is what item 2 amounts to.
2. Extract primary ops into a small `cpu_run()` with **computed goto** (`cpu_dispatch.h`).
3. Grow fast-block with common **OPR** ops that do not touch MMIO (`sum`, `diff`, `rev`, `gt`, …) -
   after fixing the `T4_FAST_*` correctness bug above.
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

`run-btl-suite.sh` matches output with `rg` (ripgrep). Without it every test
reports "expected output missing", whatever the emulator did - install it
first (`sudo apt install ripgrep`).

Set `T4_BIN=bin/t4lnx64` to run the same suite against the shipped reference
binary. Logs are kept under `/tmp/t4-smoke.*` on failure; set `KEEP_LOGS=1`
to retain successful logs as well.

### Measured 2026-11-02

Intel Core Ultra 7 265, WSL2 Ubuntu, gcc 15.2, each tree built with its own
`Makefile.linux` flags, pinned with `taskset -c 4,5`. Run-to-run noise is
about ±10% between batches, so compare builds measured in the same batch.

| Build | Dhrystone (median) |
|-------|-------------------:|
| shipped `bin/t4lnx64` (2023) | 230-260K/s |
| 2023 source (`6b60aac`) rebuilt | ~225K/s |
| `2f1bade` (2023-07) | ~203K/s |
| `1924cb4` (2026-08) | ~96K/s |
| HEAD `a31a544` | ~109K/s |
| working tree before the fixes below | 73-82K/s |
| working tree after them | 114-132K/s |

The loss is commit `1f17a7b` ("add linux, vga, uart, sdl, kb, mouse, sd"),
which does not link on its own. Under `gprof -l`, `mainloop()`'s own time on a
Dhrystone run went from 0.45 s to 1.46 s, and the code before `switch (Icode)`
from 0.19 s to 0.78 s. (The July baseline said 13.5x on a different host; the
gap on this one was ~3x.)

Fixed on 2026-11-02, cumulative Dhrystone gain in brackets:

1. **T9000 translation on every access** (+37% with item 2's fast paths).
   `T4_PROTECT` defaults on, so every load and store made two out-of-line calls
   (`t9k_translate`, `relocate_only`) - 240M per Dhrystone run - that returned
   the address unchanged. Now gated on `t9k_prot.active`.
2. **On-chip RAM through the MMIO chain.** Addresses below `ExtMemStart`
   (`0x80001000` on a T800) missed the fast path and walked 20-70 device
   compares per access; the same program ran 5x slower with its data there.
3. **Timer queues polled every 6 instructions** (+40% with items 1-2). The
   starting `delayCount1` was never adapted once the timer thread took over.
   Also: an out-of-line `t9k_check_priv_op()` on every OPR, and byte-wise
   core<->external `move`.
4. **Per-instruction overhead** (+60-75%). `volatile` heartbeat, sampler and
   recorder checks, fast-path probes with `__builtin_expect(..., 1)` while
   off by default, and helpers GCC no longer inlined.

Not a cause: register access through the thread-local `t4cpu` pointer.
Caching it in a local changed nothing.

Other workloads, same before/after builds:

| Workload | Before | After |
|----------|-------:|------:|
| C loop benchmark, workspace and data in external RAM (guest `ldtimer` ticks) | 29.9-33.7K | 20.1-22.9K |
| Linux boot to "TransputerOS 0.1.0" (`vmlinux.btl` of 2026-11-01) | 3.7-4.1 s | 3.4-3.7 s |
| Linux boot to fbwm `DESKTOP_READY` | 10.4-10.8 s | 9.95-10.2 s |

The boot gains only ~5%: its time is spread over image load, UART and
framebuffer MMIO, and host I/O rather than hot guest loops.

### Second round, 2026-11-02 evening

On a busy host (load average 4-12), so measured as **user CPU seconds** for a
fixed amount of guest work, median of 5 interleaved runs (lower is better):

| Build | Dhrystone, 500,000 passes | C loop benchmark, external RAM |
|-------|--------------------------:|-------------------------------:|
| shipped `bin/t4lnx64` (2023) | 0.99 s | - (cannot load the image) |
| 2023 source rebuilt | 1.23 s | - (same) |
| working tree this morning | 3.20 s | 2.23 s |
| after the first round | 2.05 s | 1.64 s |
| after the second round | **1.78 s** | **1.30 s** |

Second-round changes (`p.c` only):

- Optional per-instruction checks (stop breakpoint, fast-path and decode-cache
  probes, cache trace, T9000 grouper, instruction-count hooks) and every trace
  test inside the instruction cases moved behind `rare_hooks` (rule 5). An
  upper-bound probe that deleted those checks outright gained 7-17%.
- The shared-events check moved into the timer block, as it was in 2023; the
  wild-IPtr guard moved to the Icache miss path.
- `Icache`/`Acache` read once into locals, and the timer cadence counted in a
  local `timer_tick` instead of `t4cpu->count1`.

Checked: backend UART suite identical before and after (192 pass, same 7
failures), BTL smoke and benchmark 7/7, P-mode trap smoke test and the
protection unit test pass, `[WILDEXEC]` still halts a jump below MemStart,
`T4_NOISY_BOOT_TRACE=1` prints the same 1,067 trace lines as before, and
SIGTERM still stops a run at once.

### How to measure

- WSL2 exposes no hardware counters (`perf_event_open` fails with ENOENT), so
  profile with `gprof -l`: rebuild with `-O3 -g -pg`, dropping `-flto` and
  `-fomit-frame-pointer`. Inlined code is reported under the callee's name.
- A short guest program's wall time is quantised to ~100 ms by exit latency.
  Time inside the guest with `__ldtimer()` instead.
- On a loaded host, wall time and the guest's own Dhrystone figure swing by
  25% or more. Compare user CPU time for a fixed amount of guest work
  (`TIMEFORMAT=%U; time t4 ...`), interleaving the builds run by run.

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
