# T4 Emulator Rewrite Plan

## Current State

`p.c` is 8500+ lines containing:
- Instruction dispatch (208 case statements in one giant switch)
- FPU emulation (inline in the switch)
- Timer/scheduler simulation
- SDL display backend calls
- Network (TAP) polling
- Debug/trace infrastructure (~500 lines in the hot loop)
- Memory-mapped I/O (UART, VGA, SD, keyboard, mouse, network)
- Instruction cache simulation
- T4COMBINATIONS (multi-instruction optimization)
- Boot loading (BTL parser)
- Symbol table for debug

Everything is in one function (`mainloop`) with global state.

### Implementation status (2026-07-19)

| Phase | Status | Current state |
|------:|--------|---------------|
| 1. Core state | Partial | `emu.h` and related interfaces exist, but `p.c` still owns the live global CPU state and hot loop. |
| 2. MMIO | Prototype | `mmio.c`/`mmio.h` exist; most real device and iserver handling remains in `p.c`, and the Linux build does not yet compile `mmio.c`. |
| 3. Timer | Integrated | `timer_integration.c` supplies the C11 timer thread used by the current build. |
| 4. CPU dispatch | Scaffold only | `cpu.c`, `cpu_dispatch.h`, and decode-cache files exist, but the computed-goto loop is not wired into the release build. |
| 5. FPU | Not started | FPU execution remains in the legacy path. |
| 6. Debug | Partial | `debug_config.c` is built, but most tracing and symbol logic remains in `p.c`. |

The standalone smoke suite currently passes 2/2. Dhrystone is correct. The
2026-11-02 hot-path fixes recovered most of the regression against the shipped
reference binary (once ~13.5x on the July host, ~3x on the November one), but
the rebuild is still ~1.45x slower than the 2023 source (~1.8x slower than the
shipped binary). What remains is the hot loop's own structure - guest
registers live in the `cpu_t` and are reloaded after every store - which a
Phase 4 `cpu_run()` holding them in locals would address. See `T4_PERF.md` for the measurements and the
remaining causes.

## Goals

1. **Performance**: Host CPU cache-friendly hot loop
2. **Maintainability**: Separate concerns into files
3. **SIMD-ready**: Aligned memory for potential vectorization
4. **Thread-safe**: Separate timer/IO from execution
5. **Portable**: C11 with minimal platform dependencies

## Proposed File Structure

```
transputer/t4/
  main.c              — CLI parsing, startup (existing, minimal changes)
  emu.h               — Core emulator state struct, public API
  cpu.c               — Instruction fetch-decode-execute loop
  cpu_opr.c           — OPR instruction handlers (split from cpu.c)
  cpu_fpu.c           — FPU instruction handlers
  mem.c               — Memory subsystem (read/write, alignment, MMIO dispatch)
  mmio.c              — Memory-mapped I/O devices (UART, VGA ctrl, SD, input)
  timer.c             — Timer/scheduler (C11 thread or polled)
  display.c           — SDL display backend (existing display_backend_sdl.c)
  net.c               — TAP networking (existing t4net code)
  boot.c              — BTL/bootstrap loader
  debug.c             — Debug/trace infrastructure (env var checks, symbol tables)
  Makefile            — Build with per-file optimization flags
```

## Core Emulator State

```c
/* emu.h */
#include <stdint.h>
#include <stdatomic.h>
#include <stdalign.h>

/* Aligned for cache-line (64 bytes) and potential SIMD */
typedef struct alignas(64) {
    /* Hot path: touched every instruction */
    uint32_t IPtr;          /* Instruction pointer */
    uint32_t WPtr;          /* Workspace pointer */
    uint32_t AReg;          /* Eval stack top */
    uint32_t BReg;          /* Eval stack middle */
    uint32_t CReg;          /* Eval stack bottom */
    uint32_t OReg;          /* Operand register (prefix accumulator) */
    uint8_t  Icode;         /* Current instruction high nibble */
    uint8_t  Idata;         /* Current instruction low nibble */
    uint8_t  Error;         /* Error flag */
    uint8_t  HaltOnError;   /* Halt-on-error flag */
    uint32_t ProcPriority;  /* Current process priority (0=hi, 1=lo) */
} cpu_state_t;

typedef struct alignas(64) {
    /* FPU registers — separate cache line from integer */
    double   FAReg_db;
    double   FBreg_db;
    double   FCreg_db;
    float    FAReg_sn;
    float    FBreg_sn;
    float    FCreg_sn;
    uint8_t  FA_len;        /* FP_REAL32 or FP_REAL64 */
    uint8_t  FB_len;
    uint8_t  FC_len;
    uint8_t  rounding;      /* Current rounding mode */
} fpu_state_t;

typedef struct alignas(64) {
    /* Timer state — updated by timer thread or polled */
    _Atomic uint32_t ClockReg[2];  /* [0]=hi-pri (1μs), [1]=lo-pri (64μs) */
    _Atomic int      timeslice;
    _Atomic int      quit;
    int              Txxx;         /* Processor type: 414, 800, 806 */
    uint32_t         ProductIdentity;
} timer_state_t;

typedef struct {
    /* Memory — page-aligned for mmap/SIMD potential */
    uint8_t  *mem;          /* Main memory array */
    uint32_t  mem_size;     /* Size in bytes */
    uint32_t  mem_mask;     /* Byte address mask */
    uint32_t  MemStart;     /* First usable address */
} mem_state_t;

typedef struct {
    cpu_state_t   cpu;
    fpu_state_t   fpu;
    timer_state_t timer;
    mem_state_t   memory;
} emu_t;
```

## CPU Hot Loop (cpu.c)

```c
/* Computed goto dispatch — GCC/Clang extension */
void cpu_run(emu_t *emu) {
    static void *dispatch[16] = {
        &&op_j, &&op_ldlp, &&op_pfix, &&op_ldnl,
        &&op_ldc, &&op_ldnlp, &&op_nfix, &&op_ldl,
        &&op_adc, &&op_call, &&op_cj, &&op_ajw,
        &&op_eqc, &&op_stl, &&op_stnl, &&op_opr
    };

    cpu_state_t *cpu = &emu->cpu;
    uint8_t *mem = emu->memory.mem;
    uint32_t mask = emu->memory.mem_mask;

    #define FETCH() do { \
        uint8_t insn = mem[cpu->IPtr & mask]; \
        cpu->OReg = cpu->OReg + (insn & 0xF); \
        goto *dispatch[insn >> 4]; \
    } while(0)

    FETCH();

op_pfix:
    cpu->OReg <<= 4;
    cpu->IPtr++;
    FETCH();

op_nfix:
    cpu->OReg = (~cpu->OReg) << 4;
    cpu->IPtr++;
    FETCH();

op_ldl:
    cpu->CReg = cpu->BReg;
    cpu->BReg = cpu->AReg;
    cpu->AReg = *(uint32_t*)(mem + ((cpu->WPtr + cpu->OReg * 4) & mask));
    cpu->OReg = 0;
    cpu->IPtr++;
    FETCH();

op_stl:
    *(uint32_t*)(mem + ((cpu->WPtr + cpu->OReg * 4) & mask)) = cpu->AReg;
    cpu->AReg = cpu->BReg;
    cpu->BReg = cpu->CReg;
    cpu->OReg = 0;
    cpu->IPtr++;
    FETCH();

op_ldc:
    cpu->CReg = cpu->BReg;
    cpu->BReg = cpu->AReg;
    cpu->AReg = cpu->OReg;
    cpu->OReg = 0;
    cpu->IPtr++;
    FETCH();

op_adc:
    cpu->AReg += cpu->OReg;
    cpu->OReg = 0;
    cpu->IPtr++;
    FETCH();

op_sum: /* inside op_opr */
    cpu->AReg = cpu->BReg + cpu->AReg;
    cpu->BReg = cpu->CReg;
    cpu->OReg = 0;
    cpu->IPtr++;
    FETCH();

/* ... etc for all 16 primary opcodes */

op_opr:
    cpu_exec_opr(emu);  /* Dispatch to cpu_opr.c */
    FETCH();

op_j:
    cpu->IPtr = cpu->IPtr + 1 + cpu->OReg;
    cpu->OReg = 0;
    /* Timer/scheduler check on every jump */
    if (atomic_load_explicit(&emu->timer.timeslice, memory_order_relaxed) > 0)
        cpu_check_timeslice(emu);
    FETCH();

/* ... */
}
```

## Mini Threading (No pthreads)

```c
/* thread.h — Minimal C11 threading wrapper */
#include <threads.h>  /* C11 threads */

typedef struct {
    thrd_t handle;
    _Atomic int running;
} mini_thread_t;

static inline int mini_thread_create(mini_thread_t *t, int (*fn)(void*), void *arg) {
    atomic_store(&t->running, 1);
    return thrd_create(&t->handle, fn, arg);
}

static inline void mini_thread_stop(mini_thread_t *t) {
    atomic_store(&t->running, 0);
    thrd_join(t->handle, NULL);
}
```

## Timer Thread

```c
/* timer.c */
int timer_thread_fn(void *arg) {
    emu_t *emu = arg;
    struct timespec last, now;
    timespec_get(&last, TIME_UTC);

    while (atomic_load(&emu->timer.quit) == 0) {
        thrd_sleep(&(struct timespec){.tv_nsec = 64000}, NULL); /* 64μs */
        timespec_get(&now, TIME_UTC);

        long elapsed_us = (now.tv_sec - last.tv_sec) * 1000000 +
                          (now.tv_nsec - last.tv_nsec) / 1000;
        last = now;

        /* High-priority timer: 1μs resolution */
        atomic_fetch_add(&emu->timer.ClockReg[0], elapsed_us);

        /* Low-priority timer: 64μs resolution */
        static int accum = 0;
        accum += elapsed_us;
        if (accum >= 64) {
            atomic_fetch_add(&emu->timer.ClockReg[1], accum / 64);
            accum &= 63;
        }

        /* Timeslice counter */
        atomic_fetch_add(&emu->timer.timeslice, 1);
    }
    return 0;
}
```

## Memory Subsystem (mem.c)

```c
/* Aligned allocation for SIMD potential */
void *mem_alloc(size_t size) {
    void *p = aligned_alloc(4096, (size + 4095) & ~4095);
    memset(p, 0, size);
    return p;
}

/* Fast byte read — no MMIO check for code fetch */
static inline uint8_t mem_read_byte_fast(mem_state_t *m, uint32_t addr) {
    return m->mem[addr & m->mem_mask];
}

/* Full byte read with MMIO dispatch */
uint8_t mem_read_byte(emu_t *emu, uint32_t addr) {
    if (__builtin_expect(addr >= 0x10000000 && addr < 0xB0000000, 0))
        return mmio_read_byte(emu, addr);
    return emu->memory.mem[addr & emu->memory.mem_mask];
}

/* Full word read with alignment check */
uint32_t mem_read_word(emu_t *emu, uint32_t addr) {
    if (__builtin_expect(addr >= 0x10000000 && addr < 0xB0000000, 0))
        return mmio_read_word(emu, addr);
    uint32_t idx = addr & emu->memory.mem_mask;
    return *(uint32_t*)(emu->memory.mem + idx);
}
```

## Build System

```makefile
# Per-file optimization flags
CFLAGS_cpu = -O2 -march=native -fno-unroll-loops -falign-functions=64
CFLAGS_mem = -O3 -march=native
CFLAGS_timer = -O2
CFLAGS_display = -O2 $(SDL_CFLAGS)
CFLAGS_debug = -O0 -g

cpu.o: cpu.c emu.h
	$(CC) $(CFLAGS_cpu) -c -o $@ $<
```

Key: `cpu.c` gets `-O2 -fno-unroll-loops` to keep the dispatch table compact
for icache. `-O3` would over-unroll the switch cases.

## Migration Path

1. **Phase 1**: Extract `emu.h` with state structs. Keep p.c but reference
   the new structs. Verify boot still works.

2. **Phase 2**: Extract MMIO handlers to `mmio.c`. The UART/VGA/SD/input
   read/write functions are self-contained.

3. **Phase 3**: Extract timer to `timer.c` with C11 thread. Main loop
   reads `ClockReg` atomically instead of calling `update_time()`.

4. **Phase 4**: Rewrite dispatch loop in `cpu.c` with computed goto.
   This is the performance-critical change.

5. **Phase 5**: Extract FPU to `cpu_fpu.c`. The FPU handlers are
   independent of the integer dispatch.

6. **Phase 6**: Extract debug to `debug.c`. All env var checks, symbol
   tables, trace output.

Each phase must keep `make -f Makefile.linux smoke` passing. Performance phases
must also be checked with `make -f Makefile.linux benchmark` against the shipped
reference binary. Use `make -f Makefile.linux coremark` as the long-running
hot-loop profiling workload; its netload image is not accepted by the shipped
reference binary.

## Performance Targets

| Metric | Current | Target |
|--------|---------|--------|
| Boot to self-test | ~300s | ~200s |
| CoreMark (10 iter) | TBD | TBD |
| Binary size | 162KB | <100KB |
| Hot loop icache | ~32KB? | <8KB |
| Timer overhead | ~20% of cycles | ~0% (threaded) |
