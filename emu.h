/*
 * emu.h — Core emulator state for T4/T800/T806 transputer emulator
 *
 * All emulator state is collected into aligned structs for:
 * - Cache-line locality (hot CPU state in one 64-byte line)
 * - Thread safety (_Atomic for timer/scheduler state)
 * - SIMD-ready memory alignment
 */
#ifndef EMU_H
#define EMU_H

#include <stdint.h>
#include <string.h>

/* C11 atomics — fall back to volatile if not available */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && !defined(__STDC_NO_ATOMICS__)
#include <stdatomic.h>
#define EMU_ATOMIC _Atomic
#else
#define EMU_ATOMIC volatile
#define atomic_load_explicit(p, o) (*(p))
#define atomic_store_explicit(p, v, o) (*(p) = (v))
#define atomic_fetch_add_explicit(p, v, o) (*(p) += (v), *(p) - (v))
#define memory_order_relaxed 0
#define memory_order_acquire 0
#define memory_order_release 0
#endif

/* Alignment — use GCC attribute (works on both struct and variable) */
#ifdef __GNUC__
#define EMU_ALIGN(n) __attribute__((aligned(n)))
#else
#define EMU_ALIGN(n)
#endif

/* Cache line size (typical x86/ARM) */
#define EMU_CACHE_LINE 64

/* FPU precision tags */
#define FP_REAL32  1
#define FP_REAL64  2
#define FP_UNKNOWN 0

/* Processor types */
#define EMU_T414  414
#define EMU_T800  800
#define EMU_T806  806

/* MMIO address ranges */
#define MMIO_UART0_BASE     0x10000000u
#define MMIO_UART1_BASE     0x11000000u
#define MMIO_UART2_BASE     0x12000000u
#define MMIO_SD_BASE        0x20000000u
#define MMIO_NET_BASE       0x30000000u
#define MMIO_FB_BASE        0x81E00000u  /* RAM carveout; was 0x90000000 */
#define MMIO_FB_LEGACY_BASE 0x90000000u
#define MMIO_VGA_CTRL_BASE  0xA0000000u

/* -------------------------------------------------------------------
 * CPU integer state — hot path, touched every instruction.
 * Fits in one cache line (40 bytes used of 64).
 * ------------------------------------------------------------------- */
typedef struct {
    uint32_t IPtr;          /* Instruction pointer */
    uint32_t WPtr;          /* Workspace pointer */
    uint32_t AReg;          /* Eval stack top */
    uint32_t BReg;          /* Eval stack middle */
    uint32_t CReg;          /* Eval stack bottom */
    uint32_t OReg;          /* Operand register (prefix accumulator) */
    uint32_t EReg;          /* Error flag register */
    uint32_t ProcPriority;  /* Current process priority (0=hi, 1=lo) */
    uint8_t  HaltOnError;   /* Halt-on-error mode */
    uint8_t  Idle;          /* Process is idle (waiting) */
    uint8_t  _pad[6];       /* Pad to alignment */
} emu_cpu_t;

/* -------------------------------------------------------------------
 * FPU state — separate cache line from integer registers.
 * T800 has 3-deep FPU stack independent of integer stack.
 * ------------------------------------------------------------------- */
typedef struct {
    double   fa_db;         /* FAreg as REAL64 */
    double   fb_db;         /* FBreg as REAL64 */
    double   fc_db;         /* FCreg as REAL64 */
    float    fa_sn;         /* FAreg as REAL32 */
    float    fb_sn;         /* FBreg as REAL32 */
    float    fc_sn;         /* FCreg as REAL32 */
    uint8_t  fa_len;        /* FP_REAL32, FP_REAL64, or FP_UNKNOWN */
    uint8_t  fb_len;
    uint8_t  fc_len;
    uint8_t  rounding;      /* Current rounding mode */
    uint8_t  fpu_error;     /* FPU error flag */
    uint8_t  _pad[3];
} emu_fpu_t;

/* -------------------------------------------------------------------
 * Timer and scheduler state — updated by timer thread or polled.
 * Uses atomics for thread-safe access from timer thread.
 * ------------------------------------------------------------------- */
typedef struct {
    EMU_ATOMIC uint32_t ClockReg[2]; /* [0]=hi-pri 1μs, [1]=lo-pri 64μs */
    EMU_ATOMIC int32_t  timeslice;   /* Timeslice counter */
    EMU_ATOMIC int32_t  quit;        /* Quit request flag */
    int      Txxx;                   /* Processor type: 414, 800, 806 */
    uint32_t ProductIdentity;        /* LDDEVID return value */
    int      TimerEnableHi;
    int      TimerEnableLo;
} emu_timer_t;

/* -------------------------------------------------------------------
 * Memory subsystem — page-aligned for potential mmap/SIMD.
 * ------------------------------------------------------------------- */
typedef struct {
    uint8_t  *mem;          /* Main memory array (page-aligned) */
    uint32_t  size;         /* Size in bytes */
    uint32_t  word_mask;    /* Word address mask */
    uint32_t  byte_mask;    /* Byte address mask */
    uint32_t  MemStart;     /* First usable address (0x80000048 or 0x80000070) */
    uint32_t  ExtMemStart;  /* External memory start */
    uint32_t  CoreSize;     /* On-chip RAM size */
} emu_mem_t;

/* -------------------------------------------------------------------
 * Top-level emulator state — contains all subsystems.
 * ------------------------------------------------------------------- */
typedef struct {
    emu_cpu_t   cpu   EMU_ALIGN(EMU_CACHE_LINE);
    emu_fpu_t   fpu   EMU_ALIGN(EMU_CACHE_LINE);
    emu_timer_t timer EMU_ALIGN(EMU_CACHE_LINE);
    emu_mem_t   memory;
} emu_t;

/* -------------------------------------------------------------------
 * Inline memory access helpers — hot path
 * ------------------------------------------------------------------- */
static inline uint8_t emu_read_byte_fast(const emu_mem_t *m, uint32_t addr) {
    return m->mem[addr & m->byte_mask];
}

static inline uint32_t emu_read_word_fast(const emu_mem_t *m, uint32_t addr) {
    uint32_t idx = addr & m->byte_mask;
    uint32_t val;
    memcpy(&val, m->mem + idx, 4);  /* Safe unaligned read */
    return val;
}

static inline void emu_write_byte_fast(emu_mem_t *m, uint32_t addr, uint8_t val) {
    m->mem[addr & m->byte_mask] = val;
}

static inline void emu_write_word_fast(emu_mem_t *m, uint32_t addr, uint32_t val) {
    uint32_t idx = addr & m->byte_mask;
    memcpy(m->mem + idx, &val, 4);
}

/* Check if address is in MMIO range (not main memory) */
static inline int emu_is_mmio(uint32_t addr) {
    return (addr >= 0x10000000u && addr < 0xB0000000u);
}

/* -------------------------------------------------------------------
 * Timer access helpers — thread-safe
 * ------------------------------------------------------------------- */
static inline uint32_t emu_read_clock(const emu_timer_t *t, int priority) {
    return atomic_load_explicit(&t->ClockReg[priority], memory_order_relaxed);
}

static inline int emu_should_quit(const emu_timer_t *t) {
    return atomic_load_explicit(&t->quit, memory_order_relaxed);
}

static inline void emu_request_quit(emu_timer_t *t) {
    atomic_store_explicit(&t->quit, 1, memory_order_release);
}

#endif /* EMU_H */
