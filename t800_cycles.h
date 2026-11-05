/*
 * t800_cycles.h - estimated IMS T800 cycle count of the executed stream.
 *
 * T4 is functional, not cycle-accurate.  With T4_EST_CYCLES=1 it adds up,
 * before each instruction executes, that instruction's cycle count from
 * the T800 data sheet timings (the table in utils/instructions.txt), so
 * code generators can be compared on T800 silicon rather than on the
 * interpreter's wall clock or on instruction counts:
 *
 *   - every pfix/nfix costs 1 cycle;
 *   - the data-dependent ones are taken from the live registers: shl/shr
 *     n+2 and lshl/lshr n+3 (n = the count in A), norm n+5 (n = leading
 *     zeros of the double word), prod b+4 (b = the top set bit of A), cj
 *     4 taken / 2 not, lend 10 looping / 5 leaving, move/in/out 2w+8 or
 *     2w+19 (w = words), bitcnt b+2, bitrevnbits n+4;
 *   - FPU instructions use the single or double figure by the type in FA
 *     (fpentry adds the sub-operation named by A to its own cycle);
 *   - descheduling, channel waits, timers and memory wait states are not
 *     modelled.  On the T425/T414 the integer timings are the same; the
 *     T9000's are not (T4_T9000=1 is its own model).
 *
 * Instruction fusion is turned off while the model runs, so every
 * instruction is costed with its own registers.  The totals print at exit.
 * T4_EST_CYCLES=1 also makes ldtimer return the estimated cycle count
 * (low 32 bits), and =2 the executed instruction count, so a program's
 * own timer deltas become deterministic cycle or instruction counts,
 * immune to host load.
 */
#ifndef T800_CYCLES_H
#define T800_CYCLES_H

#include <stdint.h>
#include <stdio.h>

extern int t8c_enabled;                 /* 1 = cycles, 2 = instructions */
extern T4_THREAD_LOCAL uint64_t t8c_cycles, t8c_insns, t8c_prefixes;

static inline uint32_t t8c_topbit(uint32_t a)
{
    uint32_t b = 0;

    while (a >>= 1)
        b++;
    return b;
}

static inline uint32_t t8c_clz64(uint32_t hi, uint32_t lo)
{
    uint32_t n = 0;

    if (!hi && !lo)
        return 64;
    if (!hi) {
        n = 32;
        hi = lo;
    }
    while (!(hi & 0x80000000u)) {
        hi <<= 1;
        n++;
    }
    return n;
}

/* An FPU operation's cycles, single or double; the fpentry sub-operations
 * are offset by 0x1000. */
static inline uint32_t t8c_fpu(uint32_t op, int db)
{
    switch (op) {
    case 0x82: return 6;                    /* fpldnldbi */
    case 0x83: return 1;                    /* fpchkerr */
    case 0x84: return 3;                    /* fpstnldb */
    case 0x86: return 4;                    /* fpldnlsni */
    case 0x87: case 0x89: return 6;         /* fpadd fpsub */
    case 0x88: return 2;                    /* fpstnlsn */
    case 0x8a: return 3;                    /* fpldnldb */
    case 0x8b: return db ? 18 : 11;         /* fpmul */
    case 0x8c: return db ? 31 : 16;         /* fpdiv */
    case 0x8e: return 2;                    /* fpldnlsn */
    case 0x8f: return 36;                   /* fpremfirst */
    case 0x90: return 32;                   /* fpremstep */
    case 0x91: return 2;                    /* fpnan */
    case 0x92: return 3;                    /* fpordered */
    case 0x93: return 2;                    /* fpnotfinite */
    case 0x94: return 4;                    /* fpgt */
    case 0x95: return 3;                    /* fpeq */
    case 0x96: case 0x98: case 0x9a: return 8; /* fpi32tor32/64 fpb32tor64 */
    case 0x9c: return 2;                    /* fptesterr */
    case 0x9d: return 7;                    /* fprtoi32 */
    case 0x9e: return 4;                    /* fpstnli32 */
    case 0x9f: case 0xa0: return 2;         /* fpldzerosn/db */
    case 0xa1: return 5;                    /* fpint */
    case 0xa3: case 0xa4: case 0xab: return 1; /* fpdup fprev fpentry */
    case 0xa6: return 9;                    /* fpldnladddb */
    case 0xa8: return 21;                   /* fpldnlmuldb */
    case 0xaa: return 8;                    /* fpldnladdsn */
    case 0xac: return 13;                   /* fpldnlmulsn */
    case 0x1001: return 27;                 /* fpusqrtfirst */
    case 0x1002: return 42;                 /* fpusqrtstep */
    case 0x1003: return 8;                  /* fpusqrtlast */
    case 0x1007: return 3;                  /* fpur32tor64 */
    case 0x1008: return 6;                  /* fpur64tor32 */
    case 0x1009: case 0x100a: return 6;     /* fpuexpdec32/inc32 */
    case 0x100b: case 0x100d: return 2;     /* fpuabs fpunoround */
    case 0x100e: case 0x100f: return 3;     /* fpuchki32/64 */
    case 0x1011: case 0x1012: return 6;     /* fpudivby2/mulby2 */
    default: return 1;                      /* rounding modes, error flags */
    }
}

static inline uint32_t t8c_cost(uint32_t icode, uint32_t oreg, uint32_t a,
                                uint32_t b, int db)
{
    switch (icode) {
    case 0x00: return 3;                    /* j */
    case 0x10: return 1;                    /* ldlp */
    case 0x30: return 2;                    /* ldnl */
    case 0x40: return 1;                    /* ldc */
    case 0x50: return 1;                    /* ldnlp */
    case 0x70: return 2;                    /* ldl */
    case 0x80: return 1;                    /* adc */
    case 0x90: return 7;                    /* call */
    case 0xa0: return a == 0 ? 4 : 2;       /* cj */
    case 0xb0: return 1;                    /* ajw */
    case 0xc0: return 2;                    /* eqc */
    case 0xd0: return 1;                    /* stl */
    case 0xe0: return 2;                    /* stnl */
    }
    if (oreg >= 0x81 && oreg <= 0xac && oreg != 0x81)
        return t8c_fpu(oreg, db);
    switch (oreg) {
    case 0x00: case 0x02: case 0x04: case 0x05: case 0x0c: case 0x10:
    case 0x17: case 0x18: case 0x1c: case 0x1e: case 0x32: case 0x33:
    case 0x42: case 0x46: case 0x4b: case 0x50: case 0x52: case 0x54:
    case 0x57: case 0x58: case 0x5a: case 0x71:
        return 1;               /* rev bsub diff add sub ... mint and or sum dup */
    case 0x01: return 5;                    /* lb */
    case 0x03: return 13;                   /* endp */
    case 0x06: return 4;                    /* gcall */
    case 0x07: case 0x0b: return 2 * ((a + 3) / 4) + 19; /* in out */
    case 0x08: return t8c_topbit(a) + 4;    /* prod */
    case 0x0d: return 12;                   /* startp */
    case 0x0e: case 0x0f: return 23;        /* outbyte outword */
    case 0x12: return 3;                    /* resetch */
    case 0x15: return 11;                   /* stopp */
    case 0x19: return t8c_clz64(b, a) + 5;  /* norm */
    case 0x1a: return 35;                   /* ldiv */
    case 0x1f: return 37;                   /* rem */
    case 0x20: return 5;                    /* ret */
    case 0x21: return 10;                   /* lend (5 when leaving: below) */
    case 0x2b: return 30;                   /* tin */
    case 0x2c: return 39;                   /* div */
    case 0x2e: return 23;                   /* dist */
    case 0x2f: return 8;                    /* disc */
    case 0x30: return 4;                    /* diss */
    case 0x31: return 33;                   /* lmul */
    case 0x35: case 0x36: return (a > 64 ? 64 : a) + 3; /* lshr lshl */
    case 0x39: return 10;                   /* runp */
    case 0x3a: return 4;                    /* xword */
    case 0x3b: return 4;                    /* sb */
    case 0x3d: case 0x3e: return 4;         /* savel saveh */
    case 0x3f: return 5;                    /* wcnt */
    case 0x40: case 0x41: return (a > 64 ? 64 : a) + 2; /* shr shl */
    case 0x44: return 5;                    /* altwt */
    case 0x45: return 4;                    /* altend */
    case 0x47: return 8;                    /* enbt */
    case 0x48: return 7;                    /* enbc */
    case 0x49: return 3;                    /* enbs */
    case 0x4a: return 2 * ((a + 3) / 4) + 8; /* move */
    case 0x4c: return 3;                    /* csngl */
    case 0x4d: return 3;                    /* ccnt1 */
    case 0x4e: return 4;                    /* talt */
    case 0x51: return 15;                   /* taltwt */
    case 0x53: return 40;                   /* mul */
    case 0x56: return 5;                    /* cword */
    case 0x5b: return 8;                    /* move2dinit */
    case 0x63: return 15;                   /* unpacksn */
    case 0x6c: return 30;                   /* postnormsn, worst case */
    case 0x6d: return 12;                   /* roundsn */
    case 0x72: return 38;                   /* fmul */
    case 0x73: return 3;                    /* cflerr */
    case 0x74: return 35;                   /* crcword */
    case 0x75: return 11;                   /* crcbyte */
    case 0x76: return t8c_topbit(a) + 2;    /* bitcnt */
    case 0x77: return 36;                   /* bitrevword */
    case 0x78: return (a > 32 ? 32 : a) + 4; /* bitrevnbits */
    case 0x81: return 3;                    /* wsubdb */
    default: return 2;      /* gt wsub ladd lsum ldiff ldpi xdble ... */
    }
}

/* Count one instruction about to execute: icode/oreg as dispatched,
 * a/b the registers it starts from, prefixes its pfix/nfix bytes, db
 * whether FA holds a double.  lend leaves when the count word it will
 * decrement (B points at the control block) is 1; that needs memory, so it
 * is costed as looping, the common case in a hot loop. */
static inline void t8c_count(uint32_t icode, uint32_t oreg, uint32_t a,
                             uint32_t b, uint32_t prefixes, int db)
{
    uint32_t k;

    t8c_insns++;
    t8c_prefixes += prefixes;
    if (icode == 0xf0 && oreg == 0xab)      /* fpentry: its sub-op is in A */
        k = 1 + t8c_fpu(0x1000 + (a & 0xfff), db);
    else
        k = t8c_cost(icode, oreg, a, b, db);
    t8c_cycles += k + prefixes;
}

static inline void t8c_report(void)
{
    if (!t8c_enabled)
        return;
    printf("-I-EMU: T800 cycle estimate: %llu cycles, %llu instructions, "
           "%llu prefix bytes (%.2f cycles/instruction)\n",
           (unsigned long long)t8c_cycles, (unsigned long long)t8c_insns,
           (unsigned long long)t8c_prefixes,
           t8c_insns ? (double)t8c_cycles / (double)t8c_insns : 0.0);
    fflush(stdout);
}

#endif /* T800_CYCLES_H */
