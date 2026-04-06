/*
 * cpu_dispatch.h — Computed goto instruction dispatch for T4 emulator
 *
 * This header defines the DISPATCH/NEXT macros for the computed goto
 * technique. Include this in cpu.c and define the label addresses.
 *
 * Usage:
 *   FETCH();          // Fetch next instruction byte, decode, goto handler
 *   ... handler code ...
 *   NEXT();           // Fetch next instruction and continue
 *
 * GCC/Clang extension: Labels as values (&&label)
 * Falls back to switch() on other compilers.
 */
#ifndef CPU_DISPATCH_H
#define CPU_DISPATCH_H

#ifdef __GNUC__
#define USE_COMPUTED_GOTO 1
#else
#define USE_COMPUTED_GOTO 0
#endif

#if USE_COMPUTED_GOTO

/*
 * Computed goto dispatch table — 16 entries for primary opcodes.
 * Defined in cpu.c, used by DISPATCH/NEXT macros.
 *
 * Each entry is a label address (&&op_xxx) resolved at compile time.
 * Branch prediction works better than switch because each NEXT()
 * generates a unique indirect branch site with its own BTB entry.
 */
#define DECLARE_DISPATCH_TABLE \
    static const void *dispatch_table[16] = { \
        &&op_j, &&op_ldlp, &&op_pfix, &&op_ldnl, \
        &&op_ldc, &&op_ldnlp, &&op_nfix, &&op_ldl, \
        &&op_adc, &&op_call, &&op_cj, &&op_ajw, \
        &&op_eqc, &&op_stl, &&op_stnl, &&op_opr \
    }

/*
 * FETCH: Read instruction byte, decode opcode + operand nibble,
 *        accumulate into OReg, dispatch to handler.
 */
#define FETCH(cpu, mem, mask) do { \
    uint8_t _insn = (mem)[(cpu)->IPtr & (mask)]; \
    (cpu)->OReg += (_insn & 0x0F); \
    goto *dispatch_table[_insn >> 4]; \
} while(0)

/*
 * NEXT: Complete current instruction (IPtr++, OReg=0) and fetch next.
 * Use after every non-prefix instruction.
 */
#define NEXT(cpu, mem, mask) do { \
    (cpu)->IPtr++; \
    (cpu)->OReg = 0; \
    FETCH(cpu, mem, mask); \
} while(0)

/*
 * NEXT_NOADVANCE: Fetch without incrementing IPtr (for J/CJ that set IPtr).
 */
#define NEXT_NOADVANCE(cpu, mem, mask) do { \
    (cpu)->OReg = 0; \
    FETCH(cpu, mem, mask); \
} while(0)

/*
 * PREFIX_NEXT: After PFIX/NFIX, continue without resetting OReg.
 */
#define PREFIX_NEXT(cpu, mem, mask) do { \
    (cpu)->IPtr++; \
    FETCH(cpu, mem, mask); \
} while(0)

#else /* !USE_COMPUTED_GOTO — switch fallback */

#define DECLARE_DISPATCH_TABLE  /* nothing */

#define FETCH(cpu, mem, mask) do { \
    uint8_t _insn = (mem)[(cpu)->IPtr & (mask)]; \
    (cpu)->OReg += (_insn & 0x0F); \
    _dispatch_opcode = _insn >> 4; \
    goto _dispatch_switch; \
} while(0)

#define NEXT(cpu, mem, mask) do { \
    (cpu)->IPtr++; \
    (cpu)->OReg = 0; \
    FETCH(cpu, mem, mask); \
} while(0)

#define NEXT_NOADVANCE(cpu, mem, mask) do { \
    (cpu)->OReg = 0; \
    FETCH(cpu, mem, mask); \
} while(0)

#define PREFIX_NEXT(cpu, mem, mask) do { \
    (cpu)->IPtr++; \
    FETCH(cpu, mem, mask); \
} while(0)

#endif /* USE_COMPUTED_GOTO */

#endif /* CPU_DISPATCH_H */
