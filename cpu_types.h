#ifndef T4_CPU_TYPES_H
#define T4_CPU_TYPES_H

#include <stdint.h>
#include "fparithmetic.h"
#include "processor.h"

#define FP_UNKNOWN       0
#define FP_REAL32       32
#define FP_REAL64       64

typedef struct _REAL {
        uint32_t length;
        uint32_t rsvd;
        union {
                fpreal32_t  sn;
                fpreal64_t  db;
        } u;
} REAL;

#define T4_FAST_BLOCK_BITS 15
#define T4_FAST_BLOCK_SIZE (1U << T4_FAST_BLOCK_BITS)
#define T4_FAST_BLOCK_MASK (T4_FAST_BLOCK_SIZE - 1U)
#define T4_FAST_BLOCK_MAX_INSNS 32

typedef struct t4_fast_op {
        u_char op;
        u_char imm;
} t4_fast_op_t;

typedef struct t4_fast_block {
        uint32_t tag;
        u_char len;
        u_char bytes[T4_FAST_BLOCK_MAX_INSNS];
        t4_fast_op_t ops[T4_FAST_BLOCK_MAX_INSNS];
} t4_fast_block_t;

typedef struct _InstrSlot {
        uint32_t ip;
        uint32_t NextIPtr;
        uint32_t oreg;
        u_char   icode;
        u_char   rsvd[1];
        u_short  Pcode;
#ifdef EMUDEBUG
        u_char Instruction;
#endif
} InstrSlot;

typedef struct _ArgSlot {
        uint32_t _Arg0, _Arg1;
} ArgSlot;

#define IC_NOADDR       0xDEADBEEFU
#ifndef T4CACHEBITS
#define T4CACHEBITS     (14)
#endif
#define MAX_ICACHE      (1<<T4CACHEBITS)

#endif
