/*
 * t9000_group.h - IMS T9000 instruction-grouper / pipeline cycle model.
 *
 * The T4 emulator is functional, not cycle-accurate.  This adds an optional
 * *estimator* that models how the T9000's hardware grouper would pack the
 * executed instruction stream into pipeline groups, so that code (including
 * TVX) can be judged on second-generation silicon rather than on interpreter
 * wall-clock.  Enable with T4_T9000=1; a summary prints at exit.
 *
 * Model, from the IMS T9000 architecture description:
 *
 *   "Instructions are executed in a five stage pipeline: the first can fetch
 *    two local variables; the second can perform two address calculations,
 *    for accessing non-local or subscripted variables; the third stage can
 *    load two non-local variables; the next can perform an ALU or FPU
 *    operation; and the final stage can do a conditional jump or write."
 *
 *   "Up to 8 instructions can be put into one group and there may be five
 *    groups in the pipeline at any time."
 *
 * So one group per cycle, subject to per-stage resource limits:
 *
 *   stage 1  local reads          <= 2   (ldl, ldlp)
 *   stage 2  address calculations <= 2   (wsub, bsub, ldnlp, + every write)
 *   stage 3  non-local reads      <= 2   (ldnl)
 *   stage 4  ALU / FPU operation  <= 1
 *   stage 5  write or jump        <= 1   (stl, stnl, j, cj, call, ret)
 *   total instructions            <= 8
 *
 * Instructions are grouped strictly in issue order - the grouper never
 * reorders - so a resource conflict simply closes the current group.
 * Multi-cycle instructions (mul, div, the FPU, and the TVX block ops) close
 * the group and add their own latency.
 *
 * Three further constraints from the detailed pipeline description, all of
 * which the first cut of this model got wrong:
 *
 *  1. The workspace cache holds only "the first 32 locations of the
 *     workspace", and is what makes a local reference free of an address
 *     calculation: "an instruction which references one of the first 32
 *     locations of the workspace can be executed without having to compute
 *     the address".  `ldl 40` is therefore *not* a stage-1 local read - it
 *     costs a stage-2 address calculation plus a stage-3 memory read.
 *
 *  2. "Note that for [stl] (unlike for [ldl]) the address of the workspace
 *     location referenced has to be computed since a store will be made to
 *     memory as well as to the workspace cache."  Every store consumes a
 *     stage-2 slot *in addition to* the single stage-5 write slot.  This
 *     makes TCC's always-STL strategy more expensive than first modelled:
 *     each pseudo-register spill burns two pipeline resources.
 *
 *  3. The grouper "examin[es] up to 8 bytes at once", and the T9000 can
 *     "fetch only 4 bytes per cycle", so "the peak sustainable instruction
 *     execution rate of the T9000 is only 4 instructions per cycle" - not 8.
 *     Groups are therefore capped at 8 *bytes* as well as 8 instructions,
 *     and total cycles are bounded below by bytes/4.  This is what makes
 *     `pfix 0` padding cost real cycles even though the grouper folds
 *     prefixes into the instruction they modify: they still eat fetch
 *     bandwidth.  See the pfix-0 finding in the codegen audit.
 *
 * Opcode classification is checked against appendix B of the T9000
 * instruction set manual (secondary functions sorted by op-code).  The first
 * cut got several wrong by treating anything unrecognised as a stage-4 ALU
 * operation: `ret` (2356 uses in the kernel), `lb` (556), `gcall` (75) and
 * `sb` (18) are a branch, a memory read, a call and a store respectively.
 * Worse, `wsubdb` (0x81) is "form double word subscript" - an address
 * calculation - and sat inside the 0x80..0xAF range being charged as a
 * 3-cycle blocking FPU operation.
 *
 * IMPORTANT: this is a static resource model.  It does not model cache
 * misses, branch misprediction, link/DMA traffic or memory-interface stalls,
 * nor the cache-bank conflict that can stop two stage-3 reads issuing in the
 * same cycle, so it is an optimistic lower bound on cycles, useful for
 * *comparing* codegen strategies rather than predicting absolute performance.
 */

#ifndef T9000_GROUP_H
#define T9000_GROUP_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Size of the on-chip workspace cache, in words.  A reference to a workspace
 * location below this can use the instruction operand as a direct cache
 * address; above it, the address has to be computed. */
#define T9K_MAX_UNITS 8
#define T9K_MAX_ACC   8

#ifndef T9K_WS_CACHE
#define T9K_WS_CACHE 32
#endif

/* Bytes the grouper can examine at once, and bytes the fetcher delivers per
 * cycle.  The second is the one that caps sustained issue at 4 instr/cycle. */
#define T9K_GROUP_BYTES 8
#define T9K_FETCH_BYTES 4

/* Which pipeline resource an instruction consumes. */
enum {
    T9K_NONE = 0,   /* prefixes: folded into the instruction they modify */
    T9K_LOCAL,      /* stage 1: cached local read                        */
    T9K_LOCAL_FAR,  /* local beyond the cache: stage 2 + stage 3         */
    T9K_ADDR,       /* stage 2                                           */
    T9K_NONLOCAL,   /* stage 3                                           */
    T9K_ALU,        /* stage 4                                           */
    T9K_STORE,      /* stage 2 + stage 5: writes also compute an address */
    T9K_WRJMP,      /* stage 5 only: jumps                               */
    T9K_SLOW        /* multi-cycle: closes the group, adds latency       */
};

/* Machine parameters.  Defaults reproduce the documented T9000 exactly; the
 * T4_T9K_* environment variables override them so the model can be used for
 * *sensitivity analysis* - relax one resource at a time and see which ones
 * actually move the cycle count.  Guessing which pipeline change would help
 * is how architects waste silicon; this makes it measurable. */
typedef struct {
    int      enabled;
    /* per-stage issue limits */
    int      lim_local;    /* stage 1: cached local reads       (T9000: 2) */
    int      lim_addr;     /* stage 2: address calculations     (T9000: 2) */
    int      lim_nonlocal; /* stage 3: non-local reads          (T9000: 2) */
    int      lim_alu;      /* stage 4: ALU/FPU operations       (T9000: 1) */
    int      lim_wrjmp;    /* stage 5: writes or jumps          (T9000: 1) */
    int      lim_group;    /* instructions per group            (T9000: 8) */
    int      group_bytes;  /* bytes the grouper examines        (T9000: 8) */
    int      fetch_bytes;  /* bytes delivered per cycle         (T9000: 4) */
    int      ws_cache;     /* workspace-cache words             (T9000: 32) */
    /* behavioural switches */
    int      store_free_addr; /* 1: stl does not consume a stage-2 slot     */
    int      split_wr_jmp;    /* 1: a write and a jump can share a group    */
    int      slow_pipelined;  /* 1: multi-cycle ops do not close the group  */
    int      ajw_stage2;      /* 1: ajw is an address calc, not a stage-5 write */
    int      mac_lanes;       /* TVX MAC lanes; divides block latency       */
    int      n_units;         /* independent TVX functional units           */
    int      n_acc;           /* TVX accumulator registers                  */
    int      rd_bw;           /* block-op read bandwidth, bytes/cycle (8 =
                                 the documented 2 words from 4 banks)      */
    int      wr_bw;           /* block-op write bandwidth, bytes/cycle (4 =
                                 the documented single write port)         */
    /* Decoupled-execution timeline.  Only used when slow_pipelined is set;
     * the default path adds latency blindly, as before, so the documented
     * T9000 numbers are reproduced bit-for-bit. */
    uint64_t cycle;              /* groups closed + stall so far            */
    uint64_t unit_free[T9K_MAX_UNITS]; /* cycle each TVX unit is next free  */
    uint64_t acc_ready[T9K_MAX_ACC];   /* cycle each accumulator is valid   */
    int      rr_unit, rr_acc, last_acc;
    int      tvx_issue;   /* set by the classifier for TVX block ops   */
    uint64_t instrs;      /* instructions issued (prefixes excluded)   */
    uint64_t prefixes;    /* unused: see note in t9k_report()           */
    uint64_t bytes;       /* instruction bytes fetched, prefixes included */
    uint64_t groups;      /* groups formed, one per cycle              */
    uint64_t stall;       /* extra cycles from multi-cycle operations  */
    /* current group occupancy */
    int n, n_bytes, n_local, n_addr, n_nonlocal, n_alu, n_wrjmp;
    uint64_t hist[9];     /* histogram of group sizes 1..8             */
} t9k_state;

extern t9k_state t9k;
extern uint32_t tvx_src_fmt;    /* VSETFMT position format (p.c) */
extern uint32_t quant_fmt;      /* QSETFMT ggml type id (p.c)    */
extern uint32_t m2dLength;      /* move2dinit row count (p.c)    */

/* Apply the documented T9000 defaults, then any T4_T9K_* overrides. */
static inline void t9k_init(void)
{
    static const struct { const char *env; int def; size_t off; } tune[] = {
        { "T4_T9K_LOCAL",      2,  offsetof(t9k_state, lim_local)       },
        { "T4_T9K_ADDR",       2,  offsetof(t9k_state, lim_addr)        },
        { "T4_T9K_NONLOCAL",   2,  offsetof(t9k_state, lim_nonlocal)    },
        { "T4_T9K_ALU",        1,  offsetof(t9k_state, lim_alu)         },
        { "T4_T9K_WRJMP",      1,  offsetof(t9k_state, lim_wrjmp)       },
        { "T4_T9K_GROUP",      8,  offsetof(t9k_state, lim_group)       },
        { "T4_T9K_GROUPBYTES", T9K_GROUP_BYTES, offsetof(t9k_state, group_bytes) },
        { "T4_T9K_FETCH",      T9K_FETCH_BYTES, offsetof(t9k_state, fetch_bytes) },
        { "T4_T9K_WSCACHE",    T9K_WS_CACHE,    offsetof(t9k_state, ws_cache)    },
        { "T4_T9K_STOREFREE",  0,  offsetof(t9k_state, store_free_addr) },
        { "T4_T9K_SPLITWRJMP", 0,  offsetof(t9k_state, split_wr_jmp)    },
        { "T4_T9K_PIPESLOW",   0,  offsetof(t9k_state, slow_pipelined)  },
        { "T4_T9K_AJWSTAGE2",  0,  offsetof(t9k_state, ajw_stage2)      },
        { "T4_T9K_MACLANES",   1,  offsetof(t9k_state, mac_lanes)       },
        { "T4_T9K_UNITS",      1,  offsetof(t9k_state, n_units)         },
        { "T4_T9K_ACCUM",      1,  offsetof(t9k_state, n_acc)           },
        { "T4_T9K_RDBW",       8,  offsetof(t9k_state, rd_bw)           },
        { "T4_T9K_WRBW",       4,  offsetof(t9k_state, wr_bw)           },
    };
    size_t k;
    for (k = 0; k < sizeof(tune)/sizeof(tune[0]); k++) {
        const char *e = getenv(tune[k].env);
        int v = tune[k].def;
        if (e && *e) {
            v = atoi(e);
            if (v < 0) v = tune[k].def;
            fprintf(stderr, "[T9000] tuning %s = %d (default %d)\n",
                    tune[k].env, v, tune[k].def);
        }
        *(int *)((char *)&t9k + tune[k].off) = v;
    }
    {
        /* T4_T9K_PROFILE=tvx is the *recommended* TVX machine, and it is
         * exactly one change: a decoupled TVX functional unit.
         *
         * Four candidates were measured (see TVX-ISA.md section 12).  Three
         * of them - splitting the stage-5 write and branch ports, dropping
         * the stage-2 address charge for stores, and a second accumulator -
         * are worth 1%, 0% and 0% respectively, and enabling all four is one
         * cycle *worse* than the decoupled unit alone because splitting
         * write/branch perturbs group boundaries.  So the profile deliberately
         * does not bundle them; they remain available individually for
         * anyone who wants to re-run the comparison.
         *
         * Explicit T4_T9K_* settings still win, because they are applied
         * above. */
        const char *prof = getenv("T4_T9K_PROFILE");
        if (prof && (*prof == 't' || *prof == 'T')) {
            if (!getenv("T4_T9K_PIPESLOW")) t9k.slow_pipelined = 1;
            fprintf(stderr,
                "[T9000] profile 'tvx': decoupled TVX functional unit\n"
                "[T9000]   (block ops issue into stage 4, do not close the\n"
                "[T9000]    group, and cost only where the result is used)\n");
            /* T4_T9K_PROFILE=tvx2: the adopted 64-bit-path machine.
             * tvx plus: 8 MAC lanes, 64-bit read pairs (16 B/cy) and a
             * 64-bit write path (8 B/cy).  The bandwidth sweep in
             * TVX-GFX.md picked these: T&L is write-port bound at >= 4
             * lanes, MXFP4/Q8 is read-port bound at any lane count, and
             * lanes past the binding port buy nothing.  FTRANSPB's
             * record words and FSINCOSB's {sin,cos} pairs are spec'd as
             * paired 64-bit stores (8-byte-aligned destinations) to
             * make the write path real at instruction level. */
            if (prof[1] && prof[2] && prof[3] == '2') {
                if (!getenv("T4_T9K_MACLANES")) t9k.mac_lanes = 8;
                if (!getenv("T4_T9K_RDBW"))     t9k.rd_bw = 16;
                if (!getenv("T4_T9K_WRBW"))     t9k.wr_bw = 8;
                fprintf(stderr,
                    "[T9000] profile 'tvx2': + 8 MAC lanes, 64-bit "
                    "read/write paths (16/8 B per cycle)\n");
            }
        }
    }
    if (t9k.mac_lanes < 1) t9k.mac_lanes = 1;
    if (t9k.n_units < 1) t9k.n_units = 1;
    if (t9k.n_units > T9K_MAX_UNITS) t9k.n_units = T9K_MAX_UNITS;
    if (t9k.n_acc < 1) t9k.n_acc = 1;
    if (t9k.n_acc > T9K_MAX_ACC) t9k.n_acc = T9K_MAX_ACC;
}

static inline void t9k_close_group(void)
{
    if (t9k.n) {
        t9k.groups++;
        t9k.cycle++;
        if (t9k.n <= 8)
            t9k.hist[t9k.n]++;
    }
    t9k.n = t9k.n_bytes = t9k.n_local = t9k.n_addr = 0;
    t9k.n_nonlocal = t9k.n_alu = t9k.n_wrjmp = 0;
}

/* Feed one executed instruction to the model.  `kind` is a T9K_* class,
 * `latency` the extra cycle count for T9K_SLOW instructions, and `bytes` the
 * encoded length including any pfix/nfix folded into it. */
static inline void t9k_issue(int kind, int latency, int bytes)
{
    int d_local = 0, d_addr = 0, d_nonlocal = 0, d_alu = 0, d_wrjmp = 0;

    if (!t9k.enabled)
        return;
    if (bytes < 1)
        bytes = 1;
    if (kind == T9K_NONE) {          /* pfix / nfix cost no issue slot */
        t9k.prefixes++;
        t9k.bytes += (uint64_t)bytes;
        return;
    }
    t9k.instrs++;
    t9k.bytes += (uint64_t)bytes;

    if (kind == T9K_SLOW) {
        if (t9k.slow_pipelined) {
            /* Decoupled functional unit.
             *
             * The blocking model just adds `latency` to a running total,
             * which cannot express overlap at all: it is the same answer
             * whether or not anything independent follows.  Here the unit
             * has a real timeline.  A TVX op picks the earliest-free unit
             * and the next accumulator, issues as soon as both are
             * available, and *does not close the group* - independent
             * instructions keep grouping behind it.  The cost only becomes
             * visible when something actually needs the result.
             *
             * `is_tvx` distinguishes the decoupled block ops from ordinary
             * FPU work.  A non-TVX FPU instruction is treated as the
             * consumer of the most recent accumulator, which is what the
             * generated code does: the GEMV is followed by an FP store of
             * its result.  That is where a second accumulator can pay -
             * but only if the code has another row's GEMV in flight by
             * then, i.e. only if it is software-pipelined. */
            int is_tvx = (t9k.tvx_issue != 0);
            t9k.tvx_issue = 0;

            if (is_tvx) {
                int u = 0, a, i;
                uint64_t start;
                for (i = 1; i < t9k.n_units; i++)
                    if (t9k.unit_free[i] < t9k.unit_free[u])
                        u = i;
                a = t9k.rr_acc;
                t9k.rr_acc = (t9k.rr_acc + 1) % t9k.n_acc;

                start = t9k.cycle;
                if (t9k.unit_free[u] > start) start = t9k.unit_free[u];
                if (t9k.acc_ready[a] > start) start = t9k.acc_ready[a];
                if (start > t9k.cycle) {
                    t9k.stall += start - t9k.cycle;
                    t9k.cycle  = start;
                }
                t9k.unit_free[u] = start + (uint64_t)latency;
                t9k.acc_ready[a] = start + (uint64_t)latency;
                t9k.last_acc = a;
            } else {
                /* consumer of the accumulator: wait for it to be valid */
                uint64_t r = t9k.acc_ready[t9k.last_acc];
                if (r > t9k.cycle) {
                    t9k.stall += r - t9k.cycle;
                    t9k.cycle  = r;
                }
                t9k.stall += (uint64_t)(latency > 0 ? latency - 1 : 0);
                t9k.cycle += (uint64_t)(latency > 0 ? latency - 1 : 0);
            }

            if (t9k.n >= t9k.lim_group
                || t9k.n_bytes + bytes > t9k.group_bytes
                || t9k.n_alu + 1 > t9k.lim_alu)
                t9k_close_group();
            t9k.n_alu++;
            t9k.n_bytes += bytes;
            t9k.n++;
            return;
        }
        t9k.tvx_issue = 0;
        t9k_close_group();           /* cannot share a group */
        t9k.n = 1;
        t9k.n_bytes = bytes;
        t9k_close_group();
        t9k.stall += (uint64_t)(latency > 0 ? latency - 1 : 0);
        t9k.cycle += (uint64_t)(latency > 0 ? latency - 1 : 0);
        return;
    }

    switch (kind) {
    case T9K_LOCAL:     d_local = 1;               break;
    case T9K_LOCAL_FAR: d_addr = 1; d_nonlocal = 1; break;
    case T9K_ADDR:      d_addr = 1;                break;
    case T9K_NONLOCAL:  d_nonlocal = 1;            break;
    case T9K_ALU:       d_alu = 1;                 break;
    case T9K_STORE:     d_addr = t9k.store_free_addr ? 0 : 1;
                        d_wrjmp = 1;               break;
    case T9K_WRJMP:     d_wrjmp = 1;               break;
    default: break;
    }

    /* Would this instruction exceed a stage limit, the group size, or the
     * 8-byte window the grouper can examine at once? */
    if (t9k.n >= 8
        || t9k.n_bytes + bytes > T9K_GROUP_BYTES
        || t9k.n_local    + d_local    > 2
        || t9k.n_addr     + d_addr     > 2
        || t9k.n_nonlocal + d_nonlocal > 2
        || t9k.n_alu      + d_alu      > 1
        || t9k.n_wrjmp    + d_wrjmp    > 1)
        t9k_close_group();

    t9k.n_local    += d_local;
    t9k.n_addr     += d_addr;
    t9k.n_nonlocal += d_nonlocal;
    t9k.n_alu      += d_alu;
    t9k.n_wrjmp    += d_wrjmp;
    t9k.n_bytes    += bytes;
    t9k.n++;

    /* A write or jump is the last stage, so it always ends the group -
     * unless the write and branch slots have been split. */
    if (d_wrjmp && !t9k.split_wr_jmp)
        t9k_close_group();
}

/* Classify a primary (high-nibble) opcode.  `oreg` is the folded operand,
 * needed to tell a workspace-cache reference from a computed one. */
static inline int t9k_class_primary(int op, uint32_t oreg)
{
    switch (op) {
    case 0x2: case 0x6: return T9K_NONE;      /* pfix, nfix          */
    case 0x7: case 0x1:                       /* ldl, ldlp           */
        return (int)oreg < t9k.ws_cache ? T9K_LOCAL : T9K_LOCAL_FAR;
    case 0x5:           return T9K_ADDR;      /* ldnlp               */
    case 0x3:           return T9K_NONLOCAL;  /* ldnl                */
    case 0x4: case 0x8: case 0xC: return T9K_ALU; /* ldc, adc, eqc   */
    case 0xD: case 0xE: return T9K_STORE;     /* stl, stnl           */
    case 0x0: case 0xA: case 0x9:
        return T9K_WRJMP;                     /* j, cj, call         */
    case 0xB:
        /* ajw.  Charging it as a stage-5 write is the model's single largest
         * unverified assumption: it appears twice in every function, closes
         * 8.6% of all groups over a kernel corpus, and moving it to stage 2
         * lowers the estimate by 8.2%.
         *
         * The ISM defines it as `wptr' = wptr + n` - no memory access at all,
         * with AccessViolation signalled only as a protection *check* that the
         * new workspace address is writable.  Stage 5 is "a conditional jump
         * or write", which ajw is neither, so stage 2 (address calculations)
         * is the better-supported reading.
         *
         * The argument for the current default is that ajw moves Wptr, which
         * is the base every stage-1 workspace-cache read uses - so a following
         * ldl grouped with it would read against the stale base.  That argues
         * for closing the group, which T9K_WRJMP achieves, but it does not
         * argue for also consuming the single stage-5 slot, which is what
         * stops `ajw n ; ret` sharing a group in every epilogue.
         *
         * Neither manual settles it, so the default is left alone and the
         * question is made measurable instead - like every other T4_T9K_*.
         * Note that codegen *comparisons* are insensitive to this: the same
         * change set measures -26.7% with the default and -28.3% at stage 2. */
        return t9k.ajw_stage2 ? T9K_ADDR : T9K_WRJMP;
    default:            return T9K_ALU;
    }
}

/* How many units of work a TVX instruction will actually do.
 *
 * This matters more than it looks.  The block and vector instructions take
 * their count from CReg or from VSETLEN, so a single MXFP4GEMV can be one
 * block or a thousand; charging it a flat per-block latency would make a
 * whole GEMV look free and would flatter any code that uses one.  The
 * classifier runs *before* the instruction executes, so the operand
 * registers still hold the count and the real figure is available.
 *
 * Per-unit costs are the ones the model already used for a single unit, so
 * nothing changes for count == 1; only multi-unit issues get honest.
 */
static inline uint32_t t9k_opr_units(int oreg, uint32_t creg, uint32_t veclen)
{
    switch (oreg) {
    case 0x1C9:                                 /* BF16DOT: CReg elements  */
        return creg;
    case 0x1CA: case 0x1D0:                     /* GEMV: CReg blocks       */
    case 0x1D8: case 0x1D9:                     /* Q8DOT: CReg blocks      */
    case 0x1DA: case 0x1DB: case 0x1DC:         /* i-quants: CReg blocks   */
    case 0x1DE:                                 /* QDOT: CReg blocks       */
        return creg;
    case 0x1D1: case 0x1D2: case 0x1D3:         /* vector kernels:         */
    case 0x1D4: case 0x1D5:                     /*   VSETLEN elements      */
        return veclen;
    case 0x1DF: case 0x1E0: case 0x1E1:         /* rotation: 32/block      */
        return veclen / 32u;
    case 0x1E4:                                 /* FTRANSPB: VSETLEN verts */
    case 0x1E8:                                 /* FSINCOSB: VSETLEN angles*/
    case 0x1EA:                                 /* FTCULL:   VSETLEN verts */
    case 0x1EB:                                 /* TSKIN:    VSETLEN verts */
    case 0x1EE:                                 /* FLITB:    VSETLEN verts */
    case 0x1F0:                                 /* VBLEND8:  VSETLEN pixels*/
        return veclen;
    case 0x1EF:                                 /* QGEMM: rows x blocks    */
        return veclen * creg;
    case 0x1F1:                                 /* VFILL:    CReg bytes    */
        return creg;
    case 0x1F2:                                 /* VFILL2D:  width x rows  */
        return creg * m2dLength;
    case 0x1F3:                                 /* CSUM: halfwords         */
    case 0x1F6:                                 /* CSUMCPY: halfwords      */
        return creg / 2u ? creg / 2u : 1u;
    case 0x1F4:                                 /* VBLEND2D: px x rows     */
        return veclen * m2dLength;
    default:
        return 1;                               /* fixed-size instructions */
    }
}

/* Bandwidth decomposition for the high-throughput block ops: per unit of
 * work, how many MAC-equivalents does the functional array retire, how
 * many operand bytes stream through the read port, and how many result
 * bytes through the write port.  The unit cost is then
 *     max(mac/lanes, rd/rd_bw, wr/wr_bw)
 * so widening one resource exposes the next bottleneck instead of the
 * old per/lanes model, which happily predicted physically impossible
 * throughput (e.g. 5 record words per vertex written faster than the
 * single write port allows).  Ops not listed keep the legacy flat cost.
 *
 * Read figures assume streamed operands only; per-batch state (the
 * latched 4x4 matrix, Q8/MXFP4 weight *scales* re-used across a block)
 * is charged to the fixed dispatch, not per unit.  TSKIN assumes the
 * hardware holds the last joint matrix (hits dominate: weights arrive
 * joint-sorted), so its stream is weights + index pairs only. */
static inline int t9k_opr_bw(int oreg, uint32_t *mac, uint32_t *rd,
                             uint32_t *wr)
{
    switch (oreg) {
    case 0x1C8: case 0x1CA:                     /* MXFP4DOT/GEMV, f32 acts */
        *mac = 32; *rd = 17 + 128; *wr = 0; return 1;
    case 0x1D0:                                 /* MXFP4GEMVBF, bf16 acts  */
        *mac = 32; *rd = 17 + 64;  *wr = 0; return 1;
    case 0x1C9:                                 /* BF16DOT, per element    */
        *mac = 1;  *rd = 4;        *wr = 0; return 1;
    case 0x1D8:                                 /* Q8DOT, f32 acts         */
        *mac = 32; *rd = 34 + 128; *wr = 0; return 1;
    case 0x1D9:                                 /* Q8DOTBF, bf16 acts      */
        *mac = 32; *rd = 34 + 64;  *wr = 0; return 1;
    case 0x1D1: case 0x1D2:                     /* vadd/vmul, per element  */
        *mac = 1;  *rd = 8;        *wr = 4; return 1;
    case 0x1DF:                                 /* VHAD32, per 32-block    */
        *mac = 40; *rd = 128;      *wr = 128; return 1;
    case 0x1E4:                                 /* FTRANSPB, per vertex    */
        *mac = 21; *rd = tvx_src_fmt ? 6 : 12; *wr = 20; return 1;
    case 0x1E8:                                 /* FSINCOSB, per angle     */
        *mac = 11; *rd = 2;        *wr = 8; return 1;
    case 0x1EA:                                 /* FTCULL, per vertex      */
        *mac = 24; *rd = tvx_src_fmt ? 6 : 12; *wr = 1; return 1;
    case 0x1EB:                                 /* TSKIN, per vertex       */
        *mac = 26; *rd = 53;       *wr = 12; return 1;
    case 0x1EE:                                 /* FLITB, per vertex       */
        *mac = 15; *rd = tvx_src_fmt ? 6 : 12; *wr = 4; return 1;
    case 0x1EF:                                 /* QGEMM, per weight block */
        /* weight bytes per 32-elem block from the latched format (Q8_0 =
         * 34, MXFP4 and friends = 17), activation bytes from VSETFMT */
        *mac = 32;
        *rd = (quant_fmt == 8u ? 34u : 17u)
            + (tvx_src_fmt == 1u ? 64u : 128u);
        *wr = 1;
        return 1;
    case 0x1F0:                                 /* VBLEND8, per pixel      */
    case 0x1F4:                                 /* VBLEND2D, per pixel     */
        *mac = 2;  *rd = 8;        *wr = 4; return 1;
    case 0x1F1: case 0x1F2:                     /* VFILL(2D), per byte:    */
        *mac = 0;  *rd = 0;        *wr = 1; return 1;  /* pure write-bound */
    case 0x1F3:                                 /* CSUM, per halfword      */
        *mac = 1;  *rd = 2;        *wr = 0; return 1;
    case 0x1F6:                                 /* CSUMCPY, per halfword   */
        *mac = 1;  *rd = 2;        *wr = 2; return 1;
    default:
        return 0;
    }
}

/* Classify a secondary (OPR) operation, and give its latency in cycles.
 * `creg` and `veclen` are the live CReg and VSETLEN values, used to cost
 * the variable-length TVX instructions; pass 1 and 1 if unavailable. */
static inline int t9k_class_opr(int oreg, uint32_t creg, uint32_t veclen,
                                int *latency)
{
    *latency = 1;
    switch (oreg) {
    /* Classifications below are checked against appendix B of the T9000
     * instruction set manual (secondary functions sorted by op-code).
     * Anything not listed falls through to T9K_ALU. */

    /* stage 2: subscript / address calculation */
    case 0x0A:                                /* wsub                */
    case 0x02:                                /* bsub                */
    case 0x81:                                /* wsubdb - NOT an FPU
                                               * op; it is "form double
                                               * word subscript" and sits
                                               * inside the 0x80..0xAF
                                               * range by accident       */
    case 0xC1:                                /* ssub, 16-bit subscript */
        return T9K_ADDR;

    /* stage 3: memory reads */
    case 0x01:                                /* lb                  */
    case 0xB9:                                /* lbx                 */
    case 0xCA:                                /* ls                  */
    case 0xF9:                                /* lsx                 */
        return T9K_NONLOCAL;

    /* stage 2 + stage 5: memory writes */
    case 0x3B:                                /* sb                  */
    case 0xC8:                                /* ss                  */
        return T9K_STORE;

    /* stage 5: control transfer */
    case 0x06:                                /* gcall               */
    case 0x20:                                /* ret                 */
    case 0x21:                                /* lend - counted loop */
        return T9K_WRJMP;

    /* multi-cycle */
    case 0x72:                                /* fmul, fractional    */
        *latency = 2; return T9K_SLOW;
    case 0x74: case 0x75:                     /* crcword, crcbyte    */
        *latency = 2; return T9K_SLOW;
    case 0x07: case 0x0B:                     /* in, out (channel)   */
        *latency = 20; return T9K_SLOW;
    case 0x5C: case 0x5D: case 0x5E:          /* move2d*             */
        *latency = 8; return T9K_SLOW;
    case 0x53:                                /* mul: doc says 2-5   */
        *latency = 3; return T9K_SLOW;
    case 0x2C: case 0x1F:                     /* div, rem            */
        *latency = 5; return T9K_SLOW;
    case 0x31:                                /* lmul                */
        *latency = 3; return T9K_SLOW;
    case 0x1A:                                /* ldiv                */
        *latency = 5; return T9K_SLOW;
    case 0x19:                                /* norm: doc says 2-3  */
        *latency = 3; return T9K_SLOW;
    case 0x4A:                                /* move (block)        */
        *latency = 8; return T9K_SLOW;
    default:
        /* FPU loads and stores are memory operations that happen to
         * target the FP stack, not ALU work: appendix B names them
         * "floating point load/store non-local".  Charging them as
         * blocking 3-cycle FPU operations, as the first cut did, badly
         * over-costs scalar float code. */
        switch (oreg) {
        case 0x82: case 0x86: case 0x8A: case 0x8E:  /* fpldnl{dbi,sni,db,sn} */
            return T9K_NONLOCAL;
        case 0x84: case 0x88: case 0x9E:             /* fpstnl{db,sn,i32}     */
            return T9K_STORE;
        default: break;
        }
        /* T9000-only FPU secondaries (direct encodings of the fpentry
         * operations).  fpsqrt keeps the 8-cycle figure the comment above
         * quotes for the single-precision hardware; precision conversions
         * cost a normalise pass; the by-2/exp/abs monadics and the
         * rounding-mode sets are simple exponent/flag updates. */
        switch (oreg) {
        case 0xD3:                            /* fpsqrt              */
            *latency = 8; return T9K_SLOW;
        case 0xD7: case 0xD8:                 /* fpr32tor64/64tor32  */
            *latency = 3; return T9K_SLOW;
        case 0xD1: case 0xD2:                 /* fpdivby2, fpmulby2  */
        case 0xD9: case 0xDA:                 /* fpexpdec32/inc32    */
        case 0xDB:                            /* fpabs               */
            *latency = 2; return T9K_SLOW;
        case 0xD0: case 0xD4: case 0xD5: case 0xD6:  /* fprn/rp/rm/rz */
        case 0xCB: case 0xDC:                 /* fpseterr, fpclrerr  */
            return T9K_ALU;
        default: break;
        }
        if (oreg >= 0x80 && oreg <= 0xAF) {   /* T800 FPU            */
            /* "The FPU is capable of performing single precision addition
             * and multiplication in 2 cycles, and double precision
             * multiplication in 3 cycles."  The T800 encoding does not
             * distinguish the precision here - it comes from the rounding
             * mode set by the fpldnlsn/fpldnldb load - so add and multiply
             * take the faster figure and everything else keeps 3. */
            switch (oreg) {
            case 0x87: case 0x89: case 0x8B:  /* fpadd, fpsub, fpmul */
                *latency = 2; return T9K_SLOW;
            default: break;
            }
            *latency = 3;
            return T9K_SLOW;
        }
        if (oreg >= 0x1C0 && oreg <= 0x1F6) { /* TVX + TVX-GFX           */
            /* Block and vector ops do a lot of work per issue; scalar
             * conversions are single-cycle ALU operations.  The figures
             * below are *per unit of work* - see t9k_opr_units() - and are
             * multiplied by the count the instruction was actually given. */
            int per = 0;
            switch (oreg) {
            case 0x1C8: per = 32; break;             /* MXFP4DOT, 1 block */
            case 0x1C9: per =  1; break;             /* BF16DOT, per elem */
            case 0x1CA: per = 32; break;             /* MXFP4GEMV /block  */
            case 0x1D0: per = 32; break;             /* MXFP4GEMVBF/block */
            case 0x1CB: case 0x1CC: per = 8; break;  /* 8 lanes, fixed    */
            case 0x1D1: case 0x1D2:
                per = 1; break;                      /* vadd/vmul per elem*/
            case 0x1D3: case 0x1D4: case 0x1D5:
                per = 2; break;                      /* transcendental    */
            case 0x1D8: case 0x1D9: per = 32; break; /* Q8DOT   /block    */
            case 0x1DA: case 0x1DB: case 0x1DC:
            case 0x1DE: per = 64; break;             /* 256-elem block    */
            /* Block rotation.  A 32-point FWHT is 5 stages x 16 butterflies
             * = 80 add/sub pairs, but the butterflies within a stage are
             * independent, so a datapath that retires two lanes per cycle
             * runs one stage in 8 cycles and the whole transform in 40.
             * MXQROT adds the amax pass and 32 E2M1 roundings on top; those
             * are 4-bit results and pack 8 to a word, so charge the same
             * 8 cycles the R32TOMXFP4X8 lane converter costs, twice, plus
             * the amax reduction.  MXQROTBF adds a bf16 widen per lane. */
            case 0x1DF: per = 40; break;             /* VHAD32  /block    */
            case 0x1E0: per = 64; break;             /* MXQROT  /block    */
            case 0x1E1: per = 72; break;             /* MXQROTBF/block    */
            /* TVX-GFX geometry, calibrated against HRM Table 6.32 (fpadd/
             * fpmul = 2, fpdiv/fpsqrt = 8 single) on the same 1-MAC/cycle
             * array convention the TVX figures use.
             * FTRANSP: 19 operand reads @2/cy + 21 MACs + fpdiv 8 + 5
             * record writes @1/cy, imperfectly overlapped -> 30.
             * FTRANSPB: matrix latched once; steady state is 21 MACs with
             * the divide overlapped on the reciprocal unit -> 21/vertex
             * (write-port floor ~5/vertex once the MAC array is >= 4
             * lanes wide).
             * FSINCOS: the Cephes polynomials are normative, ~11 MAC-
             * equivalents + BAMS prep + 2 writes -> 12 (SH-4's 3-cycle
             * FSCA had looser 2^-21 table hardware).
             * FRSQRT: seed + 2 dependent NR steps = 8 chained MACs ->
             * parity with hardware fpsqrt(single) = 8; still wins by
             * replacing fpsqrt+fpdiv = 16 and staying in the bit domain. */
            case 0x1E2: per = 5;  break;             /* FTSETVP: 5 loads  */
            case 0x1E3: per = 30; break;             /* FTRANSP /vertex   */
            case 0x1E4: per = 21; break;             /* FTRANSPB/vertex   */
            case 0x1E5: per = 12; break;             /* FSINCOS           */
            case 0x1E6: per = 10; break;             /* FRSQRT: 3 NR steps*/
            case 0x1E7: per = 10; break;             /* FRECIP: 3 NR steps*/
            case 0x1E8: per = 12; break;             /* FSINCOSB /angle   */
            case 0x1E9: per = 2;  break;             /* VSETSTRIDE latch  */
            case 0x1EC: per = 2;  break;             /* VSETFMT latch     */
            case 0x1ED: per = 5;  break;             /* FTSETLIT: 5 loads */
            /* FTCULL: worst case 6 plane dots = 24 MACs / 4 lanes + sign
             * harvest; TSKIN: md5 average ~2.2 weights x 12 MACs / 4
             * lanes + strided store. */
            case 0x1EA: per = 8;  break;             /* FTCULL  /vertex   */
            case 0x1EB: per = 9;  break;             /* TSKIN   /vertex   */
            /* nominal only - t9k_opr_bw() supersedes these three, but a
             * missing entry here would fall through to the 1-cycle ALU
             * default and never reach the bandwidth model at all. */
            case 0x1EE: per = 8;  break;             /* FLITB   /vertex   */
            case 0x1EF: per = 40; break;             /* QGEMM   /block    */
            case 0x1F0: per = 2;  break;             /* VBLEND8 /pixel    */
            case 0x1F1: per = 1;  break;             /* VFILL   /byte     */
            case 0x1F2: per = 1;  break;             /* VFILL2D /byte     */
            case 0x1F3: per = 1;  break;             /* CSUM    /halfword */
            case 0x1F4: per = 2;  break;             /* VBLEND2D/pixel    */
            case 0x1F5: per = 8;  break;             /* STVXSTATE: 15 wr  */
            case 0x1F6: per = 1;  break;             /* CSUMCPY /halfword */
            default: return T9K_ALU;                 /* conversions       */
            }
            {
                uint64_t n = t9k_opr_units(oreg, creg, veclen);
                uint32_t mac, rdb, wrb;
                uint64_t t;

                if (n == 0) n = 1;
                if (t9k_opr_bw(oreg, &mac, &rdb, &wrb)) {
                    /* bottleneck of MAC array, read port, write port */
                    uint64_t lanes = (uint64_t)t9k.mac_lanes;
                    uint64_t tm = ((uint64_t)mac * n + lanes - 1) / lanes;
                    uint64_t tr = ((uint64_t)rdb * n
                                   + (uint64_t)t9k.rd_bw - 1)
                                  / (uint64_t)t9k.rd_bw;
                    uint64_t tw = ((uint64_t)wrb * n
                                   + (uint64_t)t9k.wr_bw - 1)
                                  / (uint64_t)t9k.wr_bw;
                    t = tm;
                    if (tr > t) t = tr;
                    if (tw > t) t = tw;
                } else {
                    t = (uint64_t)per * n;
                    /* A wider MAC array divides the per-block cost. */
                    t = (t + (uint64_t)t9k.mac_lanes - 1)
                        / (uint64_t)t9k.mac_lanes;
                }
                if (t < 1) t = 1;
                *latency = (int)(t > 0x7FFFFFFFu ? 0x7FFFFFFF : t);
                t9k.tvx_issue = 1;
            }
            return T9K_SLOW;
        }
        return T9K_ALU;
    }
}

/* Cycles so far, on the same basis as t9k_report(): whichever of the issue
 * bound and the fetch bound is larger, plus multi-cycle stall.  Does not close
 * the group in progress - this is meant to be called mid-run, and closing
 * would perturb the very grouping it is measuring.  At most one group of
 * error, which is nothing against a boot. */
static inline uint64_t t9k_cycles_so_far(void)
{
    uint64_t fetch = (t9k.bytes + (uint64_t)(t9k.fetch_bytes - 1))
                     / (uint64_t)t9k.fetch_bytes;
    return (t9k.groups > fetch ? t9k.groups : fetch) + t9k.stall;
}

/* One line of progress: what the machine has cost by the time the guest
 * printed `tag`.  Used to attribute cycles to phases of a boot rather than
 * only reporting a single total at exit. */
static inline void t9k_mark(const char *tag)
{
    if (!t9k.enabled)
        return;
    fprintf(stderr, "[T9000-MARK] %-24s instrs=%llu bytes=%llu groups=%llu "
                    "stall=%llu cycles=%llu (%.1f ms @50MHz)\n",
            tag,
            (unsigned long long)t9k.instrs, (unsigned long long)t9k.bytes,
            (unsigned long long)t9k.groups, (unsigned long long)t9k.stall,
            (unsigned long long)t9k_cycles_so_far(),
            (double)t9k_cycles_so_far() / 50000.0);
    fflush(stderr);
}

static inline void t9k_report(void)
{
    uint64_t cycles, fetch;
    int i;

    if (!t9k.enabled)
        return;
    t9k_close_group();
    /* The fetcher delivers 4 bytes/cycle, so the encoded size of the stream
     * is an independent lower bound on cycles.  Whichever bound is larger
     * is the one that binds - report both so it is obvious which. */
    fetch = (t9k.bytes + (uint64_t)(t9k.fetch_bytes - 1))
            / (uint64_t)t9k.fetch_bytes;
    cycles = (t9k.groups > fetch ? t9k.groups : fetch) + t9k.stall;
    if (!cycles)
        return;
    fprintf(stderr, "\n[T9000] instruction grouper model (estimate, no cache/branch effects)\n");
    fprintf(stderr, "[T9000]   instructions issued : %llu\n", (unsigned long long)t9k.instrs);
    /* pfix/nfix are folded into OReg by the fetch loop and never reach the
     * issue hook as separate instructions, which matches the T9000 grouper -
     * but their bytes are still charged to the fetch bound below. */
    fprintf(stderr, "[T9000]   instruction bytes   : %llu (%.2f bytes/instr)\n",
            (unsigned long long)t9k.bytes,
            t9k.instrs ? (double)t9k.bytes / (double)t9k.instrs : 0.0);
    fprintf(stderr, "[T9000]   groups (issue bound): %llu\n", (unsigned long long)t9k.groups);
    fprintf(stderr, "[T9000]   fetch bound (4 B/cy): %llu%s\n",
            (unsigned long long)fetch,
            fetch > t9k.groups ? "   <- binding: code size, not issue" : "");
    fprintf(stderr, "[T9000]   multi-cycle stall   : %llu\n", (unsigned long long)t9k.stall);
    fprintf(stderr, "[T9000]   total cycles        : %llu\n", (unsigned long long)cycles);
    fprintf(stderr, "[T9000]   instructions/cycle  : %.3f\n",
            (double)t9k.instrs / (double)cycles);
    fprintf(stderr, "[T9000]   group size histogram:");
    for (i = 1; i <= 8; i++)
        fprintf(stderr, " %d:%llu", i, (unsigned long long)t9k.hist[i]);
    fprintf(stderr, "\n[T9000]   at 50 MHz that is    : %.3f ms\n",
            (double)cycles / 50000.0);
}

#endif /* T9000_GROUP_H */
