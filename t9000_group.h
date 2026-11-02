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
 * IMPORTANT: GROUP COUNT IS A WEAK PROXY FOR CYCLES ON THIS CORE.
 *
 * The model charges one cycle per group.  The RTL measures **15.4 cycles per
 * group** at 3.34 ops/group: the core spends its time *inside* groups, not
 * switching between them, and the dominant costs are per-op, not per-group
 * (ldl 4.41, cj 9.38, j 15.66 cycles/op).  Removing a group therefore mostly
 * relocates work rather than eliminating it, and group-count deltas convert to
 * cycles at a poor and non-uniform rate - measured repeatedly:
 *
 *     grouper double-buffering (a real structural limit)   ->  -0.4% cycles
 *     uncond_free                       -7.8% groups       ->  -0.8% cycles
 *     pipelining step 1 + rider         -7.8% groups       ->  -3.0% cycles
 *
 * So a "-23.5% groups" result should be read as perhaps 3-5% of cycles until
 * the conversion rate has been measured on the workload in question.  Use the
 * group count to compare codegen *shapes*; do not quote it as a cycle saving.
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

/* Why a group ended - see t9k.why[]. */
enum {
    T9KW_OTHER = 0,
    T9KW_STORE,      /* stl/stnl: stage-5 write terminates the group   */
    T9KW_JUMP,       /* j/cj/call/ret/ajw: stage-5 branch terminates it */
    T9KW_SLOW,       /* multi-cycle op (FPU, mul, div, TVX)            */
    T9KW_S_LOCAL,    /* stage 1 full: 2 cached local reads             */
    T9KW_S_ADDR,     /* stage 2 full: 2 address calculations           */
    T9KW_S_NONLOC,   /* stage 3 full: 2 non-local reads                */
    T9KW_S_ALU,      /* stage 4 full: 1 ALU/FPU op                     */
    T9KW_BYTES,      /* the 8-byte fetch window filled                 */
    T9KW_COUNT8      /* 8 instructions                                 */
};
/* Opcode that caused the most recent close, for T4_T9K_WHY reporting. */

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
    /* 1: a write and a jump can share a group.
     *
     * This is the switch that tracks the planned grouper change on the RTL
     * side (split the write slot from CJ in t9_grouper.sv, measured -12.1%
     * groups).  It is a deliberate divergence from HRM section 2.4.1's
     * "conditional jump *or* write" - taken on the reading that the wording
     * describes their port budget rather than a correctness constraint, the
     * same reasoning already applied to splitting cu/cf.  Recorded here
     * because the compiler is now tuned against classify_uop, so every grouper
     * divergence has to be representable on this side or codegen decisions
     * are made against a machine that no longer exists. */
    int      split_wr_jmp;
    int      slow_pipelined;  /* 1: multi-cycle ops do not close the group  */
    int      ajw_stage2;      /* 1: ajw is an address calc, not a stage-5 write */
    int      hrm_class;       /* 1: pre-correction HRM classification of ldc/adc/eqc */
    int      stalls_off;      /* 1: ignore inter-group produce/consume stalls */
    uint64_t stall_cycles;    /* cycles added by produce/consume distance */
    int      g_produce;       /* max produce stage in the group being built */
    int      g_consume;       /* min consume stage in the group being built */
    int      prev_produce;    /* max produce stage of the group just closed */
    int      mac_lanes;       /* TVX MAC lanes; divides block latency       */
    int      n_units;         /* independent TVX functional units           */
    int      n_acc;           /* TVX accumulator registers                  */
    /* Group-close attribution (T4_T9K_WHY=1).  Counts why each group ended,
     * so codegen work can be aimed at the resource that actually binds
     * rather than at whichever one looks worst in a disassembly. */
    int      why_on;
    uint64_t why[8];          /* indexed by T9KW_* */
    uint64_t why_n1[8];       /* ... restricted to groups that held 1 instr */
    int      rd_bw;           /* block-op read bandwidth, bytes/cycle (8 =
                                 the documented 2 words from 4 banks)      */
    int      wr_bw;           /* block-op write bandwidth, bytes/cycle (4 =
                                 the documented single write port)         */
    int      int_lanes;       /* integer MAC lanes for the Q8_K dot of QGEMM
                                 (0 = same as mac_lanes)                   */
    int      act_buf;         /* QGEMM activation buffer, bytes: a row's
                                 activations that fit are read once per
                                 instruction (0 = streamed every row)      */
    /* Network timing (T4_T9K_NET=1).  Packets between the t4 processes of
     * a network carry the cycle their sender was ready, so every node runs
     * on one timeline: a transfer finishes at max(sender ready, receiver
     * ready) + its bytes at the link rate + one hop, and a receiver that
     * was waiting has the wait added to its cycles. */
    int      net_on;
    int      link_mbps;       /* link payload rate, MB/s (DS-link ~9)      */
    int      core_mhz;        /* core clock, MHz, to turn bytes into cycles */
    int      link_lat;        /* cycles per packet hop (header + router)   */
    uint64_t net_wait;        /* cycles spent waiting for link transfers   */
    uint64_t link_free[4];    /* cycle each outgoing link is next free     */
    uint64_t link_in_free[4]; /* NET=2: cycle each incoming link is free   */
    /* Threaded networks (-st): t9k is per thread, so every node has its own
     * clock; node is its id there and -1 otherwise.  The VCP times its own
     * packets: with net_defer set, a received packet does not stall the CPU
     * (its arrival cycle is left in net_arrival) and an outgoing one leaves
     * no earlier than net_not_before instead of the CPU's present cycle. */
    int      node;
    int      net_defer;
    uint64_t net_not_before;
    uint64_t net_arrival;
    uint64_t net_start;       /* when that packet started arriving          */
    int      c104_lat;        /* cycles a C104 adds to route a packet       */
    uint64_t link_in_last[4]; /* ready cycle of the latest packet queued    */
    uint64_t link_out_last[4];
    int      net_ooo;         /* the send being made is out of host order   */
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

#ifndef T4_THREAD_LOCAL
#define T4_THREAD_LOCAL __thread
#endif
extern T4_THREAD_LOCAL t9k_state t9k;
extern t9k_state t9k_proto;     /* the configured model, for node threads */
extern T4_THREAD_LOCAL uint32_t tvx_src_fmt;    /* VSETFMT position format (p.c) */
extern T4_THREAD_LOCAL uint32_t quant_fmt;      /* QSETFMT ggml type id (p.c)    */
extern T4_THREAD_LOCAL uint32_t m2dLength;      /* move2dinit row count (p.c)    */

/* Apply the documented T9000 defaults, then any T4_T9K_* overrides. */
static inline void t9k_init(void)
{
    t9k.g_produce   = -1;
    t9k.g_consume   = 99;   /* T9K_NO_STAGE */
    t9k.prev_produce = -1;

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
        { "T4_T9K_HRMCLASS",   0,  offsetof(t9k_state, hrm_class)       },
        { "T4_T9K_NOSTALLS",   0,  offsetof(t9k_state, stalls_off)      },
        { "T4_T9K_WHY",        0,  offsetof(t9k_state, why_on)          },
        { "T4_T9K_MACLANES",   1,  offsetof(t9k_state, mac_lanes)       },
        { "T4_T9K_UNITS",      1,  offsetof(t9k_state, n_units)         },
        { "T4_T9K_ACCUM",      1,  offsetof(t9k_state, n_acc)           },
        { "T4_T9K_RDBW",       8,  offsetof(t9k_state, rd_bw)           },
        { "T4_T9K_WRBW",       4,  offsetof(t9k_state, wr_bw)           },
        { "T4_T9K_INTLANES",   0,  offsetof(t9k_state, int_lanes)       },
        { "T4_T9K_ACTBUF",     0,  offsetof(t9k_state, act_buf)         },
        { "T4_T9K_NET",        0,  offsetof(t9k_state, net_on)          },
        { "T4_T9K_LINKMBPS",   9,  offsetof(t9k_state, link_mbps)       },
        { "T4_T9K_MHZ",        50, offsetof(t9k_state, core_mhz)        },
        { "T4_T9K_LINKLAT",    20, offsetof(t9k_state, link_lat)        },
        { "T4_T9K_C104LAT",    20, offsetof(t9k_state, c104_lat)        },
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
            if (prof[1] && prof[2] && (prof[3] == '2' || prof[3] == '3')) {
                if (!getenv("T4_T9K_MACLANES")) t9k.mac_lanes = 8;
                if (!getenv("T4_T9K_RDBW"))     t9k.rd_bw = 16;
                if (!getenv("T4_T9K_WRBW"))     t9k.wr_bw = 8;
                fprintf(stderr,
                    "[T9000] profile 'tvx2': + 8 MAC lanes, 64-bit "
                    "read/write paths (16/8 B per cycle)\n");
            }
            /* T4_T9K_PROFILE=tvx3: tvx2 plus what the integer-activation
             * path (VQ8K/VQ80, VSETFMT 3) is for - a 16 KB QGEMM activation
             * buffer and 64 integer MAC lanes.  On DeepSeek-V4.1-Flash that
             * takes a layer from 104.6M to 11.3M cycles, within 10% of the
             * 16 B/cycle weight-read floor (tests/bench/layer.sh). */
            if (prof[1] && prof[2] && prof[3] == '3') {
                if (!getenv("T4_T9K_ACTBUF"))   t9k.act_buf = 16384;
                if (!getenv("T4_T9K_INTLANES")) t9k.int_lanes = 64;
                fprintf(stderr,
                    "[T9000] profile 'tvx3': + 16 KB QGEMM activation "
                    "buffer, 64 integer MAC lanes\n");
            }
        }
    }
    if (t9k.mac_lanes < 1) t9k.mac_lanes = 1;
    if (t9k.int_lanes < 1) t9k.int_lanes = t9k.mac_lanes;
    if (t9k.n_units < 1) t9k.n_units = 1;
    if (t9k.n_units > T9K_MAX_UNITS) t9k.n_units = T9K_MAX_UNITS;
    if (t9k.n_acc < 1) t9k.n_acc = 1;
    if (t9k.n_acc > T9K_MAX_ACC) t9k.n_acc = T9K_MAX_ACC;
    t9k.node = -1;
    t9k_proto = t9k;
}

/* A node thread of a threaded network: its own copy of the configured
 * model, so its clock counts its own instructions only. */
static inline void t9k_thread_init(int node)
{
    if (!t9k_proto.enabled)
        return;
    t9k = t9k_proto;
    t9k.node = node;
}

/* HRM Table 6.4, "Stages in which instructions operate".  Stage numbering is
 * 0-based over the execute stages: 0 Local, 1 Address, 2 Read, 3 ALU, 4 Write.
 *
 *   instruction                       consumed   produced
 *   ldc, ldl, ldlp, mint                  -          0
 *   ldnlp, all subscript instructions     1          1
 *   ldnl, load16, lb                      1          2
 *   all ALU and FPU instructions          3          3
 *   cj                                    4          -
 *   all store instructions                4          -
 *
 * The table is a stall model, not just a stage assignment: "if a value is
 * produced in stage n and consumed in stage m by the following group, n - m
 * extra cycles must be allowed".  That is where the real inter-group cost
 * lives - an ALU result feeding an address calculation in the next group
 * costs 2 cycles, an ldnl feeding one costs 1 - and modelling only per-stage
 * occupancy misses all of it.
 *
 * Note the table does not name adc, eqc, ajw, j or call.  It gives specific
 * primaries and then two catch-alls ("all subscript instructions", "all ALU
 * and FPU instructions"), so those are a gap rather than a contradiction; see
 * the classification note above.  Stores and cj produce nothing here, so no
 * penalty is attributed to them as producers - the HRM separately states a
 * store-to-load penalty which this does not yet model. */
#define T9K_NO_STAGE 99

static inline int t9k_produce_stage(int kind)
{
    switch (kind) {
    case T9K_LOCAL:     return 0;   /* ldc, ldl, ldlp, mint  */
    case T9K_ADDR:      return 1;   /* ldnlp, subscripts     */
    case T9K_LOCAL_FAR: return 2;   /* address calc + read   */
    case T9K_NONLOCAL:  return 2;   /* ldnl, load16, lb      */
    case T9K_ALU:
    case T9K_SLOW:      return 3;   /* all ALU and FPU       */
    default:            return -1;  /* stores, cj: nothing   */
    }
}

static inline int t9k_consume_stage(int kind)
{
    switch (kind) {
    case T9K_ADDR:
    case T9K_LOCAL_FAR:
    case T9K_NONLOCAL:  return 1;
    case T9K_ALU:
    case T9K_SLOW:      return 3;
    case T9K_STORE:
    case T9K_WRJMP:     return 4;
    default:            return T9K_NO_STAGE;   /* ldc/ldl/ldlp consume none */
    }
}

static inline void t9k_close_group_why(int why)
{
    if (t9k.n) {
        /* Inter-group stall: this group's earliest consumer against the
         * previous group's latest producer. */
        if (!t9k.stalls_off && t9k.prev_produce >= 0 &&
            t9k.g_consume != T9K_NO_STAGE) {
            int d = t9k.prev_produce - t9k.g_consume;
            if (d > 0) {
                t9k.cycle += (uint64_t)d;
                t9k.stall_cycles += (uint64_t)d;
            }
        }
        t9k.prev_produce = t9k.g_produce;
    }
    t9k.g_produce = -1;
    t9k.g_consume = T9K_NO_STAGE;
    if (t9k.n) {
        t9k.groups++;
        t9k.cycle++;
        if (t9k.n <= 8)
            t9k.hist[t9k.n]++;
        if (t9k.why_on) {
            t9k.why[why & 7]++;
            if (t9k.n == 1)
                t9k.why_n1[why & 7]++;
        }
    }
    t9k.n = t9k.n_bytes = t9k.n_local = t9k.n_addr = 0;
    t9k.n_nonlocal = t9k.n_alu = t9k.n_wrjmp = 0;
}

static inline void t9k_close_group(void)
{
    t9k_close_group_why(T9KW_OTHER);
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
        t9k_close_group_why(T9KW_SLOW);   /* cannot share a group */
        t9k.n = 1;
        t9k.n_bytes = bytes;
        t9k_close_group_why(T9KW_SLOW);
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
    if (t9k.n >= 8)
        t9k_close_group_why(T9KW_COUNT8);
    else if (t9k.n_bytes + bytes > T9K_GROUP_BYTES)
        t9k_close_group_why(T9KW_BYTES);
    else if (t9k.n_local + d_local > 2)
        t9k_close_group_why(T9KW_S_LOCAL);
    else if (t9k.n_addr + d_addr > 2)
        t9k_close_group_why(T9KW_S_ADDR);
    else if (t9k.n_nonlocal + d_nonlocal > 2)
        t9k_close_group_why(T9KW_S_NONLOC);
    else if (t9k.n_alu + d_alu > 1 || t9k.n_wrjmp + d_wrjmp > 1)
        t9k_close_group_why(T9KW_S_ALU);

    {
        int pr = t9k_produce_stage(kind), co = t9k_consume_stage(kind);
        if (pr > t9k.g_produce) t9k.g_produce = pr;
        if (co < t9k.g_consume) t9k.g_consume = co;
    }
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
        t9k_close_group_why(kind == T9K_STORE ? T9KW_STORE : T9KW_JUMP);
}

/* The pre-2026-08-11 classification, derived from the HRM rather than from the
 * RTL's classify_uop.  Kept switchable (T4_T9K_HRMCLASS=1) because every
 * measurement taken before the correction used it. */
static inline int t9k_class_primary_hrm(int op, uint32_t oreg)
{
    switch (op) {
    case 0x2: case 0x6: return T9K_NONE;
    case 0x7: case 0x1:
        return (int)oreg < t9k.ws_cache ? T9K_LOCAL : T9K_LOCAL_FAR;
    case 0x5:           return T9K_ADDR;
    case 0x3:           return T9K_NONLOCAL;
    case 0x4: case 0x8: case 0xC: return T9K_ALU;
    case 0xD: case 0xE: return T9K_STORE;
    case 0x0: case 0xA: case 0x9: return T9K_WRJMP;
    case 0xB: return t9k.ajw_stage2 ? T9K_ADDR : T9K_WRJMP;
    default:            return T9K_ALU;
    }
}

/* Classify a primary (high-nibble) opcode.  `oreg` is the folded operand,
 * needed to tell a workspace-cache reference from a computed one. */
static inline int t9k_class_primary(int op, uint32_t oreg)
{
    if (t9k.hrm_class)
        return t9k_class_primary_hrm(op, oreg);

    /* Classification below follows the RTL's classify_uop, which disagrees
     * with the reading of the HRM this model started from and is what actually
     * governs grouping on the core:
     *
     *     U_LDL, U_LDC, U_LDLP          -> R_LOCAL  (stage 1, cap 2)
     *     U_LDNLP, U_ADC, U_EQC, U_AJW  -> R_ADDR   (stage 2, cap 2)
     *     U_LDNL                        -> R_READ   (stage 3)
     *
     * Measured per-op cost on the core confirms it.  ldlp 1.02, eqc 1.20,
     * adc 1.25, ldc 1.29, ldnlp 1.65 cycles/op - then a 5x cliff to the real
     * stage-4 ALU operations: and 6.29, or 6.74, gt 7.00, sum 7.24, and prod
     * at 45.66.
     *
     * Charging ldc/adc/eqc to stage 4, as this model did, inflated the stage-4
     * population by roughly 70% over a kernel corpus (25.1% of instructions
     * against an actual 7.0%) and pointed optimisation work at the three
     * cheapest instructions in the machine.  It also made `ldc` look like an
     * immovable 40-45% of the bottleneck when it is a stage-1 stack push.
     *
     * Note this also settles the ajw question the comment below agonised over:
     * the RTL puts it in R_ADDR, which is what t9k.ajw_stage2 selected as the
     * non-default.  T4_T9K_HRMCLASS=1 restores the old classification so the
     * two can be compared.  */
    switch (op) {
    case 0x2: case 0x6: return T9K_NONE;      /* pfix, nfix          */
    case 0x4:           return T9K_LOCAL;     /* ldc: a stack push   */
    case 0x7: case 0x1:                       /* ldl, ldlp           */
        return (int)oreg < t9k.ws_cache ? T9K_LOCAL : T9K_LOCAL_FAR;
    case 0x5: case 0x8: case 0xC: case 0xB:   /* ldnlp, adc, eqc, ajw */
                        return T9K_ADDR;
    case 0x3:           return T9K_NONLOCAL;  /* ldnl                */
    case 0xD: case 0xE: return T9K_STORE;     /* stl, stnl           */
    case 0x0: case 0xA: case 0x9:
        return T9K_WRJMP;                     /* j, cj, call         */
    /* ajw: settled.  The RTL's classify_uop puts it in R_ADDR, and HRM
     * Table 6.4 gives it no ALU/write role either.  This was previously the
     * model's largest documented unknown (T4_T9K_AJWSTAGE2), worth 8.2% of the
     * estimate on a kernel corpus. */
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
    case 0x1F7:                                 /* VQ8K: 256-elem blocks   */
        return veclen / 256u;
    case 0x1F8:                                 /* VQ80: 32-elem blocks    */
        return veclen / 32u;
    case 0x1F9:                                 /* VCVT: VSETLEN elements  */
        return veclen;
    default:
        return 1;                               /* fixed-size instructions */
    }
}

/* GGUF block geometry - elements and bytes per block - for the quantised
 * dot ops, keyed by ggml type id.  These sizes are fixed by the GGUF format
 * and must agree with tvx_gguf_block_bytes() in tvx_gguf_decode.h; that
 * header cannot be included here because main.c includes this one without
 * the decoders.  Returns 0 for a format QDOT does not decode. */
static inline int t9k_qfmt_geom(uint32_t fmt, uint32_t *ne, uint32_t *bytes)
{
    switch (fmt) {
    case 0:  *ne = 32;  *bytes = 128; return 1;    /* f32, in blocks of 32 */
    case 1:  *ne = 32;  *bytes = 64;  return 1;    /* f16     */
    case 30: *ne = 32;  *bytes = 64;  return 1;    /* bf16    */
    case 2:  *ne = 32;  *bytes = 18;  return 1;    /* q4_0    */
    case 3:  *ne = 32;  *bytes = 20;  return 1;    /* q4_1    */
    case 6:  *ne = 32;  *bytes = 22;  return 1;    /* q5_0    */
    case 7:  *ne = 32;  *bytes = 24;  return 1;    /* q5_1    */
    case 8:  *ne = 32;  *bytes = 34;  return 1;    /* q8_0    */
    case 10: *ne = 256; *bytes = 84;  return 1;    /* q2_K    */
    case 11: *ne = 256; *bytes = 110; return 1;    /* q3_K    */
    case 12: *ne = 256; *bytes = 144; return 1;    /* q4_K    */
    case 13: *ne = 256; *bytes = 176; return 1;    /* q5_K    */
    case 14: *ne = 256; *bytes = 210; return 1;    /* q6_K    */
    case 16: *ne = 256; *bytes = 66;  return 1;    /* iq2_xxs */
    case 17: *ne = 256; *bytes = 74;  return 1;    /* iq2_xs  */
    case 18: *ne = 256; *bytes = 98;  return 1;    /* iq3_xxs */
    case 19: *ne = 256; *bytes = 50;  return 1;    /* iq1_s   */
    case 20: *ne = 32;  *bytes = 18;  return 1;    /* iq4_nl  */
    case 21: *ne = 256; *bytes = 110; return 1;    /* iq3_s   */
    case 22: *ne = 256; *bytes = 82;  return 1;    /* iq2_s   */
    case 23: *ne = 256; *bytes = 136; return 1;    /* iq4_xs  */
    case 29: *ne = 256; *bytes = 56;  return 1;    /* iq1_m   */
    case 39: *ne = 32;  *bytes = 17;  return 1;    /* mxfp4   */
    default: return 0;
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
 * joint-sorted), so its stream is weights + index pairs only.
 *
 * *rd_once is read once per instruction rather than per unit, and *int_mac
 * puts the MACs on the integer lanes (T4_T9K_INTLANES) instead of the FP
 * ones: both are for QGEMM's activation buffer and its Q8_K dot. */
static inline int t9k_opr_bw(int oreg, uint32_t creg, uint32_t *mac,
                             uint32_t *rd, uint32_t *wr, uint64_t *rd_once,
                             int *int_mac)
{
    *rd_once = 0;
    *int_mac = 0;
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
    case 0x1DA: case 0x1DB: case 0x1DC:         /* IQ1M/IQ2XXS/IQ3XXSDOT   */
    case 0x1DE: {                               /* QDOT, f32 acts          */
        /* One unit is one *format* block - 256 elements for the K- and
         * i-quants, not the 32 of MXFP4 - and the f32 activations it
         * consumes outweigh the weights several times over. */
        uint32_t ne, bytes;
        uint32_t fmt = oreg == 0x1DA ? 29u : oreg == 0x1DB ? 16u
                     : oreg == 0x1DC ? 18u : quant_fmt;
        if (!t9k_qfmt_geom(fmt, &ne, &bytes))
            return 0;
        *mac = ne; *rd = bytes + 4u * ne; *wr = 0; return 1;
    }
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
    case 0x1EF: {                               /* QGEMM, per weight block */
        /* block geometry from the latched format (QSETFMT), activation
         * bytes from VSETFMT: f32, bf16, or VSETFMT 3 = quantised - Q8_K
         * (292 bytes per 256) for the K- and i-quants, Q8_0 (34 per 32) for
         * the rest - whose dot runs on the integer lanes.  A row of CReg
         * blocks whose activations fit T4_T9K_ACTBUF is read once for all
         * rows, leaving only the weights to stream. */
        uint32_t ne, bytes, act;
        if (!t9k_qfmt_geom(quant_fmt, &ne, &bytes))
            return 0;
        if (tvx_src_fmt == 3u) {
            act = ne == 256u ? 292u : 34u;
            *int_mac = 1;
        } else {
            act = ne * (tvx_src_fmt == 1u ? 2u : 4u);
        }
        *mac = ne;
        *wr = 1;
        if (t9k.act_buf > 0
            && (uint64_t)act * creg <= (uint64_t)t9k.act_buf) {
            *rd = bytes;
            *rd_once = (uint64_t)act * creg;
        } else {
            *rd = bytes + act;
        }
        return 1;
    }
    case 0x1F7:                                 /* VQ8K, per 256-elem block */
    case 0x1F8:                                 /* VQ80, per 32-elem block  */
        /* the amax pass runs alongside the stream; one scale-and-round per
         * element, then 292 bytes of block_q8_K or 34 of block_q8_0 out */
        *mac = oreg == 0x1F7 ? 256u : 32u;
        *rd = *mac * (tvx_src_fmt == 1u || tvx_src_fmt == 2u ? 2u : 4u);
        *wr = oreg == 0x1F7 ? 292u : 34u;
        return 1;
    case 0x1F9:                                 /* VCVT, per element       */
        *mac = 1;
        *rd = tvx_src_fmt == 1u || tvx_src_fmt == 2u ? 2u : 4u;
        *wr = creg == 1u || creg == 2u ? 2u : 4u;
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
        if (oreg == 0x1FA) {                  /* VXFER: creg = entries   */
            /* One in/out's process overhead for the whole group, plus the
             * three descriptor words read per entry handed to the VCP. */
            uint64_t n = creg ? creg : 1;
            if (n > 1000000u) n = 1000000u;
            *latency = 20 + 3 * (int)n;
            return T9K_SLOW;
        }
        if (oreg >= 0x1C0 && oreg <= 0x1F9) { /* TVX + TVX-GFX           */
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
            /* nominal only, for a QDOT format with no known geometry -
             * t9k_opr_bw() supersedes this for every decodable one */
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
            case 0x1F7: per = 64; break;             /* VQ8K    /block    */
            case 0x1F8: per = 8;  break;             /* VQ80    /block    */
            case 0x1F9: per = 1;  break;             /* VCVT    /element  */
            default: return T9K_ALU;                 /* conversions       */
            }
            {
                uint64_t n = t9k_opr_units(oreg, creg, veclen);
                uint32_t mac, rdb, wrb;
                uint64_t t, rd1;
                int imac;

                if (n == 0) n = 1;
                if (t9k_opr_bw(oreg, creg, &mac, &rdb, &wrb, &rd1, &imac)) {
                    /* bottleneck of MAC array, read port, write port */
                    uint64_t lanes = (uint64_t)(imac ? t9k.int_lanes
                                                     : t9k.mac_lanes);
                    uint64_t tm = ((uint64_t)mac * n + lanes - 1) / lanes;
                    uint64_t tr = ((uint64_t)rdb * n + rd1
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

/* Classify a ProfileCode-style opcode: primaries are stored as (op << 4),
 * T9000 secondaries as 0x100 + oreg.  This is the encoding the emulator's
 * combined[] table uses for the two halves of a fused pair. */
static inline int t9k_class_profile(int code, uint32_t oreg, uint32_t creg,
                                    uint32_t veclen, int *latency)
{
    *latency = 1;
    if (code < 0x100)
        return t9k_class_primary(code >> 4, oreg);
    return t9k_class_opr(code - 0x100, creg, veclen, latency);
}

/* Feed a fused instruction pair as the two instructions it really is.
 *
 * The emulator fuses common adjacent pairs into a single Icache slot with a
 * pseudo-opcode in 0x100..0x115 (combined[] in p.c) - `stl;ldl`, `ldl;adc`,
 * `eqc;cj`, `ldc;ldl`, `ldl;ldl` and so on.  Those pseudo-opcodes are not
 * T9000 secondaries, so t9k_class_opr fell through to its default and charged
 * each fused pair exactly one stage-4 ALU operation.
 *
 * That is not a small distortion.  It made `ldc;ldl` and `ldl;ldl` cost the
 * same, so moving work off the ALU stage - the whole point of the stage-4
 * bound - was invisible.  It also charged `stl;ldl` and `ajw;ret` to the ALU
 * stage, inflating the stage-4 share of group closes at the expense of the
 * store and jump stages.  Any measurement taken before this fix understates
 * ALU-pressure changes and overstates the ALU's share of the profile. */
static inline void t9k_issue_pair(int code0, uint32_t oreg0,
                                  int code1, uint32_t oreg1,
                                  uint32_t creg, uint32_t veclen, int bytes)
{
    int lat0, lat1, k0, k1, b0;

    if (!t9k.enabled)
        return;
    k0 = t9k_class_profile(code0, oreg0, creg, veclen, &lat0);
    k1 = t9k_class_profile(code1, oreg1, creg, veclen, &lat1);
    /* The fetcher pulled `bytes` for the pair; the second half of every
     * fused pair is a single byte in practice, so charge it one and give the
     * rest (any prefix chain included) to the first. */
    b0 = bytes > 1 ? bytes - 1 : 1;
    t9k_issue(k0, lat0, b0);
    t9k_issue(k1, lat1, bytes - b0 > 0 ? bytes - b0 : 1);
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
    return (t9k.groups > fetch ? t9k.groups : fetch) + t9k.stall
           + t9k.net_wait;
}

/* ---- network timing (T4_T9K_NET=1 or 2) -----------------------------
 * NET=1 times a sequential program: a transfer starts when both ends are
 * ready, and the outputting node waits for it.  Only the receiver is exact
 * there, since only it knows when it was ready; the sender is charged as
 * if the receiver were.
 *
 * NET=2 times a program whose link I/O runs in PAR with its computation,
 * as a T9000 program would be written: every transfer starts as soon as
 * its sender is ready and its link is free, the sender carries on while
 * the link's DMA runs, and transfers on different links overlap at both
 * ends.  A receiver waits only if the data has not arrived yet. */
static inline uint64_t t9k_link_xfer(uint32_t nbytes)
{
    if (t9k.link_mbps < 1)
        t9k.link_mbps = 1;
    return ((uint64_t)nbytes * (uint64_t)t9k.core_mhz
            + (uint64_t)t9k.link_mbps - 1) / (uint64_t)t9k.link_mbps;
}

static inline void t9k_advance_to(uint64_t t)
{
    uint64_t now = t9k_cycles_so_far();

    if (t > now)
        t9k.net_wait += t - now;
}

/* The cycle a packet about to leave on `link` is ready: now - or, for the
 * VCP, net_not_before - or when the link finishes the packet before it.
 * The packet carries this. */
/* The VCP's virtual links become ready to send in the order their packets
 * and acknowledges arrived, which through a switch is host order, not cycle
 * order.  A packet the VCP has ready before the latest one it sent went
 * first on the wire: it does not queue behind it, but keeps the link busy
 * for its bytes (net_ooo carries that to t9k_net_sent). */
static inline uint64_t t9k_net_ready(int link)
{
    uint64_t now = t9k.net_defer ? t9k.net_not_before : t9k_cycles_so_far();

    t9k.net_ooo = t9k.net_defer && now < t9k.link_out_last[link & 3];
    if (t9k.net_ooo)
        return now;
    t9k.link_out_last[link & 3] = now;
    return now > t9k.link_free[link & 3] ? now : t9k.link_free[link & 3];
}

static inline void t9k_net_sent(int link, uint64_t ready, uint32_t nbytes)
{
    if (t9k.net_ooo) {
        t9k.net_ooo = 0;
        t9k.link_free[link & 3] += t9k_link_xfer(nbytes);
        return;
    }
    t9k.link_free[link & 3] = ready + t9k_link_xfer(nbytes);
    if (t9k.net_on == 1 && !t9k.net_defer)
        t9k_advance_to(t9k.link_free[link & 3]);
}

/* A packet that left its sender at `ready` arrives once the link is free
 * and its bytes and one hop have passed.  An `in` waits for that; the VCP
 * (net_defer) only notes it in net_arrival - it is hardware beside the CPU,
 * and wakes a process itself when a message is done. */
static inline void t9k_net_received(int link, uint64_t ready, uint32_t nbytes)
{
    uint64_t start;

    if (t9k.net_on == 1 && !t9k.net_defer) {
        uint64_t now = t9k_cycles_so_far();
        start = ready > now ? ready : now;
    } else if (ready >= t9k.link_in_last[link & 3]) {
        t9k.link_in_last[link & 3] = ready;
        start = ready > t9k.link_in_free[link & 3] ? ready
                                                   : t9k.link_in_free[link & 3];
        t9k.link_in_free[link & 3] = start + t9k_link_xfer(nbytes);
    } else {
        /* Stamped before a packet already queued here: through a switch,
         * from another sender, it only reached us later in host time.  On
         * the wire it went first, so it does not wait behind that packet;
         * the link is busy for its bytes all the same. */
        start = ready;
        t9k.link_in_free[link & 3] += t9k_link_xfer(nbytes);
    }
    if (t9k.net_defer) {
        t9k.net_start = start;
        t9k.net_arrival = start + t9k_link_xfer(nbytes) + (uint64_t)t9k.link_lat;
        return;
    }
    t9k_advance_to(start + t9k_link_xfer(nbytes) + (uint64_t)t9k.link_lat);
}

/* An IMS C104 forwarding a packet that left its sender at `ready` (HRM 7.2,
 * wormhole routing): the header reaches the switch a hop later, takes
 * c104_lat to route, and the packet leaves once the output link is free.
 * The bytes follow the header straight through, so a path costs its hops
 * plus one transfer, not one transfer per hop.  Runs on a router thread,
 * hence the configured model rather than a node's.  *last is when the
 * latest packet queued on the output reached the switch: one that reached
 * it earlier came late only in host time and is not queued behind it (see
 * t9k_net_received).
 * Returns the packet's new stamp: the cycle it is ready on the output. */
static inline uint64_t t9k_c104_hop(uint64_t ready, uint32_t nbytes,
                                    uint64_t *out_free, uint64_t *last)
{
    const t9k_state *c = &t9k_proto;
    uint64_t mbps = c->link_mbps < 1 ? 1 : (uint64_t)c->link_mbps;
    uint64_t t = ready + (uint64_t)c->link_lat + (uint64_t)c->c104_lat;
    uint64_t x = ((uint64_t)nbytes * (uint64_t)c->core_mhz + mbps - 1) / mbps;

    if (t >= *last) {
        *last = t;
        if (t < *out_free)
            t = *out_free;
        *out_free = t + x;
    } else
        *out_free += x;
    return t;
}

/* One line of progress: what the machine has cost by the time the guest
 * printed `tag`.  Used to attribute cycles to phases of a boot rather than
 * only reporting a single total at exit. */
static inline void t9k_mark(const char *tag)
{
    char who[16] = "";

    if (!t9k.enabled)
        return;
    if (t9k.node >= 0)          /* threaded network: whose line it is */
        snprintf(who, sizeof who, "n%d ", t9k.node);
    fprintf(stderr, "[T9000-MARK] %s%-24s instrs=%llu bytes=%llu groups=%llu "
                    "stall=%llu cycles=%llu (%.1f ms @50MHz)\n",
            who, tag,
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
    /* Issue and fetch are independent lower bounds; whichever binds, the
     * produce/consume stalls of HRM Table 6.4 sit on top of it, because a
     * group waiting on the previous group's result is not issuing regardless
     * of which bound was limiting it. */
    cycles = (t9k.groups > fetch ? t9k.groups : fetch)
             + t9k.stall + t9k.stall_cycles + t9k.net_wait;
    if (!cycles)
        return;
    if (t9k.node >= 0)
        fprintf(stderr, "\n[T9000] node %d:", t9k.node);
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
    if (!t9k.stalls_off)
        fprintf(stderr, "[T9000]   produce/consume stall: %llu (HRM Table 6.4)\n",
                (unsigned long long)t9k.stall_cycles);
    if (t9k.net_on)
        fprintf(stderr, "[T9000]   link wait (network) : %llu\n",
                (unsigned long long)t9k.net_wait);
    fprintf(stderr, "[T9000]   total cycles        : %llu\n", (unsigned long long)cycles);
    fprintf(stderr, "[T9000]   instructions/cycle  : %.3f\n",
            (double)t9k.instrs / (double)cycles);
    fprintf(stderr, "[T9000]   group size histogram:");
    for (i = 1; i <= 8; i++)
        fprintf(stderr, " %d:%llu", i, (unsigned long long)t9k.hist[i]);
    fprintf(stderr, "\n[T9000]   at 50 MHz that is    : %.3f ms\n",
            (double)cycles / 50000.0);
    if (t9k.why_on) {
        static const char *const wn[8] = {
            "other", "store (stl/stnl)", "jump (j/cj/call/ret/ajw)",
            "multi-cycle op", "stage1 local x2", "stage2 addr x2",
            "stage3 nonlocal x2", "stage4 ALU x1"
        };
        uint64_t tot = 0;
        int k;
        for (k = 0; k < 8; k++) tot += t9k.why[k];
        fprintf(stderr, "[T9000]   --- why groups closed (%llu groups) ---\n",
                (unsigned long long)tot);
        for (k = 0; k < 8; k++) {
            if (!t9k.why[k]) continue;
            fprintf(stderr, "[T9000]   %-26s %9llu (%5.1f%%)  of which size-1: %llu\n",
                    wn[k], (unsigned long long)t9k.why[k],
                    tot ? 100.0 * (double)t9k.why[k] / (double)tot : 0.0,
                    (unsigned long long)t9k.why_n1[k]);
        }
    }
}

#endif /* T9000_GROUP_H */
