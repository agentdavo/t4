#ifndef T4_CPU_STATE_H
#define T4_CPU_STATE_H

#include "processor.h"
#include "cpu_types.h"
#include <stdatomic.h>
#include <sys/time.h>

#ifndef T4_THREAD_LOCAL
#if defined(_MSC_VER)
#define T4_THREAD_LOCAL __declspec(thread)
#else
#define T4_THREAD_LOCAL __thread
#endif
#endif

/* T9000 shadow registers (ISM 13.3): the state of the interrupted
 * low-priority process.  wdesc is NotProcess.p when there is none.  The FP
 * stack lives in s_F[ABC]RegSave. */
typedef struct t9k_shadow {
        uint32_t wdesc, iptr, a, b, c, e, x, status, eptr, th;
        uint32_t bm[3];
        uint32_t region[4], pstate, wdescstub, sup_iptr;
        int      is_p;
        uint32_t wl, wu;
        int      fp_error, tsdis;
} t9k_shadow_t;

struct t9k_vcp;
struct t9k_ctl;

typedef struct cpu_t {
        uint32_t s_IPtr, s_WPtr, s_AReg, s_BReg, s_CReg, s_OReg;
        uint32_t s_XReg, s_WdescReg, s_DReg, s_EReg;
        uint32_t s_ProcPriority, s_STATUSReg;
        uint32_t s_FPtrReg[2], s_BPtrReg[2];
        uint32_t s_TNextReg[2], s_TPtrLoc[2];
        _Atomic uint32_t s_ClockReg[2];
        _Atomic int      s_timeslice;
        _Atomic int      s_Timers;
        int32_t  node_quit;
        int      cpu_nodeid;
        int      cpu_serve;

        uint32_t s_t4_overflow, s_t4_carry, s_t4_normlen, s_t4_carry64;

        REAL s_FAReg, s_FBReg, s_FCReg, s_FARegSave, s_FBRegSave, s_FCRegSave;
        int  s_FP_Error, s_RoundingMode, s_ResetRounding;

        u_char  *s_mem;
        u_char  *s_core;
        u_char  *s_CLineTags;
        uint32_t MemSize, MemWordMask, MemByteMask;

        LinkIface s_Link[4];

        InstrSlot *s_Icache;
        ArgSlot   *s_Acache;
#ifndef EMUDEBUG
        t4_fast_block_t *s_fast_blocks;
#endif
        void    *dcache;
        void    *idle;

        struct timeval s_LastTOD;
        uint32_t slow_counter;

        u_char s_Instruction, s_Icode, s_Idata;
        int  s_TimerEnableHi, s_TimerEnableLo;
        int  s_EnableJ0BreakFlag;
        int  s_TimesliceDisabled, s_IntsDisabled;
        uint32_t s_ThReg;
        int s_loop, s_count1, s_count2, s_count3;
        int s_delayCount1, s_speed_multiplier;
        uint64_t s_heartbeat_counter;   /* main thread only */

        /* T9000 external channels (t9000_vcp.c).  Channel addresses in
         * [MinEventChannel, MinEventChannel + s_vcp_span) go to the VCP;
         * see T9K_EXTCHAN(). */
        struct t9k_vcp *s_vcp;
        uint32_t s_vcp_span;
        t9k_shadow_t s_sh;
        u_char  *s_loopback;            /* single-node link loopback slots */

        /* T9000 control system (t9000_ctl.c).  s_ctl_links: a control link
         * is wired; s_ctl_halt: the control unit has stopped the CPU, which
         * start_process() turns into a return from mainloop(). */
        struct t9k_ctl *s_ctl;
        LinkIface s_CLink[2];
        int      s_ctl_links;
        int      s_ctl_halt;
        /* Link faults (HRM 13.5): s_line_err is set by the far end, bit
         * 2*link + 1 for a disconnect, + 0 for a parity error (links 4-5 =
         * CLink0-1); s_link_halted has bit n set while Link n is stopped. */
        _Atomic uint32_t s_line_err;
        uint8_t  s_link_halted;
} cpu_t;

extern T4_THREAD_LOCAL cpu_t *t4cpu;
extern cpu_t                  t4_cpu0;

cpu_t *cpu_create(int nodeid, int serve);
void   cpu_destroy(cpu_t *c);
void   cpu_bind(cpu_t *c);
void   cpu_init(cpu_t *c);
void   cpu_once(void);
int    cpu_setup_ram(cpu_t *c);

#ifndef T4_CPU_NO_MACROS
#define IPtr          (t4cpu->s_IPtr)
#define WPtr          (t4cpu->s_WPtr)
#define AReg          (t4cpu->s_AReg)
#define BReg          (t4cpu->s_BReg)
#define CReg          (t4cpu->s_CReg)
#define OReg          (t4cpu->s_OReg)
#define XReg          (t4cpu->s_XReg)
#define WdescReg      (t4cpu->s_WdescReg)
#define DReg          (t4cpu->s_DReg)
#define EReg          (t4cpu->s_EReg)
#define ProcPriority  (t4cpu->s_ProcPriority)
#define STATUSReg     (t4cpu->s_STATUSReg)
#define FPtrReg       (t4cpu->s_FPtrReg)
#define BPtrReg       (t4cpu->s_BPtrReg)
#define TNextReg      (t4cpu->s_TNextReg)
#define TPtrLoc       (t4cpu->s_TPtrLoc)
#define ClockReg      (t4cpu->s_ClockReg)
#define timeslice     (t4cpu->s_timeslice)
#define Timers        (t4cpu->s_Timers)
#define nodeid        (t4cpu->cpu_nodeid)
#define serve         (t4cpu->cpu_serve)
#define t4_overflow   (t4cpu->s_t4_overflow)
#define t4_carry      (t4cpu->s_t4_carry)
#define t4_normlen    (t4cpu->s_t4_normlen)
#define t4_carry64    (t4cpu->s_t4_carry64)
#define FAReg         (t4cpu->s_FAReg)
#define FBReg         (t4cpu->s_FBReg)
#define FCReg         (t4cpu->s_FCReg)
#define FARegSave     (t4cpu->s_FARegSave)
#define FBRegSave     (t4cpu->s_FBRegSave)
#define FCRegSave     (t4cpu->s_FCRegSave)
#define FP_Error      (t4cpu->s_FP_Error)
#define RoundingMode  (t4cpu->s_RoundingMode)
#define ResetRounding (t4cpu->s_ResetRounding)
#define mem           (t4cpu->s_mem)
#define core          (t4cpu->s_core)
#define CLineTags     (t4cpu->s_CLineTags)
#define Icache        (t4cpu->s_Icache)
#define Acache        (t4cpu->s_Acache)
#ifndef EMUDEBUG
#define fast_blocks   (t4cpu->s_fast_blocks)
#endif
#define LastTOD       (t4cpu->s_LastTOD)
#define Link          (t4cpu->s_Link)
#define TimerEnableHi (t4cpu->s_TimerEnableHi)
#define TimerEnableLo (t4cpu->s_TimerEnableLo)
#define EnableJ0BreakFlag (t4cpu->s_EnableJ0BreakFlag)
#define TimesliceDisabled (t4cpu->s_TimesliceDisabled)
#define IntsDisabled  (t4cpu->s_IntsDisabled)
#define ThReg         (t4cpu->s_ThReg)
#define delayCount1   (t4cpu->s_delayCount1)
#define speed_multiplier (t4cpu->s_speed_multiplier)
#define heartbeat_counter (t4cpu->s_heartbeat_counter)
#endif

#endif
