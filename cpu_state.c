#include "cpu_state.h"
#include "t9000_vcp.h"
#include "t9000_ctl.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

T4_THREAD_LOCAL cpu_t *t4cpu;
cpu_t t4_cpu0;

extern uint32_t MemSize, MemWordMask, MemByteMask, CoreSize, CLineTagsSize;
extern uint32_t MemStart;
void init_processor(void);

void cpu_bind(cpu_t *c)
{
        t4cpu = c;
}

static int cpu_alloc_caches(cpu_t *c)
{
        c->s_Icache = calloc(MAX_ICACHE + 1, sizeof(InstrSlot));
        c->s_Acache = calloc(MAX_ICACHE, sizeof(ArgSlot));
#ifndef EMUDEBUG
        c->s_fast_blocks = calloc(T4_FAST_BLOCK_SIZE, sizeof(t4_fast_block_t));
        if (!c->s_fast_blocks)
                return -1;
#endif
        if (!c->s_Icache || !c->s_Acache)
                return -1;
        return 0;
}

cpu_t *cpu_create(int nid, int srv)
{
        cpu_t *c = calloc(1, sizeof(*c));
        if (!c)
                return NULL;
        c->cpu_nodeid = nid;
        c->cpu_serve = srv;
        if (cpu_setup_ram(c) < 0) {
                cpu_destroy(c);
                return NULL;
        }
        return c;
}

void cpu_destroy(cpu_t *c)
{
        if (!c)
                return;
        free(c->s_Icache);
        free(c->s_Acache);
#ifndef EMUDEBUG
        free(c->s_fast_blocks);
#endif
        free(c->s_core);
        free(c->s_mem);
        free(c->s_CLineTags);
        t9k_vcp_destroy(c);
        t9k_ctl_destroy(c);
        free(c->s_loopback);
        if (c != &t4_cpu0)
                free(c);
}

int cpu_setup_ram(cpu_t *c)
{
        c->MemSize = MemSize;
        c->MemWordMask = MemWordMask;
        c->MemByteMask = MemByteMask;
        c->s_TimerEnableHi = 1;
        c->s_TimerEnableLo = 1;
        c->s_ThReg = 0x80000000u;
        c->s_delayCount1 = 5;
        c->s_speed_multiplier = 1;
        c->s_FPtrReg[0] = c->s_FPtrReg[1] = 0x80000000u;
        c->s_BPtrReg[0] = c->s_BPtrReg[1] = 0x80000000u;
        c->s_TPtrLoc[0] = c->s_TPtrLoc[1] = 0x80000000u;
        c->s_vcp_span = 0x20u;          /* event channels only until used */
        c->s_sh.wdesc = 0x80000000u;
        if (!c->s_Icache && cpu_alloc_caches(c) < 0)
                return -1;
        if (!c->s_core)
                c->s_core = calloc(1, CoreSize);
        if (!c->s_mem)
                c->s_mem = calloc(1, MemSize);
        if (!c->s_CLineTags)
                c->s_CLineTags = calloc(1, CLineTagsSize ? CLineTagsSize : 1);
        if (!c->s_core || !c->s_mem || !c->s_CLineTags)
                return -1;
        return 0;
}

void cpu_once(void)
{
}

void cpu_init(cpu_t *c)
{
        cpu_t *prev = t4cpu;
        cpu_bind(c);
        init_processor();
        cpu_bind(prev ? prev : c);
}
