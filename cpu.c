/*
 * cpu.c — Clean transputer CPU dispatch loop using decode cache
 *
 * This is a standalone test of the decode cache dispatch concept.
 * It implements the core integer instructions (LDL, STL, LDC, ADC,
 * LDLP, LDNL, STNL, J, CJ, AJW, EQC, CALL, OPR) with proper
 * OReg handling and IPtr management.
 *
 * Usage: call cpu_run_cached() from mainloop instead of the original
 * fetch-decode-execute block.
 */
#include "cpu.h"
#include "decode_cache.h"
#include <stdio.h>
#include <string.h>

/* External: p.c memory access functions */
extern unsigned char byte_int(uint32_t ptr);
extern uint32_t word_int(uint32_t ptr);
extern void writeword_int(uint32_t ptr, uint32_t value);
extern void writebyte_int(uint32_t ptr, unsigned char value);

/* External: p.c globals we need */
extern uint32_t IPtr, WPtr, AReg, BReg, CReg;
extern unsigned char *mem;
extern uint32_t MemByteMask, MemWordMask;
extern int32_t quit;

/* Workspace access macros — same as p.c */
#define index(base, offset)  ((uint32_t)((base) + 4 * (int32_t)(offset)))
#define word(addr)           (*(uint32_t *)(mem + ((addr) & MemByteMask)))

/*
 * Run N instructions using the decode cache.
 * Returns the number of instructions actually executed.
 * Caller should call this in a loop with timer/display checks between batches.
 */
int cpu_run_cached(dcache_t *dc, int max_insns)
{
    int count = 0;
    uint32_t temp;

    while (count < max_insns && !quit) {
        /* Decode: lookup or fill cache */
        dcache_entry_t *d = dcache_lookup(dc, IPtr);
        if (!d)
            d = dcache_fill(dc, IPtr, mem, MemByteMask);

        int opcode = d->opcode;
        int32_t operand = d->operand;
        /* IPtr of the opcode byte (for J/CJ/CALL offset calculation) */
        uint32_t op_iptr = d->next_ip - 1;

        count++;

        switch (opcode) {
        case 0x0: /* J — unconditional jump */
            IPtr = op_iptr + 1 + operand;
            continue;  /* skip IPtr = next_ip below */

        case 0x1: /* LDLP — load local pointer */
            CReg = BReg;
            BReg = AReg;
            AReg = index(WPtr, operand);
            break;

        case 0x3: /* LDNL — load non-local */
            AReg = word(index(AReg, operand));
            break;

        case 0x4: /* LDC — load constant */
            CReg = BReg;
            BReg = AReg;
            AReg = (uint32_t)operand;
            break;

        case 0x5: /* LDNLP — load non-local pointer */
            AReg = index(AReg, operand);
            break;

        case 0x7: /* LDL — load local */
            CReg = BReg;
            BReg = AReg;
            AReg = word(index(WPtr, operand));
            break;

        case 0x8: /* ADC — add constant */
            AReg = AReg + (uint32_t)operand;
            break;

        case 0x9: /* CALL */
            /* CALL saves Areg/Breg/Creg at new workspace, sets Iptr */
            {
                uint32_t new_wptr = WPtr - 16; /* 4 words down */
                *(uint32_t *)(mem + ((new_wptr + 0)  & MemByteMask)) = op_iptr + 1; /* return addr */
                *(uint32_t *)(mem + ((new_wptr + 4)  & MemByteMask)) = AReg;
                *(uint32_t *)(mem + ((new_wptr + 8)  & MemByteMask)) = BReg;
                *(uint32_t *)(mem + ((new_wptr + 12) & MemByteMask)) = CReg;
                AReg = op_iptr + 1; /* return address in AReg for callee */
                WPtr = new_wptr;
                IPtr = op_iptr + 1 + operand;
                continue;
            }

        case 0xA: /* CJ — conditional jump */
            if (AReg == 0) {
                /* Jump taken: AReg stays 0, stack unchanged */
                IPtr = op_iptr + 1 + operand;
                continue;
            }
            /* Not taken: pop stack */
            AReg = BReg;
            BReg = CReg;
            break;

        case 0xB: /* AJW — adjust workspace */
            WPtr = index(WPtr, operand);
            break;

        case 0xC: /* EQC — equals constant */
            AReg = (AReg == (uint32_t)operand) ? 1 : 0;
            break;

        case 0xD: /* STL — store local */
            *(uint32_t *)(mem + (index(WPtr, operand) & MemByteMask)) = AReg;
            AReg = BReg;
            BReg = CReg;
            break;

        case 0xE: /* STNL — store non-local */
            *(uint32_t *)(mem + (index(AReg, operand) & MemByteMask)) = BReg;
            AReg = CReg;
            break;

        case 0xF: /* OPR — operation */
            /* Only handle the most common OPR codes here.
             * Return -1 for unhandled ones (caller falls back to p.c). */
            switch (operand) {
            case 0x00: /* REV */
                temp = AReg; AReg = BReg; BReg = temp;
                break;
            case 0x04: /* DIFF */
                AReg = BReg - AReg; BReg = CReg; break;
            case 0x05: /* ADD */
                AReg = BReg + AReg; BReg = CReg; break;
            case 0x08: /* PROD */
                AReg = BReg * AReg; BReg = CReg; break;
            case 0x09: /* GT */
                AReg = ((int32_t)BReg > (int32_t)AReg) ? 1 : 0;
                BReg = CReg; break;
            case 0x0A: /* WSUB */
                AReg = AReg + BReg * 4; BReg = CReg; break;
            case 0x20: /* RET */
                IPtr = word(WPtr);
                WPtr = WPtr + 16;
                continue;
            case 0x32: /* NOT */
                AReg = ~AReg; break;
            case 0x33: /* XOR */
                AReg = BReg ^ AReg; BReg = CReg; break;
            case 0x40: /* SHR */
                AReg = BReg >> AReg; BReg = CReg; break;
            case 0x41: /* SHL */
                AReg = BReg << AReg; BReg = CReg; break;
            case 0x42: /* MINT */
                CReg = BReg; BReg = AReg; AReg = 0x80000000u; break;
            case 0x46: /* AND */
                AReg = BReg & AReg; BReg = CReg; break;
            case 0x4B: /* OR */
                AReg = BReg | AReg; BReg = CReg; break;
            case 0x52: /* SUM */
                AReg = BReg + AReg; BReg = CReg; break;
            case 0x5A: /* DUP */
                CReg = BReg; BReg = AReg; break;
            case 0x5F: /* GTU */
                AReg = ((uint32_t)BReg > (uint32_t)AReg) ? 1 : 0;
                BReg = CReg; break;
            default:
                /* Unhandled OPR — return count so caller can use p.c fallback */
                return -count;
            }
            break;

        default:
            /* Should not reach here — all 16 opcodes handled */
            return -count;
        }

        /* Advance IPtr past the instruction (for non-jump cases) */
        IPtr = d->next_ip;
    }

    return count;
}
