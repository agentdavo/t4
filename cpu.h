/*
 * cpu.h — Transputer CPU execution interface
 *
 * The CPU module handles instruction fetch-decode-execute.
 * Uses computed goto dispatch for maximum performance.
 */
#ifndef CPU_H
#define CPU_H

#include "emu.h"

/* Run the CPU until quit is requested.
 * This is the hot loop — everything else should be in separate threads
 * or called infrequently. */
void cpu_run(emu_t *emu);

/* Execute a single OPR instruction (called from cpu_run).
 * Separated to keep the primary dispatch table small. */
void cpu_exec_opr(emu_t *emu);

/* Execute a single FPU OPR instruction. */
void cpu_exec_fpu(emu_t *emu);

/* Check if process should be descheduled (called on J/LEND). */
void cpu_check_deschedule(emu_t *emu);

#endif /* CPU_H */
