/*
 * timer_integration.h — Bridge between timer.c and p.c globals
 *
 * During the transition period, timer.c updates p.c's global
 * variables directly via pointers. Once the full emu_t migration
 * is complete, this file can be removed.
 */
#ifndef TIMER_INTEGRATION_H
#define TIMER_INTEGRATION_H

#include <stdint.h>
#include <stdatomic.h>

/* Timer thread state */
typedef struct {
    /* Pointers to p.c globals — set during init */
    _Atomic uint32_t *ClockReg;   /* ClockReg[2] in p.c */
    _Atomic int      *timeslice;  /* timeslice in p.c */
    _Atomic int      *Timers;     /* TimersGo flag in p.c */
    int                TimersGo_val; /* Value that means "timers running" */

    /* Thread control */
    _Atomic int running;
    _Atomic int quit;
} timer_bridge_t;

/* Initialize and start the timer thread.
 * Returns 1 if threaded, 0 if polled fallback. */
int timer_bridge_start(timer_bridge_t *tb);

/* Stop the timer thread. */
void timer_bridge_stop(timer_bridge_t *tb);

#endif /* TIMER_INTEGRATION_H */
