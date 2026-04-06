/*
 * timer.h — Timer and scheduler subsystem for T4 emulator
 *
 * The T800 has two hardware timers:
 * - High-priority: 1μs resolution (1 MHz)
 * - Low-priority: 64μs resolution (15,625 Hz)
 *
 * In threaded mode, a background thread updates ClockReg atomically.
 * In polled mode, the main loop calls timer_poll() periodically.
 */
#ifndef TIMER_H
#define TIMER_H

#include "emu.h"

/* Initialize timer subsystem. If threaded=1, starts background thread. */
int timer_init(emu_timer_t *t, int threaded);

/* Shut down timer subsystem (joins thread if running). */
void timer_shutdown(emu_timer_t *t);

/* Polled mode: call from main loop to advance timers.
 * Returns number of low-priority ticks elapsed since last call. */
int timer_poll(emu_timer_t *t);

/* Read current timer value (thread-safe). */
static inline uint32_t timer_read(const emu_timer_t *t, int priority) {
    return atomic_load_explicit(&t->ClockReg[priority], memory_order_relaxed);
}

#endif /* TIMER_H */
