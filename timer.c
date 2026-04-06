/*
 * timer.c — Timer and scheduler subsystem for T4 emulator
 *
 * Supports two modes:
 * 1. Threaded: background thread updates ClockReg via C11 atomics
 * 2. Polled: main loop calls timer_poll() (fallback)
 */
#include "timer.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

/* Try C11 threads first, fall back to no threading */
#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L && !defined(__STDC_NO_THREADS__)
#include <threads.h>
#define HAVE_C11_THREADS 1
#else
#define HAVE_C11_THREADS 0
#endif

/* -------------------------------------------------------------------
 * Shared state for polled mode
 * ------------------------------------------------------------------- */
static struct timeval timer_last_tod;
static int timer_lo_accum = 0;  /* Accumulator for low-priority ticks */
static int timer_ts_accum = 0;  /* Accumulator for timeslice ticks */

/* -------------------------------------------------------------------
 * Threaded mode
 * ------------------------------------------------------------------- */
#if HAVE_C11_THREADS

static thrd_t timer_thread;
static _Atomic int timer_thread_running = 0;

static int timer_thread_fn(void *arg)
{
    emu_timer_t *t = (emu_timer_t *)arg;
    struct timeval last, now;

    gettimeofday(&last, NULL);

    while (atomic_load_explicit(&timer_thread_running, memory_order_relaxed)) {
        /* Sleep ~100μs between updates (balance resolution vs overhead) */
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000 };
        thrd_sleep(&ts, NULL);

        gettimeofday(&now, NULL);
        long elapsed_us = (now.tv_sec - last.tv_sec) * 1000000L +
                          (now.tv_usec - last.tv_usec);
        last = now;

        if (elapsed_us <= 0) continue;

        /* High-priority timer: 1μs resolution */
        if (t->TimerEnableHi)
            atomic_fetch_add_explicit(&t->ClockReg[0], (uint32_t)elapsed_us,
                                      memory_order_relaxed);

        /* Low-priority timer: 64μs resolution */
        if (t->TimerEnableLo) {
            static int lo_accum = 0;
            lo_accum += (int)elapsed_us;
            if (lo_accum >= 64) {
                atomic_fetch_add_explicit(&t->ClockReg[1], (uint32_t)(lo_accum / 64),
                                          memory_order_relaxed);
                lo_accum &= 63;
            }
        }

        /* Timeslice: increment every ~1024μs */
        {
            static int ts_accum = 0;
            ts_accum += (int)elapsed_us;
            if (ts_accum >= 1024) {
                atomic_fetch_add_explicit(&t->timeslice, ts_accum / 1024,
                                          memory_order_relaxed);
                ts_accum &= 1023;
            }
        }
    }

    return 0;
}

#endif /* HAVE_C11_THREADS */

/* -------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

int timer_init(emu_timer_t *t, int threaded)
{
    gettimeofday(&timer_last_tod, NULL);
    timer_lo_accum = 0;
    timer_ts_accum = 0;

    t->TimerEnableHi = 1;
    t->TimerEnableLo = 1;
    atomic_store_explicit(&t->ClockReg[0], 0, memory_order_relaxed);
    atomic_store_explicit(&t->ClockReg[1], 0, memory_order_relaxed);
    atomic_store_explicit(&t->timeslice, 0, memory_order_relaxed);

#if HAVE_C11_THREADS
    if (threaded) {
        atomic_store(&timer_thread_running, 1);
        if (thrd_create(&timer_thread, timer_thread_fn, t) == thrd_success) {
            fprintf(stderr, "[TIMER] Background thread started (C11 threads)\n");
            return 1;  /* threaded mode active */
        }
        fprintf(stderr, "[TIMER] Thread creation failed, falling back to polled\n");
        atomic_store(&timer_thread_running, 0);
    }
#else
    (void)threaded;
#endif

    fprintf(stderr, "[TIMER] Polled mode\n");
    return 0;  /* polled mode */
}

void timer_shutdown(emu_timer_t *t)
{
    (void)t;
#if HAVE_C11_THREADS
    if (atomic_load(&timer_thread_running)) {
        atomic_store(&timer_thread_running, 0);
        thrd_join(timer_thread, NULL);
        fprintf(stderr, "[TIMER] Background thread stopped\n");
    }
#endif
}

int timer_poll(emu_timer_t *t)
{
    struct timeval now;
    long elapsed_us;
    int lo_ticks = 0;

    gettimeofday(&now, NULL);
    elapsed_us = (now.tv_sec - timer_last_tod.tv_sec) * 1000000L +
                 (now.tv_usec - timer_last_tod.tv_usec);

    if (elapsed_us <= 0) return 0;
    timer_last_tod = now;

    /* High-priority: 1μs */
    if (t->TimerEnableHi)
        atomic_fetch_add_explicit(&t->ClockReg[0], (uint32_t)elapsed_us,
                                  memory_order_relaxed);

    /* Low-priority: 64μs */
    timer_lo_accum += (int)elapsed_us;
    if (timer_lo_accum >= 64) {
        lo_ticks = timer_lo_accum / 64;
        if (t->TimerEnableLo)
            atomic_fetch_add_explicit(&t->ClockReg[1], (uint32_t)lo_ticks,
                                      memory_order_relaxed);
        timer_lo_accum &= 63;
    }

    /* Timeslice */
    timer_ts_accum += (int)elapsed_us;
    if (timer_ts_accum >= 1024) {
        atomic_fetch_add_explicit(&t->timeslice, timer_ts_accum / 1024,
                                  memory_order_relaxed);
        timer_ts_accum &= 1023;
    }

    return lo_ticks;
}
