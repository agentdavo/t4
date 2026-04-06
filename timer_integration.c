/*
 * timer_integration.c — Timer thread that updates p.c globals
 *
 * Runs a background thread that calls gettimeofday() and updates
 * ClockReg[0] (hi-pri, 1μs) and ClockReg[1] (lo-pri, 64μs)
 * without the main loop needing to call update_time().
 */
#include "timer_integration.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <threads.h>

static thrd_t timer_thrd;

static int timer_thread_fn(void *arg)
{
    timer_bridge_t *tb = (timer_bridge_t *)arg;
    struct timeval last, now;
    int lo_accum = 0;
    int ts_accum = 0;

    gettimeofday(&last, NULL);

    while (atomic_load_explicit(&tb->quit, memory_order_relaxed) == 0) {
        /* Sleep 100μs between updates */
        struct timespec ts = { .tv_sec = 0, .tv_nsec = 100000 };
        thrd_sleep(&ts, NULL);

        gettimeofday(&now, NULL);
        long elapsed_us = (now.tv_sec - last.tv_sec) * 1000000L +
                          (now.tv_usec - last.tv_usec);
        last = now;

        if (elapsed_us <= 0) continue;

        /* Only update if timers are running */
        if (tb->Timers && *tb->Timers == tb->TimersGo_val) {
            /* High-priority: 1μs resolution */
            tb->ClockReg[0] += (uint32_t)elapsed_us;

            /* Low-priority: 64μs resolution */
            lo_accum += (int)elapsed_us;
            if (lo_accum >= 64) {
                tb->ClockReg[1] += (uint32_t)(lo_accum / 64);
                lo_accum &= 63;
            }
        }

        /* Timeslice counter: increment every ~1024μs */
        ts_accum += (int)elapsed_us;
        if (ts_accum >= 1024) {
            *tb->timeslice += ts_accum / 1024;
            ts_accum &= 1023;
        }
    }

    return 0;
}

int timer_bridge_start(timer_bridge_t *tb)
{
    atomic_store(&tb->quit, 0);
    atomic_store(&tb->running, 0);

    if (thrd_create(&timer_thrd, timer_thread_fn, tb) == thrd_success) {
        atomic_store(&tb->running, 1);
        fprintf(stderr, "[TIMER_THREAD] Started (C11 threads, 100μs polling)\n");
        return 1;
    }

    fprintf(stderr, "[TIMER_THREAD] Failed to create, using polled mode\n");
    return 0;
}

void timer_bridge_stop(timer_bridge_t *tb)
{
    if (atomic_load(&tb->running)) {
        atomic_store(&tb->quit, 1);
        thrd_join(timer_thrd, NULL);
        atomic_store(&tb->running, 0);
        fprintf(stderr, "[TIMER_THREAD] Stopped\n");
    }
}
