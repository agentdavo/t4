/*
 * timer_integration.c — Timer thread that updates p.c globals
 *
 * Runs a background thread that calls gettimeofday() and updates
 * ClockReg[0] (hi-pri, 1μs) and ClockReg[1] (lo-pri, 64μs)
 * without the main loop needing to call update_time().
 */
#include "timer_integration.h"
#include "t4_fabric.h"
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

        /* Only update ClockReg if timers are running; timeslice always. */
        {
            int n = 0, i;
            cpu_t *nodes[1];
            cpu_t **list = nodes;
            nodes[0] = NULL;
            if (t4_fabric && t4_fabric->cpus) {
                list = t4_fabric->cpus;
                n = t4_fabric->nnodes;
            } else if (tb->pClockReg) {
                if (tb->pTimers && *tb->pTimers == tb->TimersGo_val) {
                    tb->pClockReg[0] += (uint32_t)elapsed_us;
                    lo_accum += (int)elapsed_us;
                    if (lo_accum >= 64) {
                        tb->pClockReg[1] += (uint32_t)(lo_accum / 64);
                        lo_accum &= 63;
                    }
                }
                ts_accum += (int)elapsed_us;
                if (ts_accum >= 1024 && tb->ptimeslice) {
                    *tb->ptimeslice += ts_accum / 1024;
                    ts_accum &= 1023;
                }
                n = 0;
            }
            for (i = 0; i < n; i++) {
                cpu_t *c = list[i];
                if (!c)
                    continue;
                if (c->s_Timers == tb->TimersGo_val) {
                    c->s_ClockReg[0] += (uint32_t)elapsed_us;
                    c->s_ClockReg[1] += (uint32_t)(elapsed_us / 64);
                }
                c->s_timeslice += (int)(elapsed_us / 1024);
            }
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
