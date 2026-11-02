#include "t4_fabric.h"
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdatomic.h>
#if defined(__linux__)
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/futex.h>
#endif

t4_fabric_t *t4_fabric;

static _Atomic int t4_net_quit_flag;

void t4_net_request_stop(void)
{
        atomic_store(&t4_net_quit_flag, 1);
        if (t4_fabric && t4_fabric->cpus) {
                int i;
                for (i = 0; i < t4_fabric->nnodes; i++)
                        if (t4_fabric->cpus[i])
                                t4_node_wake(t4_fabric->cpus[i]);
        }
}

int t4_net_stop_requested(void)
{
        return atomic_load(&t4_net_quit_flag) != 0;
}

t4_fabric_t *t4_fabric_create(int nnodes)
{
        t4_fabric_t *f;
        size_t bufsz;

        if (nnodes < 1)
                return NULL;
        f = calloc(1, sizeof(*f));
        if (!f)
                return NULL;
        f->nnodes = nnodes;
        bufsz = (size_t)8 * SCH_SIZE * (size_t)nnodes;
        f->base = calloc(1, bufsz + (size_t)nnodes);
        f->events = f->base ? f->base + bufsz : NULL;
        f->cpus = calloc((size_t)nnodes, sizeof(cpu_t *));
        if (!f->base || !f->cpus) {
                t4_fabric_destroy(f);
                return NULL;
        }
        t4_fabric = f;
        return f;
}

void t4_fabric_destroy(t4_fabric_t *f)
{
        if (!f)
                return;
        if (t4_fabric == f)
                t4_fabric = NULL;
        free(f->base);
        free(f->cpus);
        free(f);
}

void t4_fabric_set_cpu(t4_fabric_t *f, int nid, cpu_t *c)
{
        if (f && nid >= 0 && nid < f->nnodes)
                f->cpus[nid] = c;
}

int t4_idle_init(cpu_t *c)
{
        t4_idle_t *id;
        pthread_condattr_t attr;

        id = calloc(1, sizeof(*id));
        if (!id)
                return -1;
        pthread_mutex_init(&id->mu, NULL);
        pthread_condattr_init(&attr);
#ifdef CLOCK_MONOTONIC
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
#endif
        pthread_cond_init(&id->cv, &attr);
        pthread_condattr_destroy(&attr);
        c->idle = id;
        return 0;
}

void t4_idle_destroy(cpu_t *c)
{
        t4_idle_t *id = c ? c->idle : NULL;
        if (!id)
                return;
        pthread_cond_destroy(&id->cv);
        pthread_mutex_destroy(&id->mu);
        free(id);
        c->idle = NULL;
}

void t4_wake_node_id(int nid)
{
        if (t4_fabric && nid >= 0 && nid < t4_fabric->nnodes)
                t4_node_wake(t4_fabric->cpus[nid]);
}

void t4_node_wake(cpu_t *c)
{
        t4_idle_t *id;
        if (!c || !c->idle)
                return;
        id = c->idle;
        pthread_mutex_lock(&id->mu);
        id->idle_gen++;
        pthread_cond_signal(&id->cv);
#if defined(__linux__)
        syscall(SYS_futex, &id->idle_gen, FUTEX_WAKE_PRIVATE, 1, NULL, NULL, 0);
#endif
        pthread_mutex_unlock(&id->mu);
}

int t4_node_any_ready(void)
{
        int i;
        if (!t4cpu)
                return 0;
        for (i = 0; i < 4; i++) {
                Channel *in = &t4cpu->s_Link[i].In;
                Channel *out = &t4cpu->s_Link[i].Out;
                if (in->schbuf && SCHLength(in) != 0)
                        return 1;
                if (out->schbuf && out->Length && SCHLength(out) == 0)
                        return 1;
        }
        return 0;
}

int t4_node_idle_wait(cpu_t *c, int timeout_ms)
{
        t4_idle_t *id;
        unsigned gen;
        struct timespec ts;

        if (!c || !c->idle || timeout_ms <= 0)
                return 0;
        if (t4_node_any_ready())
                return 1;
#if defined(__linux__)
        {
                static int use_futex = -1;
                t4_idle_t *fid = c->idle;
                if (use_futex < 0) {
                        const char *e = getenv("T4_IDLE_FUTEX");
                        use_futex = (e && *e && *e != '0');
                }
                if (use_futex) {
                        struct timespec fts;
                        unsigned gen = fid->idle_gen;
                        fts.tv_sec = timeout_ms / 1000;
                        fts.tv_nsec = (timeout_ms % 1000) * 1000000L;
                        if (!t4_node_any_ready() && fid->idle_gen == gen)
                                syscall(SYS_futex, &fid->idle_gen, FUTEX_WAIT_PRIVATE,
                                        gen, &fts, NULL, 0);
                        return t4_node_any_ready();
                }
        }
#endif
        id = c->idle;
        pthread_mutex_lock(&id->mu);
        if (t4_node_any_ready()) {
                pthread_mutex_unlock(&id->mu);
                return 1;
        }
        gen = id->idle_gen;
#ifdef CLOCK_MONOTONIC
        clock_gettime(CLOCK_MONOTONIC, &ts);
#else
        clock_gettime(CLOCK_REALTIME, &ts);
#endif
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
        }
        while (id->idle_gen == gen && !t4_net_stop_requested()) {
                int rc = pthread_cond_timedwait(&id->cv, &id->mu, &ts);
                if (rc == ETIMEDOUT)
                        break;
        }
        pthread_mutex_unlock(&id->mu);
        return t4_node_any_ready();
}
