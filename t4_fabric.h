#ifndef T4_FABRIC_H
#define T4_FABRIC_H

#include <pthread.h>
#include "processor.h"
#include "cpu_state.h"

typedef struct t4_idle {
        pthread_mutex_t mu;
        pthread_cond_t  cv;
        unsigned        idle_gen;
} t4_idle_t;

struct t4_router;

typedef struct t4_fabric {
        int      nnodes;
        u_char  *base;
        u_char  *events;
        cpu_t  **cpus;
        int      nrouters;              /* IMS C104s from the network map */
        struct t4_router *routers;
} t4_fabric_t;

extern t4_fabric_t *t4_fabric;

t4_fabric_t *t4_fabric_create(int nnodes);
void t4_fabric_destroy(t4_fabric_t *f);
void t4_fabric_set_cpu(t4_fabric_t *f, int nid, cpu_t *c);
u_char *t4_fabric_inbox(t4_fabric_t *f, int nid, int link);
int  t4_fabric_start_routers(t4_fabric_t *f);
void t4_fabric_stop_routers(t4_fabric_t *f);
int  t4_idle_init(cpu_t *c);
void t4_idle_destroy(cpu_t *c);
void t4_node_wake(cpu_t *c);
void t4_wake_node_id(int nid);
int  t4_node_idle_wait(cpu_t *c, int timeout_ms);
int  t4_node_any_ready(void);
void t4_net_request_stop(void);
int  t4_net_stop_requested(void);

/* Node time in a threaded network (t4_fabric.c).  T4_LOCKSTEP=1
 * (t4_ls_stamps) orders link events by it; T4_LOCKSTEP_WINDOW=W
 * (t4_ls_window) keeps every running node within about W instructions of
 * the slowest.  Both are off by default; t4_ls_active is either.
 * t4_ls_tick() runs in the timer block when there is a window, the
 * park/unpark pair around a node's idle waits and its mainloop(). */
extern int t4_ls_active, t4_ls_stamps, t4_ls_window;
void t4_ls_tick(void);
void t4_ls_park(void);
void t4_ls_unpark(void);
/* the node's time, a jump forward to t, and whether a message stamped
 * `stamp` has reached the node yet */
uint64_t t4_ls_now(void);
void t4_ls_advance(uint64_t t);
int  t4_ls_visible(uint64_t stamp);

#endif
