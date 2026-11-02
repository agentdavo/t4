#include "t4_fabric.h"
#include "t9000_tl.h"
#include <stdio.h>
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
#include "netcfg.h"
#include "t9000_ctl.h"

t4_fabric_t *t4_fabric;

static _Atomic int t4_net_quit_flag;

/* ---- IMS C104 packet routing switch --------------------------------------
 *
 * HRM part 1 ch.7 (7.2 the C104, 7.2.1 header deletion, 7.3 interval
 * labelling) and part 3 ch.2.  A router node is a thread on the fabric with
 * one input mailbox per link, in the same format as a transputer's link
 * inputs; an output writes into the input mailbox at the far end of its
 * link.  The VCP sends each packet as one link message, so the switch moves
 * whole packets: wormhole routing approximated a packet at a time.  A packet
 * stays in its input mailbox until an output takes it, which stalls that
 * input - and through it the sender - as a blocked worm does.
 *
 * The first one or two bytes of a packet are its header (two-byte headers
 * are read as the T4 VCP reads them, first byte least significant).  The
 * input link's interval selector maps the header to an output link, or to
 * a group of links of which the first free one is used (grouped adaptive
 * routing).  An output set for header deletion drops those bytes, exposing
 * the next header to the device beyond.  Packets of any length pass.
 *
 * The configuration is static, from the network map (netcfg.c); on the chip
 * it is programmed over the control link, which T4 does not model.
 */

#define RT_IDLE_MS      100     /* fallback poll; wakes do the real work */

typedef struct t4_rport {
        u_char  *inbox;         /* packets arriving on this link */
        u_char  *outbox;        /* the input mailbox at the far end, or NULL */
        int      peer;          /* node at the far end, or -1 */
        int      first, last;   /* output link(s) of the waiting packet; -1: none */
        int      rr;            /* as an output: the input it serves first next */
        int      warned_drop, warned_stuck;
        t9k_tl_t out_tl;        /* as an output: its transfers (T4_T9K_NET) */
} t4_rport_t;

/* T9000 network timing (p.c, t9000_group.h) */
int      t9k_net_timed(void);
uint64_t t9k_route_packet(uint64_t ready, uint32_t nbytes, t9k_tl_t *out);

typedef struct t4_router {
        const net_router_t *cfg;
        int        id, nlinks;
        t4_rport_t port[NET_ROUTER_LINKS];
        u_char    *boxes;         /* the input mailboxes */
        t4_idle_t  idle;
        pthread_t  th;
        int        running;
        unsigned long routed, discarded;
} t4_router_t;

static int rt_debug;

static void idle_setup(t4_idle_t *id)
{
        pthread_condattr_t attr;

        pthread_mutex_init(&id->mu, NULL);
        pthread_condattr_init(&attr);
#ifdef CLOCK_MONOTONIC
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
#endif
        pthread_cond_init(&id->cv, &attr);
        pthread_condattr_destroy(&attr);
}

static void idle_wake(t4_idle_t *id)
{
        pthread_mutex_lock(&id->mu);
        id->idle_gen++;
        pthread_cond_signal(&id->cv);
#if defined(__linux__)
        syscall(SYS_futex, &id->idle_gen, FUTEX_WAKE_PRIVATE, 1, NULL, NULL, 0);
#endif
        pthread_mutex_unlock(&id->mu);
}

static void idle_deadline(struct timespec *ts, int timeout_ms)
{
#ifdef CLOCK_MONOTONIC
        clock_gettime(CLOCK_MONOTONIC, ts);
#else
        clock_gettime(CLOCK_REALTIME, ts);
#endif
        ts->tv_sec += timeout_ms / 1000;
        ts->tv_nsec += (timeout_ms % 1000) * 1000000L;
        if (ts->tv_nsec >= 1000000000L) {
                ts->tv_sec++;
                ts->tv_nsec -= 1000000000L;
        }
}

static t4_router_t *router_by_id(t4_fabric_t *f, int id)
{
        int i;

        for (i = 0; i < f->nrouters; i++)
                if (f->routers[i].id == id)
                        return &f->routers[i];
        return NULL;
}

void t4_net_request_stop(void)
{
        atomic_store(&t4_net_quit_flag, 1);
        if (t4_fabric && t4_fabric->cpus) {
                int i;
                for (i = 0; i < t4_fabric->nnodes; i++)
                        if (t4_fabric->cpus[i])
                                t4_node_wake(t4_fabric->cpus[i]);
                for (i = 0; i < t4_fabric->nrouters; i++)
                        idle_wake(&t4_fabric->routers[i].idle);
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
        int i;

        if (!f)
                return;
        t4_fabric_stop_routers(f);
        for (i = 0; i < f->nrouters; i++) {
                t4_router_t *r = &f->routers[i];

                if (rt_debug)
                        fprintf(stderr, "[C104 %d] %lu packets routed, %lu discarded\n",
                                r->id, r->routed, r->discarded);
                pthread_cond_destroy(&r->idle.cv);
                pthread_mutex_destroy(&r->idle.mu);
                free(r->boxes);
        }
        free(f->routers);
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

/* The input mailbox of a node's link: what an output connected to it
 * writes into.  NULL if there is no such node or link. */
u_char *t4_fabric_inbox(t4_fabric_t *f, int nid, int link)
{
        t4_router_t *r;

        if (!f || nid < 0 || link < 0)
                return NULL;
        /* a node's 8 slots: CLink0-1 In (links 4-5, t9000_ctl.c) in 0-1,
         * Link0-3 In in 4-7 */
        if (nid < f->nnodes)
                return link < 4 ? f->base + ((size_t)nid * 8 + 4 + link) * SCH_SIZE :
                       link < 6 ? f->base + ((size_t)nid * 8 + link - 4) * SCH_SIZE : NULL;
        r = router_by_id(f, nid);
        return r && link < r->nlinks ? r->port[link].inbox : NULL;
}

int t4_idle_init(cpu_t *c)
{
        t4_idle_t *id;

        id = calloc(1, sizeof(*id));
        if (!id)
                return -1;
        idle_setup(id);
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
        t4_fabric_t *f = t4_fabric;
        t4_router_t *r;

        if (!f || nid < 0)
                return;
        if (nid < f->nnodes)
                t4_node_wake(f->cpus[nid]);
        else if ((r = router_by_id(f, nid)) != NULL)
                idle_wake(&r->idle);
}

void t4_node_wake(cpu_t *c)
{
        if (!c || !c->idle)
                return;
        idle_wake(c->idle);
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
        return t9k_ctl_ready();
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
        idle_deadline(&ts, timeout_ms);
        while (id->idle_gen == gen && !t4_net_stop_requested()) {
                int rc = pthread_cond_timedwait(&id->cv, &id->mu, &ts);
                if (rc == ETIMEDOUT)
                        break;
        }
        pthread_mutex_unlock(&id->mu);
        return t4_node_any_ready();
}

/* ---- router engine ------------------------------------------------------- */

/* The length word of a mailbox is the handshake with the node at the other
 * end: non-zero = full, written last by the producer, cleared by the
 * consumer once the bytes are copied out. */
static inline unsigned mb_len(const u_char *mb)
{
        return __atomic_load_n((const u_short *)mb, __ATOMIC_ACQUIRE);
}

static inline void mb_set(u_char *mb, unsigned n)
{
        __atomic_store_n((u_short *)mb, (u_short)n, __ATOMIC_RELEASE);
}

static void router_take(t4_router_t *r, int i)
{
        t4_rport_t *in = &r->port[i];

        in->first = in->last = -1;
        mb_set(in->inbox, 0);
        t4_wake_node_id(in->peer);
}

/* The HRM leaves a misrouted packet to the C104's control system; with
 * none modelled the packet is dropped and the first one reported. */
static void router_discard(t4_router_t *r, int i, const char *why, unsigned hdr)
{
        if (!r->port[i].warned_drop || rt_debug)
                printf("-W-EMU104: Router %d Link%d: packet header #%X %s; discarded\n",
                       r->id, i, hdr, why);
        r->port[i].warned_drop = 1;
        r->discarded++;
        router_take(r, i);
}

/* Choose the output for the packet waiting on input i (HRM 7.3.1). */
static void router_route(t4_router_t *r, int i)
{
        const net_router_t *c = r->cfg;
        t4_rport_t *in = &r->port[i];
        const u_char *p = in->inbox + SCH_DATA;
        unsigned len = mb_len(in->inbox);
        unsigned hl = (c->hdr2 >> i & 1u) ? 2 : 1;
        unsigned hdr;
        int j, o;

        if (len < hl + 1) {
                /* not even a header and a terminator */
                router_discard(r, i, "is incomplete", p[0]);
                return;
        }
        hdr = p[0] | (hl == 2 ? (unsigned)p[1] << 8 : 0u);
        for (j = 0; j < c->nint; j++) {
                const net_interval_t *it = &c->ints[j];

                if (!(it->on >> i & 1u) || hdr < it->lo || hdr >= it->hi)
                        continue;
                in->first = it->first;
                in->last = it->last;
                if (rt_debug)
                        fprintf(stderr, "[C104 %d] Link%d header #%X (%u bytes) -> Link%d-%d\n",
                                r->id, i, hdr, len, it->first, it->last);
                for (o = it->first; o <= it->last; o++)
                        if (r->port[o].outbox)
                                return;
                /* a worm into an unconnected link never moves */
                if (!in->warned_stuck)
                        printf("-W-EMU104: Router %d Link%d: header #%X routed to unconnected "
                               "Link%d; the input is blocked\n", r->id, i, hdr, it->first);
                in->warned_stuck = 1;
                return;
        }
        router_discard(r, i, "is in no interval", hdr);
}

static void router_forward(t4_router_t *r, int i, int o)
{
        t4_rport_t *in = &r->port[i], *out = &r->port[o];
        unsigned len = mb_len(in->inbox);
        unsigned skip = 0;

        if (r->cfg->del >> o & 1u)              /* header deletion, HRM 7.2.1 */
                skip = (r->cfg->hdr2 >> i & 1u) ? 2 : 1;
        memcpy(out->outbox + SCH_DATA, in->inbox + SCH_DATA + skip, len - skip);
        if (t9k_net_timed()) {
                /* the packet's cycle stamp: when it is ready on the output */
                uint64_t ready;

                memcpy(&ready, in->inbox + SCH_STAMP, 8);
                ready = t9k_route_packet(ready, len - skip, &out->out_tl);
                memcpy(out->outbox + SCH_STAMP, &ready, 8);
        }
        mb_set(out->outbox, len - skip);
        t4_wake_node_id(out->peer);
        r->routed++;
        router_take(r, i);
}

/* The cycle stamp of the packet waiting on input i. */
static uint64_t router_stamp(t4_router_t *r, int i)
{
        uint64_t t;

        memcpy(&t, r->port[i].inbox + SCH_STAMP, 8);
        return t;
}

/* One pass of the crossbar: route new arrivals, then give each free output
 * to one of the packets waiting for it.  Each output arbitrates round robin
 * from the input after the one it served last, so every input waiting for
 * an output gets it within nlinks packets.  On a timed network the packet
 * that reached the switch first - the earliest stamp - goes first, as on
 * the wire; round robin breaks ties.  Returns whether anything moved. */
static int router_step(t4_router_t *r)
{
        int n = r->nlinks, moved = 0, k, i, o, timed = t9k_net_timed();

        for (i = 0; i < n; i++) {
                t4_rport_t *in = &r->port[i];

                if (in->first < 0 && mb_len(in->inbox)) {
                        router_route(r, i);
                        if (in->first < 0)
                                moved = 1;      /* discarded */
                }
        }
        for (o = 0; o < n; o++) {
                t4_rport_t *out = &r->port[o];

                int best = -1;
                uint64_t bt = 0;

                if (!out->outbox || mb_len(out->outbox))
                        continue;
                for (k = 0; k < n; k++) {
                        t4_rport_t *in;

                        i = (out->rr + k) % n;
                        in = &r->port[i];
                        if (in->first <= o && o <= in->last) {
                                uint64_t t = timed ? router_stamp(r, i) : 0;

                                if (best < 0 || t < bt) {
                                        best = i;
                                        bt = t;
                                }
                                if (!timed)
                                        break;
                        }
                }
                if (best >= 0) {
                        router_forward(r, best, o);
                        out->rr = (best + 1) % n;
                        moved = 1;
                }
        }
        return moved;
}

static void *router_thread(void *arg)
{
        t4_router_t *r = arg;
        t4_idle_t *id = &r->idle;

        while (!t4_net_stop_requested()) {
                struct timespec ts;
                unsigned gen;

                pthread_mutex_lock(&id->mu);
                gen = id->idle_gen;
                pthread_mutex_unlock(&id->mu);
                if (router_step(r))
                        continue;
                pthread_mutex_lock(&id->mu);
                idle_deadline(&ts, RT_IDLE_MS);
                while (id->idle_gen == gen && !t4_net_stop_requested())
                        if (pthread_cond_timedwait(&id->cv, &id->mu, &ts) == ETIMEDOUT)
                                break;
                pthread_mutex_unlock(&id->mu);
        }
        return NULL;
}

/* Create the routers of the network map and start them.  Must run before
 * any processor opens its links, since those may end at a router. */
int t4_fabric_start_routers(t4_fabric_t *f)
{
        int n = netRouterCount(), i, l, on, ol, bn, bl;
        const char *e = getenv("T4_ROUTER_DEBUG");

        if (!f || n == 0)
                return 0;
        rt_debug = e && *e && *e != '0';
        f->routers = calloc((size_t)n, sizeof(*f->routers));
        if (!f->routers)
                return -1;
        for (i = 0; i < n; i++) {
                t4_router_t *r = &f->routers[i];
                const net_router_t *c = netRouter(i);

                if (c->id < f->nnodes) {
                        printf("-E-EMU414: Router %d has the id of a processor (0-%d)\n",
                               c->id, f->nnodes - 1);
                        return -1;
                }
                r->cfg = c;
                r->id = c->id;
                r->nlinks = c->nlinks;
                r->boxes = calloc((size_t)r->nlinks, SCH_SIZE);
                if (!r->boxes)
                        return -1;
                for (l = 0; l < r->nlinks; l++) {
                        r->port[l].inbox = r->boxes + (size_t)l * SCH_SIZE;
                        r->port[l].peer = -1;
                        r->port[l].first = r->port[l].last = -1;
                }
                idle_setup(&r->idle);
                f->nrouters = i + 1;
        }
        /* outputs: an output may end at another router's input */
        for (i = 0; i < n; i++) {
                t4_router_t *r = &f->routers[i];

                for (l = 0; l < r->nlinks; l++) {
                        if (linkedNetLink(r->id, l, &on, &ol) != 0)
                                continue;
                        r->port[l].outbox = t4_fabric_inbox(f, on, ol);
                        if (!r->port[l].outbox) {
                                printf("-E-EMU414: Router %d Link%d: no node %d link %d\n",
                                       r->id, l, on, ol);
                                return -1;
                        }
                        r->port[l].peer = on;
                        /* the far end must also know it: open_channel and
                         * the routers look the link up from their side */
                        if ((on < f->nnodes ? connectedNetLink(on, ol, &bn, &bl)
                                            : linkedNetLink(on, ol, &bn, &bl)) != 0 ||
                            bn != r->id || bl != l)
                                printf("-W-EMU414: Node %d Link%d does not lead back to router %d "
                                       "Link%d; use \"Connect\"\n", on, ol, r->id, l);
                }
        }
        for (i = 0; i < n; i++) {
                t4_router_t *r = &f->routers[i];

                if (pthread_create(&r->th, NULL, router_thread, r) != 0) {
                        printf("-E-EMU414: Failed to start router %d thread\n", r->id);
                        return -1;
                }
                r->running = 1;
                printf("-I-EMU414: Started router %d (IMS C104, %d links, thread)\n",
                       r->id, r->nlinks);
        }
        return 0;
}

void t4_fabric_stop_routers(t4_fabric_t *f)
{
        int i;

        if (!f || !f->nrouters)
                return;
        t4_net_request_stop();
        for (i = 0; i < f->nrouters; i++)
                if (f->routers[i].running) {
                        pthread_join(f->routers[i].th, NULL);
                        f->routers[i].running = 0;
                }
}
