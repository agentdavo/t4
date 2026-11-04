/*
 * IMS T9000 external channels for the T4 emulator: VCP, virtual links,
 * event channels, resource channels, vin/vout and configuration space.
 * See t9000_vcp.h for the address map and the sources.
 *
 * Execution model.  Everything here runs on the node's own thread.  The CPU
 * side (the instruction handlers) records a request in the VLCB and queues
 * the virtual link on its physical link; the link side (t9k_link_tx/rx,
 * driven from linkcomms()) moves one packet per call through the existing
 * link mailboxes.  Each emulated link message is one VCP packet:
 *
 *     header (1..255 bytes) | data (0..32 bytes) | token (EOP or EOM)
 *
 * The receiver strips 1 or 2 header bytes according to its VCPLinkNMode
 * HeaderLength bit, exactly as the hardware does.  Between two nodes there
 * may be IMS C104 routers (t4_fabric.c): they route on the header with
 * interval labelling and may delete it (HRM 7.2.1), so what arrives here
 * is the header the last router left.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "processor.h"
#include "cpu_state.h"
#include "t9000_vcp.h"
#include "t9000_ctl.h"

extern int  channel_sendP (Channel *chan, u_char *data, int ndata, int doWait);
extern int  channel_recvP (Channel *chan, u_char *data, int doWait);
extern uint32_t t4_product_identity (void);
extern void t4_wake (uint32_t wdesc);
extern void t4_wake_alt (uint32_t wdesc);

/* p.c's word()/writeword() are inline in release builds; these are the
 * out-of-line accessors behind them (translation and MMIO included). */
#undef word
#undef writeword
#define word(a)         word_int (a)
#define writeword(a, x) writeword_int ((a), (x))

/* T4 keeps the T800 workspace layout for T9000 code: pw.Pointer, pw.State
 * and pw.Length share word -3; pw.Link and pw.Count share word -2. */
#define W_TEMP          0
#define W_LINK          (-2)
#define W_POINTER       (-3)
#define WSLOT(w, s)     ((w) + (uint32_t)(4 * (s)))

#define NOTPROC         0x80000000u
#define ENABLING_P      0x80000001u
#define WAITING_P       0x80000002u
#define READY_P         0x80000003u
#define NONESELECTED_O  0xFFFFFFFFu
#define DWPTR(d)        ((d) & 0xFFFFFFFEu)
#define DPRI(d)         ((d) & 1u)
#define IS_SPECIAL(x)   (((x) & 0xFFFFFFFCu) == 0x80000000u)
#define ERRFLAG         0x80000000u
#define SETERR()        (STATUSReg |= ERRFLAG)
#define CURWDESC        (WdescReg)
#define UNDEFINED       0xdeadbeefu     /* p.c Undefined_p */

/* VCP states (HRM 11.8).  RUNNING and above take packets off the links;
 * DISCARDING drops everything it receives and sends nothing. */
#define VCP_RESETTING   0
#define VCP_WAITING     1
#define VCP_RUNNING     2
#define VCP_STOPPING    3
#define VCP_DISCARDING  4

#define MAX_VLINKS      65536u          /* 16-bit headers */

/* Pending actions held while a channel is deactivated (HRM 11.3.1). */
#define PEND_ACK        0x01u
#define PEND_ALT        0x02u
#define PEND_CLAIM      0x04u

/* VCPStatus error codes (HRM table 11.9) */
#define ERR_RANGE       0u
#define ERR_SHORTHDR    1u
#define ERR_SHORTPKT    2u
#define ERR_LONGPKT     3u

typedef struct t9k_vlink {
        /* vl.HeaderCtrl */
        uint8_t  inited;
        uint8_t  link;
        uint8_t  hdr_null;
        uint8_t  hdr_len;
        uint8_t  hdr[T9K_MAX_SHORT_HEADER];
        uint32_t hdr_off;               /* header region word offset */
        uint32_t buf;                   /* vl.BufferPointer */

        /* virtual input channel (even address) */
        uint8_t  in_active, in_stopping, in_vlen, in_overrun, in_alt;
        uint8_t  in_pend;               /* PEND_* while deactivated */
        uint8_t  buf_state, buf_len;    /* readbfr: 0 none, 1 last, 2 other */
        uint8_t  rc_mode;               /* rc.Id != NotProcess.p */
        uint8_t  ack_owed, ack_prio;
        uint32_t in_wdesc;              /* vl.InputWdesc */
        uint32_t in_ptr, in_limit, in_count;
        uint32_t in_sched;              /* completion held while deactivated */

        /* virtual output channel (odd address) */
        uint8_t  out_active, out_await_ack, out_last, out_prio;
        uint32_t out_wdesc;             /* vl.OutputWdesc */
        uint32_t out_ptr, out_left;
        uint32_t out_sched;
        uint32_t stop_wdesc;            /* process in stopch */

        /* link queues: vl.DataQueueLink / vl.AckQueueLink */
        int32_t  dq_next, aq_next;
        uint8_t  in_dq, in_aq, dq_pri, aq_pri;

        /* T4_T9K_NET timing: the earliest cycle the next data packet and
         * the owed acknowledge may leave; the cycle the buffered packet
         * arrived */
        uint64_t t_data, t_ack, t_buf;

        /* VXFER: the transfer belongs to its process's group; an input's
         * length goes back to in_cnt_at.  out_posted: a group's output
         * whose last packet has left, the final acknowledge still to come */
        uint8_t  in_grp, out_grp, out_posted;
        uint32_t in_cnt_at;
} t9k_vlink_t;

typedef struct t9k_plink {
        uint32_t vmode, max_hdr, min_hdr, hdr_off;      /* VCPLinkN* */
        uint32_t ds_mode, ds_status, ds_wlock;          /* DS-link LinkN* */
        int32_t  dq_head[2], dq_tail[2], aq_head[2], aq_tail[2];
        int      ack_turn[2];           /* alternate ack and data */
        int      warned;
} t9k_plink_t;

typedef struct t9k_event {
        uint8_t  in_pin, out_pin;
        uint8_t  in_taken;              /* current EventIn high acknowledged */
        uint8_t  out_done;              /* output acknowledged while deactivated */
        uint8_t  rc_mode;
} t9k_event_t;

#define NMISC 64

typedef struct t9k_vcp {
        int      space;                 /* T9000 channel space in use */
        int      state;
        int      stop_sig;              /* control unit Stop while waiting */
        uint32_t command;
        uint32_t memstart, min_invalid, ext_rc_base, hdr_area_base;
        uint32_t status, chan_wlock;
        uint32_t initial_iptr, initial_wptr, reason, emi_bad;
        t9k_plink_t plink[T9K_NUM_LINKS];
        t9k_event_t ev[4];
        int      ev_loopback;
        uint32_t nvl;
        t9k_vlink_t *vl;
        int      debug;
        int      nmisc;
        uint32_t misc_addr[NMISC], misc_val[NMISC];
        uint64_t t_rx;                  /* arrival cycle of the packet in hand */
        uint64_t t_head;                /* and of its header (HRM 11.2.3) */
        /* processes waiting for a group of transfers (VXFER): transfers
         * left, and the latest cycle one of them finished */
        struct { uint32_t wdesc, left; uint64_t t; } grp[8];
} t9k_vcp_t;

/* T4_T9K_NET (p.c, t9000_group.h).  The VCP is hardware beside the CPU, so
 * its packets carry cycles of their own: a packet leaves when its link and
 * its virtual link allow, not when the CPU happens to look, and only a
 * process the VCP wakes waits for the packet that completed its message. */
int      t9k_net_timed (void);
void     t9k_net_defer (int on, uint64_t not_before);
uint64_t t9k_net_arrival (void);
uint64_t t9k_net_head (unsigned hl);
uint64_t t9k_net_sent_end (void);
uint64_t t9k_net_now (void);
void     t9k_net_wake (uint32_t wdesc, uint64_t t);

static uint64_t vcp_now (void)
{
        return t9k_net_timed () ? t9k_net_now () : 0;
}

/* The transfer w waited for finished at cycle t: on a timed network w
 * resumes no earlier than t, and the CPU runs other processes meanwhile. */
static void vcp_wake (uint32_t w, uint64_t t)
{
        if (t9k_net_timed ())
                t9k_net_wake (w, t);
        else
                t4_wake (w);
}

#define VDBG(v, ...) \
        do { if ((v)->debug) { fprintf (stderr, "[VCP%d] ", nodeid); \
                               fprintf (stderr, __VA_ARGS__); } } while (0)

static void ev_set_out (t9k_vcp_t *v, int n, int level);

/* ---- state ------------------------------------------------------------ */

static void vl_clear (t9k_vlink_t *l)
{
        memset (l, 0, sizeof *l);
        l->hdr_null = 1;
        l->hdr_off = T9K_NULOFFSET;
        l->in_wdesc = l->in_sched = NOTPROC;
        l->out_wdesc = l->out_sched = l->stop_wdesc = NOTPROC;
        l->dq_next = l->aq_next = -1;
}

static void plink_clear_queues (t9k_plink_t *p)
{
        p->dq_head[0] = p->dq_head[1] = p->dq_tail[0] = p->dq_tail[1] = -1;
        p->aq_head[0] = p->aq_head[1] = p->aq_tail[0] = p->aq_tail[1] = -1;
}

/* LinkNStatus.LinkStarted: a link reset by software or by an error (HRM
 * 13.5) carries nothing until it is started again.  ds_sync() mirrors the
 * halted links into t4cpu->s_link_halted, which linkcomms() tests. */
#define DS_RUNNING(v, l)        ((v)->plink[l].ds_status & 0x2u)

static void ds_sync (t9k_vcp_t *v)
{
        uint8_t m = 0;
        int i;

        for (i = 0; i < T9K_NUM_LINKS; i++)
                if (!DS_RUNNING (v, i))
                        m |= (uint8_t)(1u << i);
        t4cpu->s_link_halted = m;
}

static void vcp_update_span (t9k_vcp_t *v)
{
        uint32_t lim = v->space ? v->memstart : T9K_MIN_VIRTUAL_CHAN;

        if (INT32(lim) < INT32(T9K_MIN_VIRTUAL_CHAN))
                lim = T9K_MIN_VIRTUAL_CHAN;
        t4cpu->s_vcp_span = lim - T9K_MIN_EVENT_CHAN;
}

static t9k_vcp_t *vcp_get (void)
{
        t9k_vcp_t *v = t4cpu->s_vcp;
        const char *e;
        int i;

        if (v)
                return v;
        v = calloc (1, sizeof *v);
        if (!v) {
                printf ("-E-EMU9000: out of memory for the VCP\n");
                exit (-1);
        }
        v->state = VCP_WAITING;
        v->memstart = MemStart;
        v->min_invalid = T9K_MIN_VIRTUAL_CHAN;
        for (i = 0; i < T9K_NUM_LINKS; i++) {
                /* T4 links come up in byte-stream mode so that T800 code,
                 * the boot protocol and the host link are unaffected until
                 * software selects virtual mode. */
                v->plink[i].vmode = T9K_VLM_BYTEMODE;
                v->plink[i].max_hdr = 0xFFFFu;
                v->plink[i].ds_status = 0x2u;   /* LinkStarted */
                plink_clear_queues (&v->plink[i]);
        }
        e = getenv ("T4_VCP_DEBUG");
        v->debug = e && *e && *e != '0';
        e = getenv ("T4_EVENT_LOOPBACK");
        v->ev_loopback = e && *e && *e != '0';
        t4cpu->s_vcp = v;
        return v;
}

void t9k_vcp_reset_node (void)
{
        t9k_vcp_t *v = t4cpu->s_vcp;
        const char *e;

        if (v) {
                free (v->vl);
                free (v);
                t4cpu->s_vcp = NULL;
        }
        v = vcp_get ();
        e = getenv ("T4_T9K_CHANNELS");
        if (e && *e && *e != '0')
                v->space = 1;
        vcp_update_span (v);
        ds_sync (v);
}

void t9k_vcp_destroy (struct cpu_t *c)
{
        if (c && c->s_vcp) {
                free (c->s_vcp->vl);
                free (c->s_vcp);
                c->s_vcp = NULL;
        }
}

/* Software has started using the T9000 channel space: event channel words
 * become live (activated, empty) and illegal channel addresses are no
 * longer treated as T800-style memory channels. */
static void vcp_enable_space (t9k_vcp_t *v)
{
        int n;

        if (v->space)
                return;
        v->space = 1;
        for (n = 0; n < 4; n++) {
                writeword (T9K_MIN_EVENT_CHAN + 8u * n, NOTPROC);
                writeword (T9K_MIN_EVENT_CHAN + 8u * n + 4u, NOTPROC);
        }
        vcp_update_span (v);
        VDBG (v, "T9000 channel space enabled (MemStart=#%08X)\n", v->memstart);
}

int t9k_chanspace_active (void)
{
        return t4cpu->s_vcp && t4cpu->s_vcp->space;
}

static uint32_t vcp_num_vlinks (t9k_vcp_t *v)
{
        uint32_t n;

        if (INT32(v->min_invalid) <= INT32(T9K_MIN_VIRTUAL_CHAN))
                return 0;
        n = (v->min_invalid - T9K_MIN_VIRTUAL_CHAN) >> 3;
        return n > MAX_VLINKS ? MAX_VLINKS : n;
}

static t9k_vlink_t *vl_get (t9k_vcp_t *v, uint32_t n)
{
        if (n >= v->nvl) {
                uint32_t nn = v->nvl ? v->nvl : 16, i;
                t9k_vlink_t *p;

                while (nn <= n)
                        nn *= 2;
                if (nn > MAX_VLINKS)
                        nn = MAX_VLINKS;
                p = realloc (v->vl, nn * sizeof *p);
                if (!p) {
                        printf ("-E-EMU9000: out of memory for VLCBs\n");
                        exit (-1);
                }
                for (i = v->nvl; i < nn; i++)
                        vl_clear (&p[i]);
                v->vl = p;
                v->nvl = nn;
        }
        return &v->vl[n];
}

/* ---- channel classification (ISM fig 12.5) ------------------------------ */

enum {
        K_INTERNAL, K_BS_OUT, K_BS_IN, K_EV_IN, K_EV_OUT, K_V_IN, K_V_OUT,
        K_ILLEGAL
};

static int chan_kind (t9k_vcp_t *v, uint32_t a, uint32_t *idx)
{
        *idx = 0;
        if (INT32(a) >= INT32(v->memstart))
                return K_INTERNAL;
        if (a < 0x80000010u) {
                *idx = (a >> 2) & 3u;
                return K_BS_OUT;
        }
        if (a < T9K_MIN_EVENT_CHAN) {
                *idx = (a >> 2) & 3u;
                return K_BS_IN;
        }
        if (a < T9K_MIN_VIRTUAL_CHAN) {
                *idx = (a - T9K_MIN_EVENT_CHAN) >> 3;
                return (a & 4u) ? K_EV_OUT : K_EV_IN;
        }
        if ((a - T9K_MIN_VIRTUAL_CHAN) >> 3 < vcp_num_vlinks (v)) {
                *idx = (a - T9K_MIN_VIRTUAL_CHAN) >> 3;
                return (a & 4u) ? K_V_OUT : K_V_IN;
        }
        return K_ILLEGAL;
}

static uint32_t vin_addr (uint32_t n)  { return T9K_MIN_VIRTUAL_CHAN + 8u * n; }
static uint32_t evin_addr (int n)      { return T9K_MIN_EVENT_CHAN + 8u * (uint32_t)n; }
static uint32_t evout_addr (int n)     { return evin_addr (n) + 4u; }

/* Resource channel data structure: rc.Ptr at +0, rc.Id at +4. */
static uint32_t rc_of (t9k_vcp_t *v, uint32_t chan)
{
        if (INT32(chan) >= INT32(v->memstart))
                return chan + 4u;
        return (chan - T9K_MIN_EVENT_CHAN) + v->ext_rc_base;
}

/* Mirror an external channel's rc.Id mode in the VCP's own state, so the
 * receive path never has to read a resource channel data structure that
 * software has not set up. */
static void rc_set_flag (t9k_vcp_t *v, uint32_t chan, int on)
{
        uint32_t idx;
        int k = chan_kind (v, chan, &idx);

        if (k == K_V_IN) {
                vl_get (v, idx)->rc_mode = (uint8_t)on;
                if (!on)
                        v->vl[idx].in_pend &= (uint8_t)~PEND_CLAIM;
        } else if (k == K_EV_IN) {
                v->ev[idx].rc_mode = (uint8_t)on;
        }
}

/* Unalign: T4 has no separate trap for it, so it is reported through the
 * error flag like IntegerError. */
static int bad_align (t9k_vcp_t *v, uint32_t a, const char *who)
{
        if (0 == (a & 3u))
                return 0;
        VDBG (v, "%s: unaligned address #%08X\n", who, a);
        SETERR ();
        return 1;
}

static void int_error (t9k_vcp_t *v, const char *who, uint32_t a)
{
        VDBG (v, "%s: IntegerError (#%08X)\n", who, a);
        SETERR ();
}

/* ---- scheduling helpers ------------------------------------------------- */

static int is_alt_state (uint32_t st)
{
        return st == ENABLING_P || st == WAITING_P || st == READY_P;
}

/* A guard of the ALT process wdesc has become ready (ISM 8.7). */
static void alt_ready (uint32_t wdesc)
{
        uint32_t slot = WSLOT(DWPTR(wdesc), W_POINTER);
        uint32_t st = word (slot);

        if (!is_alt_state (st) || st == READY_P)
                return;
        writeword (slot, READY_P);
        if (st == WAITING_P)
                t4_wake_alt (wdesc);    /* off its timer list too (p.c) */
}

/* ISM 8.8.2 'claim' on behalf of resource channel chan.  Returns 1 when a
 * server waiting in grant took it (the channel is back in normal mode). */
static int rc_claim (t9k_vcp_t *v, uint32_t chan)
{
        uint32_t rc = rc_of (v, chan);
        uint32_t rds = word (rc);               /* rc.Ptr: the RDS while idle */
        uint32_t srv = word (rds);              /* rds.Proc */

        if (srv != NOTPROC) {
                uint32_t slot = WSLOT(DWPTR(srv), W_POINTER);
                uint32_t st = word (slot);

                if (!is_alt_state (st)) {
                        /* server descheduled in grant: pw.Pointer holds the
                         * identifier store address */
                        writeword (st, word (rc + 4u));
                        writeword (rc + 4u, NOTPROC);
                        writeword (rc, NOTPROC);
                        writeword (rds, NOTPROC);
                        rc_set_flag (v, chan, 0);
                        VDBG (v, "claim #%08X granted to server #%08X\n", chan, srv);
                        t4_wake (srv);
                        return 1;
                }
        }
        writeword (rc, NOTPROC);
        if (word (rds + 4u) == NOTPROC)
                writeword (rds + 4u, chan);
        else
                writeword (rc_of (v, word (rds + 8u)), chan);
        writeword (rds + 8u, chan);
        VDBG (v, "claim #%08X queued on RDS #%08X\n", chan, rds);
        if (srv != NOTPROC)
                alt_ready (srv);                /* enabled by enbg */
        return 0;
}

void t9k_internal_claim (uint32_t chan)
{
        t9k_vcp_t *v = vcp_get ();

        writeword (chan, NOTPROC);
        rc_claim (v, chan);
}

/* ---- physical link queues ----------------------------------------------- */

static void q_push (t9k_vcp_t *v, uint32_t n, int ack, int pri)
{
        t9k_vlink_t *l = &v->vl[n];
        t9k_plink_t *p = &v->plink[l->link];
        int32_t *head = ack ? p->aq_head : p->dq_head;
        int32_t *tail = ack ? p->aq_tail : p->dq_tail;

        if (ack) {
                if (l->in_aq) return;
                l->in_aq = 1; l->aq_pri = (uint8_t)pri; l->aq_next = -1;
        } else {
                if (l->in_dq) return;
                l->in_dq = 1; l->dq_pri = (uint8_t)pri; l->dq_next = -1;
        }
        if (tail[pri] < 0)
                head[pri] = (int32_t)n;
        else if (ack)
                v->vl[tail[pri]].aq_next = (int32_t)n;
        else
                v->vl[tail[pri]].dq_next = (int32_t)n;
        tail[pri] = (int32_t)n;
}

static void q_remove (t9k_vcp_t *v, uint32_t n, int ack)
{
        t9k_vlink_t *l = &v->vl[n];
        t9k_plink_t *p = &v->plink[l->link];
        int pri = ack ? l->aq_pri : l->dq_pri;
        int32_t *head = ack ? p->aq_head : p->dq_head;
        int32_t *tail = ack ? p->aq_tail : p->dq_tail;
        int32_t prev = -1, cur = head[pri];

        if (!(ack ? l->in_aq : l->in_dq))
                return;
        while (cur >= 0 && cur != (int32_t)n) {
                prev = cur;
                cur = ack ? v->vl[cur].aq_next : v->vl[cur].dq_next;
        }
        if (cur >= 0) {
                int32_t next = ack ? l->aq_next : l->dq_next;
                if (prev < 0)
                        head[pri] = next;
                else if (ack)
                        v->vl[prev].aq_next = next;
                else
                        v->vl[prev].dq_next = next;
                if (tail[pri] == (int32_t)n)
                        tail[pri] = prev;
        }
        if (ack) l->in_aq = 0; else l->in_dq = 0;
}

static int link_connected (int link)
{
        return Link[link].Out.schbuf || Link[link].Out.sock >= 0;
}

int t9k_link_virtual (int link)
{
        t9k_vcp_t *v = t4cpu->s_vcp;

        return v && !(v->plink[link].vmode & T9K_VLM_BYTEMODE);
}

int t9k_link_listening (int link)
{
        t9k_vcp_t *v = t4cpu->s_vcp;

        return v && v->state >= VCP_RUNNING && !(v->plink[link].vmode & T9K_VLM_BYTEMODE) &&
               DS_RUNNING (v, link);
}

int t9k_link_tx_pending (int link)
{
        t9k_vcp_t *v = t4cpu->s_vcp;
        t9k_plink_t *p;

        if (!v || v->state < VCP_RUNNING || v->state == VCP_DISCARDING ||
            !DS_RUNNING (v, link))
                return 0;
        p = &v->plink[link];
        return p->dq_head[0] >= 0 || p->dq_head[1] >= 0 ||
               p->aq_head[0] >= 0 || p->aq_head[1] >= 0;
}

/* ---- packet engine ------------------------------------------------------ */

static int build_header (t9k_vcp_t *v, t9k_vlink_t *l, u_char *buf)
{
        if (l->hdr_len <= T9K_MAX_SHORT_HEADER) {
                memcpy (buf, l->hdr, l->hdr_len);
        } else {
                u_char tmp[T9K_MAX_HEADER_LEN];
                u_char *src = bytes_int (v->hdr_area_base + 4u * l->hdr_off,
                                         tmp, l->hdr_len);
                memcpy (buf, src, l->hdr_len);
        }
        return l->hdr_len;
}

static void owe_ack (t9k_vcp_t *v, uint32_t n, int pri)
{
        t9k_vlink_t *l = &v->vl[n];

        l->ack_owed = 1;
        l->ack_prio = (uint8_t)pri;
        if (l->in_active && !l->hdr_null)
                q_push (v, n, 1, pri);
        else
                l->in_pend |= PEND_ACK;
}

static void out_queue_data (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];

        if (l->out_wdesc == NOTPROC || l->out_await_ack || l->in_dq ||
            l->stop_wdesc != NOTPROC)
                return;
        if (v->state == VCP_STOPPING) {
                l->out_active = 0;
                return;
        }
        if (l->out_active && !l->hdr_null)
                q_push (v, n, 0, l->out_prio);
}

/* One transfer of w's VXFER group is done; 1 if it was the last, with
 * v->t_rx set to when the group's latest transfer finished. */
static int grp_done (t9k_vcp_t *v, uint32_t w)
{
        int g;

        for (g = 0; g < 8; g++)
                if (v->grp[g].left && v->grp[g].wdesc == w)
                        break;
        if (g == 8)
                return 1;
        if (v->t_rx > v->grp[g].t)
                v->grp[g].t = v->t_rx;
        if (--v->grp[g].left)
                return 0;
        v->t_rx = v->grp[g].t;
        return 1;
}

static void out_complete (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];
        uint32_t w = l->out_wdesc;

        l->out_wdesc = NOTPROC;
        if (l->out_grp) {
                l->out_grp = 0;
                if (!grp_done (v, w))
                        return;                 /* more of its group to come */
        }
        if (l->out_active) {
                vcp_wake (w, v->t_rx);
        } else
                l->out_sched = w;
}

/* A VXFER output is done for its group once its last packet has left -
 * just now: the buffer is free then.  The VCP takes the final acknowledge
 * itself (rx_ack), and the link's next message waits for it as any next
 * packet does, so only the process stops waiting for the round trip. */
static void out_post (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];
        uint32_t w = l->out_wdesc;
        uint64_t t_rx = v->t_rx;

        l->out_grp = 0;
        l->out_posted = 1;
        l->out_wdesc = NOTPROC;
        v->t_rx = t9k_net_timed () ? t9k_net_sent_end () : 0;
        if (grp_done (v, w)) {
                vcp_wake (w, v->t_rx);
        }
        v->t_rx = t_rx;
}

static void in_complete (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];
        uint32_t w = l->in_wdesc;

        if (l->in_grp)                          /* into the VXFER entry */
                writeword (l->in_cnt_at,
                           l->in_overrun ? T9K_LENGTHERROR_P : l->in_count);
        else if (l->in_vlen)
                writeword (WSLOT(DWPTR(w), W_POINTER),
                           l->in_overrun ? T9K_LENGTHERROR_P : l->in_count);
        l->in_wdesc = NOTPROC;
        l->in_vlen = 0;
        VDBG (v, "vl%u input complete, %u bytes%s\n", n, l->in_count,
              l->in_overrun ? " (too long)" : "");
        if (l->in_grp) {
                l->in_grp = 0;
                if (!grp_done (v, w))
                        return;
        }
        if (l->in_active) {
                vcp_wake (w, v->t_rx);
        } else
                l->in_sched = w;
}

/* Data for the waiting input process on vl n. */
static void in_deliver (t9k_vcp_t *v, uint32_t n, u_char *d, uint32_t len, int last)
{
        t9k_vlink_t *l = &v->vl[n];
        uint32_t take = len;

        if (take > l->in_limit) {
                take = l->in_limit;
                l->in_overrun = 1;
        }
        if (take) {
                writebytes_int (l->in_ptr, d, take);
                l->in_ptr += take;
                l->in_limit -= take;
        }
        l->in_count += len;
        /* "The VCP sends an acknowledge packet as soon as it receives the
         * start of the data packet, provided a process is ready to receive
         * the message" (HRM 11.2.3): the next packet follows with no gap */
        l->t_ack = v->t_head;
        owe_ack (v, n, DPRI(l->in_wdesc));
        if (last)
                in_complete (v, n);
}

/* A non-attributable soft error (HRM 11.9): the packet is discarded and the
 * first one is recorded in VCPStatus (HRM 11.7.2).  Unless the link's
 * LocalizeError bit is set it is signalled to the control system (HRM
 * 13.5.1, reliable link): the CPU is stopped, the VCP sent to 'discarding'
 * (11.8.2) and a VCP error (#02) reported on CLink0 - see t9k_ctl_error().
 * A node with no control link wired and StandAloneMode clear has nothing to
 * report to; there the VCP just discards until reset and the CPU runs on. */
static void vcp_error (t9k_vcp_t *v, int link, uint32_t code, uint32_t hdr)
{
        static const char *const what[] = {
                "header out of range", "header too short",
                "short non-terminal packet", "packet too long"
        };

        if (!(v->status & 1u))
                v->status = 1u | (code << 1) | ((uint32_t)link << 4) | (hdr << 16);
        VDBG (v, "link%d: %s (header %u)\n", link, what[code & 3u], hdr);
        if (v->plink[link].vmode & T9K_VLM_LOCALIZEERROR)
                return;
        if (t9k_ctl_error (T9K_CE_VCP))
                return;
        if (v->state != VCP_DISCARDING)
                printf ("-E-EMU9000: Node%d VCP error on Link%d: %s (header %u); "
                        "VCP discarding until reset\n",
                        nodeid < 0 ? 0 : nodeid, link, what[code & 3u], hdr);
        v->state = VCP_DISCARDING;
}

static void rx_ack (t9k_vcp_t *v, int link, uint32_t n)
{
        t9k_vlink_t *l = vl_get (v, n);
        int posted;

        if (!l->out_await_ack) {
                /* not one of the HRM 11.9 error classes: nothing to ack */
                VDBG (v, "link%d: acknowledge for idle vl%u ignored\n", link, n);
                return;
        }
        posted = l->out_posted;
        l->out_await_ack = 0;
        l->out_posted = 0;
        if (v->t_rx > l->t_data)
                l->t_data = v->t_rx;    /* the next packet may go now */
        if (l->stop_wdesc != NOTPROC) {
                uint32_t s = l->stop_wdesc;

                l->stop_wdesc = NOTPROC;
                l->out_wdesc = NOTPROC;
                l->out_left = 0;
                VDBG (v, "vl%u output stopped\n", n);
                t4_wake (s);
                return;
        }
        if (posted) {                   /* its process went on long ago */
                out_queue_data (v, n);  /* the next message, if one began */
                return;
        }
        if (l->out_last)
                out_complete (v, n);
        else
                out_queue_data (v, n);
}

static void rx_data (t9k_vcp_t *v, int link, uint32_t n, u_char *d, uint32_t len, int last)
{
        t9k_vlink_t *l = vl_get (v, n);

        if (!l->inited) {
                /* undefined in the HRM; refuse rather than write through a
                 * buffer pointer software never set */
                VDBG (v, "link%d: packet for uninitialised vl%u dropped\n", link, n);
                return;
        }
        if (v->state == VCP_STOPPING)
                l->in_active = 0;
        if (l->in_stopping) {
                l->t_ack = v->t_rx;
                owe_ack (v, n, LoPriority);
                return;
        }
        if (l->in_wdesc != NOTPROC && !l->in_alt) {
                in_deliver (v, n, d, len, last);
                return;
        }
        if (l->buf_state) {
                /* the sender must wait for an acknowledge before sending
                 * another packet on this channel */
                VDBG (v, "link%d: unsolicited packet for vl%u with its buffer full\n",
                      link, n);
                return;
        }
        if (len)
                writebytes_int (l->buf, d, len);
        l->buf_len = (uint8_t)len;
        l->buf_state = last ? 1 : 2;
        l->t_buf = v->t_rx;
        VDBG (v, "vl%u: %u bytes buffered%s\n", n, len, last ? " (last)" : "");
        if (l->in_alt && l->in_wdesc != NOTPROC) {
                if (l->in_active)
                        alt_ready (l->in_wdesc);
                else
                        l->in_pend |= PEND_ALT;
        } else if (l->rc_mode) {
                if (l->in_active)
                        rc_claim (v, vin_addr (n));
                else
                        l->in_pend |= PEND_CLAIM;
        }
}

static void rx_packet (t9k_vcp_t *v, int link, u_char *pkt, int n)
{
        t9k_plink_t *p = &v->plink[link];
        int hl = (p->vmode & T9K_VLM_HEADER2) ? 2 : 1;
        uint32_t hdr, vln;
        int dl, tok;

        p->ds_status |= 0x20u;                  /* LinkStatus.TokenReceived */
        if (v->state == VCP_DISCARDING)
                return;                         /* accepted and dropped */
        if (n < hl + 1) {
                /* terminated before the header was complete */
                vcp_error (v, link, ERR_SHORTHDR, n > 1 ? pkt[0] : 0u);
                return;
        }
        hdr = pkt[0] | (hl == 2 ? (uint32_t)pkt[1] << 8 : 0u);
        tok = pkt[n - 1];
        dl = n - hl - 1;
        if (hdr < p->min_hdr || hdr > p->max_hdr) {
                vcp_error (v, link, ERR_RANGE, hdr);
                return;
        }
        vln = (hdr + 0x10000u - p->hdr_off) & 0xFFFFu;
        if (vln >= vcp_num_vlinks (v)) {
                vcp_error (v, link, ERR_RANGE, hdr);
                return;
        }
        if (dl > (int)T9K_MAX_PACKET) {
                vcp_error (v, link, ERR_LONGPKT, hdr);
                return;
        }
        if (tok == T9K_TOK_EOP && dl > 0 && dl < (int)T9K_MAX_PACKET) {
                /* only the last packet of a message may be short */
                vcp_error (v, link, ERR_SHORTPKT, hdr);
                return;
        }
        if (dl == 0 && tok == T9K_TOK_EOP)
                rx_ack (v, link, vln);
        else
                rx_data (v, link, vln, pkt + hl, (uint32_t)dl, tok == T9K_TOK_EOM);
}

void t9k_link_rx (int link)
{
        t9k_vcp_t *v = t4cpu->s_vcp;
        u_char pkt[MAX_DATA];
        int i, n;

        /* "waiting": incoming packets stall at the link inputs; a link
         * halted by an error or a reset takes nothing */
        if (!v || v->state < VCP_RUNNING || !t9k_link_virtual (link) ||
            !DS_RUNNING (v, link))
                return;
        if (!Link[link].In.schbuf && Link[link].In.sock < 0)
                return;
        for (i = 0; i < 16; i++) {
                t9k_net_defer (t9k_net_timed (), 0);
                n = channel_recvP (&Link[link].In, pkt, 0);
                t9k_net_defer (0, 0);
                if (n < 0)
                        break;
                v->t_rx = t9k_net_timed () ? t9k_net_arrival () : 0;
                v->t_head = t9k_net_timed ()
                        ? t9k_net_head ((v->plink[link].vmode & T9K_VLM_HEADER2) ? 2u : 1u)
                        : 0;
                rx_packet (v, link, pkt, n);
        }
        /* acknowledges and follow-on data packets go out at once */
        for (i = 0; i < T9K_NUM_LINKS; i++)
                t9k_link_tx (i);
}

/* Move packets on the virtual links while the CPU is stopped: linkcomms(),
 * which normally does it, only runs with the CPU. */
void t9k_vcp_serve_links (void)
{
        int i;

        for (i = 0; i < T9K_NUM_LINKS; i++)
                if (t9k_link_virtual (i) && t9k_link_listening (i))
                        t9k_link_rx (i);
        for (i = 0; i < T9K_NUM_LINKS; i++)
                t9k_link_tx (i);
}

/* Next virtual link to serve on physical link p: high priority before low,
 * acknowledges and data alternately within a priority (HRM 11.2.3).  On a
 * timed network (T4_T9K_NET) the queued packets become ready at different
 * cycles - an acknowledge once its data started arriving, data once the
 * last packet was acknowledged - and the link sends whichever is ready
 * first; the alternation only breaks ties.  Taking them in queue order let
 * a packet ready late hold the link while one ready early waited. */
static int32_t tx_pick (t9k_vcp_t *v, t9k_plink_t *p, int *is_ack, int *pri)
{
        int pr;

        for (pr = 0; pr < 2; pr++) {
                int a = p->aq_head[pr] >= 0, d = p->dq_head[pr] >= 0;
                int32_t best, n;
                uint64_t t;

                if (!a && !d)
                        continue;
                *pri = pr;
                *is_ack = a && (!d || p->ack_turn[pr]);
                best = *is_ack ? p->aq_head[pr] : p->dq_head[pr];
                if (!t9k_net_timed ())
                        return best;
                t = *is_ack ? v->vl[best].t_ack : v->vl[best].t_data;
                for (n = p->aq_head[pr]; n >= 0; n = v->vl[n].aq_next)
                        if (v->vl[n].t_ack < t) {
                                best = n; t = v->vl[n].t_ack; *is_ack = 1;
                        }
                for (n = p->dq_head[pr]; n >= 0; n = v->vl[n].dq_next)
                        if (v->vl[n].t_data < t) {
                                best = n; t = v->vl[n].t_data; *is_ack = 0;
                        }
                return best;
        }
        return -1;
}

void t9k_link_tx (int link)
{
        t9k_vcp_t *v = t4cpu->s_vcp;
        t9k_plink_t *p;
        u_char pkt[T9K_MAX_HEADER_LEN + T9K_MAX_PACKET + 1];
        int guard;

        if (!v || v->state < VCP_RUNNING || v->state == VCP_DISCARDING ||
            !t9k_link_virtual (link) || !DS_RUNNING (v, link))
                return;
        p = &v->plink[link];
        if (!link_connected (link)) {
                if (!p->warned && t9k_link_tx_pending (link)) {
                        p->warned = 1;
                        printf ("-W-EMU9000: Link%d is in virtual mode but not connected; "
                                "packets stay queued\n", link);
                }
                return;
        }
        for (guard = 0; guard < 64; guard++) {
                int is_ack = 0, pri = 0, h, len;
                int32_t n = tx_pick (v, p, &is_ack, &pri);
                t9k_vlink_t *l;
                uint32_t cnt = 0;
                int last = 0;

                if (n < 0)
                        return;
                l = &v->vl[n];
                h = build_header (v, l, pkt);
                if (is_ack) {
                        pkt[h] = T9K_TOK_EOP;
                        len = h + 1;
                } else {
                        u_char tmp[T9K_MAX_PACKET];
                        u_char *src;

                        cnt = l->out_left < T9K_MAX_PACKET ? l->out_left : T9K_MAX_PACKET;
                        last = l->out_left <= T9K_MAX_PACKET;
                        if (cnt) {
                                src = bytes_int (l->out_ptr, tmp, cnt);
                                memcpy (pkt + h, src, cnt);
                        }
                        pkt[h + cnt] = last ? T9K_TOK_EOM : T9K_TOK_EOP;
                        len = h + (int)cnt + 1;
                }
                /* T4_T9K_NET: a packet leaves when its virtual link allows -
                 * the acknowledge once its data arrived, the data once the
                 * previous packet was acknowledged - and its link is free */
                t9k_net_defer (t9k_net_timed (), is_ack ? l->t_ack : l->t_data);
                len = channel_sendP (&Link[link].Out, pkt, len, 0);
                t9k_net_defer (0, 0);
                if (len < 0)
                        return;                 /* link busy: stays queued */
                p->ack_turn[pri] = !is_ack;
                if (is_ack) {
                        q_remove (v, (uint32_t)n, 1);
                        l->ack_owed = 0;
                } else {
                        q_remove (v, (uint32_t)n, 0);
                        l->out_ptr += cnt;
                        l->out_left -= cnt;
                        l->out_await_ack = 1;
                        l->out_last = (uint8_t)last;
                        if (last && l->out_grp)
                                out_post (v, (uint32_t)n);
                }
                VDBG (v, "link%d tx vl%d %s %u bytes\n", link, n,
                      is_ack ? "ack" : (last ? "EOM" : "EOP"), cnt);
        }
}

static void vcp_kick (t9k_vcp_t *v, int link)
{
        if (v->state == VCP_RUNNING || v->state == VCP_STOPPING)
                t9k_link_tx (link);
}

/* ---- channel activation -------------------------------------------------- */

static void in_activate (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];
        uint8_t pend = l->in_pend;

        l->in_active = 1;
        l->in_pend = 0;
        if ((pend & PEND_ACK) && l->ack_owed)
                q_push (v, n, 1, l->ack_prio);
        if (l->in_sched != NOTPROC) {
                uint32_t w = l->in_sched;
                l->in_sched = NOTPROC;
                t4_wake (w);
        }
        if ((pend & PEND_ALT) && l->in_alt && l->in_wdesc != NOTPROC)
                alt_ready (l->in_wdesc);
        if ((pend & PEND_CLAIM) && l->rc_mode && l->buf_state && l->in_wdesc == NOTPROC)
                rc_claim (v, vin_addr (n));
        vcp_kick (v, l->link);
}

static void out_activate (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = &v->vl[n];

        l->out_active = 1;
        if (l->out_sched != NOTPROC) {
                uint32_t w = l->out_sched;
                l->out_sched = NOTPROC;
                t4_wake (w);
        }
        out_queue_data (v, n);
        vcp_kick (v, l->link);
}

/* ---- event channels (ISM 12.2.3) ---------------------------------------- */

static int ev_active_in (int n)  { return word (evout_addr (n)) != T9K_DEACTIVATED_P; }
static int ev_active_out (int n) { return word (evin_addr (n)) != T9K_DEACTIVATED_P; }

static int ev_in_ready (t9k_vcp_t *v, int n)
{
        return v->ev[n].in_pin && !v->ev[n].in_taken && ev_active_in (n);
}

static void ev_take (t9k_vcp_t *v, int n)
{
        v->ev[n].in_taken = 1;
        ev_set_out (v, n, 1);                   /* acknowledge the request */
}

static void ev_set_out (t9k_vcp_t *v, int n, int level)
{
        if (v->ev[n].out_pin == level)
                return;
        v->ev[n].out_pin = (uint8_t)level;
        VDBG (v, "EventOut%d %s\n", n, level ? "high" : "low");
        if (v->ev_loopback)
                t9k_event_in_pin (n ^ 1, level);
}

/* An input event channel is ready (EventIn high, not yet acknowledged, and
 * activated): complete the waiting input, ready the ALT, or claim the
 * resource.  Also run when a channel holding such an event is activated. */
static void ev_in_event (t9k_vcp_t *v, int n)
{
        uint32_t iw;

        if (!ev_in_ready (v, n))
                return;
        iw = word (evin_addr (n));
        if (iw == T9K_RESCHAN_P) {
                writeword (evin_addr (n), NOTPROC);     /* queued */
                rc_claim (v, evin_addr (n));
        } else if (!IS_SPECIAL(iw)) {
                if (is_alt_state (word (WSLOT(DWPTR(iw), W_POINTER)))) {
                        alt_ready (iw);
                } else {
                        writeword (evin_addr (n), NOTPROC);
                        ev_take (v, n);
                        t4_wake (iw);
                }
        }
}

/* setchmode has activated event channel n: carry out what was held back. */
static void ev_activate (t9k_vcp_t *v, int n, int input)
{
        uint32_t w;

        if (input) {
                ev_in_event (v, n);
                return;
        }
        w = word (evout_addr (n));
        if (IS_SPECIAL(w))
                return;
        if (v->ev[n].out_done) {                /* acknowledged meanwhile */
                v->ev[n].out_done = 0;
                writeword (evout_addr (n), NOTPROC);
                t4_wake (w);
        } else if (!v->ev[n].out_pin) {
                ev_set_out (v, n, 1);           /* the request held back */
        }
}

void t9k_event_in_pin (int n, int level)
{
        t9k_vcp_t *v = vcp_get ();
        int old;
        uint32_t ow;

        if (n < 0 || n > 3)
                return;
        old = v->ev[n].in_pin;
        v->ev[n].in_pin = (uint8_t)(level != 0);
        VDBG (v, "EventIn%d %s\n", n, level ? "high" : "low");
        if (!old && level) {
                /* output channel: the external device acknowledges */
                ow = word (evout_addr (n));
                if (!IS_SPECIAL(ow) && v->ev[n].out_pin) {
                        int act = ev_active_out (n);

                        if (act)
                                writeword (evout_addr (n), NOTPROC);
                        else
                                v->ev[n].out_done = 1;  /* pending (HRM 11.3.1) */
                        ev_set_out (v, n, 0);
                        if (act)
                                t4_wake (ow);
                        return;
                }
                ev_in_event (v, n);
        } else if (old && !level) {
                if (v->ev[n].in_taken) {
                        v->ev[n].in_taken = 0;
                        ev_set_out (v, n, 0);
                }
        }
}

int t9k_event_out_pin (int n)
{
        t9k_vcp_t *v = t4cpu->s_vcp;

        return (v && n >= 0 && n < 4) ? v->ev[n].out_pin : 0;
}

/* ---- communication on external channels --------------------------------- */

/* A transfer is set up for process w, w is descheduled, then the transfer
 * goes: done in that order because it can finish - and wake w - at once.
 * VXFER sets up several and deschedules once. */
static void v_out_begin (t9k_vcp_t *v, uint32_t n, uint32_t src, uint32_t len, uint32_t w)
{
        t9k_vlink_t *l = vl_get (v, n);

        if (l->out_wdesc != NOTPROC)
                int_error (v, "output on a busy virtual channel", vin_addr (n) + 4u);
        l->out_wdesc = w;
        l->out_ptr = src;
        l->out_left = len;
        l->out_prio = (uint8_t)ProcPriority;
        l->out_last = 0;
        /* the first packet goes from here - or, after a posted VXFER
         * output, once that one's final acknowledge is in */
        if (vcp_now () > l->t_data || !l->out_await_ack)
                l->t_data = vcp_now ();
        VDBG (v, "vl%u out %u bytes from #%08X\n", n, len, src);
}

static void v_out_go (t9k_vcp_t *v, uint32_t n)
{
        out_queue_data (v, n);
        vcp_kick (v, vl_get (v, n)->link);
}

static void v_out (t9k_vcp_t *v, uint32_t n, uint32_t src, uint32_t len)
{
        v_out_begin (v, n, src, len, CURWDESC);
        deschedule ();
        v_out_go (v, n);
}

static void v_in_begin (t9k_vcp_t *v, uint32_t n, uint32_t dst, uint32_t max, int kind,
                        uint32_t w)
{
        t9k_vlink_t *l = vl_get (v, n);

        if (l->in_wdesc != NOTPROC && !l->in_alt)
                int_error (v, "input on a busy virtual channel", vin_addr (n));
        l->in_wdesc = w;
        l->in_alt = 0;
        l->in_ptr = dst;
        l->in_limit = max;
        l->in_count = 0;
        l->in_vlen = (uint8_t)(kind == T9K_IO_VARIABLE);
        l->in_overrun = 0;
        VDBG (v, "vl%u in up to %u bytes to #%08X\n", n, max, dst);
}

static void v_in_go (t9k_vcp_t *v, uint32_t n);

static void v_in (t9k_vcp_t *v, uint32_t n, uint32_t dst, uint32_t max, int kind)
{
        v_in_begin (v, n, dst, max, kind, CURWDESC);
        deschedule ();
        v_in_go (v, n);
}

/* VXFER (TVX OPR 0x1FA, transputer-tcc __vxfer): start n transfers on
 * virtual channels at once and wake the process when every one is done.
 * list holds n {channel, buffer, length} words: an output channel sends
 * length bytes; an input channel takes up to length bytes, and the length
 * received goes back over the entry's length (LengthError.p on an
 * overrun), as vin leaves it for ldcnt.  One instruction and one
 * deschedule for the lot, where software needs a process per transfer.
 * An output is posted: it is done once its last packet has left, and the
 * group does not wait for the final acknowledge's round trip (out_post). */
void t9k_vcp_group (uint32_t list, uint32_t n)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t w = CURWDESC, i, idx, chan, e;
        int g, k, started = 0;

        if (n == 0)
                return;
        for (g = 0; g < 8 && v->grp[g].left; g++)
                ;
        if (g == 8) {
                int_error (v, "vxfer: eight groups already waiting", list);
                return;
        }
        for (i = 0; i < n; i++) {
                e = list + 12u * i;
                chan = word (e);
                if (bad_align (v, chan, "vxfer"))
                        continue;
                k = chan_kind (v, chan, &idx);
                if (k == K_V_OUT) {
                        v_out_begin (v, idx, word (e + 4u), word (e + 8u), w);
                        vl_get (v, idx)->out_grp = 1;
                } else if (k == K_V_IN) {
                        v_in_begin (v, idx, word (e + 4u), word (e + 8u), T9K_IO_VARIABLE, w);
                        vl_get (v, idx)->in_grp = 1;
                        vl_get (v, idx)->in_cnt_at = e + 8u;
                } else {
                        int_error (v, "vxfer: not a virtual channel", chan);
                        continue;
                }
                started++;
        }
        if (!started)
                return;
        v->grp[g].wdesc = w;
        v->grp[g].left = (uint32_t)started;
        v->grp[g].t = 0;
        deschedule ();
        for (i = 0; i < n; i++) {
                chan = word (list + 12u * i);
                if (chan & 3u)
                        continue;
                k = chan_kind (v, chan, &idx);
                if (k == K_V_OUT)
                        v_out_go (v, idx);
                else if (k == K_V_IN)
                        v_in_go (v, idx);
        }
}

static void v_in_go (t9k_vcp_t *v, uint32_t n)
{
        t9k_vlink_t *l = vl_get (v, n);

        if (l->buf_state) {
                u_char tmp[T9K_MAX_PACKET];
                u_char *d = bytes_int (l->buf, tmp, l->buf_len);
                uint32_t len = l->buf_len;
                int last = l->buf_state == 1;

                if (d != tmp) {
                        memcpy (tmp, d, len);
                        d = tmp;
                }
                l->buf_state = 0;
                l->buf_len = 0;
                /* taken now, or when it arrived if that is later */
                v->t_rx = vcp_now () > l->t_buf ? vcp_now () : l->t_buf;
                v->t_head = v->t_rx;
                in_deliver (v, n, d, len, last);
        }
        vcp_kick (v, l->link);
}

static void ev_out (t9k_vcp_t *v, int n)
{
        writeword (evout_addr (n), CURWDESC);
        deschedule ();
        if (ev_active_out (n))
                ev_set_out (v, n, 1);           /* request */
}

static void ev_in (t9k_vcp_t *v, int n, int kind)
{
        if (kind == T9K_IO_VARIABLE)
                writeword (WSLOT(WPtr, W_POINTER), 0);  /* no data on events */
        deschedule ();
        if (ev_in_ready (v, n)) {
                ev_take (v, n);
                t4_wake (CURWDESC);
        } else {
                writeword (evin_addr (n), CURWDESC);
        }
}

void t9k_ext_out (uint32_t chan, uint32_t src, uint32_t len, int kind)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t idx;
        int k;

        vcp_enable_space (v);
        if (bad_align (v, chan, "out"))
                return;
        k = chan_kind (v, chan, &idx);
        if (k == K_V_OUT)
                v_out (v, idx, src, len);
        else if (k == K_EV_OUT)
                ev_out (v, (int)idx);
        else
                int_error (v, "out", chan);
        (void)kind;
}

void t9k_ext_in (uint32_t chan, uint32_t dst, uint32_t len, int kind)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t idx;
        int k;

        vcp_enable_space (v);
        if (bad_align (v, chan, "in"))
                return;
        k = chan_kind (v, chan, &idx);
        if (k == K_V_IN)
                v_in (v, idx, dst, len, kind);
        else if (k == K_EV_IN)
                ev_in (v, (int)idx, kind);
        else
                int_error (v, "in", chan);
}

int t9k_ext_enbc (uint32_t chan)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t idx;
        int k;

        vcp_enable_space (v);
        if (bad_align (v, chan, "enbc"))
                return 0;
        k = chan_kind (v, chan, &idx);
        if (k == K_V_IN) {
                t9k_vlink_t *l = vl_get (v, idx);

                if (l->buf_state)
                        return 1;
                l->in_wdesc = CURWDESC;
                l->in_alt = 1;
                return 0;
        }
        if (k == K_EV_IN) {
                if (ev_in_ready (v, (int)idx))
                        return 1;
                writeword (chan, CURWDESC);
                return 0;
        }
        int_error (v, "enbc", chan);
        return 0;
}

int t9k_ext_disc (uint32_t chan)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t idx;
        int k;

        if (bad_align (v, chan, "disc"))
                return 0;
        k = chan_kind (v, chan, &idx);
        if (k == K_V_IN) {
                t9k_vlink_t *l = vl_get (v, idx);

                if (l->in_alt && l->in_wdesc == CURWDESC) {
                        l->in_alt = 0;
                        l->in_wdesc = NOTPROC;
                        l->in_pend &= (uint8_t)~PEND_ALT;
                }
                return l->buf_state != 0;
        }
        if (k == K_EV_IN) {
                if (word (chan) == CURWDESC)
                        writeword (chan, NOTPROC);
                return ev_in_ready (v, (int)idx);
        }
        int_error (v, "disc", chan);
        return 0;
}

uint32_t t9k_ext_resetch (uint32_t chan)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t idx, ret = NOTPROC;
        int k;

        vcp_enable_space (v);
        if (bad_align (v, chan, "resetch"))
                return UNDEFINED;
        k = chan_kind (v, chan, &idx);
        if (k == K_V_IN) {
                t9k_vlink_t *l = vl_get (v, idx);

                if (l->in_wdesc != NOTPROC)
                        ret = l->in_wdesc;
                else if (l->in_sched != NOTPROC)
                        ret = l->in_sched;
                q_remove (v, idx, 1);
                l->in_wdesc = l->in_sched = NOTPROC;
                l->in_alt = l->in_vlen = l->in_overrun = 0;
                l->in_stopping = 0;
                l->buf_state = l->buf_len = 0;
                l->ack_owed = 0;
                l->in_pend = 0;
        } else if (k == K_V_OUT) {
                t9k_vlink_t *l = vl_get (v, idx);

                if (l->out_wdesc != NOTPROC)
                        ret = l->out_wdesc;
                else if (l->out_sched != NOTPROC)
                        ret = l->out_sched;
                q_remove (v, idx, 0);
                l->out_wdesc = l->out_sched = l->stop_wdesc = NOTPROC;
                l->out_await_ack = 0;
                l->out_left = 0;
        } else if (k == K_EV_IN || k == K_EV_OUT) {
                uint32_t w = word (chan);

                if (!IS_SPECIAL(w))
                        ret = w;
                writeword (chan, NOTPROC);
                v->ev[idx].in_taken = 0;
                v->ev[idx].out_done = 0;
                ev_set_out (v, (int)idx, 0);
        } else {
                int_error (v, "resetch", chan);
                return UNDEFINED;
        }
        VDBG (v, "resetch #%08X -> #%08X\n", chan, ret);
        return ret;
}

/* ---- variable-length communication (ISM 8.4.3) -------------------------- */

static void int_vout (t9k_vcp_t *v, uint32_t chan, uint32_t src, uint32_t len)
{
        uint32_t other = word (chan), ow, st, max;

        if (other == T9K_RESCHAN_P) {
                writeword (chan, NOTPROC);
                rc_claim (v, chan);
                other = NOTPROC;
        }
        if (other == NOTPROC) {
                writeword (chan, CURWDESC);
                writeword (WSLOT(WPtr, W_POINTER), src);
                writeword (WSLOT(WPtr, W_LINK), len);           /* pw.Count */
                deschedule ();
                return;
        }
        ow = DWPTR(other);
        st = word (WSLOT(ow, W_POINTER));
        if (is_alt_state (st)) {
                writeword (chan, CURWDESC);
                writeword (WSLOT(WPtr, W_POINTER), src);
                writeword (WSLOT(WPtr, W_LINK), len);
                deschedule ();
                alt_ready (other);
                return;
        }
        /* inputter waiting in vin: pw.Pointer = buffer, pw.Count = maximum */
        max = word (WSLOT(ow, W_LINK));
        if (len <= max) {
                if (len)
                        movebytes_int (st, src, len);
                writeword (WSLOT(ow, W_POINTER), len);          /* pw.Length */
        } else {
                writeword (WSLOT(ow, W_POINTER), T9K_LENGTHERROR_P);
        }
        writeword (chan, NOTPROC);
        t4_wake (other);
}

static void int_vin (uint32_t chan, uint32_t dst, uint32_t max)
{
        uint32_t other = word (chan), ow, src, len;

        if (other == NOTPROC || IS_SPECIAL(other)) {
                writeword (chan, CURWDESC);
                writeword (WSLOT(WPtr, W_POINTER), dst);
                writeword (WSLOT(WPtr, W_LINK), max);
                deschedule ();
                return;
        }
        ow = DWPTR(other);
        src = word (WSLOT(ow, W_POINTER));
        len = word (WSLOT(ow, W_LINK));
        if (len <= max) {
                if (len)
                        movebytes_int (dst, src, len);
                writeword (WSLOT(WPtr, W_POINTER), len);
        } else {
                writeword (WSLOT(WPtr, W_POINTER), T9K_LENGTHERROR_P);
        }
        writeword (chan, NOTPROC);
        t4_wake (other);
}

static void op_vout (t9k_vcp_t *v)
{
        uint32_t len = AReg, chan = BReg, src = CReg, idx;
        int k;

        if (bad_align (v, chan, "vout"))
                return;
        k = chan_kind (v, chan, &idx);
        if (k == K_INTERNAL)
                int_vout (v, chan, src, len);
        else if (k == K_V_OUT || k == K_EV_OUT)
                t9k_ext_out (chan, src, len, T9K_IO_VARIABLE);
        else
                int_error (v, "vout", chan);
}

static void op_vin (t9k_vcp_t *v)
{
        uint32_t max = AReg, chan = BReg, dst = CReg, idx;
        int k;

        if (max == 0xFFFFFFFFu) {
                int_error (v, "vin: maximum is MostPosUnsigned", chan);
                return;
        }
        if (bad_align (v, chan, "vin"))
                return;
        k = chan_kind (v, chan, &idx);
        if (k == K_INTERNAL)
                int_vin (chan, dst, max);
        else if (k == K_V_IN || k == K_EV_IN)
                t9k_ext_in (chan, dst, max, T9K_IO_VARIABLE);
        else
                int_error (v, "vin", chan);
}

/* ---- resource channels (ISM 8.8, 12.7) ---------------------------------- */

static int rc_chan (t9k_vcp_t *v, uint32_t chan, uint32_t *idx, const char *who)
{
        int k;

        if (bad_align (v, chan, who))
                return -1;
        k = chan_kind (v, chan, idx);
        if (k == K_INTERNAL || k == K_V_IN || k == K_EV_IN)
                return k;
        int_error (v, who, chan);
        return -1;
}

static void rc_append (t9k_vcp_t *v, uint32_t rds, uint32_t chan)
{
        writeword (rc_of (v, chan), NOTPROC);
        if (word (rds + 4u) == NOTPROC)
                writeword (rds + 4u, chan);
        else
                writeword (rc_of (v, word (rds + 8u)), chan);
        writeword (rds + 8u, chan);
}

static void op_mkrc (t9k_vcp_t *v)
{
        uint32_t chan = AReg, rds = BReg, id = CReg, idx, rc;
        int k = rc_chan (v, chan, &idx, "mkrc");

        if (k < 0)
                return;
        rc = rc_of (v, chan);
        if (word (rc + 4u) != NOTPROC)
                return;                         /* already in resource mode */
        writeword (rc + 4u, id);
        if (k == K_INTERNAL) {
                if (word (chan) == NOTPROC) {
                        writeword (rc, rds);
                        writeword (chan, T9K_RESCHAN_P);        /* idle */
                } else {
                        rc_append (v, rds, chan);               /* queued */
                }
                return;
        }
        vcp_enable_space (v);
        writeword (rc, rds);
        rc_set_flag (v, chan, 1);
        if (k == K_V_IN) {
                t9k_vlink_t *l = &v->vl[idx];

                if (l->buf_state && l->in_wdesc == NOTPROC) {
                        if (l->in_active)
                                rc_claim (v, chan);
                        else
                                l->in_pend |= PEND_CLAIM;
                }
        } else if (ev_in_ready (v, (int)idx)) {
                rc_claim (v, chan);
        } else if (word (chan) == NOTPROC) {
                writeword (chan, T9K_RESCHAN_P);
        }
}

static void op_unmkrc (t9k_vcp_t *v)
{
        uint32_t chan = AReg, idx, rc;
        int k = rc_chan (v, chan, &idx, "unmkrc");

        AReg = BReg;
        BReg = CReg;
        if (k < 0)
                return;
        rc = rc_of (v, chan);
        if (word (rc + 4u) == NOTPROC)
                return;
        writeword (rc + 4u, NOTPROC);
        if (k != K_V_IN && word (chan) == T9K_RESCHAN_P)
                writeword (chan, NOTPROC);
        rc_set_flag (v, chan, 0);
}

static void op_grant (t9k_vcp_t *v)
{
        uint32_t rds = AReg, idaddr = BReg, front, rc, next;

        if (bad_align (v, rds, "grant") || bad_align (v, idaddr, "grant"))
                return;
        front = word (rds + 4u);
        if (front == NOTPROC) {
                writeword (rds, CURWDESC);              /* rds.Proc */
                writeword (WSLOT(WPtr, W_POINTER), idaddr);
                deschedule ();
                return;
        }
        rc = rc_of (v, front);
        next = word (rc);
        writeword (idaddr, word (rc + 4u));
        writeword (rc + 4u, NOTPROC);
        writeword (rc, NOTPROC);                        /* reverse channel */
        rc_set_flag (v, front, 0);
        writeword (rds + 4u, front == word (rds + 8u) ? NOTPROC : next);
}

static void op_enbg (t9k_vcp_t *v)
{
        if (AReg != 0) {
                if (bad_align (v, BReg, "enbg"))
                        return;
                writeword (BReg, CURWDESC);
                if (word (BReg + 4u) != NOTPROC)
                        writeword (WSLOT(WPtr, W_POINTER), READY_P);
        }
        BReg = CReg;
}

static void op_disg (t9k_vcp_t *v)
{
        uint32_t rds = CReg;

        if (BReg == 0) {
                AReg = 0;
                return;
        }
        if (bad_align (v, rds, "disg"))
                return;
        if (word (rds) == NOTPROC) {
                AReg = 0;                       /* already disabled */
                return;
        }
        writeword (rds, NOTPROC);
        if (word (rds + 4u) == NOTPROC) {
                AReg = 0;
        } else if (word (WSLOT(WPtr, W_TEMP)) == NONESELECTED_O) {
                writeword (WSLOT(WPtr, W_TEMP), AReg);
                AReg = 1;
        } else {
                AReg = 0;
        }
}

static void op_erdsq (t9k_vcp_t *v)
{
        uint32_t rds = AReg;

        if (bad_align (v, rds, "erdsq"))
                return;
        CReg = BReg;
        AReg = word (rds + 4u);
        BReg = word (rds + 8u);
        writeword (rds + 4u, NOTPROC);
}

static void op_irdsq (t9k_vcp_t *v)
{
        uint32_t rds = AReg, front = BReg, back = CReg, old;

        if (bad_align (v, rds, "irdsq"))
                return;
        old = word (rds + 4u);
        writeword (rds + 4u, front);
        if (old == NOTPROC)
                writeword (rds + 8u, back);
        else
                writeword (rc_of (v, back), old);
}

static void op_ldresptr (t9k_vcp_t *v)
{
        uint32_t idx;

        if (rc_chan (v, AReg, &idx, "ldresptr") < 0)
                return;
        AReg = word (rc_of (v, AReg));
}

static void op_stresptr (t9k_vcp_t *v)
{
        uint32_t idx;

        if (rc_chan (v, AReg, &idx, "stresptr") >= 0)
                writeword (rc_of (v, AReg), BReg);
        AReg = CReg;
}

/* ---- virtual link control blocks (ISM 12.6) ----------------------------- */

static t9k_vlink_t *vl_of (t9k_vcp_t *v, uint32_t chan, uint32_t *n, int input_only,
                           const char *who)
{
        int k;

        if (bad_align (v, chan, who))
                return NULL;
        k = chan_kind (v, chan, n);
        if (k == K_V_IN || (k == K_V_OUT && !input_only)) {
                vcp_enable_space (v);
                return vl_get (v, *n);
        }
        int_error (v, who, chan);
        return NULL;
}

static void op_initvlcb (t9k_vcp_t *v)
{
        uint32_t n, buf = BReg;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 0, "initvlcb");

        AReg = CReg;
        if (!l || bad_align (v, buf, "initvlcb"))
                return;
        q_remove (v, n, 0);
        q_remove (v, n, 1);
        vl_clear (l);
        l->inited = 1;
        l->buf = buf;
        VDBG (v, "initvlcb vl%u buffer #%08X\n", n, buf);
}

static void op_sethdr (t9k_vcp_t *v)
{
        uint32_t n, lk = BReg, off = CReg;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 0, "sethdr");

        if (!l)
                return;
        if (lk <= 3u) {
                if (off != T9K_NULOFFSET && off > T9K_MAX_HEADER_OFFSET) {
                        int_error (v, "sethdr: header offset", off);
                        return;
                }
                if (l->link != lk) {
                        int ind = l->in_dq, ina = l->in_aq;

                        q_remove (v, n, 0);
                        q_remove (v, n, 1);
                        l->link = (uint8_t)lk;
                        if (ind) q_push (v, n, 0, l->dq_pri);
                        if (ina) q_push (v, n, 1, l->aq_pri);
                }
                if (off != T9K_NULOFFSET)
                        l->hdr_off = off;
        } else if (lk == T9K_NULHEADER) {
                if (l->in_active || l->out_active) {
                        int_error (v, "sethdr: null header on an active channel", AReg);
                        return;
                }
                l->hdr_null = 1;
                l->hdr_len = 0;
        } else {
                int_error (v, "sethdr: link number", lk);
        }
}

static void op_writehdr (t9k_vcp_t *v)
{
        uint32_t n, len = BReg, src = CReg;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 0, "writehdr");
        u_char tmp[T9K_MAX_HEADER_LEN], *p;

        if (!l)
                return;
        if (len == 0 || len > T9K_MAX_HEADER_LEN ||
            (len > T9K_MAX_SHORT_HEADER && l->hdr_off == T9K_NULOFFSET)) {
                int_error (v, "writehdr: length", len);
                return;
        }
        p = bytes_int (src, tmp, len);
        if (len <= T9K_MAX_SHORT_HEADER)
                memcpy (l->hdr, p, len);
        else
                writebytes_int (v->hdr_area_base + 4u * l->hdr_off, p, len);
        l->hdr_len = (uint8_t)len;
        l->hdr_null = 0;
        VDBG (v, "writehdr vl%u len %u first #%02X link %u\n", n, len, p[0], l->link);
}

static void op_readhdr (t9k_vcp_t *v)
{
        uint32_t n, len = BReg, dst = CReg;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 0, "readhdr");
        u_char tmp[T9K_MAX_HEADER_LEN], *p;

        if (!l)
                return;
        if (l->hdr_null || len != l->hdr_len) {
                int_error (v, "readhdr: length", len);
                return;
        }
        if (len <= T9K_MAX_SHORT_HEADER) {
                writebytes_int (dst, l->hdr, len);
        } else {
                p = bytes_int (v->hdr_area_base + 4u * l->hdr_off, tmp, len);
                if (p != tmp)
                        memcpy (tmp, p, len);
                writebytes_int (dst, tmp, len);
        }
}

static void op_insphdr (t9k_vcp_t *v)
{
        uint32_t n;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 0, "insphdr");

        if (!l)
                return;
        AReg = l->link;
        if (l->hdr_null) {
                BReg = T9K_NULHEADER;
                CReg = T9K_NULOFFSET;
        } else {
                BReg = l->hdr_len;
                CReg = l->hdr_len > T9K_MAX_SHORT_HEADER ? l->hdr_off : T9K_NULOFFSET;
        }
}

static void op_readbfr (t9k_vcp_t *v)
{
        uint32_t n;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 1, "readbfr");

        if (!l)
                return;
        AReg = l->buf;
        BReg = l->buf_state ? l->buf_len : 0;
        CReg = l->buf_state;
}

static void op_swapbfr (t9k_vcp_t *v)
{
        uint32_t n, buf = BReg;
        t9k_vlink_t *l = vl_of (v, AReg, &n, 1, "swapbfr");

        if (!l || bad_align (v, buf, "swapbfr"))
                return;
        AReg = l->buf;
        l->buf = buf;
        BReg = CReg;
}

/* ---- channel mode, stop, status (ISM 12.8, 12.9) ------------------------ */

static void op_setchmode (t9k_vcp_t *v)
{
        uint32_t chan = AReg, on = BReg, idx;
        int k;

        AReg = CReg;
        if (bad_align (v, chan, "setchmode"))
                return;
        vcp_enable_space (v);
        if (on > 1u) {
                int_error (v, "setchmode: not a boolean", on);
                return;
        }
        k = chan_kind (v, chan, &idx);
        switch (k) {
        case K_V_IN:
        case K_V_OUT: {
                t9k_vlink_t *l = vl_get (v, idx);

                if (!on) {
                        if (k == K_V_IN) {
                                l->in_active = 0;
                                if (l->in_aq) {
                                        q_remove (v, idx, 1);
                                        l->in_pend |= PEND_ACK;
                                }
                        } else {
                                l->out_active = 0;
                                q_remove (v, idx, 0);
                        }
                } else if (l->hdr_null) {
                        int_error (v, "setchmode: null header", chan);
                } else if (v->state != VCP_STOPPING) {
                        if (k == K_V_IN)
                                in_activate (v, idx);
                        else
                                out_activate (v, idx);
                }
                break;
        }
        case K_EV_IN:
        case K_EV_OUT:
                /* activation lives in the reverse channel word (HRM 12.2) */
                writeword (chan ^ 4u, on ? NOTPROC : T9K_DEACTIVATED_P);
                if (on)
                        ev_activate (v, (int)idx, k == K_EV_IN);
                break;
        case K_BS_IN:
        case K_BS_OUT: {
                t9k_plink_t *p = &v->plink[idx];

                if (on)
                        p->ds_status = (p->ds_status & ~0x19u) | 0x2u;  /* started */
                else
                        p->ds_status &= ~0x3Bu;                         /* reset */
                ds_sync (v);
                break;
        }
        default:
                int_error (v, "setchmode", chan);
        }
}

static void op_stopch (t9k_vcp_t *v)
{
        uint32_t chan = AReg, n;
        t9k_vlink_t *l = vl_of (v, chan, &n, 0, "stopch");

        if (!l)
                return;
        if (!(chan & 4u)) {
                l->in_stopping = 1;
                if (l->buf_state) {
                        /* acknowledge, and so quiesce, the buffered packet */
                        l->buf_state = l->buf_len = 0;
                        l->t_ack = vcp_now ();
                        owe_ack (v, n, ProcPriority);
                        vcp_kick (v, l->link);
                }
                return;
        }
        q_remove (v, n, 0);
        if (l->out_await_ack) {
                l->stop_wdesc = CURWDESC;
                deschedule ();
        } else {
                l->out_wdesc = NOTPROC;
                l->out_left = 0;
        }
}

static void op_ldchstatus (t9k_vcp_t *v)
{
        uint32_t chan = AReg, idx, st = 0, b = NOTPROC, c = CReg;
        int k;

        if (bad_align (v, chan, "ldchstatus"))
                return;
        k = chan_kind (v, chan, &idx);
        switch (k) {
        case K_V_IN: {
                t9k_vlink_t *l = vl_get (v, idx);

                st = 0x80000000u
                   | (l->in_active ? 0x1u : 0u)
                   | (l->in_stopping ? 0x2u : 0u)
                   | (l->rc_mode ? 0x8u : 0u)
                   | ((l->buf_state || l->ack_owed) ? 0x10u : 0u)
                   | (l->in_sched != NOTPROC ? 0x20u : 0u);
                if (l->in_stopping) {
                        b = NOTPROC;
                } else if (l->rc_mode) {
                        b = word (rc_of (v, chan));
                        c = word (rc_of (v, chan) + 4u);
                } else {
                        b = l->in_wdesc;
                }
                break;
        }
        case K_V_OUT: {
                t9k_vlink_t *l = vl_get (v, idx);

                st = 0x80000000u
                   | (l->out_active ? 0x1u : 0u)
                   | (l->stop_wdesc != NOTPROC ? 0x2u : 0u)
                   | ((l->out_await_ack || l->in_dq) ? 0x10u : 0u)
                   | (l->out_sched != NOTPROC ? 0x20u : 0u);
                b = l->stop_wdesc != NOTPROC ? NOTPROC : l->out_wdesc;
                break;
        }
        case K_EV_IN:
        case K_EV_OUT: {
                uint32_t w = word (chan);
                int active = word (chan ^ 4u) != T9K_DEACTIVATED_P;
                int in = (k == K_EV_IN);

                st = (active ? 0x1u : 0u)
                   | ((in && v->ev[idx].rc_mode) ? 0x8u : 0u)
                   | ((in && v->ev[idx].in_pin && !v->ev[idx].in_taken) ? 0x10u : 0u);
                if (in && v->ev[idx].rc_mode) {
                        b = word (rc_of (v, chan));
                        c = word (rc_of (v, chan) + 4u);
                } else {
                        b = IS_SPECIAL(w) ? NOTPROC : w;
                }
                break;
        }
        case K_BS_IN:
        case K_BS_OUT: {
                uint32_t ds = v->plink[idx].ds_status, w = word (chan);

                st = ((ds & 0x19u) ? 0x1u : 0u) | (ds & 0x18u);
                b = IS_SPECIAL(w) ? NOTPROC : w;
                break;
        }
        default:
                int_error (v, "ldchstatus", chan);
                return;
        }
        AReg = st;
        BReg = b;
        CReg = c;
}

static void op_chantype (t9k_vcp_t *v)
{
        uint32_t idx;
        int k;

        if (bad_align (v, AReg, "chantype"))
                return;
        k = chan_kind (v, AReg, &idx);
        if (k == K_ILLEGAL)
                int_error (v, "chantype", AReg);
        else
                AReg = (k == K_INTERNAL) ? 1u : 0u;
}

/* ---- configuration space (HRM ch.15) ------------------------------------ */

/* Registers that are documented but have no behaviour in T4 (PMI, cache):
 * stored so that software reads back what it wrote.  System services and
 * the control links are t9000_ctl.c's. */
static int misc_reg (uint32_t a, int *writable, int *readable)
{
        *writable = *readable = 1;
        if ((a >= 0x0202u && a <= 0x0213u) || a == 0x0600u ||
            (a >= 0x0401u && a <= 0x0416u) ||
            (a >= 0x2001u && a <= 0x2005u)) {
                if (a == 0x0212u || a == 0x0416u || a == 0x2002u || a == 0x2005u)
                        *readable = 0;          /* action registers */
                if (a == 0x0213u)
                        *writable = 0;
                return 1;
        }
        return 0;
}

static uint32_t *misc_slot (t9k_vcp_t *v, uint32_t a)
{
        int i;

        for (i = 0; i < v->nmisc; i++)
                if (v->misc_addr[i] == a)
                        return &v->misc_val[i];
        if (v->nmisc == NMISC)
                return NULL;
        v->misc_addr[v->nmisc] = a;
        v->misc_val[v->nmisc] = 0;
        return &v->misc_val[v->nmisc++];
}

static void vcp_command (t9k_vcp_t *v, uint32_t cmd)
{
        int i;
        uint32_t n;

        v->command = cmd & 7u;
        if (cmd & T9K_VCPCMD_RESET) {
                for (i = 0; i < T9K_NUM_LINKS; i++)
                        plink_clear_queues (&v->plink[i]);
                for (n = 0; n < v->nvl; n++) {
                        v->vl[n].in_dq = v->vl[n].in_aq = 0;
                        v->vl[n].out_await_ack = 0;
                }
                v->status = 0;
                v->state = VCP_RESETTING;
        } else if (cmd & T9K_VCPCMD_START) {
                v->state = v->stop_sig ? VCP_STOPPING : VCP_RUNNING;
        } else if (cmd & T9K_VCPCMD_STOP) {
                v->state = VCP_STOPPING;
        } else {
                v->state = VCP_WAITING;
        }
        VDBG (v, "VCPCommand #%X -> state %d\n", cmd, v->state);
        if (v->state >= VCP_RUNNING)
                for (i = 0; i < T9K_NUM_LINKS; i++)
                        t9k_link_tx (i);
}

/* Signals from the control unit (HRM 11.8.2), and its resets (HRM table
 * 8.10: level 2 clears the VCP and the links but keeps their configuration,
 * level 1 also returns that to its power-on values). */
void t9k_vcp_signal (int sig)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t n;
        int i;

        switch (sig) {
        case T9K_VCPSIG_STOP:
                if (v->state == VCP_RUNNING)
                        v->state = VCP_STOPPING;
                else if (v->state == VCP_WAITING)
                        v->stop_sig = 1;        /* stopping once started */
                break;
        case T9K_VCPSIG_ERROR:
                if (v->state != VCP_RESETTING)
                        v->state = VCP_DISCARDING;
                break;
        case T9K_VCPSIG_RESET2:
                for (n = 0; n < v->nvl; n++)
                        vl_clear (&v->vl[n]);
                for (i = 0; i < T9K_NUM_LINKS; i++) {
                        t9k_plink_t *p = &v->plink[i];

                        plink_clear_queues (p);
                        p->ack_turn[0] = p->ack_turn[1] = 0;
                        p->ds_status = 0x2u;    /* link engine restarted, as at power-on */
                        p->ds_wlock = 0;
                        p->warned = 0;
                }
                ds_sync (v);
                memset (v->ev, 0, sizeof v->ev);
                v->status = 0;
                v->command = 0;
                v->chan_wlock = 0;
                v->stop_sig = 0;
                /* 'resetting' for a fixed number of cycles, then 'waiting' */
                v->state = VCP_WAITING;
                break;
        case T9K_VCPSIG_RESET1:
                t9k_vcp_reset_node ();
                v = vcp_get ();
                break;
        }
        VDBG (v, "control unit signal %d -> state %d\n", sig, v->state);
}

/* A parity or disconnect error on data link n's input (HRM 13.5): the link
 * records it in LinkNStatus and resets, losing what it held, and carries
 * nothing until started again.  -1: the link was not running, so there is
 * no error; 1: LinkNMode.LocalizeError keeps it from the control unit;
 * 0: report it. */
int t9k_ds_link_error (int n, int disconnect)
{
        t9k_vcp_t *v = vcp_get ();
        t9k_plink_t *p = &v->plink[n];
        Channel *in = &Link[n].In;
        u_char tmp[MAX_DATA];

        if (!DS_RUNNING (v, n))
                return -1;
        p->ds_status = (p->ds_status & ~0x2u) | 0x1u | (disconnect ? 0x10u : 0x8u);
        ds_sync (v);
        if (in->schbuf || in->sock >= 0)
                (void)channel_recvP (in, tmp, 0);
        in->npend = 0;
        VDBG (v, "link%d %s error, link halted\n", n, disconnect ? "disconnect" : "parity");
        return (p->ds_mode & 0x8u) != 0;
}

/* resetch on a byte-stream channel resets and restarts its link (HRM
 * 13.7.2).  With no VCP (T800 code) no link can have stopped, and there is
 * no need to create one. */
void t9k_ds_link_restart (int n)
{
        t9k_vcp_t *v = t4cpu->s_vcp;

        if (!v)
                return;
        v->plink[n].ds_status = 0x2u;
        ds_sync (v);
}

/* Link0-3 DS-link registers: #8n01 Mode, #8n02 Command, #8n03 Status,
 * #8n04 WriteLock. */
static int ds_reg (uint32_t a, int *link, int *reg)
{
        if ((a & 0xFC00u) != 0x8000u || (a & 0xFFu) < 1u || (a & 0xFFu) > 4u ||
            ((a >> 8) & 0xFu) > 3u)
                return 0;
        *link = (int)((a >> 8) & 3u);
        *reg = (int)(a & 0xFFu);
        return 1;
}

static int conf_read (t9k_vcp_t *v, uint32_t a, uint32_t *val)
{
        int w, r, lk, reg;
        uint32_t *s;

        if (a >= T9K_CR_VCPLINK0MODE && a < T9K_CR_VCPLINK0MODE + 16u) {
                t9k_plink_t *p = &v->plink[(a - T9K_CR_VCPLINK0MODE) >> 2];

                switch ((a - T9K_CR_VCPLINK0MODE) & 3u) {
                case 0: *val = p->vmode; break;
                case 1: *val = p->max_hdr; break;
                case 2: *val = p->min_hdr; break;
                default: *val = p->hdr_off; break;
                }
                return 1;
        }
        if (ds_reg (a, &lk, &reg)) {
                t9k_plink_t *p = &v->plink[lk];

                switch (reg) {
                case 1: *val = p->ds_mode; return 1;
                case 3: *val = p->ds_status; return 1;
                case 4: *val = p->ds_wlock; return 1;
                default: return 0;      /* LinkNCommand is write-only */
                }
        }
        switch (a) {
        case T9K_CR_MEMSTART:           *val = v->memstart; return 1;
        case T9K_CR_MININVALIDCHAN:     *val = v->min_invalid; return 1;
        case T9K_CR_EXTERNALRCBASE:     *val = v->ext_rc_base; return 1;
        case T9K_CR_HDRAREABASE:        *val = v->hdr_area_base; return 1;
        case T9K_CR_CHANWRITELOCK:      *val = v->chan_wlock; return 1;
        case T9K_CR_INITIALIPTR:        *val = v->initial_iptr; return 1;
        case T9K_CR_INITIALWPTR:        *val = v->initial_wptr; return 1;
        case T9K_CR_REASON:             *val = v->reason; return 1;
        case T9K_CR_EMIBADADDRESS:      *val = v->emi_bad; return 1;
        case T9K_CR_VCPSTATUS:          *val = v->status; return 1;
        case T9K_CR_VCPCOMMAND:         *val = v->command; return 1;
        case T9K_CR_DEVICEID:           *val = t4_product_identity () & 0xFFFFu; return 1;
        case T9K_CR_DEVICEREVISION:     *val = 0; return 1;
        }
        if ((r = t9k_ctl_conf_read (a, val)) != 0)
                return r > 0;
        if (misc_reg (a, &w, &r) && r) {
                s = misc_slot (v, a);
                *val = s ? *s : 0;
                return 1;
        }
        return 0;
}

/* by_cpu: an stconf, which write locks stop; the control unit's CPoke is
 * not (HRM 15.3: "Reads are not affected nor are writes by the control
 * unit").  A lock stops every CPU write to its subsystem, its own included,
 * so only the control unit or a reset (HRM 8.7.6) clears it. */
static int conf_write (t9k_vcp_t *v, uint32_t a, uint32_t val, int by_cpu)
{
        int w, r, lk, reg;
        uint32_t *s;

        /* ChanWriteLock guards the CPU, VCP and scheduler registers */
        if (by_cpu && v->chan_wlock &&
            ((a >= 0x0100u && a <= 0x01FFu) || (a >= 0x0800u && a <= 0x09FFu) ||
             (a >= 0x4000u && a <= 0x41FFu) || (a >= 0x4900u && a <= 0x49FFu)))
                return 0;
        if ((r = t9k_ctl_conf_write (a, val, by_cpu)) != 0)
                return r > 0;
        if (a >= T9K_CR_VCPLINK0MODE && a < T9K_CR_VCPLINK0MODE + 16u) {
                int n = (int)((a - T9K_CR_VCPLINK0MODE) >> 2);
                t9k_plink_t *p = &v->plink[n];

                vcp_enable_space (v);
                switch ((a - T9K_CR_VCPLINK0MODE) & 3u) {
                case 0:
                        p->vmode = val & 7u;
                        VDBG (v, "link%d %s mode, %d-byte headers\n", n,
                              (val & T9K_VLM_BYTEMODE) ? "byte-stream" : "virtual",
                              (val & T9K_VLM_HEADER2) ? 2 : 1);
                        break;
                case 1: p->max_hdr = val & 0xFFFFu; break;
                case 2: p->min_hdr = val & 0xFFFFu; break;
                default: p->hdr_off = val & 0xFFFFu; break;
                }
                return 1;
        }
        if (ds_reg (a, &lk, &reg)) {
                t9k_plink_t *p = &v->plink[lk];

                if (by_cpu && p->ds_wlock)
                        return 0;
                switch (reg) {
                case 1: p->ds_mode = val & 0x1Fu; return 1;
                case 2: {
                        int was = DS_RUNNING (v, lk) != 0;

                        if (val & 1u) {         /* ResetLink: buffers emptied */
                                Channel *in = &Link[lk].In;
                                u_char tmp[MAX_DATA];

                                p->ds_status = 0;
                                if (in->schbuf || in->sock >= 0)
                                        (void)channel_recvP (in, tmp, 0);
                                in->npend = 0;
                        }
                        if (val & 2u)           /* StartLink */
                                p->ds_status |= 0x2u;   /* LinkStarted */
                        /* the far end sees outputs held low as a disconnect,
                         * and a bad parity bit as a parity error (HRM 13.6) */
                        if (val & 4u) {         /* ResetOutput */
                                p->ds_status = (p->ds_status & ~0x2u) | 0x4u;
                                if (was)
                                        t9k_line_signal (lk, 1);
                        }
                        if ((val & 8u) && DS_RUNNING (v, lk))   /* WrongParity */
                                t9k_line_signal (lk, 0);
                        ds_sync (v);
                        return 1;
                }
                case 4: p->ds_wlock = val & 1u; return 1;
                default: return 0;      /* LinkNStatus is read-only */
                }
        }
        switch (a) {
        case T9K_CR_MEMSTART:
                v->memstart = val & ~3u;
                vcp_enable_space (v);
                vcp_update_span (v);
                return 1;
        case T9K_CR_MININVALIDCHAN:
                v->min_invalid = val & ~3u;
                vcp_enable_space (v);
                return 1;
        case T9K_CR_EXTERNALRCBASE:
                v->ext_rc_base = val & ~3u;
                vcp_enable_space (v);
                return 1;
        case T9K_CR_HDRAREABASE:
                v->hdr_area_base = val & ~3u;
                vcp_enable_space (v);
                return 1;
        case T9K_CR_CHANWRITELOCK:      v->chan_wlock = val & 1u; return 1;
        case T9K_CR_INITIALIPTR:        v->initial_iptr = val; return 1;
        case T9K_CR_INITIALWPTR:        v->initial_wptr = val & ~3u; return 1;
        case T9K_CR_REASON:             v->reason = val; return 1;
        case T9K_CR_EMIBADADDRESS:      v->emi_bad = val; return 1;
        case T9K_CR_VCPSTATUS:          v->status = 0; return 1;
        case T9K_CR_VCPCOMMAND:
                vcp_enable_space (v);
                vcp_command (v, val);
                return 1;
        }
        if (misc_reg (a, &w, &r) && w) {
                s = misc_slot (v, a);
                if (s)
                        *s = val;
                return 1;
        }
        return 0;
}

int t9k_cfg_read (uint32_t a, uint32_t *val)
{
        return conf_read (vcp_get (), a, val);
}

int t9k_cfg_write (uint32_t a, uint32_t val, int by_cpu)
{
        return conf_write (vcp_get (), a, val, by_cpu);
}

/* ---- dispatcher ---------------------------------------------------------- */

int t9k_vcp_exec (uint32_t op)
{
        t9k_vcp_t *v = vcp_get ();
        uint32_t val;

        switch (op) {
        case 0x7Eu:                             /* ldmemstartval */
                CReg = BReg;
                BReg = AReg;
                AReg = v->memstart;
                return 1;
        case 0xBCu: op_insphdr (v); return 1;
        case 0xBDu: op_readbfr (v); return 1;
        case 0xBEu:                             /* ldconf */
                if (conf_read (v, AReg, &val))
                        AReg = val;
                else
                        int_error (v, "ldconf", AReg);
                return 1;
        case 0xBFu:                             /* stconf */
                if (!conf_write (v, AReg, BReg, 1))
                        int_error (v, "stconf", AReg);
                AReg = CReg;
                return 1;
        case 0xC0u:                             /* ldcnt */
                val = word (WSLOT(WPtr, W_POINTER));
                if (val == T9K_LENGTHERROR_P)
                        SETERR ();
                CReg = BReg;
                BReg = AReg;
                AReg = val;
                return 1;
        case 0xC3u: op_ldchstatus (v); return 1;
        case 0xC9u: op_chantype (v); return 1;
        case 0xFFFFFFEEu: op_stopch (v); return 1;
        case 0xFFFFFFEDu: op_vout (v); return 1;
        case 0xFFFFFFECu: op_vin (v); return 1;
        case 0xFFFFFFE9u: op_swapbfr (v); return 1;
        case 0xFFFFFFE8u: op_sethdr (v); return 1;
        case 0xFFFFFFE7u: op_setchmode (v); return 1;
        case 0xFFFFFFE6u: op_initvlcb (v); return 1;
        case 0xFFFFFFE5u: op_writehdr (v); return 1;
        case 0xFFFFFFE4u: op_readhdr (v); return 1;
        case 0xFFFFFFE3u: op_disg (v); return 1;
        case 0xFFFFFFE2u: op_enbg (v); return 1;
        case 0xFFFFFFE1u: op_grant (v); return 1;
        case 0xFFFFFFDDu: op_unmkrc (v); return 1;
        case 0xFFFFFFDCu: op_mkrc (v); return 1;
        case 0xFFFFFFDBu: op_irdsq (v); return 1;
        case 0xFFFFFFDAu: op_erdsq (v); return 1;
        case 0xFFFFFFD9u: op_stresptr (v); return 1;
        case 0xFFFFFFD8u: op_ldresptr (v); return 1;
        }
        return 0;
}
