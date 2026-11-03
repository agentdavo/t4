/*
 * IMS T9000 control system: CLink0/CLink1, packet handler, control unit and
 * system services.  See t9000_ctl.h for the sources, the link format and the
 * command codes.
 *
 * Execution model.  Everything runs on the node's own thread.  linkcomms()
 * calls t9k_ctl_poll(), which moves packets through the CLink mailboxes and
 * executes commands; while the CPU is stopped (before a control-link boot,
 * after an error, Stop or Reset) t9k_ctl_after_halt() keeps serving the
 * control links until Run or Reboot starts it again.  The control unit stops
 * a running CPU through t4_cpu_halt(), which start_process() honours, and
 * finishes anything that needs the CPU stopped (resets, a reboot) once
 * mainloop() has returned.
 *
 * Link faults (HRM 13.5, 8.5.3): WrongParity and ResetOutput on a link give
 * its far end a parity or disconnect error; t9k_line_signal() and
 * t9k_line_poll() below carry and act on them.
 *
 * What is not modelled: link speeds, spontaneous link faults, the disconnect
 * a link reset causes at the far end (HRM 13.7.2), the PMI (#03) and the
 * cache.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include "processor.h"
#include "cpu_state.h"
#include "t4_fabric.h"
#include "t9000_vcp.h"
#include "t9000_ctl.h"
#include "t9000_protect.h"

extern u_char *SharedEvents;
extern int  channel_sendP (Channel *chan, u_char *data, int ndata, int doWait);
extern int  channel_recvP (Channel *chan, u_char *data, int doWait);
extern void open_link_channel (Channel *chan, uint32_t addr, int theLink, int chanIn);
extern void t4_cpu_clear (void);
extern void t4_cpu_start (uint32_t wdesc, uint32_t iptr);
extern void t4_cpu_halt (int now);
extern uint32_t t4_status_reg (void);
extern int t4_link_far_end (int link, int *node, int *flink);
extern int32_t quit;
extern volatile sig_atomic_t t4_sig_pending;
extern void t4_signal_exit (int);

/* Gross state (HRM table 8.7).  'Initial' is !started. */
#define G_READY         0
#define G_BOOTING       1
#define G_RUNNING       2
#define G_STOPPING      3
#define G_ERROR         4

/* CLinkNStatus bits (HRM table 13.7) */
#define ST_ERROR        0x01u
#define ST_STARTED      0x02u
#define ST_RESETOUT     0x04u
#define ST_PARITY       0x08u
#define ST_DISC         0x10u
#define ST_TOKEN        0x20u

/* Our message packet waiting for its acknowledge */
#define IN_HS           1
#define IN_ERR          2

#define NERR            10
#define MAX_ACKS        8
#define ROM_IPTR        0x7FFFFFF8u     /* HRM 8.4 Reboot, 10.5.2 */
#define ROM_WPTR        0x7FFFFFFCu

/* HRM table 8.9, highest priority first: bit i of err_rec is err_code[i]. */
static const uint8_t err_code[NERR] = {
        0xC0, 0xC1, 0xC2, 0x80, 0x81, 0x82, 0x83, 0x01, 0x02, 0x03
};

typedef struct t9k_ctl {
        /* system services (HRM 8.3.4) */
        uint32_t mode_status, error_code, pll, ss_wlock;
        /* CLink0-1 Mode, Status, WriteLock (HRM 15.4.8, bits as 13.6) */
        uint32_t cl_mode[2], cl_status[2], cl_wlock[2];
        int      wired[2];

        /* packet handler (HRM 8.3.2) */
        int      labelled, started;
        u_char   label[2], rethdr[2];
        u_char   down[SCH_SIZE];        /* CLink0 -> CLink1, one packet */
        int      ndown;
        u_char   up[SCH_SIZE];          /* CLink1 -> CLink0, one packet */
        int      nup, up_turn;
        u_char   ackhdr[MAX_ACKS][2];   /* acknowledges owed, each under the */
        int      nacks;                 /* return header current at the time */
        int      inflight;              /* IN_HS/IN_ERR: acknowledge state */

        /* command handler (HRM 8.3.3) */
        int      gross;
        uint32_t boot_ptr, boot_left;
        u_char   hs[8];                 /* handshake to send */
        int      hslen, hs_unacked;     /* handshake state */
        uint32_t err_rec;               /* recorded errors */
        int      err_sent;              /* index awaiting ErrorHandshake, or -1 */

        /* CPU */
        int      cpu_running, cpu_started;
        int      reset_pending, reset_hs;
        int      start_pending;
        uint32_t start_wptr, start_iptr;
        uint32_t saved[7];              /* HRM 8.5.4 state for Run/Reboot */
        int      busy, debug;
} t9k_ctl_t;

#define CDBG(c, ...) \
        do { if ((c)->debug) { fprintf (stderr, "[CTL%d] ", nodeid); \
                               fprintf (stderr, __VA_ARGS__); } } while (0)

static void ctl_tx (t9k_ctl_t *c);

static int ch_open (Channel *ch)
{
        return ch->schbuf || ch->sock >= 0;
}

static uint32_t le32 (const u_char *p)
{
        return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static void put32 (u_char *p, uint32_t v)
{
        p[0] = (u_char)v; p[1] = (u_char)(v >> 8);
        p[2] = (u_char)(v >> 16); p[3] = (u_char)(v >> 24);
}

/* ---- state ------------------------------------------------------------ */

static void save_cpu_state (t9k_ctl_t *c)
{
        c->saved[0] = WdescReg;
        c->saved[1] = IPtr;
        c->saved[2] = t4_status_reg ();
        c->saved[3] = ThReg;
        if (!t9k_cfg_read (T9K_CR_REASON, &c->saved[4]))
                c->saved[4] = 0;
        c->saved[5] = 0;        /* error type and Eptr: T4 records them only */
        c->saved[6] = 0;        /* for traps, which do not stop the CPU */
}

void t9k_ctl_reset_node (void)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        const char *e;
        int i;

        free (c);
        c = calloc (1, sizeof *c);
        if (!c) {
                printf ("-E-EMU9000: out of memory for the control system\n");
                exit (-1);
        }
        t4cpu->s_ctl = c;
        c->err_sent = -1;
        c->gross = G_READY;
        save_cpu_state (c);
        c->saved[0] = NotProcess_p;     /* HRM 8.7.1 */
        e = getenv ("T4_CTL_DEBUG");
        c->debug = e && *e && *e != '0';
        for (i = 0; i < 2; i++) {
                open_link_channel (&t4cpu->s_CLink[i].In, 0, 4 + i, 1);
                open_link_channel (&t4cpu->s_CLink[i].Out, 0, 4 + i, 0);
                c->wired[i] = ch_open (&t4cpu->s_CLink[i].Out);
        }
        t4cpu->s_ctl_links = c->wired[0] || c->wired[1];
        t4cpu->s_ctl_halt = 0;
        if (t4cpu->s_ctl_links)
                CDBG (c, "control links: CLink0 %s, CLink1 %s\n",
                      c->wired[0] ? "wired" : "open", c->wired[1] ? "wired" : "open");
}

void t9k_ctl_destroy (struct cpu_t *cpu)
{
        if (cpu && cpu->s_ctl) {
                free (cpu->s_ctl);
                cpu->s_ctl = NULL;
        }
}

static int err_index (uint32_t code)
{
        int i;

        for (i = 0; i < NERR; i++)
                if (err_code[i] == code)
                        return i;
        return -1;
}

/* HRM 8.5.1: one record per error value; ErrorCode holds the latest. */
static void ctl_record (t9k_ctl_t *c, uint32_t code)
{
        int i = err_index (code);

        c->error_code = code;
        if (i >= 0)
                c->err_rec |= 1u << i;
        CDBG (c, "error #%02X recorded\n", code);
}

/* ---- control unit actions ---------------------------------------------- */

/* The control unit addresses memory physically, never through the regions
 * of a P-process the CPU may have been running when it stopped. */
static int phys_begin (void)
{
        int save = t9k_prot.phys_bypass;

        t9k_prot.phys_bypass = 1;
        return save;
}

static void phys_end (int save)
{
        t9k_prot.phys_bypass = save;
}

static void handshake (t9k_ctl_t *c, uint32_t cmd, int status, uint32_t data)
{
        c->hs[0] = (u_char)(cmd ^ T9K_CC_RESPONSE);
        c->hs[1] = (u_char)status;
        c->hslen = 2;
        if (cmd == T9K_CC_IDENTIFY || cmd == T9K_CC_CPEEK || cmd == T9K_CC_PEEK) {
                put32 (c->hs + 2, status ? 0u : data);
                c->hslen = 6;
        }
        CDBG (c, "handshake #%02X status %d\n", c->hs[0], status);
}

/* Reset levels 1-3 (HRM 8.7, table 8.10) with the CPU stopped. */
static void apply_reset (t9k_ctl_t *c)
{
        int level = c->reset_pending, i, ph;

        c->reset_pending = 0;
        t4_cpu_clear ();
        if (level <= 2) {
                c->err_rec = 0;                 /* error state cleared */
                c->err_sent = -1;
                c->ss_wlock = 0;                /* CPU write locks cleared */
                c->cl_wlock[0] = c->cl_wlock[1] = 0;
                t9k_vcp_signal (level == 1 ? T9K_VCPSIG_RESET1 : T9K_VCPSIG_RESET2);
                ph = phys_begin ();
                for (i = 0; i < 4; i++) {
                        reset_channel (0x80000000u + 4u * i);   /* byte-stream links */
                        reset_channel (0x80000010u + 4u * i);
                }
                phys_end (ph);
        }
        /* a stand-alone error's reset keeps ErrorSinceReset for the ROM
         * code (HRM 8.5.2); it is the only reset with a reboot pending */
        if (level == 1 && !c->start_pending)
                c->mode_status &= ~T9K_MS_ERRORSINCERESET;
        c->gross = G_READY;
        c->boot_left = 0;
        CDBG (c, "reset level %d done\n", level);
        if (c->reset_hs) {
                c->reset_hs = 0;
                handshake (c, T9K_CC_RESET, 0, 0);
        }
}

/* Run and Reboot: the new process gets the old CPU state (HRM 8.5.4) and
 * runs at high priority, as a boot from ROM does (HRM 10.5.2). */
static void cpu_start (t9k_ctl_t *c)
{
        int i;

        c->start_pending = 0;
        for (i = 0; i < 7; i++)
                t9k_phys_writeword (c->start_wptr + 4u * i, c->saved[i]);
        t4_cpu_start (c->start_wptr | HiPriority, c->start_iptr);
        c->cpu_started = 1;
        c->gross = G_RUNNING;
        CDBG (c, "CPU started: Wptr #%08X Iptr #%08X\n", c->start_wptr, c->start_iptr);
}

static void request_reset (t9k_ctl_t *c, int level, int with_hs)
{
        c->reset_pending = level;
        c->reset_hs = with_hs;
        if (c->cpu_running)
                t4_cpu_halt (1);        /* finished in t9k_ctl_after_halt() */
        else
                apply_reset (c);
}

int t9k_ctl_error (uint32_t code)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        static const char *const what[] = { "CPU", "VCP", "PMI" };

        if (!c)
                return 0;
        if (code >= T9K_CE_CLINK0_CMD) {
                /* control link errors do not stop the CPU (HRM table 8.8) */
                ctl_record (c, code);
                return 1;
        }
        c->error_code = code;
        c->mode_status |= T9K_MS_ERRORSINCERESET;
        if (c->mode_status & T9K_MS_STANDALONE) {
                /* HRM 8.5.2: reset every subsystem, then boot from ROM */
                CDBG (c, "stand-alone error #%02X: reset and reboot from ROM\n", code);
                c->start_wptr = t9k_phys_word (ROM_WPTR) & ~3u;
                c->start_iptr = t9k_phys_word (ROM_IPTR);
                c->start_pending = 1;
                request_reset (c, 1, 0);
                c->gross = G_RUNNING;
                return 1;
        }
        if (!c->wired[0])
                return 0;
        /* HRM 8.5 and 13.5.1: stop the CPU, VCP to 'discarding' (11.8.2),
         * Error message on CLink0 */
        ctl_record (c, code);
        c->gross = G_ERROR;
        t9k_vcp_signal (T9K_VCPSIG_ERROR);
        if (c->cpu_running)
                t4_cpu_halt (1);
        printf ("-I-EMU9000: Node%d %s error (#%02X): CPU halted, reported on CLink0\n",
                nodeid < 0 ? 0 : nodeid,
                code < 4 ? what[code - 1] : "link", code);
        ctl_tx (c);
        return 1;
}

/* ---- commands (HRM 8.4) ------------------------------------------------- */

/* Message length including the command byte, or -1 for an invalid code. */
static int cmd_length (uint32_t code)
{
        switch (code) {
        case T9K_CC_IDENTIFY: case T9K_CC_STOP: case T9K_CC_RECOVERERROR:
        case T9K_CC_REBOOT: case T9K_CC_ERRORHANDSHAKE:
                return 1;
        case T9K_CC_RESET:
                return 2;
        case T9K_CC_START: case T9K_CC_CPEEK:
                return 3;
        case T9K_CC_PEEK:
                return 5;
        case T9K_CC_CPOKE:
                return 7;
        case T9K_CC_POKE: case T9K_CC_BOOT: case T9K_CC_RUN:
                return 9;
        case T9K_CC_BOOTDATA:
                return 17;
        }
        return -1;
}

/* HRM table 8.7 */
static int cmd_valid (int g, uint32_t code)
{
        switch (code) {
        case T9K_CC_START: case T9K_CC_IDENTIFY: case T9K_CC_RECOVERERROR:
        case T9K_CC_RESET:
                return 1;
        case T9K_CC_CPEEK: case T9K_CC_CPOKE: case T9K_CC_PEEK: case T9K_CC_POKE:
        case T9K_CC_REBOOT:
                return g == G_READY || g == G_BOOTING;
        case T9K_CC_BOOT: case T9K_CC_RUN:
                return g == G_READY;
        case T9K_CC_BOOTDATA:
                return g == G_BOOTING;
        case T9K_CC_STOP:
                return g == G_RUNNING || g == G_STOPPING;
        }
        return 0;
}

static void owe_ack (t9k_ctl_t *c)
{
        if (c->nacks == MAX_ACKS)
                return;         /* cannot happen: each needs a command first */
        memcpy (c->ackhdr[c->nacks++], c->rethdr, 2);
}

static void command (t9k_ctl_t *c, const u_char *d, int len)
{
        uint32_t code = d[0], a, v;
        int st;

        if (code == T9K_CC_START && len == 3) {
                /* the acknowledge goes under the old return header, the
                 * handshake under the new one (HRM 8.4 Start) */
                if (c->started)
                        owe_ack (c);
                if (c->hslen || c->hs_unacked) {
                        if (!c->started)
                                owe_ack (c);
                        ctl_record (c, T9K_CE_CLINK0_PROTO);
                        return;
                }
                memcpy (c->rethdr, d + 1, 2);
                if (!c->started)
                        owe_ack (c);
                c->started = 1;
                CDBG (c, "Start: label %02X%02X, return header %02X%02X\n",
                      c->label[1], c->label[0], c->rethdr[1], c->rethdr[0]);
                handshake (c, code, 0, 0);
                return;
        }
        owe_ack (c);
        if (cmd_length (code) < 0) {
                ctl_record (c, T9K_CE_CLINK0_CMD);
                return;
        }
        if (len != cmd_length (code)) {
                ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        if (code == T9K_CC_ERRORHANDSHAKE) {
                if (c->err_sent < 0) {
                        ctl_record (c, T9K_CE_CLINK0_PROTO);
                        return;
                }
                CDBG (c, "ErrorHandshake for #%02X\n", err_code[c->err_sent]);
                c->err_rec &= ~(1u << c->err_sent);
                c->err_sent = -1;
                return;
        }
        /* only Reset and RecoverError may overtake a handshake (HRM 8.1.1) */
        if (code != T9K_CC_RESET && code != T9K_CC_RECOVERERROR &&
            (c->hslen || c->hs_unacked)) {
                ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        if (code == T9K_CC_RESET || code == T9K_CC_RECOVERERROR) {
                c->hslen = 0;           /* handshake state cleared */
                c->hs_unacked = 0;
        }
        if (!cmd_valid (c->gross, code)) {
                CDBG (c, "command #%02X invalid in state %d\n", code, c->gross);
                handshake (c, code, 1, 0);
                return;
        }
        st = 0;
        v = 0;
        switch (code) {
        case T9K_CC_IDENTIFY:
                if (!t9k_cfg_read (T9K_CR_DEVICEID, &v))
                        v = 0;
                v &= 0xFFFFu;
                break;
        case T9K_CC_RESET:
                if (d[1] < 1 || d[1] > 3)
                        break;
                request_reset (c, d[1], 1);
                return;                 /* handshake when the reset is done */
        case T9K_CC_STOP:
                /* HRM 8.5.4: the CPU runs on until its process deschedules
                 * or reaches a timeslice point */
                if (c->gross == G_RUNNING) {
                        c->gross = G_STOPPING;
                        t9k_vcp_signal (T9K_VCPSIG_STOP);
                        if (c->cpu_running)
                                t4_cpu_halt (0);
                }
                break;
        case T9K_CC_RECOVERERROR:
                /* HRM 8.5.3: forget the acknowledge state, so that an error
                 * message that may have been lost goes again */
                c->inflight = 0;
                c->err_sent = -1;
                break;
        case T9K_CC_CPEEK:
                st = !t9k_cfg_read (d[1] | (uint32_t)d[2] << 8, &v);
                break;
        case T9K_CC_CPOKE:
                st = !t9k_cfg_write (d[1] | (uint32_t)d[2] << 8, le32 (d + 3), 0);
                break;
        case T9K_CC_PEEK:
                a = le32 (d + 1);
                if ((st = (a & 3u) != 0) == 0)
                        v = t9k_phys_word (a);
                break;
        case T9K_CC_POKE:
                a = le32 (d + 1);
                if ((st = (a & 3u) != 0) == 0)
                        t9k_phys_writeword (a, le32 (d + 5));
                break;
        case T9K_CC_BOOT:
                c->boot_left = le32 (d + 1);
                c->boot_ptr = le32 (d + 5);
                if ((c->boot_left & 3u) || (c->boot_ptr & 3u)) {
                        c->boot_left = 0;
                        st = 1;
                } else if (c->boot_left)
                        c->gross = G_BOOTING;
                break;
        case T9K_CC_BOOTDATA:
                /* 16 bytes each, into consecutive blocks of four words;
                 * the excess of the last is discarded */
                v = c->boot_left < 16u ? c->boot_left : 16u;
                writebytes_int (c->boot_ptr, (u_char *)d + 1, v);
                c->boot_ptr += 16u;
                c->boot_left -= v;
                if (!c->boot_left)
                        c->gross = G_READY;
                v = 0;
                break;
        case T9K_CC_RUN:
                c->start_wptr = le32 (d + 1);
                c->start_iptr = le32 (d + 5);
                if ((c->start_wptr & 3u) || c->cpu_running) {
                        st = 1;
                        break;
                }
                c->start_pending = 1;
                c->gross = G_RUNNING;
                break;
        case T9K_CC_REBOOT:
                if (c->cpu_running) {
                        st = 1;
                        break;
                }
                c->start_wptr = t9k_phys_word (ROM_WPTR) & ~3u;
                c->start_iptr = t9k_phys_word (ROM_IPTR);
                c->start_pending = 1;
                c->gross = G_RUNNING;
                break;
        }
        if (code == T9K_CC_RESET)
                st = 1;                 /* invalid level */
        handshake (c, code, st, v);
}

/* ---- link faults (HRM 13.5, 8.5.3) ------------------------------------- */

/* A link error is a line event, not data: the far end of the cable is told
 * through its s_line_err bits, not its mailbox (which may be full).  The
 * cable's far end is a node of the threaded fabric or, for a stand-alone
 * node, its own looped-back link; a C104 or a node in another process does
 * not see faults. */
void t9k_line_signal (int link, int disconnect)
{
        int node, flink;
        cpu_t *far = NULL;

        if (t4_link_far_end (link, &node, &flink) != 0)
                return;
        if (node < 0 || node == nodeid)
                far = t4cpu;
        else if (t4_fabric && node < t4_fabric->nnodes)
                far = t4_fabric->cpus[node];
        if (!far)
                return;
        atomic_fetch_or (&far->s_line_err, 1u << (2 * flink + (disconnect ? 1 : 0)));
        if (far != t4cpu)
                t4_node_wake (far);
}

/* A parity or disconnect error on this node's link: the link records it
 * and resets.  A parity error halts the link's output, which the far end
 * sees as a disconnect, and halts that output in turn (HRM 13.5).  Unless
 * localized, an error on Link0-3 goes to the control unit as #80-#83 (a
 * system error), one on CLink1 as #C2; one on CLink0 cannot be reported,
 * and its halted output is what tells the neighbour (HRM 8.5.3). */
static void line_fault (int link, int disconnect)
{
        static const char *const what[] = { "parity", "disconnect" };
        t9k_ctl_t *c = t4cpu->s_ctl;
        u_char tmp[SCH_SIZE];
        int k = link - 4, r;

        if (link < 4) {
                r = t9k_ds_link_error (link, disconnect);
                if (r < 0)
                        return;         /* not running: nothing to break */
                printf ("-I-EMU9000: Node%d Link%d %s error%s\n", nodeid < 0 ? 0 : nodeid,
                        link, what[disconnect], r ? " (localized)" : "");
                if (!disconnect)
                        t9k_line_signal (link, 1);
                if (!r)
                        t9k_ctl_error (T9K_CE_LINK (link));
                return;
        }
        if (!c || !(c->cl_status[k] & ST_STARTED))
                return;
        c->cl_status[k] = (c->cl_status[k] & ~ST_STARTED) | ST_ERROR |
                          (disconnect ? ST_DISC : ST_PARITY);
        if (ch_open (&t4cpu->s_CLink[k].In))
                (void)channel_recvP (&t4cpu->s_CLink[k].In, tmp, 0);
        if (k)
                c->ndown = c->nup = 0;
        printf ("-I-EMU9000: Node%d CLink%d %s error\n", nodeid < 0 ? 0 : nodeid,
                k, what[disconnect]);
        if (!disconnect)
                t9k_line_signal (link, 1);
        if (k)
                t9k_ctl_error (T9K_CE_CLINK1);
}

void t9k_line_poll (void)
{
        uint32_t ev = atomic_exchange (&t4cpu->s_line_err, 0u);
        int bit;

        for (bit = 0; ev; bit++, ev >>= 1)
                if (ev & 1u)
                        line_fault (bit >> 1, bit & 1);
}

/* ---- packet handler (HRM 8.3.2) ----------------------------------------- */

static void rx_ack (t9k_ctl_t *c)
{
        if (!c->inflight) {
                ctl_record (c, T9K_CE_CLINK0_PROTO);    /* unexpected acknowledge */
                return;
        }
        if (c->inflight == IN_HS)
                c->hs_unacked = 0;
        c->inflight = 0;
}

static void rx_clink0 (t9k_ctl_t *c, u_char *p, int n)
{
        int tok, dl;

        /* CLink0 starts on its first token (HRM 8.3.1) */
        c->cl_status[0] |= ST_STARTED | ST_TOKEN;
        if (n < 3) {
                ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        if (!c->labelled) {
                /* the first header seen after hard reset is the label */
                memcpy (c->label, p, 2);
                c->labelled = 1;
        }
        if (memcmp (p, c->label, 2)) {
                /* not for this device: on down the chain, or destroyed */
                if (c->wired[1] && (c->cl_status[1] & ST_STARTED)) {
                        memcpy (c->down, p, (size_t)n);
                        c->ndown = n;
                } else
                        ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        tok = p[n - 1];
        dl = n - 3;
        if (dl == 0 && tok == T9K_TOK_EOP) {
                rx_ack (c);
                return;
        }
        if (!c->started &&
            !(tok == T9K_TOK_EOM && dl == 3 && p[2] == T9K_CC_START)) {
                /* HRM 8.3.1: reported once Start programs a return header */
                ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        if (tok != T9K_TOK_EOM || dl > (int)T9K_MAX_PACKET) {
                /* commands are one packet ending in EOM */
                owe_ack (c);
                ctl_record (c, T9K_CE_CLINK0_PROTO);
                return;
        }
        command (c, p + 2, dl);
}

static int own_ready (t9k_ctl_t *c)
{
        return c->started && !c->inflight &&
               (c->hslen || (c->err_rec && c->err_sent < 0));
}

static void ctl_tx (t9k_ctl_t *c)
{
        Channel *o0 = &t4cpu->s_CLink[0].Out, *o1 = &t4cpu->s_CLink[1].Out;
        u_char pkt[2 + 8 + 1];
        int guard, len, own;

        if (c->ndown && ch_open (o1) && (c->cl_status[1] & ST_STARTED) &&
            channel_sendP (o1, c->down, c->ndown, 0) >= 0)
                c->ndown = 0;
        if (!ch_open (o0) || !(c->cl_status[0] & ST_STARTED))
                return;
        for (guard = 0; guard < 2 * MAX_ACKS + 4; guard++) {
                if (c->nacks) {
                        memcpy (pkt, c->ackhdr[0], 2);
                        pkt[2] = T9K_TOK_EOP;
                        if (channel_sendP (o0, pkt, 3, 0) < 0)
                                return;
                        memmove (c->ackhdr[0], c->ackhdr[1], (size_t)(--c->nacks) * 2);
                        continue;
                }
                /* fair arbiter between through traffic and our own */
                own = own_ready (c);
                if (c->nup && (!own || c->up_turn)) {
                        if (channel_sendP (o0, c->up, c->nup, 0) < 0)
                                return;
                        c->nup = 0;
                        c->up_turn = 0;
                        continue;
                }
                if (!own)
                        return;
                memcpy (pkt, c->rethdr, 2);
                if (c->hslen) {
                        memcpy (pkt + 2, c->hs, (size_t)c->hslen);
                        len = 2 + c->hslen;
                } else {
                        int i;

                        for (i = 0; !(c->err_rec & (1u << i)); i++)
                                ;
                        pkt[2] = T9K_CC_ERROR;
                        pkt[3] = err_code[i];
                        len = 4;
                        c->err_sent = i;
                }
                pkt[len++] = T9K_TOK_EOM;
                if (channel_sendP (o0, pkt, len, 0) < 0) {
                        if (!c->hslen)
                                c->err_sent = -1;
                        return;
                }
                if (c->hslen) {
                        c->hslen = 0;
                        c->hs_unacked = 1;
                        c->inflight = IN_HS;
                } else {
                        c->inflight = IN_ERR;
                        CDBG (c, "Error message #%02X sent\n", pkt[3]);
                }
                c->up_turn = 1;
        }
}

int t9k_ctl_poll (void)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        Channel *i0, *i1;
        u_char pkt[SCH_SIZE];
        int r = 0, n, guard;

        if (!c || !t4cpu->s_ctl_links)
                return 0;
        i0 = &t4cpu->s_CLink[0].In;
        i1 = &t4cpu->s_CLink[1].In;
        /* CLink1 -> CLink0; an unstarted link takes no tokens (HRM 13.7.1) */
        for (guard = 0; guard < 16 && !c->nup && c->wired[1] &&
                        (c->cl_status[1] & ST_STARTED); guard++) {
                n = channel_recvP (i1, c->up, 0);
                if (n < 0)
                        break;
                c->nup = n;
                c->cl_status[1] |= ST_TOKEN;
                r = T9K_CTL_ACTIVE;
                ctl_tx (c);
        }
        for (guard = 0; guard < 16 && !c->ndown && ch_open (i0); guard++) {
                n = channel_recvP (i0, pkt, 0);
                if (n < 0)
                        break;
                r = T9K_CTL_ACTIVE;
                rx_clink0 (c, pkt, n);
                ctl_tx (c);
        }
        ctl_tx (c);
        if (c->start_pending && !c->cpu_running && !c->reset_pending) {
                cpu_start (c);
                r |= T9K_CTL_START;
        }
        if (r)
                c->busy = 1;
        return r;
}

/* Whether the control links can move anything: a packet to take in, or
 * one waiting to go out on a link whose far mailbox has emptied. */
int t9k_ctl_ready (void)
{
        t9k_ctl_t *c = t4cpu ? t4cpu->s_ctl : NULL;
        Channel *i0, *i1, *o0, *o1;

        if (t4cpu && atomic_load_explicit (&t4cpu->s_line_err, memory_order_relaxed))
                return 1;               /* a link fault to act on */
        if (!c || !t4cpu->s_ctl_links)
                return 0;
        i0 = &t4cpu->s_CLink[0].In;
        i1 = &t4cpu->s_CLink[1].In;
        o0 = &t4cpu->s_CLink[0].Out;
        o1 = &t4cpu->s_CLink[1].Out;
        return (!c->ndown && i0->schbuf && SCHLength (i0)) ||
               (!c->nup && i1->schbuf && SCHLength (i1) &&
                (c->cl_status[1] & ST_STARTED)) ||
               (c->ndown && o1->schbuf && !SCHLength (o1)) ||
               ((c->nacks || c->nup || own_ready (c)) &&
                (c->cl_status[0] & ST_STARTED) && o0->schbuf && !SCHLength (o0));
}

int t9k_ctl_busy (void)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        int b = c && c->busy;

        if (c)
                c->busy = 0;
        return b;
}

void t9k_ctl_wait (int ms)
{
        t4_idle_t *id = t4cpu->idle;
        struct timespec ts;
        unsigned gen;

        if (!id) {
                usleep ((useconds_t)ms * 1000);
                return;
        }
#ifdef CLOCK_MONOTONIC
        clock_gettime (CLOCK_MONOTONIC, &ts);
#else
        clock_gettime (CLOCK_REALTIME, &ts);
#endif
        ts.tv_sec += ms / 1000;
        ts.tv_nsec += (ms % 1000) * 1000000L;
        if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
        }
        pthread_mutex_lock (&id->mu);
        gen = id->idle_gen;
        while (id->idle_gen == gen && !t9k_ctl_ready () && !t4_net_stop_requested ())
                if (pthread_cond_timedwait (&id->cv, &id->mu, &ts))
                        break;
        pthread_mutex_unlock (&id->mu);
}

/* ---- CPU start and stop ------------------------------------------------- */

void t9k_ctl_cpu_entry (void)
{
        t9k_ctl_t *c = t4cpu->s_ctl;

        if (!c)
                return;
        c->cpu_running = 1;
        c->cpu_started = 0;
        /* booted down a data link, from a file or from ROM: Running without
         * a Start (HRM 8.4.1, "doubling" of states) */
        if (c->gross == G_READY || c->gross == G_BOOTING)
                c->gross = G_RUNNING;
}

int t9k_ctl_started_cpu (void)
{
        return t4cpu->s_ctl && t4cpu->s_ctl->cpu_started;
}

int t9k_ctl_after_halt (void)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        int by_ctl;

        if (!c)
                return 0;
        t9k_prot.jmp_valid = 0;         /* mainloop()'s frame is gone */
        c->cpu_running = 0;
        by_ctl = t4cpu->s_ctl_halt;
        t4cpu->s_ctl_halt = 0;
        if (quit || t4_net_stop_requested ())
                return 0;
        save_cpu_state (c);
        if (!by_ctl) {
                /* The CPU stopped by itself.  T4's halt-on-error stands for
                 * the T9000 halting on an error with a null trap-handler
                 * (ISM 10.3.3): a CPU error to the control system. */
                if (!(STATUSReg & 0x80000000u) || !t9k_ctl_error (T9K_CE_CPU))
                        return 0;
        }
        if (c->reset_pending)
                apply_reset (c);
        if (c->start_pending) {
                cpu_start (c);
                return 1;
        }
        if (!c->wired[0])
                return 0;
        CDBG (c, "CPU stopped (state %d); serving the control links\n", c->gross);
        ctl_tx (c);
        for (;;) {
                if (atomic_load_explicit (&t4cpu->s_line_err, memory_order_relaxed))
                        t9k_line_poll ();
                if (t9k_ctl_poll () & T9K_CTL_START)
                        return 1;
                /* the VCP and links run on with the CPU stopped (HRM
                 * 8.5.4); a discarding VCP drops what arrives (11.8.2) */
                t9k_vcp_serve_links ();
                if (quit || t4_net_stop_requested () ||
                    (SharedEvents && nodeid >= 0 && SharedEvents[nodeid]) ||
                    (t4_fabric && t4_fabric->events && nodeid >= 0 &&
                     t4_fabric->events[nodeid]))
                        return 0;
                if (t4_sig_pending) {
                        int sig = (int)t4_sig_pending;

                        t4_sig_pending = 0;
                        t4_signal_exit (sig);
                }
                t9k_ctl_wait (50);
        }
}

/* ---- configuration space (HRM 8.3.4, 15.4.4, 15.4.8) ------------------- */

/* #FDnn = CLink0, #FEnn = CLink1: 1 Mode, 2 Command, 3 Status, 4 WriteLock */
static int clink_reg (uint32_t a, int *k, int *reg)
{
        if ((a & 0xFF00u) != 0xFD00u && (a & 0xFF00u) != 0xFE00u)
                return 0;
        if ((a & 0xFFu) < 1u || (a & 0xFFu) > 4u)
                return 0;
        *k = (a & 0xFF00u) == 0xFE00u;
        *reg = (int)(a & 0xFFu);
        return 1;
}

int t9k_ctl_conf_read (uint32_t a, uint32_t *val)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        int k, reg;

        if (!c)
                return 0;
        switch (a) {
        case T9K_CR_MODESTATUS:         *val = c->mode_status; return 1;
        case T9K_CR_ERRORCODE:          *val = c->error_code; return 1;
        case T9K_CR_DSLINKPLL:          *val = c->pll; return 1;
        case T9K_CR_SYSSERVWRITELOCK:   *val = c->ss_wlock; return 1;
        }
        if (!clink_reg (a, &k, &reg))
                return 0;
        switch (reg) {
        case 1: *val = c->cl_mode[k]; return 1;
        case 3: *val = c->cl_status[k]; return 1;
        case 4: *val = c->cl_wlock[k]; return 1;
        }
        return -1;                      /* CLinkNCommand is write-only */
}

int t9k_ctl_conf_write (uint32_t a, uint32_t val, int by_cpu)
{
        t9k_ctl_t *c = t4cpu->s_ctl;
        int k, reg;

        if (!c)
                return 0;
        switch (a) {
        case T9K_CR_MODESTATUS:
        case T9K_CR_ERRORCODE:
        case T9K_CR_DSLINKPLL:
        case T9K_CR_SYSSERVWRITELOCK:
                /* HRM 15.3: a write lock stops every CPU write to its
                 * subsystem, the lock included, but not the control unit */
                if (by_cpu && c->ss_wlock)
                        return -1;
                switch (a) {
                case T9K_CR_MODESTATUS:
                        c->mode_status = val & 3u;      /* clears ErrorSinceReset */
                        return 1;
                case T9K_CR_DSLINKPLL:
                        c->pll = val & 0x3Fu;
                        return 1;
                case T9K_CR_SYSSERVWRITELOCK:
                        c->ss_wlock = val & 1u;
                        return 1;
                }
                return -1;                      /* ErrorCode is read-only */
        }
        if (!clink_reg (a, &k, &reg))
                return 0;
        if (by_cpu && c->cl_wlock[k])
                return -1;
        switch (reg) {
        case 1:
                c->cl_mode[k] = val & 0x1Fu;
                return 1;
        case 2: {
                int was = (c->cl_status[k] & ST_STARTED) != 0;

                if (val & 1u) {         /* ResetLink: buffers emptied */
                        u_char tmp[SCH_SIZE];

                        c->cl_status[k] = 0;
                        if (ch_open (&t4cpu->s_CLink[k].In))
                                (void)channel_recvP (&t4cpu->s_CLink[k].In, tmp, 0);
                        if (k)
                                c->ndown = c->nup = 0;
                }
                if (val & 2u)           /* StartLink */
                        c->cl_status[k] |= ST_STARTED;
                /* as on a data link (HRM 13.6): outputs held low are a
                 * disconnect at the far end, a bad parity bit a parity error */
                if (val & 4u) {         /* ResetOutput */
                        c->cl_status[k] = (c->cl_status[k] & ~ST_STARTED) | ST_RESETOUT;
                        if (was)
                                t9k_line_signal (4 + k, 1);
                }
                if ((val & 8u) && (c->cl_status[k] & ST_STARTED))      /* WrongParity */
                        t9k_line_signal (4 + k, 0);
                CDBG (c, "CLink%d command #%X: status #%02X\n", k, val, c->cl_status[k]);
                return 1;
        }
        case 4:
                c->cl_wlock[k] = val & 1u;
                return 1;
        }
        return -1;                      /* CLinkNStatus is read-only */
}
