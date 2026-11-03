/* A timer ALT leaves the timer list whichever guard wins, and is scheduled
 * once.  One node.  Expected: S\nALL-PASS\nH
 *
 * A: dist removes the process from the timer list "if the process is still
 *    on the timer list" (ISM, dist) - as it is when a channel won.  t4's
 *    dist did not, so the ALT stayed on the list; the next timer ALT from
 *    the same workspace inserted it again, linked to itself, and t4 spun
 *    in schedule_timerq.
 * B: a timer ALT whose time comes while a high-priority process has the
 *    processor is ready, not waiting (ISM 8.7: only a Waiting.p ALT is
 *    descheduled), so a channel output to it before it runs must not
 *    schedule it again.  t4 left it Waiting.p and queued it twice.  With
 *    another process queued between the two entries (here one whose timer
 *    comes due a tick later), the second entry ran it again from wherever
 *    it had next descheduled: a timer wait ended early.
 *
 * test_main and the other process run at low priority, the peer at high
 * priority.  The ALTs are conc.h's ChanTimerAlt (lib/transputer-conc.c).
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static int pass = 1;

static void check(const char *name, int ok)
{
    if (ok)
        return;
    uart_puts(name);
    uart_puts(":F ");
    pass = 0;
}

#define ROUNDS 8

static Channel go = NOPROCESS;          /* main -> peer: which case */
static Channel c = NOPROCESS;           /* peer -> main, ALTed on */
static Channel idle = NOPROCESS;        /* nobody ever uses it */

enum { WAIT_THEN_SEND = 1, BUSY_THEN_SEND = 2 };

/* High priority: 1us ticks. */
static void peer(void)
{
    int op, t;

    for (;;) {
        op = ChanInInt(&go);
        /* main is in its ALT by the time this wait ends */
        ProcWait(300);
        if (op == BUSY_THEN_SEND) {
            /* hold the processor past main's deadline, 20 low-priority
             * ticks (1.28 ms) after go */
            t = Time() + 3000;
            while (TimeAfter(t, Time()))
                ;
        }
        ChanOutInt(&c, op);
    }
}

static int peer_ws[PROC_WS_MIN / 4];

/* Low priority: waits until the time it is sent. */
static Channel xgo = NOPROCESS;

static void sleeper(void)
{
    for (;;)
        ProcAfter(ChanInInt(&xgo));
}

static int sleeper_ws[PROC_WS_MIN / 4];

void test_main(void)
{
    Channel *g;
    int i, sel, v, start, d;
    int okA = 1, okB = 1, okB2 = 1;

    PRun(PSetup(peer_ws, sizeof peer_ws, peer));
    PRun(PSetup(sleeper_ws, sizeof sleeper_ws, sleeper) | PRI_LOW);

    /* A: the channel wins, then a timer ALT from the same frame times out */
    for (i = 0; i < ROUNDS; i++) {
        ChanOutInt(&go, WAIT_THEN_SEND);
        g = &c;
        sel = ChanTimerAlt(Time() + 200, &g, 1);
        v = ChanInInt(&c);
        if (sel != 0 || v != WAIT_THEN_SEND)
            okA = 0;
        g = &idle;
        start = Time();
        sel = ChanTimerAlt(start + 20, &g, 1);
        if (sel != -1 || TimeAfter(start + 20, Time()))
            okA = 0;
    }
    check("A-channel-then-timeout", okA);

    /* B: the time comes while the peer has the processor, the sleeper's
     * a tick later, then the peer's output arrives; afterwards a 100-tick
     * wait must last 100 ticks */
    for (i = 0; i < ROUNDS; i++) {
        d = Time() + 20;
        ChanOutInt(&xgo, d + 1);
        ChanOutInt(&go, BUSY_THEN_SEND);
        g = &c;
        sel = ChanTimerAlt(d, &g, 1);
        v = ChanInInt(&c);              /* either guard: take the message */
        if (v != BUSY_THEN_SEND)
            okB = 0;
        start = Time();
        ProcWait(100);
        if (Time() - start < 100)
            okB2 = 0;
    }
    check("B-message", okB);
    check("B-scheduled-once", okB2);

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
