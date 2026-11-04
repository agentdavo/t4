/* A threaded node with nothing to run but a timer sleeps until the timer
 * is due, or until a link wakes it, and the timer still fires on time.
 * Two threaded nodes, Link1 and Link2 cabled together, as test_link_sync.
 *
 * A node with nothing to run sleeps in linkcomms() only when its timer
 * queues are empty.  With a process waiting on a timer it polled the links
 * without waiting and went round its idle loop again, so a 2 s timer wait
 * cost 2 s of a host core (test_idle_mailbox has the other way an idle
 * node kept a core busy).
 *
 * 1. Node 0 boots node 1 down Link1.  Node 1 starts its timers, waits
 *    200 ms on its low priority clock and then outputs a word on Link1.
 *    Node 0 waits for it in an ALT with a 2 s timer guard: the link must
 *    win, about 200 ms in, which needs the word's arrival to wake a node
 *    that is asleep on its timer.
 * 2. This (low priority) process waits 1 s on its 64us clock.
 * 3. A high priority process waits 0.5 s on its 1us clock.
 * Each wait must end once its time has passed and not much later.  The
 * run spends about 1.7 s waiting on timers; run_t9000_vcp.sh checks that
 * t4 used well under that much host CPU.
 *
 * Part 1 also checks how a node starts the code it boots from a link:
 * at low priority (TCWG 9: Wdesc = ffw BITOR 1).  t4 started it at high
 * priority, where node 1's 3125 ticks last 3 ms, and the word came about
 * 50 ticks in.
 *
 * Results 2026-11-04: 1.8 s of host CPU with the old idle loop, 0.3 s
 * with the sleep.
 *
 * run_t9000_vcp.sh starts it with -st -sn -2.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned char u8;

#define WORD    0x55

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      ldc 0; sttimer                          a booted node's timers are
 *                                              stopped until sttimer
 *      ldtimer; ldc #C35; sum; tin             wait 3125 ticks: 200 ms at
 *                                              low priority, as booted
 *      ldc #55; stl 0
 *      ldlp 0; mint; adc 4; ldc 4; out         on Link1Out
 *      stopp
 * padded to 28 bytes, a whole number of words.
 */
static const u8 boot1[] = {
    28,
    0x24, 0xB0,
    0x40, 0x25, 0xF4,
    0x22, 0xF2, 0x2C, 0x23, 0x45, 0x25, 0xF2, 0x22, 0xFB,
    0x25, 0x45, 0xD0,
    0x10, 0x24, 0xF2, 0x84, 0x44, 0xFB,
    0x21, 0xF5,
    0x20, 0x20, 0x20
};

static Channel hichan;
static int ws[PROC_WS_MIN / 4];

/* high priority: 0.5 s on the 1us clock */
static void hitimer(void)
{
    int t0 = Time();

    ProcWait(500000);
    ChanOutInt(&hichan, Time() - t0);
}

static void uart_putdec(int v)
{
    char buf[12];
    int n = 0;

    if (v < 0) {
        uart_putc('-');
        v = -v;
    }
    do {
        buf[n++] = '0' + v % 10;
        v /= 10;
    } while (v);
    while (n)
        uart_putc(buf[--n]);
}

static int pass = 1;

/* a failed check names itself and the time it measured */
static void check(const char *name, int ok, int t)
{
    if (ok)
        return;
    uart_puts(name);
    uart_puts(":F=");
    uart_putdec(t);
    uart_putc(' ');
    pass = 0;
}

void test_main(void)
{
    Channel *link1[1];
    int t0, got, lo, hi;

    /* 1. a word that comes while the node sleeps on its timer guard */
    link1[0] = LINK1IN;
    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);
    t0 = Time();
    got = ChanTimerAlt(t0 + 31250, link1, 1);          /* 2 s */
    lo = Time() - t0;
    check("alt-link", got == 0, got);
    check("alt-time", lo >= 3000 && lo < 3125 + 800, lo);  /* ~200 ms, < 250 ms */
    check("word", got != 0 || ChanInInt(LINK1IN) == WORD, 0);

    /* 2. low priority: 1 s on the 64us clock */
    t0 = Time();
    ProcWait(15625);
    lo = Time() - t0;
    check("lo-time", lo >= 15625 && lo < 15625 + 320, lo);  /* < 20 ms late */

    /* 3. high priority: 0.5 s on the 1us clock */
    ChanInit(&hichan);
    PRun(PSetup(ws, sizeof ws, hitimer));
    hi = ChanInInt(&hichan);
    check("hi-time", hi >= 500000 && hi < 520000, hi);      /* < 20 ms late */

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
