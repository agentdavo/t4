/* A single node with nothing to run but a timer sleeps until the timer is
 * due, and the timer still fires on time.  One node, no links.
 *
 * test_timer_sleep checks this for threaded nodes.  A node on its own went
 * round its idle loop for as long as a timer was pending (start_process
 * slept only with the timer queues empty), so a 2 s wait cost 2 s of a
 * host core: every program that waits on a timer, sleeps in a busy loop.
 *
 * 1. This (low priority) process waits 1 s on its 64us clock.
 * 2. A high priority process waits 0.5 s on its 1us clock.
 * 3. A timer ALT on a channel nobody uses times out after 0.3 s.
 * Each wait must end once its time has passed and not much later.  The
 * run spends about 1.8 s waiting; run_t9000_vcp.sh checks that t4 used
 * well under that much host CPU.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

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

static Channel hichan = NOPROCESS;
static Channel idle = NOPROCESS;        /* nobody ever uses it */
static int ws[PROC_WS_MIN / 4];

/* high priority: 0.5 s on the 1us clock */
static void hitimer(void)
{
    int t0 = Time();

    ProcWait(500000);
    ChanOutInt(&hichan, Time() - t0);
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
    Channel *g[1];
    int t0, lo, hi, sel;

    /* 1. low priority: 1 s on the 64us clock */
    t0 = Time();
    ProcWait(15625);
    lo = Time() - t0;
    check("lo-time", lo >= 15625 && lo < 15625 + 320, lo);  /* < 20 ms late */

    /* 2. high priority: 0.5 s on the 1us clock */
    PRun(PSetup(ws, sizeof ws, hitimer));
    hi = ChanInInt(&hichan);
    check("hi-time", hi >= 500000 && hi < 520000, hi);      /* < 20 ms late */

    /* 3. a timer ALT that times out: 0.3 s is 4688 low priority ticks */
    g[0] = &idle;
    t0 = Time();
    sel = ChanTimerAlt(t0 + 4688, g, 1);
    lo = Time() - t0;
    check("alt-timeout", sel == -1, sel);
    check("alt-time", lo >= 4688 && lo < 4688 + 320, lo);

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
