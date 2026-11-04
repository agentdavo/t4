/* Low priority clock and timeslicing on a threaded node (TCWG 6.3.1: the
 * high priority clock ticks every 1us, the low priority one every 64us,
 * and a timeslice period ends every 1024 high priority ticks; after two,
 * the running low priority process is descheduled at its next j or lend).
 *
 * t4's timer thread wakes every ~100us.  For a threaded network it gave
 * each node elapsed/64 low priority ticks and elapsed/1024 timeslice
 * periods per wake, dropping the remainders: the low priority clock ran at
 * 64-85% of the high priority one, and a period ended only when the thread
 * overslept a millisecond, so a low priority process spinning through a j
 * kept the processor from the others.
 *
 * 1. Two low priority processes spin through a j each until they have seen
 *    the other run: only timeslicing lets the second one in.
 * 2. A high priority process times 100ms on its clock while this one
 *    reads the low priority clock around it: 64 low ticks per 64us.
 *
 * run_t9000_vcp.sh starts it with -st -sn -2, as test_link_sync.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

static volatile int spins, stop, stopped;
static Channel hichan;
static volatile int hi_elapsed;
static int ws[PROC_WS_MIN / 4], hws[PROC_WS_MIN / 4];

/* low priority: the back edge is a j, a timeslice point */
static void spinner(void)
{
top:
    spins++;
    if (!stop)
        goto top;
    stopped = 1;
}

/* high priority: 100ms on the 1us clock */
static void hitimer(void)
{
    int h0 = Time();

    ProcWait(100000);
    ChanOutInt(&hichan, Time() - h0);
}

static int pass = 1;

static void check(const char *name, int ok)
{
    if (ok)
        return;
    uart_puts(name);
    uart_puts(":F ");
    pass = 0;
}

void test_main(void)
{
    int l0, lo, hi, err;

    /* 1. this process holds the processor until it is timesliced */
    PRun(PSetup(ws, sizeof ws, spinner) | PRI_LOW);
wait_spin:
    if (spins == 0)
        goto wait_spin;
    stop = 1;
wait_stop:
    if (!stopped)
        goto wait_stop;
    check("timeslice", spins > 0 && stopped);

    /* 2. the low priority clock against the high priority one */
    ChanInit(&hichan);
    l0 = Time();
    PRun(PSetup(hws, sizeof hws, hitimer));     /* high priority: runs at once */
    hi = ChanInInt(&hichan);
    lo = Time() - l0;
    err = lo * 64 - hi;
    if (err < 0)
        err = -err;
    check("hi-100ms", hi >= 100000 && hi < 150000);
    check("lo-rate", err <= hi / 50 + 192);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
