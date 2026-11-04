/* The timeslice period follows the part: a period ends every 1024 ticks of
 * the high priority clock on the T800 (TCWG 6.3.1), every 256 on the T9000
 * (ISM 8.2.4, HRM 3.3).  When two have ended while the same low priority
 * process has been running, it is descheduled at its next j or lend.  A
 * process that gets the processor from a timeslice gets it just after a
 * period ended, so it keeps it for two periods: 2048us, or 512us on a
 * T9000.
 *
 * Two low priority processes spin through a j each and note the low
 * priority clock (64us ticks) whenever one finds the other was running
 * last.  NTURNS turns after the first must take NTURNS * 32 ticks, or
 * NTURNS * 8 on a T9000.  Over that many the average does not depend on
 * how coarsely t4's timer thread moves the clock (every ~100us).  The
 * first turn starts anywhere in a period and is not counted.
 *
 * lddevid tells the parts apart (ISM table A.14: a T9000 is 300-319);
 * run_t9000_vcp.sh says which part it asked t4 for with -DEXPECT_T9000.
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

#define NTURNS 32

static volatile int owner, nswitch, first, last;
static Channel done;
static int ws1[PROC_WS_MIN / 4], ws2[PROC_WS_MIN / 4];

/* low priority: the back edge is a j, a timeslice point */
static void spin(int me)
{
    int now;

top:
    now = Time();
    if (nswitch > NTURNS) {     /* before the other one counts a switch */
        ChanOutInt(&done, me);
        return;
    }
    if (owner != me) {
        if (owner != 0) {       /* the other one was timesliced */
            if (++nswitch == 1)
                first = now;
            last = now;
        }
        owner = me;
    }
    goto top;
}

static void spin1(void) { spin(1); }
static void spin2(void) { spin(2); }

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
    int t9000 = (unsigned)(__lddevid() - 300) < 20;
    int period = t9000 ? 256 : 1024;            /* us */
    int expect = NTURNS * 2 * period / 64;      /* low priority ticks */
    int got, err;

#ifdef EXPECT_T9000
    check("part", t9000 == EXPECT_T9000);
#endif
    ChanInit(&done);
    PRun(PSetup(ws1, sizeof ws1, spin1) | PRI_LOW);
    PRun(PSetup(ws2, sizeof ws2, spin2) | PRI_LOW);
    ChanInInt(&done);
    ChanInInt(&done);
    got = last - first;
    err = got - expect;
    if (err < 0)
        err = -err;
    check("period", err <= expect / 4);
    if (!pass) {
        uart_puts("turns=");
        uart_putdec(got);
        uart_puts(" expected=");
        uart_putdec(expect);
        uart_puts(" lddevid=");
        uart_putdec(__lddevid());
    }
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
