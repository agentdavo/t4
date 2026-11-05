/* Two nodes as two processes with shared-memory links (-sl): a node that
 * waits on a link is woken when the far process fills or empties its
 * mailbox, instead of looking again a millisecond later.
 *
 * An idle -sl node used to look at its mailboxes in 1 ms usleep slices,
 * so every hand-over between two processes cost up to a millisecond on
 * each side.  It now sleeps on a wake count in the shared segment that
 * the far process bumps and futex-wakes (t4_fabric.c).
 *
 * Node 0 (this program) boots node 1 down Link1 with an echo loop, then
 * sends it 1000 words and takes each back.  1000 round trips must take
 * less than 0.4 s on the low priority clock: polling took 1.1 s, a wait
 * woken by the far process takes 25-60 ms.
 *
 * run_t9000_vcp.sh starts node 1 (which creates the segment), then node 0.
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

typedef unsigned char u8;

#define ROUNDS  1000

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 * L:   ldlp 0; mint; adc #14; ldc 4; in        a word on Link1In
 *      ldlp 0; mint; adc 4; ldc 4; out         back on Link1Out
 *      j L                                     (nfix 0; j 1: -15)
 * padded to 20 bytes, a whole number of words.
 */
static const u8 boot1[] = {
    20,
    0x24, 0xB0,
    0x10, 0x24, 0xF2, 0x21, 0x84, 0x44, 0xF7,
    0x10, 0x24, 0xF2, 0x84, 0x44, 0xFB,
    0x60, 0x01,
    0x20, 0x20, 0x20
};

static int pass = 1;

static void check(const char *name, int ok, int v)
{
    if (ok)
        return;
    uart_puts(name);
    uart_puts(":F=");
    uart_putdec(v);
    uart_putc(' ');
    pass = 0;
}

void test_main(void)
{
    int i, v, bad = 0, t0, dt;

    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);
    t0 = Time();
    for (i = 0; i < ROUNDS; i++) {
        ChanOutInt(LINK1OUT, i);
        v = ChanInInt(LINK1IN);
        if (v != i)
            bad++;
    }
    dt = Time() - t0;                   /* 64us ticks */
    check("echo", bad == 0, bad);
    check("round-trips", dt < 6250, dt * 64 / 1000);    /* < 0.4 s, in ms */
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
