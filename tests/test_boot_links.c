/* Nodes of one threaded network booted through different links.  Three
 * threaded nodes: node 0's Link1 is cabled to node 1's Link1, its Link2 to
 * node 2's Link2.
 *
 * A node waiting to be booted takes its boot from whichever link speaks
 * first, then listens only to that one until it is booted (handleboot,
 * BootLink).  t4 kept that link in one variable for the whole process, so
 * once node 1 was being booted on Link1, node 2 listened only on Link1
 * too, and its boot code waited in its Link2 mailbox for ever.  Every
 * shipped network boots all of its nodes through the same link number,
 * so nothing showed it; until 2026-11-04 node 2 gave up after 60 s, now
 * it would wait as long as the network runs.
 *
 * Node 0 (this program) boots node 1 down Link1 and takes its reply, waits
 * 1.5 s - longer than one of node 2's 1 s boot waits, so that node 2 is
 * certainly listening with node 1's boot link known - then boots node 2
 * down Link2 and takes its reply.  Each booted node outputs a word on its
 * boot link and stops.
 *
 * run_t9000_vcp.sh starts it with -st -sn -3.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned char u8;

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      ldc #11; stl 0
 *      ldlp 0; mint; adc 4; ldc 4; out         on Link1Out
 *      stopp
 * padded to 16 bytes, a whole number of words.
 */
static const u8 boot1[] = {
    16,
    0x24, 0xB0,
    0x21, 0x41, 0xD0,
    0x10, 0x24, 0xF2, 0x84, 0x44, 0xFB,
    0x21, 0xF5,
    0x20, 0x20, 0x20
};

/* Node 2: the same with #22 on Link2Out (mint; adc 8). */
static const u8 boot2[] = {
    16,
    0x24, 0xB0,
    0x22, 0x42, 0xD0,
    0x10, 0x24, 0xF2, 0x88, 0x44, 0xFB,
    0x21, 0xF5,
    0x20, 0x20, 0x20
};

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
    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);
    check("node1", ChanInInt(LINK1IN) == 0x11);
    ProcWait(23437);                    /* 1.5 s on the 64us clock */
    ChanOut(LINK2OUT, (void *)boot2, sizeof boot2);
    check("node2", ChanInInt(LINK2IN) == 0x22);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
