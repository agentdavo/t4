/* A byte-stream link output completes only once the far process has input
 * the message (ISM 8.4.2, 12.2.2; an OS-link acknowledges a byte only to
 * a ready input, HRM C100 section 1.3.1).  Two threaded nodes, Link1 and
 * Link2 cabled together.
 *
 * Node 0 (this program) boots node 1 down Link1 with the code below, then
 * starts a high-priority sender that outputs an item on Link1 and records
 * when that output completes.  Node 1 first inputs a "go" word on Link2,
 * waits 200 ms, and only then inputs the item and echoes it back.  So when
 * node 0's "go" output has completed, node 1 holds "go" and is waiting:
 * the item's output must still be outstanding.  t4 used to complete a link
 * output as soon as the message was in the far node's buffer, so the
 * sender had finished before "go" was even sent.
 *
 * run_t9000_vcp.sh starts it with -st -sn -2.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned char u8;

#define ITEM    0x5A17C0DE
#define GO      0x60

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      ldc 0; sttimer                          a booted node's timers are
 *                                              stopped until sttimer
 *      ldlp 0; mint; adc #18; ldc 4; in        go, on Link2In
 *      ldtimer; ldc #C35; sum; tin             wait 3125 ticks: 200 ms at
 *                                              low priority, as booted
 *      ldlp 0; mint; adc #14; ldc 4; in        the item, on Link1In
 *      ldlp 0; mint; adc 4; ldc 4; out         back on Link1Out
 *      stopp
 * 36 bytes, a whole number of words.
 */
static const u8 boot1[] = {
    36,
    0x24, 0xB0, 0x40, 0x25, 0xF4,
    0x10, 0x24, 0xF2, 0x21, 0x88, 0x44, 0xF7,
    0x22, 0xF2, 0x2C, 0x23, 0x45, 0x25, 0xF2, 0x22, 0xFB,
    0x10, 0x24, 0xF2, 0x21, 0x84, 0x44, 0xF7,
    0x10, 0x24, 0xF2, 0x84, 0x44, 0xFB,
    0x21, 0xF5
};

static volatile int sent;
static int ws[PROC_WS_MIN / 4];

static void sender(void)
{
    ChanOutInt(LINK1OUT, ITEM);
    sent = 1;
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
    int early, echo;

    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);
    PRun(PSetup(ws, sizeof ws, sender));    /* high priority: runs at once */
    ProcWait(100);                          /* and in any case before this */
    ChanOutInt(LINK2OUT, GO);
    early = sent;
    echo = ChanInInt(LINK1IN);
    ProcWait(100);
    check("out-done-early", !early);
    check("echo", echo == ITEM);
    check("out-never-done", sent);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
