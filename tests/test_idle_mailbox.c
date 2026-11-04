/* An idle threaded node sleeps while a packet nobody is waiting for sits
 * in one of its link mailboxes.  Two threaded nodes, Link1 and Link2
 * cabled together, as test_link_sync.
 *
 * A node with nothing to run waits in linkcomms() for the links its
 * processes are waiting on (t4_node_idle_wait).  Before it sleeps it asks
 * whether one is already ready, and that question counted every full In
 * mailbox, whether or not a process was waiting on the link.  A packet
 * that had arrived on a link nobody was inputting from made it true at
 * once, every time round the idle loop, so the node spun on the host for
 * as long as the packet stayed.  In the reworked ray tracer's farm
 * (examples/transputer-raytracer/bin) that is every node a job or a pixel
 * row has reached while its own throughPut or pixelBypass is busy: in a
 * 24-node run a dozen idle node threads took a core each, and once the
 * farm deadlocked they kept doing so.
 *
 * Node 0 (this program) boots node 1 down Link1.  Node 1 outputs a word
 * on Link2, which nobody on node 0 inputs, and so waits for ever.  Node 0
 * sees it arrive with an ALT (which leaves it in the mailbox), starts a
 * process that inputs on Link1, which node 1 never sends on, and stops.
 * Node 0 is then idle with one link waited on and the other's mailbox
 * full, until the run's timeout.  The UART line says the set-up worked;
 * run_t9000_vcp.sh checks that t4 used well under a core over the run.
 *
 * run_t9000_vcp.sh starts it with -st -sn -2.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned char u8;

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      ldc #55; stl 0
 *      ldlp 0; mint; adc 8; ldc 4; out         on Link2Out: waits for ever
 *      stopp
 * padded to 16 bytes, a whole number of words.
 */
static const u8 boot1[] = {
    16,
    0x24, 0xB0,
    0x25, 0x45, 0xD0,
    0x10, 0x24, 0xF2, 0x88, 0x44, 0xFB,
    0x21, 0xF5,
    0x20, 0x20, 0x20
};

static int ws[PROC_WS_MIN / 4];

/* never satisfied: node 1 sends nothing on Link1 after its boot */
static void waiter(void)
{
    (void)ChanInInt(LINK1IN);
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
    Channel *link2[1];

    link2[0] = LINK2IN;
    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);
    /* selects Link2 once node 1's word is in the mailbox, and leaves it
     * there */
    check("arrived", ChanTimerAlt(Time() + 1000000, link2, 1) == 0);
    PRun(PSetup(ws, sizeof ws, waiter));
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
