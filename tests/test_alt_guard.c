/* ALT guards are true unless they are false (0): enbc, disc, enbs, diss,
 * enbt and dist act on any other value (ISM, those instructions' pages).
 * One node.  Expected: S\nALL-PASS\nH
 *
 * occam's TRUE is 1, and conc.h passes 0 or 1, so this uses the builtins
 * with a guard of 2.  t4 used to act only on 1: the SKIP guard was enabled
 * but never selected, and the channel guard was never enabled, so that ALT
 * waited for ever.
 *
 * Each ALT runs start to finish in one function with a spare first
 * parameter, as alt_core() in lib/transputer-conc.c does.
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

static int alt_skip(int slot0, int g)
{
    int sel = -1;

    (void)slot0;
    __alt();
    __enbs(g);
    __altwt();
    if (__diss(0, g))
        sel = 0;
    return sel;
}

static int alt_chan(int slot0, Channel *c, int g)
{
    int sel = -1;

    (void)slot0;
    __alt();
    __enbc(g, c);
    __altwt();
    if (__disc(0, g, c))
        sel = 0;
    return sel;
}

/* A false guard ahead of a true one on the same, ready channel: the false
 * one must neither be selected nor touch the channel word. */
static int alt_false_first(int slot0, Channel *c)
{
    int sel = -1;

    (void)slot0;
    __alt();
    __enbc(0, c);
    __enbc(2, c);
    __altwt();
    if (__disc(0, 0, c))
        sel = 0;
    if (__disc(1, 2, c))
        sel = 1;
    return sel;
}

static Channel chan;
static int ws[PROC_WS_MIN / 4];

static void writer(void)
{
    ChanOutInt(&chan, 42);
    ChanOutInt(&chan, 43);
}

void test_main(void)
{
    int got;

    check("skip-guard-2", alt_skip(0, 2) == 0);

    ChanInit(&chan);
    PRun(PSetup(ws, sizeof ws, writer));    /* high priority: now waiting */
    check("chan-guard-2", alt_chan(0, &chan, 2) == 0);
    got = ChanInInt(&chan);
    check("chan-data", got == 42);
    check("false-guard-first", alt_false_first(0, &chan) == 1);
    got = ChanInInt(&chan);
    check("chan-data-2", got == 43);

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
