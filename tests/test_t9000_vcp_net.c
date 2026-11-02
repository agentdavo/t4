/* Two T9000s joined by a cable: virtual channels between nodes.
 *
 * Node 0 (this program) boots node 1 down byte-stream Link1 with the echo
 * server below, then both switch Link1 to virtual-channel mode and set up
 * virtual link 0 with header 0 at each end.  Node 0 sends messages of
 * several lengths and checks what comes back.
 *
 * Needs two threaded nodes: run_t9000_vcp.sh starts it with -st -sn -2 and
 * a two-node network map.  Probes are in test_t9000_vcp_fn.S.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned int u32;
typedef unsigned char u8;

extern void t9_stconf(u32 *), t9_ldconf(u32 *), t9_initvlcb(u32 *), t9_sethdr(u32 *);
extern void t9_writehdr(u32 *), t9_setchmode(u32 *), t9_vout(u32 *), t9_vin_cnt(u32 *);

#define VIN0    0x80000040u
#define VOUT0   0x80000044u

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      mint; adc #48; ldc #10F; stconf          MinInvalidChannel: one VL
 *      ldc 0; ldc #809; stconf                  Link1: virtual, 1-byte hdr
 *      ldlp 8; mint; adc #40; initvlcb          buffer = local 8
 *      ldc -1; ldc 1; mint; adc #40; sethdr     Link1, short header
 *      ldc 0; stl 0; ldlp 0; ldc 1; mint; adc #40; writehdr    header 0
 *      ldc 2; ldc #804; stconf                  start the VCP
 *      ldc 1; mint; adc #40; setchmode          activate input ...
 *      ldc 1; mint; adc #44; setchmode          ... and output
 * loop: ldlp 16; mint; adc #40; ldc 64; vin; ldcnt; stl 1
 *      ldlp 16; mint; adc #44; ldl 1; vout; j loop
 * padded to a whole number of words: the booted process's Wptr is the
 * first byte after the code.
 */
static const u8 boot1[] = {
    88,
    0x24, 0xB0, 0x24, 0xF2, 0x24, 0x88, 0x21, 0x20, 0x4F, 0x2B, 0xFF, 0x40,
    0x28, 0x20, 0x49, 0x2B, 0xFF, 0x18, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF6,
    0x60, 0x4F, 0x41, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF8, 0x40, 0xD0, 0x10,
    0x41, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF5, 0x42, 0x28, 0x20, 0x44, 0x2B,
    0xFF, 0x41, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF7, 0x41, 0x24, 0xF2, 0x24,
    0x84, 0x61, 0xF7, 0x21, 0x10, 0x24, 0xF2, 0x24, 0x80, 0x24, 0x40, 0x61,
    0xFC, 0x2C, 0xF0, 0xD1, 0x21, 0x10, 0x24, 0xF2, 0x24, 0x84, 0x71, 0x61,
    0xFD, 0x61, 0x08, 0x00
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

static u32 op(void (*f)(u32 *), u32 a, u32 b, u32 c)
{
    u32 io[3];
    io[0] = a; io[1] = b; io[2] = c;
    f(io);
    return io[0];
}

static u32 vin(u32 chan, u32 max, void *dst)
{
    u32 io[5];
    io[0] = max; io[1] = chan; io[2] = (u32)dst;
    t9_vin_cnt(io);
    return io[4] ? io[3] : 0xFFFFFFFFu;
}

static u32 pkt[8];
static u8 tx[64], rx[64];

void test_main(void)
{
    static const int lens[] = { 0, 1, 31, 32, 33, 64, 7 };
    u32 zero = 0, n;
    int i, j, ok;

    ChanOut(LINK1OUT, (void *)boot1, sizeof boot1);

    op(t9_stconf, 0x010F, VIN0 + 8, 0);         /* MinInvalidChannel */
    op(t9_stconf, 0x0809, 0, 0);                /* Link1: virtual */
    op(t9_initvlcb, VIN0, (u32)pkt, 0);
    op(t9_sethdr, VIN0, 1, 0xFFFFFFFFu);
    op(t9_writehdr, VIN0, 1, (u32)&zero);
    op(t9_stconf, 0x0804, 2, 0);                /* start */
    op(t9_setchmode, VIN0, 1, 0);
    op(t9_setchmode, VOUT0, 1, 0);

    for (i = 0; i < (int)(sizeof lens / sizeof lens[0]); i++) {
        for (j = 0; j < lens[i]; j++)
            tx[j] = (u8)(i * 31 + j * 5);
        op(t9_vout, lens[i], VOUT0, (u32)tx);
        for (j = 0; j < 64; j++)
            rx[j] = 0xEE;
        n = vin(VIN0, 64, rx);
        ok = n == (u32)lens[i];
        for (j = 0; ok && j < lens[i]; j++)
            ok = rx[j] == tx[j];
        if (!ok)
            uart_putc('0' + i);
        check("echo", ok);
    }
    /* a packet arrived on Link1 */
    check("token", (op(t9_ldconf, 0x8103, 0, 0) & 0x20) != 0);
    check("vcpstatus", op(t9_ldconf, 0x0802, 0, 0) == 0);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
