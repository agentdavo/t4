/* Two T9000s and two IMS C104 packet routing switches: virtual channels
 * across a routed network (HRM part 1 ch.7, T4 router in t4_fabric.c).
 *
 *              +------ R2 ------+          +------ R3 ------+
 *   node 0  L1 | 0            2 |==========| 2            0 | L1  node 1
 *           L2 | 1            3 |==========| 3              |
 *              |              4 |---+      +----------------+
 *              +----------------+   +-------------------------- L2
 *           L3 --------------------- boot ------------------- L3
 *
 * Path A (the Link1s; HRM 7.2.1 and figure 7.5b): two 1-byte headers.  The
 * first names the destination node, 0 or 1, and is deleted by the switch
 * output leading to that node, which exposes the second: the virtual link
 * there.  It crosses both switches, over either cable between them (the
 * two form a group, so a packet takes whichever is free).
 *
 * Path B (the Link2s; figure 7.5a): one network-wide 2-byte label per
 * virtual channel - node 0 owns #000-#0FF, node 1 #100-#1FF - so nothing is
 * deleted and node 1 maps a label to its virtual link with
 * VCPLink2HdrOffset = #100.  Router 2 reads 2-byte headers on these inputs
 * only, with its own interval table there.
 *
 * Node 0 (this program) boots node 1 down byte-stream Link3 with the echo
 * server below, which reports back on Link3 once its VCP is up and then
 * answers on VL0 (path A), VL1 (path B) and VL2 (path A) in turn.  Node 0
 * runs the three exchanges one after another, then all three at once from
 * separate processes, so packets of both path-A channels are in the
 * switches together.
 *
 * Needs two threaded nodes plus the routers in the network map:
 * tests/run_t9000_vcp.sh starts it with -st -sn -2 and writes the map.
 * Probes are in test_t9000_vcp_fn.S.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }

typedef unsigned int u32;
typedef unsigned char u8;

extern void t9_stconf(u32 *), t9_ldconf(u32 *), t9_initvlcb(u32 *), t9_sethdr(u32 *);
extern void t9_writehdr(u32 *), t9_setchmode(u32 *), t9_vout(u32 *), t9_vin_cnt(u32 *);

#define VIN(n)  (0x80000040u + 8u * (n))
#define VOUT(n) (VIN(n) + 4u)
#define NVL     3

/* Node 1, loaded at MemStart (assembled from):
 *      ajw 64
 *      mint; adc #58; ldc #10F; stconf          MinInvalidChannel: three VLs
 *      ldc 0; ldc #809; stconf                  Link1: virtual, 1-byte hdr
 *      ldc 4; ldc #80D; stconf                  Link2: virtual, 2-byte hdr,
 *      ldc #100; ldc #810; stconf                 HdrOffset #100
 *      ldlp 8;  mint; adc #40; initvlcb         packet buffers: locals 8,
 *      ldlp 16; mint; adc #48; initvlcb           16 and 24
 *      ldlp 24; mint; adc #50; initvlcb
 *      ldc -1; ldc 1; mint; adc #40; sethdr     VL0 on Link1, short header
 *      ldc -1; ldc 2; mint; adc #48; sethdr     VL1 on Link2
 *      ldc -1; ldc 1; mint; adc #50; sethdr     VL2 on Link1
 *      ldc 0;    stl 0; ldlp 0; ldc 2; mint; adc #40; writehdr  [0, 0]
 *      ldc 1;    stl 0; ldlp 0; ldc 2; mint; adc #48; writehdr  #0001
 *      ldc #200; stl 0; ldlp 0; ldc 2; mint; adc #50; writehdr  [0, 2]
 *      ldc 2; ldc #804; stconf                  start the VCP
 *      ldc 1; mint; adc #40..#54; setchmode     activate all six channels
 *      mint; adc #C; ldc 0; outbyte             ready: a byte down Link3
 * loop: (for VL0, VL1, VL2)
 *      ldlp 32; mint; adc VIN; ldc 128; vin; ldcnt; stl 1
 *      ldlp 32; mint; adc VOUT; ldl 1; vout
 *      j loop
 * padded to a whole number of words: the booted process's Wptr is the
 * first byte after the code.
 */
static const u8 boot1[] = {
    0xEC, 0x24, 0xB0, 0x24, 0xF2, 0x25, 0x88, 0x21, 0x20, 0x4F, 0x2B, 0xFF,
    0x40, 0x28, 0x20, 0x49, 0x2B, 0xFF, 0x44, 0x28, 0x20, 0x4D, 0x2B, 0xFF,
    0x21, 0x20, 0x40, 0x28, 0x21, 0x40, 0x2B, 0xFF, 0x18, 0x24, 0xF2, 0x24,
    0x80, 0x61, 0xF6, 0x21, 0x10, 0x24, 0xF2, 0x24, 0x88, 0x61, 0xF6, 0x21,
    0x18, 0x24, 0xF2, 0x25, 0x80, 0x61, 0xF6, 0x60, 0x4F, 0x41, 0x24, 0xF2,
    0x24, 0x80, 0x61, 0xF8, 0x60, 0x4F, 0x42, 0x24, 0xF2, 0x24, 0x88, 0x61,
    0xF8, 0x60, 0x4F, 0x41, 0x24, 0xF2, 0x25, 0x80, 0x61, 0xF8, 0x40, 0xD0,
    0x10, 0x42, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF5, 0x41, 0xD0, 0x10, 0x42,
    0x24, 0xF2, 0x24, 0x88, 0x61, 0xF5, 0x22, 0x20, 0x40, 0xD0, 0x10, 0x42,
    0x24, 0xF2, 0x25, 0x80, 0x61, 0xF5, 0x42, 0x28, 0x20, 0x44, 0x2B, 0xFF,
    0x41, 0x24, 0xF2, 0x24, 0x80, 0x61, 0xF7, 0x41, 0x24, 0xF2, 0x24, 0x84,
    0x61, 0xF7, 0x41, 0x24, 0xF2, 0x24, 0x88, 0x61, 0xF7, 0x41, 0x24, 0xF2,
    0x24, 0x8C, 0x61, 0xF7, 0x41, 0x24, 0xF2, 0x25, 0x80, 0x61, 0xF7, 0x41,
    0x24, 0xF2, 0x25, 0x84, 0x61, 0xF7, 0x24, 0xF2, 0x8C, 0x40, 0xFE, 0x22,
    0x10, 0x24, 0xF2, 0x24, 0x80, 0x28, 0x40, 0x61, 0xFC, 0x2C, 0xF0, 0xD1,
    0x22, 0x10, 0x24, 0xF2, 0x24, 0x84, 0x71, 0x61, 0xFD, 0x22, 0x10, 0x24,
    0xF2, 0x24, 0x88, 0x28, 0x40, 0x61, 0xFC, 0x2C, 0xF0, 0xD1, 0x22, 0x10,
    0x24, 0xF2, 0x24, 0x8C, 0x71, 0x61, 0xFD, 0x22, 0x10, 0x24, 0xF2, 0x25,
    0x80, 0x28, 0x40, 0x61, 0xFC, 0x2C, 0xF0, 0xD1, 0x22, 0x10, 0x24, 0xF2,
    0x25, 0x84, 0x71, 0x61, 0xFD, 0x64, 0x0C, 0x00, 0x00
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

static u32 pkt[NVL][8];
static u8 tx[NVL][128], rx[NVL][128];

/* Send len bytes on virtual link vl and check node 1's echo. */
static int exchange(int vl, int len, int seed)
{
    u32 n;
    int j, ok;

    for (j = 0; j < len; j++)
        tx[vl][j] = (u8)(seed * 31 + j * 5 + vl * 77);
    op(t9_vout, len, VOUT(vl), (u32)tx[vl]);
    for (j = 0; j < 128; j++)
        rx[vl][j] = 0xEE;
    n = vin(VIN(vl), 128, rx[vl]);
    ok = n == (u32)len;
    for (j = 0; ok && j < len; j++)
        ok = rx[vl][j] == tx[vl][j];
    if (!ok) {
        uart_putc('0' + vl);
        uart_putc('a' + seed);
    }
    return ok;
}

#define ROUNDS  6
#define LONGMSG 100     /* four packets */

static int rounds(int vl)
{
    int r, ok = 1;

    for (r = 0; r < ROUNDS; r++)
        ok &= exchange(vl, LONGMSG, r + 10);
    return ok;
}

static Channel done = NOPROCESS;
static int ws1[256], ws2[256];

static void worker1(void) { int ok = rounds(1); ChanOut(&done, &ok, 4); PStop(); }
static void worker2(void) { int ok = rounds(2); ChanOut(&done, &ok, 4); PStop(); }

void test_main(void)
{
    static const int lens[] = { 0, 1, 31, 32, 33, 64, 128, 7 };
    static const u8 hdr0[2] = { 1, 0 };         /* node 1, its VL0 */
    static const u8 hdr1[2] = { 0x01, 0x01 };   /* label #101: node 1's VL1 */
    static const u8 hdr2[2] = { 1, 2 };         /* node 1, its VL2 */
    int i, vl, ok;

    ChanOut(LINK3OUT, (void *)boot1, sizeof boot1);

    op(t9_stconf, 0x010F, VIN(NVL), 0);         /* MinInvalidChannel */
    op(t9_stconf, 0x0809, 0, 0);                /* Link1: virtual, 1-byte */
    op(t9_stconf, 0x080D, 4, 0);                /* Link2: virtual, 2-byte, */
    op(t9_stconf, 0x080E, 0xFF, 0);             /*   labels #000-#0FF */
    for (vl = 0; vl < NVL; vl++)
        op(t9_initvlcb, VIN(vl), (u32)pkt[vl], 0);
    op(t9_sethdr, VIN(0), 1, 0xFFFFFFFFu);
    op(t9_sethdr, VIN(1), 2, 0xFFFFFFFFu);
    op(t9_sethdr, VIN(2), 1, 0xFFFFFFFFu);
    op(t9_writehdr, VIN(0), 2, (u32)hdr0);
    op(t9_writehdr, VIN(1), 2, (u32)hdr1);
    op(t9_writehdr, VIN(2), 2, (u32)hdr2);
    op(t9_stconf, 0x0804, 2, 0);                /* start */
    for (vl = 0; vl < NVL; vl++) {
        op(t9_setchmode, VIN(vl), 1, 0);
        op(t9_setchmode, VOUT(vl), 1, 0);
    }
    /* Until it has booted, node 1 takes the first message on any link as
     * boot data, so nothing may be routed to it before it says it is up. */
    ChanInChar(LINK3IN);

    /* one channel at a time, in node 1's order */
    for (i = 0; i < (int)(sizeof lens / sizeof lens[0]); i++)
        for (vl = 0; vl < NVL; vl++)
            check(vl == 1 ? "pathB" : "pathA", exchange(vl, lens[i], i));

    /* all three at once */
    PRun(PSetup(ws1, sizeof ws1, worker1) | PRI_LOW);
    PRun(PSetup(ws2, sizeof ws2, worker2) | PRI_LOW);
    check("par0", rounds(0));
    ChanIn(&done, &ok, 4);
    check("par", ok);
    ChanIn(&done, &ok, 4);
    check("par", ok);

    check("vcpstatus", op(t9_ldconf, 0x0802, 0, 0) == 0);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
