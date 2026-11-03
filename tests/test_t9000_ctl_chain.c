/* Three T9000s on a daisy-chained control network (HRM 8.3.1, fig 8.5).
 *
 *   node 0 Link2 --- CLink0 node 1 CLink1 --- CLink0 node 2
 *
 * Node 0's controlling process has one virtual link per device on its
 * Link2: VL0 (header #1234) for node 1, VL1 (header #1256) for node 2.
 * Node 1 takes the first header it sees as its label and passes every
 * other packet down CLink1 once that link is started (a CPoke of
 * CLink1Command); everything arriving on its CLink1 goes back up CLink0.
 * So node 2 is labelled, identified, booted, started and reset entirely
 * through node 1, and its Error message comes back the same way.  Last, a
 * parity fault on the node 1 - node 2 control cable: node 1 reports #C2,
 * the controller restarts CLink1 and resynchronises node 2 with
 * RecoverError.
 *
 * Needs three threaded nodes: run_t9000_vcp.sh starts it with -st -sn -3.
 */

#include <conc.h>

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void uart_putc(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void uart_puts(const char *s) { while (*s) uart_putc(*s++); }
static void uart_hex(unsigned int v)
{
    int i;
    for (i = 28; i >= 0; i -= 4)
        uart_putc("0123456789abcdef"[(v >> i) & 15]);
}

typedef unsigned int u32;
typedef unsigned char u8;

extern void t9_stconf(u32 *), t9_ldconf(u32 *), t9_initvlcb(u32 *), t9_sethdr(u32 *);
extern void t9_writehdr(u32 *), t9_setchmode(u32 *), t9_vout(u32 *), t9_vin_cnt(u32 *);

#define VIN(n)  (0x80000040u + 8u * (n))
#define VOUT(n) (0x80000044u + 8u * (n))

#define C_IDENTIFY  0x00
#define C_CPEEK     0x02
#define C_CPOKE     0x03
#define C_START     0x10
#define C_RESET     0x11
#define C_RECOVER   0x13
#define C_ERRORHS   0x60
#define C_PEEK      0x74
#define C_BOOT      0x76
#define C_BOOTDATA  0x77
#define C_RUN       0x78
#define M_ERROR     0xE0

#define CODE    0x80010000u
#define WS      0x80020000u

/* ldc #5A; stl 8; stopp */
static const u8 prog[16] = { 0x25, 0x4A, 0xD8, 0x21, 0xF5 };

static int pass = 1;
static int dev;                         /* virtual link = device */

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

static u32 le32(const u8 *p)
{
    return p[0] | (u32)p[1] << 8 | (u32)p[2] << 16 | (u32)p[3] << 24;
}

static void put32(u8 *p, u32 v)
{
    p[0] = (u8)v; p[1] = (u8)(v >> 8); p[2] = (u8)(v >> 16); p[3] = (u8)(v >> 24);
}

static u32 pkt[2][8];
static u8 msg[24], rsp[40];
static u32 rlen;

static void send(int n)
{
    op(t9_vout, (u32)n, VOUT(dev), (u32)msg);
}

static void receive(void)
{
    u32 io[5];
    io[0] = sizeof rsp; io[1] = VIN(dev); io[2] = (u32)rsp;
    t9_vin_cnt(io);
    rlen = io[4] ? io[3] : 0xFFFFFFFFu;
}

static int command(int n, int rlen_want)
{
    send(n);
    receive();
    if (rlen != (u32)rlen_want || rsp[0] != (u8)(msg[0] ^ 0x80)) {
        uart_putc('<');
        uart_hex(dev);
        uart_hex(msg[0]);
        uart_putc(' ');
        uart_hex(rlen);
        uart_putc(' ');
        uart_hex(rsp[0]);
        uart_putc('>');
        return -1;
    }
    return rsp[1];
}

static u32 c_identify(void)
{
    msg[0] = C_IDENTIFY;
    return command(1, 6) == 0 ? le32(rsp + 2) : 0xBAD00000u;
}

static int c_cpoke(u32 a, u32 v)
{
    msg[0] = C_CPOKE; msg[1] = (u8)a; msg[2] = (u8)(a >> 8);
    put32(msg + 3, v);
    return command(7, 2);
}

static u32 c_cpeek(u32 a)
{
    msg[0] = C_CPEEK; msg[1] = (u8)a; msg[2] = (u8)(a >> 8);
    return command(3, 6) == 0 ? le32(rsp + 2) : 0xBAD00000u;
}

static u32 c_peek(u32 a)
{
    msg[0] = C_PEEK; put32(msg + 1, a);
    return command(5, 6) == 0 ? le32(rsp + 2) : 0xBAD00000u;
}

static int c_start(int vl)
{
    msg[0] = C_START; msg[1] = (u8)vl; msg[2] = 0;
    return command(3, 2);
}

void test_main(void)
{
    static const u8 label[2][2] = { { 0x34, 0x12 }, { 0x56, 0x12 } };
    u32 id;
    int i;

    op(t9_stconf, 0x010F, VIN(2), 0);           /* MinInvalidChannel: VL0-1 */
    op(t9_stconf, 0x080D, 4, 0);                /* Link2: virtual, 2-byte headers */
    for (i = 0; i < 2; i++) {
        op(t9_initvlcb, VIN(i), (u32)pkt[i], 0);
        op(t9_sethdr, VIN(i), 2, 0xFFFFFFFFu);
        op(t9_writehdr, VIN(i), 2, (u32)label[i]);
    }
    op(t9_stconf, 0x0804, 2, 0);                /* start the VCP */
    for (i = 0; i < 2; i++) {
        op(t9_setchmode, VIN(i), 1, 0);
        op(t9_setchmode, VOUT(i), 1, 0);
    }
    id = op(t9_ldconf, 0x1001, 0, 0) & 0xFFFF;

    /* node 1: label it, then open its CLink1 to the rest of the chain */
    dev = 0;
    check("start1", c_start(0) == 0);
    check("clink1-idle", (c_cpeek(0xFE03) & 2) == 0);
    check("clink1-start", c_cpoke(0xFE02, 2) == 0);    /* StartLink */
    check("clink1-started", (c_cpeek(0xFE03) & 2) == 2);

    /* node 2, through node 1 */
    dev = 1;
    check("start2", c_start(1) == 0);
    check("identify2", c_identify() == id);
    check("clink0-2", (c_cpeek(0xFD03) & 0x22) == 0x22);
    msg[0] = C_BOOT; put32(msg + 1, sizeof prog); put32(msg + 5, CODE);
    check("boot2", command(9, 2) == 0);
    msg[0] = C_BOOTDATA;
    for (i = 0; i < 16; i++)
        msg[1 + i] = prog[i];
    check("bootdata2", command(17, 2) == 0);
    msg[0] = C_RUN; put32(msg + 1, WS); put32(msg + 5, CODE);
    check("run2", command(9, 2) == 0);

    /* both devices answer, interleaved */
    dev = 0;
    check("identify1", c_identify() == id);
    check("through", (c_cpeek(0xFE03) & 0x20) == 0x20);   /* tokens on CLink1 */

    /* node 2 again: reset its CPU and read what the program left */
    dev = 1;
    msg[0] = C_RESET; msg[1] = 3;
    check("reset2", command(2, 2) == 0);
    check("ran2", c_peek(WS + 32) == 0x5A);
    msg[0] = 0x55;                              /* an invalid command */
    send(1);
    receive();
    check("error2", rlen == 2 && rsp[0] == M_ERROR && rsp[1] == 0xC0);
    msg[0] = C_ERRORHS;
    send(1);
    check("errorcode2", c_cpeek(0x1004) == 0xC0);

    /* node 1 saw none of that as its own */
    dev = 0;
    check("errorcode1", c_cpeek(0x1004) == 0);

    /* a fault on the chain (HRM 8.5.3): bad parity out of node 1's CLink1
     * is a parity error at node 2's CLink0, which cannot report it but
     * halts its output; node 1 sees a disconnect on CLink1 and reports #C2 */
    check("wrongparity", c_cpoke(0xFE02, 8) == 0);
    receive();
    check("clink1-error", rlen == 2 && rsp[0] == M_ERROR && rsp[1] == 0xC2);
    msg[0] = C_ERRORHS;
    send(1);
    check("clink1-disc", (c_cpeek(0xFE03) & 0x1B) == 0x11);
    check("clink1-restart", c_cpoke(0xFE02, 3) == 0 && (c_cpeek(0xFE03) & 0x1B) == 0x02);
    check("errorcode1b", c_cpeek(0x1004) == 0xC2);
    /* node 2's CLink0 restarts on the next token; RecoverError resets the
     * virtual link's acknowledge state, which the fault may have upset */
    dev = 1;
    msg[0] = C_RECOVER;
    check("recover2", command(1, 2) == 0);
    check("identify2b", c_identify() == id);
    check("clink0-parity", (c_cpeek(0xFD03) & 0x0B) == 0x0B);

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
