/* Two T9000s, the second controlled over its control link (HRM ch.8).
 *
 * Node 0 (this program) is the controlling processor.  Its Link2 runs in
 * virtual mode with 2-byte headers and is cabled to node 1's CLink0: the
 * controlling process reaches node 1's control unit over virtual link 0,
 * whose header (#1234) becomes node 1's label and whose number node 1 is
 * given as its return header (HRM 8.1, 8.3.1, 8.6.1).  Link1 joins the two
 * data networks and stays in byte-stream mode at node 0.
 *
 * Node 1 is never booted down a data link.  Node 0:
 *   A  labels it (Start), identifies it, configures its VCP with CPoke,
 *      loads a program with Boot + BootData and starts it with Run;
 *   B  sends it a packet its VCP must reject: on a reliable link that is a
 *      VCP error, which stops node 1's CPU and comes back as an Error
 *      message (HRM 8.5, 13.5.1); Reset 3, then CPeek/Peek the evidence;
 *   C  Reset 2, reboots it with a program that halts on error: a CPU error;
 *   D  Stop and Reset 3 on a running program;
 *   E  control-link protocol errors: an invalid command code, RecoverError
 *      re-sending an unhandshaken error, a command overtaking a handshake.
 *
 * Command codes and handshake layout: ../t9000_ctl.h.  Probes are in
 * test_t9000_vcp_fn.S.  Needs two threaded nodes: run_t9000_vcp.sh starts
 * it with -st -sn -2 and the network map there.
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

#define VIN0    0x80000040u
#define VOUT0   0x80000044u

/* control commands and responses (../t9000_ctl.h) */
#define C_IDENTIFY  0x00
#define C_CPEEK     0x02
#define C_CPOKE     0x03
#define C_START     0x10
#define C_RESET     0x11
#define C_STOP      0x12
#define C_RECOVER   0x13
#define C_ERRORHS   0x60
#define C_PEEK      0x74
#define C_POKE      0x75
#define C_BOOT      0x76
#define C_BOOTDATA  0x77
#define C_RUN       0x78
#define M_ERROR     0xE0

/* node 1's memory */
#define CODE1   0x80010000u
#define WS1     0x80020000u
#define CODE2   0x80011000u
#define WS2     0x80021000u
#define CODE3   0x80012000u
#define WS3     0x80022000u
#define SCRATCH 0x80030000u

/* ldpri; stl 9; ldc #5A; stl 8; stopp  - proves it ran, and at which
 * priority (HRM 10.5.2: a high-priority process, as a boot from ROM) */
static const u8 prog1[32] = {
    0x21, 0xFE, 0xD9, 0x25, 0x4A, 0xD8, 0x21, 0xF5
};
/* ldc #77; stl 8; sethalterr; seterr; stopp  - halts on error */
static const u8 prog2[16] = {
    0x27, 0x47, 0xD8, 0x25, 0xF8, 0x21, 0xF0, 0x21, 0xF5
};
/* loop: ldl 8; adc 1; stl 8; ldtimer; adc 100; tin; j loop */
static const u8 prog3[16] = {
    0x78, 0x81, 0xD8, 0x22, 0xF2, 0x26, 0x84, 0x22, 0xFB, 0x60, 0x05
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

static u32 le32(const u8 *p)
{
    return p[0] | (u32)p[1] << 8 | (u32)p[2] << 16 | (u32)p[3] << 24;
}

static void put32(u8 *p, u32 v)
{
    p[0] = (u8)v; p[1] = (u8)(v >> 8); p[2] = (u8)(v >> 16); p[3] = (u8)(v >> 24);
}

static u32 pkt[8];
static u8 msg[24], rsp[40];
static u32 rlen;

/* One message to node 1's control unit; the output completes on its
 * acknowledge packet. */
static void send(int n)
{
    op(t9_vout, (u32)n, VOUT0, (u32)msg);
}

/* The next message from node 1: a handshake or an Error message. */
static u32 receive(void)
{
    u32 io[5];
    int i;

    for (i = 0; i < 40; i++)
        rsp[i] = 0xEE;
    io[0] = sizeof rsp; io[1] = VIN0; io[2] = (u32)rsp;
    t9_vin_cnt(io);
    rlen = io[4] ? io[3] : 0xFFFFFFFFu;
    return rlen;
}

/* Send a command and take its handshake: the status byte, or -1 if the
 * reply is not the expected response code and length. */
static int command(int n, int rlen_want)
{
    send(n);
    receive();
    if (rlen != (u32)rlen_want || rsp[0] != (u8)(msg[0] ^ 0x80)) {
        uart_putc('<');
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

static int c_simple(int code)
{
    msg[0] = (u8)code;
    return command(1, 2);
}

static int c_reset(int level)
{
    msg[0] = C_RESET; msg[1] = (u8)level;
    return command(2, 2);
}

static int c_cpoke(u32 a, u32 v)
{
    msg[0] = C_CPOKE; msg[1] = (u8)a; msg[2] = (u8)(a >> 8);
    put32(msg + 3, v);
    return command(7, 2);
}

/* CPeek: the value, or 0xBAD0000s with the status in s */
static u32 c_cpeek(u32 a)
{
    int st;
    msg[0] = C_CPEEK; msg[1] = (u8)a; msg[2] = (u8)(a >> 8);
    st = command(3, 6);
    return st == 0 ? le32(rsp + 2) : 0xBAD00000u | (u32)(st & 0xFF);
}

static int c_poke(u32 a, u32 v)
{
    msg[0] = C_POKE; put32(msg + 1, a); put32(msg + 5, v);
    return command(9, 2);
}

static u32 c_peek(u32 a)
{
    int st;
    msg[0] = C_PEEK; put32(msg + 1, a);
    st = command(5, 6);
    return st == 0 ? le32(rsp + 2) : 0xBAD00000u | (u32)(st & 0xFF);
}

static int c_run(u32 wptr, u32 iptr)
{
    msg[0] = C_RUN; put32(msg + 1, wptr); put32(msg + 5, iptr);
    return command(9, 2);
}

/* Boot + BootData: 0 if every handshake was clean */
static int c_load(u32 addr, const u8 *code, int len)
{
    int i, j, bad = 0;

    msg[0] = C_BOOT; put32(msg + 1, (u32)len); put32(msg + 5, addr);
    bad |= command(9, 2);
    for (i = 0; i < len; i += 16) {
        msg[0] = C_BOOTDATA;
        for (j = 0; j < 16; j++)
            msg[1 + j] = code[i + j];
        bad |= command(17, 2);
    }
    return bad;
}

/* An Error message carrying `code`, then its ErrorHandshake */
static int error_message(u32 code)
{
    int ok;
    receive();
    ok = rlen == 2 && rsp[0] == M_ERROR && rsp[1] == (u8)code;
    if (!ok) {
        uart_putc('[');
        uart_hex(rlen);
        uart_putc(' ');
        uart_hex(le32(rsp));
        uart_putc(']');
    }
    msg[0] = C_ERRORHS;
    send(1);                    /* no reply */
    return ok;
}

static void setup_controller(void)
{
    static const u8 label[2] = { 0x34, 0x12 };

    op(t9_stconf, 0x010F, VIN0 + 8, 0);         /* MinInvalidChannel: VL0 */
    op(t9_stconf, 0x080D, 4, 0);                /* Link2: virtual, 2-byte headers */
    op(t9_initvlcb, VIN0, (u32)pkt, 0);
    op(t9_sethdr, VIN0, 2, 0xFFFFFFFFu);        /* Link2, short header */
    op(t9_writehdr, VIN0, 2, (u32)label);       /* = node 1's label */
    op(t9_stconf, 0x0804, 2, 0);                /* start the VCP */
    op(t9_setchmode, VIN0, 1, 0);
    op(t9_setchmode, VOUT0, 1, 0);
}

void test_main(void)
{
    static const u8 bad[2] = { 200, 2 };        /* header 200, EOM */
    u32 id, v, c1;
    int i;

    setup_controller();
    id = op(t9_ldconf, 0x1001, 0, 0) & 0xFFFF;  /* same device type */

    /* ---- A: label, configure, load and start node 1 ---- */
    msg[0] = C_START; msg[1] = 0; msg[2] = 0;   /* return header 0 = VL0 */
    check("start", command(3, 2) == 0);
    msg[0] = C_IDENTIFY;
    check("identify", command(1, 6) == 0 && le32(rsp + 2) == id);
    check("cpoke-minvc", c_cpoke(0x010F, VIN0 + 8) == 0);
    check("cpoke-l1mode", c_cpoke(0x0809, 0) == 0);    /* virtual, reliable */
    check("cpoke-l1max", c_cpoke(0x080A, 0) == 0);
    check("cpoke-start", c_cpoke(0x0804, 2) == 0);     /* VCP running */
    check("cpeek", c_cpeek(0x010F) == VIN0 + 8 && c_cpeek(0x0809) == 0);
    check("cpeek-bad", c_cpeek(0x7777) == 0xBAD00001u);
    check("cpeek-wo", c_cpeek(0xFE02) == 0xBAD00001u); /* CLink1Command */
    check("poke", c_poke(SCRATCH, 0xCAFEF00Du) == 0 && c_peek(SCRATCH) == 0xCAFEF00Du);
    check("peek-align", c_peek(SCRATCH + 1) == 0xBAD00001u);
    check("load1", c_load(CODE1, prog1, sizeof prog1) == 0);
    msg[0] = C_BOOTDATA;
    check("bootdata-ready", command(17, 2) == 1);      /* not booting */
    check("loaded", c_peek(CODE1) == le32(prog1));
    check("run-align", c_run(WS1 + 2, CODE1) == 1);
    check("run", c_run(WS1, CODE1) == 0);
    check("peek-running", c_peek(SCRATCH) == 0xBAD00001u);

    /* ---- B: a VCP error on a reliable link ---- */
    ChanOut(LINK1OUT, (void *)bad, 2);
    check("vcp-error", error_message(0x02));
    msg[0] = C_IDENTIFY;
    check("identify-error", command(1, 6) == 0 && le32(rsp + 2) == id);
    check("cpeek-error", c_cpeek(0x0802) == 0xBAD00001u);  /* invalid in Error */
    check("reset-level", c_reset(7) == 1);
    check("reset3", c_reset(3) == 0);
    v = c_cpeek(0x0802);                        /* VCPStatus: error, range, Link1, 200 */
    check("vcpstatus", v == (1u | (1u << 4) | (200u << 16)));
    check("errorcode", c_cpeek(0x1004) == 0x02);
    check("modestatus", c_cpeek(0x1003) == 1);  /* ErrorSinceReset */
    check("clink0", (c_cpeek(0xFD03) & 0x22) == 0x22);
    check("ran", c_peek(WS1 + 32) == 0x5A);
    check("hipri", c_peek(WS1 + 36) == 0);
    check("state", c_peek(WS1) == 0x80000000u);    /* previous Wdesc */

    /* ---- C: Reset 2, reboot, a CPU error ---- */
    check("reset2", c_reset(2) == 0);
    check("vcp-cleared", c_cpeek(0x0802) == 0);
    check("vcp-config", c_cpeek(0x0809) == 0);  /* configuration kept */
    check("esr-kept", c_cpeek(0x1003) == 1);
    check("esr-clear", c_cpoke(0x1003, 0) == 0 && c_cpeek(0x1003) == 0);
    check("load2", c_load(CODE2, prog2, sizeof prog2) == 0);
    check("run2", c_run(WS2, CODE2) == 0);
    check("cpu-error", error_message(0x01));
    check("reset3b", c_reset(3) == 0);
    check("ran2", c_peek(WS2 + 32) == 0x77);
    check("errorcode2", c_cpeek(0x1004) == 0x01 && c_cpeek(0x1003) == 1);

    /* ---- D: Stop a running program ---- */
    check("load3", c_load(CODE3, prog3, sizeof prog3) == 0);
    check("zero", c_poke(WS3 + 32, 0) == 0);
    check("run3", c_run(WS3, CODE3) == 0);
    for (i = 0; i < 20; i++) {                  /* let it count a while */
        msg[0] = C_IDENTIFY;
        command(1, 6);
    }
    check("stop", c_simple(C_STOP) == 0);
    check("stop-again", c_simple(C_STOP) == 0);  /* no effect */
    check("reset3c", c_reset(3) == 0);
    c1 = c_peek(WS3 + 32);
    check("counted", c1 != 0 && c1 < 0xBAD00000u);
    for (i = 0; i < 10; i++) {
        msg[0] = C_IDENTIFY;
        command(1, 6);
    }
    check("stopped", c_peek(WS3 + 32) == c1);
    check("stop-ready", c_simple(C_STOP) == 1);  /* invalid in Ready */

    /* ---- E: control link errors ---- */
    msg[0] = 0x55;                              /* not a command */
    send(1);
    check("bad-code", error_message(0xC0));
    msg[0] = 0x55;
    send(1);
    receive();                                  /* the Error message... */
    check("bad-code2", rlen == 2 && rsp[0] == M_ERROR && rsp[1] == 0xC0);
    check("recover", c_simple(C_RECOVER) == 0); /* ...not handshaken: */
    check("resent", error_message(0xC0));       /* sent again */
    msg[0] = C_IDENTIFY;
    send(1);
    send(1);                                    /* overtakes the handshake */
    receive();
    check("overtaken", rlen == 6 && rsp[0] == 0x80);
    check("hs-protocol", error_message(0xC1));
    msg[0] = C_START; msg[1] = 0; msg[2] = 0;
    check("restart", command(3, 2) == 0);
    check("last", c_cpeek(0x1004) == 0xC1);

    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
