/* T9000 external channels on the t4 emulator (t9000_vcp.c): configuration
 * space, VLCB set-up, virtual links through the VCP, variable-length
 * messages, resource channels, event channels, byte-stream links, and the
 * shadow / trap-return state instructions.  Probes are raw encodings in
 * test_t9000_vcp_fn.S.
 *
 * Wiring, set by run_t9000_vcp.sh:
 *   T4_LINK_LOOPBACK=1,2-3   Link1 is cabled back to itself and carries the
 *                            virtual links; Link2 and Link3 are crossed and
 *                            stay in byte-stream mode.
 *   T4_EVENT_LOOPBACK=1      EventOut0 drives EventIn1 and EventOut1 drives
 *                            EventIn0, so one event pair handshakes with the
 *                            other.
 *
 * Virtual links: VL0's header names VL1 and VL1's names VL0 (likewise VL2
 * and VL3), so output on VL0 arrives at VL1's input and VL1 returns the
 * acknowledges, as two transputers joined by a cable would.
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

extern void t9_insphdr(u32 *), t9_readbfr(u32 *), t9_ldconf(u32 *), t9_stconf(u32 *);
extern void t9_ldchstatus(u32 *), t9_cir(u32 *), t9_chantype(u32 *), t9_ciru(u32 *);
extern void t9_ldmemstartval(u32 *), t9_ldth(u32 *), t9_selth(u32 *), t9_ldpri(u32 *);
extern void t9_resetch(u32 *), t9_stopch(u32 *), t9_vout(u32 *), t9_swapbfr(u32 *);
extern void t9_sethdr(u32 *), t9_setchmode(u32 *), t9_initvlcb(u32 *), t9_writehdr(u32 *);
extern void t9_readhdr(u32 *), t9_grant(u32 *), t9_unmkrc(u32 *), t9_mkrc(u32 *);
extern void t9_irdsq(u32 *), t9_erdsq(u32 *), t9_stresptr(u32 *), t9_ldresptr(u32 *);
extern void t9_stshadow(u32 *), t9_ldshadow(u32 *), t9_in(u32 *), t9_out(u32 *);
extern void t9_outword(u32 *), t9_outbyte(u32 *);
extern void t9_testerr(u32 *), t9_vin_cnt(u32 *), t9_alt_res(u32 *);
extern void t9_tret(u32 *), t9_restart(u32 *);

#define NOTPROC     0x80000000u
#define RESCHAN     0x80000002u
#define DEACT       0x80000001u
#define LENERR      0xFFFFFFFFu
#define NULHDR      0xFFFFFFFFu
#define NULOFF      0xFFFFFFFFu

#define CR_MININVALID   0x010Fu
#define CR_VCPSTATUS    0x0802u
#define CR_VCPCOMMAND   0x0804u
#define CR_LINKMODE(n)  (0x0805u + 4u * (n))
#define CR_MAXHDR(n)    (0x0806u + 4u * (n))
#define CR_MINHDR(n)    (0x0807u + 4u * (n))
#define CR_HDROFF(n)    (0x0808u + 4u * (n))
#define CR_HDRAREA      0x0901u
#define CR_DEVICEID     0x1001u
#define CR_MEMSTART     0x410Eu
#define CR_EXTRCBASE    0x4110u
#define CR_LINKSTATUS(n) (0x8003u + 0x100u * (n))

#define VIN(n)      (0x80000040u + 8u * (n))
#define VOUT(n)     (0x80000044u + 8u * (n))
#define EVIN(n)     (0x80000020u + 8u * (n))
#define EVOUT(n)    (0x80000024u + 8u * (n))
#define NVL         5

static int pass = 1;

static void check(const char *name, int ok)
{
    if (ok)
        return;
    uart_puts(name);
    uart_puts(":F ");
    pass = 0;
}

static void checkv(const char *name, u32 got, u32 want)
{
    check(name, got == want);
    if (got != want) {
        uart_puts("(g=");
        uart_hex(got);
        uart_puts(" w=");
        uart_hex(want);
        uart_puts(") ");
    }
}

/* ---- probe wrappers: each caller has its own io block ------------------ */

static u32 op1(void (*f)(u32 *), u32 a, u32 b, u32 c, u32 *bo, u32 *co)
{
    u32 io[4];
    io[0] = a; io[1] = b; io[2] = c; io[3] = 0;
    f(io);
    if (bo) *bo = io[1];
    if (co) *co = io[2];
    return io[0];
}

static int errflag(void)                /* 1 if the error flag was set */
{
    u32 t[1];
    t9_testerr(t);
    return t[0] == 0;
}

static u32 ldconf(u32 a)           { return op1(t9_ldconf, a, 0, 0, 0, 0); }
static void stconf(u32 a, u32 v)   { op1(t9_stconf, a, v, 0, 0, 0); }
static void setchmode(u32 c, u32 on) { op1(t9_setchmode, c, on, 0, 0, 0); }
static u32 chantype(u32 c)         { return op1(t9_chantype, c, 0, 0, 0, 0); }
static u32 resetch(u32 c)          { return op1(t9_resetch, c, 0, 0, 0, 0); }
static u32 ldchstatus(u32 c, u32 *b, u32 *cc) { return op1(t9_ldchstatus, c, 0, 0, b, cc); }

static void vout(u32 chan, u32 len, const void *src)
{
    op1(t9_vout, len, chan, (u32)src, 0, 0);
}

/* returns ldcnt; *err = error flag after ldcnt */
static u32 vin(u32 chan, u32 max, void *dst, int *err)
{
    u32 io[5];
    io[0] = max; io[1] = chan; io[2] = (u32)dst;
    t9_vin_cnt(io);
    if (err) *err = io[4] == 0;
    return io[3];
}

static void grant(u32 *rds, u32 *idp) { op1(t9_grant, (u32)rds, (u32)idp, 0, 0, 0); }
static void mkrc(u32 chan, u32 *rds, u32 id) { op1(t9_mkrc, chan, (u32)rds, id, 0, 0); }

/* ---- shared state ------------------------------------------------------- */

static u32 pktbuf[NVL][8];              /* one maximum packet per link */
static u32 altbuf[8];
static u32 rcblock[2 * (4 + NVL)];      /* ExternalRCbase: events then VLs */
static u32 hdrarea[4];                  /* HdrAreaBase */

static Channel done = NOPROCESS;        /* a soft channel for chantype */
static int ws1[256], ws2[256], ws3[256], ws4[256];

/* Worker completion.  Not a channel: two workers finishing before main
 * listens would both output on it, which is undefined. */
static volatile int done_cnt, done_sum;

static void finish(int code) { done_sum += code; done_cnt++; PStop(); }

/* Sum of the codes of the next n workers to finish, -1 if they do not. */
static int wait_done_n(int n)
{
    int t = 0, s;

    while (done_cnt < n && t++ < 50000)
        ProcWait(20);
    s = done_cnt >= n ? done_sum : -1;
    done_cnt = 0;
    done_sum = 0;
    return s;
}

static void fill(u8 *p, int n, int seed)
{
    int i;
    for (i = 0; i < n; i++)
        p[i] = (u8)(seed + 7 * i);
}

static int same(const u8 *p, int n, int seed)
{
    int i;
    for (i = 0; i < n; i++)
        if (p[i] != (u8)(seed + 7 * i))
            return 0;
    return 1;
}

/* ---- 1. configuration space -------------------------------------------- */

static void test_config(void)
{
    u32 v;

    errflag();
    checkv("devid", ldconf(CR_DEVICEID), 0x0806);
    v = op1(t9_ldmemstartval, 0, 0, 0, 0, 0);
    checkv("memstart", v, ldconf(CR_MEMSTART));
    stconf(CR_HDRAREA, (u32)hdrarea);
    checkv("hdrarea", ldconf(CR_HDRAREA), (u32)hdrarea);
    check("conf-noerr", !errflag());
    ldconf(0x7777);
    check("ldconf-bad", errflag());
    stconf(CR_LINKSTATUS(2), 0);        /* read-only */
    check("stconf-ro", errflag());
    check("ds-started", (ldconf(CR_LINKSTATUS(2)) & 2) != 0);
}

/* ---- 2. VCP and VLCB set-up -------------------------------------------- */

static void vcp_setup(void)
{
    int n, i;
    u32 h;

    for (i = 0; i < (int)(sizeof rcblock / 4); i++)
        rcblock[i] = NOTPROC;           /* every rc.Id normal mode */
    stconf(CR_VCPCOMMAND, 1);           /* reset ... */
    stconf(CR_VCPCOMMAND, 0);           /* ... then waiting */
    stconf(CR_MININVALID, VIN(NVL));
    stconf(CR_EXTRCBASE, (u32)rcblock);
    stconf(CR_LINKMODE(1), 0);          /* virtual mode, 1-byte headers */
    stconf(CR_HDROFF(1), 0);
    stconf(CR_MINHDR(1), 0);
    stconf(CR_MAXHDR(1), NVL - 1);
    for (n = 0; n < 4; n++) {
        op1(t9_initvlcb, VIN(n), (u32)pktbuf[n], 0, 0, 0);
        op1(t9_sethdr, VOUT(n), 1, NULOFF, 0, 0);
        h = (u32)(n ^ 1);
        op1(t9_writehdr, VIN(n), 1, (u32)&h, 0, 0);
    }
    op1(t9_initvlcb, VIN(4), (u32)pktbuf[4], 0, 0, 0);
    stconf(CR_VCPCOMMAND, 2);           /* start */
    for (n = 0; n < 4; n++) {
        setchmode(VIN(n), 1);
        setchmode(VOUT(n), 1);
    }
}

static void test_vlcb(void)
{
    u32 a, b, c, w;
    u32 lh[2];

    errflag();
    a = op1(t9_insphdr, VIN(0), 0, 0, &b, &c);
    check("insphdr", a == 1 && b == 1 && c == NULOFF);
    w = 0;
    op1(t9_readhdr, VOUT(0), 1, (u32)&w, 0, 0);
    checkv("readhdr", w & 0xFF, 1);
    check("vlcb-noerr", !errflag());

    /* VL4: a long header lives in the header region */
    op1(t9_sethdr, VIN(4), 2, 0, 0, 0);
    lh[0] = 0x44434241; lh[1] = 0x45;   /* "ABCDE" */
    op1(t9_writehdr, VOUT(4), 5, (u32)lh, 0, 0);
    a = op1(t9_insphdr, VOUT(4), 0, 0, &b, &c);
    check("longhdr", a == 2 && b == 5 && c == 0 &&
          hdrarea[0] == 0x44434241 && (hdrarea[1] & 0xFF) == 0x45);
    lh[0] = lh[1] = 0;
    op1(t9_readhdr, VIN(4), 5, (u32)lh, 0, 0);
    check("readlong", lh[0] == 0x44434241 && (lh[1] & 0xFF) == 0x45);
    w = 0x5958;
    op1(t9_writehdr, VIN(4), 2, (u32)&w, 0, 0);
    a = op1(t9_insphdr, VIN(4), 0, 0, &b, &c);
    check("shorthdr", b == 2 && c == NULOFF);
    check("hdr-noerr", !errflag());
    op1(t9_readhdr, VIN(4), 3, (u32)lh, 0, 0);
    check("readhdr-len", errflag());
    op1(t9_writehdr, VIN(4), 0, (u32)lh, 0, 0);
    check("writehdr-0", errflag());
    op1(t9_sethdr, VIN(4), 7, NULOFF, 0, 0);
    check("sethdr-link", errflag());
    op1(t9_sethdr, VIN(4), NULHDR, NULOFF, 0, 0);
    op1(t9_insphdr, VIN(4), 0, 0, &b, &c);
    check("nullhdr", b == NULHDR && c == NULOFF && !errflag());
    setchmode(VIN(4), 1);
    check("activate-null", errflag());

    checkv("chantype-int", chantype((u32)&done), 1);
    checkv("chantype-vc", chantype(VOUT(2)), 0);
    checkv("chantype-ev", chantype(EVIN(0)), 0);
    checkv("chantype-bs", chantype(0x80000008u), 0);
    check("chantype-noerr", !errflag());
    chantype(VIN(NVL));                 /* above MinInvalidChannel */
    check("chantype-bad", errflag());
}

/* ---- 3. messages over the virtual links -------------------------------- */

static const int lens[] = { 0, 1, 31, 32, 33, 100, 1000 };
#define NLENS (int)(sizeof lens / sizeof lens[0])
static u8 txbuf[1024], txbuf2[1024], rxbuf[1100], rxbuf2[1100];

static void sender_lens(void)
{
    int i;
    for (i = 0; i < NLENS; i++) {
        fill(txbuf, lens[i], i + 1);
        vout(VOUT(0), lens[i], txbuf);
    }
    finish(1);
}

static void test_vlengths(void)
{
    int i, ok = 1, e;
    u32 n;

    PRun(PSetup(ws1, sizeof ws1, sender_lens) | PRI_LOW);
    for (i = 0; i < NLENS; i++) {
        n = vin(VIN(1), sizeof rxbuf, rxbuf, &e);
        if (n != (u32)lens[i] || e || !same(rxbuf, lens[i], i + 1)) {
            ok = 0;
            uart_puts("[len ");
            uart_hex(lens[i]);
            uart_puts(" got ");
            uart_hex(n);
            uart_puts("] ");
        }
    }
    check("vlens", ok && wait_done_n(1) == 1);
}

static void sender_long(void) { fill(txbuf, 40, 9); vout(VOUT(0), 40, txbuf); finish(2); }

static void test_lengtherror(void)
{
    int e, i, ok = 1;
    u32 n;

    for (i = 0; i < 40; i++)
        rxbuf[i] = 0xEE;
    PRun(PSetup(ws1, sizeof ws1, sender_long) | PRI_LOW);
    n = vin(VIN(1), 16, rxbuf, &e);
    for (i = 16; i < 40; i++)
        if (rxbuf[i] != 0xEE)
            ok = 0;
    check("lenerr", n == LENERR && e && ok && wait_done_n(1) == 2);
}

/* fixed-length in/out in both directions of one virtual link at once */
static void out_vl1(void) { fill(txbuf, 200, 3); ChanOut((Channel *)VOUT(1), txbuf, 200); finish(3); }
static void out_vl0(void) { fill(txbuf2, 200, 4); ChanOut((Channel *)VOUT(0), txbuf2, 200); finish(4); }

static void test_bidir(void)
{
    PRun(PSetup(ws1, sizeof ws1, out_vl1) | PRI_LOW);
    PRun(PSetup(ws2, sizeof ws2, out_vl0) | PRI_LOW);
    ProcWait(200);                      /* both first packets get buffered */
    ChanIn((Channel *)VIN(0), rxbuf, 200);
    ChanIn((Channel *)VIN(1), rxbuf2, 200);
    check("bidir", same(rxbuf, 200, 3) && same(rxbuf2, 200, 4) &&
          wait_done_n(2) == 7);
}

static void wordbyte(void)
{
    ChanOutInt((Channel *)VOUT(2), (int)0xCAFEF00D);
    ChanOutChar((Channel *)VOUT(2), 0x5A);
    finish(5);
}

static void test_wordbyte(void)
{
    int w = 0;
    u8 c = 0;

    PRun(PSetup(ws1, sizeof ws1, wordbyte) | PRI_LOW);
    ChanIn((Channel *)VIN(3), &w, 4);
    ChanIn((Channel *)VIN(3), &c, 1);
    check("outword", (u32)w == 0xCAFEF00D);
    check("outbyte", c == 0x5A && wait_done_n(1) == 5);
}

/* two virtual links multiplexed over one physical link */
static void mux_a(void) { fill(txbuf, 300, 11); vout(VOUT(0), 300, txbuf); finish(10); }
static void mux_b(void) { fill(txbuf2, 300, 12); vout(VOUT(2), 300, txbuf2); finish(20); }

static void test_mux(void)
{
    int e1, e2;
    u32 n1, n2;

    PRun(PSetup(ws1, sizeof ws1, mux_a) | PRI_LOW);
    PRun(PSetup(ws2, sizeof ws2, mux_b) | PRI_LOW);
    n2 = vin(VIN(3), 1024, rxbuf2, &e2);
    n1 = vin(VIN(1), 1024, rxbuf, &e1);
    check("mux", n1 == 300 && n2 == 300 && !e1 && !e2 &&
          same(rxbuf, 300, 11) && same(rxbuf2, 300, 12) &&
          wait_done_n(2) == 30);
}

/* the first packet waits in the VLCB's packet buffer */
static void short5(void) { fill(txbuf, 5, 21); ChanOut((Channel *)VOUT(0), txbuf, 5); finish(6); }

static void test_buffer(void)
{
    u32 a, b, c, old;

    PRun(PSetup(ws1, sizeof ws1, short5) | PRI_LOW);
    ProcWait(200);
    a = op1(t9_readbfr, VIN(1), 0, 0, &b, &c);
    check("readbfr", a == (u32)pktbuf[1] && b == 5 && c == 1 &&
          same((u8 *)pktbuf[1], 5, 21));
    ChanIn((Channel *)VIN(1), rxbuf, 5);
    a = op1(t9_readbfr, VIN(1), 0, 0, &b, &c);
    check("bfr-empty", same(rxbuf, 5, 21) && c == 0 && wait_done_n(1) == 6);
    old = op1(t9_swapbfr, VIN(1), (u32)altbuf, 0, 0, 0);
    a = op1(t9_readbfr, VIN(1), 0, 0, 0, 0);
    check("swapbfr", old == (u32)pktbuf[1] && a == (u32)altbuf);
    op1(t9_swapbfr, VIN(1), (u32)pktbuf[1], 0, 0, 0);
    op1(t9_readbfr, VOUT(1), 0, 0, 0, 0);
    check("readbfr-out", errflag());
}

/* ALT over a soft channel and a virtual input channel */
static Channel never = NOPROCESS;
static void late4(void) { ProcWait(300); ChanOutInt((Channel *)VOUT(0), 0x1234); finish(7); }

static void test_alt(void)
{
    Channel *set[2];
    int sel, v = 0;

    set[0] = &never;
    set[1] = (Channel *)VIN(1);
    check("skipalt", ChanSkipAlt(set, 2) == -1);
    PRun(PSetup(ws1, sizeof ws1, late4) | PRI_LOW);
    sel = ChanAlt(set, 2);
    ChanIn((Channel *)VIN(1), &v, 4);
    check("alt-vc", sel == 1 && v == 0x1234 && wait_done_n(1) == 7);
}

/* ---- 4. resource channels ---------------------------------------------- */

static u32 rds[3];
static u32 rc1[3], rc2[3], rc3[3];      /* channel word, rc.Ptr, rc.Id */

static void rds_init(void) { rds[0] = rds[1] = rds[2] = NOTPROC; }
static void rc_init(u32 *c) { c[0] = NOTPROC; c[1] = 0; c[2] = NOTPROC; }

static void client1(void) { ChanOutInt((Channel *)rc1, 0x1111); finish(11); }
static void client2(void) { ChanOutInt((Channel *)rc2, 0x2222); finish(12); }
static void client1_late(void) { ProcWait(300); ChanOutInt((Channel *)rc1, 0x3333); finish(13); }
static void client3_late(void) { ProcWait(300); ChanOutInt((Channel *)rc3, 0x4444); finish(14); }

static void test_resource(void)
{
    u32 id = 0, a, b;
    int v = 0;

    rds_init(); rc_init(rc1); rc_init(rc2); rc_init(rc3);
    errflag();

    /* idle resource channel, claim queued, then grant without waiting */
    mkrc((u32)rc1, rds, 0x77);
    check("mkrc-idle", rc1[0] == RESCHAN && rc1[1] == (u32)rds && rc1[2] == 0x77);
    PRun(PSetup(ws1, sizeof ws1, client1) | PRI_LOW);
    ProcWait(100);
    check("claim-queued", rds[1] == (u32)rc1 && rds[2] == (u32)rc1 && rc1[0] != RESCHAN);
    grant(rds, &id);
    ChanIn((Channel *)rc1, &v, 4);
    check("grant-queued", id == 0x77 && v == 0x1111 && rc1[2] == NOTPROC &&
          rds[1] == NOTPROC && wait_done_n(1) == 11);

    /* server waits in grant, the claim wakes it */
    mkrc((u32)rc1, rds, 0x78);
    PRun(PSetup(ws1, sizeof ws1, client1_late) | PRI_LOW);
    id = 0;
    grant(rds, &id);
    ChanIn((Channel *)rc1, &v, 4);
    check("grant-wait", id == 0x78 && v == 0x3333 && rds[0] == NOTPROC &&
          wait_done_n(1) == 13);

    /* queue surgery: erdsq / ldresptr / stresptr / irdsq reverse it */
    mkrc((u32)rc1, rds, 1);
    mkrc((u32)rc2, rds, 2);
    PRun(PSetup(ws1, sizeof ws1, client1) | PRI_LOW);
    ProcWait(100);
    PRun(PSetup(ws2, sizeof ws2, client2) | PRI_LOW);
    ProcWait(100);
    a = op1(t9_erdsq, (u32)rds, 0, 0, &b, 0);
    check("erdsq", a == (u32)rc1 && b == (u32)rc2 && rds[1] == NOTPROC);
    checkv("ldresptr", op1(t9_ldresptr, (u32)rc1, 0, 0, 0, 0), (u32)rc2);
    op1(t9_stresptr, (u32)rc2, (u32)rc1, 0, 0, 0);       /* rc2 -> rc1 */
    op1(t9_irdsq, (u32)rds, (u32)rc2, (u32)rc1, 0, 0);
    check("irdsq", rds[1] == (u32)rc2 && rds[2] == (u32)rc1);
    grant(rds, &id);
    ChanIn((Channel *)rc2, &v, 4);
    check("grant-2", id == 2 && v == 0x2222);
    grant(rds, &id);
    ChanIn((Channel *)rc1, &v, 4);
    check("grant-1", id == 1 && v == 0x1111 && wait_done_n(2) == 23);

    /* unmkrc returns an idle channel to normal mode */
    mkrc((u32)rc3, rds, 3);
    op1(t9_unmkrc, (u32)rc3, 0, 0, 0, 0);
    check("unmkrc", rc3[0] == NOTPROC && rc3[2] == NOTPROC);

    /* the resource as an ALT guard (enbg/disg) */
    mkrc((u32)rc3, rds, 4);
    PRun(PSetup(ws1, sizeof ws1, client3_late) | PRI_LOW);
    {
        u32 io[3];
        io[0] = (u32)rds; io[1] = (u32)&never; io[2] = 99;
        t9_alt_res(io);
        grant(rds, &id);
        ChanIn((Channel *)rc3, &v, 4);
        check("alt-grant", io[2] == 0 && id == 4 && v == 0x4444 &&
              rds[0] == NOTPROC && wait_done_n(1) == 14);
    }
    check("res-noerr", !errflag());
}

/* a virtual input channel in resource mode: the VCP makes the claim */
static void vres_sender(void) { ProcWait(200); fill(txbuf, 8, 31); vout(VOUT(2), 8, txbuf); finish(15); }

static void test_vresource(void)
{
    u32 id = 0, st, b, c;
    int e;

    rds_init();
    mkrc(VIN(3), rds, 0x33);
    st = ldchstatus(VIN(3), &b, &c);
    check("vres-status", (st & 0x80000009u) == 0x80000009u && b == (u32)rds && c == 0x33);
    PRun(PSetup(ws1, sizeof ws1, vres_sender) | PRI_LOW);
    grant(rds, &id);
    check("vres-grant", id == 0x33 && (ldchstatus(VIN(3), 0, 0) & 8) == 0);
    check("vres-data", vin(VIN(3), 64, rxbuf, &e) == 8 && !e && same(rxbuf, 8, 31) &&
          wait_done_n(1) == 15);
}

/* ---- 5. channel modes, reset and stop ---------------------------------- */

static void blocked_out(void) { u32 x = 0xAB; vout(VOUT(2), 4, &x); finish(16); }

static void test_modes(void)
{
    u32 st, b, c, w;

    errflag();
    st = ldchstatus(VOUT(2), &b, &c);
    check("status-vc", (st & 0x80000001u) == 0x80000001u && b == NOTPROC);
    setchmode(VOUT(2), 0);
    PRun(PSetup(ws1, sizeof ws1, blocked_out) | PRI_LOW);
    ProcWait(100);
    st = ldchstatus(VOUT(2), &b, 0);
    check("deact-wait", (st & 1) == 0 && b != NOTPROC);
    w = resetch(VOUT(2));
    check("resetch-vc", w == b);
    op1(t9_readbfr, VIN(3), 0, 0, &b, &c);
    check("deact-nopkt", c == 0);
    __runp(w);                          /* let it finish */
    check("resetch-run", wait_done_n(1) == 16);
    setchmode(VOUT(2), 1);
    setchmode(VOUT(2), 2);
    check("setchmode-bool", errflag());

    op1(t9_stopch, VOUT(2), 0, 0, 0, 0);    /* nothing in flight */
    op1(t9_stopch, VIN(3), 0, 0, 0, 0);
    check("stopch-in", (ldchstatus(VIN(3), &b, 0) & 2) != 0 && b == NOTPROC);
    resetch(VIN(3));
    check("resetch-in", (ldchstatus(VIN(3), 0, 0) & 2) == 0 && !errflag());

    op1(t9_in, 4, VIN(NVL), (u32)rxbuf, 0, 0);  /* illegal channel */
    check("in-illegal", errflag());
}

/* ---- 6. event channels (EventOut0 -> EventIn1, EventOut1 -> EventIn0) -- */

static void ev_out(void)
{
    ChanOut((Channel *)EVOUT(0), 0, 0);
    ChanOut((Channel *)EVOUT(0), 0, 0);
    finish(17);
}

static void ev_out_once(void) { ChanOut((Channel *)EVOUT(0), 0, 0); finish(23); }
static void ev_activator(void) { ProcWait(200); setchmode(EVIN(1), 1); finish(22); }

static void test_events(void)
{
    u32 st, b;
    int e;

    st = ldchstatus(EVIN(1), &b, 0);
    check("ev-status", st == 1 && b == NOTPROC);
    PRun(PSetup(ws1, sizeof ws1, ev_out) | PRI_LOW);
    ChanIn((Channel *)EVIN(1), 0, 0);
    check("ev-vin", vin(EVIN(1), 8, rxbuf, &e) == 0 && !e);
    check("ev-handshake", wait_done_n(1) == 17);
    setchmode(EVIN(1), 0);
    check("ev-deact", ldchstatus(EVIN(1), 0, 0) == 0 && *(u32 *)EVOUT(1) == DEACT);
    setchmode(EVIN(1), 1);
    check("ev-noerr", !errflag());

    /* an event arriving at a deactivated input is held until setchmode */
    setchmode(EVIN(1), 0);
    PRun(PSetup(ws1, sizeof ws1, ev_out_once) | PRI_LOW);
    PRun(PSetup(ws2, sizeof ws2, ev_activator) | PRI_LOW);
    ChanIn((Channel *)EVIN(1), 0, 0);
    check("ev-pending", wait_done_n(2) == 45 && !errflag());
}

/* ---- 7a. VCP error handling (HRM 11.9, 13.5) ---------------------------- */

/* Link2 (byte-stream) is cabled to Link3: with Link3 in virtual mode a raw
 * byte-stream message arrives there as one malformed packet. */
static void inject(const u8 *p, int n) { ChanOut(LINK2OUT, (void *)p, n); ProcWait(100); }

static void stuck_sender(void) { u32 x = 0x77; vout(VOUT(0), 4, &x); finish(21); }
static void ok_sender(void) { fill(txbuf, 10, 51); vout(VOUT(0), 10, txbuf); finish(24); }

static void test_errors(void)
{
    u8 pkt[8];
    u32 st, b, w;
    int i, e;

    stconf(CR_LINKMODE(3), 2);          /* virtual, LocalizeError */
    stconf(CR_MAXHDR(3), NVL - 1);
    pkt[0] = 3;
    for (i = 1; i < 6; i++)
        pkt[i] = (u8)i;
    pkt[6] = 1;                         /* EOP after 5 bytes: not allowed */
    inject(pkt, 7);
    st = ldconf(CR_VCPSTATUS);
    check("err-short", (st & 0x37) == (1u | (2u << 1) | (3u << 4)) && (st >> 16) == 3);
    check("token-rx", (ldconf(CR_LINKSTATUS(3)) & 0x20) != 0);
    stconf(CR_VCPSTATUS, 0);
    checkv("status-clear", ldconf(CR_VCPSTATUS), 0);
    pkt[0] = 200;                       /* above VCPLink3MaxHeader */
    pkt[1] = 2;
    inject(pkt, 2);
    st = ldconf(CR_VCPSTATUS);
    check("err-range", (st & 7) == 1 && (st >> 16) == 200);
    stconf(CR_VCPSTATUS, 0);

    /* errors localised: the VCP kept running */
    PRun(PSetup(ws1, sizeof ws1, ok_sender) | PRI_LOW);
    check("localized", vin(VIN(1), 64, rxbuf, &e) == 10 && same(rxbuf, 10, 51) &&
          wait_done_n(1) == 24);

    /* a reliable link: the VCP discards until it is reset */
    stconf(CR_LINKMODE(3), 0);
    inject(pkt, 2);
    check("err-reliable", (ldconf(CR_VCPSTATUS) & 1) != 0);
    PRun(PSetup(ws1, sizeof ws1, stuck_sender) | PRI_LOW);
    ProcWait(200);
    st = ldchstatus(VOUT(0), &b, 0);
    check("discarding", (st & 0x10) != 0 && b != NOTPROC);
    stconf(CR_VCPCOMMAND, 1);
    stconf(CR_VCPCOMMAND, 0);
    stconf(CR_VCPCOMMAND, 2);
    w = resetch(VOUT(0));
    check("reset-recover", w == b);
    __runp(w);
    check("stuck-run", wait_done_n(1) == 21 && ldconf(CR_VCPSTATUS) == 0);
    PRun(PSetup(ws1, sizeof ws1, ok_sender) | PRI_LOW);
    check("after-reset", vin(VIN(1), 64, rxbuf, &e) == 10 && same(rxbuf, 10, 51) &&
          wait_done_n(1) == 24);
    stconf(CR_LINKMODE(3), 1);          /* back to byte-stream */
    check("err-noerr", !errflag());
}

/* ---- 7. byte-stream links 2 <-> 3 --------------------------------------- */

static void bs_sender(void) { ChanOut(LINK2OUT, txbuf, 12); finish(31); }

static void test_bytestream(void)
{
    u32 st, b;

    /* An out on a link completes only once the far end has input it
     * (ISM 8.4.2), so the input is in another process. */
    fill(txbuf, 12, 41);
    PRun(PSetup(ws1, sizeof ws1, bs_sender) | PRI_LOW);
    ChanIn(LINK3IN, rxbuf, 12);
    check("bs-data", same(rxbuf, 12, 41));
    check("bs-out-done", wait_done_n(1) == 31);
    st = ldchstatus((u32)LINK2OUT, &b, 0);
    check("bs-status", st == 0 && b == NOTPROC && !errflag());
}

/* ---- 8. range checks, trap return, restart, shadow state --------------- */

static void test_ranges(void)
{
    errflag();
    op1(t9_cir, (u32)-5, 10, 3, 0, 0);
    check("cir-in", !errflag());
    op1(t9_cir, (u32)-5, 10, 11, 0, 0);
    check("cir-out", errflag());
    checkv("cir-a", op1(t9_cir, 1, 9, 4, 0, 0), 4);
    op1(t9_ciru, 2, 0xF0000000u, 0x80000000u, 0, 0);
    check("ciru-in", !errflag());
    op1(t9_ciru, 2, 10, (u32)-1, 0, 0);
    check("ciru-out", errflag());
}

static u32 thds[16];

static void test_tret(void)
{
    int i;
    u32 th;

    for (i = 0; i < 16; i++)
        thds[i] = 0;
    thds[2] = NOTPROC;                  /* th.Fptr: no trap-sharing queue */
    thds[0] = 0x80000000u;              /* th.Cntl, sb.ThInUse set */
    thds[9] = 0x1111; thds[10] = 0x2222; thds[11] = 0x3333;
    t9_tret(thds);
    th = op1(t9_ldth, 0, 0, 0, 0, 0);
    check("tret", thds[12] == 0x1111 && thds[13] == 0x2222 && thds[14] == 0x3333 &&
          th == (u32)thds && (thds[0] & 0x80000000u) == 0);
    op1(t9_selth, NOTPROC, 0, 0, 0, 0);     /* null trap-handler again */
    for (i = 0; i < 16; i++)
        thds[i] = 0;
    thds[6] = 0xA; thds[7] = 0xB; thds[8] = 0xC;
    t9_restart(thds);
    check("restart", thds[12] == 0xA && thds[13] == 0xB && thds[14] == 0xC);
}

static volatile int spin_stop;
static u32 shadow[32];

static void victim(void)
{
    while (!spin_stop)
        ;
    finish(18);
}

static void snooper(void)
{
    u32 io[3];

    ProcWait(500);                      /* wakes and interrupts the victim */
    io[0] = 0; io[1] = 7; io[2] = (u32)shadow;
    t9_stshadow(io);                    /* every group */
    io[0] = 4; io[1] = 5; io[2] = (u32)&shadow[8];
    t9_ldshadow(io);                    /* put the same state back */
    spin_stop = 1;
    finish(19);
}

static void test_shadow(void)
{
    PRun(PSetup(ws3, sizeof ws3, victim) | PRI_LOW);
    PRun(PSetup(ws4, sizeof ws4, snooper) | PRI_HIGH);
    check("shadow-run", wait_done_n(2) == 37);
    /* Groups 0-3 are 4+2+1+1 words, so WdescReg.sh is shadow[8].  The
     * victim is somewhere inside its own frame, below PSetup's Wptr. */
    check("shadow-wdesc", (shadow[8] & 1) == 1 &&
          (shadow[8] & ~1u) >= (u32)ws3 && (shadow[8] & ~1u) < (u32)(ws3 + 256));
    check("shadow-iptr", shadow[9] > 0x80000070u && shadow[9] < 0x80100000u);
}

void test_main(void)
{
    test_config();
    vcp_setup();
    test_vlcb();
    test_vlengths();
    test_lengtherror();
    test_bidir();
    test_wordbyte();
    test_mux();
    test_buffer();
    test_alt();
    test_resource();
    test_vresource();
    test_modes();
    test_events();
    test_bytestream();
    test_errors();
    test_ranges();
    test_tret();
    test_shadow();
    checkv("vcpstatus", ldconf(CR_VCPSTATUS), 0);
    uart_puts(pass ? "\nALL-PASS\n" : "\nFAILED\n");
}
