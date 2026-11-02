/*
 * test_t9000_pstack.c - T9000 protection: stack extension on demand and
 * device-access-only regions (ISM 9.3, 9.5, 13.2.2), one node.
 * Expected: S\nALL-PASS\nH
 *
 * The supervisor runs code from this image as P-processes.  Region 2 maps
 * #80000000-#807FFFFF onto itself (code and globals); region 3 is the
 * stack, at the top of quarter 3, and region 0 is a device-only block at
 * the top of quarter 0.  Their physical blocks lie above #80800000, so no
 * physical address is reached through two regions (ISM 9.5).
 *
 * Stack extension: the stack region starts at 256 bytes.  Each call or ajw
 * that would put Wptr below it traps with AccessViolation before changing
 * anything; the supervisor doubles the region in place and restarts the
 * instruction by copying ps.Eptr into ps.sIptr (ISM 13.2.2).  The ajw of a
 * function prologue carries prefixes, so a restart that lost them would
 * re-run it with the wrong operand and the recursion would not add up.
 *
 * Device-only: devsw/devlw reach the block, ldnl traps, and gajw may not
 * point Wptr at it (also restartable, so the state is as before gajw).
 */
#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void up(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void us(const char *s) { while (*s) up(*s++); }
static void uh(unsigned v)
{
    int i;
    for (i = 28; i >= 0; i -= 4) {
        int n = (v >> i) & 0xF;
        up(n < 10 ? '0' + n : 'a' + n - 10);
    }
}

/* PDS slots (ISM table 9.3) */
#define PS_EPTR   1
#define PS_SWPTR  4
#define PS_SIPTR  5
#define PS_SAREG  6

/* region descriptor bits (ISM 9.5) */
#define D_W    1u
#define D_X    2u
#define D_TOP  4u
#define D_DEV  8u
#define D_NULL 0x80000000u

#define STACK_PHYS  0x80800000u     /* aligned to its largest size */
#define STACK_MAX   14              /* log2: up to 16 KiB */
#define DEV_PHYS    0x80804000u     /* STACK_PHYS + 16 KiB */
#define DEV_LA      0x3FFFFF00u     /* top 256 bytes of quarter 0 */

#define T_ERROR 0                   /* trap reasons and error types, */
#define T_SCALL 8                   /* ISM tables 10.5 and 10.10 */
#define T_TIME  6
#define ET_ACCESS 4

#define DEPTH 12

unsigned pds[11], rods[4];
unsigned g_reason, g_etype;
int g_presult;
unsigned g_devval;
static int fails;

static void check(const char *what, unsigned got, unsigned want)
{
    if (got == want)
        return;
    us(what); up(' '); uh(got); up('/'); uh(want); up('\n');
    fails++;
}

/* A region of 2^n bytes at the top of quarter 3 whose physical block grows
 * downwards from STACK_PHYS + 2^STACK_MAX, so doubling it keeps every
 * logical address on the same physical word. */
static unsigned stack_region(int n)
{
    return (STACK_PHYS + (1u << STACK_MAX) - (1u << n)) | (1u << (n - 1)) | D_TOP | D_W;
}

/* ---- P-process code -------------------------------------------------- */

static int p_sum(int n)
{
    volatile int pad[20];   /* a frame large enough for prefixed ajw */
    int i;

    for (i = 0; i < 20; i++)
        pad[i] = n + i;
    if (n == 0)
        return pad[19];
    return pad[0] + p_sum(n - 1);
}

void p_stack(void)
{
    g_presult = p_sum(DEPTH);
    __asm__ volatile("syscall");
    for (;;)
        ;
}

void p_dev(void)
{
    __asm__ volatile("ldc 0x5A5A1234\n ldc 0x3FFFFF04\n devsw\n"
                     "ldc 0x3FFFFF04\n devlw\n ldc g_devval\n stnl 0\n");
    __asm__ volatile("syscall");
    for (;;)
        ;
}

void p_nodev(void)
{
    __asm__ volatile("ldc 0x3FFFFF04\n ldnl 0\n ldc g_devval\n stnl 0\n");
    __asm__ volatile("syscall");
    for (;;)
        ;
}

void p_gajw(void)
{
    __asm__ volatile("ldc 0x3FFFFF80\n gajw\n");
    __asm__ volatile("syscall");
    for (;;)
        ;
}

/* ---- supervisor ------------------------------------------------------ */

static void go(void)
{
    __asm__ volatile("ldc rods\n ldc pds\n goprot\n"
                     "ldc g_reason\n stnl 0\n ldc g_etype\n stnl 0\n");
}

static void start(void (*fn)(void), int stack_n)
{
    int i;

    for (i = 0; i < 11; i++)
        pds[i] = 0;
    pds[PS_SWPTR] = 0xFFFFFFF0u;        /* room for a call frame above */
    pds[PS_SIPTR] = (unsigned)fn;
    rods[0] = (DEV_PHYS | 0x80u) | D_TOP | D_W | D_DEV;
    rods[1] = D_NULL;
    rods[2] = 0x80000000u | (1u << 22) | D_W | D_X;     /* 8 MiB, bottom */
    rods[3] = stack_region(stack_n);
    go();
}

/* The first byte after the prefixes of the instruction at a. */
static unsigned char opcode_at(unsigned a)
{
    const unsigned char *p = (const unsigned char *)a;

    while ((*p & 0xF0) == 0x20 || (*p & 0xF0) == 0x60)
        p++;
    return *p;
}

static void test_stack(void)
{
    int n = 8, traps = 0, ajw = 0, call = 0, guard;
    unsigned want = 19;

    for (guard = DEPTH; guard > 0; guard--)
        want += guard;
    g_presult = -1;
    start(p_stack, n);
    for (guard = 0; guard < 64; guard++) {
        if (g_reason == T_TIME) {           /* timeslice: just go on */
            go();
            continue;
        }
        if (g_reason != T_ERROR || g_etype != ET_ACCESS || n >= STACK_MAX)
            break;
        /* the instruction has not run: Wptr is still in the region */
        if (pds[PS_SWPTR] < 0u - (1u << n)) {
            check("stack swptr", pds[PS_SWPTR], 0u - (1u << n));
            break;
        }
        switch (opcode_at(pds[PS_EPTR]) & 0xF0) {
        case 0xB0: ajw++; break;
        case 0x90: call++; break;
        default:
            check("stack eptr opcode", opcode_at(pds[PS_EPTR]), 0xB0);
            break;
        }
        traps++;
        rods[3] = stack_region(++n);
        pds[PS_SIPTR] = pds[PS_EPTR];        /* restart the instruction */
        go();
    }
    check("stack reason", g_reason, T_SCALL);
    check("stack result", (unsigned)g_presult, want);
    check("stack traps>=2", traps >= 2, 1);
    check("stack ajw traps", ajw > 0, 1);
    check("stack all ajw/call", ajw + call, traps);
}

static void test_device(void)
{
    volatile unsigned *dev = (volatile unsigned *)DEV_PHYS;

    dev[1] = 0;
    g_devval = 0;
    start(p_dev, 10);               /* prologues fit: no stack traps */
    check("dev reason", g_reason, T_SCALL);
    check("dev store", dev[1], 0x5A5A1234u);
    check("dev load", g_devval, 0x5A5A1234u);

    g_devval = 0;
    start(p_nodev, 10);
    check("nodev reason", g_reason, T_ERROR);
    check("nodev etype", g_etype, ET_ACCESS);
    check("nodev value", g_devval, 0);

    start(p_gajw, 10);
    check("gajw reason", g_reason, T_ERROR);
    check("gajw etype", g_etype, ET_ACCESS);
    check("gajw areg", pds[PS_SAREG], DEV_LA + 0x80);
    check("gajw wptr", pds[PS_SWPTR] >= 0u - (1u << 10), 1);
    /* Eptr is the first byte of gajw = opr #3C: pfix 3, opr C */
    check("gajw eptr", ((const unsigned char *)pds[PS_EPTR])[0], 0x23);
    check("gajw eptr+1", ((const unsigned char *)pds[PS_EPTR])[1], 0xFC);
}

int test_main(void)
{
    test_stack();
    test_device();
    us(fails ? "\nFAILED\n" : "\nALL-PASS\n");
    return fails;
}
