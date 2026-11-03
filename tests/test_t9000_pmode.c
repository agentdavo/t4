/*
 * test_t9000_pmode.c - T9000 protected mode: instruction fetch through the
 * region descriptors, block moves, and what ps.Eptr holds for each kind of
 * trap (ISM 9.3, 9.5, 10.2, 13.2.2), one node.
 * Expected: S\nALL-PASS\nH
 *
 * Region 2 maps #80000000-#807FFFFF onto itself (this image), region 3 is
 * a 1 KiB stack at the top of quarter 3 held at #80800000, region 0 is a
 * device-only block at the top of quarter 0 held at #80804000, and region 1
 * puts 256 bytes of hand-assembled code at #40000000, held at #80804100 or
 * #80804200.  No physical address is reached through two regions (ISM 9.5).
 *
 * Fetch: a region without execute permission traps when an instruction is
 * fetched from it, with ps.Eptr = ps.sIptr = that instruction - even when
 * the supervisor has just run it in L-mode, so the emulator's decoded copy
 * must not be used.  Code relocated into region 1 runs from its physical
 * block, and replacing that block, or the descriptor, changes what runs.
 *
 * Block moves: move, devmove and move2dall translate their addresses; move
 * may not touch a device-only region (devmove may), and a move that runs
 * out of its region traps before it changes anything.  T4 copies a 2D block
 * into its framebuffer directly, which must still respect the region's
 * write permission.
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

#define IMAGE      0x80000000u      /* region 2: 8 MiB onto itself */
#define R2_RWX     (IMAGE | (1u << 22) | D_W | D_X)
#define R2_RW      (IMAGE | (1u << 22) | D_W)
#define SRC_BUF    0x80100000u      /* in region 2, past the on-chip RAM */
#define Q2_RWX     0xA0000003u      /* region 2: the whole quarter onto itself */
#define Q2_RX      0xA0000002u
#define FB_LA      0x81E00000u      /* T4's framebuffer, in quarter 2 */
#define STACK_PHYS 0x80800000u      /* region 3: 1 KiB at the top */
#define STACK_LA   0xFFFFFC00u
#define DEV_PHYS   0x80804000u      /* region 0: 256 bytes at the top */
#define DEV_LA     0x3FFFFF00u
#define CODE_PHYS  0x80804100u      /* region 1: 256 bytes at the bottom */
#define CODE2_PHYS 0x80804200u
#define CODE_LA    0x40000000u
#define R1_CODE(p) ((p) | 0x80u | D_X)

#define T_ERROR 0                   /* trap reasons and error types, */
#define T_SCALL 8                   /* ISM tables 10.5 and 10.10 */
#define ET_PRIV   1
#define ET_ACCESS 4

unsigned pds[11], rods[4];
unsigned g_reason, g_etype;
unsigned g_src, g_dst, g_len;       /* the P-process's move operands */
char g_msg[16] = "protected move!";
int g_lcount;
static int fails;

static void check(const char *what, unsigned got, unsigned want)
{
    if (got == want)
        return;
    us(what); up(' '); uh(got); up('/'); uh(want); up('\n');
    fails++;
}

/* ---- P-process code ---------------------------------------------------- */

void p_move(void)
{
    __asm__ volatile("ldc g_src\n ldnl 0\n ldc g_dst\n ldnl 0\n"
                     "ldc g_len\n ldnl 0\n move\n syscall\n");
    for (;;)
        ;
}

void p_devmove(void)
{
    __asm__ volatile("ldc g_src\n ldnl 0\n ldc g_dst\n ldnl 0\n"
                     "ldc g_len\n ldnl 0\n .byte 0x62, 0xF4\n syscall\n");
    for (;;)
        ;
}

/* two rows of 4 bytes: source stride 8, destination stride 16 */
void p_move2d(void)
{
    __asm__ volatile("ldc 8\n ldc 16\n ldc 2\n move2dinit\n"
                     "ldc g_src\n ldnl 0\n ldc g_dst\n ldnl 0\n ldc 4\n"
                     "move2dall\n syscall\n");
    for (;;)
        ;
}

void p_ldnl_dev(void)
{
    __asm__ volatile("ldc 0x3FFFFF04\n ldnl 0\n syscall\n");
    for (;;)
        ;
}

void p_priv(void)
{
    __asm__ volatile("stopp\n syscall\n");
    for (;;)
        ;
}

void p_syscall(void)
{
    __asm__ volatile("syscall\n");
    for (;;)
        ;
}

/* L-mode only: in P-mode region 2 is not executable when this is reached */
void l_helper(void)
{
    g_lcount++;
}

/* ---- supervisor -------------------------------------------------------- */

static void go(void)
{
    __asm__ volatile("ldc rods\n ldc pds\n goprot\n"
                     "ldc g_reason\n stnl 0\n ldc g_etype\n stnl 0\n");
}

static void run(unsigned iptr, unsigned r1, unsigned r2)
{
    int i;

    for (i = 0; i < 11; i++)
        pds[i] = 0;
    pds[PS_SWPTR] = 0xFFFFFFF0u;        /* room for a call frame above */
    pds[PS_SIPTR] = iptr;
    rods[0] = (DEV_PHYS | 0x80u) | D_TOP | D_W | D_DEV;
    rods[1] = r1;
    rods[2] = r2;
    rods[3] = (STACK_PHYS | 0x200u) | D_TOP | D_W;
    g_reason = g_etype = 0xFFFFFFFFu;
    go();
}

/* An instruction fn (a primary function code) with operand v, as the
 * prefixing rules build it (ISM 4.2): the bytes go to p, the count back. */
static int put(unsigned char *p, int fn, int v)
{
    int n;

    if (v >= 0 && v < 16) {
        p[0] = (unsigned char)(fn | v);
        return 1;
    }
    if (v >= 16)
        n = put(p, 0x20, v >> 4);       /* pfix */
    else
        n = put(p, 0x60, (~v) >> 4);    /* nfix */
    p[n] = (unsigned char)(fn | (v & 15));
    return n + 1;
}

/* ldc a; syscall */
static void code_ldc_syscall(unsigned phys, int a)
{
    unsigned char *p = (unsigned char *)phys;
    int n = put(p, 0x40, a);

    p[n] = 0x60;                        /* nfix 0; opr 8 */
    p[n + 1] = 0xF8;
}

/* An address in this image, so it can be dereferenced */
static int in_image(unsigned a)
{
    return a - IMAGE < (1u << 22);
}

static unsigned eptr_byte(int k)
{
    unsigned e = pds[PS_EPTR];

    return in_image(e + k) ? ((const unsigned char *)e)[k] : 0x100u;
}

static void zero(unsigned phys, int n)
{
    volatile unsigned char *p = (volatile unsigned char *)phys;

    while (n-- > 0)
        *p++ = 0;
}

static void test_moves(void)
{
    volatile unsigned char *stk = (volatile unsigned char *)STACK_PHYS;
    volatile unsigned char *dev = (volatile unsigned char *)DEV_PHYS;
    volatile unsigned char *buf = (volatile unsigned char *)SRC_BUF;
    volatile unsigned char *fb = (volatile unsigned char *)FB_LA;
    int i, same;

    /* move into the relocated stack region.  The source is in external
     * RAM: T4 moves to or from the on-chip RAM a byte at a time. */
    for (i = 0; i < 16; i++)
        buf[i] = (unsigned char)g_msg[i];
    zero(STACK_PHYS, 0x100);
    g_src = SRC_BUF;
    g_dst = STACK_LA;
    g_len = 16;
    run((unsigned)p_move, D_NULL, R2_RWX);
    check("move reason", g_reason, T_SCALL);
    for (i = 0, same = 1; i < 16; i++)
        same &= stk[i] == (unsigned char)g_msg[i];
    check("move data", same, 1);

    /* move may not read a device-only region; ps.Eptr is the move */
    for (i = 0; i < 8; i++)
        dev[0x10 + i] = (unsigned char)(0xC0 + i);
    g_src = DEV_LA + 0x10;
    g_dst = STACK_LA + 0x40;
    g_len = 8;
    run((unsigned)p_move, D_NULL, R2_RWX);
    check("move dev reason", g_reason, T_ERROR);
    check("move dev etype", g_etype, ET_ACCESS);
    check("move dev eptr", eptr_byte(0), 0x24);         /* pfix 4; opr A */
    check("move dev eptr+1", eptr_byte(1), 0xFA);
    check("move dev data", stk[0x40], 0);

    /* devmove may */
    run((unsigned)p_devmove, D_NULL, R2_RWX);
    check("devmove reason", g_reason, T_SCALL);
    for (i = 0, same = 1; i < 8; i++)
        same &= stk[0x40 + i] == (unsigned char)(0xC0 + i);
    check("devmove data", same, 1);

    /* half in the region, half below it: nothing moves */
    for (i = 0; i < 8; i++)
        stk[i] = 0xA5;
    g_src = SRC_BUF;
    g_dst = STACK_LA - 8;
    g_len = 16;
    run((unsigned)p_move, D_NULL, R2_RWX);
    check("move below reason", g_reason, T_ERROR);
    check("move below etype", g_etype, ET_ACCESS);
    for (i = 0, same = 1; i < 8; i++)
        same &= stk[i] == 0xA5;
    check("move below untouched", same, 1);

    /* out of the top of quarter 3 into quarter 0, where region 0 is the
     * device block at the top: nothing moves */
    for (i = 0; i < 8; i++)
        stk[0x3F8 + i] = 0xA5;
    g_dst = 0xFFFFFFF8u;
    run((unsigned)p_move, D_NULL, R2_RWX);
    check("move out reason", g_reason, T_ERROR);
    check("move out etype", g_etype, ET_ACCESS);
    for (i = 0, same = 1; i < 8; i++)
        same &= stk[0x3F8 + i] == 0xA5;
    check("move out untouched", same, 1);

    /* move2dall: bytes 0-3 and 8-11 of g_msg to stack +#80 and +#90 */
    g_src = SRC_BUF;
    g_dst = STACK_LA + 0x80;
    run((unsigned)p_move2d, D_NULL, R2_RWX);
    check("move2d reason", g_reason, T_SCALL);
    for (i = 0, same = 1; i < 4; i++)
        same &= stk[0x80 + i] == (unsigned char)g_msg[i] &&
                stk[0x90 + i] == (unsigned char)g_msg[8 + i] &&
                stk[0x84 + i] == 0;
    check("move2d data", same, 1);

    /* within the framebuffer, which T4 copies directly when the blocks map
     * onto themselves - but only if the region lets the move write */
    for (i = 0; i < 0x20; i++)
        fb[i] = 0;
    for (i = 0; i < 16; i++)
        fb[0x100 + i] = (unsigned char)g_msg[i];
    g_src = FB_LA + 0x100;
    g_dst = FB_LA;
    run((unsigned)p_move2d, D_NULL, Q2_RX);
    check("move2d fb ro reason", g_reason, T_ERROR);
    check("move2d fb ro etype", g_etype, ET_ACCESS);
    for (i = 0, same = 1; i < 0x20; i++)
        same &= fb[i] == 0;
    check("move2d fb ro untouched", same, 1);
    run((unsigned)p_move2d, D_NULL, Q2_RWX);
    check("move2d fb reason", g_reason, T_SCALL);
    for (i = 0, same = 1; i < 4; i++)
        same &= fb[i] == (unsigned char)g_msg[i] &&
                fb[0x10 + i] == (unsigned char)g_msg[8 + i] &&
                fb[4 + i] == 0;
    check("move2d fb data", same, 1);
}

static void test_fetch(void)
{
    unsigned char *code = (unsigned char *)CODE_PHYS;
    int n;

    /* region 2 is not executable: the first fetch traps */
    run((unsigned)p_syscall, D_NULL, R2_RW);
    check("nox reason", g_reason, T_ERROR);
    check("nox etype", g_etype, ET_ACCESS);
    check("nox eptr", pds[PS_EPTR], (unsigned)p_syscall);
    check("nox iptr", pds[PS_SIPTR], (unsigned)p_syscall);

    /* code the supervisor has just run is no more executable for that:
     * region 1's ldc l_helper; gcall reaches it */
    l_helper();
    n = put(code, 0x40, (int)(unsigned)l_helper);
    code[n] = 0xF6;                                     /* gcall */
    run(CODE_LA, R1_CODE(CODE_PHYS), R2_RW);
    check("lcode reason", g_reason, T_ERROR);
    check("lcode etype", g_etype, ET_ACCESS);
    check("lcode eptr", pds[PS_EPTR], (unsigned)l_helper);
    check("lcode iptr", pds[PS_SIPTR], (unsigned)l_helper);
    check("lcode not run", g_lcount, 1);

    /* code relocated into region 1 */
    code_ldc_syscall(CODE_PHYS, 0x2A);
    run(CODE_LA, R1_CODE(CODE_PHYS), R2_RW);
    check("reloc reason", g_reason, T_SCALL);
    check("reloc areg", pds[PS_SAREG], 0x2A);
    check("reloc eptr", pds[PS_EPTR], CODE_LA + 2);     /* the syscall */
    check("reloc iptr", pds[PS_SIPTR], CODE_LA + 4);

    /* the supervisor replaces it */
    code_ldc_syscall(CODE_PHYS, 0x2B);
    run(CODE_LA, R1_CODE(CODE_PHYS), R2_RW);
    check("rewrite areg", pds[PS_SAREG], 0x2B);

    /* the same logical address held elsewhere */
    code_ldc_syscall(CODE2_PHYS, 0x2C);
    run(CODE_LA, R1_CODE(CODE2_PHYS), R2_RW);
    check("remap areg", pds[PS_SAREG], 0x2C);
}

static void test_eptr(void)
{
    /* an access the instruction may not make: ps.Eptr is the instruction */
    run((unsigned)p_ldnl_dev, D_NULL, R2_RWX);
    check("ldnl reason", g_reason, T_ERROR);
    check("ldnl etype", g_etype, ET_ACCESS);
    check("ldnl eptr", eptr_byte(0), 0x30);             /* ldnl 0 */

    /* a privileged instruction: the next one (ISM 13.2.2) */
    run((unsigned)p_priv, D_NULL, R2_RWX);
    check("priv reason", g_reason, T_ERROR);
    check("priv etype", g_etype, ET_PRIV);
    check("priv eptr-2", eptr_byte(-2), 0x21);          /* stopp: pfix 1; */
    check("priv eptr-1", eptr_byte(-1), 0xF5);          /* opr 5 */

    /* syscall: itself, with ps.sIptr the next one */
    run((unsigned)p_syscall, D_NULL, R2_RWX);
    check("scall reason", g_reason, T_SCALL);
    check("scall eptr", eptr_byte(0), 0x60);            /* nfix 0; opr 8 */
    check("scall eptr+1", eptr_byte(1), 0xF8);
    check("scall iptr", pds[PS_SIPTR], pds[PS_EPTR] + 2);
}

int test_main(void)
{
    test_moves();
    test_fetch();
    test_eptr();
    us(fails ? "\nFAILED\n" : "\nALL-PASS\n");
    return fails;
}
