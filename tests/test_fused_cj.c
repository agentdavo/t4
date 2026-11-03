/*
 * test_fused_cj.c - the eqc;cj and gt;cj pairs that t4 fuses into one
 * pseudo-instruction (T4COMBINATIONS), checked against the ISM, one node.
 * Expected: S\nALL-PASS\nH
 *
 * eqc n leaves true or false in A; gt leaves (B > A) in A and pops B.  cj
 * then jumps if A is 0 and does not affect the integer stack, so the false
 * stays in A, and pops A only when it does not jump (ISM cj, eqc, gt).
 * occam code relies on that 0: with eqc;cj leaving the compared value in A
 * instead, the INMOS occam compiler (D7205 oc) rejected valid source.
 *
 * t4 builds a fused pair the first time its two instructions run in a row
 * and runs the fused form from then on: a probe's first call runs them as
 * two instructions and every later one fused.  Each case runs eight times.
 */
typedef unsigned int u32;

#define UART_DATA (*(volatile unsigned int *)0x10000000)
static void up(int c) { UART_DATA = (unsigned int)(unsigned char)c; }
static void us(const char *s) { while (*s) up(*s++); }
static void uh(u32 v)
{
    int i;
    for (i = 28; i >= 0; i -= 4) {
        int n = (v >> i) & 0xF;
        up(n < 10 ? '0' + n : 'a' + n - 10);
    }
}

extern void fz_eqc_cj(u32 *io), fz_gt_cj(u32 *io);

static int fails;

/* Run probe on (a, b, c) eight times; expect A' and, if checkb, B'. */
static void check(const char *name, void (*probe)(u32 *), u32 a, u32 b, u32 c,
                  u32 want_a, int checkb, u32 want_b)
{
    int i;
    for (i = 0; i < 8; i++) {
        u32 io[3];
        io[0] = a;
        io[1] = b;
        io[2] = c;
        probe(io);
        if (io[0] != want_a || (checkb && io[1] != want_b)) {
            us(name);
            us(" run ");
            up('0' + i);
            us(": A=");
            uh(io[0]);
            us(" B=");
            uh(io[1]);
            us(" want A=");
            uh(want_a);
            if (checkb) {
                us(" B=");
                uh(want_b);
            }
            up('\n');
            fails++;
            return;
        }
    }
}

int test_main(void)
{
    /* eqc 3: A = 3 is true, so cj pops: A' = B, B' = C. */
    check("eqc-equal", fz_eqc_cj, 3, 0x1111, 0x2222, 0x1111, 1, 0x2222);
    /* A = 7 is false: cj jumps and leaves the false in A, B as it was. */
    check("eqc-differ", fz_eqc_cj, 7, 0x1111, 0x2222, 0, 1, 0x1111);
    check("eqc-zero", fz_eqc_cj, 0, 0x1111, 0x2222, 0, 1, 0x1111);
    check("eqc-minus", fz_eqc_cj, 0xFFFFFFFDu, 0x1111, 0x2222, 0, 1, 0x1111);

    /* gt: B > A (signed) is true, so cj pops: A' = C (B' undefined). */
    check("gt-true", fz_gt_cj, 5, 9, 0x3333, 0x3333, 0, 0);
    check("gt-signed", fz_gt_cj, 0xFFFFFFFFu, 0, 0x3333, 0x3333, 0, 0);
    /* B > A is false: cj jumps, A' = false, B' = C from gt. */
    check("gt-false", fz_gt_cj, 9, 5, 0x3333, 0, 1, 0x3333);
    check("gt-equal", fz_gt_cj, 5, 5, 0x3333, 0, 1, 0x3333);

    us(fails ? "\nFAILED\n" : "\nALL-PASS\n");
    return fails;
}
