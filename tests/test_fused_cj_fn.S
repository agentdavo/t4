/* Probes for the conditional-jump pairs t4 fuses (T4COMBINATIONS), used by
 * test_fused_cj.c.  Each takes io: A=io[0], B=io[1], C=io[2]; runs the pair
 * with cj 0, which continues at the next instruction whether or not it
 * jumps; and stores the resulting A and B in io[0..1].  C is undefined
 * after a pop (ISM cj), so it is not stored.
 */
.section .text,"ax"

.global fz_eqc_cj
fz_eqc_cj:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xC3, 0xA0              /* eqc 3; cj 0 */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global fz_gt_cj
fz_gt_cj:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xF9, 0xA0              /* gt; cj 0 */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */
