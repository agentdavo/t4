/* Raw-encoded probes for the T9000 channel, VCP, resource, configuration
 * and shadow-state instructions implemented in the t4 emulator
 * (t9000_vcp.c).  Used by test_t9000_vcp.c and test_t9000_vcp_net.c.
 * Encodings: T9000 ISM appendix B (e.g. vin = 61 FC, stconf = 2B FF).
 *
 * Generic probe t9_X(io): A=io[0], B=io[1], C=io[2]; executes X; stores the
 * resulting A, B, C back into io[0..2].  Each runs in its own 4-word frame
 * so that pw.Temp (Wptr+0, used by outbyte/outword/disc/altend) is not the
 * return address.
 */
.section .text,"ax"

.global t9_insphdr
t9_insphdr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2B, 0xFC                /* insphdr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_readbfr
t9_readbfr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2B, 0xFD                /* readbfr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldconf
t9_ldconf:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2B, 0xFE                /* ldconf */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_stconf
t9_stconf:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2B, 0xFF                /* stconf */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldchstatus
t9_ldchstatus:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2C, 0xF3                /* ldchstatus */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_cir
t9_cir:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2C, 0xF7                /* cir */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_chantype
t9_chantype:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2C, 0xF9                /* chantype */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ciru
t9_ciru:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2C, 0xFC                /* ciru */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldmemstartval
t9_ldmemstartval:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x27, 0xFE                /* ldmemstartval */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldth
t9_ldth:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x2C, 0xF2                /* ldth */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_selth
t9_selth:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x60, 0xF9                /* selth */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldpri
t9_ldpri:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x21, 0xFE                /* ldpri */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_resetch
t9_resetch:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x21, 0xF2                /* resetch */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_stopch
t9_stopch:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xFE                /* stopch */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_vout
t9_vout:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xFD                /* vout */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_swapbfr
t9_swapbfr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF9                /* swapbfr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_sethdr
t9_sethdr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF8                /* sethdr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_setchmode
t9_setchmode:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF7                /* setchmode */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_initvlcb
t9_initvlcb:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF6                /* initvlcb */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_writehdr
t9_writehdr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF5                /* writehdr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_readhdr
t9_readhdr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF4                /* readhdr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_grant
t9_grant:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x61, 0xF1                /* grant */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_unmkrc
t9_unmkrc:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xFD                /* unmkrc */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_mkrc
t9_mkrc:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xFC                /* mkrc */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_irdsq
t9_irdsq:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xFB                /* irdsq */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_erdsq
t9_erdsq:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xFA                /* erdsq */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_stresptr
t9_stresptr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xF9                /* stresptr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldresptr
t9_ldresptr:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x62, 0xF8                /* ldresptr */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_stshadow
t9_stshadow:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x60, 0xFD                /* stshadow */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_ldshadow
t9_ldshadow:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0x60, 0xFC                /* ldshadow */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_in
t9_in:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xF7                      /* in */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_out
t9_out:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xFB                      /* out */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_outword
t9_outword:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xFF                      /* outword */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

.global t9_outbyte
t9_outbyte:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32              /* ldl 5; ldnl 2   C = io[2] */
    .byte 0x75, 0x31              /* ldl 5; ldnl 1   B = io[1] */
    .byte 0x75, 0x30              /* ldl 5; ldnl 0   A = io[0] */
    .byte 0xFE                      /* outbyte */
    .byte 0xD1, 0xD2              /* stl 1; stl 2 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2   io[2] = C' */
    .byte 0x72, 0x75, 0xE1        /* io[1] = B' */
    .byte 0x71, 0x75, 0xE0        /* io[0] = A' */
    .byte 0xB4                    /* ajw 4 */
    .byte 0x22, 0xF0              /* ret */

/* t9_testerr(io): io[0] = testerr (1 = error flag was clear); clears it */
.global t9_testerr
t9_testerr:
    .byte 0x22, 0xF9              /* testerr */
    .byte 0x71, 0xE0              /* ldl 1; stnl 0 */
    .byte 0x22, 0xF0              /* ret */

/* t9_vin_cnt(io): vin (A=io[0] max, B=io[1] chan, C=io[2] buffer), then in
 * the same frame ldcnt -> io[3] and testerr -> io[4] (ISM 8.4.3: ldcnt must
 * follow vin before Wptr changes). */
.global t9_vin_cnt
t9_vin_cnt:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x75, 0x32, 0x75, 0x31, 0x75, 0x30
    .byte 0x61, 0xFC              /* vin */
    .byte 0x2C, 0xF0              /* ldcnt */
    .byte 0x75, 0xE3              /* ldl 5; stnl 3 */
    .byte 0x22, 0xF9              /* testerr */
    .byte 0x75, 0xE4              /* ldl 5; stnl 4 */
    .byte 0xB4, 0x22, 0xF0        /* ajw 4; ret */

/* t9_alt_res(io): ALT over a resource (io[0] = RDS) and a channel
 * (io[1]); io[2] = 0 if the grant guard was selected, 1 for the channel. */
.global t9_alt_res
t9_alt_res:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x24, 0xF3              /* alt */
    .byte 0x75, 0x30, 0x41        /* rds; ldc 1 */
    .byte 0x61, 0xF2              /* enbg  (A guard, B rds) */
    .byte 0x75, 0x31, 0x41        /* chan; ldc 1 */
    .byte 0x24, 0xF8              /* enbc  (A guard, B chan) */
    .byte 0x24, 0xF4              /* altwt */
    .byte 0x75, 0x30, 0x41, 0x40  /* rds; ldc 1; ldc 0 */
    .byte 0x61, 0xF3              /* disg  (A offset, B guard, C rds) */
    .byte 0x75, 0x31, 0x41, 0x42  /* chan; ldc 1; ldc 2 */
    .byte 0x22, 0xFF              /* disc  (A offset, B guard, C chan) */
    .byte 0x24, 0xF5              /* altend */
    .byte 0x40, 0x01              /* +0: ldc 0; j +1 */
    .byte 0x41                    /* +2: ldc 1 */
    .byte 0x75, 0xE2              /* ldl 5; stnl 2 */
    .byte 0xB4, 0x22, 0xF0        /* ajw 4; ret */

/* t9_tret(io): io is a THDS.  Saves this frame's Wptr in th.sWptr (io[7])
 * and the continuation in th.sIptr (io[8]), switches Wptr to the THDS and
 * executes tret with Areg = 0.  The A, B, C it resumes with land in
 * io[12..14]. */
.global t9_tret
t9_tret:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x10, 0x75, 0xE7        /* ldlp 0; ldl 5; stnl 7 */
    .byte 0x48, 0x21, 0xFB        /* ldc 8; ldpi -> cont */
    .byte 0x75, 0xE8              /* ldl 5; stnl 8 */
    .byte 0x75, 0x23, 0xFC        /* ldl 5; gajw */
    .byte 0x40, 0x60, 0xFB        /* ldc 0; tret */
    .byte 0xD1, 0xD2, 0xD3        /* cont: stl 1; stl 2; stl 3 */
    .byte 0x71, 0x75, 0xEC        /* io[12] = A */
    .byte 0x72, 0x75, 0xED        /* io[13] = B */
    .byte 0x73, 0x75, 0xEE        /* io[14] = C */
    .byte 0xB4, 0x22, 0xF0        /* ajw 4; ret */

/* t9_restart(io): io is a PDS.  ps.sWptr (io[4]) = this frame, ps.sIptr
 * (io[5]) = the continuation; restart loads A, B, C from io[6..8], which
 * land in io[12..14]. */
.global t9_restart
t9_restart:
    .byte 0x60, 0xBC              /* ajw -4 */
    .byte 0x10, 0x75, 0xE4        /* ldlp 0; ldl 5; stnl 4 */
    .byte 0x45, 0x21, 0xFB        /* ldc 5; ldpi -> cont */
    .byte 0x75, 0xE5              /* ldl 5; stnl 5 */
    .byte 0x75, 0x62, 0xFE        /* ldl 5; restart */
    .byte 0xD1, 0xD2, 0xD3        /* cont: stl 1; stl 2; stl 3 */
    .byte 0x71, 0x75, 0xEC
    .byte 0x72, 0x75, 0xED
    .byte 0x73, 0x75, 0xEE
    .byte 0xB4, 0x22, 0xF0        /* ajw 4; ret */
