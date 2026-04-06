/*
 * IMS T9000 L/P protection — T4 emulator core (Phase 1).
 */
#include "t9000_protect.h"
#include "t9000_pmi.h"
#include "processor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* CPU regs live in p.c */
extern uint32_t IPtr, WPtr, AReg, BReg, CReg, OReg, XReg, EReg;
extern uint32_t WdescReg;
extern uint32_t word_int(uint32_t);
extern void writeword_int(uint32_t, uint32_t);
extern void UpdateWdescReg(uint32_t);

t9k_protect_state_t t9k_prot;

void t9k_protect_init(void)
{
	const char *e;
	memset(&t9k_prot, 0, sizeof t9k_prot);
	e = getenv("T4_PROTECT");
	/* Default ON; explicit 0/false/off disables. */
	if (!e || !*e)
		t9k_prot.enabled = 1;
	else if (e[0] == '0' || e[0] == 'n' || e[0] == 'N' || e[0] == 'f' || e[0] == 'F')
		t9k_prot.enabled = 0;
	else
		t9k_prot.enabled = 1;
	e = getenv("T4_PROTECT_DEBUG");
	t9k_prot.debug = (e && *e && e[0] != '0');
	if (t9k_prot.debug)
		fprintf(stderr, "[T9K-PROT] enabled=%d\n", t9k_prot.enabled);
}

int t9k_in_p(void)
{
	return t9k_prot.enabled && t9k_prot.is_p;
}

uint32_t t9k_phys_word(uint32_t pa)
{
	int save = t9k_prot.phys_bypass;
	uint32_t v;
	t9k_prot.phys_bypass = 1;
	v = word_int(pa);
	t9k_prot.phys_bypass = save;
	return v;
}

void t9k_phys_writeword(uint32_t pa, uint32_t v)
{
	int save = t9k_prot.phys_bypass;
	t9k_prot.phys_bypass = 1;
	writeword_int(pa, v);
	t9k_prot.phys_bypass = save;
}

/* Decode size n from descriptor: bit (n-1) is the single 1 in bits 4..30 size field. */
static int desc_size_n(uint32_t d)
{
	int n;
	if (d == T9K_NULL_REGION || d == 0x80000000u)
		return -1;
	/* Find the size bit: first 1 in bits [4..30] that is the n-1 marker.
	 * Spec: bit n-1 set, bits 4..n-2 clear. Scan n from 8..31. */
	for (n = 8; n <= 31; n++) {
		uint32_t size_bit = 1u << (n - 1);
		/* bits 4..n-2 must be 0: mask from bit 4 to bit n-2 */
		uint32_t mid = 0;
		if (n > 5) {
			int b;
			for (b = 4; b <= n - 2; b++)
				mid |= 1u << b;
		}
		if ((d & size_bit) && !(d & mid))
			return n;
	}
	return -1;
}

uint32_t t9k_make_region(uint32_t phys_base, uint32_t size,
			 int write, int exec, int top, int device)
{
	int n = 8;
	uint32_t d;
	uint32_t sz = size;
	if (sz < 256)
		sz = 256;
	while ((1u << n) < sz && n < 30)
		n++;
	if ((phys_base & ((1u << n) - 1u)) != 0) {
		/* force align */
		phys_base &= ~((1u << n) - 1u);
	}
	d = (write ? T9K_DESC_W : 0) | (exec ? T9K_DESC_X : 0) |
	    (top ? T9K_DESC_TOP : 0) | (device ? T9K_DESC_DEV : 0);
	d |= 1u << (n - 1);
	d |= (phys_base & ~((1u << n) - 1u)); /* reloc bits n..31 */
	return d;
}

static int translate_one(uint32_t la, int access, uint32_t *pa_out)
{
	unsigned region = (la >> 30) & 3u;
	uint32_t d = t9k_prot.region[region];
	int n;
	uint32_t mask, base_la_quarter, quarter_base;
	uint32_t offset, phys;

	if (d == T9K_NULL_REGION || d == 0x80000000u)
		return T9K_TRAP_ACCESS;

	n = desc_size_n(d);
	if (n < 8)
		return T9K_TRAP_ACCESS;

	mask = (1u << n) - 1u;
	quarter_base = region << 30;
	/* Position: top or bottom of the 1GB quarter */
	if (d & T9K_DESC_TOP) {
		/* Top 2^n of quarter: bits n..29 of LA must be 1s */
		uint32_t need = ((1u << (30 - n)) - 1u) << n;
		if ((la & need) != need)
			return T9K_TRAP_ACCESS;
		base_la_quarter = quarter_base | need;
	} else {
		/* Bottom 2^n: bits n..29 must be 0 */
		uint32_t need = ((1u << (30 - n)) - 1u) << n;
		if (la & need)
			return T9K_TRAP_ACCESS;
		base_la_quarter = quarter_base;
	}

	/* Permissions */
	if ((access & T9K_ACC_WRITE) && !(d & T9K_DESC_W))
		return T9K_TRAP_ACCESS;
	if ((access & T9K_ACC_EXEC) && !(d & T9K_DESC_X))
		return T9K_TRAP_ACCESS;
	/* reads always ok if region valid (no separate R bit in descriptor) */

	offset = la & mask;
	/* Spec: phys = relocation(bits n..31) | base(bits 0..n-1 of LA). */
	phys = (d & ~((1u << n) - 1u)) | offset;

	/* PMI bank map (optional T4_PMI=1): reject empty/ROM-write. */
	if (t9k_pmi_check_phys(phys, (access & T9K_ACC_WRITE) ? 1 : 0))
		return T9K_TRAP_ACCESS;

	(void)base_la_quarter;
	*pa_out = phys;
	return 0;
}

int t9k_probe(uint32_t la, int access, uint32_t *pa_out)
{
	if (!t9k_prot.enabled || !t9k_prot.is_p || t9k_prot.phys_bypass) {
		if (pa_out)
			*pa_out = la;
		return 0;
	}
	return translate_one(la, access, pa_out);
}

uint32_t t9k_translate(uint32_t la, int access)
{
	uint32_t pa;
	int c;
	if (!t9k_prot.enabled || !t9k_prot.is_p || t9k_prot.phys_bypass)
		return la;
	c = translate_one(la, access, &pa);
	if (c) {
		if (t9k_prot.debug)
			fprintf(stderr,
				"[T9K-PROT] ACCESS fault la=0x%08X acc=%d cause=%d\n",
				la, access, c);
		t9k_p_trap((uint32_t)c, la);
		/* not reached */
		return 0;
	}
	return pa;
}

/* Privileged secondaries (ISM §9.2) — subset matching opcodes.h / T4 */
int t9k_is_privileged_op(uint32_t oreg)
{
	switch (oreg) {
	case 0x03: /* endp */
	case 0x07: /* in */
	case 0x0B: /* out */
	case 0x0E: /* outbyte */
	case 0x0F: /* outword */
	case 0x15: /* stopp */
	case 0x2B: /* tin */
	case 0x39: /* runp */
	case 0x0D: /* startp */
	case 0x43: /* alt */
	case 0x44: /* altwt */
	case 0x45: /* altend */
	case 0x2E: /* dist */
	case 0x2F: /* disc */
	case 0x30: /* diss */
	case 0x47: /* enbt */
	case 0x48: /* enbc */
	case 0x49: /* enbs */
	case 0x4E: /* talt */
	case 0x51: /* taltwt */
	case 0x54: /* sttimer */
	case T9K_OP_GOPROT: /* goprot itself is privileged */
		return 1;
	default:
		return 0;
	}
}

void t9k_check_priv_op(uint32_t oreg)
{
	if (!t9k_in_p())
		return;
	/*
	 * goprot is privileged in the ISM, but the Linux supervisor re-enters
	 * P via the same opcode after C-call gates (which do not leave P).
	 * Let t9k_goprot() soft-leave + re-enter rather than PRIV-trap here.
	 */
	if (oreg == (uint32_t)T9K_OP_GOPROT || oreg == (uint32_t)-6)
		return;
	if (t9k_is_privileged_op(oreg)) {
		if (t9k_prot.debug)
			fprintf(stderr, "[T9K-PROT] PRIV oreg=0x%X IPtr=0x%08X\n",
				oreg, IPtr);
		t9k_p_trap(T9K_TRAP_PRIV, IPtr);
	}
}

void t9k_p_trap(uint32_t cause, uint32_t fault_la)
{
	uint32_t pds = t9k_prot.pstate_reg;
	uint32_t stub = t9k_prot.wdesc_stub;

	if (!t9k_prot.is_p) {
		fprintf(stderr, "[T9K-PROT] trap while not in P (cause=%u)\n", cause);
		return;
	}

	t9k_prot.last_trap = cause;
	t9k_prot.last_fault_la = fault_la;
	t9k_prot.trap_count++;

	/* Save P state into PDS (physical) */
	if (pds) {
		uint32_t cntl = t9k_phys_word(pds + 4 * T9K_PS_CNTL);
		cntl = (cntl & 0xFFFF0000u) | (cause & 0xFFFFu);
		t9k_phys_writeword(pds + 4 * T9K_PS_CNTL, cntl);
		t9k_phys_writeword(pds + 4 * T9K_PS_EPTR, fault_la ? fault_la : IPtr);
		t9k_phys_writeword(pds + 4 * T9K_PS_SWPTR, WPtr);
		t9k_phys_writeword(pds + 4 * T9K_PS_SIPTR, IPtr);
		t9k_phys_writeword(pds + 4 * T9K_PS_SAREG, AReg);
		t9k_phys_writeword(pds + 4 * T9K_PS_SBREG, BReg);
		t9k_phys_writeword(pds + 4 * T9K_PS_SCREG, CReg);
		t9k_phys_writeword(pds + 4 * T9K_PS_SEREG, EReg);
		t9k_phys_writeword(pds + 4 * T9K_PS_SXREG, XReg);
	}

	/* Restore supervisor L-process */
	t9k_prot.is_p = 0;
	if (stub && stub != NotProcess_p) {
		UpdateWdescReg(stub);
		/* IPtr after goprot was stashed */
		IPtr = t9k_prot.supervisor_iptr;
	} else {
		IPtr = t9k_prot.supervisor_iptr;
	}

	if (t9k_prot.debug)
		fprintf(stderr,
			"[T9K-PROT] P-trap cause=%u la=0x%08X -> L IPtr=0x%08X WPtr=0x%08X\n",
			cause, fault_la, IPtr, WPtr);

	if (t9k_prot.jmp_valid)
		longjmp(t9k_prot.jmp_env, 1);
}

void t9k_leave_p(void)
{
	/*
	 * Soft leave: Linux spawn may longjmp out of a P-session without a
	 * trap (C-call exit).  Clear P so the next goprot is legal.
	 * Does not restore IPtr/WPtr — guest longjmp already did that.
	 */
	if (t9k_prot.is_p) {
		t9k_prot.is_p = 0;
		if (t9k_prot.debug)
			fprintf(stderr, "[T9K-PROT] leave_p (soft) traps=%u\n",
				t9k_prot.trap_count);
	}
}

void t9k_goprot(uint32_t pds_phys, uint32_t rods_phys)
{
	int i;

	if (!t9k_prot.enabled) {
		/* No-op passthrough: leave machine as-is */
		IPtr++;
		return;
	}

	if (t9k_in_p()) {
		/*
		 * Re-enter from supervisor after abandoned P (spawn longjmp)
		 * or nested goprot attempt: soft-leave then continue as L.
		 * True P-code must not execute goprot (privileged) — that is
		 * still caught if we were not intentionally re-entering.
		 */
		t9k_leave_p();
	}

	/* Save supervisor resume point = instruction after goprot.
	 * IPtr still points at OPR byte; after goprot we want next insn. */
	t9k_prot.supervisor_iptr = IPtr + 1;
	t9k_prot.wdesc_stub = WdescReg;
	t9k_prot.pstate_reg = pds_phys;

	/* Load RODS */
	for (i = 0; i < 4; i++)
		t9k_prot.region[i] = t9k_phys_word(rods_phys + (uint32_t)(4 * i));

	/* Load P state from PDS */
	WPtr = t9k_phys_word(pds_phys + 4 * T9K_PS_SWPTR);
	IPtr = t9k_phys_word(pds_phys + 4 * T9K_PS_SIPTR);
	AReg = t9k_phys_word(pds_phys + 4 * T9K_PS_SAREG);
	BReg = t9k_phys_word(pds_phys + 4 * T9K_PS_SBREG);
	CReg = t9k_phys_word(pds_phys + 4 * T9K_PS_SCREG);
	EReg = t9k_phys_word(pds_phys + 4 * T9K_PS_SEREG);
	XReg = t9k_phys_word(pds_phys + 4 * T9K_PS_SXREG);

	t9k_prot.is_p = 1;

	if (t9k_prot.debug)
		fprintf(stderr,
			"[T9K-PROT] goprot PDS=0x%08X RODS=0x%08X -> P IPtr=0x%08X WPtr=0x%08X "
			"R0=%08X R1=%08X R2=%08X R3=%08X\n",
			pds_phys, rods_phys, IPtr, WPtr,
			t9k_prot.region[0], t9k_prot.region[1],
			t9k_prot.region[2], t9k_prot.region[3]);

	/* Do not IPtr++ — IPtr already loaded from PDS as next P instruction. */
}

void t9k_syscall(void)
{
	if (t9k_in_p()) {
		if (t9k_prot.debug)
			fprintf(stderr, "[T9K-PROT] syscall from P IPtr=0x%08X A=0x%08X\n",
				IPtr, AReg);
		/* ISM: Eptr = trap insn; siptr = next. Advance past syscall OPR. */
		IPtr++; /* next after syscall for siptr save */
		t9k_p_trap(T9K_TRAP_SYSCALL, IPtr - 1);
		return;
	}
	/* L-process syscall: would go to THDS; soft stub for Phase 1–2 */
	if (t9k_prot.debug)
		fprintf(stderr, "[T9K-PROT] syscall from L (THDS not modelled) IPtr=0x%08X\n",
			IPtr);
	IPtr++;
}
