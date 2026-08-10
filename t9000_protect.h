/*
 * IMS T9000 L/P-process protection model (T4 emulator).
 * Spec: t9000ism.txt ch.9–10, t9000hrm.txt §4.
 */
#ifndef T9000_PROTECT_H
#define T9000_PROTECT_H

#include <stdint.h>
#include <setjmp.h>

/* PDS word offsets (ISM table 9.3) */
#define T9K_PS_CNTL     0
#define T9K_PS_EPTR     1
#define T9K_PS_EWL      2
#define T9K_PS_EWU      3
#define T9K_PS_SWPTR    4
#define T9K_PS_SIPTR    5
#define T9K_PS_SAREG    6
#define T9K_PS_SBREG    7
#define T9K_PS_SCREG    8
#define T9K_PS_SEREG    9
#define T9K_PS_SXREG    10
#define T9K_PDS_WORDS   11

/* Trap causes (subset; align with ISM t.* codes where practical) */
#define T9K_TRAP_NONE            0
#define T9K_TRAP_ACCESS          1  /* AccessViolation */
#define T9K_TRAP_PRIV            2  /* PrivInstruction */
#define T9K_TRAP_SYSCALL         3  /* syscall */
#define T9K_TRAP_ERROR           4
#define T9K_TRAP_TIMESLICE       5
#define T9K_TRAP_BREAKPOINT      6

/* Access kinds for translate */
#define T9K_ACC_READ     1
#define T9K_ACC_WRITE    2
#define T9K_ACC_EXEC     4

/* Region descriptor helpers */
#define T9K_NULL_REGION  0x80000000u
#define T9K_DESC_W       0x1u
#define T9K_DESC_X       0x2u
#define T9K_DESC_TOP     0x4u
#define T9K_DESC_DEV     0x8u

/*
 * Secondary OReg values (ISM App.B, negative-prefix encodings):
 *   memory 60FA → OReg = -6  (goprot)
 *   memory 60F8 → OReg = -8  (syscall)
 * Stored as unsigned in T4 OReg.
 */
#define T9K_OP_GOPROT    0xFFFFFFFAu  /* -6 */
#define T9K_OP_SYSCALL   0xFFFFFFF8u  /* -8 */

typedef struct t9k_protect_state {
	int      enabled;       /* master switch (T4_PROTECT) */
	int      is_p;          /* 1 = P-process mode */
	int      debug;
	uint32_t region[4];     /* RegionReg0..3 */
	uint32_t pstate_reg;    /* PDS physical pointer */
	uint32_t wdesc_stub;    /* supervisor Wdesc */
	uint32_t supervisor_iptr; /* IPtr after goprot (resume L) */
	uint32_t last_trap;
	uint32_t last_fault_la;
	uint32_t trap_count;
	/* Bypass translation for internal PDS/RODS access */
	int      phys_bypass;
	/* longjmp back to mainloop on P-trap */
	int      jmp_valid;
	jmp_buf  jmp_env;
} t9k_protect_state_t;

extern t9k_protect_state_t t9k_prot;

void t9k_protect_init(void);

/* 1 if currently in P-mode and protection active */
int t9k_in_p(void);

/*
 * Translate logical -> physical for current mode.
 * Returns physical address.  On fault, takes P-trap (longjmp) and does not return.
 * L-mode or disabled: identity.
 */
uint32_t t9k_translate(uint32_t la, int access);

/* Non-trapping probe (for tests): 0 ok, non-zero trap cause */
int t9k_probe(uint32_t la, int access, uint32_t *pa_out);

/* Build a region descriptor (size must be power of two, >= 256) */
uint32_t t9k_make_region(uint32_t phys_base, uint32_t size,
			 int write, int exec, int top, int device);

/* goprot: A=PDS (phys), B=RODS (phys). Privileged. */
void t9k_goprot(uint32_t pds_phys, uint32_t rods_phys);

/* Force L-mode without trap (spawn longjmp abandoned P; re-goprot needs L). */
void t9k_leave_p(void);

/* Phase 10: install region descriptors without changing IPtr/WPtr/is_p. */
void t9k_load_rods(const uint32_t rods[4]);

/* syscall in P -> trap to supervisor; in L -> soft note (THDS not fully modelled) */
void t9k_syscall(void);

/* Take P-process trap with cause; restores supervisor and longjmps */
void t9k_p_trap(uint32_t cause, uint32_t fault_la);

/* Privileged secondary OReg? (ISM §9.2 subset) */
int t9k_is_privileged_op(uint32_t oreg);

/* If P-mode and oreg privileged, trap (does not return) */
void t9k_check_priv_op(uint32_t oreg);

/* Physical-memory helpers used while filling PDS (bypass) */
uint32_t t9k_phys_word(uint32_t pa);
void t9k_phys_writeword(uint32_t pa, uint32_t v);

#endif /* T9000_PROTECT_H */
