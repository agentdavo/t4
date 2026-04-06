/*
 * T9000 Programmable Memory Interface (PMI) — Phase 5 soft model.
 * Spec: t9000hrm.txt ch.10 (PMI configuration).
 *
 * Minimal bank window map for T4: describe which physical ranges are
 * DRAM/ROM/empty.  Full AC timing is not modelled.
 */
#ifndef T9000_PMI_H
#define T9000_PMI_H

#include <stdint.h>

#define T9K_PMI_MAX_BANKS 8

typedef enum {
	T9K_PMI_EMPTY = 0,
	T9K_PMI_DRAM  = 1,
	T9K_PMI_ROM   = 2,
	T9K_PMI_DEVICE = 3
} t9k_pmi_type_t;

typedef struct t9k_pmi_bank {
	uint32_t base;
	uint32_t size;
	t9k_pmi_type_t type;
	int      enabled;
} t9k_pmi_bank_t;

typedef struct t9k_pmi_state {
	int enabled;
	int configured; /* DoPMIConfigured soft flag */
	t9k_pmi_bank_t bank[T9K_PMI_MAX_BANKS];
	int n_banks;
	uint32_t errors;
} t9k_pmi_state_t;

extern t9k_pmi_state_t t9k_pmi;

void t9k_pmi_init(void);
void t9k_pmi_add_bank(uint32_t base, uint32_t size, t9k_pmi_type_t type);
/* 0 = ok, 1 = no bank / empty */
int t9k_pmi_check_phys(uint32_t pa, int is_write);

#endif
