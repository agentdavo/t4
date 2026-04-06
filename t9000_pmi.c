/*
 * T9000 PMI soft model (Phase 5).
 */
#include "t9000_pmi.h"
#include "processor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

t9k_pmi_state_t t9k_pmi;

void t9k_pmi_init(void)
{
	const char *e;
	memset(&t9k_pmi, 0, sizeof t9k_pmi);
	e = getenv("T4_PMI");
	/*
	 * Default ON for L/P OS (region + bank checks).  Disable with T4_PMI=0.
	 * Banks cover TransputerOS high RAM, XiP, UART, SD, VGA.
	 */
	if (!e || !*e)
		t9k_pmi.enabled = 1;
	else if (e[0] == '0' || e[0] == 'n' || e[0] == 'N' || e[0] == 'f' || e[0] == 'F')
		t9k_pmi.enabled = 0;
	else
		t9k_pmi.enabled = 1;

	t9k_pmi_add_bank(0x80000000u, 0x40000000u, T9K_PMI_DRAM);   /* Q2: 1GiB high RAM/FB */
	t9k_pmi_add_bank(0x7F000000u, 0x01000000u, T9K_PMI_ROM);    /* boot/SD XiP window */
	t9k_pmi_add_bank(0x10000000u, 0x01000000u, T9K_PMI_DEVICE); /* UART block */
	t9k_pmi_add_bank(0x20000000u, 0x10000000u, T9K_PMI_DEVICE); /* SD */
	t9k_pmi_add_bank(0xA0000000u, 0x00100000u, T9K_PMI_DEVICE); /* VGA ctrl */
	t9k_pmi.configured = 1;

	if (t9k_pmi.enabled)
		fprintf(stderr, "[T9K-PMI] enabled, %d banks\n", t9k_pmi.n_banks);
}

void t9k_pmi_add_bank(uint32_t base, uint32_t size, t9k_pmi_type_t type)
{
	if (t9k_pmi.n_banks >= T9K_PMI_MAX_BANKS)
		return;
	t9k_pmi.bank[t9k_pmi.n_banks].base = base;
	t9k_pmi.bank[t9k_pmi.n_banks].size = size;
	t9k_pmi.bank[t9k_pmi.n_banks].type = type;
	t9k_pmi.bank[t9k_pmi.n_banks].enabled = 1;
	t9k_pmi.n_banks++;
}

int t9k_pmi_check_phys(uint32_t pa, int is_write)
{
	int i;
	if (!t9k_pmi.enabled)
		return 0;
	for (i = 0; i < t9k_pmi.n_banks; i++) {
		t9k_pmi_bank_t *b = &t9k_pmi.bank[i];
		if (!b->enabled)
			continue;
		if (pa - b->base < b->size) {
			if (is_write && b->type == T9K_PMI_ROM) {
				t9k_pmi.errors++;
				return 1;
			}
			if (b->type == T9K_PMI_EMPTY) {
				t9k_pmi.errors++;
				return 1;
			}
			return 0;
		}
	}
	/* No bank: allow for now (compat) unless strict */
	if (getenv("T4_PMI_STRICT")) {
		t9k_pmi.errors++;
		return 1;
	}
	return 0;
}
