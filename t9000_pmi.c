/*
 * T9000 PMI soft model (Phase 5 + Phase 10 per-process private).
 */
#include "t9000_pmi.h"
#include "t9000_protect.h"
#include "processor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

t9k_pmi_state_t t9k_pmi;

/* Phase 10: active P-process private phys window (0 = unrestricted private). */
static uint32_t pmi_priv_lo;
static uint32_t pmi_priv_hi;
static uint32_t pmi_text_lo;
static uint32_t pmi_text_hi;

/* Shared / always-allowed under P (devices + graphics + kernel image). */
static int pmi_shared_ok(uint32_t pa)
{
	/* Kernel image / low high-RAM (below FB carveout) — C-call under P. */
	if (pa - 0x80000000u < 0x01a00000u)
		return 1;
	/* Scratch + compose + FB carveouts */
	if (pa - 0x81a00000u < 0x00600000u)
		return 1;
	/* UART */
	if (pa - 0x10000000u < 0x01000000u)
		return 1;
	/* SD */
	if (pa - 0x20000000u < 0x10000000u)
		return 1;
	/* VGA ctrl */
	if (pa - 0xA0000000u < 0x00100000u)
		return 1;
	/* Boot/XiP */
	if (pa - 0x7F000000u < 0x01000000u)
		return 1;
	return 0;
}

void t9k_pmi_init(void)
{
	const char *e;
	memset(&t9k_pmi, 0, sizeof t9k_pmi);
	pmi_priv_lo = pmi_priv_hi = 0;
	pmi_text_lo = pmi_text_hi = 0;
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
		fprintf(stderr, "[T9K-PMI] enabled, %d banks (private windows on)\n",
			t9k_pmi.n_banks);
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

void t9k_pmi_set_private(uint32_t lo, uint32_t hi)
{
	pmi_priv_lo = lo;
	pmi_priv_hi = hi;
}

void t9k_pmi_set_text_rom(uint32_t lo, uint32_t hi)
{
	pmi_text_lo = lo;
	pmi_text_hi = hi;
}

int t9k_pmi_check_phys(uint32_t pa, int is_write)
{
	int i;
	if (!t9k_pmi.enabled)
		return 0;

	/*
	 * Phase 10 under P:
	 *  - Text range is ROM (no write) — true RX for process text.
	 *  - Foreign private deny is opt-in (T4_PMI_PRIV=1): C-call under P
	 *    runs kernel heap in the same high RAM band as user images, so
	 *    a blanket deny breaks mkdir/VFS. Soft PRIV still isolates uaccess.
	 */
	if (t9k_in_p()) {
		/*
		 * Text-as-ROM under P: opt-in T4_PMI_TEXT_ROM=1.
		 * Some bFLT/busybox paths still touch the text image under P
		 * (reloc remnants / composite applets); soft RX still applies
		 * to uaccess.  Enable for strict RX experiments.
		 */
		if (getenv("T4_PMI_TEXT_ROM") && pmi_text_hi > pmi_text_lo &&
		    is_write && pa >= pmi_text_lo && pa < pmi_text_hi) {
			t9k_pmi.errors++;
			return 1;
		}
		if (pmi_priv_hi > pmi_priv_lo && getenv("T4_PMI_PRIV")) {
			int in_priv = (pa >= pmi_priv_lo && pa < pmi_priv_hi);
			if (!in_priv && !pmi_shared_ok(pa) &&
			    pa >= 0x82000000u && pa < 0xC0000000u) {
				t9k_pmi.errors++;
				return 1;
			}
		}
	}

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
