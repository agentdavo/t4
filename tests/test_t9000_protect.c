/*
 * Host unit test for T9000 region translate (Phase 1).
 * Build: make -C transputer/t4 test-protect
 */
#include "t9000_protect.h"
#include "processor.h"
#include "cpu_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

/* Stubs for symbols referenced from t9000_protect.c and t9000_pmi.c */
cpu_t t4_cpu0;
T4_THREAD_LOCAL cpu_t *t4cpu = &t4_cpu0;
int Txxx = 9000;
uint32_t MemSize, MemWordMask, MemByteMask, MemStart, ExtMemStart, CoreSize, CLineTagsSize;

static uint32_t fake_mem[4096];
static uint32_t fake_base = 0x10000000;

uint32_t word_int(uint32_t p)
{
	if (p >= fake_base && p < fake_base + sizeof(fake_mem))
		return fake_mem[(p - fake_base) / 4];
	return 0;
}

void writeword_int(uint32_t p, uint32_t v)
{
	if (p >= fake_base && p < fake_base + sizeof(fake_mem))
		fake_mem[(p - fake_base) / 4] = v;
}

void UpdateWdescReg(uint32_t w)
{
	WdescReg = w;
	WPtr = w & ~3u;
}

uint32_t t4_insn_start(void)
{
	return IPtr;
}

void t4_icache_context(void)
{
}

static int fails;

static void expect(int cond, const char *msg)
{
	if (!cond) {
		fprintf(stderr, "FAIL: %s\n", msg);
		fails++;
	} else {
		fprintf(stderr, "ok: %s\n", msg);
	}
}

int main(void)
{
	uint32_t d, pa;
	int c;

	setenv("T4_PROTECT", "1", 1);
	setenv("T4_PROTECT_DEBUG", "0", 1);
	t9k_protect_init();
	t9k_prot.enabled = 1;
	t9k_prot.is_p = 1;

	/* 4 KiB RW region at bottom of quarter 0, phys 0x20000000 */
	d = t9k_make_region(0x20000000u, 4096, 1, 0, 0, 0);
	t9k_prot.region[0] = d;
	t9k_prot.region[1] = T9K_NULL_REGION;
	t9k_prot.region[2] = T9K_NULL_REGION;
	t9k_prot.region[3] = T9K_NULL_REGION;

	c = t9k_probe(0x00000100u, T9K_ACC_READ, &pa);
	expect(c == 0, "read in region 0 ok");
	expect(pa == 0x20000100u, "phys = reloc | offset");

	c = t9k_probe(0x00000100u, T9K_ACC_WRITE, &pa);
	expect(c == 0, "write permitted");

	/* Execute denied (no X bit) */
	c = t9k_probe(0x00000100u, T9K_ACC_EXEC, &pa);
	expect(c == T9K_TRAP_ACCESS, "exec denied without X");

	/* Outside region size */
	c = t9k_probe(0x00001000u, T9K_ACC_READ, &pa);
	expect(c == T9K_TRAP_ACCESS, "past region size faults");

	/* Region 2 disabled */
	c = t9k_probe(0x80000000u, T9K_ACC_READ, &pa);
	expect(c == T9K_TRAP_ACCESS, "disabled quarter faults");

	/* Code region with X, no W */
	d = t9k_make_region(0x30000000u, 4096, 0, 1, 0, 0);
	t9k_prot.region[0] = d;
	c = t9k_probe(0x00000040u, T9K_ACC_EXEC, &pa);
	expect(c == 0 && pa == 0x30000040u, "exec ok on RX");
	c = t9k_probe(0x00000040u, T9K_ACC_WRITE, &pa);
	expect(c == T9K_TRAP_ACCESS, "write denied on RX");

	expect(t9k_is_privileged_op(0x07) == 1, "in is privileged");
	expect(t9k_is_privileged_op(0x00) == 0, "rev is not privileged");

	/* Block ranges (move): accessible exactly when both ends are */
	t9k_prot.region[0] = t9k_make_region(0x20000000u, 4096, 1, 0, 0, 0);
	c = t9k_probe_range(0x00000100u, 0x100, T9K_ACC_WRITE, &pa);
	expect(c == 0 && pa == 0x20000100u, "range inside region ok");
	c = t9k_probe_range(0x00000F00u, 0x200, T9K_ACC_READ, &pa);
	expect(c == T9K_TRAP_ACCESS, "range past region end faults");
	c = t9k_probe_range(0x00000F00u, 0x100, T9K_ACC_READ, &pa);
	expect(c == 0 && pa == 0x20000F00u, "range up to region end ok");
	t9k_prot.region[0] = t9k_make_region(0x20000000u, 4096, 1, 0, 0, 1);
	c = t9k_probe_range(0x00000100u, 4, T9K_ACC_READ, &pa);
	expect(c == T9K_TRAP_ACCESS, "device-only range faults for move");
	c = t9k_probe_range(0x00000100u, 4, T9K_ACC_READ | T9K_ACC_DEV, &pa);
	expect(c == 0, "device-only range ok for devmove");

	/* Quarters an Icache may keep across a mode change */
	t9k_prot.region[0] = t9k_make_region(0x20000000u, 4096, 0, 1, 0, 0);
	t9k_prot.region[2] = 0xA0000003u;	/* Linux: whole Q2 onto itself, W|X */
	t9k_prot.region[3] = 0xE0000001u;	/* whole Q3 onto itself, no X */
	expect(t9k_fetch_transparent() == 0x4u, "only an identity X quarter is transparent");
	t9k_prot.region[2] = 0xE0000003u;	/* Q2 relocated onto Q3 */
	expect(t9k_fetch_transparent() == 0x0u, "a relocated quarter is not");
	t9k_prot.region[2] = 0xA000000Bu;	/* device-only */
	expect(t9k_fetch_transparent() == 0x0u, "a device-only quarter is not");
	t9k_prot.is_p = 0;
	expect(t9k_fetch_transparent() == 0xFu, "L-mode fetches every quarter unchanged");

	/* leave_p clears P without trap (spawn longjmp path) */
	t9k_prot.is_p = 1;
	t9k_leave_p();
	expect(t9k_prot.is_p == 0, "leave_p clears is_p");

	if (fails) {
		fprintf(stderr, "\n%d failure(s)\n", fails);
		return 1;
	}
	fprintf(stderr, "\nALL PASS\n");
	return 0;
}
