"""Add T4_OPHIST=file to a private copy of t4's p.c: count every executed
instruction at its address, with no instruction fusion, and write
  addr key len count var dbl
lines at exit.  key: 0x00-0xF0 a direct function (Icode), 0x100+n the
secondary n, 0x300+n fpentry's selector n.  len: encoded bytes, prefixes
included.  var: the sum over executions of the operand a T800 timing
depends on (prod: highest set bit of A; shifts: the count; in/out/move:
words; norm: places; cj: 1 when taken; lend: 1 when it loops).  dbl: the
executions whose FA was double length (for the FPU timings)."""
import sys

p = sys.argv[1]
s = open(p).read()

def sub(old, new, count=1):
    global s
    assert s.count(old) == count, (old[:60], s.count(old))
    s = s.replace(old, new)

# 1. no fused pairs while counting
sub("""        if (t4_isa_strict)
                return;
        for (i = 0; NO_ICODE != combined[i].code0; i++)""",
"""        if (t4_isa_strict || (getenv("T4_OPHIST") && *getenv("T4_OPHIST")))
                return;
        for (i = 0; NO_ICODE != combined[i].code0; i++)""")

# 2. the counters and the dump, before run_insn_hooks
sub("""/* Next heartbeat_counter value at which run_insn_hooks() is due; 0 makes""",
"""/* T4_OPHIST=file: every instruction counted at its address */
#define OPH_BASE 0x80000000u
#define OPH_SIZE 0x00200000u
static int ophist_on;
static uint64_t *oph_cnt, *oph_var, *oph_dbl;
static uint16_t *oph_key;
static uint8_t *oph_len;

static void ophist_dump(void)
{
	const char *fn = getenv("T4_OPHIST");
	FILE *f;
	uint32_t i;

	if (!oph_cnt || !(f = fopen(fn, "w")))
		return;
	for (i = 0; i < OPH_SIZE; i++)
		if (oph_cnt[i])
			fprintf(f, "%08X %03X %u %llu %llu %llu\\n", OPH_BASE + i,
				oph_key[i], oph_len[i],
				(unsigned long long)oph_cnt[i],
				(unsigned long long)oph_var[i],
				(unsigned long long)oph_dbl[i]);
	fclose(f);
}

static void ophist_init(void)
{
	ophist_on = getenv("T4_OPHIST") && *getenv("T4_OPHIST");
	if (!ophist_on)
		return;
	oph_cnt = calloc(OPH_SIZE, sizeof *oph_cnt);
	oph_var = calloc(OPH_SIZE, sizeof *oph_var);
	oph_dbl = calloc(OPH_SIZE, sizeof *oph_dbl);
	oph_key = calloc(OPH_SIZE, sizeof *oph_key);
	oph_len = calloc(OPH_SIZE, 1);
	if (!oph_cnt || !oph_var || !oph_dbl || !oph_key || !oph_len) {
		ophist_on = 0;
		return;
	}
	atexit(ophist_dump);
}

static inline int oph_hibit(uint32_t x)
{
	return x ? 31 - __builtin_clz(x) : 0;
}

/* Next heartbeat_counter value at which run_insn_hooks() is due; 0 makes""")

# 3. turn the hooks on
sub("""	rare_hooks = !production_quiet || stop_iptr || dcache_enabled > 0 ||
	             t9k.enabled || t4_isa_strict ||""",
"""	ophist_init();
	rare_hooks = !production_quiet || stop_iptr || dcache_enabled > 0 ||
	             t9k.enabled || t4_isa_strict || ophist_on ||""")

# 4. count, at ExecuteInstr (registers still hold the operands)
sub("""	/* Feed the T9000 grouper model (T4_T9000=1); no cost when disabled. */""",
"""	if (__builtin_expect(rare_hooks && ophist_on, 0)) {
		uint32_t ip = Icache[islot].ip, a = ip - OPH_BASE;
		if (a < OPH_SIZE) {
			unsigned key = Icode == 0xF0
				? (OReg == 0xAB ? 0x300 + (AReg & 0xFF)
				   : 0x100 + (OReg & 0x1FF))
				: Icode;
			uint64_t v = 0;
			switch (key) {
			case 0xA0: v = AReg == 0; break;               /* cj taken */
			case 0x108: v = oph_hibit(AReg); break;        /* prod */
			case 0x107: case 0x10B: case 0x14A:            /* in out move */
				v = (AReg + 3) >> 2; break;
			case 0x140: case 0x141: case 0x135: case 0x136:/* shr shl lshr lshl */
				v = AReg; break;
			case 0x119: {                                  /* norm */
				uint64_t x = ((uint64_t)BReg << 32) | AReg;
				v = x ? __builtin_clzll(x) : 64;
				break;
			}
			case 0x121: v = word(index(BReg, 1)) > 1; break; /* lend loops */
			}
			oph_cnt[a]++;
			oph_var[a] += v;
			if (FAReg.length == FP_REAL64)
				oph_dbl[a]++;
			oph_key[a] = (uint16_t)key;
			oph_len[a] = (uint8_t)(IPtr - ip);
		}
	}
	/* Feed the T9000 grouper model (T4_T9000=1); no cost when disabled. */""")

open(p, "w").write(s)
print("patched", p)
