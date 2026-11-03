/* Feature test macros before any system headers (sched_setaffinity). */
#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

/*
 * Copyright (c) 1993-1996 Julian Highfield. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Julian Highfield.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define INLINE

/* Branch prediction hints for hot paths */
#ifndef likely
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#endif

/* For the few helpers on every instruction's path.  mainloop() is far past
 * GCC's inlining growth limits, so plain `inline` no longer gets them in. */
#ifdef _MSC_VER
#define HOT_INLINE      __forceinline
#else
#define HOT_INLINE      inline __attribute__((always_inline))
#endif
#include "t4debug.h"

/*
 * p.c - hand inlined processor.c!
 *
 * The transputer emulator.
 *
 */
#ifdef _MSC_VER
#include "gettimeofday.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#ifndef _MSC_VER
#include <sys/time.h>
#include <unistd.h>
#if defined(__linux__)
#include <sched.h>
#endif
#ifndef _MSC_VER
#include <pthread.h>
#endif
#endif
#include "timer_integration.h"
#include "debug.h"
#include "decode_cache.h"

/* Direct-mapped instruction decode cache (1MB, 64K entries).
 * Caches pre-resolved prefix chains for fast dispatch.
 * OFF by default: must fill Icache on hit path or boot hangs. */
static dcache_t g_dcache;
static int dcache_enabled = 0; /* set via T4_DCACHE=1 */

#ifndef EMUDEBUG
static int fast_primary_enabled = -1;
static int fast_block_enabled = -1;

static INLINE int t4_fast_block_op(uint32_t op)
{
        switch (op) {
        case 0x10: /* ldlp */
        case 0x30: /* ldnl */
        case 0x40: /* ldc */
        case 0x50: /* ldnlp */
        case 0x70: /* ldl */
        case 0x80: /* adc */
        case 0xb0: /* ajw */
        case 0xc0: /* eqc */
        case 0xd0: /* stl */
        case 0xe0: /* stnl */
                return 1;
        default:
                return 0;
        }
}

#endif

/* Global timer bridge — connects timer thread to p.c globals */
static timer_bridge_t g_timer_bridge;
static int g_timer_threaded = 0;  /* 1 = timer thread running */

/* Forward declaration — defined later in p.c */
static void schedule_timerq(int priority);
static INLINE u_char code_byte(uint32_t ptr);
#include <math.h>
#include <inttypes.h>
#ifdef T4NANOMSG
#define NN_STATIC_LIB   ON
#include <nanomsg/nn.h>
#include <nanomsg/pipeline.h>
#else
#include <errno.h>
#define AF_SP 		0
#define NN_PUSH         0
#define NN_PULL         0
#define NN_POLLIN       1
#define NN_POLLOUT      2
#define NN_SOL_SOCKET   0 
#define NN_SNDTIMEO     0
#define NN_DONTWAIT     0
int nn_errno() { return EINVAL; }
char *nn_strerror(int errnum) { return strerror (errnum); }
int nn_socket(int domain, int protocol) { return EINVAL; }
int nn_close(int s) { return EBADF; }
int nn_bind(int s, const char *addr) { return EINVAL; }
int nn_connect(int s, const char *addr) { return EINVAL; }
int nn_shutdown(int s, int how) { return EINVAL; }
int nn_send(int s, const void *buf, size_t len, int flags) { return EINVAL; }
int nn_recv(int s, void *buf, size_t len, int flags) { return EINVAL; }
int nn_setsockopt(int s, int lvl, int opt, const void *optval, size_t optvallen) { return EINVAL; }
struct nn_pollfd {
        int fd;
        short events, revents;
};
int nn_poll(struct nn_pollfd *fds, int nfds, int opt) { return EINVAL; }
#endif
#include "netcfg.h"
#ifdef T4SHLINKS
#include "shlink.h"
#else
void* shlink_attach (const char *fnm, int size){return NULL;}
int shlink_detach (void *addr){return EINVAL;}
void* shlink_alloc (const char *fnm, int size){return NULL;}
int shlink_free (void){return EINVAL;}
#endif

#include "processor.h"
#include "cpu_state.h"
#include "t4_fabric.h"
#include "display_backend.h"
#include "arithmetic.h"
#include "server.h"
#include "opcodes.h"
#include "tvx.h"
#include "tvx_vec.h"
#include "tvx_q8k.h"
#include "tvx_gfx.h"
#include "t9000_group.h"
#include "t9000_protect.h"
#include "t9000_pmi.h"
#include "t9000_vcp.h"
#include "t9000_ctl.h"
#ifdef __MWERKS__
#include "mac_input.h"
#endif
#undef TRUE
#undef FALSE
#define TRUE  0x0001
#define FALSE 0x0000


/* Processor specific parameters. */
int Txxx = 414;
int t4_isa_strict = 0;
uint32_t MemStart    = 0x80000048;

/* Memory space. */
uint32_t CoreSize    = 2 * 1024;
uint32_t ExtMemStart = 0x80000800;

uint32_t MemSize     = 1 << 21;
/* Upper bound of the plain-RAM fast path.  Distinct from the framebuffer
 * base: once the display windows move below MemStart, RAM runs all the way
 * to the top of the address space and the fast path must not be derived
 * from fb_legacy_base (that would wrap and swallow the UART at 0x10000000). */
T4_THREAD_LOCAL t9k_state t9k;  /* per node thread in a threaded network */
t9k_state t9k_proto;

/* For the C104 router threads (t4_fabric.c), which time packets with the
 * configured model rather than a node's own. */
int t9k_net_timed (void)
{
        return t9k_proto.enabled && t9k_proto.net_on;
}

uint64_t t9k_route_packet (uint64_t ready, uint32_t nbytes, t9k_tl_t *out)
{
        return t9k_c104_hop (ready, nbytes, out);
}

/* For the VCP (t9000_vcp.c), which times its own packets: with defer on, a
 * packet it sends leaves no earlier than not_before, and one it receives
 * does not stall the CPU but leaves its arrival cycle for t9k_net_arrival.
 * A process the VCP wakes resumes no earlier than t9k_net_wake's cycle. */
void t9k_net_defer (int on, uint64_t not_before)
{
        t9k.net_defer = on;
        t9k.net_not_before = not_before;
}

uint64_t t9k_net_arrival (void) { return t9k.net_arrival; }

/* When the first hl bytes - the header - of that packet were in. */
uint64_t t9k_net_head (unsigned hl)
{
        return t9k.net_start + t9k_link_xfer (hl) + (uint64_t)t9k.link_lat;
}

/* When the VCP's packet just sent has left. */
uint64_t t9k_net_sent_end (void) { return t9k.net_sent_end; }
uint64_t t9k_net_now (void)     { return t9k_cycles_so_far (); }

/* Processes the VCP holds while the CPU runs another (t9k_net_wake, near
 * t4_wake): polled between instructions, where schedule() may switch. */
static void t9k_net_release (int idle);
#define T9K_NET_POLL() \
        do { if (__builtin_expect (t9k.net_npend != 0, 0) && \
                 t9k_cycles_so_far () >= t9k.net_pend_min) \
                     t9k_net_release (0); } while (0)

uint32_t ram_fast_limit = FB_LEGACY_DEFAULT;
uint32_t fb_base        = FB_BASE_DEFAULT;
uint32_t fb_legacy_base = FB_LEGACY_DEFAULT;
uint32_t vga_ctrl_base  = VGA_CTRL_DEFAULT;

/* Move the VGA framebuffer and control window out of the RAM range when the
 * requested memory would swallow them.  RAM starts at MemStart (0x80000000)
 * and grows up; the legacy FB at 0x90000000 therefore limits RAM to 256 MB.
 * Above that we relocate both windows into the unused low half of the address
 * space, which lets RAM run all the way to 2 GB. */
void t4_relocate_mmio_for_ram (uint32_t mem_size)
{
        uint64_t ram_end = (uint64_t)0x80000000u + (uint64_t)mem_size;

        if (ram_end <= (uint64_t)FB_LEGACY_DEFAULT) {
                fb_base         = FB_BASE_DEFAULT;
                fb_legacy_base  = FB_LEGACY_DEFAULT;
                vga_ctrl_base   = VGA_CTRL_DEFAULT;
                ram_fast_limit  = FB_LEGACY_DEFAULT;
                return;
        }
        /* Move every display window into the unused low half of the address
         * space so RAM can run contiguously from MemStart to 2 GB with no
         * holes.  Inference runs are headless; the kernel and fbwm never use
         * these sizes. */
        fb_base        = 0x60000000u;
        fb_legacy_base = 0x40000000u;
        vga_ctrl_base  = 0x50000000u;
        vga_ctrl_fb_base = fb_base;
        ram_fast_limit   = 0xFFFFFFFFu;   /* RAM to the top of the map */
        printf ("-I-EMU: RAM %uMB exceeds the legacy display map; relocated "
                "FB carveout to 0x%08X, legacy FB to 0x%08X, VGA ctrl to 0x%08X\n",
                mem_size / (1024u*1024u), fb_base, fb_legacy_base, vga_ctrl_base);
}

uint32_t MemWordMask = ((uint32_t)0x001ffffc);
uint32_t MemByteMask = ((uint32_t)0x001fffff);
uint32_t CLineTagsSize;

#define InvalidInstr_p  ((uint32_t)0x2ffa2ffa)

static const uint32_t boot_call_start = 0x80000120;
static const uint32_t boot_call_end = 0x80000130;
int bootdbg_stop_after_call_enabled = 0;
int bootdbg_enabled = 0;
int bootwin_enabled = 0;
int uart_trace_enabled = 0;
int sk_byte_trace_enabled = 0;
int sk_areg_trace_enabled = 0;
int sk_addr_trace_enabled = 0;
int wptr_trace_enabled = 0;
int sk_store_trace_enabled = 0;
int text_write_trace_enabled = 0;
static uint32_t text_write_trace_start = 0x80880000;
static uint32_t text_write_trace_end = 0x80890000;
int cache_debug_trace_enabled = 0;
static uint32_t cache_debug_trace_start = 0;
static uint32_t cache_debug_trace_end = 0;
uint32_t sk_entry_iptr = 0;
int sk_entry_dump_enabled = 0;
static int prolog_map_loaded = 0;
static uint32_t prolog_text_base = 0x80000070;
typedef struct {
	uint32_t text_off;
	char name[96];
} prolog_sym;
static prolog_sym *prolog_syms = NULL;
static size_t prolog_sym_count = 0;
typedef struct {
	uint32_t sym_off;
	char name[96];
} link_sym;
static link_sym *link_syms = NULL;
static size_t link_sym_count = 0;
static uint32_t link_text_base = 0;
static int link_map_loaded = 0;
static int outbyte_trace_enabled = 0;
static int outbyte_entry_logged = 0;
static int sym_dump_logged = 0;
static int sym_trace_active = 0;
static int sym_trace_remaining = 0;
static int sym_trace_skip = 0;
static uint32_t sym_trace_entry = 0;
static char sym_trace_name[96];
static int iptr_trace_enabled = 0;
static uint32_t iptr_trace_start = 0;
static uint32_t iptr_trace_end = 0;
static int boot_dump_start_enabled = 0;
static int boot_dump_start_done = 0;
static int uart_console_enabled = 1;
static int link0_trace_enabled = 1;
static int mmio_trace_enabled = 0;
static int early_printk_trace_enabled = 0;
static int nearest_sym_enabled = 0;
static int iptr_dump_logged = 0;
static int prolog_text_base_autodone = 0;
static int sk_entry_dumped = 0;
static int stop_after_boot_call = 0;
static int last_boot_call_is_gcall = 0;
static uint32_t last_boot_call_site = 0;
static uint32_t last_boot_call_target = 0;
static uint32_t last_boot_call_return = 0;
static uint32_t last_boot_call_wptr = 0;
#define Undefined_p     ((uint32_t)0xdeadbeef)

#define FAULT_HISTORY_LEN 16384
typedef struct {
	uint32_t iptr, wptr, areg, breg, creg, oreg;
	u_char icode;
} fault_history_entry;
static fault_history_entry fault_history[FAULT_HISTORY_LEN];
static unsigned int fault_history_pos;
static int fault_history_enabled = -1;
static int fault_history_dumped;

static void fault_history_record(uint32_t iptr, uint32_t wptr,
				 uint32_t areg, uint32_t breg, uint32_t creg,
				 uint32_t oreg, u_char icode)
{
	fault_history_entry *entry;

	if (fault_history_enabled < 0) {
		const char *env = getenv("T4_HISTORY_ON_FAULT");
		fault_history_enabled = env && *env && atoi(env) != 0;
	}
	if (!fault_history_enabled)
		return;

	entry = &fault_history[fault_history_pos++ % FAULT_HISTORY_LEN];
	entry->iptr = iptr;
	entry->wptr = wptr;
	entry->areg = areg;
	entry->breg = breg;
	entry->creg = creg;
	entry->oreg = oreg;
	entry->icode = icode;
}

static void fault_history_dump(const char *reason)
{
	unsigned int available, start, i;

	if (!fault_history_enabled || fault_history_dumped)
		return;
	fault_history_dumped = 1;
	available = fault_history_pos < FAULT_HISTORY_LEN ?
		fault_history_pos : FAULT_HISTORY_LEN;
	start = fault_history_pos - available;
	fprintf(stderr, "[FAULT_HISTORY] %s: last %u instructions\n",
		reason, available);
	for (i = 0; i < available; i++) {
		fault_history_entry *entry =
			&fault_history[(start + i) % FAULT_HISTORY_LEN];
		fprintf(stderr,
			"[FAULT_HISTORY] IPtr=0x%08X WPtr=0x%08X Icode=0x%02X "
			"OReg=0x%08X A=0x%08X B=0x%08X C=0x%08X\n",
			entry->iptr, entry->wptr, entry->icode, entry->oreg,
			entry->areg, entry->breg, entry->creg);
	}
	fflush(stderr);
}

static void load_prolog_map(void) {
	const char *path;
	FILE *fp;
	char line[256];
	if (prolog_map_loaded)
		return;
	prolog_map_loaded = 1;
	path = getenv("T4_PROLOG_MAP");
	if (!path || !*path)
		return;
	{
		const char *base_env = getenv("T4_TEXT_BASE");
		if (base_env && *base_env)
			prolog_text_base = (uint32_t)strtoul(base_env, NULL, 0);
	}
	fp = fopen(path, "r");
	if (!fp) {
		fprintf(stderr, "T4_PROLOG: failed to open %s\n", path);
		return;
	}
	while (fgets(line, sizeof(line), fp)) {
		char func[96];
		unsigned int off;
		if (sscanf(line, "PROLOG: func=%95s text_off=0x%x", func, &off) == 2) {
			prolog_sym *tmp = realloc(prolog_syms, (prolog_sym_count + 1) * sizeof(*prolog_syms));
			if (!tmp)
				break;
			prolog_syms = tmp;
			prolog_syms[prolog_sym_count].text_off = off;
			strncpy(prolog_syms[prolog_sym_count].name, func,
			        sizeof(prolog_syms[prolog_sym_count].name) - 1);
			prolog_syms[prolog_sym_count].name[sizeof(prolog_syms[prolog_sym_count].name) - 1] = '\0';
			prolog_sym_count++;
		}
	}
	fclose(fp);
	if (prolog_sym_count) {
		size_t i;
		for (i = 1; i < prolog_sym_count; i++) {
			size_t j = i;
			while (j > 0 && prolog_syms[j - 1].text_off > prolog_syms[j].text_off) {
				prolog_sym tmp = prolog_syms[j - 1];
				prolog_syms[j - 1] = prolog_syms[j];
				prolog_syms[j] = tmp;
				j--;
			}
		}
		fprintf(stderr, "T4_PROLOG: loaded %zu symbols (text_base=0x%08x)\n",
		        prolog_sym_count, prolog_text_base);
	}
}

static void load_link_map(void) {
	const char *path;
	FILE *fp;
	char line[512];
	if (link_map_loaded)
		return;
	link_map_loaded = 1;
	path = getenv("T4_SYM_MAP");
	if (!path || !*path)
		return;
	{
		const char *base_env = getenv("T4_SYM_BASE");
		if (base_env && *base_env)
			link_text_base = (uint32_t)strtoul(base_env, NULL, 0);
	}
	fp = fopen(path, "r");
	if (!fp) {
		fprintf(stderr, "T4_SYM_MAP: failed to open %s\n", path);
		return;
	}
	while (fgets(line, sizeof(line), fp)) {
		char *p = line;
		char *end = NULL;
		unsigned long off;
		char name[128];
		char type = '\0';

		while (*p == ' ' || *p == '\t')
			p++;
		if (!*p || *p == '\n')
			continue;
		off = strtoul(p, &end, 16);
		if (end == p)
			continue;
		while (*end == ' ' || *end == '\t')
			end++;
		if (!*end || *end == '\n')
			continue;
		type = *end;
		end++;
		while (*end == ' ' || *end == '\t')
			end++;
		if (!*end || *end == '\n')
			continue;
		if (sscanf(end, "%127s", name) != 1)
			continue;
		(void)type;
		{
			link_sym *tmp = realloc(link_syms, (link_sym_count + 1) * sizeof(*link_syms));
			if (!tmp)
				break;
			link_syms = tmp;
			link_syms[link_sym_count].sym_off = (uint32_t)off;
			strncpy(link_syms[link_sym_count].name, name,
			        sizeof(link_syms[link_sym_count].name) - 1);
			link_syms[link_sym_count].name[sizeof(link_syms[link_sym_count].name) - 1] = '\0';
			link_sym_count++;
		}
	}
	fclose(fp);
	if (link_sym_count) {
		size_t i;
		for (i = 1; i < link_sym_count; i++) {
			size_t j = i;
			while (j > 0 && link_syms[j - 1].sym_off > link_syms[j].sym_off) {
				link_sym tmp = link_syms[j - 1];
				link_syms[j - 1] = link_syms[j];
				link_syms[j] = tmp;
				j--;
			}
		}
		fprintf(stderr, "T4_SYM_MAP: loaded %zu symbols (text_base=0x%08x)\n",
		        link_sym_count, link_text_base);
	}
}

static const prolog_sym *find_prolog_sym(uint32_t iptr, uint32_t *delta_out) {
	const prolog_sym *best = NULL;
	uint32_t off;
	size_t i;
	if (!prolog_map_loaded)
		load_prolog_map();
	if (!prolog_syms || !prolog_sym_count)
		return NULL;
	if (iptr < prolog_text_base)
		return NULL;
	off = iptr - prolog_text_base;
	for (i = 0; i < prolog_sym_count; i++) {
		if (prolog_syms[i].text_off <= off)
			best = &prolog_syms[i];
		else
			break;
	}
	if (best && delta_out)
		*delta_out = off - best->text_off;
	return best;
}

static const prolog_sym *find_prolog_sym_by_name(const char *name) {
	size_t i;
	if (!prolog_map_loaded)
		load_prolog_map();
	if (!prolog_syms || !prolog_sym_count || !name)
		return NULL;
	for (i = 0; i < prolog_sym_count; i++) {
		if (strcmp(prolog_syms[i].name, name) == 0)
			return &prolog_syms[i];
	}
	return NULL;
}

static const link_sym *find_link_sym(uint32_t iptr, uint32_t *delta_out) {
	const link_sym *best = NULL;
	uint32_t off;
	size_t i;
	if (!link_map_loaded)
		load_link_map();
	if (!link_syms || !link_sym_count)
		return NULL;
	if (!link_text_base)
		return NULL;
	if (iptr < link_text_base)
		return NULL;
	off = iptr - link_text_base;
	for (i = 0; i < link_sym_count; i++) {
		if (link_syms[i].sym_off <= off)
			best = &link_syms[i];
		else
			break;
	}
	if (best && delta_out)
		*delta_out = off - best->sym_off;
	return best;
}

static const link_sym *find_link_sym_by_name(const char *name) {
	size_t i;
	if (!link_map_loaded)
		load_link_map();
	if (!link_syms || !link_sym_count || !name)
		return NULL;
	for (i = 0; i < link_sym_count; i++) {
		if (strcmp(link_syms[i].name, name) == 0)
			return &link_syms[i];
	}
	return NULL;
}

static void maybe_log_nearest_sym(uint32_t iptr, const char *reason) {
	uint32_t delta = 0;
	const link_sym *lsym;
	const prolog_sym *psym;
	if (!nearest_sym_enabled)
		return;
	lsym = find_link_sym(iptr, &delta);
	if (lsym) {
		fprintf(stderr,
		        "[NEAR_SYM] %s IPtr=0x%08X sym=%s off=0x%X+0x%X\n",
		        reason ? reason : "iptr", iptr,
		        lsym->name, lsym->sym_off, delta);
		return;
	}
	psym = find_prolog_sym(iptr, &delta);
	if (psym) {
		fprintf(stderr,
		        "[NEAR_PROLOG] %s IPtr=0x%08X sym=%s text_off=0x%X+0x%X\n",
		        reason ? reason : "iptr", iptr,
		        psym->name, psym->text_off, delta);
	}
}

static void dump_sym_bytes_once(const char *sym_name) {
	const prolog_sym *sym;
	const link_sym *lsym;
	uint32_t entry_iptr;
	uint32_t len = 32;
	uint32_t i;
	const char *len_env;
	const char *auto_env;
	if (sym_dump_logged || !sym_name || !*sym_name)
		return;
	auto_env = getenv("T4_TEXT_BASE_AUTO");
	if (auto_env && *auto_env && !prolog_text_base_autodone)
		return;
	lsym = find_link_sym_by_name(sym_name);
	if (lsym && link_text_base) {
		entry_iptr = link_text_base + lsym->sym_off;
	} else {
		sym = find_prolog_sym_by_name(sym_name);
		if (!sym)
			return;
		entry_iptr = prolog_text_base + sym->text_off;
	}
	len_env = getenv("T4_DUMP_SYM_LEN");
	if (len_env && *len_env)
		len = (uint32_t)strtoul(len_env, NULL, 0);
	fprintf(stderr, "[SYM_DUMP] func=%s entry=0x%08X len=%u\n",
	        sym_name, entry_iptr, len);
	fprintf(stderr, "[SYM_BYTES] ");
	for (i = 0; i < len; i++) {
		fprintf(stderr, "%02X%s", byte_int(entry_iptr + i),
		        (i + 1 == len) ? "\n" : " ");
	}
	sym_dump_logged = 1;
}

static void dump_iptr_bytes_once(void) {
	static int setup_done;
	const char *iptr_env;
	const char *len_env;
	uint32_t entry_iptr;
	uint32_t len = 32;
	uint32_t i;
	if (setup_done || iptr_dump_logged)
		return;
	setup_done = 1;
	iptr_env = getenv("T4_DUMP_IPTR");
	if (!iptr_env || !*iptr_env)
		return;
	entry_iptr = (uint32_t)strtoul(iptr_env, NULL, 0);
	if (!entry_iptr)
		return;
	len_env = getenv("T4_DUMP_IPTR_LEN");
	if (len_env && *len_env)
		len = (uint32_t)strtoul(len_env, NULL, 0);
	if (len > 256)
		len = 256;
	fprintf(stderr, "[IPTR_DUMP] addr=0x%08X len=%u\n", entry_iptr, len);
	fprintf(stderr, "[IPTR_BYTES] ");
	for (i = 0; i < len; i++) {
		fprintf(stderr, "%02X%s", byte_int(entry_iptr + i),
		        (i + 1 == len) ? "\n" : " ");
	}
	iptr_dump_logged = 1;
}

static void setup_sym_trace_once(void) {
	static int env_checked;
	static const char *configured_name;
	static int auto_text_base;
	const char *sym_name;
	const char *count_env;
	const prolog_sym *sym;
	const link_sym *lsym;
	const char *auto_env;
	if (sym_trace_entry || sym_trace_active)
		return;
	if (!env_checked) {
		configured_name = getenv("T4_TRACE_SYM");
		auto_env = getenv("T4_TEXT_BASE_AUTO");
		auto_text_base = auto_env && *auto_env;
		env_checked = 1;
	}
	sym_name = configured_name;
	if (!sym_name || !*sym_name)
		return;
	if (auto_text_base && !prolog_text_base_autodone)
		return;
	lsym = find_link_sym_by_name(sym_name);
	if (lsym && link_text_base) {
		sym_trace_entry = link_text_base + lsym->sym_off;
		strncpy(sym_trace_name, lsym->name, sizeof(sym_trace_name) - 1);
	} else {
		sym = find_prolog_sym_by_name(sym_name);
		if (!sym)
			return;
		sym_trace_entry = prolog_text_base + sym->text_off;
		strncpy(sym_trace_name, sym->name, sizeof(sym_trace_name) - 1);
	}
	sym_trace_name[sizeof(sym_trace_name) - 1] = '\0';
	count_env = getenv("T4_TRACE_SYM_COUNT");
	sym_trace_remaining = count_env && *count_env ? atoi(count_env) : 64;
	if (sym_trace_remaining <= 0)
		sym_trace_remaining = 64;
	count_env = getenv("T4_TRACE_SYM_SKIP");
	sym_trace_skip = count_env && *count_env ? atoi(count_env) : 0;
	if (sym_trace_skip < 0)
		sym_trace_skip = 0;
	fprintf(stderr,
	        "[SYM_TRACE] armed func=%s entry=0x%08X count=%d skip=%d\n",
	        sym_trace_name, sym_trace_entry, sym_trace_remaining,
	        sym_trace_skip);
}

static void setup_iptr_trace_once(void) {
	static int setup_done;
	const char *start_env;
	const char *len_env;
	uint32_t len = 0;
	if (iptr_trace_enabled)
		return;
	if (setup_done)
		return;
	setup_done = 1;
	start_env = getenv("T4_TRACE_IPTR");
	if (!start_env || !*start_env)
		return;
	iptr_trace_start = (uint32_t)strtoul(start_env, NULL, 0);
	if (!iptr_trace_start)
		return;
	len_env = getenv("T4_TRACE_IPTR_LEN");
	if (len_env && *len_env)
		len = (uint32_t)strtoul(len_env, NULL, 0);
	if (!len)
		len = 64;
	iptr_trace_end = iptr_trace_start + len;
	iptr_trace_enabled = 1;
	fprintf(stderr, "[IPTR_TRACE] armed start=0x%08X len=%u\n",
	        iptr_trace_start, len);
}

static void setup_text_write_trace_once(void) {
	static int setup_done = 0;
	const char *start_env;
	const char *end_env;
	const char *len_env;
	uint32_t start;
	uint32_t end;
	uint32_t len;

	if (setup_done)
		return;
	setup_done = 1;

	start_env = getenv("T4_TEXT_WRITE_START");
	end_env = getenv("T4_TEXT_WRITE_END");
	len_env = getenv("T4_TEXT_WRITE_LEN");

	if (!start_env || !*start_env)
		return;
	start = (uint32_t)strtoul(start_env, NULL, 0);
	if (!start)
		return;

	if (end_env && *end_env) {
		end = (uint32_t)strtoul(end_env, NULL, 0);
	} else if (len_env && *len_env) {
		len = (uint32_t)strtoul(len_env, NULL, 0);
		end = start + (len ? len : 0x100);
	} else {
		end = start + 0x100;
	}

	if (end > start) {
		text_write_trace_start = start;
		text_write_trace_end = end;
		fprintf(stderr,
		        "[TEXT_WRITE] range start=0x%08X end=0x%08X\n",
		        text_write_trace_start, text_write_trace_end);
	}
}

static void setup_cache_debug_trace_once(void) {
	static int setup_done = 0;
	const char *start_env;
	const char *end_env;
	const char *len_env;
	uint32_t start;
	uint32_t end;
	uint32_t len;

	if (setup_done)
		return;
	setup_done = 1;

	start_env = getenv("T4_CACHE_DEBUG_START");
	end_env = getenv("T4_CACHE_DEBUG_END");
	len_env = getenv("T4_CACHE_DEBUG_LEN");

	if (!start_env || !*start_env)
		return;
	start = (uint32_t)strtoul(start_env, NULL, 0);
	if (!start)
		return;

	if (end_env && *end_env) {
		end = (uint32_t)strtoul(end_env, NULL, 0);
	} else if (len_env && *len_env) {
		len = (uint32_t)strtoul(len_env, NULL, 0);
		end = start + (len ? len : 0x100);
	} else {
		end = start + 0x100;
	}

	if (end > start) {
		cache_debug_trace_start = start;
		cache_debug_trace_end = end;
		cache_debug_trace_enabled = 1;
		fprintf(stderr,
		        "[CACHE_DEBUG] range start=0x%08X end=0x%08X\n",
		        cache_debug_trace_start, cache_debug_trace_end);
	}
}

static void maybe_auto_text_base(void) {
	static int env_checked;
	static int auto_enabled;
	const char *auto_env;
	const prolog_sym *sym;
	const link_sym *lsym;
	if (prolog_text_base_autodone)
		return;
	if (!env_checked) {
		auto_env = getenv("T4_TEXT_BASE_AUTO");
		auto_enabled = auto_env && *auto_env;
		env_checked = 1;
	}
	if (!auto_enabled)
		return;
	if (!last_boot_call_target)
		return;
	lsym = find_link_sym_by_name("start_kernel");
	if (lsym) {
		link_text_base = last_boot_call_target - lsym->sym_off;
		prolog_text_base_autodone = 1;
		fprintf(stderr,
		        "T4_SYM_MAP: auto text_base=0x%08X (start_kernel iptr=0x%08X off=0x%X)\n",
		        link_text_base, last_boot_call_target, lsym->sym_off);
		return;
	}
	sym = find_prolog_sym_by_name("start_kernel");
	if (!sym)
		return;
	prolog_text_base = last_boot_call_target - sym->text_off;
	prolog_text_base_autodone = 1;
	fprintf(stderr,
	        "T4_PROLOG: auto text_base=0x%08X (start_kernel iptr=0x%08X off=0x%X)\n",
	        prolog_text_base, last_boot_call_target, sym->text_off);
}

/* Simple memory-mapped UART */
#define UART_BASE       ((uint32_t)0x10000000)
#define UART_DATA       (UART_BASE + 0x00)  /* Data register (R/W) */
#define UART_STATUS     (UART_BASE + 0x04)  /* Status register (R) */

#define SD_BASE         ((uint32_t)0x20000000)  /* SD card base address */
#define SD_SIZE         (128 * 1024 * 1024)     /* 128MB SD card */
#define SD_LIMIT        (SD_BASE + SD_SIZE)     /* End of SD card */

/* T9000-style boot bank (TIS 10.5): 16MB below top-of-store, byte-wide,
 * aliasing the FIRST 16MB of the SD card.  After power-up with
 * T4_SD_BOOT=1 the control unit peeks BootWptr then BootIptr from the
 * top two words and the CPU executes in place from the bank.  The full
 * card stays visible at SD_BASE (the spec lets boot space be
 * remapped/reclaimed), so the t4_sd driver is unaffected. */
#define BOOTBANK_BASE   ((uint32_t)0x7F000000)
#define BOOTBANK_SIZE   (16 * 1024 * 1024)
#define BOOT_IPTR_ADDR  ((uint32_t)0x7FFFFFF8)
#define BOOT_WPTR_ADDR  ((uint32_t)0x7FFFFFFC)

int t4_sd_boot_enabled(void);

#define VIRTIO_NET_BASE ((uint32_t)0x30000000)  /* T4Net NIC base address */
#define UART_STATUS_TXRDY  0x01  /* Transmit ready */
#define UART_STATUS_RXAVAIL 0x02  /* Receive data available */

/* Memory-mapped Link 0 (IServer) */
#define LINK0_OUT      ((uint32_t)0x80000000)
#define LINK0_IN       ((uint32_t)0x80000010)

/* MMIO IServer state */
enum {
	MMIO_IDLE = 0,
	MMIO_LEN_LO,
	MMIO_LEN_HI,
	MMIO_PAYLOAD,
	MMIO_PUTS_STREAM,
	MMIO_PUTS_LEN_LO,
	MMIO_PUTS_LEN_HI,
	MMIO_PUTS_DATA
};

static struct {
	int state;
	uint16_t payload_len;
	uint16_t payload_left;
	uint16_t payload_index;
	uint16_t data_len;
	uint16_t data_left;
	u_char cmd;
	u_char stream;
} mmio_iserver;

static u_char mmio_resp_buf[256];
static int mmio_resp_head;
static int mmio_resp_tail;

static void uart_write_byte(u_char ch);
void mmio_early_printk_report(void);
static void uart_direct_track(u_char ch);

/* SD Card functions */
u_char sd_read_byte(uint32_t offset);
void sd_write_byte(uint32_t offset, u_char value);
u_char *t4_sd_buffer(void);

/* Virtio-Net functions */
u_char virtio_net_read_byte(uint32_t addr);
void virtio_net_write_byte(uint32_t addr, u_char value);
void t4net_poll_rx(void);

/* PNG export functions */
void save_framebuffer_png(const char *filename);
extern char *png_save_filename;

int early_printk_seen = 0;
uint64_t early_printk_bytes = 0;
uint64_t iserver_stream1_bytes = 0;
uint64_t iserver_stream1_msgs = 0;
static int iserver_stream1_active = 0;
int early_printk_seen_uart = 0;
uint64_t uart_mmio_bytes = 0;
static FILE *uart_log_file = NULL;

static void mmio_resp_push(u_char b)
{
	int next = (mmio_resp_head + 1) % (int)sizeof(mmio_resp_buf);
	if (next == mmio_resp_tail)
		return;
	mmio_resp_buf[mmio_resp_head] = b;
	mmio_resp_head = next;
}

static u_char mmio_resp_pop(void)
{
	u_char b = 0;
	if (mmio_resp_head == mmio_resp_tail)
		return 0;
	b = mmio_resp_buf[mmio_resp_tail];
	mmio_resp_tail = (mmio_resp_tail + 1) % (int)sizeof(mmio_resp_buf);
	return b;
}

static void mmio_iserver_reset(void)
{
	mmio_iserver.state = MMIO_IDLE;
	mmio_iserver.payload_len = 0;
	mmio_iserver.payload_left = 0;
	mmio_iserver.payload_index = 0;
	mmio_iserver.data_len = 0;
	mmio_iserver.data_left = 0;
	mmio_iserver.cmd = 0;
	mmio_iserver.stream = 0;
	iserver_stream1_active = 0;
}

static void mmio_iserver_finish_payload(void)
{
	if (iserver_stream1_active) {
		iserver_stream1_msgs++;
	}
	if (mmio_iserver.cmd == SP_WRITE) {
		mmio_resp_push(0);
		mmio_resp_push(0);
	}
	mmio_iserver_reset();
}

static void mmio_iserver_write_byte(u_char value)
{
	static int mmio_log_count = 0;
	if (mmio_trace_enabled && mmio_log_count < 4) {
		fprintf(stderr, "[MMIO_OUT: 0x%02X]\n", value);
		mmio_log_count++;
	}
	switch (mmio_iserver.state) {
	case MMIO_IDLE:
		if (value == SP_PUTS) {
			mmio_iserver.state = MMIO_PUTS_STREAM;
			break;
		}
		mmio_iserver.payload_len = value;
		mmio_iserver.state = MMIO_LEN_HI;
		break;
	case MMIO_LEN_HI:
		mmio_iserver.payload_len |= ((uint16_t)value << 8);
		mmio_iserver.payload_left = mmio_iserver.payload_len;
		mmio_iserver.payload_index = 0;
		mmio_iserver.data_len = 0;
		mmio_iserver.data_left = 0;
		mmio_iserver.cmd = 0;
		mmio_iserver.state = MMIO_PAYLOAD;
		break;
	case MMIO_PAYLOAD:
		if (mmio_iserver.payload_left == 0) {
			mmio_iserver_finish_payload();
			break;
		}
		if (mmio_iserver.payload_index == 0) {
			mmio_iserver.cmd = value;
		} else if (mmio_iserver.payload_index == 1) {
			mmio_iserver.stream = value;
			iserver_stream1_active = (value == 1);
		} else if (mmio_iserver.payload_index == 5) {
			mmio_iserver.data_len = value;
		} else if (mmio_iserver.payload_index == 6) {
			mmio_iserver.data_len |= ((uint16_t)value << 8);
			mmio_iserver.data_left = mmio_iserver.data_len;
		} else if (mmio_iserver.payload_index >= 7 &&
			   mmio_iserver.data_left > 0) {
			if (mmio_iserver.cmd == SP_WRITE)
				uart_write_byte(value);
			if (iserver_stream1_active)
				iserver_stream1_bytes++;
			mmio_iserver.data_left--;
		}
		mmio_iserver.payload_index++;
		mmio_iserver.payload_left--;
		if (mmio_iserver.payload_left == 0)
			mmio_iserver_finish_payload();
		break;
	case MMIO_PUTS_STREAM:
		mmio_iserver.stream = value;
		iserver_stream1_active = (value == 1);
		mmio_iserver.state = MMIO_PUTS_LEN_LO;
		break;
	case MMIO_PUTS_LEN_LO:
		mmio_iserver.data_len = value;
		mmio_iserver.state = MMIO_PUTS_LEN_HI;
		break;
	case MMIO_PUTS_LEN_HI:
		mmio_iserver.data_len |= ((uint16_t)value << 8);
		mmio_iserver.data_left = mmio_iserver.data_len;
		mmio_iserver.state = MMIO_PUTS_DATA;
		break;
	case MMIO_PUTS_DATA:
		if (mmio_iserver.data_left > 0) {
			uart_write_byte(value);
			if (iserver_stream1_active)
				iserver_stream1_bytes++;
			mmio_iserver.data_left--;
		}
		if (mmio_iserver.data_left == 0)
			mmio_iserver_reset();
		break;
	default:
		mmio_iserver_reset();
		break;
	}
}

static u_char mmio_iserver_read_byte(void)
{
	return mmio_resp_pop();
}

/* VGA framebuffer and MMIO state */
#ifdef T4_SDL_FB
/* Fixed simplefb hardware is scanning from reset; no guest enable write. */
uint32_t vga_ctrl_control = VGA_CTRL_ENABLE;
uint32_t vga_ctrl_fb_base = FB_BASE_DEFAULT;
uint32_t vga_ctrl_width = FB_WIDTH;
uint32_t vga_ctrl_height = FB_HEIGHT;
uint32_t vga_ctrl_stride = FB_WIDTH * FB_BPP;
unsigned char vga_framebuffer[FB_TOTAL];  /* front + back buffer */
int vga_dirty = 0;
uint32_t vga_front_offset = 0;   /* P2: present from this offset */
int vga_dirty_y0 = -1;
int vga_dirty_y1 = -1;
static int fb_console_enabled = 0;
static int fb_console_row = 0;
static int fb_console_col = 0;

/* 8x8 font for ASCII 0x20..0x7f (font8x8_basic) */
static const unsigned char fb_font8x8[128][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x00 */
    {0x7e,0x81,0xa5,0x81,0xbd,0x99,0x81,0x7e}, /* 0x01 */
    {0x7e,0xff,0xdb,0xff,0xc3,0xe7,0xff,0x7e}, /* 0x02 */
    {0x6c,0xfe,0xfe,0xfe,0x7c,0x38,0x10,0x00}, /* 0x03 */
    {0x10,0x38,0x7c,0xfe,0x7c,0x38,0x10,0x00}, /* 0x04 */
    {0x38,0x7c,0x38,0xfe,0xfe,0xd6,0x10,0x38}, /* 0x05 */
    {0x10,0x38,0x7c,0xfe,0xfe,0x7c,0x10,0x38}, /* 0x06 */
    {0x00,0x00,0x18,0x3c,0x3c,0x18,0x00,0x00}, /* 0x07 */
    {0xff,0xff,0xe7,0xc3,0xc3,0xe7,0xff,0xff}, /* 0x08 */
    {0x00,0x3c,0x66,0x42,0x42,0x66,0x3c,0x00}, /* 0x09 */
    {0xff,0xc3,0x99,0xbd,0xbd,0x99,0xc3,0xff}, /* 0x0a */
    {0x0f,0x07,0x0f,0x7d,0xcc,0xcc,0xcc,0x78}, /* 0x0b */
    {0x3c,0x66,0x66,0x66,0x3c,0x18,0x7e,0x18}, /* 0x0c */
    {0x3f,0x33,0x3f,0x30,0x30,0x70,0xf0,0xe0}, /* 0x0d */
    {0x7f,0x63,0x7f,0x63,0x63,0x67,0xe6,0xc0}, /* 0x0e */
    {0x99,0x5a,0x3c,0xe7,0xe7,0x3c,0x5a,0x99}, /* 0x0f */
    {0x80,0xe0,0xf8,0xfe,0xf8,0xe0,0x80,0x00}, /* 0x10 */
    {0x02,0x0e,0x3e,0xfe,0x3e,0x0e,0x02,0x00}, /* 0x11 */
    {0x18,0x3c,0x7e,0x18,0x18,0x7e,0x3c,0x18}, /* 0x12 */
    {0x66,0x66,0x66,0x66,0x66,0x00,0x66,0x00}, /* 0x13 */
    {0x7f,0xdb,0xdb,0x7b,0x1b,0x1b,0x1b,0x00}, /* 0x14 */
    {0x3e,0x63,0x38,0x6c,0x6c,0x38,0xcc,0x78}, /* 0x15 */
    {0x00,0x00,0x00,0x00,0x7e,0x7e,0x7e,0x00}, /* 0x16 */
    {0x18,0x3c,0x7e,0x18,0x7e,0x3c,0x18,0xff}, /* 0x17 */
    {0x18,0x3c,0x7e,0x18,0x18,0x18,0x18,0x00}, /* 0x18 */
    {0x18,0x18,0x18,0x18,0x7e,0x3c,0x18,0x00}, /* 0x19 */
    {0x00,0x18,0x0c,0xfe,0x0c,0x18,0x00,0x00}, /* 0x1a */
    {0x00,0x30,0x60,0xfe,0x60,0x30,0x00,0x00}, /* 0x1b */
    {0x00,0x00,0xc0,0xc0,0xc0,0xfe,0x00,0x00}, /* 0x1c */
    {0x00,0x24,0x66,0xff,0x66,0x24,0x00,0x00}, /* 0x1d */
    {0x00,0x18,0x3c,0x7e,0xff,0xff,0x00,0x00}, /* 0x1e */
    {0x00,0xff,0xff,0x7e,0x3c,0x18,0x00,0x00}, /* 0x1f */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x20 */
    {0x18,0x3c,0x3c,0x18,0x18,0x00,0x18,0x00}, /* 0x21 */
    {0x36,0x36,0x12,0x00,0x00,0x00,0x00,0x00}, /* 0x22 */
    {0x36,0x36,0x7f,0x36,0x7f,0x36,0x36,0x00}, /* 0x23 */
    {0x0c,0x3e,0x03,0x1e,0x30,0x1f,0x0c,0x00}, /* 0x24 */
    {0x00,0x63,0x33,0x18,0x0c,0x66,0x63,0x00}, /* 0x25 */
    {0x1c,0x36,0x1c,0x6e,0x3b,0x33,0x6e,0x00}, /* 0x26 */
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, /* 0x27 */
    {0x18,0x0c,0x06,0x06,0x06,0x0c,0x18,0x00}, /* 0x28 */
    {0x06,0x0c,0x18,0x18,0x18,0x0c,0x06,0x00}, /* 0x29 */
    {0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00}, /* 0x2a */
    {0x00,0x0c,0x0c,0x3f,0x0c,0x0c,0x00,0x00}, /* 0x2b */
    {0x00,0x00,0x00,0x00,0x00,0x0c,0x0c,0x06}, /* 0x2c */
    {0x00,0x00,0x00,0x3f,0x00,0x00,0x00,0x00}, /* 0x2d */
    {0x00,0x00,0x00,0x00,0x00,0x0c,0x0c,0x00}, /* 0x2e */
    {0x60,0x30,0x18,0x0c,0x06,0x03,0x01,0x00}, /* 0x2f */
    {0x3e,0x63,0x73,0x7b,0x6f,0x67,0x3e,0x00}, /* 0x30 */
    {0x0c,0x0e,0x0c,0x0c,0x0c,0x0c,0x3f,0x00}, /* 0x31 */
    {0x1e,0x33,0x30,0x1c,0x06,0x33,0x3f,0x00}, /* 0x32 */
    {0x1e,0x33,0x30,0x1c,0x30,0x33,0x1e,0x00}, /* 0x33 */
    {0x38,0x3c,0x36,0x33,0x7f,0x30,0x78,0x00}, /* 0x34 */
    {0x3f,0x03,0x1f,0x30,0x30,0x33,0x1e,0x00}, /* 0x35 */
    {0x1c,0x06,0x03,0x1f,0x33,0x33,0x1e,0x00}, /* 0x36 */
    {0x3f,0x33,0x30,0x18,0x0c,0x0c,0x0c,0x00}, /* 0x37 */
    {0x1e,0x33,0x33,0x1e,0x33,0x33,0x1e,0x00}, /* 0x38 */
    {0x1e,0x33,0x33,0x3e,0x30,0x18,0x0e,0x00}, /* 0x39 */
    {0x00,0x0c,0x0c,0x00,0x00,0x0c,0x0c,0x00}, /* 0x3a */
    {0x00,0x0c,0x0c,0x00,0x00,0x0c,0x0c,0x06}, /* 0x3b */
    {0x18,0x0c,0x06,0x03,0x06,0x0c,0x18,0x00}, /* 0x3c */
    {0x00,0x00,0x3f,0x00,0x00,0x3f,0x00,0x00}, /* 0x3d */
    {0x06,0x0c,0x18,0x30,0x18,0x0c,0x06,0x00}, /* 0x3e */
    {0x1e,0x33,0x30,0x18,0x0c,0x00,0x0c,0x00}, /* 0x3f */
    {0x3e,0x63,0x7b,0x7b,0x7b,0x03,0x1e,0x00}, /* 0x40 */
    {0x0c,0x1e,0x33,0x33,0x3f,0x33,0x33,0x00}, /* 0x41 */
    {0x3f,0x66,0x66,0x3e,0x66,0x66,0x3f,0x00}, /* 0x42 */
    {0x3c,0x66,0x03,0x03,0x03,0x66,0x3c,0x00}, /* 0x43 */
    {0x1f,0x36,0x66,0x66,0x66,0x36,0x1f,0x00}, /* 0x44 */
    {0x7f,0x46,0x16,0x1e,0x16,0x46,0x7f,0x00}, /* 0x45 */
    {0x7f,0x46,0x16,0x1e,0x16,0x06,0x0f,0x00}, /* 0x46 */
    {0x3c,0x66,0x03,0x03,0x73,0x66,0x7c,0x00}, /* 0x47 */
    {0x33,0x33,0x33,0x3f,0x33,0x33,0x33,0x00}, /* 0x48 */
    {0x1e,0x0c,0x0c,0x0c,0x0c,0x0c,0x1e,0x00}, /* 0x49 */
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1e,0x00}, /* 0x4a */
    {0x67,0x66,0x36,0x1e,0x36,0x66,0x67,0x00}, /* 0x4b */
    {0x0f,0x06,0x06,0x06,0x46,0x66,0x7f,0x00}, /* 0x4c */
    {0x63,0x77,0x7f,0x7f,0x6b,0x63,0x63,0x00}, /* 0x4d */
    {0x63,0x67,0x6f,0x7b,0x73,0x63,0x63,0x00}, /* 0x4e */
    {0x1c,0x36,0x63,0x63,0x63,0x36,0x1c,0x00}, /* 0x4f */
    {0x3f,0x66,0x66,0x3e,0x06,0x06,0x0f,0x00}, /* 0x50 */
    {0x1e,0x33,0x33,0x33,0x3b,0x1e,0x38,0x00}, /* 0x51 */
    {0x3f,0x66,0x66,0x3e,0x36,0x66,0x67,0x00}, /* 0x52 */
    {0x1e,0x33,0x07,0x0e,0x38,0x33,0x1e,0x00}, /* 0x53 */
    {0x3f,0x2d,0x0c,0x0c,0x0c,0x0c,0x1e,0x00}, /* 0x54 */
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3f,0x00}, /* 0x55 */
    {0x33,0x33,0x33,0x33,0x33,0x1e,0x0c,0x00}, /* 0x56 */
    {0x63,0x63,0x63,0x6b,0x7f,0x77,0x63,0x00}, /* 0x57 */
    {0x63,0x63,0x36,0x1c,0x1c,0x36,0x63,0x00}, /* 0x58 */
    {0x33,0x33,0x33,0x1e,0x0c,0x0c,0x1e,0x00}, /* 0x59 */
    {0x7f,0x63,0x31,0x18,0x4c,0x66,0x7f,0x00}, /* 0x5a */
    {0x1e,0x06,0x06,0x06,0x06,0x06,0x1e,0x00}, /* 0x5b */
    {0x03,0x06,0x0c,0x18,0x30,0x60,0x40,0x00}, /* 0x5c */
    {0x1e,0x18,0x18,0x18,0x18,0x18,0x1e,0x00}, /* 0x5d */
    {0x08,0x1c,0x36,0x63,0x00,0x00,0x00,0x00}, /* 0x5e */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff}, /* 0x5f */
    {0x0c,0x0c,0x18,0x00,0x00,0x00,0x00,0x00}, /* 0x60 */
    {0x00,0x00,0x1e,0x30,0x3e,0x33,0x6e,0x00}, /* 0x61 */
    {0x07,0x06,0x06,0x3e,0x66,0x66,0x3b,0x00}, /* 0x62 */
    {0x00,0x00,0x1e,0x33,0x03,0x33,0x1e,0x00}, /* 0x63 */
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6e,0x00}, /* 0x64 */
    {0x00,0x00,0x1e,0x33,0x3f,0x03,0x1e,0x00}, /* 0x65 */
    {0x1c,0x36,0x06,0x0f,0x06,0x06,0x0f,0x00}, /* 0x66 */
    {0x00,0x00,0x6e,0x33,0x33,0x3e,0x30,0x1f}, /* 0x67 */
    {0x07,0x06,0x36,0x6e,0x66,0x66,0x67,0x00}, /* 0x68 */
    {0x0c,0x00,0x0e,0x0c,0x0c,0x0c,0x1e,0x00}, /* 0x69 */
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1e}, /* 0x6a */
    {0x07,0x06,0x66,0x36,0x1e,0x36,0x67,0x00}, /* 0x6b */
    {0x0e,0x0c,0x0c,0x0c,0x0c,0x0c,0x1e,0x00}, /* 0x6c */
    {0x00,0x00,0x33,0x7f,0x7f,0x6b,0x63,0x00}, /* 0x6d */
    {0x00,0x00,0x1b,0x37,0x33,0x33,0x33,0x00}, /* 0x6e */
    {0x00,0x00,0x1e,0x33,0x33,0x33,0x1e,0x00}, /* 0x6f */
    {0x00,0x00,0x3b,0x66,0x66,0x3e,0x06,0x0f}, /* 0x70 */
    {0x00,0x00,0x6e,0x33,0x33,0x3e,0x30,0x78}, /* 0x71 */
    {0x00,0x00,0x1b,0x36,0x36,0x0e,0x06,0x0f}, /* 0x72 */
    {0x00,0x00,0x3e,0x03,0x1e,0x30,0x1f,0x00}, /* 0x73 */
    {0x08,0x0c,0x3e,0x0c,0x0c,0x2c,0x18,0x00}, /* 0x74 */
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6e,0x00}, /* 0x75 */
    {0x00,0x00,0x33,0x33,0x33,0x1e,0x0c,0x00}, /* 0x76 */
    {0x00,0x00,0x63,0x6b,0x7f,0x7f,0x36,0x00}, /* 0x77 */
    {0x00,0x00,0x63,0x36,0x1c,0x36,0x63,0x00}, /* 0x78 */
    {0x00,0x00,0x33,0x33,0x33,0x3e,0x30,0x1f}, /* 0x79 */
    {0x00,0x00,0x3f,0x19,0x0c,0x26,0x3f,0x00}, /* 0x7a */
    {0x38,0x0c,0x0c,0x07,0x0c,0x0c,0x38,0x00}, /* 0x7b */
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, /* 0x7c */
    {0x07,0x0c,0x0c,0x38,0x0c,0x0c,0x07,0x00}, /* 0x7d */
    {0x6e,0x3b,0x00,0x00,0x00,0x00,0x00,0x00}, /* 0x7e */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  /* 0x7f */
};

extern int32_t quit;

/* Map guest FB address to offset in host vga_framebuffer[].
 * Primary: 0x81E00000 carveout. Legacy alias: 0x90000000. */
static inline int fb_addr_to_offset(uint32_t addr, uint32_t *off_out)
{
    if (addr >= FB_BASE && addr < FB_LIMIT) {
        *off_out = addr - FB_BASE;
        return 1;
    }
    if (addr >= FB_LEGACY_BASE && addr < FB_LEGACY_LIMIT) {
        *off_out = addr - FB_LEGACY_BASE;
        return 1;
    }
    return 0;
}

static inline int fb_addr_in_range(uint32_t addr)
{
    uint32_t off;
    return fb_addr_to_offset(addr, &off);
}

static inline int fb_range_intersects(uint32_t addr, uint32_t len)
{
    uint64_t start = addr;
    uint64_t end = start + (uint64_t)len;

    if (len == 0)
        return 0;
    /* Primary carveout */
    if (!(end <= FB_BASE || start >= FB_LIMIT))
        return 1;
    /* Legacy alias */
    if (!(end <= FB_LEGACY_BASE || start >= FB_LEGACY_LIMIT))
        return 1;
    return 0;
}

static void fb_console_scroll(void)
{
    /* 32bpp: each row is FB_WIDTH * 4 bytes, scroll 8 rows at a time */
    size_t row_bytes = FB_WIDTH * FB_BPP;
    size_t scroll_bytes = row_bytes * 8;  /* 8 pixel rows per text row */
    memmove(vga_framebuffer, vga_framebuffer + scroll_bytes,
            row_bytes * (FB_HEIGHT - 8));
    memset(vga_framebuffer + row_bytes * (FB_HEIGHT - 8), 0, scroll_bytes);
    vga_dirty = 1;
    vga_dirty_y0 = 0;
    vga_dirty_y1 = FB_HEIGHT - 1;
    fb_console_row = FB_CONSOLE_ROWS - 1;
    fb_console_col = 0;
}

static void fb_console_putc(u_char ch)
{
    int row, col, x, y;
    unsigned char glyph;
    int i, j;

    if (!fb_console_enabled)
        return;

    if (ch == '\n') {
        fb_console_row++;
        fb_console_col = 0;
        if (fb_console_row >= FB_CONSOLE_ROWS)
            fb_console_scroll();
        return;
    }
    if (ch == '\r') {
        fb_console_col = 0;
        return;
    }
    if (ch < 0x20 || ch >= 0x80)
        ch = '.';

    row = fb_console_row;
    col = fb_console_col;
    x = col * 8;
    y = row * 8;

    /* 32bpp: write full 32-bit pixels for text rendering */
    uint32_t *fb32 = (uint32_t *)vga_framebuffer;
    for (i = 0; i < 8; i++) {
        glyph = fb_font8x8[(unsigned char)ch][i];
        for (j = 0; j < 8; j++) {
            int px = x + j;
            int py = y + i;
            if (px < 0 || px >= FB_WIDTH || py < 0 || py >= FB_HEIGHT)
                continue;
            /* White (0xFFFFFFFF) or black (0x00000000) */
            fb32[py * FB_WIDTH + px] = (glyph & (1 << j)) ? 0xFFFFFFFF : 0x00000000;
        }
    }
    vga_dirty = 1;

    fb_console_col++;
    if (fb_console_col >= FB_CONSOLE_COLS) {
        fb_console_col = 0;
        fb_console_row++;
        if (fb_console_row >= FB_CONSOLE_ROWS)
            fb_console_scroll();
    }
}

static inline void fb_write_byte(uint32_t addr, unsigned char value)
{
    uint32_t off;
    if (!fb_addr_to_offset(addr, &off))
        return;
    vga_framebuffer[off] = value;
    vga_mark_dirty_offset(off);
}

static inline unsigned char fb_read_byte(uint32_t addr)
{
    uint32_t off;
    if (!fb_addr_to_offset(addr, &off))
        return 0;
    return vga_framebuffer[off];
}

static inline void fb_write_bytes(uint32_t addr, const unsigned char *src, size_t len)
{
    size_t i;
    if (!len)
        return;
    /* Byte-wise so both primary and legacy windows map correctly. */
    for (i = 0; i < len; i++)
        fb_write_byte(addr + (uint32_t)i, src[i]);
}

/* BMP file header structure */
#pragma pack(push, 1)
typedef struct {
    unsigned short bfType;
    unsigned int bfSize;
    unsigned short bfReserved1;
    unsigned short bfReserved2;
    unsigned int bfOffBits;
} BITMAPFILEHEADER;

typedef struct {
    unsigned int biSize;
    int biWidth;
    int biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned int biCompression;
    unsigned int biSizeImage;
    int biXPelsPerMeter;
    int biYPelsPerMeter;
    unsigned int biClrUsed;
    unsigned int biClrImportant;
} BITMAPINFOHEADER;
#pragma pack(pop)

/* Save framebuffer to BMP file for debugging */
void vga_dump_framebuffer_bmp(const char *filename, int frame_number)
{
    FILE *fb_dump = fopen(filename, "wb");
    if (!fb_dump) {
        fprintf(stderr, "[VGA] Failed to save framebuffer to %s\n", filename);
        return;
    }

    // BMP header for 24-bit RGB image
    BITMAPFILEHEADER bmfh;
    BITMAPINFOHEADER bmih;

    int width = vga_ctrl_width ? vga_ctrl_width : FB_WIDTH;
    int height = vga_ctrl_height ? vga_ctrl_height : FB_HEIGHT;
    int bytes_per_pixel = 3; // 24-bit RGB
    int row_size = ((width * bytes_per_pixel + 3) & ~3); // DWORD aligned
    int image_size = row_size * height;

    // Fill BMP file header
    bmfh.bfType = 0x4D42; // 'BM'
    bmfh.bfSize = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER) + image_size;
    bmfh.bfReserved1 = 0;
    bmfh.bfReserved2 = 0;
    bmfh.bfOffBits = sizeof(BITMAPFILEHEADER) + sizeof(BITMAPINFOHEADER);

    // Fill BMP info header
    bmih.biSize = sizeof(BITMAPINFOHEADER);
    bmih.biWidth = width;
    bmih.biHeight = -height; // Negative for top-down
    bmih.biPlanes = 1;
    bmih.biBitCount = 24;
    bmih.biCompression = 0; // BI_RGB
    bmih.biSizeImage = image_size;
    bmih.biXPelsPerMeter = 2835; // 72 DPI
    bmih.biYPelsPerMeter = 2835; // 72 DPI
    bmih.biClrUsed = 0;
    bmih.biClrImportant = 0;

    // Write headers
    fwrite(&bmfh, sizeof(BITMAPFILEHEADER), 1, fb_dump);
    fwrite(&bmih, sizeof(BITMAPINFOHEADER), 1, fb_dump);

    /* Present from current front buffer (P2 flip offset). */
    uint32_t *fb32 = (uint32_t *)(vga_framebuffer + vga_front_offset);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint32_t pixel = fb32[y * width + x];

            /* Transputer writes ARGB as 0xAARRGGBB to memory.
             * Little-endian x86 reads the same uint32_t as 0xAARRGGBB.
             * BMP pixel order is B, G, R. */
            fputc((pixel >> 0) & 0xFF, fb_dump);   /* B */
            fputc((pixel >> 8) & 0xFF, fb_dump);   /* G */
            fputc((pixel >> 16) & 0xFF, fb_dump);  /* R */
        }

        // Pad row to DWORD boundary
        int padding = (4 - (width * 3) % 4) % 4;
        for (int p = 0; p < padding; p++) {
            fputc(0, fb_dump);
        }
    }

    fclose(fb_dump);
    fprintf(stderr, "[VGA] Framebuffer saved to %s (frame %d)\n", filename, frame_number);
}

/* Write-watch ring (see writeword_int) */
uint32_t watch_ring[128][3];
uint32_t watch_ring_n = 0;

/* T4_IPTR_SAMPLE ring (see main loop); dumped by t4_iptr_sample_dump(). */
uint32_t iptr_samples[8192];
uint32_t wptr_samples[8192];
uint32_t iptr_sample_n = 0;
void t4_iptr_sample_dump(void)
{
	uint32_t n = iptr_sample_n > 8192u ? 8192u : iptr_sample_n;
	uint32_t i, last;
	if (!n)
		return;
	fprintf(stderr, "[IPTR-SAMPLES] %u samples:\n", n);
	for (i = 0; i < n; i++)
		fprintf(stderr, "S 0x%08X\n", iptr_samples[i]);
	/* workspace of the most recent sample, for post-mortem frame reads */
	last = (iptr_sample_n - 1) & 8191u;
	fprintf(stderr, "[LAST-FRAME] IPtr=0x%08X WPtr=0x%08X\n",
	        iptr_samples[last], wptr_samples[last]);
	for (i = 0; i < 32; i++)
		fprintf(stderr, "W%-2u 0x%08X\n", i,
		        word_int(wptr_samples[last] + 4u * i));
}

/* Frame recorder.  T4_FB_RECORD=N samples changing full-screen frames every
 * N guest instructions (useful before the desktop/application is alive).
 * T4_FB_RECORD_ON_FLIP=1 additionally records completed-screen boundaries
 * and stops periodic sampling after the first one, so a row-by-row game blit
 * can never be captured halfway through.  T4_FB_RECORD_FLIP_LIMIT=N requests
 * a clean emulator exit after N completed application frames. */
static uint32_t t4_fb_record_last_hash = 1;
static int t4_fb_record_count = 0;
static int t4_fb_record_present_seen = 0;
static int t4_fb_record_flip_seen = 0;
static uint64_t t4_fb_record_flip_count = 0;

static int t4_env_truthy(const char *name)
{
	const char *e = getenv(name);
	return e && *e && strcmp(e, "0") != 0;
}

static void t4_fb_record_capture(int force)
{
	uint32_t w = vga_ctrl_width ? vga_ctrl_width : FB_WIDTH;
	uint32_t h = vga_ctrl_height ? vga_ctrl_height : FB_HEIGHT;
	uint32_t n = w * h, i;
	const uint32_t *fb = (const uint32_t *)(vga_framebuffer
	                                        + vga_front_offset);
	uint32_t hash = 2166136261u;

	/* A sparse hash misses thin console glyphs.  Boot polling is infrequent,
	 * so hashing the complete 800x600 surface is cheap and reliable. */
	for (i = 0; i < n; i++)
		hash = (hash ^ fb[i]) * 16777619u;
	if (!force && hash == t4_fb_record_last_hash)
		return;
	t4_fb_record_last_hash = hash;
	{
		char name[64];
		snprintf(name, sizeof(name), "fbrec_%05d.bmp",
		         t4_fb_record_count);
		vga_dump_framebuffer_bmp(name, t4_fb_record_count);
		t4_fb_record_count++;
	}
}

static void t4_fb_record_poll(void)
{
	if (t4_fb_record_present_seen &&
	    t4_env_truthy("T4_FB_RECORD_ON_FLIP"))
		return;
	t4_fb_record_capture(0);
}

static void t4_fb_record_flip(int application_frame)
{
	static uint64_t limit = ~0ULL;

	if (!t4_env_truthy("T4_FB_RECORD_ON_FLIP"))
		return;
	t4_fb_record_present_seen = 1;
	/* A zero-valued FLIP is fbwm's completed-screen marker.  Record changed
	 * splash/login/desktop screens but do not start the timed DOOM section. */
	if (!application_frame) {
		t4_fb_record_capture(0);
		return;
	}
	if (!t4_fb_record_flip_seen) {
		const char *e = getenv("T4_FB_RECORD_FLIP_LIMIT");
		limit = (e && *e) ? strtoull(e, NULL, 0) : 0;
		t4_fb_record_flip_seen = 1;
		fprintf(stderr, "[VGA] Completed-frame recording starts at frame %d\n",
		        t4_fb_record_count);
	}
	t4_fb_record_capture(1);
	t4_fb_record_flip_count++;
	if (limit && t4_fb_record_flip_count >= limit) {
		fprintf(stderr,
		        "[VGA] Completed-frame limit reached: %llu frames\n",
		        (unsigned long long)t4_fb_record_flip_count);
		quit = TRUE;
	}
}

/* Save framebuffer to file for debugging */
void vga_dump_framebuffer(const char *filename)
{
    FILE *fb_dump = fopen(filename, "wb");
    if (fb_dump) {
        fwrite(vga_framebuffer, 1, FB_SIZE, fb_dump);
        fclose(fb_dump);
        fprintf(stderr, "[VGA] Framebuffer saved to %s\n", filename);
    } else {
        fprintf(stderr, "[VGA] Failed to save framebuffer to %s\n", filename);
    }
}

void vga_update(void)
{
    display_present_force();
}

#endif /* T4_SDL_FB */

uint32_t word_int (uint32_t);

#define SN(reg)   (reg.u.sn)
#define DB(reg)   (reg.u.db)

#define Instruction   (t4cpu->s_Instruction)
#define Icode         (t4cpu->s_Icode)
#define Idata         (t4cpu->s_Idata)
#define loop          (t4cpu->s_loop)
#define count1        (t4cpu->s_count1)
#define count2        (t4cpu->s_count2)
#define count3        (t4cpu->s_count3)

/* Processor state the instructions latch: per node thread in a threaded
 * network (-st), where every node runs TVX and move2d at once. */
T4_THREAD_LOCAL tvx_viewport tvx_vp = {1.0f, 1.0f, 0.0f, 0.0f, 1.0f};
T4_THREAD_LOCAL tvx_light tvx_lit = {{0.0f, 0.0f, 1.0f}, 0.0f, 1.0f};
T4_THREAD_LOCAL uint32_t vec_len;
T4_THREAD_LOCAL uint32_t quant_fmt;
T4_THREAD_LOCAL uint32_t elem_stride;
T4_THREAD_LOCAL uint32_t stride_mask;
T4_THREAD_LOCAL uint32_t tvx_src_fmt;
#define TVX_STEP(bit, dense) \
        (((stride_mask & (bit)) && elem_stride) ? elem_stride : (dense))
T4_THREAD_LOCAL uint32_t m2dSourceStride;       /* move2d source stride */
T4_THREAD_LOCAL uint32_t m2dDestStride;         /* move2d destination stride */
T4_THREAD_LOCAL uint32_t m2dLength;             /* move2d length (no. of rows) */

#define ProcessQEmpty           ((NotProcess_p == FPtrReg[0]) && (NotProcess_p == FPtrReg[1]))
#define TimerQEmpty             ((NotProcess_p == TPtrLoc[0]) && (NotProcess_p == TPtrLoc[1]))



/* The interrupted low-priority process lives in the T9000 shadow registers
 * (ISM 13.3).  interrupt() also mirrors it into the T800 save area at
 * #8000002C.. for T800 code, until the T9000 channel space is used: those
 * words are event channel words there. */
#define ClearInterrupt          (t4cpu->s_sh.wdesc = NotProcess_p)
#define ReadInterrupt           (NotProcess_p != GetDescWPtr(t4cpu->s_sh.wdesc))
void t4_wake (uint32_t wdesc);
static void shadow_save (void);
static void shadow_resume (void);
static void shadow_transfer (uint32_t from, uint32_t to, uint32_t ptr, int store);

/* T9000 StatusReg bits (ISM table 5.5) that T4 models.  Bit 28 is unused by
 * the T9000 and carries T4's HaltOnError, which it has no equivalent of. */
#define SB_FPERROR      (1u << 2)
#define SB_INTOV        (1u << 6)
#define SB_TSDIS        (1u << 25)
#define SB_ISP          (1u << 27)
#define SB_T4HALT       (1u << 28)
static uint32_t t9k_status_pack (uint32_t raw, int fp_error, int tsdis, int is_p);
static uint32_t t9k_status_raw (uint32_t sb);

#define GotoSNPBit              0x00000001
#define HaltOnErrorFlag         0x00000080
#define ErrorFlag               0x80000000

#define SetGotoSNP              STATUSReg |= GotoSNPBit
#define ClearGotoSNP            STATUSReg &= ~GotoSNPBit
#define ReadGotoSNP             (STATUSReg & GotoSNPBit)

#define SetError                STATUSReg |= ErrorFlag
#define ClearError              STATUSReg &= ~ErrorFlag
#define ReadError               (STATUSReg & ErrorFlag)

#define SetHaltOnError          STATUSReg |= HaltOnErrorFlag
#define ClearHaltOnError        STATUSReg &= ~HaltOnErrorFlag
#define ReadHaltOnError         (STATUSReg & HaltOnErrorFlag)


#define Temp_s          ( 0)
#define Iptr_s          (-1)
#define Link_s          (-2)
#define State_s         (-3)
#define Pointer_s       (-3)
#define TLink_s         (-4)
#define Time_s          (-5)


#define GetDescPriority(wdesc)  ((wdesc) & 0x00000001)
#define GetDescWPtr(wdesc)      ((wdesc) & 0xfffffffe)

#define BitsPerByte             8
#define BytesPerWord            4
#define ByteSelectMask          0x00000003
#define BitsPerWord             (BitsPerByte * BytesPerWord)
#define WordsRead(addr,len)     (((addr&(BytesPerWord-1))?1:0)+(len+(BytesPerWord-1))/BytesPerWord)
#define BytesRead(addr,len)     (WordsRead(addr,len)*BytesPerWord)

#ifdef NDEBUG
static HOT_INLINE uint32_t word_fast (uint32_t ptr);
static HOT_INLINE void     writeword_fast (uint32_t ptr, uint32_t value);
static HOT_INLINE u_char   byte_fast (uint32_t ptr);
static HOT_INLINE void     writebyte_fast (uint32_t ptr, u_char value);
#define writeword(a,x)  writeword_fast(a,x)
#define writebyte(a,x)  writebyte_fast(a,x)
#define byte(a)         byte_fast(a)
#define word(a)         word_fast(a)
#endif

/* TVX-GFX position fetch: component j of an xyz vertex at base, in the
 * format latched by VSETFMT.  The narrow formats run the TVX stream
 * converters in front of the geometry datapath. */
static float tvx_fetch_pos (uint32_t base, uint32_t j)
{
        uint32_t h;

        switch (tvx_src_fmt) {
        case 1:                                 /* bf16 */
                h = (uint32_t)byte (base + 2*j)
                  | ((uint32_t)byte (base + 2*j + 1) << 8);
                return tvx_bits_to_float (tvx_bf16_to_f32 (h));
        case 2:                                 /* IEEE f16 */
                h = (uint32_t)byte (base + 2*j)
                  | ((uint32_t)byte (base + 2*j + 1) << 8);
                return tvx_bits_to_float (tvx_f16_to_f32 (h));
        default:                                /* f32 */
                return real32 (base + 4*j).fp;
        }
}
#define TVX_POS_DENSE() (tvx_src_fmt ? 6u : 12u)

/* QDOT and QGEMM also take unquantised weights - F32 (ggml type 0), F16 (1)
 * and BF16 (30) - in blocks of 32 elements, so a router or a small float
 * projection needs no instruction of its own.  Everything else is a GGUF
 * block format. */
static int tvx_qfmt_block_bytes (int fmt)
{
        switch (fmt) {
        case 0:  return 32 * 4;
        case 1:
        case 30: return 32 * 2;
        default: return tvx_gguf_block_bytes (fmt);
        }
}

static int tvx_qfmt_decode_block (int fmt, const uint8_t *raw, float *out)
{
        int i;

        switch (fmt) {
        case 0:
                for (i = 0; i < 32; i++)
                        out[i] = tvx_bits_to_float ((uint32_t)raw[4*i]
                                 | (uint32_t)raw[4*i + 1] << 8
                                 | (uint32_t)raw[4*i + 2] << 16
                                 | (uint32_t)raw[4*i + 3] << 24);
                return 32;
        case 1:
        case 30:
                for (i = 0; i < 32; i++) {
                        uint32_t h = (uint32_t)raw[2*i]
                                   | (uint32_t)raw[2*i + 1] << 8;
                        out[i] = tvx_bits_to_float (fmt == 1
                                 ? tvx_f16_to_f32 (h) : tvx_bf16_to_f32 (h));
                }
                return 32;
        default:
                return tvx_gguf_decode_block (fmt, raw, out);
        }
}

/* VSETFMT 3: QGEMM activations are already quantised, in the format llama.cpp
 * pairs the QSETFMT weights with - block_q8_K (VQ8K) for the K- and i-quants,
 * block_q8_0 (VQ80) for q4_0, q5_0, q8_0, iq4_nl and mxfp4. */
#define TVX_FMT_QACT 3u

/* VQ8K / VQ80: quantise VSETLEN activations (f32, bf16 or f16 per VSETFMT)
 * with llama.cpp's quantize_row_q8_K_ref (256 per block) or
 * quantize_row_q8_0_ref (32 per block).  A trailing partial block is
 * ignored, as VHAD32 ignores a partial group. */
static void tvx_vqact (uint32_t dst, uint32_t src, int act)
{
        static T4_THREAD_LOCAL float *x;     /* per node thread */
        static T4_THREAD_LOCAL uint8_t *y;
        static T4_THREAD_LOCAL size_t xcap, ycap;
        uint32_t qk = act == TVX_GGML_Q8_K ? QK_K : QK8_0;
        size_t bb = act == TVX_GGML_Q8_K ? sizeof (block_q8_K)
                                         : sizeof (block_q8_0);
        uint32_t nb = vec_len / qk, n = nb * qk, i;

        if (nb == 0)
                return;
        if (n > xcap) {
                float *p = realloc (x, (size_t)n * sizeof *x);
                if (!p)
                        return;
                x = p;
                xcap = n;
        }
        if (nb * bb > ycap) {
                uint8_t *p = realloc (y, nb * bb);
                if (!p)
                        return;
                y = p;
                ycap = nb * bb;
        }
        for (i = 0; i < n; i++)
                x[i] = tvx_fetch_pos (src, i);
        memset (y, 0, nb * bb);         /* bsums of an all-zero Q8_K block */
        if (act == TVX_GGML_Q8_K)
                tvx_quantize_row_q8_K (x, (block_q8_K *)y, n);
        else
                tvx_quantize_row_q8_0 (x, (block_q8_0 *)y, n);
        for (i = 0; i < nb * (uint32_t)bb; i++)
                writebyte_int (dst + i, y[i]);
}

/* QGEMM with quantised activations: each row is llama.cpp's generic integer
 * dot (tvx_q8k.h) of nblk weight blocks against the same nblk activation
 * blocks, so the activations are read once for every row.  Returns the
 * number of rows written - 0 when the QSETFMT format has no integer kernel
 * (q4_1 and q5_1, whose partner is Q8_1, and the float formats). */
static uint32_t tvx_qgemm_int (uint32_t dst, uint32_t aptr, uint32_t wptr,
                               uint32_t nblk)
{
        static T4_THREAD_LOCAL uint8_t *wbuf, *abuf;  /* per node thread */
        static T4_THREAD_LOCAL size_t wcap, acap;
        int act = tvx_qint_act_type ((int)quant_fmt);
        size_t bsz = (size_t)tvx_gguf_block_bytes ((int)quant_fmt);
        size_t ab = act == TVX_GGML_Q8_K ? sizeof (block_q8_K)
                                         : sizeof (block_q8_0);
        uint32_t ne = act == TVX_GGML_Q8_K ? QK_K : QK8_0;
        size_t wn = nblk * bsz, an = nblk * ab, i;
        uint32_t r;
        fpreal32_t out;

        if (act == 0 || bsz == 0 || nblk == 0)
                return 0;
        if (wn > wcap) {
                uint8_t *p = realloc (wbuf, wn);
                if (!p)
                        return 0;
                wbuf = p;
                wcap = wn;
        }
        if (an > acap) {
                uint8_t *p = realloc (abuf, an);
                if (!p)
                        return 0;
                abuf = p;
                acap = an;
        }
        for (i = 0; i < an; i++)
                abuf[i] = byte (aptr + i);
        for (r = 0; r < vec_len; r++) {
                uint32_t rp = wptr + r * (uint32_t)wn;

                for (i = 0; i < wn; i++)
                        wbuf[i] = byte (rp + i);
                tvx_qint_vec_dot ((int)quant_fmt, (int)(nblk * ne), &out.fp,
                                  wbuf, abuf);
                writereal32 (dst + 4 * r, out);
        }
        return vec_len;
}

/* Internal variables. */

static int noisy_boot_trace_enabled(void)
{
	static int init = 0;
	static int enabled = 0;
	const char *env;

	if (!init) {
		env = getenv("T4_NOISY_BOOT_TRACE");
		enabled = (env && *env && strcmp(env, "0") != 0);
		init = 1;
	}

	return enabled;
}

/*
 * The release emulator used to pass every guest instruction through the
 * complete boot-debug framework even when no trace was requested.  Linux
 * executes enough instructions in scheduler and initcall paths for those
 * helper calls and lazy environment probes to dominate runtime.
 *
 * Keep the original decoder/cache as the correctness reference, but let an
 * ordinary run bypass diagnostic-only work.  Mentioning any diagnostic
 * option retains the full path, including its one-shot setup logic.
 */
static int runtime_diagnostics_requested(void)
{
	static const char *const names[] = {
		"T4_BOOTDBG", "T4_BOOTWIN", "T4_STOP_AFTER_GCALL",
		"T4_STOP_IPTR", "T4_STOP_RETURN", "T4_STOP_AREG",
		"T4_STOP_B_NON_SIGNEXT", "T4_STOP_B_RANGE", "T4_STOP_W3_MIN",
		"T4_NOISY_BOOT_TRACE", "T4_HISTORY_ON_FAULT",
		"T4_WPTR_RANGE", "T4_WPTR_TRACE", "T4_WATCH_ADDR",
		"T4_ENTRY_TRACE", "T4_ENTRY_START", "T4_ENTRY_END",
		"T4_ENTRY_IPTR", "T4_SK_ENTRY_IPTR", "T4_SK_BYTE_TRACE",
		"T4_SK_AREG_TRACE", "T4_SK_ADDR_TRACE", "T4_SK_STORE_TRACE",
		"T4_TRACE_SYM", "T4_TRACE_IPTR", "T4_TRACE_FB_LDC",
		"T4_TEXT_WRITE_START", "T4_CACHE_DEBUG_START",
		"T4_DUMP_IPTR", "T4_DUMP_SYM", "T4_DUMP_BOOT_START",
		"T4_OUTBYTE_TRACE", "T4_NEAREST_SYM", NULL
	};
	unsigned int i;

	if (bootdbg_enabled || bootwin_enabled ||
	    bootdbg_stop_after_call_enabled || outbyte_trace_enabled ||
	    sk_byte_trace_enabled || sk_areg_trace_enabled ||
	    sk_addr_trace_enabled || sk_store_trace_enabled ||
	    text_write_trace_enabled || cache_debug_trace_enabled ||
	    wptr_trace_enabled || nearest_sym_enabled)
		return 1;

	for (i = 0; names[i]; i++) {
		const char *value = getenv(names[i]);
		if (value && *value && strcmp(value, "0") != 0)
			return 1;
	}
	return 0;
}

/* Product identity for LDDEVID instruction.
 * Real hardware: T414=0, T800-20=5, T805=6.
 * T806 is our custom 200MHz variant. Use ID 0x806 (distinguishable). */
static uint32_t ProductIdentity = 0x00000000;
#define TimersGo   1
#define TimersStop 0
int32_t quit = FALSE;
int32_t quitstatus;

/* Signal handler. */
void handler (int);

u_char *SharedLinks;
u_char *SharedEvents;
#define SHLINK_EXTRA        1024

#define StrPrio(p)      ((p) ? "Lo" : "Hi")

void UpdateWdescReg (uint32_t wdesc)
{
        WdescReg     = wdesc;
        WPtr         = GetDescWPtr(wdesc);
        ProcPriority = GetDescPriority(wdesc);
}

static void handle_j0_break(void)
{
        uint32_t offset = (ProcPriority == HiPriority) ? 0 : 2;
        uint32_t base = MemStart + (offset * BytesPerWord);
        writeword_int(base, WPtr);
        writeword_int(base + BytesPerWord, IPtr);
        WPtr = word_int(base);
        IPtr = word_int(base + BytesPerWord);
        UpdateWdescReg(WPtr | ProcPriority);
}

#define Wdesc           WdescReg
#define Idle            (NotProcess_p == WPtr)
#define IdleProcess_p   (NotProcess_p | LoPriority)

/* External variables. */
extern int analyse;
extern int verbose;
extern int exitonerror;
extern int FromServerLen;
extern int emudebug;
extern int memdebug;
extern int memnotinit;
extern int msgdebug;
extern int cachedebug;
extern char NetConfigName[256];

/* Macros. */
#define index(a,b)		((a)+(BytesPerWord*(b)))

/* Profile information. */
uint32_t instrprof[0x400];
uint32_t combinedprof[0x400][0x400];

#define Arg0            Acache[islot]._Arg0
#define Arg1            Acache[islot]._Arg1

#define OprCombined(x,y)(((x) == 0xf0)&&((y)>0xff)&&((y)<0x17c))
#define CLINE_SIZE      ((uint32_t)(1 << T4CLINEBITS))

static HOT_INLINE uint32_t IHASH(uint32_t a)
{
        return a & (MAX_ICACHE-1);
}

static HOT_INLINE u_char IsCached(uint32_t a)
{
        a &= MemByteMask; a >>= T4CLINEBITS;
        return CLineTags[a >> 3] & (1 << (a & 7));
}

static void SetCached(uint32_t a)
{
        a &= MemByteMask; a >>= T4CLINEBITS;
        CLineTags[a >> 3] |= (1 << (a & 7));
}

static void ClearCached(uint32_t a)
{
        a &= MemByteMask; a >>= T4CLINEBITS;
        CLineTags[a >> 3] &= ~(1 << (a & 7));
}

#if 1

#define INVALIDATE_ADDR(a)      InvalidateAddr(a)

static uint32_t InvalidateAddr (uint32_t a)
{
        int i;

        if (IsCached(a)) {
                ClearCached(a);
                a &= ~(CLINE_SIZE - 1);
                for (i = 0; i < CLINE_SIZE; a++, i++) {
                        uint32_t x = IHASH(a);
                        if (a == Icache[x].ip)
                                Icache[x].ip = IC_NOADDR;
                }
                return a;
        }
        return (a & ~(CLINE_SIZE-1)) + CLINE_SIZE;
}

static void InvalidateRange (uint32_t a, uint32_t n)
{
        uint64_t lines;
#ifdef T4_SDL_FB
        /* FB/MMIO memory never contains code — skip icache invalidation */
        if (a >= FB_BASE && (a + n) <= FB_LIMIT)
                return;
#endif

        /* Count the lines the range covers rather than comparing against
         * its end address.  With 32-bit `a + n`, a store into the top line
         * of the address space stepped `a` round to 0, every later line
         * start stayed below the end, and the loop - and so the instruction
         * - never finished.  The SIGTERM check between instructions was
         * never reached either: c-testsuite 00207's runaway Wptr kept
         * `timeout 10 t4` alive for hours.  A range that wraps now also
         * invalidates its wrapped part, where it used to invalidate none. */
        if (n == 0)
                return;
        lines = ((uint64_t)(a & (CLINE_SIZE - 1)) + n + CLINE_SIZE - 1) >> T4CLINEBITS;
        while (lines--)
                a = InvalidateAddr (a);
}

#else

#define INVALIDATE_ADDR(a)      InvalidateSlot(IHASH(a), a)

static void InvalidateSlot (uint32_t x, uint32_t a)
{
        if (a == Icache[x].ip)
                Icache[x].ip = IC_NOADDR;
}

static void InvalidateRange (uint32_t a, uint32_t n)
{
        uint32_t i, x;

        x = IHASH(a);
Again:  if (x + n > MAX_ICACHE)
        {
                n -= (MAX_ICACHE - x);
                for (; x < MAX_ICACHE;)
                        InvalidateSlot (x++, a++);
                x = 0;
                goto Again;
        }
        else
                for (i = 0; i < n; i++)
                        InvalidateSlot (x++, a++);
}

#endif

/* T4_WATCH_ADDR as writeword_int() last read it; 0 until then.  The inline
 * fast paths below defer to the *_int versions unless it is known unset. */
static uint32_t ram_watch_addr;

/* On-chip core that is plain RAM: everything below ExtMemStart except the
 * Link0 channel words, which the slow paths treat as MMIO in some
 * configurations. */
#define CORE_RAM_ADDR(p) \
        ((uint32_t)((p) - MostNeg) < (uint32_t)(ExtMemStart - MostNeg) && \
         ((p) & ~3u) != LINK0_IN && ((p) & ~3u) != LINK0_OUT)

/* [a, a+n) lies wholly in the external-RAM window of the fast paths.
 * ExtMemAddr() alone is a signed compare that also takes the positive MMIO
 * addresses (UART, SD, virtio). */
#define EXT_RAM_RANGE(a,n) \
        ((uint32_t)((a) - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart) && \
         (uint32_t)(n) <= (uint32_t)(ram_fast_limit - (a)))

#ifdef NDEBUG
/* Inline fast paths for plain RAM - external memory and the on-chip core -
 * in front of the out-of-line accessors.  Everything else (MMIO, T9000
 * address translation, the framebuffer windows, the Link0 channel words, a
 * T4_WATCH_ADDR watchpoint) takes the *_int slow path unchanged.  Returns
 * the backing array for ptr, or NULL for the slow path. */
static HOT_INLINE u_char *ram_fast_base (uint32_t ptr)
{
        if (unlikely((t9k_prot.active && !t9k_prot.phys_bypass) ||
                     ram_watch_addr != 0xFFFFFFFF))
                return NULL;
        if (likely((uint32_t)(ptr - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart))) {
#ifdef T4_SDL_FB
                if (unlikely(fb_addr_in_range(ptr)))
                        return NULL;
#endif
                return mem;
        }
        if (CORE_RAM_ADDR(ptr))
                return core;
        return NULL;
}

static HOT_INLINE uint32_t word_fast (uint32_t ptr)
{
        u_char *base = ram_fast_base (ptr);

        if (likely(base != NULL))
                return *(uint32_t *)(base + (MemWordMask & ptr));
        return word_int (ptr);
}

static HOT_INLINE void writeword_fast (uint32_t ptr, uint32_t value)
{
        u_char *base = ram_fast_base (ptr);

        if (likely(base != NULL)) {
                *(uint32_t *)(base + (MemWordMask & ptr)) = value;
                /* The store is word-aligned, so it touches only ptr's
                 * icache line; skip the call when no code is cached there. */
                if (unlikely(IsCached (ptr)))
                        InvalidateRange (ptr & ~3u, 4);
                return;
        }
        writeword_int (ptr, value);
}

static HOT_INLINE u_char byte_fast (uint32_t ptr)
{
        u_char *base = ram_fast_base (ptr);

        if (likely(base != NULL))
                return base[MemByteMask & ptr];
        return byte_int (ptr);
}

static HOT_INLINE void writebyte_fast (uint32_t ptr, u_char value)
{
        u_char *base = ram_fast_base (ptr);

        if (likely(base != NULL)) {
                base[MemByteMask & ptr] = value;
                if (unlikely(IsCached (ptr)))
                        INVALIDATE_ADDR (ptr);
                return;
        }
        writebyte_int (ptr, value);
}
#endif

#include <signal.h>
extern volatile sig_atomic_t t4_sig_pending;
extern void t4_signal_exit (int);

#define NO_ICODE        0x400

static struct {
        u_short code0, code1;
        uint32_t ccode;
} combined[] = {
        {  0xd0 /* stl  */,  0x70 /* ldl      */, 0x100 },
        {  0x70 /* ldl  */,  0x30 /* ldnl     */, 0x101 },
        {  0xc0 /* eqc  */,  0xa0 /* cj       */, 0x102 },
        {  0x70 /* ldl  */,  0x50 /* ldnlp    */, 0x103 },
        {  0x70 /* ldl  */,  0x70 /* ldl      */, 0x104 },
        {  0x10 /* ldlp */, 0x18a /* fpldnldb */, 0x105 },
        {  0xb0 /* ajw  */, 0x120 /* ret      */, 0x106 },
        {  0x10 /* ldlp */,  0x40 /* ldc      */, 0x107 },
        {  0x10 /* ldlp */,  0x70 /* ldl      */, 0x108 },
        {  0xd0 /* stl  */,  0xd0 /* stl      */, 0x109 },
        {  0x70 /* ldl  */, 0x173 /* cflerr   */, 0x10a },
        {  0x10 /* ldlp */, 0x188 /* fpstnlsn */, 0x10b },
        {  0x10 /* ldlp */, 0x18e /* fpldnlsn */, 0x10c },
        {  0x70 /* ldl  */,  0x80 /* adc      */, 0x10d },
        {  0x70 /* ldl  */,  0xe0 /* stnl     */, 0x10e },
        {  0x40 /* ldc  */,  0x70 /* ldl      */, 0x10f },
        {  0xd0 /* stl  */,  0x00 /* j        */, 0x110 },
        { 0x109 /* gt   */,  0xa0 /* cj       */, 0x111 },
        { 0x10a /* wsub */,  0xe0 /* stnl     */, 0x112 },
        {  0x70 /* ldl  */, 0x10a /* wsub     */, 0x113 },
        { 0x15a /* dup  */,  0xd0 /* stl      */, 0x114 },
        { 0x142 /* mint */, 0x146 /* and      */, 0x115 },
        { NO_ICODE, NO_ICODE, NO_ICODE }
};
/* The fused pseudo-opcodes occupy 0x100.. contiguously and are dispatched only
 * by the combined cases in the OPR switch - no real T9000 secondary lives
 * there.  The T9000 grouper model uses this range to decompose a fused slot
 * back into the pair it stands for. */
#define T9K_COMBINED_BASE   0x100
#define T9K_COMBINED_COUNT  ((int)(sizeof combined / sizeof combined[0]) - 1)
static u_char combinations[0x400 * 0x400];

static void t4_fill_combinations(void)
{
        int i;
        memset(combinations, 0, sizeof(combinations));
        /* -s414/-s425 check each instruction on its own (t4xx_has_opr):
         * a fused pair would hide an FPU or T425 instruction */
        if (t4_isa_strict)
                return;
        for (i = 0; NO_ICODE != combined[i].code0; i++)
                combinations[(combined[i].code0 << 10) + combined[i].code1] = (u_char)(1 + i);
}

/* Support functions. */

#ifdef _MSC_VER
#define t4_bitcount(x)			__popcnt (x)
#endif

#ifdef __GNUC__
#define t4_bitcount(x)			__builtin_popcount (x)
#endif

#ifndef t4_bitcount
uint32_t t4_bitcount(uint32_t x)
{
        uint32_t result;

        result = 0;
        while (x)
        {
                if (x & 1)
                        result++;
                x >>= 1;
        }
        return result;
}
#endif

#ifdef __clang__
#define t4_bitreverse(x)        __builtin_bitreverse32 (x)
#endif

#ifndef t4_bitreverse
uint32_t t4_bitreverse (uint32_t x)
{
	unsigned int s = BitsPerWord;
	uint32_t mask = ~0;
	while ((s >>= 1) > 0)
	{
		mask ^= mask << s;
		x = ((x >> s) & mask) | ((x << s) & ~mask);
	}
	return x;
}
#endif

void fp_drop (void)
{
        FAReg = FBReg;
        FBReg = FCReg;
}

void fp_drop2 (void)
{
        FAReg = FCReg;
        FBReg = FCReg;
}

/* Pop a REAL64 from the floating point stack. */
void fp_popdb (fpreal64_t *fp)
{
#ifndef NDEBUG
        if (FAReg.length == FP_REAL64)
#endif
                *fp = DB(FAReg);
#ifndef NDEBUG
        else
        {
                printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fp_popdb)\n");
                *fp = DUndefined;
        }
#endif
        fp_drop ();
}


/* Peek two REAL64s on the floating point stack. */
void fp_peek2db (fpreal64_t *fb, fpreal64_t *fa)
{
#ifndef NDEBUG
        if (FBReg.length == FP_REAL64 && FAReg.length == FP_REAL64)
        {
#endif
                *fb = DB(FBReg);
                *fa = DB(FAReg);
#ifndef NDEBUG
        }
        else
        {
                printf ("-W-EMUFPU: Warning - FBReg/FAReg are not REAL64! (fp_peek2db)\n");
                *fb = DUndefined;
                *fa = DUndefined;
        }
#endif
}


/* Pop two REAL64s from the floating point stack. */
void fp_pop2db (fpreal64_t *fb, fpreal64_t *fa)
{
        fp_peek2db (fb, fa);
        fp_drop2 ();
}


/* Push a REAL64 to the floating point stack. */
void fp_pushdb (fpreal64_t fp)
{
        FCReg = FBReg;
        FBReg = FAReg;
        FAReg.length = FP_REAL64;
        DB(FAReg) = fp;
}


/* Pop a REAL32 from the floating point stack. */
void fp_popsn (fpreal32_t *fp)
{
#ifndef NDEBUG
        if (FP_REAL32 == FAReg.length)
#endif
                *fp = SN(FAReg);
#ifndef NDEBUG
        else
        {
                printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fp_popsn)\n");
                *fp = RUndefined;
        }
#endif
        fp_drop ();
}


/* Peek two REAL32s on the floating point stack. */
void fp_peek2sn (fpreal32_t *fb, fpreal32_t *fa)
{
#ifndef NDEBUG
        if (FBReg.length == FP_REAL32 && FAReg.length == FP_REAL32)
        {
#endif
                *fb = SN(FBReg);
                *fa = SN(FAReg);
#ifndef NDEBUG
        }
        else
        {
                printf ("-W-EMUFPU: Warning - FBReg/FAReg are not REAL64!\n");
                *fb = RUndefined;
                *fa = RUndefined;
        }
#endif
}


/* Pop two REAL32s from the floating point stack. */
void fp_pop2sn (fpreal32_t *fb, fpreal32_t *fa)
{
        fp_peek2sn (fb, fa);
        fp_drop2 ();
}


/* Push a REAL32 to the floating point stack. */
void fp_pushsn (fpreal32_t fp)
{
        FCReg = FBReg;
        FBReg = FAReg;
        FAReg.length = FP_REAL32;
        SN(FAReg) = fp;
}


/* Do a binary floating point operation. */
#ifdef T4RELEASE
#define fp_dobinary(dbop, snop) \
{ \
        fpreal64_t dbtemp1, dbtemp2; \
        fpreal32_t sntemp1, sntemp2; \
 \
        ResetRounding = TRUE; \
 \
        if (FP_REAL64 == FAReg.length) \
        { \
                fp_pop2db (&dbtemp1, &dbtemp2); \
                fp_pushdb (dbop (dbtemp1, dbtemp2)); \
        } \
        else \
        { \
                fp_pop2sn (&sntemp1, &sntemp2); \
                fp_pushsn (snop (sntemp1, sntemp2)); \
        } \
}
#else
void fp_dobinary (fpreal64_t (*dbop)(fpreal64_t,fpreal64_t),
                  fpreal32_t (*snop)(fpreal32_t,fpreal32_t))
{
        fpreal64_t dbtemp1, dbtemp2;
        fpreal32_t sntemp1, sntemp2;

        ResetRounding = TRUE;

        switch (FAReg.length)
        {
                case FP_REAL64:
                        fp_pop2db (&dbtemp1, &dbtemp2);
                        fp_pushdb (dbop (dbtemp1, dbtemp2));
                        break;
                case FP_REAL32:
                        fp_pop2sn (&sntemp1, &sntemp2);
                        fp_pushsn (snop (sntemp1, sntemp2));
                        break;
                default       :
                        /* Just pop 2 items and set FAReg to unknown. */
                        printf ("-W-EMUFPU: Warning - FAReg is undefined! (fp_dobinary)\n");
                        fp_drop2 ();
                        fp_pushdb (DUndefined);
                        FAReg.length = FP_UNKNOWN;
                        break;
        }
}
#endif


/* Do a binary floating point operation. */
int fp_binary2word (int (*dbop)(fpreal64_t,fpreal64_t),
                    int (*snop)(fpreal32_t,fpreal32_t))
{
        fpreal64_t dbtemp1, dbtemp2;
        fpreal32_t sntemp1, sntemp2;
        int result;

        ResetRounding = TRUE;

        switch (FAReg.length)
        {
                case FP_REAL64:
                        fp_pop2db (&dbtemp1, &dbtemp2);
                        result = dbop (dbtemp1, dbtemp2);
                        break;
                case FP_REAL32:
                        fp_pop2sn (&sntemp1, &sntemp2);
                        result = snop (sntemp1, sntemp2);
                        break;
                default       :
                        /* Just pop 2 items and set FAReg to unknown. */
                        printf ("-W-EMUFPU: Warning - FAReg is undefined! (fp_binary2word)\n");
                        fp_drop2 ();
                        result = FALSE;
                        break;
        }
        return result;
}


/* Do an unary floating point operation. */
#ifdef T4RELEASE
#define fp_dounary(dbop, snop) \
{ \
        fpreal64_t dbtemp; \
        fpreal32_t sntemp; \
 \
        ResetRounding = TRUE; \
 \
        if (FP_REAL64 == FAReg.length) \
        { \
                fp_popdb (&dbtemp); \
                fp_pushdb (dbop (dbtemp)); \
        } \
        else \
        { \
                fp_popsn (&sntemp); \
                fp_pushsn (snop (sntemp)); \
        } \
}
#else
void fp_dounary (fpreal64_t (*dbop)(fpreal64_t), fpreal32_t (*snop)(fpreal32_t))
{
        fpreal64_t dbtemp;
        fpreal32_t sntemp;

        ResetRounding = TRUE;

        switch (FAReg.length)
        {
                case FP_REAL64:
                        fp_popdb (&dbtemp);
                        fp_pushdb (dbop (dbtemp));
                        break;
                case FP_REAL32:
                        fp_popsn (&sntemp);
                        fp_pushsn (snop (sntemp));
                        break;
                default       :
                        /* Just pop 2 items and set FAReg to unknown. */
                        printf ("-W-EMUFPU: Warning - FAReg is undefined! (fp_dounary)\n");
                        fp_drop ();
                        fp_pushdb (DUndefined);
                        FAReg.length = FP_UNKNOWN;
                        break;
        }
}

#endif

void reschedule (void);

/* ---- T9000 fpstall / fpldall (ISM 11.12.3, table 11.26) ----------------
 * Seven-word structure: FPstatusReg @0, FPAreg @1-2, FPBreg @3-4,
 * FPCreg @5-6 (single precision occupies the lower-addressed word).
 * The real T9000 status word layout is not modelled bit-for-bit; this is a
 * self-consistent local encoding (lengths, rounding mode, FP_Error) so a
 * context switch round-trips the FPU state faithfully under emulation. */
static uint32_t t9k_fp_lenc (int length)
{
        if (length == FP_REAL32) return 1;
        if (length == FP_REAL64) return 2;
        return 0;
}
static int t9k_fp_ldec (uint32_t code)
{
        if (code == 1) return FP_REAL32;
        if (code == 2) return FP_REAL64;
        return FP_UNKNOWN;
}
static void t9k_fp_store_reg (uint32_t ptr, REAL *r)
{
        if (r->length == FP_REAL64)
        {
                writeword (ptr,     (uint32_t)(r->u.db.bits & 0xFFFFFFFFu));
                writeword (ptr + 4, (uint32_t)(r->u.db.bits >> 32));
        }
        else
        {
                writeword (ptr,     r->u.sn.bits);
                writeword (ptr + 4, 0);
        }
}
static void t9k_fp_load_reg (uint32_t ptr, REAL *r, int length)
{
        r->length = length;
        if (length == FP_REAL64)
                r->u.db.bits = ((uint64_t)word (ptr + 4) << 32) | word (ptr);
        else
                r->u.sn.bits = word (ptr);
}


/* Update time-of-day. */
void update_tod (struct timeval *tp)
{
        int rc;

        rc = gettimeofday (tp, (void *)0);
        if (rc < 0)
        {
                printf ("-W-EMU414: Failed to get time value.\n");

                *tp = LastTOD;
                tp->tv_usec++;
                if (0 == tp->tv_usec)
                        tp->tv_sec++;
        }
}

#define C_UNKNOWN -1
#define C_POKE  0
#define C_PEEK  1
#define C_BOOT  2

static uint32_t BootLink = 0;
/*
 * Boot-protocol state.
 *
 * This used to be one file-scope `CtrlByte` shared by every link channel.
 * The state machine reads a control byte, then treats following bytes as
 * payload until chan->Length runs out - so it only sets chan->Address =
 * MemStart on the packet's *first* call.  With one global, a channel that
 * entered handleboot() while the variable was still set from another
 * channel's packet skipped that initialisation entirely and ran with
 * chan->Address = 0: `UpdateWdescReg(0 | ProcPriority)` gave WPtr = 0, IPtr
 * was never moved off 0, and the node wild-jumped to address 0 the moment it
 * started executing.
 *
 * This was found while chasing the 16-node boot failure and is NOT its
 * cause - that turned out to be the Link0In read shim, see word_int().
 * It is a latent bug on its own account, so it is fixed here: the state
 * belongs to the channel, and now lives in Channel.
 */

int channel_sendP (Channel *chan, u_char *data, int ndata, int doWait);

int handleboot (Channel *chan, u_char *data, int ndata)
{
        uint32_t address, value;
        Channel *outchan;

        EMUDBG2 ("-I-EMUDBG: Handle boot, control byte = %d.\n", chan->CtrlByte);

        if (C_UNKNOWN == chan->CtrlByte)
        {
                chan->CtrlByte = data[0];
                chan->Address = MemStart;
                if (C_POKE == chan->CtrlByte)
                        chan->Length = 8;
                else if (C_PEEK == chan->CtrlByte)
                        chan->Length = 4;
                else
                {
                        chan->Length = chan->CtrlByte;
                        chan->CtrlByte = C_BOOT;
                        BootLink = chan->LinkAddress;
                }
                EMUDBG2 ("-I-EMUDBG: Control byte #%02X.\n", chan->CtrlByte);
                data++; ndata--;
        }
        if (chan->Length)
        {
                int len = ndata;
                if (chan->Length < len)
                        len = chan->Length;
                writebytes_int (chan->Address, data, len);
                chan->Address += len; chan->Length -= len;
        }
        if (0 == chan->Length)
        {
                switch (chan->CtrlByte)
                {
                        case C_POKE:
                                address = word_int (MemStart);
                                value   = word_int (MemStart + 4);
                                writeword_int (address, value);
                                chan->CtrlByte = C_UNKNOWN;
                                break;
                        case C_PEEK:
                                address = word_int (MemStart);
                                value = word_int (address);
                                data[0] = value & 255; value >>= 8;
                                data[1] = value & 255; value >>= 8;
                                data[2] = value & 255; value >>= 8;
                                data[3] = value & 255;
                                outchan = &Link[chan->linkno].Out;
                                ndata = 4;
                                if (channel_sendP (outchan, data, ndata, TRUE) != 4)
                                {
                                        printf ("-E-EMU414: Failed to send %d bytes on Link%dOut @ handleboot()\n", ndata, outchan->linkno);
                                        handler (-1);
                                }
                                chan->CtrlByte = C_UNKNOWN;
                                break;
                        case C_BOOT:
                                UpdateWdescReg (chan->Address | ProcPriority);
                                chan->CtrlByte = C_UNKNOWN;
                                return 0;
                }
        }
        return 1;
}

int channel_ready (Channel *chan)
{
        struct nn_pollfd pfd[1];
        int ret;

        EMUDBG2 ("-I-EMUDBG: ChannelReady. Link%dIn ready ?\n", chan->linkno);

        if (chan->npend)        /* left over from the last packet */
        {
                ret = 1;
                goto Exit;
        }
        if (chan->schbuf)
        {
                ret = 0 != SCHLength(chan);
                goto Exit;
        }

        pfd[0].fd = chan->sock;
        pfd[0].events = NN_POLLIN;
        pfd[0].revents = 0;

        EMUDBG2 ("-I-EMUDBG: Polling Link%dIn.\n", chan->linkno);

        ret = nn_poll (pfd, 1, LTO_POLL);
        if (-1 == ret) /* error */
        {
                printf ("-E-EMU414: Failed polling Link%dIn (%s)\n", chan->linkno, nn_strerror (nn_errno ()));
                handler (-1);
        }
Exit:
        if (0 == ret) /* timeout */
        {
                MSGDBG2 ("-I-EMUDBG: Link%dIn timeout.\n", chan->linkno);
                return 1;
        }

        return 0;
}

int channel_recvP (Channel *chan, u_char *data, int doWait)
{
        int ret;

        if (chan->schbuf)
        {
                if (0 == SCHLength(chan))
                {
                        errno = EAGAIN;
                        ret = -1;
                }
                else
                {
                        ret = SCHLength(chan);
                        memcpy (data, &chan->schbuf[SCH_DATA], ret);
                        if (t9k.enabled && t9k.net_on)
                        {
                                /* the sender's ready cycle, beside the data
                                 * and ordered with it by the length word */
                                uint64_t ready;

                                memcpy (&ready, &chan->schbuf[SCH_STAMP], 8);
                                t9k_net_received (chan->linkno, ready, (uint32_t)ret);
                        }
                        SCHSetLength(chan, 0);
                        t4_wake_node_id(chan->peer);
                }
        }
        else if (t9k.enabled && t9k.net_on)
        {
                /* T4_T9K_NET: each packet leads with its sender's ready
                 * cycle (see t9k_net_received) */
                u_char tbuf[8 + MAX_DATA];
                uint64_t ready;

                ret = nn_recv (chan->sock, tbuf, sizeof tbuf, doWait ? 0 : NN_DONTWAIT);
                if (ret >= 8)
                {
                        memcpy (&ready, tbuf, 8);
                        ret -= 8;
                        memcpy (data, tbuf + 8, ret);
                        t9k_net_received (chan->linkno, ready, (uint32_t)ret);
                }
        }
        else
                ret = nn_recv (chan->sock, data, MAX_DATA, doWait ? 0 :NN_DONTWAIT);
        if (-1 == ret)
        {
                if ((EAGAIN == errno) && !doWait)
                        return -1;

                printf ("-E-EMU414: channel_recvP: Receive failed on Link%dIn (%s)\n",
                        chan->linkno,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        MSGDBG4 ("-I-EMUDBG: channel_recvP: Received %d bytes on Link%dIn (#%08X).\n",
                ret,
                chan->linkno,
                chan->LinkAddress);
        return ret;
}

int channel_recvmemP (Channel *chan, u_char *data, int doMemWrite, int doWait)
{
        int ret, n;
        u_char *buf;

        /* A link is a byte stream: an in takes exactly the bytes it asked
         * for, wherever the sender's packets happen to break.  The part of a
         * packet past the end of the in waits in pend for the next in -
         * delivering the whole packet would write past the in's buffer. */
        if (doMemWrite && (chan->npend || chan->Length < MAX_DATA))
        {
                if (0 == chan->npend)
                {
                        ret = channel_recvP (chan, chan->pend, doWait);
                        if (ret < 0)
                                return ret;
                        chan->npend = ret;
                        chan->pendoff = 0;
                }
                n = chan->npend < (int)chan->Length ? chan->npend
                                                    : (int)chan->Length;
                writebytes_int (chan->Address, chan->pend + chan->pendoff, n);
                chan->pendoff += n; chan->npend -= n;
                chan->Address += n; chan->Length -= n;
                return n;
        }

        buf = doMemWrite ? memrange (chan->Address, data, MAX_DATA) : data;
        ret = channel_recvP (chan, buf, doWait);
        if (ret < 0)
                return ret;

        if (doMemWrite)
        {
                if (buf == data)
                        writebytes_int (chan->Address, data, ret);
                else
                        InvalidateRange (chan->Address, ret);
                chan->Address += ret; chan->Length -= ret;
        }
        return ret;
}


int channel_sendP (Channel *chan, u_char *data, int ndata, int doWait)
{
        int ret;

        ret = 0;
        if (chan->schbuf)
        {
                if (SCHLength(chan))
                {
                        errno = EAGAIN;
                        ret = -1;
                }
                else
                {
#ifndef NDEBUG
                        if (ndata > MAX_DATA)
                        {
                                printf ("-E-EMU414: channel_sendP: schbuf[] overflow! (%d).\n", ndata);
                        }
#endif
                        memcpy (&chan->schbuf[SCH_DATA], data, ndata);
                        if (t9k.enabled && t9k.net_on)
                        {
                                uint64_t ready = t9k_net_ready (chan->linkno, (uint32_t)ndata);

                                memcpy (&chan->schbuf[SCH_STAMP], &ready, 8);
                                t9k_net_sent (chan->linkno, ready, (uint32_t)ndata);
                        }
                        SCHSetLength(chan, ndata);
                        ret = ndata;
                        t4_wake_node_id(chan->peer);
                }
        }
        else if (t9k.enabled && t9k.net_on)
        {
                u_char tbuf[8 + MAX_DATA];
                uint64_t ready = t9k_net_ready (chan->linkno, (uint32_t)ndata);

                memcpy (tbuf, &ready, 8);
                memcpy (tbuf + 8, data, ndata);
                ret = nn_send (chan->sock, tbuf, 8 + ndata, doWait ? 0 : NN_DONTWAIT);
                if (ret == 8 + ndata)
                {
                        ret = ndata;
                        t9k_net_sent (chan->linkno, ready, (uint32_t)ndata);
                }
        }
        else
                ret = nn_send (chan->sock, data, ndata, doWait ? 0 : NN_DONTWAIT);

        if (-1 == ret)
        {
                if ((EAGAIN == errno) && !doWait)
                        return -1;

                printf ("-E-EMU414: channel_sendP: Send failed on Link%dOut (%s).\n",
                        chan->linkno,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        if (ret != ndata)
        {
                printf ("-E-EMU414: channel_sendP: Failed to send %d bytes on Link%dOut (%s).\n",
                        ndata,
                        chan->linkno,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        MSGDBG4 ("-I-EMUDBG: channel_sendP: Sent %d bytes on Link%dOut (#%08X).\n",
                ndata,
                chan->linkno,
                chan->LinkAddress);
        return ndata;
}

int channel_sendmemP (Channel *chan, int doWait)
{
        u_char data[MAX_DATA];
        int ndata;
        u_char *buf;

        ndata = MAX_DATA;
        if (chan->Length < MAX_DATA)
                ndata = chan->Length;

        buf = bytes_int (chan->Address, data, ndata);
        chan->Address += ndata; chan->Length -= ndata;
        return channel_sendP (chan, buf, ndata, doWait);
}

/* Receive at most MAX_DATA bytes without waiting. */
int Precv_channel (Channel *chan, uint32_t address, uint32_t len)
{
        u_char data[MAX_DATA];

        if (len > MAX_DATA)
                return 1;

        chan->Address = address;
        chan->Length  = len;
        PROFILE(chan->IOBytes += len);
        return channel_recvmemP (chan, data, TRUE, FALSE) < 0;
}

/* Send at most MAX_DATA bytes without waiting. */
int Psend_channel (Channel *chan, uint32_t address, uint32_t len)
{
        if (len > MAX_DATA)
                return 1;

        chan->Address = address;
        chan->Length  = len;
        PROFILE(chan->IOBytes += len);
        return channel_sendmemP (chan, FALSE) < 0;
}

#define recv_channel(c,a,n)     1
#define send_channel(c,a,n)     Psend_channel(c,a,n)


void alt_channel(Channel *chan)
{
        uint32_t linkWdesc, linkWPtr, linkPtr, altState;
        linkWdesc = word (chan->LinkAddress);
	/* Ready. */
        linkWPtr  = GetDescWPtr(linkWdesc);
	altState = linkPtr = word (index (linkWPtr, State_s));
        MSGDBG2 ("-I-EMUDBG: Link(1): Memory address/ALT state=#%08X.\n", altState);
	if ((altState & 0xfffffffc) == MostNeg)
	{
	        /* ALT guard test - not ready to communicate. */
                MSGDBG ("-I-EMUDBG: Link(2): ALT guard test - not ready to communicate.\n");

	        /* The alt is waiting. Reschedule it? */
		if (altState != Ready_p)
		{
		        /* The alt has not already been rescheduled. */
#ifdef EMUDEBUG
                        if (msgdebug || emudebug)
                        {
                                printf ("-I-EMUDBG: Link(3): ALT state=Ready_p.\n");
                                printf ("-I-EMUDBG: Link(4): Reschedule ALT process (Wdesc=#%08X, IPtr=#%08X).\n",
                                        linkWdesc,
                                        word (index (linkWPtr, Iptr_s)));
                        }
#endif
                        /* NB. HiPrio process executes J between ENBC and ALTWT.
                         *     D_check() calls linkcomms().
                         * if (Wdesc == linkWdesc)
                         * {
                         *        printf ("-E-EMU414: schedule Wdesc=#%08X is running.\n", Wdesc);
                         *         handler (-1);
                         * }
                         */
                        /* Mark channel control word */
                        writeword_int (chan->LinkAddress, IdleProcess_p);

			writeword (index (linkWPtr, State_s), Ready_p);
                        if (Waiting_p == altState)
                                schedule (linkWdesc);
                }
        }
        else
        {
	        /* Ready. */
                MSGDBG ("-I-EMUDBG: Link(5): Ready, communicate.\n");
        }
}

#define SCH_POLL        50
#define SCH_POLL_IDLE   1000

int linkcomms (char *where, int doBoot, int timeOut)
{
        struct nn_pollfd pfd[8];
        int npfd;
        uint32_t linkWdesc;
        Channel *channels[8];
        int vlk[8];             /* >= 0: VCP link (T9000 virtual mode) */
        u_char data[MAX_DATA];
        int ndata;
        int i, ret;
        int poll;
#ifdef EMUDEBUG
        char chnames[128], tmp[16];

        chnames[0] = '\0';
#endif

        if (nodeid < 0 && !t4cpu->s_loopback)
                return 0;
        /* T9000 control links (t9000_ctl.c): a Run or Reboot command ends
         * a boot wait */
        if (t4cpu->s_ctl_links && (t9k_ctl_poll () & T9K_CTL_START))
                return 1;
        for (i = 0; i < 8; i++)
                vlk[i] = -1;

        MSGDBG2 ("-I-EMUDBG: Link comms %s.\n", where);
        npfd = 0;
        poll = TRUE;
        for (i = 0; i < 4; i++)
        {
                if ((0 == i) && serve) /* skip Host link */
                        continue;
                /* a link put in virtual mode (a CPoke during a control-link
                 * boot) is the VCP's, not a byte-stream boot link */
                if (doBoot && t9k_link_virtual (i))
                        continue;

                if (!doBoot && t9k_link_virtual (i))
                {
                        /* T9000 virtual-channel mode: the link carries VCP
                         * packets, which may arrive at any time. */
                        if (t9k_link_listening (i) &&
                            (Link[i].In.schbuf || Link[i].In.sock >= 0))
                        {
                                pfd[npfd].fd = Link[i].In.sock;
                                pfd[npfd].events = NN_POLLIN;
                                pfd[npfd].revents = 0;
                                vlk[npfd] = i;
                                channels[npfd++] = &Link[i].In;
                        }
                        if (t9k_link_tx_pending (i) &&
                            (Link[i].Out.schbuf || Link[i].Out.sock >= 0))
                        {
                                pfd[npfd].fd = Link[i].Out.sock;
                                pfd[npfd].events = NN_POLLOUT;
                                pfd[npfd].revents = 0;
                                vlk[npfd] = i;
                                channels[npfd++] = &Link[i].Out;
                                poll = FALSE;
                        }
                        continue;
                }

                linkWdesc = word (Link[i].In.LinkAddress);
                /* IdleProcess_p has WPtr == NotProcess_p.  Polling it and
                 * then schedule()'ing the result enqueues MinInt. */
                if (doBoot || (GetDescWPtr(linkWdesc) != NotProcess_p))
                {
                        /* Select only BootLink */
                        if (doBoot && BootLink)
                        {
                                if (Link[i].In.LinkAddress != BootLink)
                                        continue;
                        }
                        pfd[npfd].fd = Link[i].In.sock;
                        pfd[npfd].events = NN_POLLIN;
                        pfd[npfd].revents = 0;
                        channels[npfd++] = &Link[i].In;
#ifdef EMUDEBUG
                        if (msgdebug || emudebug)
                        {
                                sprintf (tmp, " Link%dIn", i);
                                strcat (chnames, tmp);
                        }
#endif
                        poll = poll && (0 == Link[i].In.Length);
                }
                if (doBoot)
                        continue;

                linkWdesc = word (Link[i].Out.LinkAddress);
                if ((GetDescWPtr(linkWdesc) != NotProcess_p) && Link[i].Out.Length)
                {
                        pfd[npfd].fd = Link[i].Out.sock;
                        pfd[npfd].events = NN_POLLOUT;
                        pfd[npfd].revents = 0;
                        channels[npfd++] = &Link[i].Out;
#ifdef EMUDEBUG
                        if (msgdebug || emudebug)
                        {
                                sprintf (tmp, " Link%dOut", i);
                                strcat (chnames, tmp);
                        }
#endif
                        poll = FALSE;
                }
        }
        if (0 == npfd)
        {
                MSGDBG ("-I-EMUDBG: No active channels.\n");
                /* Not a deadlock: a sender may still be about to fill a
                 * buffer.  Returning 0 here is what fired "stopped no
                 * Link/Process/Timer activity" the moment idle polling
                 * dropped below a second. */
                if (doBoot)
                {
                        /* only the control links can boot this node now */
                        if (t4cpu->s_ctl_links && timeOut)
                                t9k_ctl_wait (timeOut);
                        return 0;
                }
                if (timeOut && t4cpu->s_ctl_links)
                        t9k_ctl_wait (timeOut);     /* woken by a command */
                else if (timeOut)
                        usleep (timeOut * 1000);
                /* A stand-alone node (T4_LINK_LOOPBACK) has nobody
                 * outside it to send. */
                return nodeid >= 0;
        }
#ifdef EMUDEBUG
        if (msgdebug || emudebug)
        {       printf ("-I-EMUDBG: Channels:%s\n", chnames);
                printf ("-I-EMUDBG: Checking %d channels, timeout = %dms%s.\n",
                        npfd,
                        timeOut,
                        poll ? " (poll)" : "");
        }
#endif
        /* Bytes left over from a packet are ready without the transport. */
        ret = 0;
        for (i = 0; i < npfd; i++)
                if ((pfd[i].events & NN_POLLIN) && channels[i]->npend)
                {
                        pfd[i].revents = NN_POLLIN;
                        ret++;
                }
        if (ret)
                ;
        else if (channels[0]->schbuf) /* shared channels? */
        {
                timeOut *= 1000;
                do
                {
                        ret = 0;
                        for (i = 0; i < npfd; i++)
                        {
                                int doadd = FALSE;
                                if (pfd[i].events & NN_POLLIN)
                                        doadd = 0 != SCHLength(channels[i]);
                                else
                                        doadd = 0 == SCHLength(channels[i]);
                                if (doadd)
                                {
                                        pfd[i].revents = pfd[i].events;
                                        ret++;
                                }
                        }
                        if (ret)
                                break;
                        if (timeOut)
                        {
                                if (t4cpu && t4cpu->idle) {
                                        int ms = timeOut / 1000;
                                        if (ms < 1) ms = 1;
                                        t4_node_idle_wait(t4cpu, ms);
                                        timeOut = 0;
                                        continue;
                                } else {
                                        int slice = (timeOut >= SCH_POLL_IDLE) ? SCH_POLL_IDLE : SCH_POLL;
                                        usleep (slice); timeOut -= slice;
                                        if (timeOut < 0) timeOut = 0;
                                }
                        }
                } while ((0 == ret) && (timeOut));
        }
        else
        {
                ret = nn_poll (pfd, npfd, timeOut);
        }
        if (0 == ret) /* timeout */
        {
                MSGDBG ("-I-EMUDBG: Comms timeout.\n");
                return (doBoot || nodeid < 0) ? 0 : 1;
        }
        if (-1 == ret) /* error */
        {
                printf ("-E-EMU414: Failed polling Links (%s)\n", nn_strerror (nn_errno ()));
                handler (-1);
        }
        for (i = 0; i < npfd; i++)
        {
                int revents;

                revents = 0;
                ndata = 0;
                if (0 == pfd[i].revents)
                        continue;

                if (vlk[i] >= 0)
                {
                        if (pfd[i].revents & NN_POLLIN)
                                t9k_link_rx (vlk[i]);
                        else
                                t9k_link_tx (vlk[i]);
                        continue;
                }

                if (pfd[i].revents & NN_POLLIN)
                {
                        revents = NN_POLLIN;
                        if (doBoot || channels[i]->Length)
                        {
                                ndata = ret = channel_recvmemP (channels[i], data, !doBoot, TRUE);
                                if (doBoot)
                                {
                                        if (0 == handleboot (channels[i], data, ret))
                                        {
                                                CReg = BootLink;
                                                return 1;
                                        }
                                        return 0;
                                }
                        }
#ifdef EMUDEBUG
                        else if (msgdebug || emudebug)
                                printf ("-I-EMUDBG: Polled on Link%dIn (#%08X).\n", channels[i]->linkno, channels[i]->LinkAddress);
#endif
                }
                else if (pfd[i].revents & NN_POLLOUT)
                {
                        revents = NN_POLLOUT;
                        ndata = channel_sendmemP (channels[i], TRUE);
                }
                /* Still processing message? */
                if (channels[i]->Length)
                        continue;

                linkWdesc = word (channels[i]->LinkAddress);
                if ((0 == ndata) && (NN_POLLIN == revents))
                {
                        alt_channel (channels[i]);
                }
                else
                {
                        reset_channel (channels[i]->LinkAddress);
#ifndef NDEBUG
                        if (Wdesc == linkWdesc)
                        {
                                printf ("-E-EMU414: schedule Wdesc=#%08X is running.\n", Wdesc);
                                handler (-1);
                        }
#endif
                        schedule (linkWdesc);
                }
        }
        return 1;
}


/* Close link channels */
void close_channels (void)
{
        int i;
        int nsocks;

        if (SharedLinks)
        {
                shlink_detach (SharedLinks);
                SharedLinks = NULL; SharedEvents = NULL;
                if (0 == nodeid)
                        shlink_free ();

                return;
        }

        nsocks = 0;
        for (i = 0; i < 4; i++)
        {
                if (-1 != Link[i].Out.sock)
                {
                        nsocks++;
                        break;
                }
                else if (-1 != Link[i].In.sock)
                {
                        nsocks++;
                        break;
                }
        }

        if (0 == nsocks)
                return;

        /* see comment for NN_LINGER at
         *      https://nanomsg.org/v1.1.5/nn_setsockopt.html
         */

        sleep (1);

        for (i = 0; i < 4; i++)
                if (-1 != Link[i].Out.sock)
                        nn_close (Link[i].Out.sock);

        for (i = 0; i < 4; i++)
                if (-1 != Link[i].In.sock)
                        nn_close (Link[i].In.sock);
}


/* Reset a link channel */
Channel *reset_channel (uint32_t addr)
{
        Channel *chan;
        int theLink;


        /* Reset channel control word. */
        writeword (addr, NotProcess_p);

        chan = (Channel *)0;
        theLink = TheLink(addr);
        if (IsLinkIn(addr))
        {
                chan = &Link[theLink].In;
        }
        else if (IsLinkOut(addr))
                chan = &Link[theLink].Out;

        if (!chan)
                return NULL;

        chan->Address = MostNeg;
        chan->Length  = 0;
        chan->CtrlByte = C_UNKNOWN;   /* a reset abandons any part-read packet */

        return chan;
}

/* Single-node link loopback for exercising links without a network:
 * T4_LINK_LOOPBACK="1-2,3" wires Link1Out to Link2In and Link2Out to
 * Link1In, and Link3Out back to Link3In.  Each link In gets a one-packet
 * mailbox, as in the in-process fabric. */
static int t4_loopback_peer[4] = { -1, -1, -1, -1 };

static void init_link_loopback (void)
{
        const char *e = getenv ("T4_LINK_LOOPBACK");
        int i, a, b, n;

        for (i = 0; i < 4; i++)
                t4_loopback_peer[i] = -1;
        if (nodeid >= 0 || !e || !*e)
                return;
        while (*e)
        {
                if (1 == sscanf (e, "%d%n", &a, &n) && a >= 0 && a < 4)
                {
                        e += n;
                        b = a;
                        if ('-' == *e && 1 == sscanf (e + 1, "%d%n", &b, &n) && b >= 0 && b < 4)
                                e += 1 + n;
                        t4_loopback_peer[a] = b;
                        t4_loopback_peer[b] = a;
                }
                while (*e && ',' != *e)
                        e++;
                if (',' == *e)
                        e++;
        }
        if (!t4cpu->s_loopback)
                t4cpu->s_loopback = calloc (4, SCH_SIZE);
        if (!t4cpu->s_loopback)
                handler (-1);
}

/* Open a link channel */
void open_channel (uint32_t addr)
{
        open_link_channel (reset_channel (addr), addr, TheLink(addr), IsLinkIn(addr));
}

/* Mailbox slot of a node link in shared memory: Link0-3 In at 4-7,
 * CLink0-1 In (network-map links 4 and 5) at 0-1; t4_fabric_inbox() is
 * the threaded fabric's equivalent. */
#define LINK_SLOT(l)    ((l) < 4 ? 4 + (l) : (l) - 4)

/* Wire one end of a link as the network map says: theLink 0-3 is a data
 * link, 4-5 a control link (t9000_ctl.c, which passes addr 0). */
void open_link_channel (Channel *chan, uint32_t addr, int theLink, int chanIn)
{
        int othernode, otherlink;
        int ret;
        u_char *nodeBase;

        chan->Address = MostNeg;
        chan->Length  = 0;
        chan->LinkAddress = addr;
        chan->linkno = theLink;
        chan->url[0] = '\0';
        chan->sock = -1;
        chan->schbuf = NULL;
        chan->IOBytes = 0;
        chan->CtrlByte = C_UNKNOWN;
        chan->peer = -1;
        chan->npend = 0;
        chan->pendoff = 0;

        if (serve && 0 == theLink) /* host link */
                return;
        if (nodeid < 0 && t4cpu->s_loopback)
        {
                /* T4_LINK_LOOPBACK: a cable from LinkaOut to LinkbIn */
                int other = theLink < 4 ? t4_loopback_peer[theLink] : -1;

                if (other >= 0)
                        chan->schbuf = t4cpu->s_loopback +
                                       (chanIn ? theLink : other) * SCH_SIZE;
                return;
        }
        if (nodeid < 0) /* no Node ID */
                return;

        if (t4_fabric) {
                if (chanIn) {
                        chan->schbuf = t4_fabric_inbox(t4_fabric, nodeid, theLink);
                        if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink))
                                chan->peer = othernode;
                } else if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink)) {
                        /* the far end may be a router (t4_fabric.c) */
                        chan->schbuf = t4_fabric_inbox(t4_fabric, othernode, otherlink);
                        if (chan->schbuf)
                                chan->peer = othernode;
                }
                return;
        }

        if (sharedLinks ())
        {
                int sharedSize;
                int maxnode;

                maxnode = maxNodeID ();
                sharedSize = 8 * SCH_SIZE * (1 + maxnode);
                if (NULL == SharedLinks)
                {
                        if ((1 == nodeid) || (0 == maxnode))
                        {
                                SharedLinks = shlink_alloc (NetConfigName, sharedSize + SHLINK_EXTRA);
                                if (SharedLinks)
                                        memset (SharedLinks, 0, sharedSize + SHLINK_EXTRA);
                        }
                        else
                                SharedLinks = shlink_attach (NetConfigName, sharedSize + SHLINK_EXTRA);
                        if (NULL == SharedLinks)
                                handler (-1);
                        SharedEvents = SharedLinks + sharedSize;
                }
                if (chanIn)
                {
                        nodeBase = SharedLinks + (nodeid * 8 * SCH_SIZE);
                        chan->schbuf = nodeBase + LINK_SLOT(theLink) * SCH_SIZE;
                        if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink))
                                chan->peer = othernode;
                        if (verbose)
                                printf ("-I-EMU414: Link%dIn  at #%08lX %d:%d.\n",
                                        theLink,
                                        chan->schbuf - SharedLinks,
                                        nodeid, theLink);
                }
                else if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink))
                {
                        nodeBase = SharedLinks + (othernode * 8 * SCH_SIZE);
                        chan->schbuf = nodeBase + LINK_SLOT(otherlink) * SCH_SIZE;
                        chan->peer = othernode;
                        if (verbose)
                                printf ("-I-EMU414: Link%dIn  at #%08lX %d:%d.\n",
                                        theLink,
                                        chan->schbuf - SharedLinks,
                                        othernode, otherlink);
                }
                return;
        }
        /* a control link that is not in the map gets no socket */
        if (theLink >= 4 && 0 != connectedNetLink(nodeid, theLink, &othernode, &otherlink))
                return;
        if (chanIn)
        {
                strcpy (&chan->url[0], netLinkURL (nodeid, theLink));
                if ((chan->sock = nn_socket (AF_SP, NN_PULL)) < 0)
                {
                        printf ("-E-EMU414: Error - Cannot create socket for Link%dIn\n", theLink);
                        handler (-1);
                }
                if ((ret = nn_bind (chan->sock, chan->url)) < 0)
                {
                        printf ("-E-EMU414: Error - Cannot bind Link%dIn to %s\n", theLink, chan->url);
                        handler (-1);
                }
                if (verbose)
                        printf ("-I-EMU414: Link%dIn  at %s\n", theLink, chan->url);
        }
        else if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink)) /* only if connected to other node */
        {
                int send_timeout = 5000;

                strcpy (&chan->url[0], netLinkURL (othernode, otherlink));
                if ((chan->sock = nn_socket(AF_SP, NN_PUSH)) < 0)
                {
                        printf ("-E-EMU414: Error - Cannot create socket for Link%dOut (%s).\n", theLink, nn_strerror (nn_errno ()));
                        handler(-1);
                }
                if ((ret = nn_connect (chan->sock, chan->url)) < 0)
                {
                        printf ("-E-EMU414: Error - Cannot connect Link%dOut to %s (%s)\n", theLink, chan->url, nn_strerror (nn_errno ()));
                        handler (-1);
                }
                if ((ret = nn_setsockopt (chan->sock, NN_SOL_SOCKET, NN_SNDTIMEO, &send_timeout, sizeof(int))) < 0)
                {
                        printf ("-E-EMU414: Error - Cannot set SNDTIMEO on Link%dOut (%s)\n", theLink, nn_strerror (nn_errno ()));
                        handler (-1);
                }
                if (verbose)
                        printf ("-I-EMU414: Link%dOut at %s\n", theLink, chan->url);
        }
}

void print_fpreg (char *ident, char name, REAL *fpreg, int printempty)
{
        fpreal64_t r64;
        fpreal32_t r32;
        char tmp[32];

        /* Sync FP_Error with native FPU exceptions. */
        fp_syncexcept ();

        /* Here FP_Error is synchronized. */
        /* NativeFPU exceptions MAY or MAY NOT be cleared. */

#ifndef NDEBUG
        fp_chkexcept ("Enter print_fpreg ()");
#endif

        tmp[0] = '\0';
        if (fpreg->length == FP_REAL64)
        {
                r64 = fpreg->u.db;
                if (db_inf (r64.bits))
                        strcpy (tmp, "(inf)");
                else if (db_nan (r64.bits))
                        strcpy (tmp, "(nan)");
/*
                else
                        sprintf (tmp, "%.15le", r64.fp);
*/
                printf ("%sF%cReg          #%016" PRIx64 "   (%s)\n", ident, name, r64.bits, tmp);
        }
        else if (fpreg->length == FP_REAL32)
        {
                r32 = fpreg->u.sn;
                if (sn_inf (r32.bits))
                        strcpy (tmp, "(inf)");
                else if (sn_nan (r32.bits))
                        strcpy (tmp, "(nan)");
/*
                else
                        sprintf (tmp, "%.7e", r32.fp);
*/
                printf ("%sF%cReg                  #%08X   (%s)\n", ident, name, r32.bits, tmp);
        } 
        else if (printempty)
        {
                r64 = fpreg->u.db;
                printf ("%sF%cReg          #%016" PRIx64 "   (Empty)\n", ident, name, r64.bits);
        }

        fp_clrexcept ();

#ifndef NDEBUG
        fp_chkexcept ("Leave print_fpreg ()");
#endif
}

/* Print processor state. */
void processor_state (void)
{
        printf ("-I-EMU414: Processor state\n");
        printf ("\tIPtr           #%08X\n", IPtr);
        printf ("\tWPtr           #%08X\n", WPtr);
        printf ("\tAReg           #%08X\n", AReg);
        printf ("\tBReg           #%08X\n", BReg);
        printf ("\tCReg           #%08X\n", CReg);
        printf ("\tError          %s\n", ReadError ? "Set" : "Clear");
        printf ("\tHalt on Error  %s\n", ReadHaltOnError ? "Set" : "Clear");
        if (IsT800 || IsTVS)
        {
                print_fpreg ("\t", 'A', &FAReg, 1);
                print_fpreg ("\t", 'B', &FBReg, 1);
                print_fpreg ("\t", 'C', &FCReg, 1);
                fp_syncexcept ();
                printf ("\tFP_Error       %s\n", FP_Error ? "Set" : "Clear");
        }
        printf ("\tFPtr1 (Low     #%08X\n", FPtrReg[1]);
        printf ("\tBPtr1  queue)  #%08X\n", BPtrReg[1]);
        printf ("\tFPtr0 (High    #%08X\n", FPtrReg[0]);
        printf ("\tBPtr0  queue)  #%08X\n", BPtrReg[0]);
        printf ("\tTPtr1 (Timer   #%08X\n", TPtrLoc[1]);
        printf ("\tTPtr0  queues) #%08X\n", TPtrLoc[0]);
}

void save_dump (void)
{
        FILE *fout;
        unsigned int bytesWritten;

        fout = fopen ("dump", "wb");
        if (fout == NULL)
        {
                printf ("-E-EMU404: Error - failed to open dump file.\n");
                handler (-1);
        }
        bytesWritten = fwrite (mem, sizeof (u_char), MemSize, fout);
        if (bytesWritten != MemSize)
        {
                printf ("-E-EMU414: Error - failed to write dump file.\n");
                fclose (fout);
                unlink ("dump");
                handler (-1);
        }
        fclose (fout);
}

char *mnemonic(u_char icode, uint32_t oreg, uint32_t fpuentry, int onlymnemo)
{
        char *mnemo;
        char bad[16];
        static char str[32];

        mnemo = 0;
        if ((icode > 239) && (oreg != MostNeg))
        {
                if ((oreg == 0xab) && (fpuentry != MostNeg))
                {
                        if (fpuentry == 0x9c)
                                mnemo = "FPUCLRERR";
                        else if (fpuentry == 0x23)
                                mnemo = "FPUSETERR";
                        else if (fpuentry == 0x22)
                                mnemo = "FPURN";
                        else if (fpuentry <= MAX_FPUENTRIES)
                                mnemo = FpuEntries[fpuentry];
                        else
                        {
                                sprintf (bad, "--FPU%02X--", fpuentry);
                                mnemo = bad;
                        }
                }
                else if (oreg == 0x1ff)
                        mnemo = "START";
                else if (oreg == 0x17c)
                        mnemo = "LDDEVID";
                else if (oreg <= MAX_SECONDARIES)
                        mnemo = Secondaries[oreg];
                else if (MIN_COMBINATIONS <= oreg && oreg <= MAX_COMBINATIONS)
                        mnemo = Combinations[oreg - MIN_COMBINATIONS];
                else if (MIN_TVX <= oreg && oreg <= MAX_TVX)
                        mnemo = TvxOps[oreg - MIN_TVX];
                else if (MIN_T9KOPS <= oreg && oreg <= MAX_T9KOPS)
                        mnemo = T9kOps[oreg - MIN_T9KOPS];
                else if (oreg >= (uint32_t)-MAX_T9KNEGOPS)
                        mnemo = T9kNegOps[(uint32_t)-oreg];
                else if (oreg == 0xFFFFFFC0u)
                        mnemo = "NOP";
                else if (oreg == 0xFFFFFF7Cu)
                        mnemo = "LDPRODID";

                if (mnemo == NULL)
                {
                        sprintf (bad, "--%02X--", oreg);
                        mnemo = bad;
                }
                sprintf (str, "%s", mnemo);
                return str;
        }

        if (onlymnemo)
                return Primaries[icode >> 4];

        sprintf (str, "%-7s #%X", Primaries[icode >> 4], oreg);
        return str;
}

void init_memory (void)
{
#ifndef NDEBUG
        unsigned int i;

        for (i = 0; i < CoreSize; i += 4)
                writeword_int (MostNeg + i, InvalidInstr_p);

        for (i = 0; i < MemSize; i += 4)
                writeword_int (ExtMemStart + i, InvalidInstr_p);
#endif
        return;
}

void init_processor (void)
{
        int i, j;

        for (i = 0; i < MAX_ICACHE+1; i++)
        {
                Icache[i].ip  = IC_NOADDR;
                Icache[i].Pcode = 0x3ff;
        }

        if (!t4_fabric)
                SharedLinks = NULL;

        FPtrReg[0] = FPtrReg[1] = NotProcess_p;
        BPtrReg[0] = BPtrReg[1] = NotProcess_p;

        t9k_vcp_reset_node ();
        init_link_loopback ();

        /* M.Bruestle 15.2.2012 */
        open_channel (Link0In);
        open_channel (Link1In);
        open_channel (Link2In);
        open_channel (Link3In);

        open_channel (Link0Out);
        open_channel (Link1Out);
        open_channel (Link2Out);
        open_channel (Link3Out);

        IPtr = MemStart;
        CReg = Link0In;
        TPtrLoc[0] = NotProcess_p;
        TPtrLoc[1] = NotProcess_p;
        ClearInterrupt; /* XXX not required ??? */

        /* T9000 control system and CLink0-1, after the CPU's reset state */
        t9k_ctl_reset_node ();

        /* Init TOD. */
        LastTOD.tv_sec  = 0;
        LastTOD.tv_usec = 0;
        update_tod (&LastTOD);


#ifdef EMUDEBUG
        if (IsT800 || IsTVS)
#endif
        {
                fp_init ();
                FAReg.length = FP_UNKNOWN;
                FBReg.length = FP_UNKNOWN;
                FCReg.length = FP_UNKNOWN;
        }

        /* ErrorFlag is in an indeterminate state on power up. */

#ifdef T4PROFILE
        if (profiling)
                for (i = 0; i < 0x400; i++)
                {
                        instrprof[i] = 0;
                        for (j = 0; j < 0x400; j++)
                                combinedprof[i][j] = 0;
                }
#endif
        {
                static pthread_once_t once = PTHREAD_ONCE_INIT;
                pthread_once(&once, t4_fill_combinations);
        }
}

#define FLAG(x,y)       ((x) ? (y) : '-')


#ifdef EMUDEBUG
void checkWPtr (char *where, uint32_t wptr)
{
        if (wptr & ByteSelectMask)
        {
                if (emudebug)
                {
                        printf ("-W-EMU414: Warning - byte selector of WPtr should be zero! (%s)\n", where);
                        // handler (-1);
                }
        }
        if (NotProcess_p == wptr)
        {
                printf ("-E-EMU414: WPtr = NotProcess_p.\n");
                handler (-1);
        }
}

void checkWordAligned (char *where, uint32_t ptr)
{
        if (ptr & ByteSelectMask)
        {
                if (emudebug)
                {
                        printf ("-W-EMU414: Warning - byte selector of register should be zero! (%s)\n", where);
                        // handler (-1);
                }
        }
}
#endif

#ifdef T4PROFILE
static struct timeval StartTOD, EndTOD;
double ElapsedSecs;
#endif

u_short ProfileCode (u_char instrCode, uint32_t oprCode)
{
        u_short ret;

        ret = instrCode;
        if (0xF0 == instrCode)
        {
                if (0xab == oprCode)
                        ret = 0x300; /* XXX */
                else if (oprCode < 0x300)
                        ret = 0x100 + oprCode;
                else
                        /* Out-of-range OReg (negative-prefix secondaries, or
                         * pfix-built values like the kernel's 0x60F9 selth
                         * spelling): never fold these into combinations - the
                         * table is indexed by 10-bit codes and 0x100+oprCode
                         * would run off its end. */
                        ret = 0x3FF;
        }
        return ret;
}

/* Does the part -s414/-s425 selects have secondary instruction `op`?  The
 * T414 has the base set, its floating-point support (unpacksn, roundsn,
 * postnormsn, ldinf, cflerr) and fmul; the T425 adds dup, the move2d
 * group, crcword/crcbyte, bitcnt, the bit reversals, wsubdb, pop, the
 * timer enables, ldmemstartval, the j0 breakpoints, lddevid and start
 * (Norcroft xpuops.h; Compiler Writer's Guide D.4).  Neither has an FPU
 * or any T9000 instruction - gtu included, which t4 runs in other modes. */
static int t4xx_has_opr (int32_t op)
{
	switch (op) {
	case 0x11: case 0x14:
		return 0;
	case 0x63: case 0x6C: case 0x6D: case 0x71: case 0x72: case 0x73:
		return 1;
	case 0x5A: case 0x5B: case 0x5C: case 0x5D: case 0x5E:
	case 0x74: case 0x75: case 0x76: case 0x77: case 0x78: case 0x79:
	case 0x7A: case 0x7B: case 0x7C: case 0x7D: case 0x7E: case 0x81:
	case 0xB1: case 0xB2: case 0xB3: case 0xB4: case 0x17C: case 0x1FF:
		return t4_isa_strict == 425;
	}
	return op >= 0 && op <= 0x59;
}

/* Stop on a secondary the selected T4xx lacks, rather than run it */
static void t4xx_bad_opr (int32_t op)
{
	printf ("-E-EMU414: Error - #%X (%s) is not a T%d instruction\n",
	        (uint32_t)op, mnemonic (0xF0, op, AReg, 0), t4_isa_strict);
	maybe_log_nearest_sym (IPtr, "not-t4xx");
	processor_state ();
	handler (-1);
}

/* Next heartbeat_counter value at which run_insn_hooks() is due; 0 makes
 * the first instruction resolve the hooks' settings. */
static uint64_t insn_hook_next;

/* mainloop()'s instruction-count hooks, kept out of the per-instruction
 * path: only called once heartbeat_counter reaches insn_hook_next. */
static void run_insn_hooks (void)
{
	static uint64_t smp_every = 0, smp_next = 0;
	static uint64_t rec_every = 0;   /* 0 = unresolved, ~0 = off */
	static uint64_t rec_next = 0;
	/* T4_IPTR_SAMPLE=N: every N instructions, record IPtr into a ring;
	 * dumped at exit as a sorted histogram (hot-loop profiler). */
	{
		if (__builtin_expect(smp_every == 0, 0)) {
			extern void t4_iptr_sample_dump(void);
			const char *e = getenv("T4_IPTR_SAMPLE");
			smp_every = (e && *e) ? strtoull(e, NULL, 0) : ~0ULL;
			if (smp_every == 0)
				smp_every = ~0ULL;
			if (smp_every != ~0ULL)
				atexit(t4_iptr_sample_dump);
			smp_next = smp_every;
		}
		if (__builtin_expect(heartbeat_counter >= smp_next, 0)
		    && smp_every != ~0ULL) {
			extern uint32_t iptr_samples[8192];
			extern uint32_t iptr_sample_n;
			smp_next = heartbeat_counter + smp_every;
			iptr_samples[iptr_sample_n & 8191u] = IPtr;
			wptr_samples[iptr_sample_n & 8191u] = WPtr;
			iptr_sample_n++;
			if (iptr_sample_n == 0)
				iptr_sample_n = 8192; /* saturate wrap flag */
			if ((iptr_sample_n & 2047u) == 0) {
				uint32_t k;
				const char *dw;
				fprintf(stderr,
					"[LIVE-FRAME] IPtr=0x%08X WPtr=0x%08X\n",
					IPtr, WPtr);
				for (k = 0; k < 32; k++)
					fprintf(stderr, "W%-2u 0x%08X\n", k,
						word_int(WPtr + 4u * k));
				/* last write-watch hits (see writeword_int) */
				{
					extern uint32_t watch_ring[128][3];
					extern uint32_t watch_ring_n;
					uint32_t nback = watch_ring_n < 16u
					    ? watch_ring_n : 16u;
					for (k = 0; k < nback; k++) {
						uint32_t idx =
						    (watch_ring_n - nback + k)
						    & 127u;
						fprintf(stderr,
							"WW val=0x%08X "
							"IPtr=0x%08X hb=%u\n",
							watch_ring[idx][0],
							watch_ring[idx][1],
							watch_ring[idx][2]);
					}
				}
				/* T4_DUMP_WORDS=a[:n],b[:m],...: print n words
				 * starting at each address (default 1) */
				dw = getenv("T4_DUMP_WORDS");
				while (dw && *dw) {
					char *end;
					uint32_t a = (uint32_t)
					    strtoul(dw, &end, 0);
					uint32_t cnt = 1, j;
					if (end && *end == ':')
						cnt = (uint32_t)
						    strtoul(end + 1, NULL, 0);
					if (cnt > 256)
						cnt = 256;
					for (j = 0; j < cnt; j++)
						fprintf(stderr,
							"M 0x%08X 0x%08X\n",
							a + 4*j,
							word_int(a + 4*j));
					dw = strchr(dw, ',');
					if (dw)
						dw++;
				}
				fflush(stderr);
			}
		}
	}
	/* headless framebuffer video capture (see t4_fb_record_poll) */
	{
		if (__builtin_expect(rec_every == 0, 0)) {
			const char *e = getenv("T4_FB_RECORD");
			rec_every = (e && *e) ? strtoull(e, NULL, 0) : ~0ULL;
			if (rec_every == 0)
				rec_every = ~0ULL;
			rec_next = rec_every;
		}
		if (__builtin_expect(heartbeat_counter >= rec_next, 0)
		    && rec_every != ~0ULL) {
			rec_next = heartbeat_counter + rec_every;
			t4_fb_record_poll();
		}
	}
	insn_hook_next = smp_every != ~0ULL ? smp_next : ~0ULL;
	if (rec_every != ~0ULL && rec_next < insn_hook_next)
		insn_hook_next = rec_next;
}

void mainloop (void)
{
        /* The node's decode tables never move while it runs.  Through
         * t4cpu their pointers are reloaded after every guest store (a
         * store through u_char * may alias the cpu_t), so take them once.
         * The macros are restored after mainloop(). */
        InstrSlot *const mainloop_icache = Icache;
        ArgSlot   *const mainloop_acache = Acache;
#undef  Icache
#undef  Acache
#define Icache  mainloop_icache
#define Acache  mainloop_acache
        /* Instructions since the timer block last ran (see delayCount1). */
        int timer_tick = 0;
        /* Any optional per-instruction machinery in use; see below. */
        int rare_hooks;
        uint32_t temp, temp2;
        uint32_t otherWdesc, otherWPtr, otherPtr, altState;
        uint32_t PrevError;
        u_char pixel;
        fpreal32_t sntemp1, sntemp2;
        fpreal64_t dbtemp1, dbtemp2;
        REAL       fptemp;
        unsigned int islot, pslot;
	int i;
	int trace_fb_ldc_enabled;
	uint32_t stop_iptr;
	uint32_t stop_return;
	uint32_t stop_areg;
	int stop_areg_set;
	int stop_b_non_signext;
	uint32_t stop_b_lo;
	uint32_t stop_b_hi;
	int stop_b_range_set;
	uint32_t stop_w3_min;
	int production_quiet;

#ifdef EMUDEBUG
        fpreal32_t r32temp;
        int   printIPtr, instrBytes;
        int   asmLines;
        int   currFPInstr, prevFPInstr;
        char *mnemo;

        printIPtr   = TRUE;
        prevFPInstr = FALSE;
        instrBytes  = 0;
        asmLines    = 0;
#endif
	m2dSourceStride = m2dDestStride = m2dLength = Undefined_p;
	{
		const char *env = getenv("T4_TRACE_FB_LDC");
		trace_fb_ldc_enabled = env && *env && strcmp(env, "0") != 0;
	}
	{
		const char *env = getenv("T4_STOP_IPTR");
		stop_iptr = env && *env ? (uint32_t)strtoul(env, NULL, 0) : 0;
		env = getenv("T4_STOP_RETURN");
		stop_return = env && *env ? (uint32_t)strtoul(env, NULL, 0) : 0;
		env = getenv("T4_STOP_AREG");
		stop_areg_set = env && *env;
		stop_areg = stop_areg_set ? (uint32_t)strtoul(env, NULL, 0) : 0;
		env = getenv("T4_STOP_B_NON_SIGNEXT");
		stop_b_non_signext = env && *env && strcmp(env, "0") != 0;
		env = getenv("T4_STOP_B_RANGE");
		stop_b_range_set = env && *env &&
			sscanf(env, "%x,%x", &stop_b_lo, &stop_b_hi) == 2;
		env = getenv("T4_STOP_W3_MIN");
		stop_w3_min = env && *env ? (uint32_t)strtoul(env, NULL, 0) : 0;
	}

	fprintf(stderr, "[DEBUG: Entering mainloop, IPtr=0x%08X Wptr=0x%08X]\n", IPtr, WPtr);

	/* T9000 protection / PMI, once per node.  A CPU the control unit
	 * restarts (Run, Reboot: the only re-entry) keeps the PMI set-up
	 * (HRM table 8.10) and starts outside any P-process. */
	{
		static T4_THREAD_LOCAL int t9k_entered;

		if (t9k_entered)
			t9k_leave_p();
		else {
			t9k_protect_init();
			t9k_pmi_init();
			t9k_entered = 1;
		}
	}
	t9k_ctl_cpu_entry();

	/* Decode cache: opt-in (T4_DCACHE=1). Default off — correctness first. */
	{
		const char *de = getenv("T4_DCACHE");
		dcache_enabled = (de && *de && strtoul(de, NULL, 0) != 0) ? 1 : 0;
		dcache_init(&g_dcache);
		if (dcache_enabled)
			fprintf(stderr, "[DCACHE] Enabled (%d entries, %zuKB)\n",
				DCACHE_SIZE, sizeof(dcache_t) / 1024);
		else
			fprintf(stderr, "[DCACHE] Disabled (set T4_DCACHE=1 to enable)\n");
	}

	/* Pin hot loop to one CPU for stable L1/L2 residency (optional). */
	{
		const char *pin = getenv("T4_CPU_PIN");
		if (pin && *pin) {
#if defined(__linux__)
			cpu_set_t set;
			int cpu = (int)strtol(pin, NULL, 0);
			CPU_ZERO(&set);
			CPU_SET(cpu, &set);
			if (sched_setaffinity(0, sizeof(set), &set) == 0)
				fprintf(stderr, "[CPU] Pinned hot loop to CPU %d\n", cpu);
			else
				perror("[CPU] sched_setaffinity");
#endif
		}
	}
	/* Verify memory at watched address after BTL loading */
	{
	    const char *wa_env = getenv("T4_WATCH_ADDR");
	    if (wa_env) {
	        uint32_t wa = (uint32_t)strtoul(wa_env, NULL, 16);
	        fprintf(stderr, "[VERIFY_MEM] addr=0x%08X bytes: %02X %02X %02X %02X (mem idx 0x%X)\n",
	                wa, byte_int(wa), byte_int(wa+1), byte_int(wa+2), byte_int(wa+3),
	                wa & MemByteMask);
	        /* Also check raw mem array */
	        uint32_t idx = wa & MemByteMask;
	        fprintf(stderr, "[VERIFY_RAW] mem[0x%X]: %02X %02X %02X %02X\n",
	                idx, mem[idx], mem[idx+1], mem[idx+2], mem[idx+3]);
	    }
	}

	count1 = 0;
	count2 = 0;
	count3 = 0;
	timeslice = 0;
	Timers = TimersStop;

	/* Set product identity for LDDEVID instruction */
	if (Txxx == 414)      ProductIdentity = 0x00000000;  /* T414 */
	else if (Txxx == 800) ProductIdentity = 0x00000005;  /* T800-20 */
	else if (Txxx == 806) ProductIdentity = 0x00000806;  /* T806 (custom 200MHz) */
	else                  ProductIdentity = 0x00000000;

	/* T806: Custom 200MHz T800 variant. */
	speed_multiplier = 1;
	if (IsT806)
		fprintf(stderr, "[T806] T800-compatible @ 200MHz (LDDEVID=0x%03x)\n", ProductIdentity);

	/* Start timer thread once (node 0). Workers share it. */
	if (nodeid <= 0) {
		g_timer_bridge.pClockReg = ClockReg;
		g_timer_bridge.ptimeslice = &timeslice;
		g_timer_bridge.pTimers = &Timers;
		g_timer_bridge.TimersGo_val = TimersGo;
		g_timer_threaded = timer_bridge_start(&g_timer_bridge);
	} else
		g_timer_threaded = 1;
	/* update_time() adapts delayCount1 only when it runs the clock itself.
	 * The timer thread moves ClockReg every 100us, so polling the timer
	 * queues every 6 instructions (the starting value) bought nothing. */
	if (g_timer_threaded)
		delayCount1 = 63;
	TimerEnableHi = 1;
	TimerEnableLo = 1;

	/* Initialize debug config from environment variables.
	 * Only override p.c defaults if env var is explicitly set.
	 * debug_config.c collects all env var reads in one place. */
	{
		static debug_config_t dbg;
		debug_init(&dbg);

		/* These default to 0 in p.c — safe to set from dbg */
		if (dbg.outbyte_trace_enabled)   outbyte_trace_enabled = 1;
		if (dbg.mmio_trace_enabled)      mmio_trace_enabled = 1;
		if (dbg.early_printk_trace_enabled) early_printk_trace_enabled = 1;
		if (dbg.nearest_sym_enabled)     nearest_sym_enabled = 1;
		if (dbg.bootdbg_enabled)         bootdbg_enabled = 1;

		/* uart_console: default is 1 in p.c, only change if env var set */
		{
			const char *uc = getenv("T4_UART_CONSOLE");
			if (uc && *uc) uart_console_enabled = (int)strtoul(uc, NULL, 0) != 0;
		}
		/* uart_console_force overrides */
		{
			const char *ucf = getenv("T4_UART_CONSOLE_FORCE");
			if (ucf && *ucf && (int)strtoul(ucf, NULL, 0) != 0) {
				uart_console_enabled = 1;
				uart_trace_enabled = 0;
			}
		}

		/* link0: default is 1 in p.c */
		if (dbg.link0_trace_enabled) link0_trace_enabled = 1;

#ifdef T4_SDL_FB
		if (dbg.fb_console_enabled) fb_console_enabled = 1;
#endif
		{
			const char *bd = getenv("T4_BOOT_DUMP_START");
			if (bd && *bd) boot_dump_start_enabled = 1;
		}
	}

	/* Open UART log file — use timestamped name to avoid conflicts
	 * with concurrent T4 instances (e.g., test runners).  Once per
	 * process: a threaded node starting its mainloop later would
	 * otherwise truncate what node 0 has already logged. */
	if (!uart_log_file)
	{
		const char *uart_log_env = getenv("T4_UART_LOG");
		char uart_log_auto[64];
		const char *uart_log_path;

		if (uart_log_env && *uart_log_env) {
			uart_log_path = uart_log_env;
		} else {
			/* Generate timestamped filename: boot-{pid}.uart */
			snprintf(uart_log_auto, sizeof(uart_log_auto),
				 "boot-%d.uart", (int)getpid());
			uart_log_path = uart_log_auto;
			/* Also create/overwrite "boot.uart" symlink for convenience */
			unlink("boot.uart");
			symlink(uart_log_auto, "boot.uart");
		}
		uart_log_file = fopen(uart_log_path, "w");
		if (uart_log_file) {
			fprintf(stderr, "[UART] Logging to %s\n", uart_log_path);
			setbuf(uart_log_file, NULL);  /* Unbuffered for real-time logging */
		}
	}

        islot = MAX_ICACHE;
	PROFILE(update_tod (&StartTOD));
	production_quiet = !runtime_diagnostics_requested();
	if (production_quiet)
		fprintf(stderr, "[FAST] Quiet production decoder path enabled\n");
#ifndef EMUDEBUG
	{
		const char *env = getenv("T4_FAST_PRIMARY");
		/* The accelerated decoder is opt-in until the Linux boot
		 * differential suite proves full scheduler equivalence. */
		fast_primary_enabled = env && *env &&
			strtoul(env, NULL, 0) != 0;
		env = getenv("T4_FAST_BLOCK");
		/* Basic-block fusion is likewise opt-in: incorrect context
		 * state is worse than a slower, reproducible emulator. */
		fast_block_enabled = env && *env &&
			strtoul(env, NULL, 0) != 0;
	}
#endif

	/* Diagnostics, a stop breakpoint, the opt-in fast paths and decode
	 * cache, the T9000 grouper and the instruction-count hooks are all
	 * settled by now.  One loop-invariant test keeps every one of them off
	 * the common path; production runs pay a register test, not loads. */
	rare_hooks = !production_quiet || stop_iptr || dcache_enabled > 0 ||
	             t9k.enabled || t4_isa_strict ||
#ifndef EMUDEBUG
	             fast_block_enabled || fast_primary_enabled ||
#endif
	             (getenv("T4_IPTR_SAMPLE") && *getenv("T4_IPTR_SAMPLE")) ||
	             (getenv("T4_FB_RECORD") && *getenv("T4_FB_RECORD"));

	/*
	 * T9000 P-trap return: longjmp resumes here (after one-time init)
	 * with L IPtr/WPtr restored.  Clear OReg so prefix state is clean.
	 * timer_tick changed after setjmp(), so it is indeterminate here.
	 */
	if (setjmp(t9k_prot.jmp_env) != 0) {
		OReg = 0;
		timer_tick = 0;
		if (t9k_prot.debug)
			fprintf(stderr,
				"[T9K-PROT] resume L after trap IPtr=0x%08X cause=%u\n",
				IPtr, t9k_prot.last_trap);
	} else {
		t9k_prot.jmp_valid = 1;
	}

	while (1)
	{
                /* Fast-path: minimal overhead per instruction. */
                PrevError = ReadError;
                /* T4_T9K_NET: between instructions, a process the VCP
                 * holds joins the queue once this node's clock reaches the
                 * end of its transfer */
                if (unlikely(rare_hooks) && OReg == 0)
                        T9K_NET_POLL ();
		if (unlikely(rare_hooks && stop_iptr && IPtr == stop_iptr &&
		    (!stop_return || word_int(WPtr) == stop_return) &&
		    (!stop_areg_set || AReg == stop_areg) &&
		    (!stop_b_non_signext || (BReg != 0 && BReg != 0xFFFFFFFFu)) &&
		    (!stop_b_range_set || (BReg >= stop_b_lo && BReg <= stop_b_hi)) &&
		    (!stop_w3_min || word_int(index(WPtr, 3)) >= stop_w3_min))) {
			fprintf(stderr,
			        "[STOP_IPTR] IPtr=0x%08X WPtr=0x%08X "
			        "A=0x%08X B=0x%08X C=0x%08X O=0x%08X "
			        "W0=0x%08X W1=0x%08X W2=0x%08X W3=0x%08X "
			        "W25=0x%08X W26=0x%08X W27=0x%08X W28=0x%08X "
			        "W29=0x%08X W33=0x%08X W34=0x%08X W35=0x%08X "
			        "W36=0x%08X W37=0x%08X W38=0x%08X W53=0x%08X\n",
			        IPtr, WPtr, AReg, BReg, CReg, OReg,
			        word_int(index(WPtr, 0)), word_int(index(WPtr, 1)),
			        word_int(index(WPtr, 2)), word_int(index(WPtr, 3)),
			        word_int(index(WPtr, 25)), word_int(index(WPtr, 26)),
			        word_int(index(WPtr, 27)), word_int(index(WPtr, 28)),
			        word_int(index(WPtr, 29)), word_int(index(WPtr, 33)),
			        word_int(index(WPtr, 34)), word_int(index(WPtr, 35)),
			        word_int(index(WPtr, 36)), word_int(index(WPtr, 37)),
			        word_int(index(WPtr, 38)), word_int(index(WPtr, 53)));
			{
				uint32_t dump_base;
				int dump_i;

				dump_base = word_int(index(WPtr, 38));
				fprintf(stderr, "  M[W38]:");
				for (dump_i = 0; dump_i < 18; dump_i++)
					fprintf(stderr, " %08X", word_int(index(dump_base, dump_i)));
				fprintf(stderr, "\n");
				dump_base = word_int(index(WPtr, 35));
				fprintf(stderr, "  M[W35]:");
				for (dump_i = 0; dump_i < 32; dump_i++)
					fprintf(stderr, " %08X", word_int(index(dump_base, dump_i)));
				fprintf(stderr, "\n");
			}
			break;
		}

                /* Timer and periodic checks.
                 * With threaded timer, only queue schedule + rare I/O.
                 * Headless: skip SDL entirely (present is already no-op). */
                if (__builtin_expect(++timer_tick > delayCount1, 0))
                {
                        timer_tick = 0;
                        /* Another node asked this one to stop. */
                        if (SharedEvents && (nodeid >= 0) && SharedEvents[nodeid])
                                return;
                        if (g_timer_threaded) {
                                if (TimerEnableHi) schedule_timerq(HiPriority);
                                if (TimerEnableLo) schedule_timerq(LoPriority);
                        } else {
                                update_time();
                        }

                        {
                                /* Every 1024 timer ticks ≈ rare host I/O */
                                if (nodeid <= 0 && __builtin_expect((++t4cpu->slow_counter & 0x3FF) == 0, 0)) {
#ifdef T4_SDL_FB
                                        if (!display_headless) {
                                                display_pump();
                                                display_present();
                                        }
#endif
                                        t4net_poll_rx();
                                        if (quit) break;
                                }
                        }
                }

#ifndef EMUDEBUG
                /* Release block path for straight-line runs of simple one-byte
                 * primary ops. It avoids the legacy icache and giant switch for
                 * stack/local arithmetic code, while branches, prefixes, OPR,
                 * calls, scheduler/channel ops, and debug tracing stay on the
                 * full path. Cached bytes are rechecked before execution, so
                 * self-modifying writes naturally fall back and rebuild. */
                if (__builtin_expect(rare_hooks && fast_block_enabled && OReg == 0 &&
                    !t9k_prot.is_p &&   /* no P-mode checks in here */
                    !bootdbg_enabled && !cache_debug_trace_enabled &&
                    !iptr_trace_enabled && !sym_trace_active &&
                    !text_write_trace_enabled && !sk_areg_trace_enabled &&
                    !sk_addr_trace_enabled && !sk_store_trace_enabled &&
                    !wptr_trace_enabled, 0)) {
                        t4_fast_block_t *fb = &fast_blocks[IPtr & T4_FAST_BLOCK_MASK];
                        int valid = (fb->tag == IPtr && fb->len != 0);

                        if (valid) {
                                for (i = 0; i < fb->len; i++) {
                                        if (unlikely(code_byte(IPtr + (uint32_t)i) != fb->bytes[i])) {
                                                valid = 0;
                                                break;
                                        }
                                }
                        }
                        if (!valid) {
                                uint32_t scan_ip = IPtr;
                                fb->tag = IPtr;
                                fb->len = 0;
                                for (i = 0; i < T4_FAST_BLOCK_MAX_INSNS; i++) {
                                        uint8_t insn = code_byte(scan_ip);
                                        uint32_t op = insn & 0xf0;
                                        if (!t4_fast_block_op(op))
                                                break;
                                        fb->bytes[i] = insn;
                                        fb->ops[i].op = (u_char)op;
                                        fb->ops[i].imm = (u_char)(insn & 0x0f);
                                        fb->len++;
                                        scan_ip++;
                                }
                                valid = fb->len >= 2;
                        }
                        if (valid) {
                                for (i = 0; i < fb->len; i++) {
                                        uint32_t imm = fb->ops[i].imm;
                                        switch (fb->ops[i].op) {
                                        case 0x10: /* ldlp */
                                                CReg = BReg;
                                                BReg = AReg;
                                                AReg = index(WPtr, imm);
                                                break;
                                        case 0x30: /* ldnl */
                                                AReg = word(index(AReg, imm));
                                                break;
                                        case 0x40: /* ldc */
                                                CReg = BReg;
                                                BReg = AReg;
                                                AReg = imm;
                                                break;
                                        case 0x50: /* ldnlp */
                                                AReg = index(AReg, imm);
                                                break;
                                        case 0x70: /* ldl */
                                                CReg = BReg;
                                                BReg = AReg;
                                                AReg = word(index(WPtr, imm));
                                                break;
                                        case 0x80: /* adc */
                                                t4_overflow = FALSE;
                                                t4_carry = 0;
                                                AReg = t4_eadd32(AReg, imm);
                                                if (t4_overflow == TRUE)
                                                        SetError;
                                                break;
                                        case 0xb0: /* ajw */
                                                UpdateWdescReg(index(WPtr, (int32_t)imm) | ProcPriority);
                                                break;
                                        case 0xc0: /* eqc */
                                                AReg = (AReg == imm) ? true_t : false_t;
                                                break;
                                        case 0xd0: /* stl */
                                                writeword(index(WPtr, imm), AReg);
                                                AReg = BReg;
                                                BReg = CReg;
                                                break;
                                        case 0xe0: /* stnl */
                                                writeword(index(AReg, imm), BReg);
                                                AReg = CReg;
                                                break;
                                        }
                                }
                                IPtr += fb->len;
                                if (__builtin_expect((!PrevError && ReadError) &&
                                    (exitonerror || ReadHaltOnError), 0))
                                        goto FastPrimaryError;
                                continue;
                        }
                }

                /* Single-instruction fallback for common primary ops when a
                 * block is too short to cache.  Decode the complete PFIX/NFIX
                 * chain here: compiler-generated kernel code rarely fits its
                 * frame offsets and constants in a single nibble, and making
                 * only one-byte primaries fast leaves nearly all C code on the
                 * diagnostic-heavy legacy path. */
                if (__builtin_expect(rare_hooks && fast_primary_enabled && OReg == 0 &&
                    !t9k_prot.is_p &&   /* no P-mode checks in here */
                    !bootdbg_enabled && !cache_debug_trace_enabled &&
                    !iptr_trace_enabled && !sym_trace_active &&
                    !text_write_trace_enabled && !sk_areg_trace_enabled &&
                    !sk_addr_trace_enabled && !sk_store_trace_enabled &&
                    !wptr_trace_enabled && !trace_fb_ldc_enabled, 0)) {
	                        dcache_entry_t *decoded;
	                        uint32_t fast_ip;
	                        uint32_t fast_oreg;
                        uint32_t op;

	                        /* Share the same prefix decoder as T4_DCACHE.
	                         * This makes PFIX/NFIX operand construction a
	                         * single, testable implementation instead of a
	                         * second hand-written decoder. */
	                        decoded = dcache_lookup(&g_dcache, IPtr);
	                        if (!decoded)
	                                decoded = dcache_fill(&g_dcache, IPtr,
	                                                      mem, MemByteMask);
	                        fast_ip = decoded->next_ip - 1;
	                        fast_oreg = (uint32_t)decoded->operand;
	                        op = (uint32_t)decoded->opcode << 4;

                        switch (op) {
                        case 0x10: /* ldlp */
                                CReg = BReg;
                                BReg = AReg;
                                AReg = index(WPtr, fast_oreg);
                                IPtr = fast_ip + 1;
                                continue;
                        case 0x30: /* ldnl */
                                AReg = word(index(AReg, fast_oreg));
                                IPtr = fast_ip + 1;
                                continue;
                        case 0x40: /* ldc */
                                CReg = BReg;
                                BReg = AReg;
                                AReg = fast_oreg;
                                IPtr = fast_ip + 1;
                                continue;
                        case 0x50: /* ldnlp */
                                AReg = index(AReg, fast_oreg);
                                IPtr = fast_ip + 1;
                                continue;
                        case 0x70: /* ldl */
                                CReg = BReg;
                                BReg = AReg;
                                AReg = word(index(WPtr, fast_oreg));
                                IPtr = fast_ip + 1;
                                continue;
                        case 0x80: /* adc */
                                t4_overflow = FALSE;
                                t4_carry = 0;
                                AReg = t4_eadd32(AReg, fast_oreg);
                                if (t4_overflow == TRUE)
                                        SetError;
                                IPtr = fast_ip + 1;
                                if (__builtin_expect((!PrevError && ReadError) &&
                                    (exitonerror || ReadHaltOnError), 0))
                                        goto FastPrimaryError;
                                continue;
                        case 0xb0: /* ajw */
                                UpdateWdescReg(index(WPtr, fast_oreg) | ProcPriority);
                                IPtr = fast_ip + 1;
                                continue;
                        case 0xc0: /* eqc */
                                AReg = (AReg == fast_oreg) ? true_t : false_t;
                                IPtr = fast_ip + 1;
                                continue;
                        case 0xd0: /* stl */
                                writeword(index(WPtr, fast_oreg), AReg);
                                AReg = BReg;
                                BReg = CReg;
                                IPtr = fast_ip + 1;
                                continue;
                        case 0xe0: /* stnl */
                                writeword(index(AReg, fast_oreg), BReg);
                                AReg = CReg;
                                IPtr = fast_ip + 1;
                                continue;
                        default:
                                break;
                        }
                }
#endif

                /* Fast path disabled — caused FPU rounding regression.
                 * TODO: ensure fast path updates icache and handles T4COMBINATIONS. */
#if 0 /* defined(NDEBUG) && !defined(EMUDEBUG) */
                Instruction = byte_int(IPtr);
                Icode = Instruction & 0xf0;
                Idata = Instruction & 0x0f;
                OReg = OReg + Idata;
                /* Inline prefix loop (PFIX/NFIX) — most common instructions */
                while (Icode == 0x20) {         /* PFIX */
                        OReg = OReg << 4;
                        IPtr++;
                        Instruction = byte_int(IPtr);
                        Icode = Instruction & 0xf0;
                        Idata = Instruction & 0x0f;
                        OReg = OReg + Idata;
                }
                if (Icode == 0x60) {            /* NFIX */
                        OReg = (~OReg) << 4;
                        IPtr++;
                        Instruction = byte_int(IPtr);
                        Icode = Instruction & 0xf0;
                        Idata = Instruction & 0x0f;
                        OReg = OReg + Idata;
                        /* Handle PFIX after NFIX */
                        while (Icode == 0x20) {
                                OReg = OReg << 4;
                                IPtr++;
                                Instruction = byte_int(IPtr);
                                Icode = Instruction & 0xf0;
                                Idata = Instruction & 0x0f;
                                OReg = OReg + Idata;
                        }
                }
                goto FastDispatch;
#endif
		if (__builtin_expect(production_quiet, 1))
			goto ProductionFetch;

#ifndef NDEBUG
                temp = temp2 = Undefined_p;
                otherWdesc = otherWPtr = otherPtr = altState = Undefined_p;
                dbtemp1 = dbtemp2 = DUndefined;
                sntemp1 = sntemp2 = RUndefined;
#endif
#ifdef EMUDEBUG
                r32temp = RUndefined;
#endif

                if (bootdbg_enabled && bootdbg_stop_after_call_enabled && stop_after_boot_call) {
                        fprintf(stderr,
                                "[BOOTDBG] stopping after %s @ 0x%08X target=0x%08X return=0x%08X WPtr=0x%08X\n",
                                last_boot_call_is_gcall ? "GCALL" : "CALL",
                                last_boot_call_site, last_boot_call_target,
                                last_boot_call_return, last_boot_call_wptr);
                        return;
                }

                /* Periodic IPtr heartbeat for early-boot hangs */
                if (bootdbg_enabled) {
                        static uint32_t iptr_heartbeat = 0;
                        if ((iptr_heartbeat++ % 1000000) == 0) {
                                fprintf(stderr, "[BOOTDBG] heartbeat IPtr=0x%08X WPtr=0x%08X\n",
                                        IPtr, WPtr);
                        }
                }

		if (outbyte_trace_enabled && !outbyte_entry_logged) {
			const link_sym *lsym = NULL;
			const prolog_sym *sym = NULL;
			uint32_t entry_iptr = 0;
			lsym = find_link_sym_by_name("__outbyte");
			if (!lsym)
				lsym = find_link_sym_by_name("transputer_outbyte");
			if (lsym && link_text_base) {
				entry_iptr = link_text_base + lsym->sym_off;
				fprintf(stderr,
				        "[OUTBYTE_DUMP] func=%s entry=0x%08X\n",
				        lsym->name, entry_iptr);
			} else {
				sym = find_prolog_sym_by_name("__outbyte");
				if (!sym)
					sym = find_prolog_sym_by_name("transputer_outbyte");
				if (sym)
					entry_iptr = prolog_text_base + sym->text_off;
				if (sym) {
					fprintf(stderr,
					        "[OUTBYTE_DUMP] func=%s entry=0x%08X\n",
					        sym->name, entry_iptr);
				}
			}
			if (entry_iptr) {
				fprintf(stderr, "[OUTBYTE_BYTES] ");
				for (i = 0; i < 32; i++) {
					fprintf(stderr, "%02X%s", byte_int(entry_iptr + i),
					        (i == 31) ? "\n" : " ");
				}
				outbyte_entry_logged = 1;
			}
		}
		if (!sym_dump_logged) {
			static int dump_sym_env_checked;
			static const char *dump_sym_name;
			if (!dump_sym_env_checked) {
				dump_sym_name = getenv("T4_DUMP_SYM");
				dump_sym_env_checked = 1;
			}
			if (dump_sym_name && *dump_sym_name)
				dump_sym_bytes_once(dump_sym_name);
			else
				sym_dump_logged = 1;
		}
		if (boot_dump_start_enabled && !boot_dump_start_done && IPtr == MemStart) {
			uint32_t w0 = word(index(WPtr, 0));
			uint32_t w1 = word(index(WPtr, 1));
			fprintf(stderr,
			        "[BOOTSTART] IPtr=0x%08X WPtr=0x%08X WPtr[0]=0x%08X WPtr[1]=0x%08X\n",
			        IPtr, WPtr, w0, w1);
			boot_dump_start_done = 1;
		}
		if (!iptr_dump_logged)
			dump_iptr_bytes_once();
		setup_sym_trace_once();
		setup_iptr_trace_once();
		setup_text_write_trace_once();
		setup_cache_debug_trace_once();

                /* Short-range trace around head.S BSS loop */
                if (bootdbg_enabled && IPtr >= 0x80000090 && IPtr <= 0x800000B0) {
                        static int iptr_trace_count = 0;
                        if (iptr_trace_count < 32) {
                                fprintf(stderr, "[BOOTDBG] IPtr=0x%08X\n", IPtr);
                                iptr_trace_count++;
                        }
                }
                /* Short-range trace around bare-metal bootstrap entry. */
                if (bootdbg_enabled && IPtr >= 0x80000070 && IPtr <= 0x80000090) {
                        static int iptr_trace0_count = 0;
                        if (iptr_trace0_count < 64) {
                                uint8_t op0 = byte_int(IPtr);
                                uint8_t op1 = byte_int(IPtr + 1);
                                const char *mnemo = mnemonic(Icode, OReg, AReg, 0);
                                fprintf(stderr,
                                        "[BOOTDBG] entry IPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X O=0x%08X op=%02X %02X err=%d ho=%d icode=0x%02X ins=0x%02X idata=0x%X mnemo=%s\n",
                                        IPtr, AReg, BReg, CReg, OReg, op0, op1,
                                        ReadError ? 1 : 0, ReadHaltOnError ? 1 : 0,
                                        Icode, Instruction, Idata, mnemo);
                                iptr_trace0_count++;
                        }
                }
                if (bootdbg_enabled && IPtr >= 0x800000B5 && IPtr <= 0x80000110) {
                        static int iptr_trace2_count = 0;
                        if (iptr_trace2_count < 64) {
                                fprintf(stderr, "[BOOTDBG] post-bss IPtr=0x%08X\n", IPtr);
                                iptr_trace2_count++;
                        }
                }
                if (bootdbg_enabled && IPtr >= 0x80000180 && IPtr <= 0x800001B0) {
                        static int iptr_trace3_count = 0;
                        if (iptr_trace3_count < 128) {
                                uint8_t op0 = byte_int(IPtr);
                                uint8_t op1 = byte_int(IPtr + 1);
                                fprintf(stderr,
                                        "[BOOTDBG] boot2 IPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X O=0x%08X op=%02X %02X icode=0x%02X ins=0x%02X idata=0x%X mnemo=%s\n",
                                        IPtr, AReg, BReg, CReg, OReg, op0, op1,
                                        Icode, Instruction, Idata,
                                        mnemonic(Icode, OReg, AReg, 0));
                                iptr_trace3_count++;
                        }
                }
                /* Bare-metal entry trace: first hit into .text or an explicit range. */
                {
                        static int entry_trace_init = 0;
                        static int entry_trace_enabled = 0;
                        static int entry_trace_done = 0;
                        static unsigned int entry_trace_hits = 0;
                        static unsigned int entry_trace_target_hit = 1;
                        static unsigned int entry_trace_max_hits = 1;
                        static int entry_trace_deref = 0;
                        static uint32_t entry_start = 0;
                        static uint32_t entry_end = 0;
                        if (!entry_trace_init) {
                                const char *env = getenv("T4_ENTRY_TRACE");
                                const char *env_start = getenv("T4_ENTRY_START");
                                const char *env_end = getenv("T4_ENTRY_END");
                                const char *env_iptr = getenv("T4_ENTRY_IPTR");
                                const char *env_hit = getenv("T4_ENTRY_HIT");
                                const char *env_hits = getenv("T4_ENTRY_HITS");
                                const char *env_deref = getenv("T4_ENTRY_DEREF");
                                entry_trace_enabled = (env && *env);
                                if (env_hit && *env_hit) {
                                        entry_trace_target_hit = (unsigned int)strtoul(env_hit, NULL, 0);
                                        if (!entry_trace_target_hit)
                                                entry_trace_target_hit = 1;
                                }
                                if (env_hits && *env_hits) {
                                        entry_trace_max_hits = (unsigned int)strtoul(env_hits, NULL, 0);
                                        if (!entry_trace_max_hits)
                                                entry_trace_max_hits = 1;
                                }
                                entry_trace_deref = env_deref && *env_deref;
                                if (env_iptr && *env_iptr) {
                                        entry_start = (uint32_t)strtoul(env_iptr, NULL, 0);
                                        entry_end = entry_start;
                                } else {
                                        entry_start = env_start && *env_start ? (uint32_t)strtoul(env_start, NULL, 0) : 0x80000120;
                                        entry_end = env_end && *env_end ? (uint32_t)strtoul(env_end, NULL, 0) : 0x80002000;
                                }
                                entry_trace_init = 1;
                        }
                        if (entry_trace_enabled && !entry_trace_done) {
                                if ((entry_start == entry_end && IPtr == entry_start) ||
                                    (entry_start != entry_end && IPtr >= entry_start && IPtr <= entry_end)) {
                                        entry_trace_hits++;
                                        if (entry_trace_hits >= entry_trace_target_hit &&
                                            entry_trace_hits < entry_trace_target_hit + entry_trace_max_hits) {
                                                int off;
                                                fprintf(stderr,
                                                        "[ENTRY] hit=%u IPtr=0x%08X WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X O=0x%08X\n",
                                                        entry_trace_hits, IPtr, WPtr, AReg, BReg, CReg, OReg);
                                                for (off = 0; off <= 32; off += 4)
                                                        fprintf(stderr, "[ENTRY] WPtr+%02d 0x%08X\n",
                                                                off, word_int(WPtr + off));
                                                if (entry_trace_deref) {
                                                        uint32_t arg0 = word_int(WPtr + 4);
                                                        for (off = 0; off <= 32; off += 4)
                                                                fprintf(stderr, "[ENTRY] arg0+%02d 0x%08X\n",
                                                                        off, word_int(arg0 + off));
                                                }
                                                if (entry_trace_hits + 1 >=
                                                    entry_trace_target_hit + entry_trace_max_hits)
                                                        entry_trace_done = 1;
                                        }
                                }
                        }
                }
                if (bootdbg_enabled && IPtr == 0x800000B5) {
                        static int bss_done_logged = 0;
                        if (!bss_done_logged) {
                                fprintf(stderr, "[BOOTDBG] Reached bss_clear_done\n");
                                bss_done_logged = 1;
                        }
                }
                if (bootdbg_enabled && IPtr >= 0x80883780 && IPtr <= 0x80883840) {
                        static int start_kernel_seen = 0;
                        if (!start_kernel_seen) {
                                fprintf(stderr,
                                        "[BOOTDBG] entered start_kernel IPtr=0x%08X WPtr=0x%08X\n",
                                        IPtr, WPtr);
                                start_kernel_seen = 1;
                        }
                }
                if (bootdbg_enabled && last_boot_call_target != 0 && IPtr == last_boot_call_target) {
                        static int post_gcall_seen = 0;
                        if (!post_gcall_seen) {
                                fprintf(stderr,
                                        "[BOOTDBG] post-GCALL IPtr=0x%08X return=0x%08X WPtr=0x%08X\n",
                                        IPtr, last_boot_call_return, WPtr);
                                post_gcall_seen = 1;
                        }
                }

                /* One-shot snapshot at bss_clear_loop in head.S */
                if (bootdbg_enabled && IPtr == 0x800000A3) {
                        static int bss_logged = 0;
                        static uint32_t bss_loop_count = 0;
                        uint32_t bss_start = word_int(index(WPtr, 3));
                        uint32_t bss_stop = word_int(index(WPtr, 4));
                        if (!bss_logged) {
                                fprintf(stderr,
                                        "[BOOTDBG] IPtr=0x%08X WPtr=0x%08X bss_start=0x%08X bss_stop=0x%08X\n",
                                        IPtr, WPtr, bss_start, bss_stop);
                                bss_logged = 1;
                        }
                        if ((bss_loop_count++ % 100000) == 0) {
                                fprintf(stderr,
                                        "[BOOTDBG] bss_loop=%u current_ptr=0x%08X end_ptr=0x%08X\n",
                                        bss_loop_count, bss_start, bss_stop);
                        }
                }


ProductionFetch:
                if (ReadGotoSNP)
                {
                        if (noisy_boot_trace_enabled()) {
                                fprintf(stderr, "[SNP] GotoSNP set! IPtr=0x%08X WPtr=0x%08X heartbeat=%llu\n",
                                        IPtr, WPtr, (unsigned long long)heartbeat_counter);
                                fflush(stderr);
                        }
                        if (start_process ())
                                return;
                }

		/* Execute an instruction. */
        ResetRounding = FALSE;

        pslot = islot;
        islot = IHASH(IPtr);
        if (unlikely(IPtr != Icache[islot].ip))
        {
                /* Checked when an address is first cached: a hit
                 * is an address that already passed. */
		if (IPtr < MemStart &&
		    !(t4_sd_boot_enabled() &&
		      (IPtr - BOOTBANK_BASE < BOOTBANK_SIZE ||
		       (IPtr >= SD_BASE && IPtr < SD_LIMIT)))) {
			/* A P-process that runs off its regions is an AccessViolation
			 * (ISM 9.5): the machine traps to the supervisor, which is
			 * exactly the case protection exists to contain.  Killing the
			 * host instead loses the supervisor's diagnosis and turns a
			 * contained guest fault into a dead emulator.  L-mode has no
			 * supervisor to trap to, so it still halts below. */
			if (t9k_in_p()) {
				fprintf(stderr,
					"[T9K-PROT] P-process left RAM: IPtr=0x%08X "
					"WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X "
					"— AccessViolation to supervisor\n",
					IPtr, WPtr, AReg, BReg, CReg);
				fflush(stderr);
				t9k_p_trap(T9K_TRAP_ACCESS, IPtr);
				/* not reached: t9k_p_trap longjmps to the mainloop */
			}
			fault_history_dump("execution left configured RAM");
			/* Fatal: continuing would fetch from a negative mem[] index
			 * (host SIGSEGV) or spin in garbage until timeout.  Stop with
			 * a diagnostic instead — both prior failure faces become
			 * this one message. */
			fprintf(stderr,
				"[WILDEXEC] IPtr=0x%08X left RAM (MemStart=0x%08X) "
				"WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X — halting\n",
				IPtr, MemStart, WPtr, AReg, BReg, CReg);
			fflush(NULL);
			if (t4_fabric) {
				t4_net_request_stop();
				return;
			}
			exit(2);
		}
                PROFILE(profile[PRO_ICMISS]++);
                Icache[islot].ip = IPtr;
                SetCached(IPtr);
                /* OReg is preserved across icache misses; do not reset here. */
                /* Cache debug tracing for miss */
                if (cache_debug_trace_enabled &&
                    IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
                        uint8_t mem_op0 = byte_int(IPtr);
                        uint8_t mem_op1 = byte_int(IPtr + 1);
                        uint8_t mem_op2 = byte_int(IPtr + 2);
                        uint8_t mem_op3 = byte_int(IPtr + 3);
                        fprintf(stderr,
                                "[CACHE_MISS] slot=0x%X IPtr=0x%08X mem=%02X%02X%02X%02X OReg=0x%08X WPtr=0x%08X\n",
                                islot, IPtr, mem_op0, mem_op1, mem_op2, mem_op3, OReg, WPtr);
                }
#ifdef EMUDEBUG
                if (cachedebug)
                        printf ("-I-EMU414: Icache miss @ #%08X\n", IPtr);
#endif

FetchNext:
                /*
                 * Decode cache: OFF by default.
                 * A previous attempt jumped to FastDispatch without writing
                 * Icache[islot].{NextIPtr,OReg,Icode}, so the next direct-mapped
                 * Icache hit loaded garbage and hung early boot at "SLG".
                 * Re-enable only with T4_DCACHE=1 after filling Icache properly.
                 */
                if (dcache_enabled > 0 && __builtin_expect(OReg == 0, 1)) {
                        dcache_entry_t *dc = dcache_lookup(&g_dcache, IPtr);
                        if (!dc)
                                dc = dcache_fill(&g_dcache, IPtr, mem, MemByteMask);
                        if (dc && dc->next_ip > IPtr + 1) {
                                OReg = (uint32_t)dc->operand;
                                Icode = (u_char)(dc->opcode << 4);
                                Instruction = (u_char)(Icode | (OReg & 0xF));
                                IPtr = dc->next_ip - 1;
                                /* Must populate Icache before execute — hit path
                                 * reloads NextIPtr/OReg/Icode from this slot. */
                                Icache[islot].NextIPtr = IPtr;
                                Icache[islot].oreg     = OReg;
                                Icache[islot].icode    = Icode;
#ifdef T4COMBINATIONS
                                Icache[islot].Pcode = ProfileCode(Icode, OReg);
#endif
                                goto ExecuteInstr;
                        }
                }
                /* Slow path: original decode */
                Instruction = code_byte (IPtr);
	        Icode = Instruction & 0xf0;
                if (sk_entry_dump_enabled && !sk_entry_dumped && IPtr == sk_entry_iptr) {
                        int off;
                        fprintf(stderr, "[SKWPT] IPtr=0x%08X WPtr=0x%08X\n", IPtr, WPtr);
                        for (off = -16; off <= 16; off += 4) {
                                uint32_t addr = WPtr + off;
                                uint32_t val = word_int(addr);
                                fprintf(stderr,
                                        "[SKWPT] 0x%08X: 0x%08X\n",
                                        addr, val);
                        }
                        sk_entry_dumped = 1;
                }
                Idata = Instruction & 0x0f;
                OReg = OReg + Idata;

		if (sym_trace_entry && !sym_trace_active && IPtr == sym_trace_entry) {
			if (sym_trace_skip > 0)
				sym_trace_skip--;
			else
				sym_trace_active = 1;
		}
                if (sym_trace_active && sym_trace_remaining > 0) {
                        fprintf(stderr,
                                "[SYM_TRACE] func=%s IPtr=0x%08X Instruction=0x%02X Icode=0x%02X OReg=0x%08X Mnemo=%s AReg=0x%08X BReg=0x%08X CReg=0x%08X\n",
                                sym_trace_name, IPtr, Instruction, Icode, OReg,
                                mnemonic(Icode, OReg, AReg, 0), AReg, BReg, CReg);
                        sym_trace_remaining--;
                        if (sym_trace_remaining == 0)
                                sym_trace_active = 0;
                }
                if (iptr_trace_enabled && IPtr >= iptr_trace_start && IPtr < iptr_trace_end) {
                        fprintf(stderr,
                                "[IPTR_TRACE] IPtr=0x%08X Instruction=0x%02X Icode=0x%02X OReg=0x%08X Mnemo=%s AReg=0x%08X BReg=0x%08X CReg=0x%08X\n",
                                IPtr, Instruction, Icode, OReg,
                                mnemonic(Icode, OReg, AReg, 0), AReg, BReg, CReg);
                }

#ifdef EMUDEBUG
                if (cachedebug)
                        printf ("-I-EMU414: Fetched @ #%08X Icode = #%02X Idata = #%02X OReg = #%08X\n",
                                        IPtr, Icode, Idata, OReg);
#endif

                if (0x20 == Icode)
                {
		        OReg = OReg << 4;
			IPtr++;
			/* Batch skip consecutive PFIX 0 bytes (NOP padding).
			 * Each PFIX 0 just shifts OReg left by 4 with no data.
			 * Skip them all in one tight loop. */
			while (code_byte(IPtr) == 0x20) {
			        OReg <<= 4;
			        IPtr++;
			}
                        goto FetchNext;
                }
                else if (0x60 == Icode)
                {
			OReg = (~OReg) << 4;
			IPtr++;
                        goto FetchNext;
                }
                if (bootdbg_enabled && IPtr >= 0x80000190 && IPtr <= 0x80000198) {
                        fprintf(stderr,
                                "[BOOTDBG2] OReg new=0x%08X IPtr=0x%08X Icode=0x%02X Idata=0x%X\n",
                                OReg, IPtr, Icode, Idata);
                }
                Icache[islot].NextIPtr = IPtr;
                Icache[islot].oreg     = OReg;
                Icache[islot].icode    = Icode;
                /* Cache debug tracing for store */
                if (cache_debug_trace_enabled &&
                    Icache[islot].ip >= cache_debug_trace_start &&
                    Icache[islot].ip < cache_debug_trace_end) {
                        fprintf(stderr,
                                "[CACHE_STORE] slot=0x%X origIPtr=0x%08X NextIPtr=0x%08X OReg=0x%08X Icode=0x%02X WPtr=0x%08X\n",
                                islot, Icache[islot].ip, IPtr, OReg, Icode, WPtr);
                }

#ifdef EMUDEBUG
                Icache[islot].Instruction = Instruction;
                if (cachedebug)
                        printf ("-I-EMU414: Cached Icode = #%02X OReg = #%08X\n", Icode, OReg);
#endif

#ifdef T4COMBINATIONS
                {
                        u_short code0 = Icache[pslot].Pcode;
                        u_short code1 = ProfileCode (Icode, OReg);
                        Icache[islot].Pcode = code1;
                        if ((i = combinations[(code0 << 10) + code1])
                                && (Icache[pslot].NextIPtr == (Icache[islot].ip - 1)))
                        {
                                i--;
                                EMUDBG3 ("-I-EMU414: code0=#%03X code1=#%03X\n", code0, code1);
                                EMUDBG2 ("-I-EMU414: combined code=#%03X\n", combined[i].ccode);

                                Acache[pslot]._Arg0 = Icache[pslot].oreg;
                                Acache[pslot]._Arg1 = Icache[islot].oreg;

                                Icache[pslot].NextIPtr = Icache[islot].NextIPtr;
                                Icache[pslot].oreg  = combined[i].ccode;
                                Icache[pslot].icode = 0xF0;
                                Icache[pslot].Pcode = ProfileCode (Icache[pslot].icode, Icache[pslot].oreg);
                                EMUDBG4 ("-I-EMU414: OReg=#%X Arg0=#%X Arg1=#%X\n", Icache[pslot].oreg, Acache[pslot]._Arg0, Acache[pslot]._Arg1);
                        }
                }
#endif
                goto ExecuteInstr;
        }
#ifdef T4PROFILE
        else
                PROFILE(profile[PRO_ICHIT]++);
#endif
        IPtr  = Icache[islot].NextIPtr;
        OReg  = Icache[islot].oreg;
        Icode = Icache[islot].icode;
        /* Cache debug tracing */
        if (rare_hooks && cache_debug_trace_enabled &&
            IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
                uint8_t mem_op0 = byte_int(Icache[islot].ip);
                uint8_t mem_op1 = byte_int(Icache[islot].ip + 1);
                uint8_t mem_op2 = byte_int(Icache[islot].ip + 2);
                uint8_t mem_op3 = byte_int(Icache[islot].ip + 3);
                fprintf(stderr,
                        "[CACHE_HIT] slot=0x%X origIPtr=0x%08X NextIPtr=0x%08X OReg=0x%08X Icode=0x%02X mem=%02X%02X%02X%02X WPtr=0x%08X\n",
                        islot, Icache[islot].ip, IPtr, OReg, Icode,
                        mem_op0, mem_op1, mem_op2, mem_op3, WPtr);
        }

ExecuteInstr:
	if (__builtin_expect (t4_sig_pending != 0, 0)) {
		int sig = (int) t4_sig_pending;
		t4_sig_pending = 0;
		t4_signal_exit (sig);
	}
	/* Feed the T9000 grouper model (T4_T9000=1); no cost when disabled. */
	if (__builtin_expect(rare_hooks && t9k.enabled, 0)) {
		/* Icache[islot].ip is the start of the whole prefixed
		 * instruction (the slot is hashed once, before the pfix/nfix
		 * loop), so this is the encoded length the fetcher must pull. */
		int t9k_bytes = (int)(IPtr - Icache[islot].ip);
		if (Icode == 0xf0 && (int)OReg >= T9K_COMBINED_BASE &&
		    (int)OReg < T9K_COMBINED_BASE + T9K_COMBINED_COUNT) {
			/* A fused pair (combined[] above): issue both halves,
			 * or the model charges the pair one ALU op and cannot
			 * see the difference between e.g. `ldc;ldl` and
			 * `ldl;ldl`. */
			const int ci = (int)OReg - T9K_COMBINED_BASE;
			t9k_issue_pair (combined[ci].code0, Arg0,
					combined[ci].code1, Arg1,
					CReg, vec_len, t9k_bytes);
		} else if (Icode == 0xf0) {
			/* QGEMM's activation buffer is tagged by address */
			if (OReg == 0x1ef)
				t9k.qg_acts = word (BReg);
			/* VXFER counts its entries in B, not C */
			int lat, kind = t9k_class_opr((int)OReg,
						      OReg == 0x1fa ? BReg : CReg,
						      vec_len, &lat);
			t9k_issue(kind, lat, t9k_bytes);
		} else {
			t9k_issue(t9k_class_primary(Icode >> 4, OReg), 1,
				  t9k_bytes);
		}
	}
	heartbeat_counter++;
	/* The IPtr sampler and the framebuffer recorder both fire on
	 * instruction counts: one compare against the nearer of their next
	 * firings (all ones while neither is on) stands in for both. */
	if (__builtin_expect(rare_hooks && heartbeat_counter >= insn_hook_next, 0))
		run_insn_hooks ();
	if (__builtin_expect(production_quiet, 1))
		goto FastDispatch;
	fault_history_record(IPtr, WPtr, AReg, BReg, CReg, OReg, Icode);
	/* TEMP: T4_WPTR_RANGE=lo,hi dumps up to 20 times when IPtr in [lo,hi]. */
	{
		static uint32_t wlo = 0, whi = 0;
		static int wrange_left = -1;
		if (wrange_left < 0) {
			const char *e = getenv("T4_WPTR_RANGE");
			if (e && *e) {
				unsigned long a = 0, b = 0;
				if (sscanf(e, "%li,%li", &a, &b) == 2) {
					wlo = (uint32_t)a;
					whi = (uint32_t)b;
					wrange_left = 20;
				} else
					wrange_left = 0;
			} else
				wrange_left = 0;
		}
		if (wrange_left > 0 && IPtr >= wlo && IPtr <= whi) {
			fprintf(stderr,
				"[WPTR_RANGE] IPtr=0x%08X WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X O=0x%08X Icode=0x%02X\n",
				IPtr, WPtr, AReg, BReg, CReg, OReg, Icode);
			fprintf(stderr, "  W[7]=0x%08X W[9]=0x%08X W[10]=0x%08X W[23]=0x%08X W[24]=0x%08X\n",
				word_int(index(WPtr, 7)), word_int(index(WPtr, 9)),
				word_int(index(WPtr, 10)), word_int(index(WPtr, 23)),
				word_int(index(WPtr, 24)));
			fflush(stderr);
			wrange_left--;
		}
	}
#ifdef EMUDEBUG
        Instruction = Icache[islot].Instruction;
        if (cachedebug)
        {
                if (OprCombined(Icode,OReg))
                        printf ("-I-EMU414: Icache hit @ #%08X Icode = #%02X OReg = #%08X Arg0=#%X Arg1=#%X\n",
                                IPtr, Icode, OReg, Arg0, Arg1);
                else
                        printf ("-I-EMU414: Icache hit @ #%08X Icode = #%02X OReg = #%08X\n",
                                IPtr, Icode, OReg);
        }
#endif

        /* Disable interrupts on PFIX or NFIX. */
        /* Using Icache IntEnabled is always TRUE. */

#ifndef NDEBUG
        if (Idle)
        {
                printf ("-E-EMU414: Processor is in IDLE state.\n");
                handler (-1);
        }
#endif

#ifdef EMUDEBUG
        if (emudebug)
        {
	        /* General debugging messages. */
                if (printIPtr)
                {
                        char statusCode = ' ';
                        if (0 == asmLines++ % 25)
                        {
                                if (IsT414)
                                        printf ("-IPtr------Code-----------------------Mnemonic------------HE---AReg-----BReg-----CReg-------WPtr-----WPtr[0]-\n");
                                else
                                        printf ("-IPtr------Code-----------------------Mnemonic------------HEFR---AReg-----BReg-----CReg-------WPtr-----WPtr[0]-\n");
                        }
                        if (HiPriority == ProcPriority)
                                statusCode = 'H';
                        if (Idle)
                                statusCode = 'I';
                        printf ("%c%08X: ", statusCode, IPtr);
                        printIPtr = FALSE;
                        instrBytes = 0;
                }
                printf("%02X ", Instruction); instrBytes++;
                if ((0x20 == Icode) || (0x60 == Icode))
                        ;
                else
                {
                        for (; instrBytes < 9; instrBytes++)
                                printf("   ");
                        mnemo = mnemonic (Icode, OReg, AReg, 0);
                        printf("%-17s", mnemo);
	                printf ("   %c%c", FLAG(ReadHaltOnError, 'H'), FLAG(      ReadError, 'E'));
                        if (IsT800 || IsTVS)
                        {
                                fp_syncexcept ();
	                        printf ("%c%c", FLAG(FP_Error, 'F'), RMODE[RoundingMode-1]);
                        }
                        printf ("   %8X %8X %8X   %08X %8X\n", 
                                AReg, BReg, CReg, WPtr, word_int (WPtr));
                        if (IsT800 || IsTVS)
                        {
                                currFPInstr = 0 == strncmp (mnemo, "FP", 2);
                                if (currFPInstr || prevFPInstr)
                                {
                                        print_fpreg("\t\t\t\t\t\t\t  ", 'A', &FAReg, 0);
                                        print_fpreg("\t\t\t\t\t\t\t  ", 'B', &FBReg, 0);
                                        print_fpreg("\t\t\t\t\t\t\t  ", 'C', &FCReg, 0);
                                        prevFPInstr = currFPInstr;
                                }
                        }
                        printIPtr = TRUE;
                }
                fflush (stdout);
        }
#endif

#ifdef T4PROFILEINSTR
	if (profiling && (0xF0 != Icode))
		add_profile (Icode);
#endif

	/* Periodic memory integrity check */
	{
	    static uint32_t watch_idx = 0xFFFFFFFF;
	    static u_char watch_expected = 0;
	    static int watch_init = 0;
	    if (!watch_init) {
	        const char *wa = getenv("T4_WATCH_ADDR");
	        if (wa) {
	            uint32_t addr = (uint32_t)strtoul(wa, NULL, 16);
	            watch_idx = addr & MemByteMask;
	            /* Check offset +2 since that's where the corruption happens */
	            watch_idx += 2;
	            watch_expected = mem[watch_idx];
	            fprintf(stderr, "[WATCH_INIT] monitoring mem[0x%X] = 0x%02X\n", watch_idx, watch_expected);
	        }
	        watch_init = 1;
	    }
	    if (watch_idx != 0xFFFFFFFF && (heartbeat_counter % 1000) == 0) {
	        u_char cur = mem[watch_idx];
	        if (cur != watch_expected) {
	            fprintf(stderr, "[WATCH_CORRUPT] mem[0x%X] changed 0x%02X->0x%02X at hb=%llu IPtr=0x%08X WPtr=0x%08X\n",
	                    watch_idx, watch_expected, cur, (unsigned long long)heartbeat_counter, IPtr, WPtr);
	            watch_expected = cur;  /* Only report once */
	        }
	    }
	}
	if (noisy_boot_trace_enabled() && (heartbeat_counter % 100) == 0) {
		fprintf(stderr, "[HB] %llu IPtr=0x%08X WPtr=0x%08X AReg=0x%08X Icode=0x%02X\n",
			(unsigned long long)heartbeat_counter, IPtr, WPtr, AReg, Icode);
		fflush(stderr);
	}

FastDispatch:
	switch (Icode)
	{
		case 0x00: /* j     */
			   IPtr++;
			   if (OReg == 0 && EnableJ0BreakFlag) {
			           handle_j0_break();
			   } else {
			           IPtr = IPtr + OReg;
			           D_check();
			   }
			   break;
		case 0x10: /* ldlp  */
			   {
			       uint32_t oldA = AReg;
			       CReg = BReg;
			       BReg = AReg;
			       AReg = index (WPtr, OReg);
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDLP IPtr=0x%08X OReg=0x%X AReg:0x%08X->0x%08X WPtr=0x%08X\n",
			                   IPtr, OReg, oldA, AReg, WPtr);
			       }
			       if (rare_hooks && sk_addr_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKADDR] LDLP IPtr=0x%08X OReg=0x%X WPtr=0x%08X addr=0x%08X\n",
			                   IPtr, OReg, WPtr, AReg);
			       }
			   }
			   IPtr++;
			   break;
		case 0x20: /* pfix  */
			   IPtr++;
			   break;
		case 0x30: /* ldnl  */
                           T4DEBUG(checkWordAligned ("LDNL", AReg));
			   {
			       uint32_t oldA = AReg;
			       uint32_t addr = index(AReg, OReg);
			       uint32_t value = word(addr);
			       /* Debug: Show critical ldnl operations in bootstrap range
			        * (boot debugging only: a small program's own loops
			        * live there too, and printed every pass) */
			       if (rare_hooks && bootdbg_enabled &&
			           IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			           printf("[LDNL] IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X value=0x%08X\n",
			                  IPtr, AReg, OReg, addr, value);
			       }
			       AReg = value;
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDNL IPtr=0x%08X OReg=0x%X addr=0x%08X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, addr, oldA, AReg);
			       }
			       if (rare_hooks && sk_addr_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKADDR] LDNL IPtr=0x%08X OReg=0x%X addr=0x%08X value=0x%08X\n",
			                   IPtr, OReg, addr, value);
			       }
			   }
			   IPtr++;
			   break;
		case 0x40: /* ldc   */
			   {
			       uint32_t oldA = AReg;
			       CReg = BReg;
			       BReg = AReg;
			       AReg = OReg;
			       /* Optional trace for framebuffer-address LDC spam during kernel boot.
			        * Leave this off by default so boot logs stay readable. */
			       if (rare_hooks && trace_fb_ldc_enabled && fb_addr_in_range(AReg)) {
			           fprintf(stderr, "[FB_LDC] IPtr=0x%08X loaded=0x%08X\n", IPtr, AReg);
			       }
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDC IPtr=0x%08X OReg=0x%X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, oldA, AReg);
			       }
			   }
			   IPtr++;
			   break;
		case 0x50: /* ldnlp */
                           /* NB. Minix demo uses unaligned AReg! */
                           T4DEBUG(checkWordAligned ("LDNLP", AReg));
			   {
			       uint32_t oldA = AReg;
			       AReg = index (AReg, OReg);
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDNLP IPtr=0x%08X OReg=0x%X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, oldA, AReg);
			       }
			   }
			   IPtr++;
			   break;
		case 0x60: /* nfix  */
			   IPtr++;
			   break;
		case 0x70: /* ldl   */
			   {
			       uint32_t oldA = AReg;
			       uint32_t addr = index(WPtr, OReg);
			       CReg = BReg;
			       BReg = AReg;
			       AReg = word(addr);
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDL IPtr=0x%08X OReg=0x%X addr=0x%08X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, addr, oldA, AReg);
			       }
			       if (rare_hooks && sk_addr_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKADDR] LDL IPtr=0x%08X OReg=0x%X WPtr=0x%08X addr=0x%08X value=0x%08X\n",
			                   IPtr, OReg, WPtr, addr, AReg);
			       }
			   }
			   /* Debug: Trace ldl in bootstrap range */
			   if (rare_hooks && noisy_boot_trace_enabled() &&
			       IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			       printf("[LDL] IPtr=0x%08X local=%d addr=0x%08X value=0x%08X → AReg\n",
			              IPtr, OReg, index(WPtr, OReg), AReg);
			   }
			   IPtr++;
			   break;
		case 0x80: /* adc   */
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   {
			       uint32_t oldA = AReg;
			       AReg = t4_eadd32 (AReg, OReg);
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] ADC IPtr=0x%08X OReg=0x%X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, oldA, AReg);
			       }
			   }
			   if (t4_overflow == TRUE)
				SetError;
			   IPtr++;
			   break;
		case 0x90: /* call  */
			   /* ISM 9.3: restartable stack-extension trap */
			   if (unlikely(t9k_prot.is_p))
				   t9k_check_wptr(index(WPtr, -4), Icache[islot].ip);
			   IPtr++;
			   /* Log kernel function calls */
			   if (rare_hooks && noisy_boot_trace_enabled() &&
			       IPtr >= 0x80000000 && IPtr < 0x81000000) {
			       printf("[CALL] from=0x%08X to=0x%08X WPtr=0x%08X→0x%08X\n",
			              IPtr - 1, IPtr + OReg, WPtr, index(WPtr, -4));
			   }
			   if (rare_hooks) maybe_log_nearest_sym(IPtr - 1, "CALL");
			   if (rare_hooks && wptr_trace_enabled) {
			       printf("[WPtr] CALL IPtr=0x%08X target=0x%08X WPtr=0x%08X→0x%08X\n",
			              IPtr - 1, IPtr + OReg, WPtr, index(WPtr, -4));
			   }
			   if (rare_hooks && bootdbg_stop_after_call_enabled &&
			       (IPtr - 1) >= boot_call_start && (IPtr - 1) <= boot_call_end) {
			       last_boot_call_is_gcall = 0;
			       last_boot_call_site = IPtr - 1;
			       last_boot_call_target = IPtr + OReg;
			       last_boot_call_return = IPtr;
			       last_boot_call_wptr = WPtr;
			       maybe_auto_text_base();
			       stop_after_boot_call = 1;
			       if (rare_hooks && bootdbg_enabled)
			               fprintf(stderr,
			                       "[BOOTDBG] armed stop after CALL @ 0x%08X target=0x%08X return=0x%08X\n",
			                       last_boot_call_site, last_boot_call_target, last_boot_call_return);
			   }
			   writeword (index (WPtr, -1), CReg);
			   writeword (index (WPtr, -2), BReg);
			   writeword (index (WPtr, -3), AReg);
			   writeword (index (WPtr, -4), IPtr);
                           UpdateWdescReg (index ( WPtr, -4) | ProcPriority);
                           T4DEBUG(checkWPtr ("CALL", WPtr));
			   AReg = IPtr;
                           /* Pop BReg. */
                           BReg = CReg;
			   IPtr = IPtr + OReg;
			   /* TEMP: dump CALL frame when T4_CALL_DUMP=1 (first 40 only). */
			   {
				   static int call_dump_left = -1;
				   if (call_dump_left < 0) {
					   const char *e = getenv("T4_CALL_DUMP");
					   call_dump_left = (e && *e) ? 40 : 0;
				   }
				   if (call_dump_left > 0) {
					   call_dump_left--;
					   fprintf(stderr,
						   "[CALLFRAME] ret=0x%08X A=0x%08X B=0x%08X C=0x%08X "
						   "W[0]=0x%08X W[1]=0x%08X W[2]=0x%08X W[3]=0x%08X "
						   "target=0x%08X\n",
						   AReg,
						   word_int(index(WPtr, 1)),
						   word_int(index(WPtr, 2)),
						   word_int(index(WPtr, 3)),
						   word_int(index(WPtr, 0)),
						   word_int(index(WPtr, 1)),
						   word_int(index(WPtr, 2)),
						   word_int(index(WPtr, 3)),
						   IPtr);
				   }
			   }
			   /* Cache debug tracing for CALL target */
			   if (rare_hooks && cache_debug_trace_enabled &&
			       IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
			       uint8_t t0 = byte_int(IPtr);
			       uint8_t t1 = byte_int(IPtr + 1);
			       uint8_t t2 = byte_int(IPtr + 2);
			       uint8_t t3 = byte_int(IPtr + 3);
			       uint8_t t4 = byte_int(IPtr + 4);
			       uint8_t t5 = byte_int(IPtr + 5);
			       uint8_t t6 = byte_int(IPtr + 6);
			       uint8_t t7 = byte_int(IPtr + 7);
			       fprintf(stderr,
			               "[CALL_TARGET] IPtr=0x%08X bytes=%02X%02X%02X%02X%02X%02X%02X%02X WPtr=0x%08X ret=0x%08X\n",
			               IPtr, t0, t1, t2, t3, t4, t5, t6, t7, WPtr, AReg);
			   }
			   break;
		case 0xa0: /* cj    */
			   if (AReg != 0)
			   {
				AReg = BReg;
				BReg = CReg;
				IPtr++;
			   }
			   else
			   {
				IPtr++;
				IPtr = IPtr + OReg;
			   }
			   break;
		case 0xb0: /* ajw   */
			   {
			       /* OReg is already correctly set by prefix instructions (PFIX/NFIX).
			        * NFIX complements OReg for negative values.
			        * No sign-extension needed here - bare 4-bit values 0-15 are positive. */
			       int32_t delta = (int32_t)OReg;
			       uint32_t new_wptr = index(WPtr, delta);
			       /* ISM 9.3: restartable stack-extension trap */
			       if (unlikely(t9k_prot.is_p))
				       t9k_check_wptr(new_wptr, Icache[islot].ip);
			       /* Cache debug tracing for AJW */
			       if (rare_hooks && cache_debug_trace_enabled &&
			           IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
			           fprintf(stderr,
			                   "[AJW_DEBUG] IPtr=0x%08X OReg=0x%08X delta=%d WPtr=0x%08X→0x%08X\n",
			                   IPtr, OReg, delta, WPtr, new_wptr);
			       }
			       /* Log ALL workspace changes in bootstrap range, or significant changes in kernel */
			       if (rare_hooks && noisy_boot_trace_enabled() &&
			           ((IPtr >= 0x80000070 && IPtr <= 0x80000200) ||
			            (IPtr >= 0x80000000 && IPtr < 0x81000000 && (delta > 10 || delta < -10)))) {
			           printf("[AJW] IPtr=0x%08X delta=%d WPtr=0x%08X→0x%08X\n",
			                  IPtr, delta, WPtr, new_wptr);
			       }
			       /* Validate workspace pointer - use actual configured memory size.
			        * Q3 LAs (0xC0000000+) are legal under T9000 multi-quarter when
			        * RODS region 3 is loaded; t9k_translate relocates them. */
			       {
				   int wptr_q3_ok = (new_wptr >= 0xC0000000u) &&
						    t9k_in_p();
				   /* Also OK if region[3] still maps after leave_p */
				   if (!wptr_q3_ok && new_wptr >= 0xC0000000u &&
				       t9k_prot.enabled &&
				       t9k_prot.region[3] != 0u &&
				       t9k_prot.region[3] != 0x80000000u)
					   wptr_q3_ok = 1;
				   if (!wptr_q3_ok &&
				       (new_wptr == 0x2FFA2FFA || new_wptr < MemStart ||
				        (uint64_t)new_wptr >= (uint64_t)MemStart + (uint64_t)MemSize)) {
					   /* Report the first few, then only at powers of
					    * two: a guest looping over a bad AJW once wrote
					    * one line per instruction, a 24 GB boot log. */
					   static unsigned long bad_ajw_count;
					   bad_ajw_count++;
					   if (bad_ajw_count <= 16 ||
					       (bad_ajw_count & (bad_ajw_count - 1)) == 0)
						   printf("[ERROR] Invalid WPtr after ajw: 0x%08X at IPtr=0x%08X (valid range: 0x%08X-0x%08X) [%lu so far]\n",
							  new_wptr, IPtr, MemStart,
							  (uint32_t)((uint64_t)MemStart + (uint64_t)MemSize - 1),
							  bad_ajw_count);
					   fault_history_dump("invalid AJW workspace");
				   }
			       }
                               UpdateWdescReg (new_wptr | ProcPriority);
                               T4DEBUG(checkWPtr ("AJW", WPtr));
			   }
			   IPtr++;
			   break;
		case 0xc0: /* eqc   */
			   if (AReg == OReg)
			   {
				AReg = true_t;
			   }
			   else
			   {
				AReg = false_t;
			   }
			   IPtr++;
			   break;
		case 0xd0: /* stl   */
			   {
			       uint32_t addr = index(WPtr, OReg);
			       if (rare_hooks && text_write_trace_enabled &&
			           addr >= text_write_trace_start &&
			           addr < text_write_trace_end) {
			           uint32_t delta = 0;
			           const link_sym *lsym = find_link_sym(IPtr, &delta);
			           const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
			           fprintf(stderr,
			                   "[TEXT_STL] IPtr=0x%08X WPtr=0x%08X OReg=0x%X addr=0x%08X AReg=0x%08X%s%s\n",
			                   IPtr, WPtr, OReg, addr, AReg,
			                   (lsym || sym) ? " sym=" : "",
			                   lsym ? lsym->name : (sym ? sym->name : ""));
			           if (lsym) {
			               fprintf(stderr, "            sym_off=0x%X+0x%X\n",
			                       lsym->sym_off, delta);
			           } else if (sym) {
			               fprintf(stderr, "            sym_off=0x%X+0x%X\n",
			                       sym->text_off, delta);
			           }
			       }
			       writeword(addr, AReg);
			   }
			   /* Debug: Trace stl in bootstrap range */
			   if (rare_hooks && noisy_boot_trace_enabled() &&
			       IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			       printf("[STL] IPtr=0x%08X local=%d addr=0x%08X AReg=0x%08X stored\n",
			              IPtr, OReg, index(WPtr, OReg), AReg);
			   }
			   if (rare_hooks && sk_store_trace_enabled && sk_entry_iptr &&
			       IPtr >= sk_entry_iptr && IPtr <= (sk_entry_iptr + 0x200)) {
			       printf("[SKSTL] IPtr=0x%08X local=%d addr=0x%08X AReg=0x%08X\n",
			              IPtr, OReg, index(WPtr, OReg), AReg);
			   }
			   if (rare_hooks && sk_areg_trace_enabled &&
			       IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			       fprintf(stderr,
			               "[SKA] STL IPtr=0x%08X AReg:0x%08X->0x%08X\n",
			               IPtr, AReg, BReg);
			   }
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0xe0: /* stnl  */
                           T4DEBUG(checkWordAligned ("STNL", AReg));
			   /* Debug: catch any STNL to framebuffer/VGA area */
			   {
			       uint32_t stnl_addr = index(AReg, OReg);
			       /* Trace VGA register writes (0xA0000000 range) */
			       if (rare_hooks && (stnl_addr & 0xFFFFF000) == 0xA0000000) {
			           fprintf(stderr, "[VGA_STNL] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                   IPtr, stnl_addr, BReg);
			       }
			       /* VGA screenshot trigger: write 0x5356 ("SV") to VGA control saves framebuffer */
			       if (stnl_addr == 0xA0000000 && BReg == 0x5356) {
			           static int screenshot_count = 0;
			           char sname[64];
			           snprintf(sname, sizeof(sname), "screenshot_%d.bmp", screenshot_count++);
			           vga_dump_framebuffer_bmp(sname, screenshot_count - 1);
			       }
			       /* Debug: trace ALL STNLs in setup_arch to find FB writes */
			       if (rare_hooks && IPtr >= 0x8053c57b && IPtr < 0x8053d57b) {
			           static int setup_stnl_count = 0;
			           if (setup_stnl_count < 200) {
			               fprintf(stderr, "[SETUP_STNL#%d] IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X val=0x%08X\n",
			                       setup_stnl_count, IPtr, AReg, OReg, stnl_addr, BReg);
			               setup_stnl_count++;
			           }
			       }
			       if (rare_hooks && fb_addr_in_range(stnl_addr)) {
			           static int fb_stnl_count = 0;
			           if (fb_stnl_count < 50) {
			               fprintf(stderr, "[FB_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_stnl_count, IPtr, stnl_addr, BReg);
			               fb_stnl_count++;
			           }
			       }
			   }
			   if (rare_hooks && bootdbg_enabled && IPtr >= 0x80883FE0 && IPtr <= 0x80884020) {
			       uint32_t addr = index(AReg, OReg);
			       fprintf(stderr, "[BOOTDBG] STNL IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X BReg=0x%08X\n",
			               IPtr, AReg, OReg, addr, BReg);
			   }
			   if (rare_hooks && sk_store_trace_enabled && sk_entry_iptr &&
			       IPtr >= sk_entry_iptr && IPtr <= (sk_entry_iptr + 0x200)) {
			       uint32_t addr = index(AReg, OReg);
			       printf("[SKSTNL] IPtr=0x%08X addr=0x%08X BReg=0x%08X\n",
			              IPtr, addr, BReg);
			   }
			   {
			       uint32_t addr = index(AReg, OReg);
			       if (rare_hooks && text_write_trace_enabled &&
			           addr >= 0x80880000 && addr < 0x80890000) {
			           uint32_t delta = 0;
			           const link_sym *lsym = find_link_sym(IPtr, &delta);
			           const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
			           fprintf(stderr,
			                   "[TEXT_STNL] IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X BReg=0x%08X WPtr=0x%08X%s%s\n",
			                   IPtr, AReg, OReg, addr, BReg, WPtr,
			                   (lsym || sym) ? " sym=" : "",
			                   lsym ? lsym->name : (sym ? sym->name : ""));
			           if (lsym) {
			               fprintf(stderr, "             sym_off=0x%X+0x%X\n",
			                       lsym->sym_off, delta);
			           } else if (sym) {
			               fprintf(stderr, "             sym_off=0x%X+0x%X\n",
			                       sym->text_off, delta);
			           }
			       }
			   }
			   writeword (index (AReg, OReg), BReg);
			   if (rare_hooks && sk_areg_trace_enabled &&
			       IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			       fprintf(stderr,
			               "[SKA] STNL IPtr=0x%08X AReg:0x%08X->0x%08X\n",
			               IPtr, AReg, CReg);
			   }
			   AReg = CReg;
			   IPtr++;
			   break;
		case 0xf0: /* opr   */

	PROFILEINSTR(add_profile (0x100 + OReg));
	/* -s414/-s425: an instruction that part lacks stops the run */
	if (unlikely(rare_hooks && t4_isa_strict) && !t4xx_has_opr ((int32_t)OReg)) {
		t4xx_bad_opr ((int32_t)OReg);
		break;
	}
	/* T9000: privileged secondaries trap when running as P-process. */
	if (unlikely(t9k_prot.is_p))
		t9k_check_priv_op(OReg);
	switch (OReg)
	{
		case 0x00: /* rev         */
			   {
			       uint32_t oldA = AReg;
			       temp = AReg;
			       AReg = BReg;
			       BReg = temp;
			       if (rare_hooks && sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] REV IPtr=0x%08X AReg:0x%08X->0x%08X BReg=0x%08X\n",
			                   IPtr, oldA, AReg, BReg);
			       }
			   }
			   IPtr++;
			   break;
		case 0x01: /* lb          */
			   AReg = byte (AReg);
			   IPtr++;
			   break;
		case 0x02: /* bsub        */
			   AReg = AReg + BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x03: /* XXX endp        */
			   temp = word (index (AReg, 1));
			   if (temp == 1)
			   {
                                EMUDBG ("-I-EMUDBG: endp: Do successor process.\n");

				/* Do successor process. */
				UpdateWdescReg (AReg | ProcPriority);
                                T4DEBUG(checkWPtr ("ENDP", WPtr));
				IPtr = word (index (AReg, 0));
			   }
			   else
			   {
				/* Have not finished all parallel branches. */
                                EMUDBG2 ("-I-EMUDBG: endp: Waiting for parallel branches (%d).\n", temp);

				/* start_process (); */
                                SetGotoSNP;
			   }
                           temp--;
                           writeword (index (AReg, 1), temp);
			   break;
		case 0x04: /* diff        */
			   AReg = BReg - AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x05: /* add         */
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   AReg = t4_eadd32 (BReg, AReg);
			   if (t4_overflow == TRUE)
				SetError;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x06: /* gcall       */
			   {
			       uint32_t call_site = IPtr;
			   /* Debug: Show gcall in bootstrap range */
			   if (rare_hooks && IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			       printf("[GCALL] IPtr=0x%08X AReg=0x%08X (target) WPtr=0x%08X BReg=0x%08X CReg=0x%08X\n",
			              IPtr, AReg, WPtr, BReg, CReg);
			   }
			   if (rare_hooks) maybe_log_nearest_sym(IPtr, "GCALL");
			   if (rare_hooks && wptr_trace_enabled) {
			       printf("[WPtr] GCALL IPtr=0x%08X target=0x%08X WPtr=0x%08X\n",
			              IPtr, AReg, WPtr);
			   }
			   IPtr++;
			   temp = AReg;
			   AReg = IPtr;
			   IPtr = temp;
			   /* Wild-jump detector: indirect call through a bad function
			    * pointer (e.g. an un-relocated bFLT pointer) lands below
			    * MemStart. Report the call site + eval stack once. */
			   if (temp < MemStart &&
			       !(t4_sd_boot_enabled() &&
			         temp - BOOTBANK_BASE < BOOTBANK_SIZE)) {
			       static int wj_gcall = 0;
			       if (wj_gcall < 20) {
			           printf("[WILDJUMP] GCALL from=0x%08X to=0x%08X (target<MemStart) "
			                  "ret=0x%08X WPtr=0x%08X BReg=0x%08X CReg=0x%08X\n",
			                  call_site, temp, AReg, WPtr, BReg, CReg);
				   if ((WPtr >> 30) == 3u) {
					   int wi;
					   uint32_t pa = 0;
					   (void)t9k_probe(WPtr, T9K_ACC_READ, &pa);
					   fprintf(stderr,
						   "[Q3-GCALL0] is_p=%d WPtr=0x%08X pa=0x%08X B=0x%08X frame:",
						   t9k_in_p(), WPtr, pa, BReg);
					   for (wi = 0; wi < 32; wi++)
						   fprintf(stderr, " [%d]=%08X",
							   wi, word_int(index(WPtr, wi)));
					   fprintf(stderr, "\n");
					   /* B as structure: dump first words if in RAM */
					   if (BReg >= 0x80000000u && BReg < 0x90000000u) {
						   fprintf(stderr, "[Q3-GCALL0] *B:");
						   for (wi = 0; wi < 8; wi++)
							   fprintf(stderr, " %08X",
								   word_int(BReg + 4u * (uint32_t)wi));
						   fprintf(stderr, "\n");
					   }
					   /* current_thread_info is a global at fixed VA on this port */
					   {
						   uint32_t cti = word_int(0x80408000u);
						   uint32_t task = cti ? word_int(cti) : 0;
						   fprintf(stderr,
							   "[Q3-GCALL0] __cti_ptr@80408000=%08X task=%08X\n",
							   cti, task);
					   }
				   }
			           wj_gcall++;
			       }
			   }
			   if (rare_hooks && bootdbg_enabled && temp >= 0x804BAC00 && temp <= 0x804BB000) {
			       fprintf(stderr,
			               "[BOOTDBG] gcall target IPtr=0x%08X return_addr=0x%08X WPtr=0x%08X\n",
			               IPtr, AReg, WPtr);
			   }
			   /* Show where we're jumping to */
			   if (rare_hooks && ((IPtr >= 0x80000070 && IPtr <= 0x80000200) || (temp >= 0x80000070 && temp <= 0x80000200))) {
			       printf("[GCALL] Jumping to IPtr=0x%08X return_addr=0x%08X\n", IPtr, AReg);
			   }
			   last_boot_call_is_gcall = 1;
			   last_boot_call_site = call_site;
			   last_boot_call_target = IPtr;
			   last_boot_call_return = AReg;
			   last_boot_call_wptr = WPtr;
			   maybe_auto_text_base();
			   if (rare_hooks && bootdbg_stop_after_call_enabled &&
			       call_site >= boot_call_start && call_site <= boot_call_end) {
			       stop_after_boot_call = 1;
			       if (rare_hooks && bootdbg_enabled)
			               fprintf(stderr,
			                       "[BOOTDBG] armed stop after GCALL @ 0x%08X target=0x%08X return=0x%08X\n",
			                       last_boot_call_site, last_boot_call_target, last_boot_call_return);
			   }
			   }
			   break;
		case 0x07: /* in          */
OprIn:                     if (__builtin_expect (T9K_EXTCHAN(BReg), 0))
                           {
                                /* T9000 event or virtual channel */
                                IPtr++;
                                t9k_ext_in (BReg, CReg, AReg, T9K_IO_FIXED);
                                break;
                           }
                           if (IsLinkOut(BReg)) /* M.Bruestle 22.1.2012 */
                           {
                                MSGDBG2 ("-W-EMUDBG: Warning - doing IN on Link%dOut.\n", TheLink(BReg));
                                goto OprOut;
                           }
                           PROFILE(profile[PRO_CHANIN] += AReg);
			   MSGDBG4 ("-I-EMUDBG: in(1): Channel=#%08X, to memory at #%08X, length #%X.\n", BReg, CReg, AReg);
			   IPtr++;
			   if (!IsLinkIn(BReg))
			   {
				/* Internal communication. */
				otherWdesc = word (BReg);
                                MSGDBG2 ("-I-EMUDBG: in(2): Internal communication. Channel word=#%08X.\n", otherWdesc);
				if (otherWdesc == NotProcess_p)
				{
					/* Not ready. */
                                        MSGDBG ("-I-EMUDBG: in(3): Not ready.\n");
					writeword (BReg, Wdesc);
					writeword (index (WPtr, Pointer_s), CReg);
                                        deschedule ();
				}
				else
				{
					/* Ready. */
                                        otherWPtr = GetDescWPtr(otherWdesc);
                                        T4DEBUG(checkWPtr ("IN", otherWPtr));
					otherPtr = word (index (otherWPtr, Pointer_s));
					MSGDBG2 ("-I-EMUDBG: in(3): Transferring message from #%08X.\n", otherPtr);

                                        if (1 == AReg)
                                                writebyte_int (CReg, byte_int (otherPtr));
                                        else if (4 == AReg)
                                                writeword_int (CReg, word_int (otherPtr));
                                        else
                                                movebytes_int (CReg, otherPtr, AReg);
                                        CReg = CReg + BytesRead(CReg, AReg);
					writeword (BReg, NotProcess_p);
					schedule (otherWdesc);
				}
			   }
			   else
			   {
				/* Link communication. */
                                MSGDBG3 ("-I-EMUDBG: in(2): Link%d communication. Old channel word=#%08X.\n", TheLink(BReg), word (BReg));
                                if (serve && (Link0In == BReg))
                                        goto DescheduleIn;

                                if (recv_channel (&Link[TheLink(BReg)].In, CReg, AReg))
                                {
DescheduleIn:
				        writeword (BReg, Wdesc);
                                        Link[TheLink(BReg)].In.Address = CReg;
                                        Link[TheLink(BReg)].In.Length  = AReg;
                                        PROFILE(Link[TheLink(BReg)].In.IOBytes += AReg);
                                        deschedule ();
                                }
			   }
			   break;
		case 0x08: /* prod        */
			   AReg = BReg * AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x09: /* gt          */
			   if (INT32(BReg) > INT32(AReg))
			   {
				AReg = true_t;
			   }
			   else
			   {
				AReg = false_t;
			   }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x0a: /* wsub        */
			   AReg = index (AReg, BReg);
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x0b: /* out         */
OprOut:                    if (__builtin_expect (T9K_EXTCHAN(BReg), 0))
                           {
                                IPtr++;
                                t9k_ext_out (BReg, CReg, AReg, T9K_IO_FIXED);
                                break;
                           }
                           if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
                           {
                                MSGDBG2 ("-W-EMUDBG: Warning - doing OUT on Link%dIn.\n", TheLink(BReg));
                                goto OprIn;
                           }
                           PROFILE(profile[PRO_CHANOUT] += AReg);
			   MSGDBG4 ("-I-EMUDBG: out(1): Channel=#%08X, length #%X, from memory at #%08X.\n", BReg, AReg, CReg);
			   IPtr++;
			   if (!IsLinkOut(BReg))
			   {
				/* Internal communication. */
				otherWdesc = word (BReg);
                                MSGDBG2 ("-I-EMUDBG: out(2): Internal communication. Channel word=#%08X.\n", otherWdesc);
                                if (__builtin_expect (T9K_RESCHAN_P == otherWdesc, 0))
                                {
                                        /* resource channel: claim, then wait */
                                        t9k_internal_claim (BReg);
                                        otherWdesc = NotProcess_p;
                                }
				if (otherWdesc == NotProcess_p)
				{
					/* Not ready. */
                                        MSGDBG ("-I-EMUDBG: out(3): Not ready.\n");
					writeword (BReg, Wdesc);
					writeword (index (WPtr, Pointer_s), CReg);
                                        deschedule ();
				}
				else
				{
					/* Ready. */
                                        otherWPtr  = GetDescWPtr(otherWdesc);
					altState = otherPtr = word (index (otherWPtr, State_s));
                                        MSGDBG2 ("-I-EMUDBG: out(3): Memory address/ALT state=#%08X.\n", altState);
					if ((altState & 0xfffffffc) == MostNeg)
					{
						/* ALT guard test - not ready to communicate. */
                                                MSGDBG ("-I-EMUDBG: out(4): ALT guard test - not ready to communicate.\n");

						writeword (BReg, Wdesc);
						writeword (index (WPtr, Pointer_s), CReg);
                                                deschedule ();

						/* The alt is waiting. Rechedule it? */
						if (altState != Ready_p)
						{
							/* The alt has not already been rescheduled. */
#ifdef EMUDEBUG
                                                        if (msgdebug || emudebug)
                                                        {
                                                                printf ("-I-EMUDBG: out(5): ALT state=Ready_p.\n");
                                                                printf ("-I-EMUDBG: out(6): Reschedule ALT process (Wdesc=#%08X, IPtr=#%08X).\n",
                                                                                                otherWdesc, word (index (otherWPtr, Iptr_s)));
                                                        }
#endif
							writeword (index (otherWPtr, State_s), Ready_p);
							/* ISM 8.7.3: only a Waiting ALT is descheduled;
							 * an Enabling one is running or queued. */
							if (Waiting_p == altState)
								t4_wake (otherWdesc);
						}
          				}
					else
					{
						/* Ready. */
                                                MSGDBG ("-I-EMUDBG: out(4): Ready, communicate.\n");
                                                movebytes_int (otherPtr, CReg, AReg);
                                                CReg = CReg + BytesRead(CReg, AReg);
						writeword (BReg, NotProcess_p);
						schedule (otherWdesc);
					}
				}
			   }
			   else
			   {
				/* Link communication. */
                                MSGDBG3 ("-I-EMUDBG: out(2): Link%d communication. Old channel word=#%08X.\n", TheLink(BReg), word (BReg));

                                if (serve && (Link0Out == BReg))
                                        goto DescheduleOut;

                                if (send_channel (&Link[TheLink(BReg)].Out, CReg, AReg))
                                {
DescheduleOut:
				        writeword (BReg, Wdesc);
                                        Link[TheLink(BReg)].Out.Address = CReg;
                                        Link[TheLink(BReg)].Out.Length  = AReg;
                                        PROFILE(Link[TheLink(BReg)].Out.IOBytes += AReg);
                                        deschedule ();
                                }
			   }
			   break;
		case 0x0c: /* sub         */
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   AReg = t4_esub32 (BReg, AReg);
			   if (t4_overflow == TRUE)
				SetError;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x0d: /* startp      */
			   temp = GetDescWPtr(AReg);
			   IPtr++;
			   writeword (index (temp, Iptr_s), (IPtr + BReg));
			   schedule (temp | ProcPriority);
			   break;
		case 0x0e: /* outbyte     */
                           PROFILE(profile[PRO_CHANOUT]++);
			   MSGDBG2 ("-I-EMUDBG: outbyte: Channel=#%08X.\n", BReg);
			   IPtr++;
                           if (__builtin_expect (T9K_EXTCHAN(BReg), 0))
                           {
                                writeword (WPtr, AReg);         /* pw.Temp */
                                t9k_ext_out (BReg, WPtr, 1, T9K_IO_FIXED);
                           }
                           else if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
                           {
			        MSGDBG3 ("-W-EMUDBG: Warning - doing OUTWORD on Link%dIn. Old channel word=#%08X.\n", TheLink(BReg), word (BReg));
				/* Link communication. */
				writeword (BReg, Wdesc);
				writeword (WPtr, AReg);
                                Link0InDest   = WPtr;
                                Link0InLength = 1;
                                PROFILE(Link[0].In.IOBytes++);
                                deschedule ();
                           }
			   else if (!IsLinkOut(BReg))
			   {
				/* Internal communication. */
				otherWdesc = word (BReg);
                                if (__builtin_expect (T9K_RESCHAN_P == otherWdesc, 0))
                                {
                                        t9k_internal_claim (BReg);
                                        otherWdesc = NotProcess_p;
                                }
				if (otherWdesc == NotProcess_p)
				{
					/* Not ready. */
					writeword (BReg, Wdesc);
					writeword (WPtr, AReg);
					writeword (index (WPtr, Pointer_s), WPtr);
                                        deschedule ();
				}
				else
				{
					/* Ready. */
                                        otherWPtr = GetDescWPtr(otherWdesc);
                                        T4DEBUG(checkWPtr ("OUTBYTE", otherWPtr));
					altState = otherPtr = word (index (otherWPtr, Pointer_s));
					if ((altState & 0xfffffffc) == MostNeg)
					{
						/* ALT guard test - not ready to communicate. */

						writeword (BReg, Wdesc);
						writeword (WPtr, AReg);
						writeword (index (WPtr, Pointer_s), WPtr);
                                                deschedule ();

						/* The alt is waiting. Rechedule it? */
						if (altState != Ready_p)
						{
							/* The alt has not already been rescheduled. */
							writeword (index (otherWPtr, State_s), Ready_p);
							/* ISM 8.7.3: only a Waiting ALT is descheduled;
							 * an Enabling one is running or queued. */
							if (Waiting_p == altState)
								t4_wake (otherWdesc);
						}
          				}
					else
					{
						/* Ready. */
						writebyte (otherPtr, AReg);
						writeword (BReg, NotProcess_p);
                                                CReg = otherPtr + BytesPerWord;
						schedule (otherWdesc);
					}
				}
			   }
			   else
			   {
				/* Link communication. */
				writeword (WPtr, AReg);

                                if (serve && (Link0Out == BReg))
                                        goto DescheduleOutByte;

                                if (send_channel (&Link[TheLink(BReg)].Out, WPtr, 1))
                                {
DescheduleOutByte:
				        writeword (BReg, Wdesc);
                                        Link[TheLink(BReg)].Out.Address = WPtr;
                                        Link[TheLink(BReg)].Out.Length  = 1;
                                        PROFILE(Link[TheLink(BReg)].Out.IOBytes++);
                                        deschedule ();
                                }
			   }
			   break;
		case 0x0f: /* outword     */
                           PROFILE(profile[PRO_CHANOUT] += 4);
			   MSGDBG2 ("-I-EMUDBG: outword(1): Channel=#%08X.\n", BReg);
			   IPtr++;
                           if (__builtin_expect (T9K_EXTCHAN(BReg), 0))
                           {
                                writeword (WPtr, AReg);         /* pw.Temp */
                                t9k_ext_out (BReg, WPtr, 4, T9K_IO_FIXED);
                           }
                           else if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
                           {
			        MSGDBG3 ("-W-EMUDBG: Warning - doing OUTWORD on Link%dIn. Old channel word=#%08X.\n", TheLink(BReg), word (BReg));
				/* Link communication. */
				writeword (BReg, Wdesc);
				writeword (WPtr, AReg);
                                Link0InDest   = WPtr;
                                Link0InLength = 4;
                                PROFILE(Link[0].In.IOBytes += 4);
                                deschedule ();
                           }
			   else if (!IsLinkOut(BReg))
			   {
				/* Internal communication. */
				otherWdesc = word (BReg);
                                MSGDBG2 ("-I-EMUDBG: outword(2): Internal communication. Channel word=#%08X.\n", otherWdesc);
                                if (__builtin_expect (T9K_RESCHAN_P == otherWdesc, 0))
                                {
                                        t9k_internal_claim (BReg);
                                        otherWdesc = NotProcess_p;
                                }
				if (otherWdesc == NotProcess_p)
				{
                                        MSGDBG ("-I-EMUDBG: outword(3): Not ready.\n");
					/* Not ready. */
					writeword (BReg, Wdesc);
					writeword (WPtr, AReg);
					writeword (index (WPtr, Pointer_s), WPtr);
                                        deschedule ();
				}
				else
				{
					/* Ready. */
                                        otherWPtr = GetDescWPtr(otherWdesc);
                                        T4DEBUG(checkWPtr ("OUTWORD", otherWPtr));
					altState = otherPtr =  word (index (otherWPtr, State_s));
					if ((altState & 0xfffffffc) == MostNeg)
					{
						/* ALT guard test - not ready to communicate. */
                                                MSGDBG ("-I-EMUDBG: outword(3): ALT guard test - not ready.\n");

						writeword (BReg, Wdesc);
						writeword (WPtr, AReg);
						writeword (index (WPtr, Pointer_s), WPtr);
                                                deschedule ();

						/* The alt is waiting. Rechedule it? */
						if (altState != Ready_p)
						{
#ifdef EMUDEBUG
                                                        if (msgdebug || emudebug)
                                                        {
                                                                printf ("-I-EMUDBG: outword(4): ALT state=Ready_p.\n");
                                                                printf ("-I-EMUDBG: outword(5): Reschedule ALT process (Wdesc=#%08X, IPtr=#%08X).\n",
                                                                                                otherWdesc, word (index (otherWPtr, Iptr_s)));
                                                        }
#endif
							/* The alt has not already been rescheduled. */
							writeword (index (otherWPtr, State_s), Ready_p);
							/* ISM 8.7.3: only a Waiting ALT is descheduled;
							 * an Enabling one is running or queued. */
							if (Waiting_p == altState)
								t4_wake (otherWdesc);
						}
          				}
					else
					{
                                                MSGDBG ("-I-EMUDBG: outword(3): Ready.\n");

						/* Ready. */
						writeword (otherPtr, AReg);
						writeword (BReg, NotProcess_p);
                                                CReg = otherPtr + BytesPerWord;
						schedule (otherWdesc);
					}
				}
			   }
			   else
			   {
				/* Link communication. */
                                MSGDBG3 ("-I-EMUDBG: out(2): Link%d communication. Old channel word=#%08X.\n", TheLink(BReg), word (BReg));
				writeword (WPtr, AReg);
                                if (serve && (Link0Out == BReg))
                                        goto DescheduleOutWord;
                                if (send_channel (&Link[TheLink(BReg)].Out, WPtr, 4))
                                {
DescheduleOutWord:
				        writeword (BReg, Wdesc);
                                        Link[TheLink(BReg)].Out.Address = WPtr;
                                        Link[TheLink(BReg)].Out.Length  = 4;
                                        PROFILE(Link[TheLink(BReg)].Out.IOBytes += 4);
                                        deschedule ();
                                }
			   }
			   break;
		case 0x10: /* seterr      */
			   SetError;
			   IPtr++;
			   break;
		case 0x12: /* XXX resetch     */
			   temp = AReg;
                           if (__builtin_expect (T9K_EXTCHAN(temp), 0))
                                AReg = t9k_ext_resetch (temp);
                           else if ((IsLinkIn(temp) || IsLinkOut(temp)) &&
                                    t9k_link_virtual (TheLink(temp)))
                                AReg = Undefined_p;     /* not in byte-stream mode */
                           else
                           {
			        AReg = word (temp);
                                reset_channel (temp);
                           }
			   IPtr++;
			   break;
		case 0x13: /* csub0       */
			   if (BReg >= AReg)
			   {
				SetError;
			   }
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x15: /* stopp       */
			   IPtr++;
                           deschedule ();
			   break;
		case 0x16: /* ladd        */
			   t4_overflow = FALSE;
			   t4_carry = CReg & 0x00000001;
			   AReg = t4_eadd32 (BReg, AReg);
			   if (t4_overflow == TRUE)
				SetError;
			   IPtr++;
			   break;
		case 0x17: /* stlb        */
			   BPtrReg[1] = AReg;
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x18: /* sthf        */
			   FPtrReg[0] = AReg;
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x19: /* norm        */
			   AReg = t4_norm64 (BReg, AReg);
			   BReg = t4_carry;
			   CReg = t4_normlen;
			   IPtr++;
			   break;
		case 0x1a: /* ldiv        */
			   if (CReg >= AReg)
			   {
                                AReg = BReg;
                                BReg = CReg;
				SetError;
			   }
			   else if (CReg != 0)
			   {
				t4_carry = 0;
				AReg = t4_longdiv (CReg, BReg, AReg);
				BReg = t4_carry;
			   }
			   else
			   {
				temp = BReg / AReg;
				temp2 = BReg % AReg;
				AReg = temp;
				BReg = temp2;
			   }
                           CReg = BReg;
			   IPtr++;
			   break;
		case 0x1b: /* ldpi        */
			   IPtr++;
			   AReg = IPtr + AReg;
			   break;
		case 0x1c: /* stlf        */
			   FPtrReg[1] = AReg;
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x1d: /* xdble       */
			   CReg = BReg;
			   if (INT32(AReg) < 0)
			   {
				BReg = -1;
			   }
			   else
			   {
				BReg = 0;
			   }
			   IPtr++;
			   break;
		case 0x1e: /* ldpri       */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = ProcPriority;
			   IPtr++;
			   break;
		case 0x1f: /* rem         */
			   if (AReg==0)
                           {
                                AReg = BReg;
                                temp = CReg;
				SetError;
                           }
                           else if ((INT32(AReg)==-1) && (BReg==0x80000000))
                           {
                                AReg = 0x00000000;
                                temp = AReg;
                           }
			   else
                           {
				AReg = INT32(BReg) % INT32(AReg);
                                temp = abs (INT32(AReg));
                           }
                           BReg = CReg;
                           CReg = temp;
			   IPtr++;
			   break;
		case 0x20: /* ret         */
			   {
			       uint32_t return_addr = word(WPtr);
			       uint32_t new_wptr = index(WPtr, 4);
			       uint32_t old_iptr = IPtr;
			       /* Log kernel function returns */
			       if (rare_hooks && noisy_boot_trace_enabled() &&
			           IPtr >= 0x80000000 && IPtr < 0x81000000) {
			           printf("[RET] from=0x%08X to=0x%08X WPtr=0x%08X→0x%08X\n",
			                  IPtr, return_addr, WPtr, new_wptr);
			       }
			       /* Q3 WPtr + null ret: dump frame (Phase 9 hard-P debug) */
			       if (return_addr < MemStart && (WPtr >> 30) == 3u) {
				       uint32_t pa = 0;
				       int pr = t9k_probe(WPtr, T9K_ACC_READ, &pa);
				       fprintf(stderr,
					       "[Q3-RET0] from=0x%08X WPtr=0x%08X is_p=%d probe=%d pa=0x%08X "
					       "w[0..3]=%08X %08X %08X %08X A=%08X B=%08X C=%08X\n",
					       old_iptr, WPtr, t9k_in_p(), pr, pa,
					       word_int(WPtr), word_int(index(WPtr, 1)),
					       word_int(index(WPtr, 2)), word_int(index(WPtr, 3)),
					       AReg, BReg, CReg);
			       }
			       /* Validate return address */
			       if (return_addr == 0x2FFA2FFA || return_addr == 0x80000000) {
			           /* Throttled like the AJW check above. */
			           static unsigned long bad_ret_count;
			           bad_ret_count++;
			           if (bad_ret_count <= 16 ||
			               (bad_ret_count & (bad_ret_count - 1)) == 0)
			               printf("[ERROR] Invalid return address 0x%08X at IPtr=0x%08X WPtr=0x%08X [%lu so far]\n",
			                      return_addr, IPtr, WPtr, bad_ret_count);
			       }
			       /* TCWG spec: "The ret instruction restores the Iptr and adjusts the
			        * workspace pointer to deallocate the four locations." */
			   if (return_addr < MemStart &&
			       !(t4_sd_boot_enabled() &&
			         return_addr - BOOTBANK_BASE < BOOTBANK_SIZE)) {
			       fault_history_dump("wild RET target");
			       static int wj_ret = 0;
			           if (wj_ret < 20) {
			               printf("[WILDJUMP] RET from=0x%08X to=0x%08X (ret<MemStart) WPtr=0x%08X\n",
			                      old_iptr, return_addr, WPtr);
			               wj_ret++;
			           }
			       }
			       IPtr = return_addr;
			       UpdateWdescReg (new_wptr | ProcPriority);
                               T4DEBUG(checkWPtr ("RET", WPtr));
			       /* Cache debug tracing for RET */
			       if (rare_hooks && cache_debug_trace_enabled &&
			           (old_iptr >= cache_debug_trace_start && old_iptr < cache_debug_trace_end)) {
			           fprintf(stderr,
			                   "[RET_DEBUG] from=0x%08X to=0x%08X WPtr=0x%08X\n",
			                   old_iptr, IPtr, WPtr);
			       }
			   }
			   break;
		case 0x21: /* lend        */ /****/
			   temp = word (index (BReg, 1));
			   IPtr++;
			   if (temp > 1)
			   {
				writeword (index (BReg, 1), (temp - 1));
                                CReg = word (BReg) + 1;
				writeword (BReg, CReg);
				IPtr =  IPtr - AReg;
				D_check();
			   }
			   else
			   {
                                CReg = 0;
				writeword (index (BReg, 1), (temp - 1));
			   }
			   break;
		case 0x22: /* ldtimer     */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = ClockReg[ProcPriority];
			   IPtr++;
			   break;
                case 0x23: /* testlds     */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = STATUSReg;
                           IPtr++;
                           break;
                case 0x24: /* testlde     */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = EReg;
                           IPtr++;
                           break;
                case 0x25: /* testldd     */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = DReg;
                           IPtr++;
                           break;
                case 0x26: /* teststs     */
                           STATUSReg = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x27: /* testste     */
                           EReg = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x28: /* teststd     */
                           DReg = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
		case 0x29: /* testerr     */
			   CReg = BReg;
			   BReg = AReg;
			   if (ReadError)
			   {
				AReg = false_t;
			   }
			   else
			   {
				AReg = true_t;
			   }
			   ClearError;
			   IPtr++;
			   break;
		case 0x2a: /* testpranal  */
			   CReg = BReg;
			   BReg = AReg;
			   if (analyse) AReg = true_t; else AReg = false_t;
			   IPtr++;
			   break;
		case 0x2b: /* XXX: missing Waiting_p tin         */
			   IPtr++;
			   if (INT32(ClockReg[ProcPriority] - AReg) > 0)
			        ;
			   else
			   {
				insert (AReg);
                                deschedule ();;
			   }
			   break;
		case 0x2c: /* div         */
			   if ((AReg==0) || ((INT32(AReg)==-1)&&(BReg==0x80000000)))
                           {
                                temp = CReg;
				SetError;
                           }
			   else
                           {
                                temp  = abs (INT32(AReg));
                                temp2 = abs (INT32(BReg));
				AReg  = INT32(BReg) / INT32(AReg);
                                /* kudos to M.Bruestle */
                                temp  = temp2 - (abs (INT32(AReg)) | 1) * temp;
                           }
			   BReg = CReg;
                           CReg = temp;
			   IPtr++;
			   break;
		case 0x2d: /* testhardchan */
                           /* Shift BReg into the selected link channel's
                            * register stack: Count -> Ptr -> DBuff -> AReg.
                            * PtrReg/CountReg are the live transfer pointer
                            * and count, so - as on silicon - this is only
                            * valid while no transfer is in progress. */
                           if (!IsLinkIn(AReg) && !IsLinkOut(AReg))
                           {
                                printf ("-E-EMU414: testhardchan on non-link channel #%08X.\n", AReg);
                                processor_state ();
                                handler (-1);
                                break;
                           }
                           {
                                Channel *chan = IsLinkIn(AReg) ? &Link[TheLink(AReg)].In
                                                               : &Link[TheLink(AReg)].Out;
                                AReg           = chan->DBuffReg;
                                chan->DBuffReg = chan->Address;
                                chan->Address  = chan->Length;
                                chan->Length   = BReg;
                           }
                           BReg = CReg;
			   IPtr++;
			   break;
		case 0x2e: /* XXX dist        */
			   EMUDBG2 ("-I-EMUDBG: dist(1): Time=%8X.\n", CReg);
			   temp = ClockReg[ProcPriority];
			   if ((BReg == true_t) &&
                               (INT32(temp - CReg)>=0) &&
                               (word (index (WPtr, Temp_s)) == NoneSelected_o))
			   {
				EMUDBG2 ("-I-EMUDBG: dist(2): Taking branch #%8X.\n", AReg);
				writeword (index (WPtr, Temp_s), AReg);
				AReg = true_t;
                                CReg = TimeNotSet_p;
			   }
			   else
			   {
				EMUDBG ("-I-EMUDBG: dist(2): Not taking this branch.\n");
				AReg = false_t;
			   }
			   IPtr++;
			   break;
		case 0x2f: /* XXX: support ALT construct on Link0 disc        */
			   EMUDBG2 ("-I-EMUDBG: disc(1): Channel=#%08X.\n", CReg);
                           if (__builtin_expect (T9K_EXTCHAN(CReg), 0))
                           {
                                /* T9000 event or virtual channel */
                                if ((BReg != false_t) && t9k_ext_disc (CReg) &&
                                    (word (index (WPtr, Temp_s)) == NoneSelected_o))
                                {
                                        writeword (index (WPtr, Temp_s), AReg);
                                        AReg = true_t;
                                }
                                else
                                        AReg = false_t;
                                IPtr++;
                                break;
                           }
                           if (serve && (CReg == Link0In))
          		   {
				EMUDBG ("-I-EMUDBG: disc(2): Link.\n");

				/* External link. */
          			if (FromServerLen > 0)
          				temp = TRUE;
          			else
          				temp = FALSE;
          		   }
          		   else
          		   {
                                otherWdesc = word (CReg);
#ifdef EMUDEBUG
                                if (emudebug)
                                {
				        printf ("-I-EMUDBG: disc(2): ");
                                        if (IsLinkIn(CReg))
				                printf ("Link%dIn.", TheLink(CReg));
                                        else
				                printf ("Internal channel.");
                                        printf (" Channel word=#%08X.\n", otherWdesc);
                                }
#endif
				/* Internal/Link channel. */
          			if (otherWdesc == NotProcess_p)
				{
                                        EMUDBG ("-I-EMUDBG: disc(3): Channel not ready.\n");

					/* Channel not ready. */
					temp = FALSE;
				}
				else if (otherWdesc == Wdesc)
				{
                                        EMUDBG ("-I-EMUDBG: disc(3): Channel not ready, but this process enabled it.\n");

					/* Channel not ready, but was inited by this process's enbc. */
                                        temp = FALSE;

					/* Reset channel word to NotProcess_p to avoid confusion. */
					writeword (CReg, NotProcess_p);
				}
          			else
				{
                                        EMUDBG ("-I-EMUDBG: disc(3): Channel ready.\n");

					/* Channel ready. */
          				temp = TRUE;
				}
          		   }
			   if ((BReg == true_t) &&
                               (temp == TRUE) &&
                               (word (index (WPtr, Temp_s)) == NoneSelected_o))
			   {
				EMUDBG2 ("-I-EMUDBG: disc(4): Taking branch #%8X.\n", AReg);
				writeword (index (WPtr, Temp_s), AReg);
				AReg = true_t;
			   }
			   else
			   {
				EMUDBG ("-I-EMUDBG: disc(3): Not taking this branch.\n");
				AReg = false_t;
			   }
			   IPtr++;
			   break;
		case 0x30: /* diss        */
			   if ((BReg == true_t) && (word (index (WPtr, Temp_s)) == NoneSelected_o))
			   {
				writeword (index (WPtr, Temp_s), AReg);
				AReg = true_t;
			   }
			   else
			   {
				AReg = false_t;
			   }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x31: /* lmul        */
			   t4_overflow = FALSE;
			   t4_carry = CReg;
			   AReg = t4_mul32 (BReg, AReg);
			   BReg = t4_carry;
                           CReg = BReg;
			   IPtr++;
			   break;
		case 0x32: /* not         */
			   AReg = ~ AReg;
			   IPtr++;
			   break;
		case 0x33: /* xor         */
			   AReg = BReg ^ AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x34: /* bcnt        */
			   AReg = AReg * BytesPerWord;
			   IPtr++;
			   break;
		case 0x35: /* lshr        */
			   AReg = t4_shr64 (CReg, BReg, AReg);
			   BReg = t4_carry;
                           CReg = BReg;
			   IPtr++;
			   break;
		case 0x36: /* lshl        */
			   AReg = t4_shl64 (CReg, BReg, AReg);
			   BReg = t4_carry;
                           CReg = BReg;
			   IPtr++;
			   break;
		case 0x37: /* lsum        */
			   t4_overflow = FALSE;
			   t4_carry = CReg & 0x00000001;
			   AReg = t4_add32 (BReg, AReg);
			   BReg = t4_carry;
			   IPtr++;
			   break;
		case 0x38: /* lsub        */
			   t4_overflow = FALSE;
			   t4_carry = CReg & 0x00000001;
			   AReg = t4_esub32 (BReg, AReg);
			   if (t4_overflow == TRUE)
				SetError;
			   IPtr++;
			   break;
		case 0x39: /* runp        */
			   IPtr++;
			   schedule (AReg);
			   break;
		case 0x3a: /* xword       */
                           /* ACWG preconditions */
                           /*   bitcount(AReg) = 1 /\ 2*AReg > BReg */
                           if (t4_bitcount (AReg) != 1)
                           {
                                AReg = Undefined_p;
                           }
                           else if (0 == (BReg & (~(AReg | (AReg - 1)))))
                           {
                                /* Bits are clear above the sign bit (AReg). */
                                if (AReg > BReg)
                                        AReg = BReg;
                                else
                                        AReg = BReg - (2*AReg);
                           }
                           else
                           {
                                /* T425 implementation. */
                                EMUDBG ("-W-EMU414: Warning - XWORD undefined behavior!\n");
                                if (0 == (AReg & BReg))
				        AReg = BReg;
			        else
                                        AReg = BReg | ~(AReg - 1);
                           }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x3b: /* XXX sb          */
			   if (rare_hooks && sk_areg_trace_enabled &&
			       IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			       fprintf(stderr,
			               "[SKA] SB IPtr=0x%08X AReg=0x%08X BReg=0x%08X\n",
			               IPtr, AReg, BReg);
                               if (AReg == 0x10000000) {
                                       fprintf(stderr,
                                               "[SKA] SB_UART_HIT IPtr=0x%08X BReg=0x%08X\n",
                                               IPtr, BReg);
                               }
			   }
			   if (rare_hooks && sk_store_trace_enabled && sk_entry_iptr &&
			       IPtr >= sk_entry_iptr && IPtr <= (sk_entry_iptr + 0x200)) {
			       printf("[SKSB] IPtr=0x%08X addr=0x%08X BReg=0x%08X\n",
			              IPtr, AReg, BReg);
			   }
			   writebyte (AReg, BReg);
			   AReg = CReg;
                           CReg = 0;
			   IPtr++;
			   break;
		case 0x3c: /* gajw        */
                           /* XXX: proc prio toggle trick of AReg lsb=1       */
                           T4DEBUG(checkWordAligned ("GAJW", AReg));
			   /* ISM 9.3: restartable stack-extension trap */
			   if (unlikely(t9k_prot.is_p))
				   t9k_check_wptr(AReg & ~3u, Icache[islot].ip);
			   temp = AReg;
                           if (rare_hooks && wptr_trace_enabled) {
                                   printf("[WPtr] GAJW IPtr=0x%08X WPtr=0x%08X→0x%08X\n",
                                          IPtr, WPtr, temp);
                           }
			   AReg = WPtr;
			   UpdateWdescReg (temp | ProcPriority);
                           T4DEBUG(checkWPtr ("GAJW", WPtr));
			   IPtr++;
			   break;
		case 0x3d: /* savel       */
			   writeword (index (AReg, 0), FPtrReg[1]);
			   writeword (index (AReg, 1), BPtrReg[1]);
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x3e: /* saveh       */
			   writeword (index (AReg, 0), FPtrReg[0]);
			   writeword (index (AReg, 1), BPtrReg[0]);
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x3f: /* wcnt        */
			   CReg = BReg;
			   BReg = AReg & ByteSelectMask;
			   AReg = INT32(AReg) >> 2;
			   IPtr++;
			   break;
		case 0x40: /* shr         */
			   if (AReg < BitsPerWord)
				AReg = BReg >> AReg;
			   else
				AReg = 0;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x41: /* shl         */
			   if (AReg < BitsPerWord)
				AReg = BReg << AReg;
			   else
				AReg = 0;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x42: /* mint        */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = 0x80000000;
			   IPtr++;
			   break;
		case 0x43: /* alt         */
			   EMUDBG ("-I-EMUDBG: alt: (W-3)=Enabling_p\n");
			   writeword (index (WPtr, State_s), Enabling_p);
			   IPtr++;
			   break;
		case 0x44: /* altwt       */
#ifdef EMUDEBUG
                           if (emudebug)
                           {
			        printf ("-I-EMUDBG: altwt(1): (W  )=NoneSelected_o\n");
			        printf ("-I-EMUDBG: altwt(2): (W-3)=#%08X\n", word (index (WPtr, State_s)));
                           }
#endif
			   writeword (index (WPtr, Temp_s), NoneSelected_o);
			   IPtr++;
			   if ((word (index (WPtr, State_s))) != Ready_p)
			   {
				/* No guards are ready, so deschedule process. */
				EMUDBG ("-I-EMUDBG: altwt(3): (W-3)=Waiting_p\n");
				writeword (index (WPtr, State_s), Waiting_p);
                                deschedule ();
			   }
			   break;
		case 0x45: /* altend      */
			   EMUDBG2 ("-I-EMUDBG: altend: IPtr+#%8X.\n", word (index (WPtr,0)));
			   IPtr++;
			   IPtr = IPtr + word (index (WPtr, Temp_s));
			   break;
		case 0x46: /* and         */
			   AReg = BReg & AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x47: /* enbt        */
			   EMUDBG2 ("-I-EMUDBG: enbt(1): Channel=%08X.\n", BReg);
			   if ((AReg == true_t) && (word (index (WPtr, TLink_s)) == TimeNotSet_p))
			   {
				EMUDBG ("-I-EMUDBG: enbt(2): Time not yet set.\n");
				/* Set ALT time to this guard's time. */
				writeword (index (WPtr, TLink_s), TimeSet_p);
				writeword (index (WPtr, Time_s), BReg);
			   }
			   else if ((AReg == true_t) &&
                                    (word (index (WPtr, TLink_s)) == TimeSet_p) &&
                                    (INT32(BReg - word (index (WPtr, Time_s))) >= 0))
			   {
				EMUDBG ("-I-EMUDBG: enbt(2): Time already set earlier than or equal to this one.\n");
				/* ALT time is before this guard's time. Ignore. */
			   }
			   else if ((AReg == true_t) && (word (index (WPtr, TLink_s)) == TimeSet_p))
			   {
				EMUDBG ("-I-EMUDBG: enbt(2): Time already set, but later than this one.\n");
				/* ALT time is after this guard's time. Replace ALT time. */
				writeword (index (WPtr, Time_s), BReg);
			   }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x48: /* XXX: support ALT construct on Link0  enbc        */
			   EMUDBG2 ("-I-EMUDBG: enbc(1): Channel=#%08X.\n", BReg);
                           if (__builtin_expect (T9K_EXTCHAN(BReg), 0))
                           {
                                if ((AReg != false_t) && t9k_ext_enbc (BReg))
                                        writeword (index (WPtr, State_s), Ready_p);
                                BReg = CReg;
                                IPtr++;
                                break;
                           }
                           otherWdesc = word (BReg);
			   if ((AReg == true_t) && (otherWdesc == NotProcess_p))
			   {
				EMUDBG ("-I-EMUDBG: enbc(2): Link or non-waiting channel.\n");
				/* Link or unwaiting channel. */
                                int link = TheLink(BReg);
				if (serve && (BReg == Link0In))
				{
					EMUDBG ("-I-EMUDBG: enbc(3): Link0In.\n");
					/* Link. */
					if (FromServerLen > 0)
					{
						EMUDBG ("-I-EMUDBG: enbc(4): Ready link: (W-3)=Ready_p\n");
						writeword (index (WPtr, State_s), Ready_p);
					}
					else
					{
						EMUDBG ("-I-EMUDBG: enbc(4): Empty link: Initialise link.\n");
						writeword (BReg, Wdesc);
					}
				}
				else if (IsLinkIn(BReg))
                                {

					EMUDBG2 ("-I-EMUDBG: enbc(3): Link%dIn.\n", link);
                                        if (0 == channel_ready (&Link[link].In))
                                        {
						EMUDBG ("-I-EMUDBG: enbc(4): Ready link: (W-3)=Ready_p\n");
						writeword (index (WPtr, State_s), Ready_p);
                                                /* Mark channel control word */
					        writeword (BReg, IdleProcess_p);
                                        }
                                        else
                                        {
						EMUDBG ("-I-EMUDBG: enbc(4): Empty link: Initialise link.\n");
					        writeword (BReg, Wdesc);
                                        }
                                }
#ifndef NDEBUG
                                else if (IsLinkOut(BReg))
                                {
                                        printf ("-E-EMU414: enbc on Link%dOut.\n", link);
                                        handler (-1);
                                }
#endif
                                else
                                {
                                        /* Initialise with Wdesc. */
				        writeword (BReg, Wdesc);
                                }
			   }
			   else if ((AReg == true_t) && (otherWdesc == Wdesc))
			   {
				EMUDBG ("-I-EMUDBG: enbc(2): This process enabled the channel.\n");
				/* This process initialised the channel. Do nothing. */
				;
			   }
			   else if (AReg == true_t)
			   {
#ifndef NDEBUG
                                if (IsLinkIn(BReg))
                                {
                                        /* XXX Link hardware already marked it ??? */
                                        if (IdleProcess_p != otherWdesc)
                                        {
                                                printf ("-E-EMU414: enbc(2): Waiting Wdesc=#%08X on external Link%dIn.\n", otherWdesc, TheLink(BReg));
                                                handler (-1);
                                        }
                                }
#endif
				EMUDBG ("-I-EMUDBG: enbc(2): Waiting internal channel: (W-3)=Ready_p\n");
				/* Waiting internal channel. */
				writeword (index (WPtr, State_s), Ready_p);
			   }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x49: /* enbs        */
                           /* Any non-zero value is valid. */
			   if (AReg)
                                writeword (index (WPtr, State_s), Ready_p);
			   IPtr++;
			   break;
		case 0x4a: /* move        */
                           if (INT32(AReg) > 0)
                           {
                                movebytes_int (BReg, CReg, AReg);
                                CReg = CReg + WordsRead(CReg, AReg) * BytesPerWord;
                           }
                           else
                           {
                                AReg = BReg = CReg = Undefined_p;
                           }
			   IPtr++;
			   break;
		case 0x4b: /* or          */
			   AReg = BReg | AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x4c: /* csngl       */
			   if (((INT32(AReg)<0) && (INT32(BReg)!=-1)) ||
                               ((INT32(AReg)>=0) && (BReg!=0)))
			   {
				SetError;
			   }
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x4d: /* ccnt1       */
			   if (BReg == 0)
			   {
				SetError;
			   }
			   else if (BReg > AReg)
			   {
				SetError;
			   }
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x4e: /* talt        */
			   writeword (index (WPtr, State_s), Enabling_p);
			   writeword (index (WPtr, TLink_s), TimeNotSet_p);
			   IPtr++;
			   break;
		case 0x4f: /* ldiff       */
			   t4_overflow = FALSE;
			   t4_carry = CReg & 0x00000001;
			   AReg = t4_sub32 (BReg, AReg);
			   BReg = t4_carry;
			   IPtr++;
			   break;
		case 0x50: /* sthb        */
			   BPtrReg[0] = AReg;
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x51: /* taltwt      */
#ifdef EMUDEBUG
                           if (emudebug)
                           {
			        printf ("-I-EMUDBG: taltwt(1): (W  )=NoneSelected_o\n");
			        printf ("-I-EMUDBG: taltwt(2): (W-3)=#%08X\n", word (index (WPtr, State_s)));
                           }
#endif
			   writeword (index (WPtr, Temp_s), NoneSelected_o);
			   IPtr++;
			   if ((word (index (WPtr, State_s))) != Ready_p)
			   {
				/* No guards are ready, so deschedule process, after putting time in timer queue. */
#ifdef EMUDEBUG
                                if (emudebug)
                                {
				        printf ("-I-EMUDBG: taltwt(3): (W-3)=Waiting_p\n");
				        printf ("-I-EMUDBG: taltwt(3): Waiting until #%8X.\n", word (index (WPtr, Time_s)));
                                }
#endif
				/* Put time into timer queue. */
				temp = word (index (WPtr, Time_s));
				insert (temp);

				writeword (index (WPtr, State_s), Waiting_p);
                                deschedule ();
			   }
			   break;
		case 0x52: /* sum         */
			   AReg = BReg + AReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x53: /* mul         */
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   AReg = t4_emul32 (BReg, AReg);
			   if (t4_overflow == TRUE)
				SetError;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x54: /* sttimer     */
			   ClockReg[0] = AReg;
			   ClockReg[1] = AReg;
			   Timers = TimersGo;
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x55: /* XXX stoperr     */
			   IPtr++;
			   if (ReadError)
			   {
                                deschedule ();
			   }
			   break;
		case 0x56: /* cword       */
                           if (t4_bitcount (AReg) != 1)
                                ;
                           else if (AReg==MostNeg)
                                ;
                           else if ((INT32(BReg)>=INT32(AReg)) || (INT32(BReg)<INT32(-AReg)))
			   {
                                /* ST20CORE implementation. */
				SetError;
			   }
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0x57: /* clrhalterr  */
                           ClearHaltOnError;
			   IPtr++;
			   break;
		case 0x58: /* sethalterr  */
                           SetHaltOnError;
			   IPtr++;
			   break;
		case 0x59: /* testhalterr */
			   CReg = BReg;
			   BReg = AReg;
			   if (ReadHaltOnError)
			   {
				AReg = true_t;
			   }
			   else
			   {
				AReg = false_t;
			   }
			   IPtr++;
			   break;
		case 0x5a: /* dup    */
		           BADCODE(IsT414);
                           CReg = BReg;
                           BReg = AReg;
		           IPtr++;
		           break;
		case 0x5b: /* XXX move2dinit    */
		           BADCODE(IsT414);
                           m2dLength = AReg;
                           m2dDestStride = BReg;
                           m2dSourceStride = CReg;
		           IPtr++;
		           break;

#define m2dWidth                AReg
#define m2dDestAddress          BReg
#define m2dSourceAddress        CReg

		case 0x5c: /* XXX move2dall    */
		           BADCODE(IsT414);
                           {
                               int is_fb_flip = 0; /* screenshot via VGA register now */
                           if (INT32(m2dWidth) >= 0 && INT32(m2dLength) >= 0)
                           {
#ifdef T4_SDL_FB
                                /* Fast path: both src and dst entirely in FB */
                                if (m2dWidth > 0 && m2dLength > 0 &&
                                    fb_addr_in_range(m2dSourceAddress) &&
                                    fb_addr_in_range(m2dSourceAddress + (m2dLength-1) * m2dSourceStride + m2dWidth - 1) &&
                                    fb_addr_in_range(m2dDestAddress) &&
                                    fb_addr_in_range(m2dDestAddress + (m2dLength-1) * m2dDestStride + m2dWidth - 1))
                                {
                                    u_char *fb = vga_framebuffer;
                                    for (temp = 0; temp < m2dLength; temp++)
                                    {
                                        uint32_t doff, soff;
                                        if (!fb_addr_to_offset(m2dDestAddress, &doff) ||
                                            !fb_addr_to_offset(m2dSourceAddress, &soff))
                                            break;
                                        memmove(fb + doff, fb + soff, m2dWidth);
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                    }
                                    vga_dirty = 1;
                                }
                                else
#endif
                                {
                                if (is_fb_flip)
                                    fprintf(stderr, "[MOVE2D_SLOW] fb_flip: dst=0x%08X src=0x%08X w=%d len=%d\n",
                                            m2dDestAddress, m2dSourceAddress, m2dWidth, m2dLength);
                                for (temp = 0; temp < m2dLength; temp++)
                                {
                                        movebytes_int (m2dDestAddress, m2dSourceAddress, m2dWidth);
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                }
                                } /* close else block */
                           }
                           } /* end scope */
#ifndef NDEBUG
                           m2dSourceStride = m2dDestStride = m2dLength = Undefined_p;
#endif
		           IPtr++;
		           break;
		case 0x5d: /* XXX move2dnonzero    */
		           BADCODE(IsT414);
                           if (INT32(m2dWidth) >= 0 && INT32(m2dLength) >= 0)
                           {
#ifdef T4_SDL_FB
                                /* Fast path: both src and dst entirely in FB */
                                if (m2dWidth > 0 && m2dLength > 0 &&
                                    fb_addr_in_range(m2dSourceAddress) &&
                                    fb_addr_in_range(m2dSourceAddress + (m2dLength-1) * m2dSourceStride + m2dWidth - 1) &&
                                    fb_addr_in_range(m2dDestAddress) &&
                                    fb_addr_in_range(m2dDestAddress + (m2dLength-1) * m2dDestStride + m2dWidth - 1))
                                {
                                    u_char *fb = vga_framebuffer;
                                    for (temp = 0; temp < m2dLength; temp++)
                                    {
                                        uint32_t doff, soff;
                                        if (!fb_addr_to_offset(m2dDestAddress, &doff) ||
                                            !fb_addr_to_offset(m2dSourceAddress, &soff))
                                            break;
                                        u_char *sp = fb + soff;
                                        u_char *dp = fb + doff;
                                        for (temp2 = 0; temp2 < m2dWidth; temp2++)
                                            if (sp[temp2]) dp[temp2] = sp[temp2];
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                    }
                                    vga_dirty = 1;
                                }
                                else
#endif
                                for (temp = 0; temp < m2dLength; temp++)
                                {
                                        for (temp2 = 0; temp2 < m2dWidth; temp2++)
                                        {
                                                pixel = byte_int (m2dSourceAddress + temp2);
                                                if (pixel)
                                                        writebyte_int (m2dDestAddress + temp2, pixel);
                                        }
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                }
                           }
#ifndef NDEBUG
                           m2dSourceStride = m2dDestStride = m2dLength = Undefined_p;
#endif
		           IPtr++;
		           break;
		case 0x5e: /* XXX move2dzero    */
		           BADCODE(IsT414);
                           if (INT32(m2dWidth) >= 0 && INT32(m2dLength) >= 0)
                           {
#ifdef T4_SDL_FB
                                /* Fast path: both src and dst entirely in FB */
                                if (m2dWidth > 0 && m2dLength > 0 &&
                                    fb_addr_in_range(m2dSourceAddress) &&
                                    fb_addr_in_range(m2dSourceAddress + (m2dLength-1) * m2dSourceStride + m2dWidth - 1) &&
                                    fb_addr_in_range(m2dDestAddress) &&
                                    fb_addr_in_range(m2dDestAddress + (m2dLength-1) * m2dDestStride + m2dWidth - 1))
                                {
                                    u_char *fb = vga_framebuffer;
                                    for (temp = 0; temp < m2dLength; temp++)
                                    {
                                        uint32_t doff, soff;
                                        if (!fb_addr_to_offset(m2dDestAddress, &doff) ||
                                            !fb_addr_to_offset(m2dSourceAddress, &soff))
                                            break;
                                        u_char *sp = fb + soff;
                                        u_char *dp = fb + doff;
                                        for (temp2 = 0; temp2 < m2dWidth; temp2++)
                                            if (sp[temp2] == 0) dp[temp2] = 0;
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                    }
                                    vga_dirty = 1;
                                }
                                else
#endif
                                for (temp = 0; temp < m2dLength; temp++)
                                {
                                        for (temp2 = 0; temp2 < m2dWidth; temp2++)
                                        {
                                                pixel = byte_int (m2dSourceAddress + temp2);
                                                if (0 == pixel)
                                                        writebyte_int (m2dDestAddress + temp2, pixel);
                                        }
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                }
                           }
#ifndef NDEBUG
                           m2dSourceStride = m2dDestStride = m2dLength = Undefined_p;
#endif
		           IPtr++;
		           break;

#undef m2dWidth
#undef m2dDestAddress
#undef m2dSourceAddress

		case 0x5f: /* gtu - unsigned greater than (T800) */
			   BADCODE(IsT414);
			   /* AReg = (BReg >u AReg) ? 1 : 0 */
			   temp = ((uint32_t)BReg > (uint32_t)AReg) ? 1 : 0;
			   AReg = temp;
			   BReg = CReg;
			   CReg = Undefined_p;
			   IPtr++;
			   break;

		/* ---- IMS T9000 narrow load/store/extend group -------------
		 * These do not exist on the T414/T800/T805; they are modelled
		 * here so that code built with tcc -mt9000 can be executed and
		 * validated.  Memory is little-endian, matching the two-byte
		 * sequences the T805 code generator emits for the same access.
		 */
		case 0xb8: /* xbword - extend byte to word              (T9000) */
			   AReg = (uint32_t)(int32_t)(int8_t)(AReg & 0xFF);
			   IPtr++;
			   break;
		case 0xb9: /* lbx - load byte and sign extend           (T9000) */
			   AReg = (uint32_t)(int32_t)(int8_t)(byte (AReg) & 0xFF);
			   IPtr++;
			   break;
		case 0xc1: /* ssub - 16 bit word subscript              (T9000) */
			   /* AReg := AReg + 2*BReg (cf. bsub +1*, wsub +4*) */
			   AReg = AReg + 2 * BReg;
			   BReg = CReg;
			   CReg = Undefined_p;
			   IPtr++;
			   break;
		case 0xc8: /* ss - store sixteen                        (T9000) */
			   writebyte (AReg,     BReg        & 0xFF);
			   writebyte (AReg + 1, (BReg >> 8) & 0xFF);
			   AReg = CReg;
			   CReg = 0;
			   IPtr++;
			   break;
		case 0xca: /* ls - load sixteen (zero extended)         (T9000) */
			   AReg = (byte (AReg) & 0xFF)
			        | ((byte (AReg + 1) & 0xFF) << 8);
			   IPtr++;
			   break;
		case 0xf8: /* xsword - sign extend 16 bit word          (T9000) */
			   AReg = (uint32_t)(int32_t)(int16_t)(AReg & 0xFFFF);
			   IPtr++;
			   break;
		case 0xf9: /* lsx - load sixteen and sign extend        (T9000) */
			   AReg = (uint32_t)(int32_t)(int16_t)
			          (((byte (AReg) & 0xFF)
			            | ((byte (AReg + 1) & 0xFF) << 8)) & 0xFFFF);
			   IPtr++;
			   break;

		case 0x63: /* unpacksn    */
                           BADCODE(IsT800);
#ifdef EMUDEBUG
                           if (emudebug)
                           {
                                r32temp.bits = AReg;
                                printf ("\t\t\t\t\t\t\t  AReg %08X (%.7e)\n", AReg, r32temp.fp);
                           }
#endif
                           temp = AReg;
	                   CReg = BReg << 2;
	                   AReg = (temp & 0x007fffff) << 8;
	                   BReg = (temp & 0x7f800000) >> 23;
	                   if (t4_iszero (temp))
	                        temp2 = 0x00000000;
	                   else if (t4_isinf (temp))
	                        temp2 = 0x00000002;
	                   else if (t4_isnan (temp))
	                        temp2 = 0x00000003;
	                   else if ((0 == BReg) && (0 != AReg))
                           {
                                /* Denormalised. */
                                temp2 = 0x00000001;
                                BReg = 1;
                           }
                           else
                           {
                                /* Normalised. */
                                temp2 = 0x00000001;
                                AReg  = AReg | 0x80000000;
                           }
	                   CReg = CReg | temp2;
			   IPtr++;
			   break;
		case 0x6c: /* postnormsn  */
                           BADCODE(IsT800);
			   temp = (INT32(word (index (WPtr, Temp_s))) - INT32(CReg));
                           if (INT32(temp) <= -BitsPerWord)
                           {
                                /* kudos to M.Bruestle: too small. */
                                AReg = BReg = CReg = 0;
                           }
                           else if (INT32(temp) > 0x000000ff)
				CReg = 0x000000ff;
			   else if (INT32(temp) <= 0)
			   {
				temp  = 1 - INT32(temp);
				CReg  = 0;
                                temp2 = AReg;
				AReg  = t4_shr64 (BReg, AReg, temp);
                                AReg  = AReg | temp2;
				BReg  = t4_carry;
			   }
			   else
				CReg = temp;
			   IPtr++;
			   break;
		case 0x6d: /* roundsn     */
                           BADCODE(IsT800);
			   if (INT32(CReg) >= 0x000000ff)
			   {
				AReg = t4_infinity ();
				CReg = BReg << 1;
			   }
			   else
			   {
                                /* kudos to M.Bruestle */
                                temp  = ((CReg & 0x000001ff) << 23)|((BReg & 0x7fffff00) >> 8);
                                if ((BReg & 0x80) == 0)
                                        AReg = temp;
                                else if ((AReg | (BReg & 0x7f)) != 0)
                                        AReg = temp + 1;
                                else
                                        AReg = temp + (temp & 1);
				BReg = AReg;
				CReg = CReg >> 9;
			   }
#ifdef EMUDEBUG
                           if (emudebug)
                           {
                                r32temp.bits = AReg;
                                printf ("\t\t\t\t\t\t\t  AReg %08X (%.7e)\n", AReg, r32temp.fp);
                           }
#endif
			   IPtr++;
			   break;
		case 0x71: /* ldinf       */
                           BADCODE(IsT800);
			   CReg = BReg;
			   BReg = AReg;
			   AReg = t4_infinity ();
			   IPtr++;
			   break;
		case 0x72: /* fmul        */
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   if ((AReg==0x80000000)&&(BReg==0x80000000))
                           {
                                t4_carry = AReg;
				SetError;
                           }
                           else
                                AReg = t4_fmul (AReg, BReg);
			   BReg = CReg;
                           CReg = t4_carry;
			   IPtr++;
			   break;
		case 0x73: /* cflerr      */
                           BADCODE(IsT800);
			   if ((t4_isinf (AReg)) || (t4_isnan (AReg)))
				SetError;
			   IPtr++;
			   break;
		case 0x74: /* crcword    */
		           BADCODE(IsT414);
                           for (temp = 0; temp < BitsPerWord; temp++)
                           {
			        AReg = t4_shl64 (BReg, AReg, 1);
                                BReg = t4_carry;
                                if (t4_carry64)
                                        BReg = BReg ^ CReg;
                           }
                           AReg = BReg;
                           BReg = CReg;
                           CReg = AReg;
		           IPtr++;
		           break;
		case 0x75: /* crcbyte    */
		           BADCODE(IsT414);
                           /* Data must be in the most significant byte of the word. */
                           for (temp = 0; temp < BitsPerByte; temp++)
                           {
			        AReg = t4_shl64 (BReg, AReg, 1);
                                BReg = t4_carry;
                                if (t4_carry64)
                                        BReg = BReg ^ CReg;
                           }
                           AReg = BReg;
                           BReg = CReg;
                           CReg = AReg;
		           IPtr++;
		           break;
		case 0x76: /* bitcnt    */
		           BADCODE(IsT414);
                           temp = t4_bitcount (AReg);
                           AReg = temp + BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x77: /* bitrevword    */
		           BADCODE(IsT414);
                           AReg = t4_bitreverse (AReg);
		           IPtr++;
		           break;
		case 0x78: /* bitrevnbits    */
		           BADCODE(IsT414);
                           if (AReg == 0)
                                AReg = 0;
                           else if (AReg <= BitsPerWord)
                                AReg = t4_bitreverse (BReg) >> (BitsPerWord - AReg);
                           else
                           {
                                /* kudos to M.Bruestle */
                                EMUDBG ("-W-EMU414: Warning - BITREVNBITS undefined behavior!\n");
                                if (AReg >= 2 * BitsPerWord)
                                        temp = 0;
                                else
                                        temp = t4_bitreverse (BReg) << (AReg - BitsPerWord);
                                AReg = temp;
                           }
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x79: /* pop - pop integer stack (T425, T9000 ISM p.398) */
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   break;
		case 0xb1: /* break */
			   IPtr++;
			   handle_j0_break();
			   break;
		case 0xb2: /* clrj0break */
			   EnableJ0BreakFlag = 0;
			   IPtr++;
			   break;
		case 0xb3: /* setj0break */
			   EnableJ0BreakFlag = 1;
			   IPtr++;
			   break;
		case 0xb4: /* testj0break */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = EnableJ0BreakFlag ? true_t : false_t;
			   IPtr++;
			   break;
		case 0x7a: /* timerdisableh */
			   TimerEnableHi = 0;
			   IPtr++;
			   break;
		case 0x7b: /* timerdisablel */
			   TimerEnableLo = 0;
			   IPtr++;
			   break;
		case 0x7c: /* timerenableh */
			   TimerEnableHi = 1;
			   IPtr++;
			   break;
		case 0x7d: /* timerenablel */
			   TimerEnableLo = 1;
			   IPtr++;
			   break;
		case 0x7e: /* ldmemstartval: the MemStart config register */
			   t9k_vcp_exec (OReg);
			   IPtr++;
			   break;
		case 0x80: /* XXX fpsttest -- M.Bruestle  */
                           temp = FAReg.length;
                           if (FAReg.length == FP_REAL64)
                           {
                                fp_popdb (&dbtemp1);
                           }
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                                fp_popsn (&sntemp1);
                                dbtemp1.bits = sntemp1.bits;
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpsttest)\n");
                                temp    = FP_REAL64;
                                dbtemp1 = DUndefined;
                           }
#endif
                           dbtemp2 = fp_state (temp, dbtemp1, &temp2);
                           writereal64 (AReg, dbtemp2);
                           writeword (index (AReg, 2), temp2);
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
                           IPtr++;
                           break;
		case 0x81: /* wsubdb    */
		           BADCODE(IsT414);
			   AReg = index (AReg, 2*BReg);
			   BReg = CReg;
		           IPtr++;
		           break;
		case 0x82: /* XXX fpldnldbi    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLDBI", AReg));
                           fp_pushdb (real64 (index (AReg, 2*BReg)));
                           AReg = CReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x83: /* fpchkerr    */
		           BADCODE(IsT414);
                           fp_syncexcept ();
                           if (FP_Error)
                           {
                                SetError;
                           }
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x84: /* fpstnldb    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPSTNLDB", AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL64)
#endif
                                fp_popdb (&dbtemp1);
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpstnldb)\n");
                                dbtemp1 = DUndefined;
                           }
#endif
                           writereal64 (AReg, dbtemp1);
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
                case 0x85: /* XXX fpldtest -- M.Bruestle */
                           dbtemp1 = real64 (AReg);
                           temp    = word (index (AReg, 2));
                           fp_setstate (dbtemp1, temp);
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE; /* XXX */
                           IPtr++;
                           break;
		case 0x86: /* XXX fpldnlsni    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLSNI", AReg));
                           fp_pushsn (real32 (index (AReg, BReg)));
                           AReg = CReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x87: /* fpadd    */
		           BADCODE(IsT414);
                           fp_dobinary (fp_adddb, fp_addsn);
		           IPtr++;
		           break;
		case 0x88: /* fpstnlsn    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPSTNLSN", AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                fp_popsn (&sntemp1);
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpstnlsn)\n");
                                sntemp1 = RUndefined;
                           }
#endif
                           writereal32 (AReg, sntemp1);
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x89: /* fpsub    */
		           BADCODE(IsT414);
                           fp_dobinary (fp_subdb, fp_subsn);
		           IPtr++;
		           break;
		case 0x8a: /* fpldnldb    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLDB", AReg));
                           fp_pushdb (real64 (AReg));
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x8b: /* fpmul    */
		           BADCODE(IsT414);
                           fp_dobinary (fp_muldb, fp_mulsn);
		           IPtr++;
		           break;
		case 0x8c: /* fpdiv    */
		           BADCODE(IsT414);
                           fp_dobinary (fp_divdb, fp_divsn);
		           IPtr++;
		           break;
		case 0x8e: /* fpldnlsn    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLSN", AReg));
                           fp_pushsn (real32 (AReg));
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x8f: /* XXX fpremfirst    */
		           BADCODE(IsT414);
                           /* Do calculation at fpremfirst. Push true to AReg, to execute one more fpremstep. */
                           fp_dobinary (fp_remfirstdb, fp_remfirstsn);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = true_t;
		           IPtr++;
		           break;
		case 0x90: /* XXX fpremstep    */
		           BADCODE(IsT414);
                           /* Do nothing here. Terminate loop with false. */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = false_t;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x91: /* fpnan    */
		           BADCODE(IsT414);
                           temp = true_t;
                           if (FAReg.length == FP_REAL64)
                                temp = fp_nandb (DB(FAReg));
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                temp = fp_nansn (SN(FAReg));
#ifdef EMUDEBUG
                           else
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpnan)\n");
#endif
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp ? true_t : false_t;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x92: /* fpordered    */
		           BADCODE(IsT414);
                           temp = false_t;
                           if (FAReg.length == FP_REAL64)
                           {
                                fp_peek2db (&dbtemp1, &dbtemp2);
                                temp = fp_ordereddb (dbtemp1, dbtemp2);
                           }
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                                fp_peek2sn (&sntemp1, &sntemp2);
                                temp = fp_orderedsn (sntemp1, sntemp2);
                           }
#ifdef EMUDEBUG
                           else
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpordered)\n");
#endif
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x93: /* fpnotfinite    */
		           BADCODE(IsT414);
                           temp = true_t;
                           if (FAReg.length == FP_REAL64)
                                temp = fp_notfinitedb (DB(FAReg));
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                temp = fp_notfinitesn (SN(FAReg));
#ifdef EMUDEBUG
                           else
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpnotfinite)\n");
#endif
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp ? true_t : false_t;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x94: /* fpgt    */
		           BADCODE(IsT414);
                           temp = fp_binary2word (fp_gtdb, fp_gtsn);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
		           IPtr++;
		           break;
		case 0x95: /* fpeq    */
		           BADCODE(IsT414);
                           temp = fp_binary2word (fp_eqdb, fp_eqsn);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
		           IPtr++;
		           break;
		case 0x96: /* fpi32tor32    */
		           BADCODE(IsT414);
                           temp = word (AReg);
                           fp_pushsn (fp_i32tor32 (temp));
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x98: /* fpi32tor64    */
		           BADCODE(IsT414);
                           temp = word (AReg);
                           fp_pushdb (fp_i32tor64 (temp));
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9a: /* fpb32tor64    */
		           BADCODE(IsT414);
                           temp = word (AReg);
                           fp_pushdb (fp_b32tor64 (temp));
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9c: /* fptesterr    */
		           BADCODE(IsT414);
                           fp_syncexcept ();            /* Sync FP_Error with native FPU excp. */
                           if (FP_Error)
                                temp = false_t;
                           else
                                temp = true_t;
                           fp_clrexcept ();             /* Clear native FPU excp. */
                           FP_Error = FALSE;
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9d: /* fprtoi32    */
		           BADCODE(IsT414);
                           if (FAReg.length == FP_REAL64)
                                DB(FAReg) = fp_rtoi32db (DB(FAReg));
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                SN(FAReg) = fp_rtoi32sn (SN(FAReg));
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fprtoi32)\n");
                                FAReg.length = FP_UNKNOWN;
                                SN(FAReg)    = RUndefined;
                           }
#endif
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9e: /* fpstnli32    */
		           BADCODE(IsT414);
                           if (FAReg.length == FP_REAL64)
                           {
                                fp_popdb (&dbtemp1);
                                temp = fp_stnli32db (dbtemp1);
                           }
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                                fp_popsn (&sntemp1);
                                temp = fp_stnli32sn (sntemp1);
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpstnli32)\n");
                                fp_drop ();
                                temp = Undefined_p;
                           }
#endif
                           writeword (AReg, temp);
                           AReg = BReg;
                           BReg = CReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9f: /* fpldzerosn    */
		           BADCODE(IsT414);
                           fp_pushsn (Zero);
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xa0: /* fpldzerodb    */
		           BADCODE(IsT414);
                           fp_pushdb (DZero);
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xa1: /* fpint    */
		           BADCODE(IsT414);
                           if (FAReg.length == FP_REAL64)
                                DB(FAReg) = fp_intdb (DB(FAReg));
                           else
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                SN(FAReg) = fp_intsn (SN(FAReg));
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpint)\n");
                                FAReg.length = FP_UNKNOWN;
                                DB(FAReg)    = DUndefined;
                           }
#endif
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xa3: /* fpdup    */
		           BADCODE(IsT414);
                           FCReg = FBReg;
                           FBReg = FAReg;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xa4: /* fprev    */
		           BADCODE(IsT414);
                           fptemp = FAReg;
                           FAReg  = FBReg;
                           FBReg  = fptemp;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xa6: /* fpldnladddb    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLADDDB", AReg));
                           fp_pushdb (real64 (AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL64)
#endif
                           {
                                fp_pop2db (&dbtemp1, &dbtemp2);
                                fp_pushdb (fp_adddb (dbtemp1, dbtemp2));
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpldnladddb)\n");
                                fp_drop ();
                                FAReg.length = FP_UNKNOWN;
                                DB(FAReg)    = DUndefined;
                           }
#endif
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0xa8: /* fpldnlmuldb    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLMULDB", AReg));
                           fp_pushdb (real64 (AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL64)
#endif
                           {
                                fp_pop2db (&dbtemp1, &dbtemp2);
                                fp_pushdb (fp_muldb (dbtemp1, dbtemp2));
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpldnlmuldb)\n");
                                fp_drop ();
                                FAReg.length = FP_UNKNOWN;
                                DB(FAReg)    = DUndefined;
                           }
#endif
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0xaa: /* fpldnladdsn    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLADDSN", AReg));
                           fp_pushsn (real32 (AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                                fp_pop2sn (&sntemp1, &sntemp2);
                                fp_pushsn (fp_addsn (sntemp1, sntemp2));
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpldnladdsn)\n");
                                fp_drop ();
                                FAReg.length = FP_UNKNOWN;
                                SN(FAReg)    = RUndefined;
                           }
#endif
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0x8d: /* fprange - fp range reduce                 (T9000) */
		           BADCODE(IsT414);
                           /* FBreg := integer quotient, FAreg := IEEE
                            * remainder of FBreg / FAreg; FCreg unaffected.
                            * The remainder is exact; the quotient is
                            * recovered from it and verified with fma so the
                            * pair is consistent (ISM 11.8). */
                           if (FAReg.length == FP_REAL64)
                           {
                               double b = DB(FBReg).fp, a = DB(FAReg).fp;
                               double r = remainder (b, a);
                               double q = nearbyint ((b - r) / a);
                               int k;
                               for (k = 0; k < 4 && fma (-q, a, b) != r; k++)
                                   q += (fma (-q, a, b) > r) ==
                                        (a > 0.0) ? 1.0 : -1.0;
                               if (!isfinite (b) || !isfinite (a) || a == 0.0)
                                   FP_Error = TRUE;
                               DB(FBReg).fp = q;
                               DB(FAReg).fp = r;
                           }
                           else
                           {
                               double b = (double)SN(FBReg).fp;
                               double a = (double)SN(FAReg).fp;
                               double r = remainder (b, a);
                               if (!isfinite (b) || !isfinite (a) || a == 0.0)
                                   FP_Error = TRUE;
                               SN(FBReg).fp = (float)nearbyint ((b - r) / a);
                               SN(FAReg).fp = (float)r;
                           }
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0x9b: /* fplg - fp less than or greater than       (T9000) */
		           BADCODE(IsT414);
                           temp = fp_binary2word (fp_lgdb, fp_lgsn);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
		           IPtr++;
		           break;
		case 0xb0: /* settimeslice                              (T9000) */
		           BADCODE(IsT414);
                           temp = TimesliceDisabled ? 0 : 1;
                           TimesliceDisabled = (AReg == 0);
                           AReg = temp;
		           IPtr++;
		           break;
		case 0xb6: /* ldflags                                   (T9000) */
		           BADCODE(IsT414);
                           /* Local flag encoding (see stflags): bit0 =
                            * IntegerError, bit1 = FP_Error.  Trap-enable
                            * bits are not modelled. */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = ((STATUSReg & ErrorFlag) ? 1u : 0u)
                                | (FP_Error ? 2u : 0u);
		           IPtr++;
		           break;
		case 0xb7: /* stflags                                   (T9000) */
		           BADCODE(IsT414);
                           if (AReg & 1u) { SetError; } else { ClearError; }
                           FP_Error = (AReg & 2u) ? TRUE : FALSE;
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
		case 0xba: /* cb - check byte                           (T9000) */
		           BADCODE(IsT414);
                           if (INT32(AReg) < -128 || INT32(AReg) > 127)
                               SetError;
		           IPtr++;
		           break;
		case 0xbb: /* cbu - check byte unsigned                 (T9000) */
		           BADCODE(IsT414);
                           if (AReg > 255u)
                               SetError;
		           IPtr++;
		           break;
		case 0xc2:   /* ldth - load trap-handler pointer          (T9000) */
		case 0x2CF2: /* ldth via the kernel stub's pfix-built OReg.
		              * thds_selth.S encodes App B's "2C F2" byte pair as
		              * a positive prefix chain; accept both spellings. */
		           BADCODE(IsT414);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = ThReg;
		           IPtr++;
		           break;
		case 0xc4: /* intdis - disable interruption+timeslice   (T9000) */
		           BADCODE(IsT414);
                           IntsDisabled = 1;
		           IPtr++;
		           break;
		case 0xc5: /* intenb - enable interruption              (T9000) */
		           BADCODE(IsT414);
                           IntsDisabled = 0;
		           IPtr++;
		           break;
		case 0xcf: /* fprem - fp IEEE remainder                 (T9000) */
		           BADCODE(IsT414);
                           /* FAreg := FBreg REM FAreg (exact); FCreg pops
                            * into FBreg (ISM 11.8, table 11.12). */
                           if (FAReg.length == FP_REAL64)
                           {
                               double b = DB(FBReg).fp, a = DB(FAReg).fp;
                               if (!isfinite (b) || !isfinite (a) || a == 0.0)
                                   FP_Error = TRUE;
                               DB(FAReg).fp = remainder (b, a);
                               FAReg.length = FP_REAL64;
                           }
                           else
                           {
                               double b = (double)SN(FBReg).fp;
                               double a = (double)SN(FAReg).fp;
                               if (!isfinite (b) || !isfinite (a) || a == 0.0)
                                   FP_Error = TRUE;
                               /* exact in double; result representable */
                               SN(FAReg).fp = (float)remainder (b, a);
                               FAReg.length = FP_REAL32;
                           }
                           FBReg = FCReg;
                           FCReg.length = FP_UNKNOWN;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xdd: /* fpadddbsn - add doubles to single         (T9000) */
		           BADCODE(IsT414);
                           /* Correctly-rounded (double+double)->single.
                            * 80-bit extended holds the sum with one rounding
                            * to 64-bit significand; 64 >= 2*24+2, so the
                            * second rounding to single is innocuous (no
                            * double-rounding error).  ISM 11.11. */
                           SN(FAReg).fp = (float)((long double)DB(FBReg).fp
                                                + (long double)DB(FAReg).fp);
                           FAReg.length = FP_REAL32;
                           FBReg = FCReg;
                           FCReg.length = FP_UNKNOWN;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xde: /* fpchki32                                  (T9000) */
		           BADCODE(IsT414);
                           if (FAReg.length == FP_REAL64)
                               fp_chki32db (DB(FAReg));
                           else
                               fp_chki32sn (SN(FAReg));
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xdf: /* fpchki64                                  (T9000) */
		           BADCODE(IsT414);
                           if (FAReg.length == FP_REAL64)
                               fp_chki64db (DB(FAReg));
                           else
                               fp_chki64sn (SN(FAReg));
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xf0: /* devlb - device load byte                  (T9000) */
		           BADCODE(IsT414);
                           /* No cache or write buffering is modelled, so the
                            * device accesses behave exactly like lb/ls/ldnl 0
                            * etc. (ISM table 7.23 says they use the same
                            * registers and, absent reordering, the same
                            * behavior).  dev_access lets them into regions a
                            * P-process may reach only by device instructions
                            * (ISM 9.5 bit 3). */
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           AReg = byte (AReg) & 0xFF;
                           t9k_prot.dev_access = 0;
		           IPtr++;
		           break;
		case 0xf1: /* devsb - device store byte                 (T9000) */
		           BADCODE(IsT414);
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           writebyte (AReg, BReg);
                           t9k_prot.dev_access = 0;
                           AReg = CReg;
                           CReg = 0;
		           IPtr++;
		           break;
		case 0xf2: /* devls - device load sixteen               (T9000) */
		           BADCODE(IsT414);
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           AReg = (byte (AReg) & 0xFF)
                                | ((byte (AReg + 1) & 0xFF) << 8);
                           t9k_prot.dev_access = 0;
		           IPtr++;
		           break;
		case 0xf3: /* devss - device store sixteen              (T9000) */
		           BADCODE(IsT414);
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           writebyte (AReg,     BReg        & 0xFF);
                           writebyte (AReg + 1, (BReg >> 8) & 0xFF);
                           t9k_prot.dev_access = 0;
                           AReg = CReg;
                           CReg = 0;
		           IPtr++;
		           break;
		case 0xf4: /* devlw - device load word                  (T9000) */
		           BADCODE(IsT414);
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           AReg = word (AReg);
                           t9k_prot.dev_access = 0;
		           IPtr++;
		           break;
		case 0xf5: /* devsw - device store word                 (T9000) */
		           BADCODE(IsT414);
                           t9k_prot.dev_access = T9K_ACC_DEV;
                           writeword (AReg, BReg);
                           t9k_prot.dev_access = 0;
                           AReg = CReg;
                           CReg = 0;
		           IPtr++;
		           break;
		/* T9000 VCP, channel and configuration instructions
		 * (t9000_vcp.c). */
		case 0xbc: /* insphdr */
		case 0xbd: /* readbfr */
		case 0xbe: /* ldconf */
		case 0xbf: /* stconf */
		case 0xc0: /* ldcnt */
		case 0xc3: /* ldchstatus */
		case 0xc9: /* chantype */
		           BADCODE(IsT414);
			   IPtr++;
			   t9k_vcp_exec (OReg);
			   break;
		case 0xc7: /* cir - check in range                      (T9000) */
		           BADCODE(IsT414);
                           if (INT32(CReg) < INT32(AReg) || INT32(CReg) > INT32(BReg))
                               SetError;
                           AReg = CReg;
		           IPtr++;
		           break;
		case 0xcc: /* ciru - check in range unsigned            (T9000) */
		           BADCODE(IsT414);
                           if (CReg < AReg || CReg > BReg)
                               SetError;
                           AReg = CReg;
		           IPtr++;
		           break;
		case 0xfa: /* cs - check sixteen                        (T9000) */
		           BADCODE(IsT414);
                           if (INT32(AReg) < -32768 || INT32(AReg) > 32767)
                               SetError;
		           IPtr++;
		           break;
		case 0xfb: /* csu - check sixteen unsigned              (T9000) */
		           BADCODE(IsT414);
                           if (AReg > 65535u)
                               SetError;
		           IPtr++;
		           break;
		case 0x97: /* fpge - fp greater than or equals          (T9000) */
		           BADCODE(IsT414);
                           temp = fp_binary2word (fp_gedb, fp_gesn);
                           CReg = BReg;
                           BReg = AReg;
                           AReg = temp;
		           IPtr++;
		           break;
		case 0xcb: /* fpseterr                                  (T9000) */
		           BADCODE(IsT414);
                           FP_Error = TRUE;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xd0: /* fprn - set round to nearest               (T9000) */
		           BADCODE(IsT414);
                           fp_setrounding ("fprn", ROUND_N);
                           /* Do not reset rounding mode. */
		           IPtr++;
		           break;
		case 0xd1: /* fpdivby2                                  (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_divby2db, fp_divby2sn);
		           IPtr++;
		           break;
		case 0xd2: /* fpmulby2                                  (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_mulby2db, fp_mulby2sn);
		           IPtr++;
		           break;
		case 0xd3: /* fpsqrt                                    (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_sqrtlastdb, fp_sqrtlastsn);
		           IPtr++;
		           break;
		case 0xd4: /* fprp - set round to plus infinity         (T9000) */
		           BADCODE(IsT414);
                           fp_setrounding ("fprp", ROUND_P);
                           /* Do not reset rounding mode. */
		           IPtr++;
		           break;
		case 0xd5: /* fprm - set round to minus infinity        (T9000) */
		           BADCODE(IsT414);
                           fp_setrounding ("fprm", ROUND_M);
                           /* Do not reset rounding mode. */
		           IPtr++;
		           break;
		case 0xd6: /* fprz - set round to zero                  (T9000) */
		           BADCODE(IsT414);
                           fp_setrounding ("fprz", ROUND_Z);
                           /* Do not reset rounding mode. */
		           IPtr++;
		           break;
		case 0xd7: /* fpr32tor64                                (T9000) */
		           BADCODE(IsT414);
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                               FAReg.length = FP_REAL64;
                               DB(FAReg)    = fp_r32tor64 (SN(FAReg));
                           }
#ifdef EMUDEBUG
                           else
                           {
                               printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpr32tor64)\n");
                               FAReg.length = FP_UNKNOWN;
                               DB(FAReg)    = DUndefined;
                           }
#endif
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xd8: /* fpr64tor32                                (T9000) */
		           BADCODE(IsT414);
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL64)
#endif
                           {
                               FAReg.length = FP_REAL32;
                               SN(FAReg) = fp_r64tor32 (DB(FAReg));
                           }
#ifdef EMUDEBUG
                           else
                           {
                               printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpr64tor32)\n");
                               FAReg.length = FP_UNKNOWN;
                               SN(FAReg)    = RUndefined;
                           }
#endif
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xd9: /* fpexpdec32 - divide by 2^32               (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_expdec32db, fp_expdec32sn);
		           IPtr++;
		           break;
		case 0xda: /* fpexpinc32 - multiply by 2^32             (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_expinc32db, fp_expinc32sn);
		           IPtr++;
		           break;
		case 0xdb: /* fpabs                                     (T9000) */
		           BADCODE(IsT414);
                           fp_dounary (fp_absdb, fp_abssn);
		           IPtr++;
		           break;
		case 0xdc: /* fpclrerr                                  (T9000) */
		           BADCODE(IsT414);
                           fp_clrexcept ();
                           FP_Error = FALSE;
                           ResetRounding = TRUE;
		           IPtr++;
		           break;
		case 0xab: /* fpentry    */
		           BADCODE(IsT414);
                           temp = AReg;
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;

                           PROFILEINSTR(add_profile (0x300 + temp));
                           switch (temp) {
			   case 0x01: /* fpusqrtfirst    */
                                      if (FAReg.length == FP_REAL64)
                                          DB(FAReg) = fp_sqrtfirstdb (DB(FAReg));
                                      else
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL32)
#endif
                                          SN(FAReg) = fp_sqrtfirstsn (SN(FAReg));
#ifdef EMUDEBUG
                                      else
                                      {
                                          printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpusqrtfirst)\n");
                                          FAReg.length = FP_UNKNOWN;
                                          DB(FAReg)    = DUndefined;
                                      }
#endif
                                      FBReg.length = FP_UNKNOWN;
                                      FCReg.length = FP_UNKNOWN;
                                      ResetRounding = TRUE;
			              break;
			   case 0x02: /* fpusqrtstep    */
                                      FBReg.length = FP_UNKNOWN;
                                      FCReg.length = FP_UNKNOWN;
                                      ResetRounding = TRUE;
			              break;
			   case 0x03: /* fpusqrtlast    */
                                      fp_dounary (fp_sqrtlastdb, fp_sqrtlastsn);
			              break;
			   case 0x04: /* fpurp    */
                                      fp_setrounding ("fpurp", ROUND_P);
                                      /* Do not reset rounding mode. */
			              break;
			   case 0x05: /* fpurm    */
                                      fp_setrounding ("fpurm", ROUND_M);
                                      /* Do not reset rounding mode. */
			              break;
			   case 0x06: /* fpurz    */
                                      fp_setrounding ("fpurz", ROUND_Z);
                                      /* Do not reset rounding mode. */
			              break;
			   case 0x07: /* fpur32tor64    */
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL32)
#endif
                                      {
                                          FAReg.length = FP_REAL64;
                                          DB(FAReg)    = fp_r32tor64 (SN(FAReg));
                                      }
#ifdef EMUDEBUG
                                      else
                                      {
                                          printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpur32tor64)\n");
                                          FAReg.length = FP_UNKNOWN;
                                          DB(FAReg)    = DUndefined;
                                      }
#endif
                                      ResetRounding = TRUE;
			              break;
			   case 0x08: /* fpur64tor32    */
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL64)
#endif
                                      {
                                          FAReg.length = FP_REAL32;
                                          SN(FAReg) = fp_r64tor32 (DB(FAReg));
                                      }
#ifdef EMUDEBUG
                                      else
                                      {
                                          printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpur64tor32)\n");
                                          FAReg.length = FP_UNKNOWN;
                                          SN(FAReg)    = RUndefined;
                                      }
#endif
                                      ResetRounding = TRUE;
			              break;
			   case 0x09: /* fpuexpdec32    */
                                      fp_dounary (fp_expdec32db, fp_expdec32sn);
			              break;
			   case 0x0a: /* fpuexpinc32    */
                                      fp_dounary (fp_expinc32db, fp_expinc32sn);
			              break;
			   case 0x0b: /* fpuabs    */
                                      fp_dounary (fp_absdb, fp_abssn);
			              break;
			   case 0x0d: /* fpunoround    */
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL64)
#endif
                                      {
                                          FAReg.length = FP_REAL32;
                                          SN(FAReg) = fp_norounddb (DB(FAReg));
                                      }
#ifdef EMUDEBUG
                                      else
                                      {
                                          printf ("-W-EMUFPU: Warning - FAReg is not REAL64! (fpunoround)\n");
                                          FAReg.length = FP_UNKNOWN;
                                          SN(FAReg)    = RUndefined;
                                      }
#endif
                                      ResetRounding = TRUE;
			              break;
			   case 0x0e: /* fpuchki32    */
                                      if (FAReg.length == FP_REAL64)
                                          fp_chki32db (DB(FAReg));
                                      else
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL32)
#endif
                                          fp_chki32sn (SN(FAReg));
#ifdef EMUDEBUG
                                      else
                                          printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpuchki32)\n");
#endif
                                      ResetRounding = TRUE;
			              break;
			   case 0x0f: /* fpuchki64    */
                                      if (FAReg.length == FP_REAL64)
                                          fp_chki64db (DB(FAReg));
                                      else
#ifdef EMUDEBUG
                                      if (FAReg.length == FP_REAL32)
#endif
                                          fp_chki64sn (SN(FAReg));
#ifdef EMUDEBUG
                                      else
                                          printf ("-W-EMUFPU: Warning - FAReg is undefined! (fpuchki64)\n");
#endif
                                      ResetRounding = TRUE;
			              break;
			   case 0x11: /* fpudivby2    */
                                      fp_dounary (fp_divby2db, fp_divby2sn);
			              break;
			   case 0x12: /* fpumulby2    */
                                      fp_dounary (fp_mulby2db, fp_mulby2sn);
			              break;
			   case 0x22: /* fpurn    */
                                      fp_setrounding ("fpurn", ROUND_N);
                                      /* Do not reset rounding mode. */
			              break;
			   case 0x23: /* fpuseterr    */
                                      FP_Error = TRUE;
                                      ResetRounding = TRUE;
			              break;
			   case 0x9c: /* fpuclrerr    */
                                      fp_clrexcept ();
                                      FP_Error = FALSE;
                                      ResetRounding = TRUE;
			              break;
                           default  :  
                                      printf ("-E-EMU414: Error - bad Icode! (#%02X - %s)\n", OReg, mnemonic (Icode, OReg, AReg, 0));
                                      maybe_log_nearest_sym(IPtr, "bad-icode");
                                      processor_state ();
			              handler (-1);
			              break;
                           }
		           break;
		case 0xac: /* fpldnlmulsn    */
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLMULSN", AReg));
                           fp_pushsn (real32 (AReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                           {
                                fp_pop2sn (&sntemp1, &sntemp2);
                                fp_pushsn (fp_mulsn (sntemp1, sntemp2));
                           }
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpldnlmulsn)\n");
                                fp_drop ();
                                FAReg.length = FP_UNKNOWN;
                                SN(FAReg)    = RUndefined;
                           }
#endif
                           AReg = BReg;
                           BReg = CReg;
		           IPtr++;
		           break;
#ifdef T4COMBINATIONS
                case 0x100: /* stl ldl */
			   writeword (index (WPtr, Arg0), AReg);
                           if (Arg0 != Arg1)
			        AReg = word (index (WPtr, Arg1));
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x101: /* ldl ldnl */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
			   AReg = word (index (AReg, Arg1));
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                 case 0x102: /* eqc cj */
			   /* Preserve exact standalone semantics for:
			    *   EQC Arg0; CJ Arg1
			    * EQC sets AReg to 1 when equal, else 0.
			    * CJ pops on non-zero (continue), jumps on zero (no pop). */
			   IPtr++;
			   if (AReg == Arg0)
			   {
				AReg = BReg;
				BReg = CReg;
			   }
			   else
			   {
				IPtr = IPtr + Arg1;
			   }
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x103: /* ldl ldnlp */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
			   AReg = index (AReg, Arg1);
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x104: /* ldl ldl */
			   CReg = AReg;
			   BReg = word (index (WPtr, Arg0));
			   AReg = word (index (WPtr, Arg1));
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x105: /* ldlp fpldnldb */
			   XReg = index (WPtr, Arg0);
                           fp_pushdb (real64 (XReg));
		           IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x106: /* ajw ret */
			   /* ajw comes first, so the pair starts at its first byte */
			   if (unlikely(t9k_prot.is_p))
				   t9k_check_wptr(index(WPtr, Arg0), Icache[islot].ip);
                           WPtr = index (WPtr, Arg0);
                           T4DEBUG(checkWPtr ("AJW", WPtr));
			   {
				   uint32_t ra106 = word(WPtr);
				   if (ra106 < MemStart && (WPtr >> 30) == 3u) {
					   uint32_t pa = 0;
					   (void)t9k_probe(WPtr, T9K_ACC_READ, &pa);
					   fprintf(stderr,
						   "[Q3-AJWRET0] IPtr=0x%08X WPtr=0x%08X ajw=%d is_p=%d pa=0x%08X "
						   "w0=%08X w1=%08X w2=%08X w3=%08X A=%08X B=%08X\n",
						   IPtr, WPtr, (int)Arg0, t9k_in_p(), pa,
						   word_int(WPtr), word_int(index(WPtr, 1)),
						   word_int(index(WPtr, 2)), word_int(index(WPtr, 3)),
						   AReg, BReg);
				   }
				   IPtr = ra106;
			   }
			   UpdateWdescReg (index (WPtr, 4) | ProcPriority);
                           T4DEBUG(checkWPtr ("RET", WPtr));
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x107: /* ldlp ldc */
			   CReg = AReg;
			   BReg = index (WPtr, Arg0);
			   AReg = Arg1;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x108: /* ldlp ldl */
			   CReg = AReg;
			   BReg = index (WPtr, Arg0);
			   AReg = word (index (WPtr, Arg1));
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x109: /* stl stl */
			   writeword (index (WPtr, Arg0), AReg);
			   writeword (index (WPtr, Arg1), BReg);
			   AReg = CReg;
			   BReg = CReg;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10a: /* ldl cflerr */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
                           BADCODE(IsT800);
			   if ((t4_isinf (AReg)) || (t4_isnan (AReg)))
				SetError;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10b: /* ldlp fpstnlsn */
			   XReg = index (WPtr, Arg0);
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPSTNLSN", XReg));
#ifdef EMUDEBUG
                           if (FAReg.length == FP_REAL32)
#endif
                                fp_popsn (&sntemp1);
#ifdef EMUDEBUG
                           else
                           {
                                printf ("-W-EMUFPU: Warning - FAReg is not REAL32! (fpstnlsn)\n");
                                sntemp1 = RUndefined;
                           }
#endif
                           writereal32 (XReg, sntemp1);
                           ResetRounding = TRUE;
		           IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10c: /* ldlp fpldnlsn */
			   XReg = index (WPtr, Arg0);
		           BADCODE(IsT414);
                           T4DEBUG(checkWordAligned ("FPLDNLSN", XReg));
                           fp_pushsn (real32 (XReg));
		           IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10d: /* ldl adc */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
			   t4_overflow = FALSE;
			   t4_carry = 0;
			   AReg = t4_eadd32 (AReg, Arg1);
			   if (t4_overflow == TRUE)
				SetError;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10e: /* ldl stnl */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
                           T4DEBUG(checkWordAligned ("STNL", AReg));
			   {
			       uint32_t addr = index (AReg, Arg1);
			       /* Debug: catch framebuffer combo stores */
			       if (fb_addr_in_range(addr)) {
			           static int fb_combo_count = 0;
			           if (fb_combo_count < 50) {
			               fprintf(stderr, "[FB_LDL_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_combo_count, IPtr, addr, BReg);
			               fb_combo_count++;
			           }
			       }
			       if (rare_hooks && text_write_trace_enabled &&
			           addr >= text_write_trace_start &&
			           addr < text_write_trace_end) {
			           uint32_t delta = 0;
			           const link_sym *lsym = find_link_sym(IPtr, &delta);
			           const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
			           fprintf(stderr,
			                   "[TEXT_STNL_COMBO] IPtr=0x%08X addr=0x%08X BReg=0x%08X WPtr=0x%08X%s%s\n",
			                   IPtr, addr, BReg, WPtr,
			                   (lsym || sym) ? " sym=" : "",
			                   lsym ? lsym->name : (sym ? sym->name : ""));
			       }
			       writeword (addr, BReg);
			   }
			   AReg = CReg;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x10f: /* ldc ldl */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = Arg0;
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg1));
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x110: /* stl j */
			   writeword (index (WPtr, Arg0), AReg);
			   AReg = BReg;
			   BReg = CReg;
			   IPtr++;
			   IPtr = IPtr + Arg1;
			   D_check();
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x111: /* gt cj */
			   if (INT32(BReg) > INT32(AReg))
			   {
				AReg = true_t;
			   }
			   else
			   {
				AReg = false_t;
			   }
			   BReg = CReg;
			   IPtr++;
			   if (AReg != 0)
			   {
				AReg = BReg;
				BReg = CReg;
			   }
			   else
			   {
				IPtr = IPtr + Arg1;
			   }
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x112: /* wsub stnl */
			   AReg = index (AReg, BReg);
			   BReg = CReg;
                           T4DEBUG(checkWordAligned ("STNL", AReg));
			   {
			       uint32_t addr = index (AReg, Arg1);
			       /* Debug: catch framebuffer combo stores */
			       if (fb_addr_in_range(addr)) {
			           static int fb_wsub_count = 0;
			           if (fb_wsub_count < 50) {
			               fprintf(stderr, "[FB_WSUB_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_wsub_count, IPtr, addr, BReg);
			               fb_wsub_count++;
			           }
			       }
			       if (rare_hooks && text_write_trace_enabled &&
			           addr >= text_write_trace_start &&
			           addr < text_write_trace_end) {
			           uint32_t delta = 0;
			           const link_sym *lsym = find_link_sym(IPtr, &delta);
			           const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
			           fprintf(stderr,
			                   "[TEXT_STNL_COMBO] IPtr=0x%08X addr=0x%08X BReg=0x%08X WPtr=0x%08X%s%s\n",
			                   IPtr, addr, BReg, WPtr,
			                   (lsym || sym) ? " sym=" : "",
			                   lsym ? lsym->name : (sym ? sym->name : ""));
			       }
			       writeword (addr, BReg);
			   }
			   AReg = CReg;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x113: /* ldl wsub  */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = word (index (WPtr, Arg0));
			   AReg = index (AReg, BReg);
			   BReg = CReg;
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x114: /* dup stl   */
		           BADCODE(IsT414);
                           CReg = BReg;
			   writeword (index (WPtr, Arg1), AReg);
			   IPtr++;
		           PROFILE(profile[PRO_INSTR]++);
                           break;
                case 0x115: /* mint and */
			   CReg = BReg;
			   AReg &= 0x80000000;
			   IPtr++;
                           break;
#endif
                /* ---------------------------------------------------------
                 * TVX - Transputer Vector eXtension (MXFP4 / bfloat16).
                 * See transputer/t4/tvx.h and transputer/TVX-ISA.md.
                 * --------------------------------------------------------- */
                case 0x1c0: /* bf16tor32   A := f32bits(bf16 A[15:0])       */
                           AReg = tvx_bf16_to_f32 (AReg);
                           IPtr++;
                           break;
                case 0x1c1: /* r32tobf16   A := bf16bits(f32 A), RNE        */
                           AReg = tvx_f32_to_bf16 (AReg);
                           IPtr++;
                           break;
                case 0x1c2: /* bf16pack    A := (bf16(B)<<16)|bf16(A)       */
                           AReg = tvx_bf16_pack (BReg, AReg);
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1c3: /* bf16unpkh   A := f32bits(bf16 A[31:16])      */
                           AReg = tvx_bf16_unpack_hi (AReg);
                           IPtr++;
                           break;
                case 0x1c4: /* mxfp4tor32  A := f32bits(E2M1 A[3:0])        */
                           AReg = tvx_mxfp4_to_f32 (AReg);
                           IPtr++;
                           break;
                case 0x1c5: /* r32tomxfp4  A := E2M1(f32 A), RNE+saturate   */
                           AReg = tvx_f32_to_mxfp4 (AReg);
                           IPtr++;
                           break;
                case 0x1c6: /* mxe8m0tor32 A := f32bits(E8M0 A[7:0])        */
                           AReg = tvx_e8m0_to_f32 (AReg);
                           IPtr++;
                           break;
                case 0x1c7: /* mxe8m0of    A := E8M0 scale for amax f32 A   */
                           AReg = tvx_e8m0_of (AReg);
                           IPtr++;
                           break;
                case 0x1c8: /* mxfp4dot    A=block ptr, B=act ptr           */
                           {
                                uint8_t  nib[16];
                                uint32_t act[32];
                                fpreal32_t r;
                                float t, s;
                                uint8_t scale;
                                int i;

                                BADCODE(IsT414);
                                T4DEBUG(checkWordAligned ("MXFP4DOT", BReg));

                                scale = byte (AReg);
                                for (i = 0; i < 16; i++)
                                        nib[i] = byte (AReg + 1 + i);
                                for (i = 0; i < 32; i++)
                                        act[i] = real32 (BReg + 4 * i).bits;

                                t = tvx_mxfp4_block_acc (nib, act);
                                s = tvx_bits_to_float (
                                        tvx_e8m0_to_f32 (scale));

                                fp_popsn (&sntemp1);
                                r.fp = sntemp1.fp + (float)(s * t);
                                fp_pushsn (r);
                           }
                           AReg = CReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1c9: /* bf16dot     A=x ptr, B=y ptr, C=count        */
                           {
                                fpreal32_t r;
                                float t = 0.0f;
                                uint32_t n = CReg, i;

                                BADCODE(IsT414);
                                for (i = 0; i < n; i++) {
                                        uint32_t xb, yb;
                                        xb = (uint32_t)byte (AReg + 2 * i)
                                           | ((uint32_t)byte (AReg + 2 * i + 1) << 8);
                                        yb = (uint32_t)byte (BReg + 2 * i)
                                           | ((uint32_t)byte (BReg + 2 * i + 1) << 8);
                                        t = t + (float)(
                                            tvx_bits_to_float (tvx_bf16_to_f32 (xb)) *
                                            tvx_bits_to_float (tvx_bf16_to_f32 (yb)));
                                }

                                fp_popsn (&sntemp1);
                                r.fp = sntemp1.fp + t;
                                fp_pushsn (r);
                           }
                           IPtr++;
                           break;
                case 0x1ca: /* mxfp4gemv   A=blocks ptr, B=act ptr, C=count */
                           {
                                uint8_t  nib[16];
                                uint32_t act[32];
                                fpreal32_t r;
                                float acc;
                                uint32_t nblk = CReg, b;
                                int i;

                                BADCODE(IsT414);
                                fp_popsn (&sntemp1);
                                acc = sntemp1.fp;

                                /* Defined to be exactly nblk chained
                                 * MXFP4DOTs, so the two are interchangeable. */
                                for (b = 0; b < nblk; b++) {
                                        uint32_t bp = AReg + b * 17;
                                        uint32_t ap = BReg + b * 128;
                                        float t, s;

                                        for (i = 0; i < 16; i++)
                                                nib[i] = byte (bp + 1 + i);
                                        for (i = 0; i < 32; i++)
                                                act[i] = real32 (ap + 4 * i).bits;

                                        t = tvx_mxfp4_block_acc (nib, act);
                                        s = tvx_bits_to_float (
                                                tvx_e8m0_to_f32 (byte (bp)));
                                        acc = acc + (float)(s * t);
                                }
                                r.fp = acc;
                                fp_pushsn (r);
                           }
                           IPtr++;
                           break;
                /* ---- sub-word SIMD: 8 MXFP4 lanes / 2 bfloat16 lanes ---- */
                case 0x1cb: /* mxfp4unpk8  A=packed nibbles, B=dst ptr; pop2 */
                           {
                                uint32_t w = AReg;
                                int i;
                                for (i = 0; i < 8; i++) {
                                        fpreal32_t v;
                                        v.bits = tvx_mxfp4_to_f32 ((w >> (4*i)) & 0xFu);
                                        writereal32 (BReg + 4 * i, v);
                                }
                           }
                           AReg = CReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1cc: /* r32tomxfp4x8 A=src ptr -> A=packed 8 nibbles  */
                           {
                                uint32_t w = 0;
                                int i;
                                for (i = 0; i < 8; i++)
                                        w |= tvx_f32_to_mxfp4 (real32 (AReg + 4*i).bits)
                                             << (4 * i);
                                AReg = w;
                           }
                           IPtr++;
                           break;
                case 0x1cd: /* bf16mul2    A = B *packed* A, 2 lanes; pop 1  */
                           AReg = tvx_bf16_pair_mul (BReg, AReg);
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1ce: /* bf16add2    A = B +packed* A, 2 lanes; pop 1  */
                           AReg = tvx_bf16_pair_add (BReg, AReg);
                           BReg = CReg;
                           IPtr++;
                           break;
                /* ---- vector kernels for transformer inference ----------
                 * Length comes from VSETLEN, in the MOVE2DINIT tradition,
                 * because the 3-deep stack cannot carry dst+src+src2+count. */
                case 0x1cf: /* vsetlen     A = element count; pop 1          */
                           vec_len = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1d0: /* mxfp4gemvbf A=blocks, B=bf16 acts, C=nblocks */
                           {
                                uint8_t  nib[16];
                                uint32_t act[32];
                                float acc;
                                uint32_t nblk = CReg, b;
                                int i;

                                BADCODE(IsT414);
                                fp_popsn (&sntemp1);
                                acc = sntemp1.fp;
                                for (b = 0; b < nblk; b++) {
                                        uint32_t bp = AReg + b * 17;
                                        uint32_t ap = BReg + b * 64; /* 32 bf16 */
                                        float t, s;

                                        for (i = 0; i < 16; i++)
                                                nib[i] = byte (bp + 1 + i);
                                        for (i = 0; i < 32; i++) {
                                                uint32_t h = (uint32_t)byte (ap + 2*i)
                                                    | ((uint32_t)byte (ap + 2*i + 1) << 8);
                                                act[i] = tvx_bf16_to_f32 (h);
                                        }
                                        t = tvx_mxfp4_block_acc (nib, act);
                                        s = tvx_bits_to_float (
                                                tvx_e8m0_to_f32 (byte (bp)));
                                        acc = acc + (float)(s * t);
                                }
                                sntemp1.fp = acc;
                                fp_pushsn (sntemp1);
                           }
                           IPtr++;
                           break;
                case 0x1d1: /* vaddsn      A=dst, B=src1, C=src2; pop 3      */
                           {
                                uint32_t i;
                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        fpreal32_t v;
                                        v.fp = (float)((double)real32 (BReg + 4*i).fp
                                                     + (double)real32 (CReg + 4*i).fp);
                                        writereal32 (AReg + 4*i, v);
                                }
                           }
                           IPtr++;
                           break;
                case 0x1d2: /* vmulsn      A=dst, B=src1, C=src2; pop 3      */
                           {
                                uint32_t i;
                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        fpreal32_t v;
                                        v.fp = (float)((double)real32 (BReg + 4*i).fp
                                                     * (double)real32 (CReg + 4*i).fp);
                                        writereal32 (AReg + 4*i, v);
                                }
                           }
                           IPtr++;
                           break;
                case 0x1d3: /* vsilusn     A=dst, B=src; pop 2               */
                           {
                                uint32_t i;
                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        fpreal32_t v;
                                        v.fp = (float)tvx_silu_exact (
                                                (double)real32 (BReg + 4*i).fp);
                                        writereal32 (AReg + 4*i, v);
                                }
                           }
                           AReg = CReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1d4: /* vrmsnorm    A=dst, B=src, C=gain; pop 3       */
                           {
                                uint32_t i;
                                double ss = 0.0, r;
                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        double v = (double)real32 (BReg + 4*i).fp;
                                        ss += v * v;
                                }
                                r = tvx_rmsnorm_scale (&ss, vec_len ? vec_len : 1);
                                for (i = 0; i < vec_len; i++) {
                                        fpreal32_t v;
                                        v.fp = (float)((double)real32 (BReg + 4*i).fp * r
                                                     * (double)real32 (CReg + 4*i).fp);
                                        writereal32 (AReg + 4*i, v);
                                }
                           }
                           IPtr++;
                           break;
                case 0x1d5: /* vsoftmaxsn  A=dst, B=src; pop 2               */
                           {
                                uint32_t i;
                                double mx, sum = 0.0;
                                BADCODE(IsT414);
                                if (vec_len == 0) { AReg = CReg; BReg = CReg; IPtr++; break; }
                                mx = (double)real32 (BReg).fp;
                                for (i = 1; i < vec_len; i++) {
                                        double v = (double)real32 (BReg + 4*i).fp;
                                        if (v > mx) mx = v;
                                }
                                for (i = 0; i < vec_len; i++)
                                        sum += exp ((double)real32 (BReg + 4*i).fp - mx);
                                for (i = 0; i < vec_len; i++) {
                                        fpreal32_t v;
                                        v.fp = (float)(exp ((double)real32 (BReg + 4*i).fp - mx)
                                                       / sum);
                                        writereal32 (AReg + 4*i, v);
                                }
                           }
                           AReg = CReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                /* ---- IEEE binary16 (ggml_half) and Q8_0 ---------------- */
                case 0x1d6: /* f16tor32    A := f32bits(binary16 A[15:0])   */
                           AReg = tvx_f16_to_f32 (AReg);
                           IPtr++;
                           break;
                case 0x1d7: /* r32tof16    A := binary16 bits of f32 A, RNE */
                           AReg = tvx_f32_to_f16 (AReg);
                           IPtr++;
                           break;
                case 0x1d8: /* q8dot       A=q8_0 blocks, B=f32 acts, C=n   */
                case 0x1d9: /* q8dotbf     A=q8_0 blocks, B=bf16 acts, C=n  */
                           {
                                int8_t   qs[32];
                                uint32_t act[32];
                                float acc;
                                uint32_t nblk = CReg, b;
                                int is_bf = (OReg == 0x1d9);
                                int i;

                                BADCODE(IsT414);
                                fp_popsn (&sntemp1);
                                acc = sntemp1.fp;

                                for (b = 0; b < nblk; b++) {
                                        uint32_t bp = AReg + b * 34;
                                        uint32_t ap = BReg + b * (is_bf ? 64 : 128);
                                        uint32_t h;
                                        float t, d;

                                        h = (uint32_t)byte (bp)
                                          | ((uint32_t)byte (bp + 1) << 8);
                                        for (i = 0; i < 32; i++)
                                                qs[i] = (int8_t)byte (bp + 2 + i);
                                        if (is_bf) {
                                                for (i = 0; i < 32; i++) {
                                                        uint32_t w =
                                                            (uint32_t)byte (ap + 2*i)
                                                          | ((uint32_t)byte (ap + 2*i + 1) << 8);
                                                        act[i] = tvx_bf16_to_f32 (w);
                                                }
                                        } else {
                                                for (i = 0; i < 32; i++)
                                                        act[i] = real32 (ap + 4*i).bits;
                                        }
                                        t = tvx_q8_block_acc (qs, act);
                                        d = tvx_bits_to_float (tvx_f16_to_f32 (h));
                                        acc = acc + (float)(d * t);
                                }
                                sntemp1.fp = acc;
                                fp_pushsn (sntemp1);
                           }
                           IPtr++;
                           break;
                /* ---- GGUF i-quants: IQ1_M / IQ2_XXS / IQ3_XXS --------- */
                case 0x1da: /* iq1mdot    A=blocks, B=f32 acts, C=nblocks  */
                case 0x1db: /* iq2xxsdot                                    */
                case 0x1dc: /* iq3xxsdot                                    */
                           {
                                static T4_THREAD_LOCAL uint8_t raw[128];
                                static T4_THREAD_LOCAL float dec[256];
                                float acc;
                                uint32_t nblk = CReg, b;
                                uint32_t bsz = (OReg == 0x1da) ? TVX_IQ1M_BYTES
                                             : (OReg == 0x1db) ? TVX_IQ2XXS_BYTES
                                             :                   TVX_IQ3XXS_BYTES;
                                int i;

                                BADCODE(IsT414);
                                fp_popsn (&sntemp1);
                                acc = sntemp1.fp;

                                for (b = 0; b < nblk; b++) {
                                        uint32_t bp = AReg + b * bsz;
                                        uint32_t ap = BReg + b * 256 * 4;
                                        float t = 0.0f;

                                        for (i = 0; i < (int)bsz; i++)
                                                raw[i] = byte (bp + i);
                                        if (OReg == 0x1da)      tvx_iq1m_decode (raw, dec);
                                        else if (OReg == 0x1db) tvx_iq2xxs_decode (raw, dec);
                                        else                    tvx_iq3xxs_decode (raw, dec);

                                        for (i = 0; i < 256; i++)
                                                t = t + (float)(dec[i] * real32 (ap + 4*i).fp);
                                        acc = acc + t;
                                }
                                sntemp1.fp = acc;
                                fp_pushsn (sntemp1);
                           }
                           IPtr++;
                           break;
                /* ---- generic GGUF quantised dot ----------------------
                 * Covers every ggml quantisation type via llama.cpp's own
                 * vendored decoders.  The format is a ggml type id set by
                 * QSETFMT, so new formats need no new opcode. */
                case 0x1dd: /* qsetfmt     A = ggml type id; pop 1          */
                           quant_fmt = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1de: /* qdot        A=blocks, B=f32 acts, C=nblocks  */
                           {
                                static T4_THREAD_LOCAL uint8_t raw[256];
                                static T4_THREAD_LOCAL float dec[256];
                                float acc;
                                uint32_t nblk = CReg, b;
                                int bsz = tvx_qfmt_block_bytes ((int)quant_fmt);
                                int i, n;

                                BADCODE(IsT414);
                                fp_popsn (&sntemp1);
                                acc = sntemp1.fp;

                                if (bsz > 0 && bsz <= (int)sizeof(raw)) {
                                    for (b = 0; b < nblk; b++) {
                                        uint32_t bp = AReg + (uint32_t)b * (uint32_t)bsz;
                                        float t = 0.0f;
                                        uint32_t ap;

                                        for (i = 0; i < bsz; i++)
                                                raw[i] = byte (bp + i);
                                        n = tvx_qfmt_decode_block ((int)quant_fmt, raw, dec);
                                        if (n <= 0)
                                                break;
                                        ap = BReg + (uint32_t)b * (uint32_t)n * 4u;
                                        for (i = 0; i < n; i++)
                                                t = t + (float)(dec[i] * real32 (ap + 4*i).fp);
                                        acc = acc + t;
                                    }
                                }
                                sntemp1.fp = acc;
                                fp_pushsn (sntemp1);
                           }
                           IPtr++;
                           break;
                /* ---- block rotation (BRQ) ----------------------------
                 * A 32-point randomised Hadamard applied *within* each MX
                 * block.  Global rotation is what QuaRot/SpinQuant do and it
                 * degrades MXFP4 badly (arXiv:2511.04214); confining the
                 * rotation to the block size fixes that, and - because the
                 * dimension is fixed at 32 - makes it one instruction rather
                 * than another GEMV.  Length comes from VSETLEN and must be a
                 * multiple of 32; a trailing partial group is ignored. */
                case 0x1df: /* vhad32   A=dst, B=src, C=sign mask; pop 3    */
                           {
                                float v[32];
                                uint32_t g, ngrp = vec_len / 32, i;

                                BADCODE(IsT414);
                                for (g = 0; g < ngrp; g++) {
                                        uint32_t sp = BReg + g * 128;
                                        uint32_t dp = AReg + g * 128;
                                        for (i = 0; i < 32; i++)
                                                v[i] = real32 (sp + 4*i).fp;
                                        tvx_had32_rotate (v, CReg);
                                        for (i = 0; i < 32; i++) {
                                                fpreal32_t r;
                                                r.fp = v[i];
                                                writereal32 (dp + 4*i, r);
                                        }
                                }
                           }
                           IPtr++;
                           break;
                case 0x1e0: /* mxqrot   A=dst blocks, B=f32 src, C=mask     */
                case 0x1e1: /* mxqrotbf A=dst blocks, B=bf16 src, C=mask    */
                           {
                                float   v[32];
                                uint8_t blk[17];
                                int      is_bf = (OReg == 0x1e1);
                                uint32_t g, ngrp = vec_len / 32, i;

                                BADCODE(IsT414);
                                for (g = 0; g < ngrp; g++) {
                                        uint32_t sp = BReg + g * (is_bf ? 64 : 128);
                                        uint32_t dp = AReg + g * 17;

                                        for (i = 0; i < 32; i++) {
                                                if (is_bf) {
                                                        uint32_t h =
                                                            (uint32_t)byte (sp + 2*i)
                                                          | ((uint32_t)byte (sp + 2*i + 1) << 8);
                                                        v[i] = tvx_bits_to_float (
                                                                tvx_bf16_to_f32 (h));
                                                } else {
                                                        v[i] = real32 (sp + 4*i).fp;
                                                }
                                        }
                                        tvx_had32_rotate (v, CReg);
                                        tvx_mxfp4_quantize_block (blk, v);
                                        for (i = 0; i < 17; i++)
                                                writebyte_int (dp + i, blk[i]);
                                }
                           }
                           IPtr++;
                           break;
                /* ---------------------------------------------------------
                 * TVX-GFX - Floating-point Transform & Project (T&L geometry).
                 * See transputer/t4/tvx_gfx.h for the architectural spec.
                 * --------------------------------------------------------- */
                case 0x1e2: /* ftsetvp  A = viewport descriptor; pop 1      */
                           {
                                float d[5];
                                uint32_t i;

                                BADCODE(IsT414);
                                for (i = 0; i < 5; i++)
                                        d[i] = real32 (AReg + 4*i).fp;
                                tvx_viewport_set (&tvx_vp, d);
                           }
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1e3: /* ftransp  A=dst rec, B=src xyz, C=4x4 mtx     */
                case 0x1e4: /* ftranspb same but VSETLEN vertices, B/A step */
                           {
                                float mtx[16], v[3], out[4];
                                uint32_t n = (OReg == 0x1e4) ? vec_len : 1;
                                uint32_t i, j, and_code = 0x3Fu;

                                BADCODE(IsT414);
                                for (j = 0; j < 16; j++)
                                        mtx[j] = real32 (CReg + 4*j).fp;
                                for (i = 0; i < n; i++) {
                                        uint32_t sp = BReg + ((OReg == 0x1e4)
                                            ? i * TVX_STEP(2u, TVX_POS_DENSE())
                                            : 0);
                                        uint32_t dp = AReg + ((OReg == 0x1e4)
                                            ? i * TVX_STEP(1u,
                                                  4 * TVX_RECORD_WORDS) : 0);
                                        uint32_t code;
                                        fpreal32_t r;

                                        for (j = 0; j < 3; j++)
                                                v[j] = tvx_fetch_pos (sp, j);
                                        code = tvx_transp (out, v, mtx,
                                                           &tvx_vp);
                                        for (j = 0; j < 4; j++) {
                                                r.fp = out[j];
                                                writereal32 (dp + 4*j, r);
                                        }
                                        writeword_int (dp + 16, code);
                                        and_code &= code;
                                }
                                AReg = and_code;
                           }
                           IPtr++;
                           break;
                case 0x1e5: /* fsincos  A=dst {sin,cos}, B=BAMS angle; pop2 */
                           {
                                float s, c;
                                fpreal32_t r;

                                BADCODE(IsT414);
                                tvx_sincos (BReg, &s, &c);
                                r.fp = s;
                                writereal32 (AReg, r);
                                r.fp = c;
                                writereal32 (AReg + 4, r);
                           }
                           AReg = CReg;
                           IPtr++;
                           break;
                case 0x1e6: /* frsqrt   A = bits(1/sqrt(f32 A))             */
                           AReg = tvx_rsqrt_bits (AReg);
                           IPtr++;
                           break;
                case 0x1e7: /* frecip   A = bits(1/f32 A)                   */
                           AReg = tvx_recip_bits (AReg);
                           IPtr++;
                           break;
                case 0x1e8: /* fsincosb A=dst pairs, B=u16 BAMS[]; VSETLEN  */
                           {
                                uint32_t i;

                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        uint32_t sp = BReg
                                            + i * TVX_STEP(2u, 2);
                                        uint32_t dp = AReg
                                            + i * TVX_STEP(1u, 8);
                                        uint32_t a =
                                            (uint32_t)byte (sp)
                                          | ((uint32_t)byte (sp + 1) << 8);
                                        float s, c;
                                        fpreal32_t r;

                                        tvx_sincos (a, &s, &c);
                                        r.fp = s;
                                        writereal32 (dp, r);
                                        r.fp = c;
                                        writereal32 (dp + 4, r);
                                }
                           }
                           AReg = CReg;
                           IPtr++;
                           break;
                case 0x1e9: /* vsetstride A=bytes (0=dense), B=mask; pop 2 */
                           elem_stride = AReg;
                           stride_mask = BReg;
                           AReg = CReg;
                           IPtr++;
                           break;
                case 0x1ec: /* vsetfmt  A = position format; pop 1         */
                           tvx_src_fmt = AReg;
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1ed: /* ftsetlit A = light descriptor; pop 1        */
                           {
                                float d[5];
                                uint32_t i;

                                BADCODE(IsT414);
                                for (i = 0; i < 5; i++)
                                        d[i] = real32 (AReg + 4*i).fp;
                                tvx_light_set (&tvx_lit, d);
                           }
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1ee: /* flitb    A=dst recs, B=normals, C=3x3 mtx   */
                           {
                                float m9[9], n[3];
                                uint32_t i, k, all_dark = 1;

                                BADCODE(IsT414);
                                for (k = 0; k < 9; k++)
                                        m9[k] = real32 (CReg + 4*k).fp;
                                for (i = 0; i < vec_len; i++) {
                                        uint32_t sp = BReg
                                            + i * TVX_STEP(2u,
                                                           TVX_POS_DENSE());
                                        uint32_t dp = AReg
                                            + i * TVX_STEP(1u,
                                                  4 * TVX_RECORD_WORDS);
                                        float I;
                                        fpreal32_t r;

                                        for (k = 0; k < 3; k++)
                                                n[k] = tvx_fetch_pos (sp, k);
                                        I = tvx_lit_vertex (n, m9, &tvx_lit);
                                        r.fp = I;
                                        writereal32 (dp + 20, r);
                                        if (I > tvx_lit.ambient)
                                                all_dark = 0;
                                }
                                AReg = all_dark;
                           }
                           IPtr++;
                           break;
                case 0x1ef: /* qgemm    A=dst f32[], B=param, C=blocks/row */
                           {
                                static T4_THREAD_LOCAL uint8_t raw[256];
                                static T4_THREAD_LOCAL float dec[256];
                                /* bit 0: the vector is unchanged since a
                                 * QGEMM loaded it (timing only) */
                                uint32_t aptr = word (BReg) & ~1u;
                                uint32_t wptr = word (BReg + 4);
                                uint32_t nblk = CReg, r_, b;
                                int bsz = tvx_qfmt_block_bytes
                                              ((int)quant_fmt);
                                int i, n;

                                BADCODE(IsT414);
                                if (tvx_src_fmt == TVX_FMT_QACT) {
                                        AReg = tvx_qgemm_int (AReg, aptr,
                                                              wptr, nblk);
                                        IPtr++;
                                        break;
                                }
                                if (bsz > 0 && bsz <= (int)sizeof(raw)) {
                                    for (r_ = 0; r_ < vec_len; r_++) {
                                        float acc = 0.0f;
                                        uint32_t rp = wptr
                                            + r_ * nblk * (uint32_t)bsz;
                                        uint32_t ap = aptr;
                                        fpreal32_t out;

                                        for (b = 0; b < nblk; b++) {
                                            uint32_t bp = rp
                                                + b * (uint32_t)bsz;
                                            float t = 0.0f;

                                            for (i = 0; i < bsz; i++)
                                                raw[i] = byte (bp + i);
                                            n = tvx_qfmt_decode_block
                                                ((int)quant_fmt, raw, dec);
                                            if (n <= 0)
                                                break;
                                            for (i = 0; i < n; i++) {
                                                float av;
                                                if (tvx_src_fmt == 1) {
                                                    uint32_t h =
                                                      (uint32_t)byte (ap + 2*i)
                                                    | ((uint32_t)byte (ap
                                                        + 2*i + 1) << 8);
                                                    av = tvx_bits_to_float (
                                                        tvx_bf16_to_f32 (h));
                                                } else {
                                                    av = real32 (ap
                                                                 + 4*i).fp;
                                                }
                                                t = t + (float)(dec[i] * av);
                                            }
                                            ap += (uint32_t)n
                                                * (tvx_src_fmt == 1 ? 2u
                                                                    : 4u);
                                            acc = acc + t;
                                        }
                                        out.fp = acc;
                                        writereal32 (AReg + 4 * r_, out);
                                    }
                                }
                                AReg = vec_len;
                           }
                           IPtr++;
                           break;
                case 0x1f0: /* vblend8  A=dst px, B=src px, C=alpha ctl    */
                           {
                                uint32_t i;

                                for (i = 0; i < vec_len; i++) {
                                        uint32_t da = AReg
                                            + i * TVX_STEP(1u, 4);
                                        uint32_t sa = BReg
                                            + i * TVX_STEP(2u, 4);
                                        writeword_int (da,
                                            tvx_blend_pixel (word (da),
                                                             word (sa),
                                                             CReg));
                                }
                           }
                           IPtr++;
                           break;
                case 0x1f1: /* vfill    A=dst, B=pattern, C=byte count     */
                           {
                                uint32_t i;

                                for (i = 0; i < CReg; i++) {
                                        uint32_t a = AReg + i;
                                        writebyte_int (a,
                                            tvx_fill_byte (BReg, a));
                                }
                           }
                           IPtr++;
                           break;
                case 0x1f2: /* vfill2d  BMReg rows/stride; C=width bytes   */
                           {
                                uint32_t r, i;

                                for (r = 0; r < m2dLength; r++) {
                                        uint32_t base = AReg
                                            + r * m2dDestStride;
                                        for (i = 0; i < CReg; i++) {
                                                uint32_t a = base + i;
                                                writebyte_int (a,
                                                    tvx_fill_byte (BReg,
                                                                   a));
                                        }
                                }
                           }
                           IPtr++;
                           break;
                case 0x1f3: /* csum     A=src, B=init, C=bytes; A<-folded  */
                           {
                                uint64_t acc = BReg;
                                uint32_t i, n2 = CReg & ~1u;

                                for (i = 0; i < n2; i += 2)
                                        acc += (uint64_t)
                                            ((uint32_t)byte (AReg + i)
                                           | ((uint32_t)byte (AReg + i + 1)
                                              << 8));
                                if (CReg & 1u)
                                        acc += byte (AReg + n2);
                                acc = (acc & 0xFFFFFFFFu)
                                    + (acc >> 32);
                                AReg = tvx_csum_fold (
                                    tvx_csum_fold ((uint32_t)acc
                                        + (uint32_t)(acc >> 32)));
                           }
                           IPtr++;
                           break;
                case 0x1f4: /* vblend2d BMReg rows/strides; VSETLEN px/row */
                           {
                                uint32_t r, i;

                                for (r = 0; r < m2dLength; r++) {
                                        uint32_t db = AReg
                                            + r * m2dDestStride;
                                        uint32_t sb = BReg
                                            + r * m2dSourceStride;
                                        for (i = 0; i < vec_len; i++)
                                                writeword_int (db + 4*i,
                                                    tvx_blend_pixel (
                                                        word (db + 4*i),
                                                        word (sb + 4*i),
                                                        CReg));
                                }
                           }
                           IPtr++;
                           break;
                case 0x1f6: /* csumcpy  A=dst, B=src, C=bytes; A<-folded   */
                           {
                                uint64_t acc = 0;
                                uint32_t i, n2 = CReg & ~1u;

                                for (i = 0; i < n2; i += 2) {
                                        uint32_t b0 = byte (BReg + i);
                                        uint32_t b1 = byte (BReg + i + 1);
                                        writebyte_int (AReg + i, b0);
                                        writebyte_int (AReg + i + 1, b1);
                                        acc += b0 | (b1 << 8);
                                }
                                if (CReg & 1u) {
                                        uint32_t b0 = byte (BReg + n2);
                                        writebyte_int (AReg + n2, b0);
                                        acc += b0;
                                }
                                acc = (acc & 0xFFFFFFFFu) + (acc >> 32);
                                AReg = tvx_csum_fold (
                                    tvx_csum_fold ((uint32_t)acc
                                        + (uint32_t)(acc >> 32)));
                           }
                           IPtr++;
                           break;
                case 0x1f5: /* stvxstate A = 15-word save area; pop 1      */
                           {
                                fpreal32_t r;
                                uint32_t k;

                                writeword_int (AReg, vec_len);
                                writeword_int (AReg + 4, quant_fmt);
                                writeword_int (AReg + 8, elem_stride);
                                writeword_int (AReg + 12, stride_mask);
                                writeword_int (AReg + 16, tvx_src_fmt);
                                r.fp = tvx_vp.sx;
                                writereal32 (AReg + 20, r);
                                r.fp = tvx_vp.sy;
                                writereal32 (AReg + 24, r);
                                r.fp = tvx_vp.ox;
                                writereal32 (AReg + 28, r);
                                r.fp = tvx_vp.oy;
                                writereal32 (AReg + 32, r);
                                r.fp = tvx_vp.guard;
                                writereal32 (AReg + 36, r);
                                for (k = 0; k < 3; k++) {
                                        r.fp = tvx_lit.dir[k];
                                        writereal32 (AReg + 40 + 4*k, r);
                                }
                                r.fp = tvx_lit.ambient;
                                writereal32 (AReg + 52, r);
                                r.fp = tvx_lit.intensity;
                                writereal32 (AReg + 56, r);
                           }
                           AReg = BReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1f9: /* vcvt     A=dst, B=src, C=dst fmt; pop 3     */
                           /* VSETLEN elements from the VSETFMT format to
                            * C: 0 f32, 1 bf16, 2 f16, round-to-nearest-even
                            * - bf16 is how partial sums cross a link. */
                           {
                                uint32_t i, f;
                                BADCODE(IsT414);
                                for (i = 0; i < vec_len; i++) {
                                        f = tvx_float_to_bits (tvx_fetch_pos (BReg, i));
                                        if (CReg == 1 || CReg == 2) {
                                                f = CReg == 1 ? tvx_f32_to_bf16 (f)
                                                              : tvx_f32_to_f16 (f);
                                                writebyte_int (AReg + 2*i, (u_char)f);
                                                writebyte_int (AReg + 2*i + 1, (u_char)(f >> 8));
                                        } else
                                                writeword_int (AReg + 4*i, f);
                                }
                           }
                           IPtr++;
                           break;
                case 0x1fa: /* vxfer    A=list of {chan,buf,len}, B=count  */
                           BADCODE(IsT414);
                           IPtr++;
                           t9k_vcp_group (AReg, BReg);
                           break;
                case 0x1f7: /* vq8k     A=dst block_q8_K[], B=src; pop 2   */
                case 0x1f8: /* vq80     A=dst block_q8_0[], B=src; pop 2   */
                           BADCODE(IsT414);
                           tvx_vqact (AReg, BReg, OReg == 0x1f7
                                                  ? TVX_GGML_Q8_K
                                                  : TVX_GGML_Q8_0);
                           AReg = CReg;
                           BReg = CReg;
                           IPtr++;
                           break;
                case 0x1ea: /* ftcull   A=dst codes, B=xyz[], C=param      */
                           {
                                float planes[24], v[3];
                                uint32_t nplanes = word (CReg);
                                uint32_t mode    = word (CReg + 4);
                                float    radius  = real32 (CReg + 8).fp;
                                uint32_t xmask   = word (CReg + 12);
                                uint32_t i, k, orbits = 0;
                                uint32_t sstep = TVX_STEP(2u, TVX_POS_DENSE());

                                BADCODE(IsT414);
                                if (nplanes > 6) nplanes = 6;
                                for (k = 0; k < nplanes * 4; k++)
                                        planes[k] =
                                            real32 (CReg + 16 + 4*k).fp;
                                for (i = 0; i < vec_len; i++) {
                                        uint32_t sp = BReg + i * sstep;
                                        for (k = 0; k < 3; k++)
                                                v[k] = tvx_fetch_pos (sp, k);
                                        if (mode == 2) {
                                                fpreal32_t r;
                                                r.fp = tvx_plane_dist
                                                    (planes, v);
                                                writereal32 (AReg
                                                    + i * TVX_STEP(1u, 4), r);
                                        } else {
                                                uint32_t c = tvx_cull_code
                                                    (planes, nplanes, v,
                                                     mode, radius, xmask);
                                                writebyte_int (AReg
                                                    + i * TVX_STEP(1u, 1), c);
                                                orbits |= c;
                                        }
                                }
                                AReg = (mode == 2) ? 0 : orbits;
                           }
                           IPtr++;
                           break;
                case 0x1eb: /* tskin    A=dst xyz, B=joints, C=param       */
                           {
                                uint32_t mode = word (CReg);
                                uint32_t wp   = word (CReg + 4);
                                uint32_t ip   = word (CReg + 8);
                                uint32_t i, k, j = 0;
                                uint32_t dstep = TVX_STEP(1u, 12);
                                float mm[6];

                                BADCODE(IsT414);
                                if (mode & 1u)
                                        for (k = 0; k < 6; k++)
                                                mm[k] = real32
                                                    (CReg + 12 + 4*k).fp;
                                for (i = 0; i < vec_len; i++) {
                                        float acc[3], w[4], m[12];
                                        uint32_t off, flag;
                                        fpreal32_t r;

                                        acc[0] = acc[1] = acc[2] = 0.0f;
                                        do {
                                                off  = word (ip + 8*j);
                                                flag = word (ip + 8*j + 4);
                                                for (k = 0; k < 4; k++)
                                                        w[k] = real32
                                                            (wp + 16*j
                                                             + 4*k).fp;
                                                for (k = 0; k < 12; k++)
                                                        m[k] = real32
                                                            (BReg + off
                                                             + 4*k).fp;
                                                tvx_skin_acc (acc, m, w);
                                                j++;
                                        } while (flag == 0);
                                        for (k = 0; k < 3; k++) {
                                                r.fp = acc[k];
                                                writereal32 (AReg + i*dstep
                                                             + 4*k, r);
                                        }
                                        if (mode & 1u)
                                                for (k = 0; k < 3; k++) {
                                                        if (acc[k] < mm[k])
                                                                mm[k] = acc[k];
                                                        if (acc[k] > mm[k+3])
                                                                mm[k+3] =
                                                                    acc[k];
                                                }
                                }
                                if (mode & 1u) {
                                        fpreal32_t r;
                                        for (k = 0; k < 6; k++) {
                                                r.fp = mm[k];
                                                writereal32 (CReg + 12
                                                             + 4*k, r);
                                        }
                                }
                                AReg = j;
                           }
                           IPtr++;
                           break;
                case 0x17c: /* lddevid */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = ProductIdentity;
                           IPtr++;
                           break;
		case 0xFFFFFFF9u: /* selth - select trap-handler (T9000, mem 60F9) */
		case 0x60F9:      /* selth via the kernel stub's pfix-built OReg */
                           /* ThReg := Areg; stacks undefined (popped).  The
                            * ISM status/control-bit exchange with the THDS is
                            * omitted - L-trap delivery is not modelled. */
                           temp = ThReg;
                           ThReg = AReg;
                           AReg = temp;
                           BReg = CReg;
			   IPtr++;
			   break;
		case 0xFFFFFFFFu: /* fpstall - store all FPU state (T9000, 6X FF) */
                           {
                               uint32_t ptr = AReg;
                               writeword (ptr,
                                          t9k_fp_lenc (FAReg.length)
                                        | (t9k_fp_lenc (FBReg.length) << 2)
                                        | (t9k_fp_lenc (FCReg.length) << 4)
                                        | (FP_Error ? (1u << 6) : 0u)
                                        | (((uint32_t)RoundingMode & 7u) << 8));
                               t9k_fp_store_reg (ptr + 4,  &FAReg);
                               t9k_fp_store_reg (ptr + 12, &FBReg);
                               t9k_fp_store_reg (ptr + 20, &FCReg);
                               AReg = BReg;
                               BReg = CReg;
                               FAReg.length = FP_UNKNOWN;
                               FBReg.length = FP_UNKNOWN;
                               FCReg.length = FP_UNKNOWN;
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFFEu: /* fpldall - load all FPU state (T9000, 6X FE) */
                           {
                               uint32_t ptr = AReg;
                               uint32_t st = word (ptr);
                               t9k_fp_load_reg (ptr + 4,  &FAReg, t9k_fp_ldec (st & 3u));
                               t9k_fp_load_reg (ptr + 12, &FBReg, t9k_fp_ldec ((st >> 2) & 3u));
                               t9k_fp_load_reg (ptr + 20, &FCReg, t9k_fp_ldec ((st >> 4) & 3u));
                               FP_Error = (st & (1u << 6)) ? TRUE : FALSE;
                               fp_setrounding ("fpldall", (int)((st >> 8) & 7u));
                               AReg = BReg;
                               BReg = CReg;
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFF5u: /* wait - semaphore P (T9000, ISM 8.6) */
                           {
                               uint32_t sem = AReg;
                               uint32_t count = word (sem);
                               AReg = BReg;
                               BReg = CReg;
                               IPtr++;
                               if (count != 0)
                               {
                                   writeword (sem, count - 1);
                               }
                               else
                               {
                                   /* Append current process to the semaphore
                                    * queue (s.Front @4, s.Back @8) and
                                    * deschedule. */
                                   uint32_t wdesc = WPtr | ProcPriority;
                                   uint32_t front = word (sem + 4);
                                   if (front == NotProcess_p)
                                       writeword (sem + 4, wdesc);
                                   else
                                       writeword (index (GetDescWPtr (word (sem + 8)), Link_s), wdesc);
                                   writeword (sem + 8, wdesc);
                                   deschedule ();
                               }
                           }
			   break;
		case 0xFFFFFFF4u: /* signal - semaphore V (T9000, ISM 8.6) */
                           {
                               uint32_t sem = AReg;
                               uint32_t front = word (sem + 4);
                               AReg = BReg;
                               BReg = CReg;
                               if (front == NotProcess_p)
                               {
                                   writeword (sem, word (sem) + 1);
                               }
                               else
                               {
                                   if (front == word (sem + 8))
                                       writeword (sem + 4, NotProcess_p);
                                   else
                                       writeword (sem + 4,
                                                  word (index (GetDescWPtr (front), Link_s)));
                                   schedule (front);
                               }
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFF3u: /* timeslice - force a timeslice (T9000) */
			   IPtr++;
                           reschedule ();
                           IntsDisabled = 0;
                           SetGotoSNP;
			   break;
		case 0xFFFFFFF2u: /* insertqueue (T9000, ISM 13.7) */
                           {
                               uint32_t pri = AReg & 1u;
                               if (BReg != NotProcess_p)
                               {
                                   uint32_t oldf = FPtrReg[pri];
                                   FPtrReg[pri] = BReg;
                                   if (oldf == NotProcess_p)
                                       BPtrReg[pri] = CReg;
                                   else
                                       writeword (index (CReg, Link_s), oldf);
                               }
                               /* integer stack left undefined */
                               AReg = BReg;
                               BReg = CReg;
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFF1u: /* swaptimer (T9000, ISM 13.7) */
                           {
                               uint32_t pri = AReg & 1u;
                               uint32_t oldf = TPtrLoc[pri];
                               TPtrLoc[pri] = BReg;
                               AReg = oldf;
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFF0u: /* swapqueue (T9000, ISM 13.7) */
                           {
                               uint32_t pri  = AReg & 1u;
                               uint32_t oldf = FPtrReg[pri];
                               uint32_t oldb = BPtrReg[pri];
                               FPtrReg[pri] = BReg;
                               BPtrReg[pri] = CReg;
                               AReg = oldf;
                               BReg = oldb;
                           }
			   IPtr++;
			   break;
		case 0xFFFFFFE0u: /* stmove2dinit - store 2D move state (T9000) */
                           writeword (AReg,     m2dLength);
                           writeword (AReg + 4, m2dDestStride);
                           writeword (AReg + 8, m2dSourceStride);
                           AReg = BReg;
                           BReg = CReg;
			   IPtr++;
			   break;
		case 0xFFFFFFDFu: /* causeerror (T9000) */
                           SetError;
			   IPtr++;
			   break;
		case 0xFFFFFFD4u: /* devmove - device block move (T9000) */
                           /* Same data movement as move; the device ordering
                            * guarantees are trivially met (no reordering). */
                           if (INT32(AReg) > 0)
                               movebytes_int (BReg, CReg, AReg);
                           AReg = BReg = CReg = Undefined_p;
			   IPtr++;
			   break;
		case 0xFFFFFFD3u: /* icl - invalidate cache line  (T9000): no cache */
		case 0xFFFFFFD2u: /* fdcl - flush dirty cache line (T9000) */
			   IPtr++;
			   break;
		case 0xFFFFFFD1u: /* ica - invalidate cache address (T9000) */
		case 0xFFFFFFD0u: /* fdca - flush dirty cache address (T9000) */
                           AReg = BReg;
                           BReg = CReg;
			   IPtr++;
			   break;
		case 0xFFFFFFC0u: /* nop (T9000) */
			   IPtr++;
			   break;
		case 0xFFFFFF7Cu: /* ldprodid (T9000) */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = ProductIdentity;
			   IPtr++;
			   break;
		/* T9000 VCP, channel and resource instructions (t9000_vcp.c) */
		case 0xFFFFFFEEu: /* stopch */
		case 0xFFFFFFEDu: /* vout */
		case 0xFFFFFFECu: /* vin */
		case 0xFFFFFFE9u: /* swapbfr */
		case 0xFFFFFFE8u: /* sethdr */
		case 0xFFFFFFE7u: /* setchmode */
		case 0xFFFFFFE6u: /* initvlcb */
		case 0xFFFFFFE5u: /* writehdr */
		case 0xFFFFFFE4u: /* readhdr */
		case 0xFFFFFFE3u: /* disg */
		case 0xFFFFFFE2u: /* enbg */
		case 0xFFFFFFE1u: /* grant */
		case 0xFFFFFFDDu: /* unmkrc */
		case 0xFFFFFFDCu: /* mkrc */
		case 0xFFFFFFDBu: /* irdsq */
		case 0xFFFFFFDAu: /* erdsq */
		case 0xFFFFFFD9u: /* stresptr */
		case 0xFFFFFFD8u: /* ldresptr */
			   IPtr++;
			   t9k_vcp_exec (OReg);
			   break;
		case 0xFFFFFFFDu: /* stshadow (T9000, ISM 13.3) */
		case 0xFFFFFFFCu: /* ldshadow */
                           shadow_transfer (AReg, BReg, CReg, OReg == 0xFFFFFFFDu);
                           AReg = BReg = CReg = Undefined_p;
			   IPtr++;
			   break;
		case 0xFFFFFFDEu: /* restart - resume an interrupted L-process (ISM 13.4) */
                           {
                               uint32_t pds = AReg;

                               if (pds & ByteSelectMask)
                               {
                                   SetError;
                                   IPtr++;
                                   break;
                               }
                               if (NotProcess_p != ThReg)
                                   writeword (ThReg, t9k_status_pack (STATUSReg, FP_Error,
                                                                      TimesliceDisabled, 0));
                               UpdateWdescReg (GetDescWPtr(word (index (pds, T9K_PS_SWPTR))) | ProcPriority);
                               IPtr = word (index (pds, T9K_PS_SIPTR));
                               AReg = word (index (pds, T9K_PS_SAREG));
                               BReg = word (index (pds, T9K_PS_SBREG));
                               CReg = word (index (pds, T9K_PS_SCREG));
                               EReg = word (index (pds, T9K_PS_SEREG));
                               XReg = word (index (pds, T9K_PS_SXREG));
                               temp = word (index (pds, T9K_PS_CNTL));
                               STATUSReg = t9k_status_raw (temp) | (STATUSReg & GotoSNPBit);
                               TimesliceDisabled = (temp & SB_TSDIS) != 0;
                               FP_Error = (temp & SB_FPERROR) ? TRUE : FALSE;
                               IntsDisabled = 0;
                           }
			   break;
		case 0xFFFFFFFBu: /* tret - trap return (ISM 10.4, 13.2.2) */
                           /* Wptr is the THDS; the trap-handler itself must
                            * have a null trap-handler. */
                           if (NotProcess_p != ThReg)
                           {
                               SetError;
                               IPtr++;
                               break;
                           }
                           {
                               uint32_t thds = WPtr, front = word (index (thds, 2));
                               uint32_t cntl = word (thds) & 0x7FFFFFFFu;  /* sb.ThInUse */

                               writeword (thds, cntl);
                               if (NotProcess_p != front)
                               {
                                   /* trap-sharing queue goes to the front of
                                    * the scheduling list at this priority */
                                   uint32_t back = word (index (thds, 3));
                                   uint32_t oldf = FPtrReg[ProcPriority];

                                   FPtrReg[ProcPriority] = front;
                                   if (NotProcess_p == oldf)
                                       BPtrReg[ProcPriority] = back;
                                   else
                                       writeword (index (back, Link_s), oldf);
                                   writeword (index (thds, 2), NotProcess_p);
                               }
                               if (0 == AReg)
                               {
                                   UpdateWdescReg (GetDescWPtr(word (index (thds, 7))) | ProcPriority);
                                   IPtr = word (index (thds, 8));
                                   AReg = word (index (thds, 9));
                                   BReg = word (index (thds, 10));
                                   CReg = word (index (thds, 11));
                                   STATUSReg = t9k_status_raw (cntl) | (STATUSReg & GotoSNPBit);
                                   TimesliceDisabled = (cntl & SB_TSDIS) != 0;
                                   FP_Error = (cntl & SB_FPERROR) ? TRUE : FALSE;
                                   ThReg = thds;
                               }
                               else
                               {
                                   /* terminate the trapped process */
                                   IPtr++;
                                   SetGotoSNP;
                               }
                           }
			   break;
		case 0xFFFFFFF7u: /* swapgstatus - not in the ISM (tables B.4, C) */
                           printf ("-E-EMU9000: swapgstatus (OReg #%08X) is not an IMS T9000 instruction\n", OReg);
                           maybe_log_nearest_sym(IPtr, "t9k-not-ism");
                           processor_state ();
			   handler (-1);
			   break;
		case T9K_OP_SYSCALL: /* OReg -8 — system call (T9000, mem 60F8) */
			   t9k_syscall();
			   break;
		case T9K_OP_GOPROT: /* OReg -6 — go protected (T9000, mem 60FA) */
			   /* Areg = PDS, Breg = RODS (physical while in L) */
			   t9k_goprot(AReg, BReg);
			   break;
                case 0x1ff: /* XXX start    */
                           quit = TRUE;
                           IPtr++;
                           break;
		default  : 
#ifdef EMUDEBUG
BadCode:
#endif
                           printf ("-E-EMU414: Error - bad Icode! (#%02X - %s)\n", OReg, mnemonic (Icode, OReg, AReg, 0));
                           maybe_log_nearest_sym(IPtr, "bad-icode");
                           processor_state ();
                           handler (-1);
                           break;
	} /* switch OReg */
			   break;
		default  : 
                           printf ("-E-EMU414: Error - bad Icode! (#%02X - %s)\n", OReg, mnemonic (Icode, OReg, AReg, 0));
                           maybe_log_nearest_sym(IPtr, "bad-icode");
                           processor_state ();
                           handler (-1);
                           break;
	} /* switch (Icode) */

        /* Clear operand accumulator after executing a non-prefix instruction. */
        if (Icode != 0x20 && Icode != 0x60) {
                OReg = 0;
        }
                /* Reset rounding mode to round nearest. */
                if (ResetRounding && (RoundingMode != ROUND_N))
                        fp_setrounding ("reset", ROUND_N);

		PROFILE(profile[PRO_INSTR]++);

                /* Halt when Error flag was set */
#ifndef EMUDEBUG
FastPrimaryError:
#endif
		if ((!PrevError && ReadError) &&
                    (exitonerror || (ReadHaltOnError))) {
			fprintf(stderr, "[EXIT] Error flag set! IPtr=0x%08X WPtr=0x%08X AReg=0x%08X exitonerror=%d HaltOnError=%d heartbeat=%llu\n",
				IPtr, WPtr, AReg, exitonerror, ReadHaltOnError, (unsigned long long)heartbeat_counter);
			fflush(stderr);
			break;
		}
		if (quit == TRUE) {
			fprintf(stderr, "[EXIT] quit=TRUE IPtr=0x%08X WPtr=0x%08X heartbeat=%llu\n",
				IPtr, WPtr, (unsigned long long)heartbeat_counter);
			fflush(stderr);
			break;
		}

#ifndef NDEBUG
                fp_chkexcept ("mainloop");
#endif
	}
#ifdef T4PROFILE
        if (profiling)
        {
                update_tod (&EndTOD);
                ElapsedSecs = (EndTOD.tv_sec - StartTOD.tv_sec) * 1000 +
                              (EndTOD.tv_usec - StartTOD.tv_usec) / 1000;
                profile[PRO_ELAPSEDMS] = ElapsedSecs;
        }
#endif
	if (ReadError)
	{
                if (ReadHaltOnError)
                        printf ("-I-EMU414: Transputer HaltOnError flag was set.\n");
		printf ("-I-EMU414: Transputer Error flag is set.\n");

                processor_state ();

		/* Save dump file for later debugging if needed. *****/
                printf ("-I-EMU414: Saving memory dump.\n");
                save_dump ();
                printf ("-I-EMU414: Done.\n");
	}
}
#undef  Icache
#undef  Acache
#define Icache        (t4cpu->s_Icache)
#define Acache        (t4cpu->s_Acache)


void Enqueue (uint32_t wptr, uint32_t pri)
{
	uint32_t ptr;

        /* Get front of process list pointer. */
        EMUDBG2 ("-I-EMUDBG: Enqueue(1): Get front of %s priority process list pointer.\n",
                        StrPrio(pri));

        ptr = FPtrReg[pri];
	if (ptr == NotProcess_p)
	{
                EMUDBG ("-I-EMUDBG: Enqueue(2): Empty process list, create.\n");

	        /* Empty process list. Create. */
	        FPtrReg[pri] = wptr;
	        BPtrReg[pri] = wptr;
	}
	else
	{
	        /* Process list already exists. Update. */
                EMUDBG ("-I-EMUDBG: Enqueue(2): Update process list.\n");

		/* Get workspace pointer of last process in list. */
		ptr = BPtrReg[pri];

		/* Link new process onto end of list. */
		writeword (index (ptr, Link_s), wptr);

		/* Update end-of-process-list pointer. */
		BPtrReg[pri] = wptr;
	}
}

/* Make a process ready from inside an instruction that may already have
 * descheduled the current one.  schedule() treats the current low-priority
 * process as running and would save it as the interrupted process when a
 * high-priority process becomes ready; a descheduled process is waiting,
 * so the high-priority one just joins its queue. */
void t4_wake (uint32_t wdesc)
{
        if (ReadGotoSNP && LoPriority == ProcPriority &&
            HiPriority == GetDescPriority(wdesc))
                Enqueue (GetDescWPtr(wdesc), HiPriority);
        else
                schedule (wdesc);
}

/* The VCP finished wdesc's transfer at cycle t.  An idle CPU waits for it.
 * A busy one does not: the VCP works beside the CPU, so wdesc is held and
 * joins the queue when the CPU's clock reaches t (t9k_net_release).
 * Called at a point where schedule() may switch processes. */
void t9k_net_wake (uint32_t wdesc, uint64_t t)
{
        int n = t9k.net_npend;

        if (Idle || t <= t9k_cycles_so_far () ||
            n == (int)(sizeof t9k.net_pend / sizeof t9k.net_pend[0])) {
                t9k_advance_to (t);
                t4_wake (wdesc);
                return;
        }
        t9k.net_pend[n].wdesc = wdesc;
        t9k.net_pend[n].t = t;
        t9k.net_npend = n + 1;
        if (n == 0 || t < t9k.net_pend_min)
                t9k.net_pend_min = t;
}

/* Wake the held processes whose transfers are done by now - when idle,
 * after waiting for the first of them. */
static void t9k_net_release (int idle)
{
        uint64_t now;
        int i;

        if (idle)
                t9k_advance_to (t9k.net_pend_min);
        now = t9k_cycles_so_far ();
        t9k.net_pend_min = UINT64_MAX;
        for (i = 0; i < t9k.net_npend; ) {
                if (t9k.net_pend[i].t <= now) {
                        uint32_t w = t9k.net_pend[i].wdesc;

                        t9k.net_pend[i] = t9k.net_pend[--t9k.net_npend];
                        t4_wake (w);
                } else {
                        if (t9k.net_pend[i].t < t9k.net_pend_min)
                                t9k.net_pend_min = t9k.net_pend[i].t;
                        i++;
                }
        }
}

uint32_t t4_product_identity (void)
{
        return ProductIdentity;
}

/* CPU control for the T9000 control system (t9000_ctl.c). */

/* StatusReg in the ISM table 5.5 layout, for the state block a Run or
 * Reboot hands to the new process (HRM 8.5.4). */
uint32_t t4_status_reg (void)
{
        return t9k_status_pack (STATUSReg, FP_Error, TimesliceDisabled, t9k_in_p ());
}

/* Stop the CPU: now, or (Stop command, HRM 8.5.4) when the current process
 * next deschedules or reaches a timeslice point.  start_process() then
 * returns from mainloop(). */
void t4_cpu_halt (int now)
{
        t4cpu->s_ctl_halt = 1;
        if (now)
                SetGotoSNP;
}

/* "CPU cleared" (HRM table 8.10): stopped, Wdesc NotProcess.p, scheduling
 * and timer queues empty (HRM 8.7.1). */
void t4_cpu_clear (void)
{
        FPtrReg[0] = FPtrReg[1] = NotProcess_p;
        BPtrReg[0] = BPtrReg[1] = NotProcess_p;
        TPtrLoc[0] = TPtrLoc[1] = NotProcess_p;
        ClearInterrupt;
        UpdateWdescReg (NotProcess_p | HiPriority);
        STATUSReg = 0;
        IntsDisabled = 0;
        TimesliceDisabled = 0;
        ThReg = NotProcess_p;
}

/* Run or Reboot: the CPU starts the process wdesc at iptr. */
void t4_cpu_start (uint32_t wdesc, uint32_t iptr)
{
        UpdateWdescReg (wdesc);
        IPtr = iptr;
        ClearGotoSNP;
        t4cpu->s_ctl_halt = 0;
}

/* Add a process to the relevant priority process queue. */
void schedule (uint32_t wdesc)
{
        uint32_t wptr, pri;
	uint32_t temp;

        wptr = GetDescWPtr(wdesc);
        if (NotProcess_p == wptr)
                return;
        pri  = GetDescPriority(wdesc);

#ifdef EMUDEBUG
        if (emudebug)
        {
                printf ("-I-EMUDBG: Schedule(1): Process = #%08X at priority %s\n", wptr, StrPrio(pri));
                printf ("-I-EMUDBG: Schedule(1): %s Idle=%d FPtrReg[Hi]=#%08X FPtrReg[Lo]=#%08X Int=#%08X\n",
                        StrPrio(ProcPriority), Idle, FPtrReg[0], FPtrReg[1], word (0x8000002C));
        }
#endif

	/* Remove from timer queue if a ready alt. */
        /* !!! XXX - READ OF NOT INITIALIZED MEMORY !!! */
	temp = word_int (index (wptr, State_s));
	if (temp == Ready_p)
		purge_timer ();

        if (HiPriority == ProcPriority)
        {
                EMUDBG ("-I-EMUDBG: Processor at Hi priority. Enqueue\n");
                Enqueue (wptr, pri);
        }
        else if (LoPriority == ProcPriority)
        {
                if (HiPriority == pri)
                {
                        EMUDBG ("-I-EMUDBG: Processor at Lo priority. Activate Hi priority process\n");

		        interrupt ();

                        /* Preserve Error and HaltOnError flags only. */
                        STATUSReg &= (ErrorFlag | HaltOnErrorFlag);

                        /* ??? HaltOnErrorFlag is cleared before the process starts. */
                        /* ClearHaltOnError; */

                        EMUDBG2 ("-I-EMU414: Activate Hi priority process Wdesc=#%08X\n", wdesc);

                        UpdateWdescReg (wdesc);
                        T4DEBUG(checkWPtr ("Schedule", WPtr));
		        IPtr = word (index (WPtr, Iptr_s));
                }
                else if (LoPriority == pri)
                {
                        EMUDBG3 ("-I-EMUDBG: Processor at Lo priority. Idle=%d WPtr=#%08X\n", Idle, WPtr);
                        if (Idle)
                        {
                                EMUDBG2 ("-I-EMU414: Lo priority process list empty. Activate process Wdesc=#%08X\n", wdesc);
                                UpdateWdescReg (wdesc);
                                T4DEBUG(checkWPtr ("Schedule", WPtr));
		                IPtr = word (index (WPtr, Iptr_s));
                        }
                        else
                                Enqueue (wptr, LoPriority);
                }
        }
}

/* Run a process, HiPriority if available. */
int run_process (void)
{
	uint32_t ptr;
	uint32_t lastptr;

	/* Is the HiPriority process list non-empty? */
	if (NotProcess_p != FPtrReg[0])
	{
	        EMUDBG ("-I-EMUDBG: RunProcess: HiPriority process list non-empty.\n");
		/* There is a HiPriority process available. */
		ProcPriority = HiPriority;
	}
        /* Is there an interrupted LoPriority process? */
        else if (ReadInterrupt)
        {
	        EMUDBG ("-I-EMUDBG: RunProcess: There is an interrupted LoPriority process.\n");
                ProcPriority = LoPriority;
        }
	else if (NotProcess_p != FPtrReg[1])
	{
	        EMUDBG ("-I-EMUDBG: RunProcess: LoPriority process list non-empty.\n");
		/* There are only LoPriority processes available. */
		ProcPriority = LoPriority;
	}
        else
        {
                EMUDBG ("-I-EMUDBG: RunProcess: Empty process list. Cannot start!\n");

		/* Empty process list. Cannot start! */
                UpdateWdescReg (IdleProcess_p);
                return (-1);
        }

	/* Get front of process list pointer. */
	ptr = FPtrReg[ProcPriority];
	if ((ProcPriority == LoPriority) && (ReadInterrupt))
	{
		/* Return to interrupted LoPriority process. */
		shadow_resume ();
                T4DEBUG(checkWPtr ("RunProcess(1)", WPtr));
	        EMUDBG3 ("-I-EMUDBG: RunProcess: ProcPriority = %s, WPtr = #%08X.\n",
                                StrPrio(ProcPriority), WPtr);
	}
	else
	{
	        lastptr = BPtrReg[ProcPriority];

		/* AUDIT: an interrupted lo-pri process must only resume via
		 * the register-restoring branch above.  If it is ALSO on the
		 * run queue and gets dequeued here, it runs with stale
		 * A/B/C - the suspected cause of the nondeterministic
		 * garbage-argument hangs (see pcpu heisenbug). */
		{
			static int audit = -1;
			if (audit < 0) {
				const char *e = getenv("T4_INT_AUDIT");
				audit = (e && *e) ? atoi(e) : 0;
			}
			if (audit >= 1 && ReadInterrupt &&
			    (ProcPriority == LoPriority) &&
			    GetDescWPtr(ptr) ==
			        GetDescWPtr(t4cpu->s_sh.wdesc)) {
				fprintf(stderr,
					"[INT-ALARM] interrupted process "
					"0x%08X resumed via QUEUE (A/B/C "
					"lost) at IPtr=0x%08X\n",
					ptr, IPtr);
				fflush(stderr);
			}
		}

	        EMUDBG5 ("-I-EMUDBG: RunProcess: ProcPriority = %s, ptr = #%08X. FPtrReg[Hi] = #%08X, FPtrReg[Lo] = #%08X.\n",
                                StrPrio(ProcPriority), ptr, FPtrReg[0], FPtrReg[1]);

		if (ptr == lastptr)
		{
			/* Only one process in list. */
			UpdateWdescReg (ptr | ProcPriority);
                        T4DEBUG(checkWPtr ("RunProcess(2)", WPtr));

			/* Get Iptr. */
			IPtr = word (index (WPtr, Iptr_s));

			/* Empty list now. */
			FPtrReg[ProcPriority] = NotProcess_p;
		}
		else
		{
			/* List. */
			UpdateWdescReg (ptr | ProcPriority);
                        T4DEBUG(checkWPtr ("RunProcess(3)", WPtr));

			/* Get Iptr. */
			IPtr = word (index (WPtr, Iptr_s));

			/* Point at second process in chain. */
			FPtrReg[ProcPriority] = word (index (WPtr, Link_s));
		}
	}

	return (0);
}

/* Start a process. */
int start_process (void)
{
        int active, links_active;

        /* IntEnabled is always TRUE.
         * if ((ProcPriority == LoPriority) && !IntEnabled)
         *        return;
         */

        /* First, clear GotoSNP flag. */
        ClearGotoSNP;

	/* Second, handle any host link communication. */
        do
        {
                active = TRUE;
                /* the control system has stopped the CPU (t9000_ctl.c) */
                if (t4cpu->s_ctl_halt)
                        return 1;
                if (0 == run_process ())
                        break;
                /* nothing to run: wait for a process the VCP holds */
                if (t9k.net_npend) {
                        t9k_net_release (1);
                        if (!Idle)
                                break;
                        continue;
                }

#ifdef EMUDEBUG
                if (emudebug)
                {
		        printf ("-I-EMUDBG: StartProcess: Empty process list. Update comms.\n");
                        fflush (stdout);
                }
#endif

                /* Update host comms. */
                active = FALSE;
                if (serve)
                {
		        active = 0 != server ();
                        if (!Idle)
                        {
                                active = TRUE;
                                break;
                        }
                }

                if (Idle && ProcessQEmpty && TimerQEmpty)
                        links_active = (0 != linkcomms ("idle", FALSE, LTO_IDLE));
                else
                        links_active = (0 != linkcomms ("running", FALSE, LTO_COMM));
                active = active || links_active;
                if (!Idle)
                {
                        active = TRUE;
                        break;
                }

		/* Check timer queue, update timers.  The timer thread moves
                 * ClockReg; the queues are still ours to run, or an idle
                 * processor never wakes a process waiting in tin. */
                active = active || (!TimerQEmpty);
                if (g_timer_threaded)
                {
                        if (TimerEnableHi) schedule_timerq (HiPriority);
                        if (TimerEnableLo) schedule_timerq (LoPriority);
                }
                else if (++count1 > delayCount1 * speed_multiplier)
		        update_time ();
                if (!Idle)
                {
                        active = TRUE;
                        break;
                }
                if (SharedEvents && (nodeid >= 0))
                {
                        if (SharedEvents[nodeid])
                                return 1;
                }
                if (t4_fabric && nodeid >= 0 && t4_fabric->events && t4_fabric->events[nodeid])
                        return 1;
                if (t4_net_stop_requested())
                        return 1;
                /* A pending SIGINT/SIGTERM is normally serviced at
                 * ExecuteInstr, which an idle emulator never reaches: a
                 * guest parked on a timer that never fires kept
                 * `timeout 8s t4` running for hours. */
                if (__builtin_expect (t4_sig_pending != 0, 0)) {
                        int sig = (int) t4_sig_pending;
                        t4_sig_pending = 0;
                        t4_signal_exit (sig);
                }
        } while (active);

        if (!active)
        {
                printf ("-E-EMU414: Node%d Error - stopped no Link/Process/Timer activity!\n", nodeid < 0 ? 0 : nodeid);
                processor_state ();
                if (t4_fabric) {
                        t4_net_request_stop();
                        return 1;
                }
                handler (-1);
        }

	/* Reset timeslice counter. */
	timeslice = 0;

	PROFILE(profile[PRO_STARTP]++);

        return 0;
}

/* Save the current process and start a new process. */
void deschedule (void)
{
#ifndef NDEBUG
        if (NotProcess_p == WPtr)
        {
                printf ("-E-EMU414: Cannot deschedule NotProcess_p.\n");
                handler (-1);
        }
#endif
        EMUDBG2 ("-I-EMUDBG: Deschedule process #%08X.\n", Wdesc);
        /* ISM 13.6: interruption is automatically re-enabled when a
         * process is descheduled. */
        IntsDisabled = 0;
        /* Write Iptr into workspace */
	writeword (index (WPtr, Iptr_s), IPtr);

        /* Set StartNewProcess flag. */
        SetGotoSNP;
}

/* Save the current process and place it on the relevant priority process queue. */
void reschedule (void)
{
#ifndef NDEBUG
        if (NotProcess_p == WPtr)
        {
                printf ("-E-EMU414: Cannot reschedule NotProcess_p.\n");
                handler (-1);
        }
#endif
	/* Write Iptr into worksapce. */
	writeword (index (WPtr, Iptr_s), IPtr);

	/* Put on process list. */
	schedule (WPtr | ProcPriority);
}

/* Check whether the current process needs rescheduling,  */
/* i.e. has executed for a timeslice period.              */
void D_check (void)
{
	/* Called only from 'j' and 'lend'. */

	/* First, handle any host link communication. */
        /* IntEnabled is always TRUE.
         * if ((ProcPriority == HiPriority) || IntEnabled)
         */
        {
	        if (serve) server ();
                linkcomms ("pri", FALSE, LTO_HI);
        }

        /* High priority processes never timesliced. */
        if (ProcPriority == HiPriority)
                return;

        /* T9000: settimeslice(false)/intdis suppress timeslicing. */
        if (TimesliceDisabled || IntsDisabled)
                return;

	/* Check for timeslice. */
	if (timeslice > 1)
	{
                EMUDBG2 ("-I-EMUDBG: Timeslice process #%08X.\n", Wdesc);

		/* Must change process! */
		timeslice = 0;

		/* reschedule really moves the process to the end of the queue! */
		reschedule ();

		/* Set StartNewProcess flag. */
                SetGotoSNP;
	}
	PROFILE(profile[PRO_DCHECK]++);
}

/* Interrupt a low priority process.                    */
/* Can only occur due to comms or timer becoming ready. */
void interrupt (void)
{
	/* A high priority process has become ready, interrupting a low priority one. */

#ifndef NDEBUG
	/* Sanity check. */
	if (ReadInterrupt)
	{
		printf ("-E-EMU414: Error - multiple interrupts of low priority processes!\n");
		handler (-1);
	}
#endif

	/* Store the registers. */
	{
		static int audit = -1;
		if (audit < 0) {
			const char *e = getenv("T4_INT_AUDIT");
			audit = (e && *e) ? atoi(e) : 0;
		}
		if (audit >= 2)
			fprintf(stderr,
				"[INT] IPtr=0x%08X Wdesc=0x%08X A=0x%08X "
				"B=0x%08X C=0x%08X\n",
				IPtr, Wdesc, AReg, BReg, CReg);
	}
        if (IdleProcess_p != Wdesc)
                EMUDBG ("-I-EMUDBG: Interrupt LoPriority process.\n");
        shadow_save ();
        /* Note: that an interrupted process is not placed onto the scheduling lists. */
}

/* ---- T9000 shadow state (ISM 13.3) --------------------------------------- */

static uint32_t t9k_status_pack (uint32_t raw, int fp_error, int tsdis, int is_p)
{
        return ((raw & ErrorFlag) ? SB_INTOV : 0u)
             | ((raw & HaltOnErrorFlag) ? SB_T4HALT : 0u)
             | (fp_error ? SB_FPERROR : 0u)
             | (tsdis ? SB_TSDIS : 0u)
             | (is_p ? SB_ISP : 0u);
}

static uint32_t t9k_status_raw (uint32_t sb)
{
        return ((sb & SB_INTOV) ? ErrorFlag : 0u)
             | ((sb & SB_T4HALT) ? HaltOnErrorFlag : 0u);
}

/* Full context switch on interruption: the low-priority process's state goes
 * into the shadow registers.  A null interrupted process is recorded as
 * WdescReg.sh = NotProcess.p, StatusReg.sh = 0. */
static void shadow_save (void)
{
        t9k_shadow_t *sh = &t4cpu->s_sh;
        int i;

        sh->th = ThReg;
        if (IdleProcess_p == Wdesc)
        {
                sh->wdesc = NotProcess_p;
                sh->status = 0;
        }
        else
        {
                sh->wdesc = Wdesc;
                sh->iptr = IPtr;
                sh->a = AReg;
                sh->b = BReg;
                sh->c = CReg;
                sh->e = EReg;
                sh->x = XReg;
                sh->status = STATUSReg;
                sh->tsdis = TimesliceDisabled;
                sh->bm[0] = m2dLength;
                sh->bm[1] = m2dDestStride;
                sh->bm[2] = m2dSourceStride;
                sh->fp_error = FP_Error;
#ifdef EMUDEBUG
                if (IsT800 || IsTVS)
#endif
                {
                        FARegSave = FAReg;
                        FBRegSave = FBReg;
                        FCRegSave = FCReg;
                }
        }
        /* A P-process is interrupted as a whole: the high-priority process
         * runs as an L-process (ISM tables 13.2, 13.6). */
        sh->is_p = t9k_prot.is_p;
        for (i = 0; i < 4; i++)
                sh->region[i] = t9k_prot.region[i];
        sh->pstate = t9k_prot.pstate_reg;
        sh->wdescstub = t9k_prot.wdesc_stub;
        sh->sup_iptr = t9k_prot.supervisor_iptr;
        if (sh->is_p)
                t9k_leave_p ();

        if (!t9k_chanspace_active ())
        {
                /* T800 save area (WdescIntSaveLoc ...) for T800 code */
                writeword (index (MostNeg, 11), Wdesc);
                if (IdleProcess_p != Wdesc)
                {
                        writeword (index (MostNeg, 12), IPtr);
                        writeword (index (MostNeg, 13), AReg);
                        writeword (index (MostNeg, 14), BReg);
                        writeword (index (MostNeg, 15), CReg);
                        writeword (index (MostNeg, 16), STATUSReg);
                }
        }
}

/* Reload the interrupted process (possibly one installed by ldshadow). */
static void shadow_resume (void)
{
        t9k_shadow_t *sh = &t4cpu->s_sh;

        UpdateWdescReg (GetDescWPtr(sh->wdesc) | LoPriority);
        IPtr = sh->iptr;
        AReg = sh->a;
        BReg = sh->b;
        CReg = sh->c;
        EReg = sh->e;
        XReg = sh->x;
        STATUSReg = sh->status;
        TimesliceDisabled = sh->tsdis;
        ThReg = sh->th;
        m2dLength = sh->bm[0];
        m2dDestStride = sh->bm[1];
        m2dSourceStride = sh->bm[2];
        FP_Error = sh->fp_error;
#ifdef EMUDEBUG
        if (IsT800 || IsTVS)
#endif
        {
                FAReg = FARegSave;
                FBReg = FBRegSave;
                FCReg = FCRegSave;
        }
        if (sh->is_p)
                t9k_resume_p (sh->region, sh->pstate, sh->wdescstub, sh->sup_iptr);
        ClearInterrupt;
}

/* stshadow / ldshadow (ISM 13.3, figure 13.1).  Areg and Breg are shadow
 * markers 0..7, Creg the block of store.  The floating-point group uses the
 * fpstall layout. */
static void shadow_transfer (uint32_t from, uint32_t to, uint32_t ptr, int store)
{
        t9k_shadow_t *sh = &t4cpu->s_sh;
        uint32_t g, i, sb;

        if (from >= to || to > 7u)
                return;                         /* undefined effect */
        for (g = from; g < to; g++)
        {
                switch (g)
                {
                case 0:
                        for (i = 0; i < 4; i++, ptr += 4)
                                if (store) writeword (ptr, sh->region[i]);
                                else sh->region[i] = word (ptr);
                        break;
                case 1:
                        if (store)
                        {
                                writeword (ptr, sh->pstate);
                                writeword (ptr + 4, sh->wdescstub);
                        }
                        else
                        {
                                sh->pstate = word (ptr);
                                sh->wdescstub = word (ptr + 4);
                        }
                        ptr += 8;
                        break;
                case 2:
                        if (store) writeword (ptr, sh->th);
                        else sh->th = word (ptr);
                        ptr += 4;
                        break;
                case 3:
                        if (store)
                        {
                                sb = (NotProcess_p == sh->wdesc) ? 0u :
                                     t9k_status_pack (sh->status, sh->fp_error,
                                                      sh->tsdis, sh->is_p);
                                writeword (ptr, sb);
                        }
                        else
                        {
                                sb = word (ptr);
                                sh->status = t9k_status_raw (sb);
                                sh->tsdis = (sb & SB_TSDIS) != 0;
                                sh->is_p = (sb & SB_ISP) != 0;
                                sh->fp_error = (sb & SB_FPERROR) != 0;
                        }
                        ptr += 4;
                        break;
                case 4:
                {
                        uint32_t *r[11];

                        r[0] = &sh->wdesc; r[1] = &sh->iptr; r[2] = &sh->a;
                        r[3] = &sh->b; r[4] = &sh->c; r[5] = &sh->e; r[6] = &sh->x;
                        r[7] = &sh->bm[0]; r[8] = &sh->bm[1]; r[9] = &sh->bm[2];
                        r[10] = &sh->eptr;
                        for (i = 0; i < 11; i++, ptr += 4)
                                if (store) writeword (ptr, *r[i]);
                                else *r[i] = word (ptr);
                        break;
                }
                case 5:
                        if (store)
                        {
                                writeword (ptr,
                                           t9k_fp_lenc (FARegSave.length)
                                         | (t9k_fp_lenc (FBRegSave.length) << 2)
                                         | (t9k_fp_lenc (FCRegSave.length) << 4)
                                         | (sh->fp_error ? (1u << 6) : 0u));
                                t9k_fp_store_reg (ptr + 4,  &FARegSave);
                                t9k_fp_store_reg (ptr + 12, &FBRegSave);
                                t9k_fp_store_reg (ptr + 20, &FCRegSave);
                        }
                        else
                        {
                                uint32_t st = word (ptr);
                                t9k_fp_load_reg (ptr + 4,  &FARegSave, t9k_fp_ldec (st & 3u));
                                t9k_fp_load_reg (ptr + 12, &FBRegSave, t9k_fp_ldec ((st >> 2) & 3u));
                                t9k_fp_load_reg (ptr + 20, &FCRegSave, t9k_fp_ldec ((st >> 4) & 3u));
                                sh->fp_error = (st & (1u << 6)) != 0;
                        }
                        ptr += 28;
                        break;
                case 6:
                        if (store)
                        {
                                writeword (ptr, sh->wl);
                                writeword (ptr + 4, sh->wu);
                        }
                        else
                        {
                                sh->wl = word (ptr);
                                sh->wu = word (ptr + 4);
                        }
                        ptr += 8;
                        break;
                }
        }
}

/* Insert a process into the relevant priority process queue. */
void insert (uint32_t time)
{
        uint32_t ptr;
	uint32_t nextptr;
	uint32_t timetemp;

	writeword (index (WPtr, Time_s), (time + 1));

        ptr = TPtrLoc[ProcPriority];
	if (ptr == NotProcess_p)
	{
		/* Empty list. */
		/*writeword (ptr, WPtr); Strange! */
		writeword (index (WPtr, TLink_s), NotProcess_p);
                TPtrLoc[ProcPriority] = WPtr;
	}
	else
	{
		/* One or more entries. */
		timetemp = word (index (ptr, Time_s));
		if (INT32(timetemp - time) > 0)
		{
			/* Put in front of first entry. */
			writeword (index (WPtr, TLink_s), ptr);
                        TPtrLoc[ProcPriority] = WPtr;
		}
		else
		{
			/* Somewhere after the first entry. */
			/* Run along list until ptr is before the time and nextptr is after it. */
			nextptr = word (index (ptr, TLink_s));
			if (nextptr != NotProcess_p)
				timetemp = word (index (nextptr, Time_s));
			while ((INT32(time - timetemp) > 0) && (nextptr != NotProcess_p))
			{
				ptr = nextptr;
				nextptr = word (index (ptr, TLink_s));
				if (nextptr != NotProcess_p)
					timetemp = word (index (ptr, Time_s));
			}

			/* Insert into list. */
			writeword (index (ptr, TLink_s), WPtr);
			writeword (index (WPtr, TLink_s), nextptr);
		}
	}
}

/* Purge a process from the timer queue, if it is there. */
void purge_timer (void)
{
        uint32_t ptr;
	uint32_t oldptr;

        if (Idle)
                return;

	/* Delete any entries at the beginning of the list. */
	while (TPtrLoc[ProcPriority] == WPtr)
	{
	        TPtrLoc[ProcPriority] = word (index (WPtr, TLink_s));
	}

	ptr = TPtrLoc[ProcPriority];
	oldptr = ptr;

	/* List exists. */
	while (ptr != NotProcess_p)
	{
		if (ptr == WPtr)
		{
			ptr = word (index (ptr, TLink_s));
			writeword (index (oldptr, TLink_s), ptr);
		}
		else
		{
			oldptr = ptr;
			ptr = word (index (ptr, TLink_s));
		}
	}	
}


void schedule_timerq (int prio)
{
        uint32_t temp3;

        /* IntEnabled is always TRUE. */
        if ((TPtrLoc[prio] != NotProcess_p)
                /* && ((ProcPriority == HiPriority) || IntEnabled) */
           )
        {
	        temp3 = word (index (TPtrLoc[prio], Time_s));
		while ((INT32(ClockReg[prio] - temp3) > 0) && (TPtrLoc[prio] != NotProcess_p))
		{
		        schedule (TPtrLoc[prio] | prio);

			TPtrLoc[prio] = word (index (TPtrLoc[prio], TLink_s));
                        if (TPtrLoc[prio] != NotProcess_p)
			        temp3 = word (index (TPtrLoc[prio], Time_s));
		}
        }
}

/* XXX Update time, check timer queues. */
INLINE void update_time (void)
{
        struct timeval tv;
        unsigned long elapsed_usec;

	/* Move timers on if necessary, and increment timeslice counter. */
	count1 = 0;

        /* Check TOD clock, on UNIX ~ 1us resolution. */
        update_tod (&tv);

        /* Calculate elapsed usecs. */
        elapsed_usec = (tv.tv_sec  - LastTOD.tv_sec) * 1000000 +
                       (tv.tv_usec - LastTOD.tv_usec);
                
        /* Time not lapsed ? Return. */
        if (0 == elapsed_usec)
        {
                delayCount1++;
                return;
        }
        else if (elapsed_usec > 1)
        {
                delayCount1--;
                if (delayCount1 < 5)
                        delayCount1 = 5;
        }

        /* printf ("-I-EMUDBG: Elapsed time %lu.\n", elapsed_usec); */

        /* Update last known TOD clock. */
        LastTOD = tv;

	if (Timers == TimersGo)
                ClockReg[0] += elapsed_usec;

	count2 += elapsed_usec;

        /* Check high priority timer queue if enabled. */
        if (TimerEnableHi)
                schedule_timerq (HiPriority);

	if (count2 > 64) /* ~ 64us */
	{
	        if (Timers == TimersGo)
                        ClockReg[1] += (count2 / 64);
		count3 += (count2 / 64);
		count2  =  count2 & 63;

		/* Check low priority timer queue if enabled. */
                if (TimerEnableLo)
                        schedule_timerq (LoPriority);

		if (count3 > 16) /* ~ 1024us */
		{
		        timeslice += (count3 / 16);
		        count3     = count3 & 15;
		}
				
#ifdef __MWERKS__
		/* Check for events. */
		check_input();
#endif
	}
#ifdef T4_SDL_FB
    display_pump();
    display_present();
#endif
}

#define CoreAddr(a)     (INT32(a) < INT32(ExtMemStart))
#define ExtMemAddr(a)   (INT32(ExtMemStart) <= INT32(a))
#define CoreRange(a,n)  (CoreAddr(a) && CoreAddr((a)+(n)))
#define ExtMemRange(a,n)(ExtMemAddr(a) && ExtMemAddr((a)+(n)))
#define MemWordPtr(a)   ((CoreAddr(a) ? core : mem) + (MemWordMask & (a)))

static INLINE u_char code_byte(uint32_t ptr)
{
        /* Positive addresses are never RAM/core (both live in the signed-
         * negative half), so XiP fetch from the SD windows costs normal
         * execution a single never-taken branch. */
        if (unlikely((int32_t)ptr >= 0)) {
                if (ptr - BOOTBANK_BASE < BOOTBANK_SIZE)
                        return sd_read_byte(ptr - BOOTBANK_BASE);
                if (ptr >= SD_BASE && ptr < SD_LIMIT)
                        return sd_read_byte(ptr - SD_BASE);
        }
        if (likely(!CoreAddr(ptr)))
                return mem[ptr & MemByteMask];
        return core[ptr & MemByteMask];
}

/* UART helper functions */
static int uart_rx_available(void)
{
#ifndef _MSC_VER
	struct timeval tv = {0, 0};
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);
	return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0;
#else
	return 0;
#endif
}

static u_char uart_read_byte(void)
{
	u_char ch = 0;
#ifndef _MSC_VER
	ssize_t ret __attribute__((unused));
	ret = read(STDIN_FILENO, &ch, 1);
#endif
	return ch;
}

#ifdef T4_SDL_FB
	static void fb_console_flush_buffer(const char *buf)
{
	if (!fb_console_enabled || !buf || !*buf)
		return;
	for (const char *p = buf; *p; ++p)
		fb_console_putc((u_char)*p);
}
#endif

static void uart_write_byte(u_char ch)
{
	static char early_buf[128];
	static size_t early_len = 0;
#ifdef T4_SDL_FB
	static int fb_console_active = 0;
#endif

	early_printk_bytes++;
	if (ch >= 32 && ch < 127) {
		if (early_len < sizeof(early_buf) - 1) {
			early_buf[early_len++] = (char)ch;
		} else {
			memmove(early_buf, early_buf + 1, sizeof(early_buf) - 2);
			early_buf[sizeof(early_buf) - 2] = (char)ch;
		}
		early_buf[early_len < sizeof(early_buf) ? early_len : sizeof(early_buf) - 1] = '\0';
		if (!early_printk_seen) {
			if (strstr(early_buf, "Linux version") ||
			    strstr(early_buf, "early console") ||
			    strstr(early_buf, "Transputer Linux") ||
			    strstr(early_buf, "EPK:")) {
				early_printk_seen = 1;
				if (early_printk_trace_enabled)
					fprintf(stderr, "[EARLY_PRINTK] detected\n");
#ifdef T4_SDL_FB
				if (fb_console_enabled) {
					fb_console_active = 1;
					fb_console_flush_buffer(early_buf);
				}
#endif
			}
		}
	}
	/* Write directly to stdout */
	putchar(ch);
	fflush(stdout);
#ifdef T4_SDL_FB
	if (fb_console_enabled && fb_console_active)
		fb_console_putc(ch);
#endif
}

void mmio_early_printk_report(void)
{
	fprintf(stderr,
	        "[EARLY_PRINTK] %s bytes=%llu iserver_stream1=%llu msgs=%llu uart_mmio=%llu uart_seen=%s\n",
	        early_printk_seen ? "seen" : "not-seen",
	        (unsigned long long)early_printk_bytes,
	        (unsigned long long)iserver_stream1_bytes,
	        (unsigned long long)iserver_stream1_msgs,
	        (unsigned long long)uart_mmio_bytes,
	        early_printk_seen_uart ? "yes" : "no");
}

static void uart_direct_track(u_char ch)
{
	static char uart_buf[128];
	static size_t uart_len = 0;

	/* Log to UART file */
	if (uart_log_file) {
		fputc(ch, uart_log_file);
	}

	/* T4_PLAY_MOUSE: headless input injection.  When the guest prints
	 * T4_PLAY_MARKER (default "DESKTOP_READY"), push the packet list
	 * "dx,dy,btn;dx,dy,btn;..." into the mouse UART fifo as 3-byte
	 * serial packets (0x80|btn, dx, dy) — enough to move the fbwm
	 * cursor and click a desktop icon without SDL.  One-shot. */
	{
		static const char *play_pkts = (const char *)-1;
		static const char *play_marker;
		static size_t play_match;

		if (play_pkts == (const char *)-1) {
			play_pkts = getenv("T4_PLAY_MOUSE");
			play_marker = getenv("T4_PLAY_MARKER");
			if (!play_marker || !*play_marker)
				play_marker = "DESKTOP_READY";
		}
		if (play_pkts && *play_pkts) {
			if (ch == (u_char)play_marker[play_match]) {
				play_match++;
				if (!play_marker[play_match]) {
					const char *p = play_pkts;
					int npkt = 0;
					while (*p) {
						int dx = 0, dy = 0, bt = 0;
						if (sscanf(p, "%d,%d,%d",
							   &dx, &dy, &bt) == 3) {
							mouse_fifo_push((unsigned char)(0x80 | (bt & 7)));
							mouse_fifo_push((unsigned char)(dx & 0xFF));
							mouse_fifo_push((unsigned char)(dy & 0xFF));
							npkt++;
						}
						p = strchr(p, ';');
						if (!p)
							break;
						p++;
					}
					fprintf(stderr,
						"[PLAY_MOUSE] marker \"%s\" seen — injected %d packets\n",
						play_marker, npkt);
					play_pkts = NULL;
				}
			} else {
				play_match = (ch == (u_char)play_marker[0]) ? 1 : 0;
			}
		}
	}

	/* Attribute grouper cycles to phases of the run.  The guest's own
	 * progress markers are the only phase boundaries the emulator can see, so
	 * every complete line of UART output gets a cycle snapshot when
	 * T4_T9K_MARKS is set (T4_T9000=1 is what enables the model at all).
	 * The line is what the guest printed, so filtering to SPLASH_DONE /
	 * LOGIN / DESKTOP_READY is a grep on the output rather than a rebuild. */
	{
		/* per thread: threaded nodes print lines of their own */
		static T4_THREAD_LOCAL char mark_buf[160];
		static T4_THREAD_LOCAL size_t mark_len = 0;
		static int mark_on = -1;
		static int record_on = -1;

		if (mark_on < 0) {
			const char *e = getenv("T4_T9K_MARKS");
			mark_on = (e && *e && strcmp(e, "0") != 0) ? 1 : 0;
			record_on = t4_env_truthy("T4_FB_RECORD_ON_FLIP");
		}
		if (mark_on || record_on) {
			if (ch == '\n' || ch == '\r') {
				if (mark_len) {
					mark_buf[mark_len] = '\0';
					if (mark_on)
						t9k_mark(mark_buf);
#ifdef T4_SDL_FB
					/* Current kernels may carry an fbwm built before the
					 * FLIP marker ABI.  Its existing completion messages
					 * provide equivalent, tear-free boot boundaries. */
					if (record_on &&
					    (!strcmp(mark_buf, "SPLASH_DONE") ||
					     !strcmp(mark_buf, "LOGIN_DONE") ||
					     !strcmp(mark_buf, "DESKTOP_READY"))) {
						t4_fb_record_present_seen = 1;
						t4_fb_record_capture(0);
					}
#endif
					mark_len = 0;
				}
			} else if (ch >= 32 && ch < 127 &&
			           mark_len < sizeof(mark_buf) - 1) {
				mark_buf[mark_len++] = (char)ch;
			}
		}
	}

	uart_mmio_bytes++;
	if (ch >= 32 && ch < 127) {
		if (uart_len < sizeof(uart_buf) - 1) {
			uart_buf[uart_len++] = (char)ch;
		} else {
			memmove(uart_buf, uart_buf + 1, sizeof(uart_buf) - 2);
			uart_buf[sizeof(uart_buf) - 2] = (char)ch;
		}
		uart_buf[uart_len < sizeof(uart_buf) ? uart_len : sizeof(uart_buf) - 1] = '\0';
		if (!early_printk_seen_uart) {
			if (strstr(uart_buf, "Linux version") ||
			    strstr(uart_buf, "early console") ||
			    strstr(uart_buf, "Transputer Linux") ||
			    strstr(uart_buf, "EPK:")) {
				early_printk_seen_uart = 1;
				if (early_printk_trace_enabled)
					fprintf(stderr, "[EARLY_PRINTK] uart-direct detected\n");
			}
		}
	}
}

/* Read a word from memory. */
uint32_t word_int (uint32_t ptr)
{
	uint32_t result;

	/* T9000: translate when P-mode, or L-mode with RODS still loaded
	 * (soft leave_p mid C-call keeps Q3 WPtr — must relocate). */
	if (unlikely(t9k_prot.active && !t9k_prot.phys_bypass))
		ptr = t9k_translate(ptr, T9K_ACC_READ);

	/* Fast path: normal extended memory below the historical MMIO cliff
	 * (0x90000000). Guest FB carveout at 0x81E00000 sits inside that
	 * window and is handled separately. */
	if (likely((uint32_t)(ptr - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart))) {
#ifdef T4_SDL_FB
		if (unlikely(fb_addr_in_range(ptr))) {
			result = 0;
			for (int i = 0; i < 4; i++)
				result |= ((uint32_t)fb_read_byte(ptr + i)) << (8 * i);
			return result;
		}
#endif
		return *(uint32_t *)(mem + (MemWordMask & ptr));
	}
	if (likely(CORE_RAM_ADDR(ptr)))
		return *(uint32_t *)(core + (MemWordMask & ptr));

	/* Word reads from the SD windows — T9000 boot bank ("reads for
	 * configuration words are assembled from bytes", TIS 10.5.1) and
	 * the flat card mapping. */
	if (ptr - BOOTBANK_BASE < BOOTBANK_SIZE - 3 ||
	    (ptr >= SD_BASE && ptr < SD_LIMIT - 3)) {
		uint32_t off = (ptr - BOOTBANK_BASE < BOOTBANK_SIZE)
			? ptr - BOOTBANK_BASE : ptr - SD_BASE;
		return (uint32_t)sd_read_byte(off) |
		       ((uint32_t)sd_read_byte(off + 1) << 8) |
		       ((uint32_t)sd_read_byte(off + 2) << 16) |
		       ((uint32_t)sd_read_byte(off + 3) << 24);
	}

	/* Link0In is an architectural channel word whenever anything real is
	 * using it as one: with -ss the host server stores the waiting
	 * workspace descriptor there, and on any node of a configured network
	 * (nodeid >= 0) the guest does.  Only a standalone run with no server
	 * and no network may treat it as the byte-stream MMIO shim.
	 *
	 * Getting this wrong is subtle because it is READ-ONLY: writes always
	 * went to memory, so reset_channel's `writeword(Link0In, NotProcess_p)`
	 * appeared to work and then read back as 0 (whatever the shim
	 * returned).  The receive path took that 0 for the descriptor of a
	 * blocked process and scheduled process 0 - Wdesc = 0, IPtr fetched
	 * from word(0xFFFFFFFC) - so the node wild-jumped into the memory fill
	 * pattern the instant it was booted.
	 *
	 * Only worker nodes are affected (node 0 has serve set), and only when
	 * the network map actually gives a worker link 0 - which is why it hid
	 * until the 16-processor c215 map, whose node 1 receives on link 0
	 * instead of link 1. */
	if (unlikely(!serve && nodeid < 0 && ptr == LINK0_IN))
		return (uint32_t)mmio_iserver_read_byte();

	/* Check for UART register access */
	if (ptr == UART_STATUS) {
		if (uart_trace_enabled)
			fprintf(stderr, "[UART_RD_WORD: addr=0x%08X]\n", ptr);
		result = UART_STATUS_TXRDY;  /* TX always ready */
		if (uart_rx_available())
			result |= UART_STATUS_RXAVAIL;
		return result;
	}
	if (ptr == UART_DATA) {
		if (uart_trace_enabled)
			fprintf(stderr, "[UART_RD_WORD: addr=0x%08X]\n", ptr);
		result = uart_read_byte();
		return result;
	}

	/* T4Net MMIO word read */
	if (ptr >= VIRTIO_NET_BASE && ptr < VIRTIO_NET_BASE + 0x3000) {
		result = 0;
		for (int i = 0; i < 4; i++)
			result |= ((uint32_t)virtio_net_read_byte(ptr + i)) << (8 * i);
		return result;
	}

#ifdef T4_SDL_FB
	/* VGA controller register reads */
	if (ptr == VGA_CTRL_CONTROL) return vga_ctrl_control;
	if (ptr == VGA_CTRL_FB_BASE) return vga_ctrl_fb_base;
	if (ptr == VGA_CTRL_WIDTH)   return vga_ctrl_width;
	if (ptr == VGA_CTRL_HEIGHT)  return vga_ctrl_height;
	if (ptr == VGA_CTRL_STRIDE)  return vga_ctrl_stride;

	/* Keyboard UART (UART1 @ 0x11000000) */
	if (ptr == KBD_UART_STATUS) {
		uint32_t s = INPUT_UART_STATUS_TXRDY;
		if (kbd_fifo_available()) s |= INPUT_UART_STATUS_RXAVAIL;
		return s;
	}
	if (ptr == KBD_UART_DATA)
		return (uint32_t)kbd_fifo_pop();

	/* Mouse UART (UART2 @ 0x12000000) */
	if (ptr == MOUSE_UART_STATUS) {
		uint32_t s = INPUT_UART_STATUS_TXRDY;
		if (mouse_fifo_available()) s |= INPUT_UART_STATUS_RXAVAIL;
		return s;
	}
	if (ptr == MOUSE_UART_DATA)
		return (uint32_t)mouse_fifo_pop();

	if (fb_range_intersects(ptr, sizeof(uint32_t))) {
		result = 0;
		for (int i = 0; i < 4; i++) {
			result |= ((uint32_t)fb_read_byte(ptr + i)) << (8 * i);
		}
		return result;
	}
#endif
#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        uint32_t *wptr;

        wptr = (uint32_t *)MemWordPtr(ptr);
        result = *wptr;
#else
	u_char val[4];

	/* Get bytes, ensuring memory references are in range. */
        if (CoreAddr(ptr))
        {
	        val[0] = core[(ptr & MemWordMask)];
	        val[1] = core[(ptr & MemWordMask)+1];
	        val[2] = core[(ptr & MemWordMask)+2];
	        val[3] = core[(ptr & MemWordMask)+3];
        }
        else
        {
	        val[0] = mem[(ptr & MemWordMask)];
	        val[1] = mem[(ptr & MemWordMask)+1];
	        val[2] = mem[(ptr & MemWordMask)+2];
	        val[3] = mem[(ptr & MemWordMask)+3];
        }

	result = (val[0]) | (val[1]<<8) | (val[2]<<16) | (val[3]<<24);
#endif

	return (result);
}

void word_notinit (void)
{
        printf ("-E-EMU414: Error - read of not initialized memory!\n");
        handler (-1);
}

#ifndef NDEBUG
uint32_t word (uint32_t ptr)
{
        uint32_t result;

        result = word_int (ptr);

        if (memdebug)
                printf ("-I-EMUMEM: RW: Mem[%08X] ? %08X\n", ptr, result);

        if ((memnotinit) && (result == InvalidInstr_p))
                word_notinit ();

	return (result);
}
#endif


/* Write a word to memory. */
void writeword_int (uint32_t ptr, uint32_t value)
{
#ifndef NDEBUG
	static int write_count = 0;
#endif
	static int uart_col = 0;  /* Track column position for console output */
	u_char ch = 0;
	static uint32_t watch_addr = 0;

	/* T9000: translate (+ write check in P-mode). L-mode still relocates Q3. */
	if (unlikely(t9k_prot.active && !t9k_prot.phys_bypass))
		ptr = t9k_translate(ptr, T9K_ACC_WRITE);

	/* Keep the write watchpoint ahead of the normal-RAM fast return. */
	if (!watch_addr) {
		const char *wa = getenv("T4_WATCH_ADDR");
		watch_addr = wa ? (uint32_t)strtoul(wa, NULL, 16) : 0xFFFFFFFF;
		ram_watch_addr = watch_addr;
	}
	if (unlikely(watch_addr != 0xFFFFFFFF &&
	    (ptr & ~3u) == (watch_addr & ~3u))) {
		/* Ring-buffer the hits instead of printing (the address is
		 * usually a reused stack slot with millions of legitimate
		 * writes); the last entries are dumped with each LIVE-FRAME,
		 * so at a stall the final writers are visible. */
		extern uint32_t watch_ring[128][3];
		extern uint32_t watch_ring_n;
		watch_ring[watch_ring_n & 127u][0] = value;
		watch_ring[watch_ring_n & 127u][1] = IPtr;
		watch_ring[watch_ring_n & 127u][2] =
		    (uint32_t)heartbeat_counter;
		watch_ring_n++;
	}

	/* Word writes to the SD windows (boot bank alias + flat card). */
	if (ptr - BOOTBANK_BASE < BOOTBANK_SIZE - 3 ||
	    (ptr >= SD_BASE && ptr < SD_LIMIT - 3)) {
		uint32_t off = (ptr - BOOTBANK_BASE < BOOTBANK_SIZE)
			? ptr - BOOTBANK_BASE : ptr - SD_BASE;
		sd_write_byte(off, (u_char)(value & 0xFF));
		sd_write_byte(off + 1, (u_char)((value >> 8) & 0xFF));
		sd_write_byte(off + 2, (u_char)((value >> 16) & 0xFF));
		sd_write_byte(off + 3, (u_char)((value >> 24) & 0xFF));
		return;
	}

	/* Fast path: normal extended memory below the historical MMIO cliff.
	 * FB carveout (0x81E00000) and legacy alias (0x90000000) redirect
	 * into the host VGA surface so SDL present sees the pixels. */
	if (likely((uint32_t)(ptr - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart))) {
#ifdef T4_SDL_FB
		if (unlikely(fb_addr_in_range(ptr))) {
			uint32_t fb_off;
			if (fb_addr_to_offset(ptr, &fb_off) && fb_off + 4 <= FB_SIZE) {
				*(uint32_t *)(vga_framebuffer + fb_off) = value;
				vga_mark_dirty_offset(fb_off);
			} else {
				for (int i = 0; i < 4; i++)
					writebyte_int(ptr + i, (u_char)((value >> (8 * i)) & 0xFF));
			}
			return;
		}
#endif
		*(uint32_t *)(mem + (MemWordMask & ptr)) = value;
		InvalidateRange(ptr, 4);
		return;
	}
	if (likely(CORE_RAM_ADDR(ptr))) {
		*(uint32_t *)(core + (MemWordMask & ptr)) = value;
		InvalidateRange(ptr, 4);
		return;
	}

	/* Preserve the architectural Link0Out channel word in -ss mode. */
	if (unlikely(!serve && ptr == LINK0_OUT)) {
		static int mmio_word_log_count = 0;
		if (link0_trace_enabled && mmio_word_log_count < 8) {
		if (mmio_trace_enabled) {
			fprintf(stderr, "[MMIO_OUT_WORD: addr=0x%08X val=0x%08X]\n", ptr, value);
		}
			mmio_word_log_count++;
		}
		mmio_iserver_write_byte((u_char)(value & 0xFF));
		return;
	}

	/* T4Net MMIO word write */
	if (ptr >= VIRTIO_NET_BASE && ptr < VIRTIO_NET_BASE + 0x3000) {
		for (int i = 0; i < 4; i++)
			virtio_net_write_byte(ptr + i, (u_char)((value >> (8 * i)) & 0xFF));
		return;
	}

	/* Debug: Log ALL writes to see UART activity */
	/* DISABLED: Too verbose during memory init */
	/* fprintf(stderr, "[WR#%d: addr=0x%08X val=0x%08X]\n", write_count, ptr, value); */

	/* BUT: Log UART writes specifically for debugging */
	if (ptr == UART_DATA) {
		ch = (u_char)(value & 0xFF);
		if (uart_trace_enabled) {
			fprintf(stderr, "[UART_WR_WORD: addr=0x%08X val=0x%08X]\n", ptr, value);
			fprintf(stderr, "[UART_WR_WORD_IPTR: IPtr=0x%08X addr=0x%08X val=0x%08X]\n",
			        IPtr, ptr, value);
		}
		if (outbyte_trace_enabled) {
			uint32_t delta = 0;
			const link_sym *lsym = find_link_sym(IPtr, &delta);
			const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
			fprintf(stderr,
			        "[UART_WR_IPTR] IPtr=0x%08X val=0x%02X%s%s\n",
			        IPtr, value & 0xFF,
			        (lsym || sym) ? " sym=" : "",
			        lsym ? lsym->name : (sym ? sym->name : ""));
			if (lsym) {
				fprintf(stderr, "             sym_off=0x%X+0x%X\n",
				        lsym->sym_off, delta);
			} else if (sym) {
				fprintf(stderr, "             sym_off=0x%X+0x%X\n",
				        sym->text_off, delta);
			}
		}
		if (uart_console_enabled && uart_trace_enabled) {
			fprintf(stderr, "[UART: val=0x%02X='%c']\n", value & 0xFF,
			        (value >= 32 && value < 127) ? (char)(value & 0xFF) : '?');
		}
	}
#ifndef NDEBUG
	write_count++;
#endif

	/* Trace writes into kernel text region to catch self-modifying clobbers. */
	if (text_write_trace_enabled &&
	    ptr >= 0x80880000 && ptr < 0x80890000 &&
	    IPtr != 0) {
		uint32_t delta = 0;
		const link_sym *lsym = find_link_sym(IPtr, &delta);
		const prolog_sym *sym = lsym ? NULL : find_prolog_sym(IPtr, &delta);
		u_char op0 = byte_int(IPtr);
		u_char op1 = byte_int(IPtr + 1);
		u_char op2 = byte_int(IPtr + 2);
		u_char op3 = byte_int(IPtr + 3);
		fprintf(stderr,
		        "[TEXT_WR] IPtr=0x%08X op=%02X %02X %02X %02X A=0x%08X B=0x%08X C=0x%08X O=0x%08X addr=0x%08X val=0x%08X%s%s\n",
		        IPtr, op0, op1, op2, op3, AReg, BReg, CReg, OReg, ptr, value,
		        (lsym || sym) ? " sym=" : "",
		        lsym ? lsym->name : (sym ? sym->name : ""));
		if (lsym) {
			fprintf(stderr, "         sym_off=0x%X+0x%X\n",
			        lsym->sym_off, delta);
		} else if (sym) {
			fprintf(stderr, "         sym_off=0x%X+0x%X\n",
			        sym->text_off, delta);
		}
	}

	/* Check for UART register access FIRST (before general logging) */
		if (ptr == UART_DATA) {
		/* Debug: log to stderr that we're writing to UART */
		static int uart_verbose = -1;
		if (uart_verbose < 0) {
			const char *env = getenv("T4_UART_VERBOSE");
			uart_verbose = env && *env && strcmp(env, "0") != 0;
		}
		if (uart_verbose) {
			fprintf(stderr, "[UART_LOG] IPtr=0x%08X WPtr=0x%08X ch=0x%02X '%c'\n",
				IPtr, WPtr, ch, (ch >= 32 && ch < 127) ? ch : '?');
		}
		uart_direct_track(ch);
		if (!uart_console_enabled)
			return;

		// Check for banner to trigger PNG save
		static char banner_buf[100];
		static int banner_idx = 0;
		static int banner_detected = 0;

		if (!banner_detected) {
			banner_buf[banner_idx++] = ch;
			if (banner_idx >= sizeof(banner_buf) - 1) {
				banner_buf[banner_idx] = '\0'; // Null terminate
				banner_idx = 0;
			}

			// Check if we have the init banner (not just any "Transputer" string)
			if (strstr(banner_buf, "PID1:init-ready")) {
				banner_detected = 1;
				if (png_save_filename) {
					char banner_name[512];
					snprintf(banner_name, sizeof(banner_name), "%s.banner.png", png_save_filename);
					fprintf(stderr, "[BANNER] Detected init-ready banner, saving framebuffer to %s\n", banner_name);
					save_framebuffer_png(banner_name);
				}
			}
		}

		/* Emit to console - handle newlines properly */
		if (ch == '\n') {
			putchar('\n');
			fflush(stdout);
			uart_col = 0;
		} else if (ch == '\r') {
			putchar('\r');
			fflush(stdout);
			uart_col = 0;
		} else if (ch >= 32 && ch < 127) {
			/* Printable ASCII */
			putchar(ch);
			fflush(stdout);
			uart_col++;
			if (uart_col >= 80) {
				putchar('\n');
				fflush(stdout);
				uart_col = 0;
			}
		} else {
			/* Non-printable - suppress RET instruction artifacts */
			if (ch != 0xF0) {  /* Filter out OPR 0 (REV/RET) artifacts */
				fprintf(stderr, "[0x%02X]", ch);
				uart_col += 6;
			}
			/* Suppress 0xF0 which appears from RET instruction logging */
		}
		return;
	}
	if (ptr == UART_STATUS) {
		if (uart_trace_enabled)
			fprintf(stderr, "[UART_WR_WORD_STATUS: addr=0x%08X ignored]\n", ptr);
		/* Status register is read-only, ignore writes */
		fprintf(stderr, "[UART_WR_STATUS: addr=0x%08X ignored]\n", ptr);
		return;
	}

#ifdef T4_SDL_FB
	/* VGA Controller register handling at 0xA0000000 */
	if (ptr == VGA_CTRL_CONTROL) {
		uint32_t old_ctrl = vga_ctrl_control;
		vga_ctrl_control = value;
		/* Force present when display is first enabled */
		if ((value & VGA_CTRL_ENABLE) && !(old_ctrl & VGA_CTRL_ENABLE)) {
			if (vga_ctrl_width != FB_WIDTH || vga_ctrl_height != FB_HEIGHT)
				display_resize(vga_ctrl_width, vga_ctrl_height);
			vga_dirty = 1;
			vga_dirty_y0 = 0;
			vga_dirty_y1 = (int)(vga_ctrl_height ? vga_ctrl_height : FB_HEIGHT) - 1;
			display_present_force();
		}
		return;
	}
	if (ptr == VGA_CTRL_FB_BASE) {
		vga_ctrl_fb_base = value;
		return;
	}
	if (ptr == VGA_CTRL_WIDTH) {
		vga_ctrl_width = value;
		return;
	}
	if (ptr == VGA_CTRL_HEIGHT) {
		vga_ctrl_height = value;
		return;
	}
	if (ptr == VGA_CTRL_STRIDE) {
		vga_ctrl_stride = value;
		return;
	}
	if (ptr == VGA_CTRL_FLIP) {
		/* P2: present from back buffer without full-frame memcpy.
		 * value is guest VA of buffer to show (or 0 = FB_BASE front).
		 * Guest only owns FB_SIZE; host double-buffer offsets stay
		 * in vga_framebuffer[FB_TOTAL]. */
		uint32_t src = value ? value : FB_BASE;
		uint32_t src_off;
		if (fb_addr_to_offset(src, &src_off)) {
			if (src_off != 0 && src_off + FB_SIZE <= FB_TOTAL)
				vga_front_offset = src_off;
			else
				vga_front_offset = 0;
		}
		vga_dirty = 1;
		vga_dirty_y0 = 0;
		vga_dirty_y1 = (int)(vga_ctrl_height ? vga_ctrl_height : FB_HEIGHT) - 1;
		display_present_force();
		t4_fb_record_flip(value != 0);
		return;
	}
	if (ptr == VGA_CTRL_SCREENSHOT) {
		static int ss_count = 0;
		char sname[64];
		snprintf(sname, sizeof(sname), "screenshot_%d.bmp", ss_count++);
		fprintf(stderr, "[VGA] SCREENSHOT: saving %s\n", sname);
		vga_dump_framebuffer_bmp(sname, ss_count - 1);
		return;
	}
	if (ptr == VGA_CTRL_LEAVE_P) {
		/* Kernel SPAWN_EXIT: abandon P without trap (C-call exit path). */
		t9k_leave_p();
		return;
	}
	/* Phase 10: per-task RODS install (scheduler / soft rebind). */
	{
		static uint32_t rods_stage[4];
		if (ptr == VGA_CTRL_RODS0) { rods_stage[0] = value; return; }
		if (ptr == VGA_CTRL_RODS1) { rods_stage[1] = value; return; }
		if (ptr == VGA_CTRL_RODS2) { rods_stage[2] = value; return; }
		if (ptr == VGA_CTRL_RODS3) { rods_stage[3] = value; return; }
		if (ptr == VGA_CTRL_LOAD_RODS) {
			t9k_load_rods(rods_stage);
			return;
		}
	}
	/* Phase 10 PMI private + text ROM (always available, even headless). */
	{
		static uint32_t pmi_priv_lo, pmi_priv_hi, pmi_text_lo, pmi_text_hi;
		if (ptr == VGA_CTRL_PMI_PRIV_LO) { pmi_priv_lo = value; return; }
		if (ptr == VGA_CTRL_PMI_PRIV_HI) { pmi_priv_hi = value; return; }
		if (ptr == VGA_CTRL_PMI_TEXT_LO) { pmi_text_lo = value; return; }
		if (ptr == VGA_CTRL_PMI_TEXT_HI) { pmi_text_hi = value; return; }
		if (ptr == VGA_CTRL_PMI_COMMIT) {
			t9k_pmi_set_private(pmi_priv_lo, pmi_priv_hi);
			t9k_pmi_set_text_rom(pmi_text_lo, pmi_text_hi);
			return;
		}
	}

	if (fb_range_intersects(ptr, sizeof(uint32_t))) {
		uint32_t fb_off;
		if (fb_addr_to_offset(ptr, &fb_off) && fb_off + 4 <= FB_SIZE) {
			/* Single word store into guest-visible front buffer */
			*(uint32_t *)(vga_framebuffer + fb_off) = value;
			vga_mark_dirty_offset(fb_off);
		} else {
			for (int i = 0; i < 4; i++)
				writebyte_int(ptr + i, (u_char)((value >> (8 * i)) & 0xFF));
		}
		return;
	}
#endif

#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        uint32_t *wptr;

        wptr = (uint32_t *)MemWordPtr(ptr);
        *wptr = value;

#else
	u_char val[4];

	val[0] = (value & 0x000000ff);
	val[1] = ((value & 0x0000ff00)>>8);
	val[2] = ((value & 0x00ff0000)>>16);
	val[3] = ((value & 0xff000000)>>24);

	/* Write bytes, ensuring memory references are in range. */
        if (CoreAddr(ptr))
        {
	        core[(ptr & MemWordMask)]   = val[0];
	        core[(ptr & MemWordMask)+1] = val[1];
	        core[(ptr & MemWordMask)+2] = val[2];
	        core[(ptr & MemWordMask)+3] = val[3];
        }
        else
        {
	        mem[(ptr & MemWordMask)]   = val[0];
	        mem[(ptr & MemWordMask)+1] = val[1];
	        mem[(ptr & MemWordMask)+2] = val[2];
	        mem[(ptr & MemWordMask)+3] = val[3];
        }
#endif
        InvalidateRange (ptr, 4);
}


#ifndef NDEBUG
void writeword (uint32_t ptr, uint32_t value)
{
        writeword_int (ptr, value);

        if (memdebug)
                printf ("-I-EMUMEM: WW: Mem[%08X] ! %08X\n", ptr, value);
}
#endif


/* Read a byte from memory. */
u_char byte_int (uint32_t ptr)
{
	u_char result;

	if (unlikely(t9k_prot.active && !t9k_prot.phys_bypass))
		ptr = t9k_translate(ptr, T9K_ACC_READ);

	/* Fast path: RAM below historical MMIO cliff; FB carveout redirected. */
	if (likely((uint32_t)(ptr - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart))) {
#ifdef T4_SDL_FB
		if (unlikely(fb_addr_in_range(ptr)))
			return fb_read_byte(ptr);
#endif
		return mem[ptr & MemByteMask];
	}
	/* Core bytes, unless the read watchpoint at the end must see them. */
	if (likely(CORE_RAM_ADDR(ptr) && ram_watch_addr == 0xFFFFFFFF))
		return core[ptr & MemByteMask];

	/* Same rule as word_int(): see the comment there. */
	if (unlikely(!serve && nodeid < 0 && ptr == LINK0_IN))
		return mmio_iserver_read_byte();

	/* Check for SD card access */
	if (ptr - BOOTBANK_BASE < BOOTBANK_SIZE) {
		return sd_read_byte(ptr - BOOTBANK_BASE);
	}

	if (ptr >= SD_BASE && ptr < SD_LIMIT) {
		return sd_read_byte(ptr - SD_BASE);
	}

	/* Check for virtio-net access */
	if (ptr >= VIRTIO_NET_BASE && ptr < VIRTIO_NET_BASE + 0x3000) {
		return virtio_net_read_byte(ptr);
	}

	/* Check for UART register access */
	if ((ptr & ~3) == UART_BASE) {
		int offset = ptr & 3;
		if (ptr == UART_STATUS) {
			uint32_t status = UART_STATUS_TXRDY;
			if (uart_rx_available())
				status |= UART_STATUS_RXAVAIL;
			return (u_char)((status >> (offset * 8)) & 0xFF);
		}

		/* Treat all other offsets in the 4-byte window as data register */
		result = uart_read_byte();
		return result;
	}

#ifdef T4_SDL_FB
	/* Keyboard UART byte read */
	if ((ptr & ~3) == KBD_UART_BASE) {
		if (ptr == KBD_UART_STATUS) {
			uint32_t s = INPUT_UART_STATUS_TXRDY;
			if (kbd_fifo_available()) s |= INPUT_UART_STATUS_RXAVAIL;
			return (u_char)s;
		}
		return kbd_fifo_pop();
	}

	/* Mouse UART byte read */
	if ((ptr & ~3) == MOUSE_UART_BASE) {
		if (ptr == MOUSE_UART_STATUS) {
			uint32_t s = INPUT_UART_STATUS_TXRDY;
			if (mouse_fifo_available()) s |= INPUT_UART_STATUS_RXAVAIL;
			return (u_char)s;
		}
		return mouse_fifo_pop();
	}

	if (fb_addr_in_range(ptr))
		return fb_read_byte(ptr);
#endif

	/* Get byte, ensuring memory reference is in range. */
        if (CoreAddr(ptr))
	        result = core[(ptr & MemByteMask)];
        else
	        result = mem[(ptr & MemByteMask)];

	/* Memory read watchpoint (byte) */
	{
	    static uint32_t watch_addr_rb = 0;
	    if (!watch_addr_rb) {
	        const char *wa = getenv("T4_WATCH_ADDR");
	        watch_addr_rb = wa ? (uint32_t)strtoul(wa, NULL, 16) : 0xFFFFFFFF;
	    }
	    if (watch_addr_rb != 0xFFFFFFFF && ptr >= watch_addr_rb && ptr < watch_addr_rb + 4) {
	        fprintf(stderr, "[WATCH_RB] addr=0x%08X val=0x%02X IPtr=0x%08X WPtr=0x%08X core=%d idx=0x%X\n",
	                ptr, result, IPtr, WPtr, CoreAddr(ptr) ? 1 : 0, ptr & MemByteMask);
	    }
	}

	return (result);
}

#ifndef NDEBUG
u_char byte (uint32_t ptr)
{
        u_char result;

        result = byte_int (ptr);
        if (memdebug)
                printf ("-I-EMUMEM: RB: Mem[%08X] ! %02X\n", ptr, result);

        return result;
}
#endif

/* Write a byte to memory. */
INLINE void writebyte_int (uint32_t ptr, u_char value)
{
	if (unlikely(t9k_prot.active && !t9k_prot.phys_bypass))
		ptr = t9k_translate(ptr, T9K_ACC_WRITE);

	/* Fast path: RAM below historical MMIO cliff; FB carveout redirected. */
	if (likely((uint32_t)(ptr - ExtMemStart) < (uint32_t)(ram_fast_limit - ExtMemStart))) {
#ifdef T4_SDL_FB
		if (unlikely(fb_addr_in_range(ptr))) {
			fb_write_byte(ptr, value);
			return;
		}
#endif
		mem[ptr & MemByteMask] = value;
		INVALIDATE_ADDR(ptr);
		return;
	}
	/* Core bytes, unless the write watchpoint at the end must see them. */
	if (likely(CORE_RAM_ADDR(ptr) && ram_watch_addr == 0xFFFFFFFF)) {
		core[ptr & MemByteMask] = value;
		INVALIDATE_ADDR(ptr);
		return;
	}

	if (unlikely(!serve && ptr == LINK0_OUT)) {
		mmio_iserver_write_byte(value);
		return;
	}

	/* Check for SD card access */
	if (ptr - BOOTBANK_BASE < BOOTBANK_SIZE) {
		sd_write_byte(ptr - BOOTBANK_BASE, value);
		return;
	}

	if (ptr >= SD_BASE && ptr < SD_LIMIT) {
		sd_write_byte(ptr - SD_BASE, value);
		return;
	}

	/* Check for virtio-net access */
	if (ptr >= VIRTIO_NET_BASE && ptr < VIRTIO_NET_BASE + 0x3000) {
		virtio_net_write_byte(ptr, value);
		return;
	}

	/* Check for UART register access */
	if ((ptr & ~3) == UART_BASE) {
		if (uart_trace_enabled) {
			fprintf(stderr, "[UART_WR_BYTE: addr=0x%08X val=0x%02X]\n", ptr, value);
			fprintf(stderr, "[UART_WR_BYTE_IPTR: IPtr=0x%08X addr=0x%08X val=0x%02X]\n",
			        IPtr, ptr, value);
		}
		if (ptr == UART_STATUS) {
			if (uart_trace_enabled)
				fprintf(stderr, "[UART_WR_BYTE_STATUS: addr=0x%08X ignored]\n", ptr);
			return;
		}
		uart_direct_track(value);
		if (uart_console_enabled) {
			if (uart_trace_enabled) {
				fprintf(stderr, "[UART_WR_BYTE: addr=0x%08X val=0x%02X ch='%c']\n",
				        ptr, value, (value >= 32 && value < 127) ? (char)value : '.');

			}
			putchar(value);
			fflush(stdout);
		}
		return;
	}

#ifdef T4_SDL_FB
	if (fb_addr_in_range(ptr)) {
		fb_write_byte(ptr, value);
		return;
	}
#endif

	/* Memory write watchpoint (byte) */
	{
	    static uint32_t watch_addr_b = 0;
	    if (!watch_addr_b) {
	        const char *wa = getenv("T4_WATCH_ADDR");
	        watch_addr_b = wa ? (uint32_t)strtoul(wa, NULL, 16) : 0xFFFFFFFF;
	    }
	    if (watch_addr_b != 0xFFFFFFFF && ptr >= watch_addr_b && ptr < watch_addr_b + 4) {
	        fprintf(stderr, "[WATCH_WB] addr=0x%08X val=0x%02X IPtr=0x%08X WPtr=0x%08X\n",
	                ptr, value, IPtr, WPtr);
	    }
	}

	/* Write byte, ensuring memory reference is in range. */
        if (CoreAddr(ptr))
                core[(ptr & MemByteMask)] = value;
        else
	        mem[(ptr & MemByteMask)] = value;

        INVALIDATE_ADDR (ptr);
}

/* Return pointer to memory or data[] */
INLINE u_char* memrange (uint32_t ptr, u_char *data, uint32_t len)
{
        u_char *dst;
#ifdef T4_SDL_FB
        /* A range that only partly overlaps an FB window lives in two host
         * arrays, so it has no single pointer: hand back data[] for the
         * caller to write through writebytes_int().  data[] is NOT filled -
         * this is a write destination; to read a range use bytes_int(). */
        if (len && fb_range_intersects(ptr, len))
        {
                uint32_t off;
                if (fb_addr_to_offset(ptr, &off) && len <= FB_SIZE - off)
                        return vga_framebuffer + off;
                return data;
        }
#endif
        /* SD windows FIRST: ExtMemAddr() below is a SIGNED comparison, so
         * positive window addresses (0x20000000/0x7F000000) would match
         * "extended memory" and hand back RAM garbage.  Direct pointers
         * into the card buffer let block moves (move instruction, iserver
         * DMA) work from/to the card. */
        if (len && ptr - BOOTBANK_BASE < BOOTBANK_SIZE &&
            (uint64_t)(ptr - BOOTBANK_BASE) + len <= BOOTBANK_SIZE)
                return t4_sd_buffer() + (ptr - BOOTBANK_BASE);
        if (len && ptr >= SD_BASE && ptr < SD_LIMIT &&
            (uint64_t)(ptr - SD_BASE) + len <= SD_SIZE)
                return t4_sd_buffer() + (ptr - SD_BASE);
        if (CoreRange(ptr,len))
        {
                dst = core + (ptr & MemByteMask);
                return dst;
        }
        else if (ExtMemAddr(ptr))
        {
                dst = mem + (ptr & MemByteMask);
                return dst;
        }
        return data;
}

/* Write bytes to memory. */
INLINE void writebytes_int (uint32_t ptr, u_char *data, uint32_t len)
{
        u_char *dst;
#ifdef T4_SDL_FB
        if (fb_range_intersects(ptr, len))
                fb_write_bytes(ptr, data, len);
#endif
	/* Write byte, ensuring memory reference is in range. */
        if (CoreRange(ptr,len))
        {
                dst = core + (ptr & MemByteMask);
                memcpy (dst, data, len);
                InvalidateRange (ptr, len);
        }
        else if (ExtMemAddr(ptr))
        {
                /* Memory write watchpoint (bulk) */
                {
                    static uint32_t watch_addr_bk = 0;
                    if (!watch_addr_bk) {
                        const char *wa = getenv("T4_WATCH_ADDR");
                        watch_addr_bk = wa ? (uint32_t)strtoul(wa, NULL, 16) : 0xFFFFFFFF;
                    }
                    if (watch_addr_bk != 0xFFFFFFFF) {
                        uint32_t wend = watch_addr_bk + 4;
                        if (ptr <= wend && (ptr + len) > watch_addr_bk) {
                            fprintf(stderr, "[WATCH_WBULK] ptr=0x%08X len=%u IPtr=0x%08X WPtr=0x%08X\n",
                                    ptr, len, IPtr, WPtr);
                        }
                    }
                }
                dst = mem + (ptr & MemByteMask);
                memcpy (dst, data, len);
                InvalidateRange (ptr, len);
        }
        else
        {
                int i;
                for (i = 0; i < len; i++)
                        writebyte_int (ptr++, data[i]);
        }
}

/* Move bytes to memory. */
INLINE void movebytes_int (uint32_t dst, uint32_t src, uint32_t len)
{
        u_char *p, *q;
#ifdef T4_SDL_FB
        if (len && (fb_range_intersects(dst, len) || fb_range_intersects(src, len)))
        {
                /* Fast path: bulk read from source, bulk write to dest.
                 * Use a stack buffer for small moves, heap for large. */
                unsigned char tmp_stack[4096];
                unsigned char *tmp = (len <= sizeof(tmp_stack)) ? tmp_stack : malloc(len);
                if (!tmp) goto fb_slow;

                /* Read source bytes — use direct pointer when possible.
                 * bytes_int, not memrange: memrange's fallback returns tmp
                 * unfilled, so a source straddling the core/external RAM
                 * boundary copied the previous move's leftovers. */
                {
                    u_char *sp = bytes_int(src, tmp, len);
                    if (sp != tmp)
                        memcpy(tmp, sp, len);
                }

                /* Write dest: bulk FB path when dest hits either FB window */
                if (fb_range_intersects(dst, len)) {
                        size_t i;
                        for (i = 0; i < len; i++) {
                                uint32_t a = dst + (uint32_t)i;
                                if (fb_addr_in_range(a))
                                        fb_write_byte(a, tmp[i]);
                                else
                                        /* writebyte_int routes the on-chip
                                         * core region (0x80000000-0x800007FF)
                                         * to core[]; a raw mem[] store here
                                         * left code_byte reading zeros for a
                                         * kernel loaded at 0x80000070. */
                                        writebyte_int(a, tmp[i]);
                        }
                        vga_dirty = 1;
                } else if (CoreAddr(dst)) {
                        /* Destination starts in the on-chip core region —
                         * route byte-wise so the core/mem split holds. */
                        size_t i;
                        for (i = 0; i < len; i++)
                                writebyte_int(dst + (uint32_t)i, tmp[i]);
                } else {
                        /* Dest not in FB, just regular memory */
                        p = mem + (dst & MemByteMask);
                        memcpy(p, tmp, len);
                }

                if (tmp != tmp_stack)
                        free(tmp);
                InvalidateRange(dst, len);
                return;
        fb_slow:
                /* Fallback: byte-at-a-time */
                if (dst < src) {
                        for (uint32_t i = 0; i < len; i++)
                                writebyte_int(dst + i, byte_int(src + i));
                } else if (dst > src) {
                        for (uint32_t i = len; i > 0; i--)
                                writebyte_int(dst + i - 1, byte_int(src + i - 1));
                }
                return;
        }
#endif

	/* SD windows: ExtMemAddr() below is signed and would claim the
	 * positive window addresses, moving RAM garbage instead of card
	 * bytes.  Route any move touching a card window through the
	 * byte accessors (which know the windows). */
        if ((src - BOOTBANK_BASE < BOOTBANK_SIZE) ||
            (dst - BOOTBANK_BASE < BOOTBANK_SIZE) ||
            (src >= SD_BASE && src < SD_LIMIT) ||
            (dst >= SD_BASE && dst < SD_LIMIT))
        {
                uint32_t i;
                if (dst <= src)
                        for (i = 0; i < len; i++)
                                writebyte_int(dst + i, byte_int(src + i));
                else
                        for (i = len; i > 0; i--)
                                writebyte_int(dst + i - 1, byte_int(src + i - 1));
                InvalidateRange (dst, len);
                return;
        }

	/* Write byte, ensuring memory reference is in range. */
        if (CoreRange(src,len) && CoreRange(dst,len))
        {
                p = core + (dst & MemByteMask);
                q = core + (src & MemByteMask);
                memmove (p, q, len);
                InvalidateRange (dst, len);
        }
        else if (ExtMemAddr(src) && ExtMemAddr(dst))
        {
                /* Memory move watchpoint */
                {
                    static uint32_t watch_addr_mv = 0;
                    if (!watch_addr_mv) {
                        const char *wa = getenv("T4_WATCH_ADDR");
                        watch_addr_mv = wa ? (uint32_t)strtoul(wa, NULL, 16) : 0xFFFFFFFF;
                    }
                    if (watch_addr_mv != 0xFFFFFFFF) {
                        uint32_t wend = watch_addr_mv + 4;
                        if (dst <= wend && (dst + len) > watch_addr_mv) {
                            fprintf(stderr, "[WATCH_MOVE] dst=0x%08X src=0x%08X len=%u IPtr=0x%08X WPtr=0x%08X\n",
                                    dst, src, len, IPtr, WPtr);
                        }
                    }
                }
                p = mem + (dst & MemByteMask);
                q = mem + (src & MemByteMask);
                memmove (p, q, len);
                InvalidateRange (dst, len);
        }
        else if ((CoreRange(src,len) || EXT_RAM_RANGE(src,len)) &&
                 (CoreRange(dst,len) || EXT_RAM_RANGE(dst,len)) &&
                 ram_watch_addr == 0xFFFFFFFF)
        {
                /* One end in the core, the other in external RAM (the FB
                 * windows were diverted above): two host arrays, so the
                 * ranges cannot overlap. */
                p = (CoreAddr(dst) ? core : mem) + (dst & MemByteMask);
                q = (CoreAddr(src) ? core : mem) + (src & MemByteMask);
                memcpy (p, q, len);
                InvalidateRange (dst, len);
        }
        else
        {
                int i;
                for (i = 0; i < len; i++)
                        writebyte_int (dst++, byte_int (src++));
        }
}

/* Read bytes from memory. */
/* If range entirely in core/extmem then return pointer to memory */
/* Otherwise copy to data[] and return data[] */
INLINE u_char* bytes_int (uint32_t ptr, u_char *data, uint32_t len)
{
        u_char *dst;
#ifdef T4_SDL_FB
        /* Partly in an FB window: the bytes are split between
         * vga_framebuffer[] and RAM, so gather them one at a time. */
        if (len && fb_range_intersects(ptr, len))
        {
                uint32_t off, i;
                if (fb_addr_to_offset(ptr, &off) && len <= FB_SIZE - off)
                        return vga_framebuffer + off;
                for (i = 0; i < len; i++)
                        data[i] = byte_int (ptr + i);
                return data;
        }
#endif
	/* Write byte, ensuring memory reference is in range. */
        if (CoreRange(ptr,len))
        {
                dst = core + (ptr & MemByteMask);
                return dst;
        }
        else if (ExtMemAddr(ptr))
        {
                dst = mem + (ptr & MemByteMask);
                return dst;
        }
        else
        {
                int i;
                for (i = 0; i < len; i++)
                        data[i] = byte_int (ptr++);
                return data;
        }
}

#ifndef NDEBUG
void writebyte (uint32_t ptr, u_char value)
{
        if (memdebug)
                printf ("-I-EMUMEM: WB: Mem[%08X] ! %02X\n", ptr, value);

        writebyte_int (ptr, value);
}
#endif

/* Read a REAL32 from memory. */
fpreal32_t real32 (uint32_t ptr)
{
        fpreal32_t x;
#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        REAL32 *p;

        ResetRounding = TRUE;

        p = (REAL32 *)MemWordPtr(ptr);
        x.fp = *p;
#else
        ResetRounding = TRUE;

        x.bits = word (ptr);
#endif
        return x;
}

/* Write a REAL32 to memory. */
void writereal32 (uint32_t ptr, fpreal32_t value)
{
#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        REAL32 *p;

        ResetRounding = TRUE;

        p = (REAL32 *)MemWordPtr(ptr);
        *p = value.fp;
#else
        ResetRounding = TRUE;

        writeword (ptr, value.bits);
#endif
        InvalidateRange (ptr, 4);
}

/* Read a REAL64 from memory. */
fpreal64_t real64 (uint32_t ptr)
{
        fpreal64_t x;
#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        REAL64 *p;

        ResetRounding = TRUE;

        p = (REAL64 *)MemWordPtr(ptr);
        x.fp = *p;
#else
        uint32_t lobits, hibits;

        ResetRounding = TRUE;

        lobits = word (ptr);
        hibits = word (ptr + 4);

        x.bits = ((uint64_t)(hibits) << 32) | lobits;
#endif
        return x;
}

/* Write a REAL64 to memory. */
void writereal64 (uint32_t ptr, fpreal64_t value)
{
#if BYTE_ORDER==1234
#ifndef _MSC_VER
#warning Using little-endian access!
#endif
        REAL64 *p;

        ResetRounding = TRUE;

        p = (REAL64 *)MemWordPtr(ptr);
        *p = value.fp;
#else
        ResetRounding = TRUE;

        writeword (ptr,     value.bits & 0xffffffff);
        writeword (ptr + 4, (value.bits >> 32) & 0xffffffff);
#endif
        InvalidateRange (ptr, 8);
}

/* SD Card Emulation — file-backed persistent storage
 *
 * If $T4_SD_IMAGE is set, the SD card is loaded from and saved to that file.
 * This gives TransputerOS a persistent "hard drive".
 * If the file doesn't exist, it's created as a 128MB zero-filled image.
 * The image is saved back on emulator exit (sd_save).
 *
 * Without $T4_SD_IMAGE, SD card is volatile (calloc, lost on exit).
 */
static uint8_t *sd_card = NULL;
static const char *sd_image_path = NULL;
static int sd_dirty = 0;

static void sd_init(void) {
    FILE *f;
    if (sd_card) return;

    sd_card = calloc(SD_SIZE, 1);
    if (!sd_card) {
        fprintf(stderr, "Failed to allocate SD card memory\n");
        exit(1);
    }

    sd_image_path = getenv("T4_SD_IMAGE");
    if (sd_image_path && sd_image_path[0]) {
        f = fopen(sd_image_path, "rb");
        if (f) {
            size_t n = fread(sd_card, 1, SD_SIZE, f);
            fclose(f);
            fprintf(stderr, "[SD_INIT] Loaded %zuMB from %s\n", n / (1024*1024), sd_image_path);
        } else {
            fprintf(stderr, "[SD_INIT] Image %s not found — creating fresh 128MB\n", sd_image_path);
        }
    } else {
        fprintf(stderr, "[SD_INIT] Volatile 128MB SD card (set T4_SD_IMAGE for persistence)\n");
    }
}

void sd_save(void) {
    FILE *f;
    if (!sd_card || !sd_image_path || !sd_image_path[0]) return;
    if (!sd_dirty) {
        fprintf(stderr, "[SD_SAVE] No writes — skipping save\n");
        return;
    }
    f = fopen(sd_image_path, "wb");
    if (f) {
        fwrite(sd_card, 1, SD_SIZE, f);
        fclose(f);
        fprintf(stderr, "[SD_SAVE] Saved 128MB to %s\n", sd_image_path);
    } else {
        fprintf(stderr, "[SD_SAVE] Failed to write %s\n", sd_image_path);
    }
}

u_char *t4_sd_buffer(void) {
    if (!sd_card) sd_init();
    return sd_card;
}

int t4_sd_boot_enabled(void) {
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("T4_SD_BOOT");
        on = (e && *e && atoi(e) != 0) ? 1 : 0;
    }
    return on;
}

u_char sd_read_byte(uint32_t offset) {
    if (!sd_card) sd_init();
    if (offset >= SD_SIZE) return 0;
    return sd_card[offset];
}

void sd_write_byte(uint32_t offset, u_char value) {
    if (!sd_card) sd_init();
    if (offset >= SD_SIZE) return;
    sd_card[offset] = value;
    sd_dirty = 1;
}

/*
 * T4Net — Simple MMIO Ethernet NIC with TAP backend
 *
 * Register map (base 0x30000000):
 *   0x00  STATUS    (R)   bit0=link_up, bit1=rx_ready, bit2=tx_ready
 *   0x04  TX_LEN    (W)   write packet length → triggers TX send
 *   0x08  RX_LEN    (R)   length of pending RX packet (0 = none)
 *   0x0C  MAC_LO    (R)   MAC bytes 0-3 (little-endian)
 *   0x10  MAC_HI    (R)   MAC bytes 4-5 (little-endian, upper 16 bits = 0)
 *   0x14  RX_ACK    (W)   write any value → acknowledge RX packet consumed
 *   0x1000-0x15FF  TX_BUF  (W)   1536-byte TX packet buffer
 *   0x2000-0x25FF  RX_BUF  (R)   1536-byte RX packet buffer
 */

#include <sys/ioctl.h>
#include <poll.h>
#include <net/if.h>
#include <linux/if_tun.h>

#define T4NET_BASE       VIRTIO_NET_BASE   /* 0x30000000 */
#define T4NET_REG_STATUS 0x00
#define T4NET_REG_TX_LEN 0x04
#define T4NET_REG_RX_LEN 0x08
#define T4NET_REG_MAC_LO 0x0C
#define T4NET_REG_MAC_HI 0x10
#define T4NET_REG_RX_ACK 0x14
#define T4NET_TX_BUF     0x1000
#define T4NET_RX_BUF     0x2000
#define T4NET_BUF_SIZE   1536    /* MTU 1500 + ethernet header 14 + margin */
#define T4NET_MMIO_SIZE  0x3000  /* Total MMIO window */

static int    t4net_tap_fd = -1;
static uint8_t t4net_mac[6] = {0x52, 0x54, 0x00, 0x12, 0x34, 0x56};
static uint8_t t4net_tx_buf[T4NET_BUF_SIZE];
static uint8_t t4net_rx_buf[T4NET_BUF_SIZE];
static int     t4net_rx_len = 0;   /* length of pending RX packet, 0=none */

/* Open TAP device. Returns fd or -1. */
int t4net_tap_open(const char *dev_name)
{
    struct ifreq ifr;
    int fd;

    fd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[T4NET] Cannot open /dev/net/tun: %s\n", strerror(errno));
        return -1;
    }

    memset(&ifr, 0, sizeof(ifr));
    ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
    if (dev_name && dev_name[0])
        strncpy(ifr.ifr_name, dev_name, IFNAMSIZ - 1);

    if (ioctl(fd, TUNSETIFF, &ifr) < 0) {
        fprintf(stderr, "[T4NET] TUNSETIFF failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    fprintf(stderr, "[T4NET] TAP device '%s' opened (fd=%d)\n", ifr.ifr_name, fd);
    return fd;
}

void t4net_init(const char *tap_name)
{
    t4net_tap_fd = t4net_tap_open(tap_name);
    if (t4net_tap_fd >= 0)
        fprintf(stderr, "[T4NET] NIC online, MAC=%02x:%02x:%02x:%02x:%02x:%02x\n",
                t4net_mac[0], t4net_mac[1], t4net_mac[2],
                t4net_mac[3], t4net_mac[4], t4net_mac[5]);
    else
        fprintf(stderr, "[T4NET] Running without TAP (no host networking)\n");
}

void t4net_cleanup(void)
{
    if (t4net_tap_fd >= 0) {
        close(t4net_tap_fd);
        t4net_tap_fd = -1;
    }
}

/* Poll TAP for an incoming packet. Called periodically from emulator loop. */
void t4net_poll_rx(void)
{
    if (t4net_tap_fd < 0 || t4net_rx_len > 0)
        return;  /* no TAP, or already have a pending packet */

    struct pollfd pfd = { .fd = t4net_tap_fd, .events = POLLIN };
    if (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        int n = read(t4net_tap_fd, t4net_rx_buf, T4NET_BUF_SIZE);
        if (n > 0) {
            t4net_rx_len = n;
        }
    }
}

/* Send TX buffer to TAP device. */
static void t4net_do_tx(int len)
{
    if (len <= 0 || len > T4NET_BUF_SIZE)
        return;

    if (t4net_tap_fd >= 0) {
        int n = write(t4net_tap_fd, t4net_tx_buf, len);
        if (n < 0)
            fprintf(stderr, "[T4NET] TX write error: %s\n", strerror(errno));
    }
}

/* MMIO read (byte level, called from byte_int) */
u_char virtio_net_read_byte(uint32_t addr)
{
    uint32_t offset = addr - T4NET_BASE;
    uint32_t reg = offset & ~3U;     /* register-aligned offset */
    int      byte_pos = offset & 3;  /* byte within 32-bit word */
    uint32_t val = 0;

    switch (reg) {
    case T4NET_REG_STATUS:
        val = 0x04;  /* tx_ready always set */
        if (t4net_tap_fd >= 0)
            val |= 0x01;  /* link_up */
        if (t4net_rx_len > 0)
            val |= 0x02;  /* rx_ready */
        break;
    case T4NET_REG_RX_LEN:
        val = (uint32_t)t4net_rx_len;
        break;
    case T4NET_REG_MAC_LO:
        val = t4net_mac[0] | (t4net_mac[1] << 8) |
              (t4net_mac[2] << 16) | (t4net_mac[3] << 24);
        break;
    case T4NET_REG_MAC_HI:
        val = t4net_mac[4] | (t4net_mac[5] << 8);
        break;
    default:
        /* RX buffer read */
        if (offset >= T4NET_RX_BUF && offset < T4NET_RX_BUF + T4NET_BUF_SIZE) {
            int idx = offset - T4NET_RX_BUF;
            return (idx < t4net_rx_len) ? t4net_rx_buf[idx] : 0;
        }
        break;
    }

    return (uint8_t)(val >> (byte_pos * 8));
}

/* MMIO write (byte level, called from writebyte_int) */
void virtio_net_write_byte(uint32_t addr, u_char value)
{
    uint32_t offset = addr - T4NET_BASE;

    /* TX buffer write (byte-by-byte accumulation) */
    if (offset >= T4NET_TX_BUF && offset < T4NET_TX_BUF + T4NET_BUF_SIZE) {
        int idx = offset - T4NET_TX_BUF;
        t4net_tx_buf[idx] = value;
        return;
    }

    /* TX_LEN register: accumulate bytes, trigger on final byte (byte 3) */
    if ((offset & ~3U) == T4NET_REG_TX_LEN) {
        int byte_pos = offset & 3;
        /* Accumulate into static so 32-bit STNL (4 byte writes) works */
        static uint32_t tx_len_accum = 0;
        tx_len_accum &= ~(0xFF << (byte_pos * 8));
        tx_len_accum |= (uint32_t)value << (byte_pos * 8);
        if (byte_pos == 3) {
            t4net_do_tx((int)tx_len_accum);
            tx_len_accum = 0;
        }
        return;
    }

    /* RX_ACK register: consume pending RX packet */
    if ((offset & ~3U) == T4NET_REG_RX_ACK) {
        t4net_rx_len = 0;
        return;
    }
}

#ifdef T4_SDL_FB
#include <png.h>

void save_framebuffer_png(const char *filename) {
    fprintf(stderr, "[PNG] save_framebuffer_png called with filename: %s\n", filename);
    FILE *fp = fopen(filename, "wb");
    if (!fp) {
        fprintf(stderr, "[PNG] Failed to open %s for writing\n", filename);
        return;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) {
        fclose(fp);
        return;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, NULL);
        fclose(fp);
        return;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return;
    }

    png_init_io(png, fp);

    // Set image attributes
    png_set_IHDR(png, info, FB_WIDTH, FB_HEIGHT, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png, info);

    // Write image data
    png_bytep row = (png_bytep)malloc(3 * FB_WIDTH * sizeof(png_byte));
    if (row) {
        for (int y = 0; y < FB_HEIGHT; y++) {
            for (int x = 0; x < FB_WIDTH; x++) {
                uint32_t pixel = word_int(FB_BASE + (y * FB_WIDTH + x) * 4);
                // Debug pixels at key positions
                if ((y < 10 && x < 3) || (y == 16 && x < 3) || (y == 32 && x < 3)) {
                    fprintf(stderr, "[PNG_PIXEL] x=%d y=%d addr=0x%08X pixel=0x%08X\n",
                            x, y, FB_BASE + (y * FB_WIDTH + x) * 4, pixel);
                }
                // Convert ARGB to RGB
                row[x*3 + 0] = (pixel >> 16) & 0xFF; // R
                row[x*3 + 1] = (pixel >> 8) & 0xFF;  // G
                row[x*3 + 2] = pixel & 0xFF;         // B
            }
            png_write_row(png, row);
        }
        free(row);
    }

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(fp);

    fprintf(stderr, "[PNG] Framebuffer saved to %s (%dx%d)\n", filename, FB_WIDTH, FB_HEIGHT);
}
#endif

/* Add an executing instruction to the profile list. */
static uint32_t previnstruction = NO_ICODE;

void add_profile (uint32_t instruction)
{
        if (instruction > 0x3ff)
        {
                printf ("-E-EMU414: Error - profile invalid instruction! (%u)", instruction);
                return;
        }
        instrprof[instruction]++;
        if (NO_ICODE != previnstruction)
                combinedprof[previnstruction][instruction]++;
        previnstruction = instruction;
}

char* profmnemonic(uint32_t i, char *buf)
{
        if (i < 0x100)
                sprintf (buf, "  %02X %-13s", i, mnemonic (i, MostNeg, MostNeg, 1));
	else if (i < 0x300)
                sprintf (buf, "%04X %-13s", i - 0x100, mnemonic (0xF0, i - 0x100, MostNeg, 1));
        else
                sprintf (buf, "S%03X %-13s", i - 0x300, mnemonic (0xF0, 0xAB, i - 0x300, 1));
        return buf;
}

void print_profile (void)
{
        int i, j;
	extern FILE *ProfileFile;
        char buf1[64], buf2[64];

        fprintf (ProfileFile, "-----Instruction--------Freq------\n");
	for (i = 0; i < 0x400; i++)
	{
                /* Skip empty counters. */
                if (0 == instrprof[i])
                        continue;

                fprintf (ProfileFile, "%s     %u\n", profmnemonic (i, buf1), instrprof[i]);
        }
        fprintf (ProfileFile, "-----Combined-----------------------------Freq------\n");
	for (i = 0; i < 0x400; i++)
	{
                for (j = 0; j < 0x400; j++)
                {
                        /* Skip empty counters. */
                        if (0 == combinedprof[i][j])
                                continue;

                        fprintf (ProfileFile, "%s%s     %u\n", profmnemonic (i, buf1), profmnemonic (j, buf2), combinedprof[i][j]);
                }
        }
}
