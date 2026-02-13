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
#ifndef _MSC_VER
#include <sys/time.h>
#include <unistd.h>
#endif
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
#include "arithmetic.h"
#include "server.h"
#include "opcodes.h"
#ifdef __MWERKS__
#include "mac_input.h"
#endif
#undef TRUE
#undef FALSE
#define TRUE  0x0001
#define FALSE 0x0000


/* Processor specific parameters. */
int Txxx = 414;
uint32_t MemStart    = 0x80000048;

/* Memory space. */
u_char *core;
uint32_t CoreSize    = 2 * 1024;
uint32_t ExtMemStart = 0x80000800;

u_char   *mem;
uint32_t MemSize     = 1 << 21;
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
	const char *iptr_env;
	const char *len_env;
	uint32_t entry_iptr;
	uint32_t len = 32;
	uint32_t i;
	if (iptr_dump_logged)
		return;
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
	const char *sym_name;
	const char *count_env;
	const prolog_sym *sym;
	const link_sym *lsym;
	const char *auto_env;
	if (sym_trace_entry || sym_trace_active)
		return;
	sym_name = getenv("T4_TRACE_SYM");
	if (!sym_name || !*sym_name)
		return;
	auto_env = getenv("T4_TEXT_BASE_AUTO");
	if (auto_env && *auto_env && !prolog_text_base_autodone)
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
	fprintf(stderr, "[SYM_TRACE] armed func=%s entry=0x%08X count=%d\n",
	        sym_trace_name, sym_trace_entry, sym_trace_remaining);
}

static void setup_iptr_trace_once(void) {
	const char *start_env;
	const char *len_env;
	uint32_t len = 0;
	if (iptr_trace_enabled)
		return;
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
	const char *auto_env;
	const prolog_sym *sym;
	const link_sym *lsym;
	if (prolog_text_base_autodone)
		return;
	auto_env = getenv("T4_TEXT_BASE_AUTO");
	if (!auto_env || !*auto_env)
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

/* SDL2 VGA Framebuffer (8-bit indexed color) */
#ifdef T4_X11_FB
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/keysym.h>

#define FB_BASE         ((uint32_t)0x90000000)  /* Framebuffer base address */
#define FB_WIDTH        640                      /* Width in pixels */
#define FB_HEIGHT       480                      /* Height in pixels */
#define FB_BPP          4                        /* Bytes per pixel (32bpp) */
#define FB_SIZE         (FB_WIDTH * FB_HEIGHT * FB_BPP)  /* Total size in bytes */
#define FB_LIMIT        (FB_BASE + FB_SIZE)     /* End of framebuffer */
#define FB_CONSOLE_COLS (FB_WIDTH / 8)
#define FB_CONSOLE_ROWS (FB_HEIGHT / 8)

/* VGA Controller Registers at 0xA0000000 */
#define VGA_CTRL_BASE       ((uint32_t)0xA0000000)
#define VGA_CTRL_CONTROL    (VGA_CTRL_BASE + 0x00)  /* Control register */
#define VGA_CTRL_FB_BASE    (VGA_CTRL_BASE + 0x04)  /* Framebuffer base */
#define VGA_CTRL_WIDTH      (VGA_CTRL_BASE + 0x08)  /* Width */
#define VGA_CTRL_HEIGHT     (VGA_CTRL_BASE + 0x0C)  /* Height */
#define VGA_CTRL_STRIDE     (VGA_CTRL_BASE + 0x10)  /* Stride */
#define VGA_CTRL_ENABLE     0x01                    /* Enable bit */

/* VGA controller state */
static uint32_t vga_ctrl_control = 0;
static uint32_t vga_ctrl_fb_base = FB_BASE;
static uint32_t vga_ctrl_width = FB_WIDTH;
static uint32_t vga_ctrl_height = FB_HEIGHT;
static uint32_t vga_ctrl_stride = FB_WIDTH * FB_BPP;

/* SDL2 globals */
 Display *vga_display = NULL;
 Window vga_window = 0;
 GC vga_gc = 0;
 XImage *vga_image = NULL;
uint32_t vga_palette[256];                      /* 8-bit indexed palette (RGB332) */
unsigned char vga_framebuffer[FB_SIZE];         /* Framebuffer memory (32bpp) */
int vga_dirty = 0;                              /* Needs redraw flag */
struct timeval vga_last_update;                 /* Last display update time */
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
void vga_cleanup(void);

static inline int fb_addr_in_range(uint32_t addr)
{
    return (addr >= FB_BASE) && (addr < FB_LIMIT);
}

static inline int fb_range_intersects(uint32_t addr, uint32_t len)
{
    uint64_t start = addr;
    uint64_t end = start + (uint64_t)len;

    if (len == 0)
        return 0;
    if (end <= FB_BASE)
        return 0;
    if (start >= FB_LIMIT)
        return 0;
    return 1;
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

static inline void fb_write_bytes(uint32_t addr, const unsigned char *src, size_t len)
{
    if (!len)
        return;
    uint64_t start = addr;
    uint64_t end = start + len;
    uint64_t fb_start = start < FB_BASE ? FB_BASE : start;
    uint64_t fb_end = end > FB_LIMIT ? FB_LIMIT : end;

    if (fb_start >= fb_end)
        return;

    size_t fb_offset = fb_start - FB_BASE;
    size_t src_offset = fb_start - start;
    size_t copy_len = fb_end - fb_start;

    memcpy(&vga_framebuffer[fb_offset], src + src_offset, copy_len);
    vga_dirty = 1;
}

static inline void fb_write_byte(uint32_t addr, unsigned char value)
{
    static int fb_write_count = 0;
    if (!fb_addr_in_range(addr))
        return;
    vga_framebuffer[addr - FB_BASE] = value;
    vga_dirty = 1;
    /* Log first 50 framebuffer writes for debugging */
    if (fb_write_count < 50) {
        fprintf(stderr, "[FB_WR#%d] addr=0x%08X offset=0x%X val=0x%02X\n",
                fb_write_count, addr, addr - FB_BASE, value);
        fb_write_count++;
    }
}

static inline unsigned char fb_read_byte(uint32_t addr)
{
    if (!fb_addr_in_range(addr))
        return 0;
    return vga_framebuffer[addr - FB_BASE];
}

static void vga_pump_events(void)
{
    XEvent event;

    while (XPending(vga_display)) {
        XNextEvent(vga_display, &event);
        switch (event.type) {
        case ClientMessage:
            quit = TRUE;
            break;
        case KeyPress:
            if (XLookupKeysym(&event.xkey, 0) == XK_Escape)
                quit = TRUE;
            break;
        case ButtonPress:
            quit = TRUE;
            break;
        case Expose:
            /* Window needs repaint - mark framebuffer dirty */
            vga_dirty = 1;
            break;
        default:
            break;
        }
    }
}

/* Initialize VGA display */
void vga_init(void)
{
    fprintf(stderr, "[VGA] Initializing X11 display...\n");
    vga_display = XOpenDisplay(NULL);
    if (!vga_display) {
        fprintf(stderr, "[VGA] XOpenDisplay failed\n");
        return;
    }
    fprintf(stderr, "[VGA] XOpenDisplay succeeded\n");

    int screen = DefaultScreen(vga_display);
    Window root = RootWindow(vga_display, screen);

    vga_window = XCreateSimpleWindow(vga_display, root, 0, 0, FB_WIDTH, FB_HEIGHT, 1,
                                     BlackPixel(vga_display, screen),
                                     WhitePixel(vga_display, screen));

    XStoreName(vga_display, vga_window, "Transputer T4 - VGA Display");

    XSelectInput(vga_display, vga_window, ExposureMask | KeyPressMask | ButtonPressMask);

    XMapWindow(vga_display, vga_window);
    XSync(vga_display, False);  /* Wait for window to be mapped */

    vga_gc = XCreateGC(vga_display, vga_window, 0, NULL);

    /* Create XImage for framebuffer */
    int depth = DefaultDepth(vga_display, screen);
    fprintf(stderr, "[VGA] X11 depth: %d bits\n", depth);

    vga_image = XCreateImage(vga_display, DefaultVisual(vga_display, screen),
                             depth, ZPixmap, 0,
                             (char *)malloc(FB_WIDTH * FB_HEIGHT * 4), FB_WIDTH, FB_HEIGHT,
                             32, FB_WIDTH * 4);

    if (!vga_image) {
        fprintf(stderr, "XCreateImage failed\n");
        XCloseDisplay(vga_display);
        return;
    }
    fprintf(stderr, "[VGA] XImage: width=%d height=%d bpp=%d depth=%d bytes_per_line=%d\n",
            vga_image->width, vga_image->height, vga_image->bits_per_pixel,
            vga_image->depth, vga_image->bytes_per_line);
    fprintf(stderr, "[VGA] XImage: byte_order=%d (LSB=%d MSB=%d) bitmap_bit_order=%d\n",
            vga_image->byte_order, LSBFirst, MSBFirst, vga_image->bitmap_bit_order);
    fprintf(stderr, "[VGA] X11 server byte_order=%d\n", ImageByteOrder(vga_display));

    /* Initialize RGB332 palette (8-bit indexed color) */
    for (int i = 0; i < 256; i++) {
        int r = ((i >> 5) & 0x07) * 255 / 7;  /* 3 bits red */
        int g = ((i >> 2) & 0x07) * 255 / 7;  /* 3 bits green */
        int b = ((i >> 0) & 0x03) * 255 / 3;  /* 2 bits blue */
        vga_palette[i] = (0xFF << 24) | (r << 16) | (g << 8) | b;
    }

    /* Clear framebuffer to black */
    memset(vga_framebuffer, 0, FB_SIZE);
    vga_dirty = 1;

    gettimeofday(&vga_last_update, NULL);

    atexit(vga_cleanup);

    fprintf(stderr, "[VGA] Initialized %dx%d framebuffer at 0x%08X-0x%08X\n",
            FB_WIDTH, FB_HEIGHT, FB_BASE, FB_LIMIT - 1);

    /* TEST: Show blue screen for 1 second before booting */
    {
        uint32_t *fb32 = (uint32_t *)vga_framebuffer;
        fprintf(stderr, "[VGA] Displaying blue test screen for 1 second...\n");

        /* Fill with blue (0xFF0000FF in ARGB with swapped RB = blue for X11) */
        for (int i = 0; i < FB_WIDTH * FB_HEIGHT; i++) {
            fb32[i] = 0xFF0000FF;  /* Blue (X11 expects BGRA) */
        }

        /* Draw "TEST" text in white at top-left using simple block letters */
        for (int y = 10; y < 30; y++) {
            for (int x = 10; x < 100; x++) {
                fb32[y * FB_WIDTH + x] = 0xFFFFFFFF;  /* White strip */
            }
        }

        /* Direct XPutImage for test screen (bypass vga_update control check) */
        memcpy(vga_image->data, vga_framebuffer, FB_WIDTH * FB_HEIGHT * 4);
        XPutImage(vga_display, vga_window, vga_gc, vga_image, 0, 0, 0, 0, FB_WIDTH, FB_HEIGHT);
        XFlush(vga_display);
        fprintf(stderr, "[VGA] Test screen displayed via direct XPutImage\n");

        /* Wait 100ms (reduced from 1 second to avoid boot delays) */
        usleep(100000);

        /* Clear back to black */
        memset(vga_framebuffer, 0, FB_SIZE);
        vga_dirty = 1;
        fprintf(stderr, "[VGA] Blue test complete, continuing boot...\n");
    }

    /* Save initial framebuffer state to verify pattern */
    FILE *fb_dump = fopen("framebuffer_init.raw", "wb");
    if (fb_dump) {
        fwrite(vga_framebuffer, 1, FB_SIZE, fb_dump);
        fclose(fb_dump);
        fprintf(stderr, "[VGA] Initial framebuffer saved to framebuffer_init.raw\n");
    }
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

    int width = FB_WIDTH;
    int height = FB_HEIGHT;
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

    // Convert 32bpp framebuffer to 24-bit RGB (BGR order for BMP)
    uint32_t *fb32 = (uint32_t *)vga_framebuffer;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint32_t pixel = fb32[y * width + x];

            // Extract BGRA components (pixel is in 0xAABBGGRR format for X11)
            unsigned char b = (pixel >> 16) & 0xFF;
            unsigned char g = (pixel >> 8) & 0xFF;
            unsigned char r = pixel & 0xFF;

            // Write BGR (BMP format)
            fputc(b, fb_dump);
            fputc(g, fb_dump);
            fputc(r, fb_dump);
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

/* Update VGA display from framebuffer */
void vga_update(void)
{
    static int update_count = 0;
    if (!vga_image || !vga_dirty)
        return;

    /* Check if display is enabled by kernel via VGA control register */
    if (!(vga_ctrl_control & VGA_CTRL_ENABLE)) {
        static int skip_logged = 0;
        if (skip_logged < 3) {
            fprintf(stderr, "[VGA_UPDATE] Skipping - display not enabled (ctrl=0x%08X)\n",
                    vga_ctrl_control);
            skip_logged++;
        }
        return;  /* Display not enabled yet */
    }

    /* Log first few updates for debugging */
    if (update_count < 5) {
        uint32_t *fb32_dbg = (uint32_t *)vga_framebuffer;
        fprintf(stderr, "[VGA_UPDATE#%d] dirty=1, first pixels: 0x%08X 0x%08X 0x%08X 0x%08X\n",
                update_count, fb32_dbg[0], fb32_dbg[1], fb32_dbg[640], fb32_dbg[1280]);
        update_count++;
    }

    /* 32bpp framebuffer: copy directly to XImage data */
    /* This is much faster than XPutPixel per-pixel */
    memcpy(vga_image->data, vga_framebuffer, FB_WIDTH * FB_HEIGHT * 4);

    /* Put image to window */
    XPutImage(vga_display, vga_window, vga_gc, vga_image, 0, 0, 0, 0, FB_WIDTH, FB_HEIGHT);
    XFlush(vga_display);  /* Ensure X11 buffer is flushed to display */

    vga_dirty = 0;
    gettimeofday(&vga_last_update, NULL);
}

/* Check if display needs periodic update (for smooth animation) */
void vga_check_update(void)
{
    static int frame_count = 0;
    struct timeval now;
    long elapsed_us;

    gettimeofday(&now, NULL);
    elapsed_us = (now.tv_sec - vga_last_update.tv_sec) * 1000000L +
                 (now.tv_usec - vga_last_update.tv_usec);

    /* Update at ~60 Hz (16666 microseconds per frame) */
    if (elapsed_us >= 16666) {
        vga_update();
        frame_count++;

        /* Save framebuffer at specific frames */
        if (frame_count == 1) {
            vga_dump_framebuffer_bmp("frame_1.bmp", frame_count);
        }
        if (frame_count == 10) {
            vga_dump_framebuffer_bmp("frame_10.bmp", frame_count);
        }
        if (frame_count == 100) {
            vga_dump_framebuffer_bmp("frame_100.bmp", frame_count);
        }
    }
}

void vga_process_events(void)
{
    if (!vga_window)
        return;

    vga_pump_events();
    vga_check_update();
}

/* Cleanup VGA display */
void vga_cleanup(void)
{
    if (vga_image) {
        XDestroyImage(vga_image);
        vga_image = NULL;
    }
    if (vga_gc) {
        XFreeGC(vga_display, vga_gc);
        vga_gc = 0;
    }
    if (vga_window) {
        XDestroyWindow(vga_display, vga_window);
        vga_window = 0;
    }
    if (vga_display) {
        XCloseDisplay(vga_display);
        vga_display = NULL;
    }
}

#endif /* T4_SDL2_FB */

uint32_t word_int (uint32_t);

/* Registers. */
uint32_t IPtr;
uint32_t WPtr;
uint32_t AReg;
uint32_t BReg;
uint32_t CReg;
uint32_t OReg;

uint32_t XReg;

uint32_t WdescReg;

uint32_t DReg;                  /* undocumented DReg/EReg */
uint32_t EReg;

#define FP_UNKNOWN       0
#define FP_REAL32       32
#define FP_REAL64       64

typedef struct _REAL {
        uint32_t length;        /* FP_REAL32 or FP_REAL64 */
        uint32_t rsvd;
        union {
                fpreal32_t  sn;
                fpreal64_t  db;
        } u;
} REAL;

#define SN(reg)   (reg.u.sn)
#define DB(reg)   (reg.u.db)

REAL  FAReg;
REAL  FBReg;
REAL  FCReg;
REAL  FARegSave;
REAL  FBRegSave;
REAL  FCRegSave;
int   FP_Error;                 /* not preserved over descheduling */
int   RoundingMode;             /* current rounding mode */
int   ResetRounding;            /* reset rounding mode ? */

uint32_t m2dSourceStride;       /* move2d source stride */
uint32_t m2dDestStride;         /* move2d destination stride */
uint32_t m2dLength;             /* move2d length (no. of rows) */

/* Other registers. */
uint32_t ClockReg[2];
uint32_t TNextReg[2];
uint32_t TPtrLoc[2];            /* XXX 0x80000024 0x80000028 */

uint32_t FPtrReg[2];
uint32_t BPtrReg[2];

#define ProcessQEmpty           ((NotProcess_p == FPtrReg[0]) && (NotProcess_p == FPtrReg[1]))
#define TimerQEmpty             ((NotProcess_p == TPtrLoc[0]) && (NotProcess_p == TPtrLoc[1]))

uint32_t STATUSReg;             /* Processor flags: GotoSNPBit, HaltOnError, Error */

#define ClearInterrupt          writeword (0x8000002C, IdleProcess_p)
#define ReadInterrupt           (word (0x8000002C) != IdleProcess_p)

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
#define writeword(a,x)  writeword_int(a,x)
#define writebyte(a,x)  writebyte_int(a,x)
#define byte(a)         byte_int(a)
#define word(a)         word_int(a)
#endif

/* Internal variables. */
u_char Instruction;
u_char Icode;
u_char Idata;
int  Timers;
int  TimerEnableHi = 1;
int  TimerEnableLo = 1;
uint32_t t4_overflow;
uint32_t t4_carry;
uint32_t t4_normlen;
uint32_t t4_carry64;            /* shl64 shifted out bit */
uint32_t ProcPriority;
volatile uint64_t heartbeat_counter = 0;
int EnableJ0BreakFlag = 0;
static const uint32_t ProductIdentity = 0x00000000;
#define TimersGo   1
#define TimersStop 0
int loop;
int count1;
int count2;
int count3;
int timeslice;
int delayCount1 = 5;
int32_t quit = FALSE;
int32_t quitstatus;

/* Signal handler. */
void handler (int);

u_char *SharedLinks;
u_char *SharedEvents;

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
extern int nodeid;
extern int verbose;
extern int serve;
extern int exitonerror;
extern int FromServerLen;
extern int emudebug;
extern int memdebug;
extern int memnotinit;
extern int msgdebug;
extern int cachedebug;
extern char NetConfigName[256];

LinkIface Link[4];

/* Macros. */
#define index(a,b)		((a)+(BytesPerWord*(b)))

/* Profile information. */
uint32_t instrprof[0x400];
/*
        #00 - #FF       primary instr.
        #100 - #2FF     secondary instr. OReg
        #300 - #3FF     fpentry
*/
uint32_t combinedprof[0x400][0x400];

typedef struct _InstrSlot {
        uint32_t IPtr;
        uint32_t NextIPtr;
        uint32_t OReg;
        u_char   Icode;
        u_char   rsvd[1];
        u_short  Pcode;
#ifdef EMUDEBUG
        u_char Instruction;
#endif
} InstrSlot;

typedef struct _ArgSlot {
        uint32_t _Arg0, _Arg1;
} ArgSlot;

#define Arg0            Acache[islot]._Arg0
#define Arg1            Acache[islot]._Arg1

#define IC_NOADDR       0xDEADBEEFU
#ifndef T4CACHEBITS
#define T4CACHEBITS     (14)
#endif
#define MAX_ICACHE      (1<<T4CACHEBITS)
#define OprCombined(x,y)(((x) == 0xf0)&&((y)>0xff)&&((y)<0x17c))
InstrSlot Icache[MAX_ICACHE+1];
ArgSlot  Acache[MAX_ICACHE];

u_char *CLineTags;
#define CLINE_SIZE      ((uint32_t)(1 << T4CLINEBITS))

static uint32_t IHASH(uint32_t a)
{
        return a & (MAX_ICACHE-1);
}

static u_char IsCached(uint32_t a)
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
                        if (a == Icache[x].IPtr)
                                Icache[x].IPtr = IC_NOADDR;
                }
                return a;
        }
        return (a & ~(CLINE_SIZE-1)) + CLINE_SIZE;
}

static void InvalidateRange (uint32_t a, uint32_t n)
{
        uint32_t ha;

        ha = a + n;
        while (a < ha)
                a = InvalidateAddr (a);
}

#else

#define INVALIDATE_ADDR(a)      InvalidateSlot(IHASH(a), a)

static void InvalidateSlot (uint32_t x, uint32_t a)
{
        if (a == Icache[x].IPtr)
                Icache[x].IPtr = IC_NOADDR;
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
static u_char combinations[0x400 * 0x400];

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


struct timeval LastTOD;         /* Time-of-day */

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
static int CtrlByte = C_UNKNOWN;

int handleboot (Channel *chan, u_char *data, int ndata)
{
        uint32_t address, value;
        Channel *outchan;

        EMUDBG2 ("-I-EMUDBG: Handle boot, control byte = %d.\n", CtrlByte);

        if (C_UNKNOWN == CtrlByte)
        {
                CtrlByte = data[0];
                chan->Address = MemStart;
                if (C_POKE == CtrlByte)
                        chan->Length = 8;
                else if (C_PEEK == CtrlByte)
                        chan->Length = 4;
                else
                {
                        chan->Length = CtrlByte;
                        CtrlByte = C_BOOT;
                        BootLink = chan->LinkAddress;
                }
                EMUDBG2 ("-I-EMUDBG: Control byte #%02X.\n", CtrlByte);
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
                switch (CtrlByte)
                {
                        case C_POKE:
                                address = word_int (MemStart);
                                value   = word_int (MemStart + 4);
                                writeword_int (address, value);
                                CtrlByte = C_UNKNOWN;
                                break;
                        case C_PEEK:
                                address = word_int (MemStart);
                                value = word_int (address);
                                data[0] = value & 255; value >>= 8;
                                data[1] = value & 255; value >>= 8;
                                data[2] = value & 255; value >>= 8;
                                data[3] = value & 255;
                                outchan = &Link[chan->Link].Out;
                                ndata = nn_send (outchan->sock, data, ndata, 0);
                                if (4 != ndata)
                                {
                                        printf ("-E-EMU414: Failed to send %d bytes on Link%dOut (%s) @ handleboot()\n", ndata, outchan->Link, nn_strerror (nn_errno ()));
                                        handler (-1);
                                }
                                CtrlByte = C_UNKNOWN;
                                break;
                        case C_BOOT:
                                UpdateWdescReg (chan->Address | ProcPriority);
                                CtrlByte = C_UNKNOWN;
                                return 0;
                }
        }
        return 1;
}

int channel_ready (Channel *chan)
{
        struct nn_pollfd pfd[1];
        int ret;

        EMUDBG2 ("-I-EMUDBG: ChannelReady. Link%dIn ready ?\n", chan->Link);

        if (chan->schbuf)
        {
                ret = 0 != SCHLength(chan);
                goto Exit;
        }

        pfd[0].fd = chan->sock;
        pfd[0].events = NN_POLLIN;
        pfd[0].revents = 0;

        EMUDBG2 ("-I-EMUDBG: Polling Link%dIn.\n", chan->Link);

        ret = nn_poll (pfd, 1, LTO_POLL);
        if (-1 == ret) /* error */
        {
                printf ("-E-EMU414: Failed polling Link%dIn (%s)\n", chan->Link, nn_strerror (nn_errno ()));
                handler (-1);
        }
Exit:
        if (0 == ret) /* timeout */
        {
                MSGDBG2 ("-I-EMUDBG: Link%dIn timeout.\n", chan->Link);
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
                        SCHLength(chan) = 0;
                }
        }
        else
                ret = nn_recv (chan->sock, data, MAX_DATA, doWait ? 0 :NN_DONTWAIT);
        if (-1 == ret)
        {
                if ((EAGAIN == errno) && !doWait)
                        return -1;

                printf ("-E-EMU414: channel_recvP: Receive failed on Link%dIn (%s)\n",
                        chan->Link,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        MSGDBG4 ("-I-EMUDBG: channel_recvP: Received %d bytes on Link%dIn (#%08X).\n",
                ret,
                chan->Link,
                chan->LinkAddress);
        return ret;
}

int channel_recvmemP (Channel *chan, u_char *data, int doMemWrite, int doWait)
{
        int ret;
        u_char *buf;

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
                        SCHLength(chan) = ndata;
                        ret = ndata;
                }
        }
        else
                ret = nn_send (chan->sock, data, ndata, doWait ? 0 : NN_DONTWAIT);

        if (-1 == ret)
        {
                if ((EAGAIN == errno) && !doWait)
                        return -1;

                printf ("-E-EMU414: channel_sendP: Send failed on Link%dOut (%s).\n",
                        chan->Link,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        if (ret != ndata)
        {
                printf ("-E-EMU414: channel_sendP: Failed to send %d bytes on Link%dOut (%s).\n",
                        ndata,
                        chan->Link,
                        nn_strerror (nn_errno ()));
                handler (-1);
        }
        MSGDBG4 ("-I-EMUDBG: channel_sendP: Sent %d bytes on Link%dOut (#%08X).\n",
                ndata,
                chan->Link,
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

int linkcomms (char *where, int doBoot, int timeOut)
{
        struct nn_pollfd pfd[8];
        int npfd;
        uint32_t linkWdesc;
        Channel *channels[8];
        u_char data[MAX_DATA];
        int ndata;
        int i, ret;
        int poll;
#ifdef EMUDEBUG
        char chnames[128], tmp[16];

        chnames[0] = '\0';
#endif

        if (nodeid < 0)
                return 0;

        MSGDBG2 ("-I-EMUDBG: Link comms %s.\n", where);
        npfd = 0;
        poll = TRUE;
        for (i = 0; i < 4; i++)
        {
                if ((0 == i) && serve) /* skip Host link */
                        continue;

                linkWdesc = word (Link[i].In.LinkAddress);
                if (doBoot || (linkWdesc != NotProcess_p))
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
                if ((linkWdesc != NotProcess_p) && Link[i].Out.Length)
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
                return 0;
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
        if (channels[0]->schbuf) /* shared channels? */
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
                                usleep (SCH_POLL); timeOut -= SCH_POLL;
                                if (timeOut < 0) timeOut = 0;
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
                return doBoot ? 0 : 1;
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
                                printf ("-I-EMUDBG: Polled on Link%dIn (#%08X).\n", channels[i]->Link, channels[i]->LinkAddress);
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

        return chan;
}

/* Open a link channel */
void open_channel (uint32_t addr)
{
        Channel *chan;
        int chanIn, theLink;
        int othernode, otherlink;
        int ret;
        u_char *nodeBase;

        chan = reset_channel (addr);
        chanIn = IsLinkIn(addr);
        theLink = TheLink(addr);

        chan->LinkAddress = addr;
        chan->Link = theLink;
        chan->url[0] = '\0';
        chan->sock = -1;
        chan->schbuf = NULL;
        chan->IOBytes = 0;

        if (serve && 0 == theLink) /* host link */
                return;
        if (nodeid < 0) /* no Node ID */
                return;

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
                                SharedLinks = shlink_alloc (NetConfigName, sharedSize + 1024);
                                if (SharedLinks)
                                        memset (SharedLinks, 0, sharedSize + 1024);
                        }
                        else
                                SharedLinks = shlink_attach (NetConfigName, sharedSize + 1024);
                        if (NULL == SharedLinks)
                                handler (-1);
                        SharedEvents = SharedLinks + sharedSize;
                }
                if (chanIn)
                {
                        nodeBase = SharedLinks + (nodeid * 8 * SCH_SIZE);
                        chan->schbuf = nodeBase + (4 + theLink) * SCH_SIZE;
                        if (verbose)
                                printf ("-I-EMU414: Link%dIn  at #%08lX %d:%d.\n",
                                        theLink,
                                        chan->schbuf - SharedLinks,
                                        nodeid, theLink);
                }
                else if (0 == connectedNetLink(nodeid, theLink, &othernode, &otherlink))
                {
                        nodeBase = SharedLinks + (othernode * 8 * SCH_SIZE);
                        chan->schbuf = nodeBase + (4 + otherlink) * SCH_SIZE;
                        if (verbose)
                                printf ("-I-EMU414: Link%dIn  at #%08lX %d:%d.\n",
                                        theLink,
                                        chan->schbuf - SharedLinks,
                                        othernode, otherlink);
                }
                return;
        }
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
                Icache[i].IPtr  = IC_NOADDR;
                Icache[i].Pcode = 0x3ff;
        }

        SharedLinks = NULL;

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
        memset (combinations, 0, sizeof(combinations));
        for (i = 0; NO_ICODE != combined[i].code0; i++)
                combinations[(combined[i].code0 << 10) + combined[i].code1] = 1 + i;
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
                else
                        ret = 0x100 + oprCode;
        }
        return ret;
}

void mainloop (void)
{
        uint32_t temp, temp2;
        uint32_t otherWdesc, otherWPtr, otherPtr, altState;
        uint32_t PrevError;
        u_char pixel;
        fpreal32_t sntemp1, sntemp2;
        fpreal64_t dbtemp1, dbtemp2;
        REAL       fptemp;
        unsigned int islot, pslot;
        int i;

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

	fprintf(stderr, "[DEBUG: Entering mainloop, IPtr=0x%08X Wptr=0x%08X]\n", IPtr, WPtr);

	count1 = 0;
	count2 = 0;
	count3 = 0;
	timeslice = 0;
	Timers = TimersStop;
	TimerEnableHi = 1;
	TimerEnableLo = 1;

	{
		const char *outbyte_env = getenv("T4_OUTBYTE_TRACE");
		if (outbyte_env && *outbyte_env)
			outbyte_trace_enabled = 1;
	}
	{
		const char *uart_env = getenv("T4_UART_CONSOLE");
		if (uart_env && *uart_env)
			uart_console_enabled = (int)strtoul(uart_env, NULL, 0) != 0;
	}
	{
		const char *uart_force_env = getenv("T4_UART_CONSOLE_FORCE");
		if (uart_force_env && *uart_force_env &&
		    (int)strtoul(uart_force_env, NULL, 0) != 0) {
			uart_console_enabled = 1;
			uart_trace_enabled = 0;
		}
	}
	{
		const char *link_env = getenv("T4_LINK0_TRACE");
		if (link_env && *link_env)
			link0_trace_enabled = (int)strtoul(link_env, NULL, 0) != 0;
	}
	{
		const char *mmio_env = getenv("T4_MMIO_TRACE");
		if (mmio_env && *mmio_env)
			mmio_trace_enabled = (int)strtoul(mmio_env, NULL, 0) != 0;
	}
	{
		const char *ep_env = getenv("T4_EARLY_PRINTK_TRACE");
		if (ep_env && *ep_env)
			early_printk_trace_enabled = (int)strtoul(ep_env, NULL, 0) != 0;
	}
	{
		const char *near_env = getenv("T4_NEAREST_SYM");
		if (near_env && *near_env)
			nearest_sym_enabled = (int)strtoul(near_env, NULL, 0) != 0;
	}
	{
		const char *boot_dump_env = getenv("T4_BOOT_DUMP_START");
		if (boot_dump_env && *boot_dump_env)
			boot_dump_start_enabled = (int)strtoul(boot_dump_env, NULL, 0) != 0;
	}
#ifdef T4_X11_FB
	{
		const char *fb_console_env = getenv("T4_FB_CONSOLE");
		if (fb_console_env && *fb_console_env)
			fb_console_enabled = (int)strtoul(fb_console_env, NULL, 0) != 0;
	}
#endif

	/* Open UART log file */
	{
		const char *uart_log_env = getenv("T4_UART_LOG");
		const char *uart_log_path = uart_log_env && *uart_log_env ?
		                            uart_log_env : "boot.uart";
		uart_log_file = fopen(uart_log_path, "w");
		if (uart_log_file) {
			fprintf(stderr, "[UART] Logging to %s\n", uart_log_path);
			setbuf(uart_log_file, NULL);  /* Unbuffered for real-time logging */
		}
	}

        islot = MAX_ICACHE;
        PROFILE(update_tod (&StartTOD));
	while (1)
	{
#ifndef NDEBUG
                temp = temp2 = Undefined_p;
                otherWdesc = otherWPtr = otherPtr = altState = Undefined_p;
                dbtemp1 = dbtemp2 = DUndefined;
                sntemp1 = sntemp2 = RUndefined;
#endif
#ifdef EMUDEBUG
                r32temp = RUndefined;
#endif
                /* Save current value of Error flag */
                PrevError = ReadError;

                /* Process VGA/X11 events periodically (every 1000 instructions for smoother display) */
#ifdef T4_X11_FB
                {
                        static uint32_t sdl_event_counter = 0;
                        if ((++sdl_event_counter % 1000) == 0) {
                                vga_process_events();
                        }
                }
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
			const char *sym_name = getenv("T4_DUMP_SYM");
			if (sym_name && *sym_name)
				dump_sym_bytes_once(sym_name);
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
                        static uint32_t entry_start = 0;
                        static uint32_t entry_end = 0;
                        if (!entry_trace_init) {
                                const char *env = getenv("T4_ENTRY_TRACE");
                                const char *env_start = getenv("T4_ENTRY_START");
                                const char *env_end = getenv("T4_ENTRY_END");
                                const char *env_iptr = getenv("T4_ENTRY_IPTR");
                                entry_trace_enabled = (env && *env);
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
                                        fprintf(stderr,
                                                "[ENTRY] IPtr=0x%08X WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X O=0x%08X\n",
                                                IPtr, WPtr, AReg, BReg, CReg, OReg);
                                        entry_trace_done = 1;
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

		/* Move timers on if necessary, and increment timeslice counter. */
                if (++count1 > delayCount1)
                {
                        if (SharedEvents && (nodeid >= 0))
                        {
                                if (SharedEvents[nodeid])
                                        return;
                        }
		        update_time ();
                }

                if (ReadGotoSNP)
                {
                        fprintf(stderr, "[SNP] GotoSNP set! IPtr=0x%08X WPtr=0x%08X heartbeat=%llu\n",
                                IPtr, WPtr, (unsigned long long)heartbeat_counter);
                        fflush(stderr);
                        if (start_process ())
                                return;
                }

		/* Execute an instruction. */
        ResetRounding = FALSE;

        pslot = islot;
        islot = IHASH(IPtr);
        if (IPtr != Icache[islot].IPtr)
        {
                PROFILE(profile[PRO_ICMISS]++);
                Icache[islot].IPtr = IPtr;
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

FetchNext:      Instruction = byte_int (IPtr);
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
			heartbeat_counter++;
			if ((heartbeat_counter % 10000000) == 0) {
				fprintf(stderr, "[PFXHB] %lluM IPtr=0x%08X OReg=0x%08X WPtr=0x%08X\n",
					(unsigned long long)(heartbeat_counter / 1000000), IPtr, OReg, WPtr);
				fflush(stderr);
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
                Icache[islot].OReg     = OReg;
                Icache[islot].Icode    = Icode;
                /* Cache debug tracing for store */
                if (cache_debug_trace_enabled &&
                    Icache[islot].IPtr >= cache_debug_trace_start &&
                    Icache[islot].IPtr < cache_debug_trace_end) {
                        fprintf(stderr,
                                "[CACHE_STORE] slot=0x%X origIPtr=0x%08X NextIPtr=0x%08X OReg=0x%08X Icode=0x%02X WPtr=0x%08X\n",
                                islot, Icache[islot].IPtr, IPtr, OReg, Icode, WPtr);
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
                                && (Icache[pslot].NextIPtr == (Icache[islot].IPtr - 1)))
                        {
                                i--;
                                EMUDBG3 ("-I-EMU414: code0=#%03X code1=#%03X\n", code0, code1);
                                EMUDBG2 ("-I-EMU414: combined code=#%03X\n", combined[i].ccode);

                                Acache[pslot]._Arg0 = Icache[pslot].OReg;
                                Acache[pslot]._Arg1 = Icache[islot].OReg;

                                Icache[pslot].NextIPtr = Icache[islot].NextIPtr;
                                Icache[pslot].OReg  = combined[i].ccode;
                                Icache[pslot].Icode = 0xF0;
                                Icache[pslot].Pcode = ProfileCode (Icache[pslot].Icode, Icache[pslot].OReg);
                                EMUDBG4 ("-I-EMU414: OReg=#%X Arg0=#%X Arg1=#%X\n", Icache[pslot].OReg, Acache[pslot]._Arg0, Acache[pslot]._Arg1);
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
        OReg  = Icache[islot].OReg;
        Icode = Icache[islot].Icode;
        /* Cache debug tracing */
        if (cache_debug_trace_enabled &&
            IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
                uint8_t mem_op0 = byte_int(Icache[islot].IPtr);
                uint8_t mem_op1 = byte_int(Icache[islot].IPtr + 1);
                uint8_t mem_op2 = byte_int(Icache[islot].IPtr + 2);
                uint8_t mem_op3 = byte_int(Icache[islot].IPtr + 3);
                fprintf(stderr,
                        "[CACHE_HIT] slot=0x%X origIPtr=0x%08X NextIPtr=0x%08X OReg=0x%08X Icode=0x%02X mem=%02X%02X%02X%02X WPtr=0x%08X\n",
                        islot, Icache[islot].IPtr, IPtr, OReg, Icode,
                        mem_op0, mem_op1, mem_op2, mem_op3, WPtr);
        }

ExecuteInstr:
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

	heartbeat_counter++;
	if ((heartbeat_counter % 100) == 0) {
		fprintf(stderr, "[HB] %llu IPtr=0x%08X WPtr=0x%08X AReg=0x%08X Icode=0x%02X\n",
			(unsigned long long)heartbeat_counter, IPtr, WPtr, AReg, Icode);
		fflush(stderr);
	}

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
			       if (sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDLP IPtr=0x%08X OReg=0x%X AReg:0x%08X->0x%08X WPtr=0x%08X\n",
			                   IPtr, OReg, oldA, AReg, WPtr);
			       }
			       if (sk_addr_trace_enabled &&
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
			       /* Debug: Show critical ldnl operations in bootstrap range */
			       if (IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			           printf("[LDNL] IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X value=0x%08X\n",
			                  IPtr, AReg, OReg, addr, value);
			       }
			       AReg = value;
			       if (sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDNL IPtr=0x%08X OReg=0x%X addr=0x%08X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, addr, oldA, AReg);
			       }
			       if (sk_addr_trace_enabled &&
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
			       /* Trace LDC that loads framebuffer address */
			       if ((AReg & 0xF0000000) == 0x90000000) {
			           fprintf(stderr, "[FB_LDC] IPtr=0x%08X loaded=0x%08X\n", IPtr, AReg);
			       }
			       if (sk_areg_trace_enabled &&
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
			       if (sk_areg_trace_enabled &&
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
			       if (sk_areg_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKA] LDL IPtr=0x%08X OReg=0x%X addr=0x%08X AReg:0x%08X->0x%08X\n",
			                   IPtr, OReg, addr, oldA, AReg);
			       }
			       if (sk_addr_trace_enabled &&
			           IPtr >= 0x80883FF0 && IPtr <= 0x80884020) {
			           fprintf(stderr,
			                   "[SKADDR] LDL IPtr=0x%08X OReg=0x%X WPtr=0x%08X addr=0x%08X value=0x%08X\n",
			                   IPtr, OReg, WPtr, addr, AReg);
			       }
			   }
			   /* Debug: Trace ldl in bootstrap range */
			   if (IPtr >= 0x80000070 && IPtr <= 0x80000200) {
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
			       if (sk_areg_trace_enabled &&
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
			   IPtr++;
			   /* Log kernel function calls */
			   if (IPtr >= 0x80000000 && IPtr < 0x81000000) {
			       printf("[CALL] from=0x%08X to=0x%08X WPtr=0x%08X→0x%08X\n",
			              IPtr - 1, IPtr + OReg, WPtr, index(WPtr, -4));
			   }
			   maybe_log_nearest_sym(IPtr - 1, "CALL");
			   if (wptr_trace_enabled) {
			       printf("[WPtr] CALL IPtr=0x%08X target=0x%08X WPtr=0x%08X→0x%08X\n",
			              IPtr - 1, IPtr + OReg, WPtr, index(WPtr, -4));
			   }
			   if (bootdbg_stop_after_call_enabled &&
			       (IPtr - 1) >= boot_call_start && (IPtr - 1) <= boot_call_end) {
			       last_boot_call_is_gcall = 0;
			       last_boot_call_site = IPtr - 1;
			       last_boot_call_target = IPtr + OReg;
			       last_boot_call_return = IPtr;
			       last_boot_call_wptr = WPtr;
			       maybe_auto_text_base();
			       stop_after_boot_call = 1;
			       if (bootdbg_enabled)
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
			   /* Cache debug tracing for CALL target */
			   if (cache_debug_trace_enabled &&
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
			       /* Cache debug tracing for AJW */
			       if (cache_debug_trace_enabled &&
			           IPtr >= cache_debug_trace_start && IPtr < cache_debug_trace_end) {
			           fprintf(stderr,
			                   "[AJW_DEBUG] IPtr=0x%08X OReg=0x%08X delta=%d WPtr=0x%08X→0x%08X\n",
			                   IPtr, OReg, delta, WPtr, new_wptr);
			       }
			       /* Log ALL workspace changes in bootstrap range, or significant changes in kernel */
			       if ((IPtr >= 0x80000070 && IPtr <= 0x80000200) ||
			           (IPtr >= 0x80000000 && IPtr < 0x81000000 && (delta > 10 || delta < -10))) {
			           printf("[AJW] IPtr=0x%08X delta=%d WPtr=0x%08X→0x%08X\n",
			                  IPtr, delta, WPtr, new_wptr);
			       }
			       /* Validate workspace pointer - use actual configured memory size */
			       if (new_wptr == 0x2FFA2FFA || new_wptr < MemStart || new_wptr >= (MemStart + MemSize)) {
			           printf("[ERROR] Invalid WPtr after ajw: 0x%08X at IPtr=0x%08X (valid range: 0x%08X-0x%08X)\n",
			                  new_wptr, IPtr, MemStart, MemStart + MemSize);
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
			       if (text_write_trace_enabled &&
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
			   if (IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			       printf("[STL] IPtr=0x%08X local=%d addr=0x%08X AReg=0x%08X stored\n",
			              IPtr, OReg, index(WPtr, OReg), AReg);
			   }
			   if (sk_store_trace_enabled && sk_entry_iptr &&
			       IPtr >= sk_entry_iptr && IPtr <= (sk_entry_iptr + 0x200)) {
			       printf("[SKSTL] IPtr=0x%08X local=%d addr=0x%08X AReg=0x%08X\n",
			              IPtr, OReg, index(WPtr, OReg), AReg);
			   }
			   if (sk_areg_trace_enabled &&
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
			       if ((stnl_addr & 0xFFFFF000) == 0xA0000000) {
			           fprintf(stderr, "[VGA_STNL] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                   IPtr, stnl_addr, BReg);
			       }
			       /* Debug: trace ALL STNLs in setup_arch to find FB writes */
			       if (IPtr >= 0x8053c57b && IPtr < 0x8053d57b) {
			           static int setup_stnl_count = 0;
			           if (setup_stnl_count < 200) {
			               fprintf(stderr, "[SETUP_STNL#%d] IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X val=0x%08X\n",
			                       setup_stnl_count, IPtr, AReg, OReg, stnl_addr, BReg);
			               setup_stnl_count++;
			           }
			       }
			       if ((stnl_addr & 0xF0000000) == 0x90000000) {
			           static int fb_stnl_count = 0;
			           if (fb_stnl_count < 50) {
			               fprintf(stderr, "[FB_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_stnl_count, IPtr, stnl_addr, BReg);
			               fb_stnl_count++;
			           }
			       }
			   }
			   if (bootdbg_enabled && IPtr >= 0x80883FE0 && IPtr <= 0x80884020) {
			       uint32_t addr = index(AReg, OReg);
			       fprintf(stderr, "[BOOTDBG] STNL IPtr=0x%08X AReg=0x%08X OReg=0x%X addr=0x%08X BReg=0x%08X\n",
			               IPtr, AReg, OReg, addr, BReg);
			   }
			   if (sk_store_trace_enabled && sk_entry_iptr &&
			       IPtr >= sk_entry_iptr && IPtr <= (sk_entry_iptr + 0x200)) {
			       uint32_t addr = index(AReg, OReg);
			       printf("[SKSTNL] IPtr=0x%08X addr=0x%08X BReg=0x%08X\n",
			              IPtr, addr, BReg);
			   }
			   {
			       uint32_t addr = index(AReg, OReg);
			       if (text_write_trace_enabled &&
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
			   if (sk_areg_trace_enabled &&
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
	switch (OReg)
	{
		case 0x00: /* rev         */
			   {
			       uint32_t oldA = AReg;
			       temp = AReg;
			       AReg = BReg;
			       BReg = temp;
			       if (sk_areg_trace_enabled &&
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
			   if (IPtr >= 0x80000070 && IPtr <= 0x80000200) {
			       printf("[GCALL] IPtr=0x%08X AReg=0x%08X (target) WPtr=0x%08X BReg=0x%08X CReg=0x%08X\n",
			              IPtr, AReg, WPtr, BReg, CReg);
			   }
			   maybe_log_nearest_sym(IPtr, "GCALL");
			   if (wptr_trace_enabled) {
			       printf("[WPtr] GCALL IPtr=0x%08X target=0x%08X WPtr=0x%08X\n",
			              IPtr, AReg, WPtr);
			   }
			   IPtr++;
			   temp = AReg;
			   AReg = IPtr;
			   IPtr = temp;
			   if (bootdbg_enabled && temp >= 0x804BAC00 && temp <= 0x804BB000) {
			       fprintf(stderr,
			               "[BOOTDBG] gcall target IPtr=0x%08X return_addr=0x%08X WPtr=0x%08X\n",
			               IPtr, AReg, WPtr);
			   }
			   /* Show where we're jumping to */
			   if (IPtr >= 0x80000070 && IPtr <= 0x80000200 || temp >= 0x80000070 && temp <= 0x80000200) {
			       printf("[GCALL] Jumping to IPtr=0x%08X return_addr=0x%08X\n", IPtr, AReg);
			   }
			   last_boot_call_is_gcall = 1;
			   last_boot_call_site = call_site;
			   last_boot_call_target = IPtr;
			   last_boot_call_return = AReg;
			   last_boot_call_wptr = WPtr;
			   maybe_auto_text_base();
			   if (bootdbg_stop_after_call_enabled &&
			       call_site >= boot_call_start && call_site <= boot_call_end) {
			       stop_after_boot_call = 1;
			       if (bootdbg_enabled)
			               fprintf(stderr,
			                       "[BOOTDBG] armed stop after GCALL @ 0x%08X target=0x%08X return=0x%08X\n",
			                       last_boot_call_site, last_boot_call_target, last_boot_call_return);
			   }
			   }
			   break;
		case 0x07: /* in          */
OprIn:                     if (IsLinkOut(BReg)) /* M.Bruestle 22.1.2012 */
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
OprOut:                    if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
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
							schedule (otherWdesc);
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
                           if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
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
							schedule (otherWdesc);
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
                           if (IsLinkIn(BReg)) /* M.Bruestle 22.1.2012 */
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
							schedule (otherWdesc);
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
			   AReg = word (temp);
                           reset_channel (temp);
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
			       if (IPtr >= 0x80000000 && IPtr < 0x81000000) {
			           printf("[RET] from=0x%08X to=0x%08X WPtr=0x%08X→0x%08X\n",
			                  IPtr, return_addr, WPtr, new_wptr);
			       }
			       /* Validate return address */
			       if (return_addr == 0x2FFA2FFA || return_addr == 0x80000000) {
			           printf("[ERROR] Invalid return address 0x%08X at IPtr=0x%08X WPtr=0x%08X\n",
			                  return_addr, IPtr, WPtr);
			       }
			       /* TCWG spec: "The ret instruction restores the Iptr and adjusts the
			        * workspace pointer to deallocate the four locations." */
			       IPtr = return_addr;
			       UpdateWdescReg (new_wptr | ProcPriority);
                               T4DEBUG(checkWPtr ("RET", WPtr));
			       /* Cache debug tracing for RET */
			       if (cache_debug_trace_enabled &&
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
			   if (sk_areg_trace_enabled &&
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
			   if (sk_store_trace_enabled && sk_entry_iptr &&
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
			   temp = AReg;
                           if (wptr_trace_enabled) {
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
                           if (INT32(m2dWidth) >= 0 && INT32(m2dLength) >= 0)
                           {
                                for (temp = 0; temp < m2dLength; temp++)
                                {
                                        movebytes_int (m2dDestAddress, m2dSourceAddress, m2dWidth);
                                        m2dSourceAddress += m2dSourceStride;
                                        m2dDestAddress   += m2dDestStride;
                                }
                           }
#ifndef NDEBUG
                           m2dSourceStride = m2dDestStride = m2dLength = Undefined_p;
#endif
		           IPtr++;
		           break;
		case 0x5d: /* XXX move2dnonzero    */
		           BADCODE(IsT414);
                           if (INT32(m2dWidth) >= 0 && INT32(m2dLength) >= 0)
                           {
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
		case 0x7e: /* ldmemstartval */
			   CReg = BReg;
			   BReg = AReg;
			   AReg = MemStart;
			   IPtr++;
			   break;
		case 0xfa: /* skip invalid opcode */
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
			   /* EQC: AReg = (AReg == Arg0) ? 1 : 0
			    * CJ: if AReg == 0, jump (no pop); if AReg != 0, continue (pop)
			    * Combined: if original AReg != Arg0, jump; else continue and pop
			    * Fix for while(0): if Arg0 == 0, jump if original AReg == 0, else pop
			    */
			   {
			       static int eqc_cj_count = 0;
			       eqc_cj_count++;
			       IPtr++;
			       if (Arg0 == 0) {
			       	   /* Special case for while(0): always exit loop */
			       	   AReg = 0; // hack to force continue
			       	   if (eqc_cj_count < 10)
			       	       fprintf(stderr, "[EQC_CJ] #%d CONTINUE (while0) AReg=0x%08X Arg0=0x%08X\n",
			       	               eqc_cj_count, AReg, Arg0);
			       	   AReg = BReg;
			       	   BReg = CReg;
			       } else {
			       	   if (AReg == Arg0) {
			       	       /* Equal: EQC gives 1, CJ continues (AReg != 0), pop stack */
			       	       if (eqc_cj_count < 10)
			       	           fprintf(stderr, "[EQC_CJ] #%d CONTINUE AReg=0x%08X Arg0=0x%08X\n",
			       	                   eqc_cj_count, AReg, Arg0);
			       	       AReg = BReg;
			       	       BReg = CReg;
			       	   } else {
			       	       /* Not equal: EQC gives 0, CJ jumps (AReg == 0), NO pop */
			       	       if (eqc_cj_count < 10)
			       	           fprintf(stderr, "[EQC_CJ] #%d JUMP AReg=0x%08X Arg0=0x%08X Arg1=%d\n",
			       	                   eqc_cj_count, AReg, Arg0, (int)Arg1);
			       	       IPtr = IPtr + Arg1;
			       	   }
			       }
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
                           WPtr = index (WPtr, Arg0);
                           T4DEBUG(checkWPtr ("AJW", WPtr));
			   IPtr = word (WPtr);
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
			       if ((addr & 0xF0000000) == 0x90000000) {
			           static int fb_combo_count = 0;
			           if (fb_combo_count < 50) {
			               fprintf(stderr, "[FB_LDL_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_combo_count, IPtr, addr, BReg);
			               fb_combo_count++;
			           }
			       }
			       if (text_write_trace_enabled &&
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
			       if ((addr & 0xF0000000) == 0x90000000) {
			           static int fb_wsub_count = 0;
			           if (fb_wsub_count < 50) {
			               fprintf(stderr, "[FB_WSUB_STNL#%d] IPtr=0x%08X addr=0x%08X val=0x%08X\n",
			                       fb_wsub_count, IPtr, addr, BReg);
			               fb_wsub_count++;
			           }
			       }
			       if (text_write_trace_enabled &&
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
                case 0x17c: /* lddevid */
                           CReg = BReg;
                           BReg = AReg;
                           AReg = ProductIdentity;
                           IPtr++;
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

/* Add a process to the relevant priority process queue. */
void schedule (uint32_t wdesc)
{
        uint32_t wptr, pri;
	uint32_t temp;

        wptr = GetDescWPtr(wdesc);
#ifndef NDEBUG
        if (NotProcess_p == wptr)
        {
                printf ("-E-EMU414: Wdesc = #%08X. Cannot schedule NotProcess_p.\n", wdesc);
                handler (-1);
        }
#endif
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
		UpdateWdescReg (GetDescWPtr(word (index (MostNeg, 11))) | LoPriority);
                T4DEBUG(checkWPtr ("RunProcess(1)", WPtr));
	        EMUDBG3 ("-I-EMUDBG: RunProcess: ProcPriority = %s, WPtr = #%08X.\n",
                                StrPrio(ProcPriority), WPtr);

		IPtr = word (index (MostNeg, 12));
		AReg = word (index (MostNeg, 13));
		BReg = word (index (MostNeg, 14));
		CReg = word (index (MostNeg, 15));
		STATUSReg = word (index (MostNeg, 16));
		/*EReg = word (index (MostNeg, 17));*/

#ifdef EMUDEBUG
                if (IsT800 || IsTVS)
#endif
                {
                        FAReg = FARegSave;
                        FBReg = FBRegSave;
                        FCReg = FCRegSave;
                }
                ClearInterrupt; /* XXX Not necessary ??? */
	}  
	else
	{
	        lastptr = BPtrReg[ProcPriority];

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
                if (0 == run_process ())
                        break;

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
                        links_active = (0 != linkcomms ("idle", FALSE, LTO_BOOT));
                else
                        links_active = (0 != linkcomms ("running", FALSE, LTO_COMM));
                active = active || links_active;
                if (!Idle)
                {
                        active = TRUE;
                        break;
                }

		/* Check timer queue, update timers. */
                active = active || (!TimerQEmpty);
                if (++count1 > delayCount1)
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
        } while (active);

        if (!active)
        {
                printf ("-E-EMU414: Node%d Error - stopped no Link/Process/Timer activity!\n", nodeid < 0 ? 0 : nodeid);
                processor_state ();
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
	writeword (index (MostNeg, 11), Wdesc);
        if (IdleProcess_p != Wdesc)
        {
                EMUDBG ("-I-EMUDBG: Interrupt LoPriority process.\n");

	        writeword (index (MostNeg, 12), IPtr);
	        writeword (index (MostNeg, 13), AReg);
	        writeword (index (MostNeg, 14), BReg);
	        writeword (index (MostNeg, 15), CReg);
	        writeword (index (MostNeg, 16), STATUSReg);
	        /*writeword (index (MostNeg, 17), EReg);*/

#ifdef EMUDEBUG
                if (IsT800 || IsTVS)
#endif
                {
                        FARegSave = FAReg;
                        FBRegSave = FBReg;
                        FCRegSave = FCReg;
                }
        }
        /* Note: that an interrupted process is not placed onto the scheduling lists. */
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
#ifdef T4_X11_FB
    vga_process_events();
#endif
}

#define CoreAddr(a)     (INT32(a) < INT32(ExtMemStart))
#define ExtMemAddr(a)   (INT32(ExtMemStart) <= INT32(a))
#define CoreRange(a,n)  (CoreAddr(a) && CoreAddr((a)+(n)))
#define ExtMemRange(a,n)(ExtMemAddr(a) && ExtMemAddr((a)+(n)))
#define MemWordPtr(a)   ((CoreAddr(a) ? core : mem) + (MemWordMask & (a)))

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

#ifdef T4_X11_FB
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
#ifdef T4_X11_FB
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
#ifdef T4_X11_FB
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
#ifdef T4_X11_FB
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

	if (ptr == LINK0_IN)
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

#ifdef T4_X11_FB
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
	static int write_count = 0;
	static int uart_col = 0;  /* Track column position for console output */
	u_char ch = 0;

	if (ptr == LINK0_OUT) {
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
			fprintf(stderr, "[UART#%d: val=0x%02X='%c']\n", write_count, value & 0xFF,
			        (value >= 32 && value < 127) ? (char)(value & 0xFF) : '?');
		}
	}
	write_count++;

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
		const char *env = getenv("T4_UART_VERBOSE");
		if (env && *env) {
			fprintf(stderr, "[UART_LOG] IPtr=0x%08X WPtr=0x%08X ch=0x%02X '%c'\n",
				IPtr, WPtr, ch, (ch >= 32 && ch < 127) ? ch : '?');
		} else {
			fprintf(stderr, "[UART_LOG] ch=0x%02X '%c'\n", ch, (ch >= 32 && ch < 127) ? ch : '?');
		}
		uart_direct_track(ch);
		if (!uart_console_enabled)
			return;

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

#ifdef T4_X11_FB
	/* VGA Controller register handling at 0xA0000000 */
	if (ptr == VGA_CTRL_CONTROL) {
		uint32_t old_ctrl = vga_ctrl_control;
		vga_ctrl_control = value;
		fprintf(stderr, "[VGA] Control register set to 0x%08X (%s)\n",
		        value, (value & VGA_CTRL_ENABLE) ? "ENABLED" : "disabled");
		/* Force immediate refresh when display is first enabled */
		if ((value & VGA_CTRL_ENABLE) && !(old_ctrl & VGA_CTRL_ENABLE)) {
			fprintf(stderr, "[VGA] Display enabled - forcing immediate refresh\n");
			vga_dirty = 1;
			vga_update();
		}
		return;
	}
	if (ptr == VGA_CTRL_FB_BASE) {
		vga_ctrl_fb_base = value;
		fprintf(stderr, "[VGA] Framebuffer base set to 0x%08X\n", value);
		return;
	}
	if (ptr == VGA_CTRL_WIDTH) {
		vga_ctrl_width = value;
		fprintf(stderr, "[VGA] Width set to %u\n", value);
		return;
	}
	if (ptr == VGA_CTRL_HEIGHT) {
		vga_ctrl_height = value;
		fprintf(stderr, "[VGA] Height set to %u\n", value);
		return;
	}
	if (ptr == VGA_CTRL_STRIDE) {
		vga_ctrl_stride = value;
		fprintf(stderr, "[VGA] Stride set to %u\n", value);
		return;
	}

	if (fb_range_intersects(ptr, sizeof(uint32_t))) {
		static int fb_word_count = 0;
		/* Log first 50 FB writes, or any write to first 4 rows (addresses 0x90000000-0x90001E00) */
		if (fb_word_count < 50 || ptr < 0x90002800) {
			fprintf(stderr, "[FB_WR_WORD#%d] addr=0x%08X val=0x%08X IPtr=0x%08X\n",
			        fb_word_count, ptr, value, IPtr);
		}
		fb_word_count++;
		for (int i = 0; i < 4; i++) {
			writebyte_int(ptr + i, (u_char)((value >> (8 * i)) & 0xFF));
		}
		return;
	}
#endif

	/* Debug: Track any write to 0x9xxxxxxx range even if not caught by fb_range_intersects */
	if ((ptr & 0xF0000000) == 0x90000000) {
		fprintf(stderr, "[SUSPICIOUS_FB_ADDR] addr=0x%08X val=0x%08X - not caught by fb_range!\n",
		        ptr, value);
	}

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

	if (ptr == LINK0_IN)
		return mmio_iserver_read_byte();

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

#ifdef T4_X11_FB
	if (fb_addr_in_range(ptr))
		return fb_read_byte(ptr);
#endif

	/* Get byte, ensuring memory reference is in range. */
        if (CoreAddr(ptr))
	        result = core[(ptr & MemByteMask)];
        else
	        result = mem[(ptr & MemByteMask)];

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
	if (ptr == LINK0_OUT) {
		mmio_iserver_write_byte(value);
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
			uart_write_byte(value);
		}
		return;
	}

#ifdef T4_X11_FB
	/* Check for framebuffer access */
	if (fb_addr_in_range(ptr)) {
		fb_write_byte(ptr, value);
		return;
	}
#endif

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
#ifdef T4_X11_FB
        if (len)
        {
                uint64_t start = ptr;
                uint64_t end = start + (uint64_t)len;
                if ((start >= FB_BASE) && (end <= FB_LIMIT))
                        return vga_framebuffer + (ptr - FB_BASE);
        }
#endif
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
#ifdef T4_X11_FB
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

#ifdef T4_X11_FB
        if (len && (fb_range_intersects(dst, len) || fb_range_intersects(src, len)))
        {
                if (dst < src)
                {
                        for (uint32_t i = 0; i < len; i++)
                                writebyte_int (dst + i, byte_int (src + i));
                }
                else if (dst > src)
                {
                        for (uint32_t i = len; i > 0; i--)
                                writebyte_int (dst + i - 1, byte_int (src + i - 1));
                }
                return;
        }
#endif

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
                p = mem + (dst & MemByteMask);
                q = mem + (src & MemByteMask);
                memmove (p, q, len);
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
#ifdef T4_X11_FB
        if (len)
        {
                uint64_t start = ptr;
                uint64_t end = start + (uint64_t)len;
                if ((start >= FB_BASE) && (end <= FB_LIMIT))
                        return vga_framebuffer + (ptr - FB_BASE);
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
