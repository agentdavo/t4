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


/*
 * main.c
 *
 * Read and parse command line. Start emulator.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#ifdef _MSC_VER
#include <io.h>
#include <fcntl.h>
#include <process.h>
typedef intptr_t pid_t;
#else
#include <unistd.h>
#include <pthread.h>
#endif
#include "t4debug.h"
#include "processor.h"
#include "cpu_state.h"
#include "t4_fabric.h"
#include "t9000_group.h"
#include "t800_cycles.h"
#include "t9000_ctl.h"
#include "netcfg.h"
#include "display_backend.h"

#ifdef T4_SDL_FB
extern void save_framebuffer_png(const char *filename);
extern void vga_dump_framebuffer_bmp(const char *filename, int frame_number);
#endif

/* T4Net — simple MMIO NIC with TAP backend */
extern void t4net_init(const char *tap_name);
extern void t4net_cleanup(void);
extern void t4net_poll_rx(void);

#ifdef __MWERKS__
  #include "mac_input.h"
  #include <profiler.h>
#else
  #define true TRUE
  #define false FALSE
  #ifdef CURTERM
    #include "curterm.h"
  #else
    #ifdef SUN
      #include <termio.h>
    #else
      #include <sys/termios.h>
      #include <sys/ioctl.h>
      #define TCGETS TIOCGETA
      #define TCSETS TIOCSETA
      #define termio termios
    #endif
  #endif
#endif

#undef TRUE
#undef FALSE
#define TRUE  0x0001
#define FALSE 0x0000

void handler (int);

int verbose     = FALSE;
int analyse     = FALSE;
int want_threads = FALSE;
static pthread_t t4_wth[1024];
static cpu_t *t4_wc[1024];
static int t4_nworkers;

/* Set while this thread runs mainloop(), which services t4_sig_pending
 * (see handler()). */
static T4_THREAD_LOCAL int t4_in_mainloop;

/* T4_NODE_BTL=pattern, e.g. n%d.btl: a threaded node other than 0 loads its
 * own netload image (the file -sb takes) instead of waiting for boot code on
 * a link, so every node of a threaded network - C104 routers included - can
 * run a C program.  "%d" in the pattern becomes the node id.  Returns 1 if
 * booted, 0 if there is no pattern, -1 on error. */
static int t4_boot_node_file(int node)
{
        const char *pat = getenv("T4_NODE_BTL"), *pc;
        char path[512];
        unsigned char buf[256];
        uint32_t addr = MemStart;
        FILE *f;
        int chunk;

        if (!pat || !*pat)
                return 0;
        pc = strstr(pat, "%d");
        if (pc)
                snprintf(path, sizeof path, "%.*s%d%s", (int)(pc - pat), pat, node, pc + 2);
        else
                snprintf(path, sizeof path, "%s", pat);
        f = fopen(path, "rb");
        if (!f || getc(f) != 0) {
                printf("-E-EMU414: Node%d: %s is not a netload image\n", node, path);
                if (f)
                        fclose(f);
                return -1;
        }
        while ((chunk = getc(f)) != EOF && chunk != 0) {
                if (fread(buf, 1, (size_t)chunk, f) != (size_t)chunk)
                        break;
                writebytes_int(addr, buf, (uint32_t)chunk);
                addr += (uint32_t)chunk;
        }
        fclose(f);
        WPtr = (addr + 3u) & ~3u;
        UpdateWdescReg(WPtr | LoPriority);
        return 1;
}

static void *t4_node_thread(void *arg)
{
        cpu_t *c = arg;
        int booted;
        cpu_bind(c);
        t9k_thread_init(c->cpu_nodeid);         /* T4_T9000: a clock of its own */
        init_processor();
        booted = t4_boot_node_file(c->cpu_nodeid);
        if (booted < 0) {
                t4_net_request_stop();
                return NULL;
        }
        /* A transputer waits for its boot code for as long as it takes
         * (TCWG 9), and a threaded network stops when node 0 does, so a
         * node nobody boots ends with it.  Giving up after 60 s, as this
         * did, stopped the whole network whenever a long chain took longer
         * to boot: 80 nodes on a loaded host, the last five while node 0
         * was still booting them. */
        while (!booted) {
                if (t4_net_stop_requested())
                        return NULL;
                booted = 0 != linkcomms("boot", TRUE, LTO_BOOT);
        }
        /* a T9000 control system restarts a stopped CPU (Run, Reboot) */
        t4_in_mainloop = 1;
        do {
                t4_ls_unpark();         /* T4_LOCKSTEP: running from here */
                mainloop();
                t4_ls_park();
        } while (t9k_ctl_after_halt());
        t4_in_mainloop = 0;
        t9k_report();
        t8c_report();
        return NULL;
}

static void t4_spawn_thread_nodes(int n)
{
        int i;
        pthread_attr_t attr;

        if (!t4_fabric_create(n)) {
                printf("-E-EMU414: Failed to create in-process link fabric\n");
                handler(-1);
        }
        /* before any node opens a link that may end at one */
        if (t4_fabric_start_routers(t4_fabric) < 0)
                handler(-1);
        t4_idle_init(&t4_cpu0);
        t4_fabric_set_cpu(t4_fabric, 0, &t4_cpu0);
        if (t9k.enabled)
                t9k.node = 0;                   /* this thread is node 0 */
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 1024 * 1024);
        t4_nworkers = n - 1;
        for (i = 1; i < n; i++) {
                t4_wc[i] = cpu_create(i, 0);
                if (!t4_wc[i]) {
                        printf("-E-EMU414: Failed to allocate node %d\n", i);
                        handler(-1);
                }
                t4_idle_init(t4_wc[i]);
                t4_fabric_set_cpu(t4_fabric, i, t4_wc[i]);
                if (pthread_create(&t4_wth[i], &attr, t4_node_thread, t4_wc[i]) != 0) {
                        printf("-E-EMU414: Failed to start processor %d thread\n", i);
                        handler(-1);
                }
                printf("-I-EMU414: Started processor %d (thread)\n", i);
        }
        pthread_attr_destroy(&attr);
}

static void t4_join_thread_nodes(void)
{
        int i;
        t4_net_request_stop();
        t4_fabric_stop_routers(t4_fabric);
        for (i = 1; i <= t4_nworkers; i++) {
                if (t4_wth[i])
                        pthread_join(t4_wth[i], NULL);
                if (t4_wc[i]) {
                        t4_idle_destroy(t4_wc[i]);
                        cpu_destroy(t4_wc[i]);
                        t4_wc[i] = NULL;
                }
        }
        t4_idle_destroy(&t4_cpu0);
        t4_fabric_destroy(t4_fabric);
}
int copy        = FALSE;
int exitonerror = false;
int peeksize    = 8;
int nxputers    = 0;
pid_t xputers[1024 + 1];

int membits     = 0;
char *png_save_filename = NULL;
char *tap_device_name = NULL;

#ifdef EMUDEBUG
int tracing     = 0;
int emudebug    = FALSE;
int memdebug    = FALSE;
int memnotinit  = FALSE;
int msgdebug    = FALSE;
int cachedebug  = FALSE;
char *dbgtrigger = NULL;
#endif
int usetvs      = FALSE;
int shlinks     = FALSE;

extern int32_t quit;
extern int32_t quitstatus;

char CommandLineAll[256]  = "\0";
char CommandLineMost[256] = "\0";

/* Append to one of the 256-byte command-line strings above, truncating.
 * Unbounded strcat overflowed them on a long path to t4 or a boot file,
 * and glibc's fortified build aborts on that before main loop starts. */
static void cmdline_cat (char *dst, const char *src)
{
        size_t used = strlen (dst);

        if (used < sizeof CommandLineAll - 1)
                strncat (dst, src, sizeof CommandLineAll - 1 - used);
}

char NetConfigName[256] = "\0";
char ProfileName[256] = "\0";

FILE *CopyIn;
FILE *InpFile, *OutFile;

extern int t4_sd_boot_enabled (void);
void handler (int);

/* Profile flag. */
int profiling = false;

/* Profile array. */
uint64_t profile[10];

/* Profile file. */
FILE *ProfileFile;

/* Signal handler. */
void handler (int);

#ifdef __MWERKS__
int argc; /*= 4;*/
char argv[4][40]; /* = {"jserver","-se","-sb","solport.b4"};*/
#else
#ifdef CURTERM
#include "curterm.h"
#else
/* Terminal settings. */
struct termios t_init;
struct termios t_getk;
struct termios t_poll;
enum {INIT, GETK, POLL} t_state;
#endif
#endif

#ifdef _MSC_VER
pid_t SpawnVP (const char *file, char *argv[])
{
        return _spawnvp (_P_NOWAIT, file, argv);
}
#else
pid_t SpawnVP (const char *file, char *argv[])
{
        pid_t child = fork();

        if (child < 0)
                return child;

        if (0 == child)
        {
                execvp (file, argv);
                handler (-1);
        }
        return child;
}
#endif

void stop_xputers (void)
{
        extern u_char *SharedEvents;

        if (nxputers && SharedEvents)
        {
                int i;

                for (i = 1; -1 != xputers[i]; i++)
                {
                        SharedEvents[i] = 1;
                        t4_wake_node_id (i);    /* if it sleeps on a link */
                        xputers[i] = -1;
                }
        }
}


/* 201113AP Gavin Crate */
int isswopt(int c)
{
        return (c == '-') || (c == '/');
}

#ifdef EMUDEBUG
void set_debug (void)
{
        emudebug = (tracing & 1) ? TRUE : FALSE;
        msgdebug = (tracing & 2) ? TRUE : FALSE;
        memdebug = (tracing & 4) ? TRUE : FALSE;
        cachedebug = (tracing & 8) ? TRUE : FALSE;
}
#endif

const char* Humanoid (uint64_t bytes)
{
        static const char *units = " kMGT";
        static char buf[32];
        int i;
        uint64_t lot;

        lot = 1;
        buf[0] = '\0';
        for (i = 0; units[i]; i++)
        {
                if (bytes < 1024 * lot)
                {
                        sprintf (buf, "%5.1f%cB", (double)bytes / lot, units[i]);
                        break;
                }
                lot *= 1024;
        }
        if (*buf == '\0')
                sprintf (buf, "%5.1fPB", (double)bytes / lot);
        return buf;
}

#ifdef __MWERKS__
void main(void)
#else
int main (int argc, char **argv)
#endif
{
	cpu_bind(&t4_cpu0);
	t4_cpu0.cpu_nodeid = -1;
	t4_cpu0.cpu_serve = FALSE;
	static char CopyFileName[256];
        static char InpFileName[256], OutFileName[256];
        char IBoardSize[32], SpyNet[256];
	int reset __attribute__((unused)) = FALSE;
	int arg;
	int temp;
	int temp2;

#ifdef _MSC_VER
	_set_fmode (_O_BINARY);
#endif	

#ifdef EMUDEBUG
        msgdebug = NULL != getenv ("MSGDEBUG");
#endif
        {
                char *stop_after_gcall = getenv("T4_STOP_AFTER_GCALL");
                char *bootdbg = getenv("T4_BOOTDBG");
                char *bootwin = getenv("T4_BOOTWIN");
                char *uart_trace = getenv("T4_UART_TRACE");
                char *sk_bytes = getenv("T4_SK_BYTE_TRACE");
                char *sk_areg = getenv("T4_SK_AREG_TRACE");
                char *sk_addr = getenv("T4_SK_ADDR_TRACE");
                char *sk_store = getenv("T4_SK_STORE_TRACE");
                char *text_write = getenv("T4_TEXT_WRITE_TRACE");
                char *wptr_trace = getenv("T4_WPTR_TRACE");
                char *sk_entry = getenv("T4_SK_ENTRY_IPTR");
                extern int bootdbg_stop_after_call_enabled;
                extern int bootdbg_enabled;
                extern int bootwin_enabled;
                extern int uart_trace_enabled;
                extern int sk_byte_trace_enabled;
                extern int sk_areg_trace_enabled;
                extern int sk_addr_trace_enabled;
                extern int sk_store_trace_enabled;
                extern int text_write_trace_enabled;
                extern int wptr_trace_enabled;
                extern uint32_t sk_entry_iptr;
                extern int sk_entry_dump_enabled;
                if (stop_after_gcall && stop_after_gcall[0] != '\0')
                        bootdbg_stop_after_call_enabled = atoi(stop_after_gcall) != 0;
                if (bootdbg && bootdbg[0] != '\0')
                        bootdbg_enabled = atoi(bootdbg) != 0;
                if (bootwin && bootwin[0] != '\0')
                        bootwin_enabled = atoi(bootwin) != 0;
                if (uart_trace && uart_trace[0] != '\0')
                        uart_trace_enabled = atoi(uart_trace) != 0;
                if (sk_bytes && sk_bytes[0] != '\0')
                        sk_byte_trace_enabled = atoi(sk_bytes) != 0;
                if (sk_areg && sk_areg[0] != '\0')
                        sk_areg_trace_enabled = atoi(sk_areg) != 0;
                if (sk_addr && sk_addr[0] != '\0')
                        sk_addr_trace_enabled = atoi(sk_addr) != 0;
                if (sk_store && sk_store[0] != '\0')
                        sk_store_trace_enabled = atoi(sk_store) != 0;
                if (text_write && text_write[0] != '\0')
                        text_write_trace_enabled = atoi(text_write) != 0;
                if (wptr_trace && wptr_trace[0] != '\0')
                        wptr_trace_enabled = atoi(wptr_trace) != 0;
                if (sk_entry && sk_entry[0] != '\0') {
                        sk_entry_iptr = (uint32_t)strtoul(sk_entry, NULL, 0);
                        sk_entry_dump_enabled = 1;
                }
        }
        CopyIn = InpFile = OutFile = (FILE *) NULL;
        for (temp = 0; temp < 4; temp++)
        {
                Link[temp].In.sock  = -1;
                Link[temp].Out.sock = -1;
        }

#ifdef __MWERKS__
	/* Create some menus, etc. */
	menu ();
#endif
	if (argc<2)
	{
		printf("\n");
		printf("Usage : t4 [options] [program arguments]\n\n");
		printf("t4 V1.6a    04/04/2023\n\n");
		printf("Options: use either -sX or /sX\n");
		printf("    -sa                  Analyse transputer.\n");
		printf("    -sb filename         Boot program \"filename\".\n");
		printf("    -sc filename         Copy file \"filename\" to transputer.\n");
		printf("    -se                  Terminate on transputer error.\n");
		printf("    -si                  Output progress messages.\n");
		printf("    -sr                  Reset transputer.\n");
		printf("    -sp number           Peek \"number\" kilobytes on analyse.\n");
		printf("    -ss                  Provide host services to transputer.\n");
                printf("Extra options:\n");
                printf("    -s4                  Select T414 mode. (default)\n");
                printf("    -s8                  Select T800 mode.\n");
                printf("    -s806                Select T806 mode (T800 @ 200MHz).\n");
                printf("    -s9                  Select T9000 mode (256us timeslice period).\n");
                printf("    -s414, -s425         Select a T414 or T425 (no FPU) that stops on any\n");
                printf("                         instruction it lacks; -s4 runs them all.\n");
#ifndef NDEBUG
                printf("    -sg                  Halt on uninitialized memory read.\n");
#endif
                printf("    -sl                  Use SHM (shared memory) links.\n");
                printf("    -sm #bits            Memory size in address bits, 16..31 (default 21, 2Mbyte).\n");
                printf("    -sn id               Node ID. (If id < 0, start SHM network with -id nodes).\n");
                printf("    -st                  Threaded nodes (with -sn -N): one pthread per transputer.\n");
#ifdef T4PROFILE
                printf("    -su                  Instruction profiling.\n");
#endif
                printf("    -sv inp.tbo inp.bin out.bin\n");
                printf("                         Select Mike's TVS: T800 + T414 FP support.\n");
#ifdef EMUDEBUG
                printf("    -sw \"string\"         Trigger execution trace on SP_WRITE (string).\n");
                printf("    -sx [number]         Execution trace (8 - cache, 4 - mem ld/st, 2 - iserver, 1 - instructions).\n");
                printf("    --debug              Enable all debug output (same as -sx 15).\n");
#endif
		printf("\n");
		handler (-1);
	}

	/* Handle long options before regular parsing */
#ifdef EMUDEBUG
	for (arg=1; arg<argc; arg++) {
		if (strcmp(argv[arg], "--debug") == 0) {
			tracing = 15;  /* Enable all: instructions, iserver, mem, cache */
			set_debug();
			printf("[DEBUG] Enabled all debug tracing (tracing=%d)\n", tracing);
		}
	}
#endif
	for (arg=1; arg<argc; arg++) {
		if (strcmp(argv[arg], "--save-fb") == 0 && arg+1 < argc) {
			png_save_filename = argv[arg+1];
			arg++;
			printf("[PNG] Framebuffer will be saved to %s after execution\n", png_save_filename);
		}
		if (strcmp(argv[arg], "--tap") == 0 && arg+1 < argc) {
			tap_device_name = argv[arg+1];
			arg++;
		} else if (strcmp(argv[arg], "--tap") == 0) {
			tap_device_name = "tap0";  /* default */
		}
	}

	/* Initialize T4Net NIC (TAP backend) */
	t4net_init(tap_device_name);

	/* Set up signal handler. */
	signal (SIGINT, handler);
	signal (SIGTERM, handler);

	/* Build full command line string for server to return on request. */
	for (arg=0;arg<argc;arg++)
	{
		if (arg>0) cmdline_cat (CommandLineAll, " ");
		cmdline_cat (CommandLineAll, argv[arg]);
	}
#ifdef EMUDEBUG
        if (emudebug)
	        printf ("\nFull command line is : %s\n", CommandLineAll);
#endif

	for (arg=1;arg<argc;arg++)
	{
		if (isswopt(argv[arg][0]) && (argv[arg][1]=='s'))
		{
			switch (argv[arg][2])
			{
				case '4': if (!strcmp (argv[arg] + 2, "414") ||
					      !strcmp (argv[arg] + 2, "425"))
					  {
                                                /* a strict T414 or T425:
                                                 * see t4xx_has_opr().  The
                                                 * T414 has 2K of on-chip RAM
                                                 * and MemStart #80000048,
                                                 * where tcc -mt414 links. */
                                                Txxx = t4_isa_strict = atoi (argv[arg] + 2);
                                                CoreSize    = (Txxx == 425 ? 4 : 2) * 1024;
                                                MemStart    = Txxx == 425 ? 0x80000070 : 0x80000048;
                                                ExtMemStart = Txxx == 425 ? 0x80001000 : 0x80000800;
					  }
					  else if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
                                          {
                                                Txxx=414;
                                                CoreSize    = 2 * 1024;
                                                MemStart    = 0x80000048;
                                                ExtMemStart = 0x80000800;
                                          }
					  break;
				case '8': if (argv[arg][3]=='0' && argv[arg][4]=='6' && argv[arg][5]=='\0')
					  {
                                                /* T806: T800 core @ 200MHz */
                                                Txxx=806;
                                                CoreSize    = 4 * 1024;
                                                MemStart    = 0x80000070;
                                                ExtMemStart = 0x80001000;
					  }
					  else if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
                                          {
                                                Txxx=800;
                                                CoreSize    = 4 * 1024;
                                                MemStart    = 0x80000070;
                                                ExtMemStart = 0x80001000;
                                          }
					  break;
				case '9': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
                                          {
                                                /* T9000: the T9000's timeslice
                                                 * period and device id, and
                                                 * its on-chip memory: after
                                                 * reset the 16 Kbyte cache
                                                 * (four 4 Kbyte banks) is the
                                                 * RAM at #80000000-#80003FFF
                                                 * (HRM 9.6.1, figure 9.13).
                                                 * RamSize/DoRamSize and
                                                 * DoAllocate (#2001-#2005)
                                                 * are only stored: a line
                                                 * that changes role writes
                                                 * back and refills (HRM
                                                 * 9.5), so without a cache
                                                 * model no memory a program
                                                 * reads changes.  MemStart
                                                 * is the T800's, as the
                                                 * VCP's channel words
                                                 * assume. */
                                                Txxx=9000;
                                                CoreSize    = 16 * 1024;
                                                MemStart    = 0x80000070;
                                                ExtMemStart = 0x80004000;
                                          }
					  break;
				case 'a': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else analyse=TRUE;
					  break;
				case 'b': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						arg++;
						if (arg>=argc)
						{
							printf("\nMissing filename after -sb\n");
							handler (-1);
						}
						strcpy (CopyFileName, argv[arg]);
						reset=TRUE;
						serve=TRUE;
						if (!want_threads)
							verbose=TRUE;
						copy=TRUE;
					  }
					  break;
				case 'c': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						arg++;
						if (arg>=argc)
						{
							printf("\nMissing filename after -sc\n");
							handler (-1);
						}
						strcpy (CopyFileName, argv[arg]);
						copy=TRUE;
					  }
					  break;
				case 'e': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else exitonerror=true;
					  break;
#ifndef NDEBUG
				case 'g': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else memnotinit=true;
					  break;
#endif
				case 'i': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else verbose=TRUE;
					  break;
				case 'l': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else shlinks=TRUE;
					  break;
				case 't': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else want_threads=TRUE;
					  break;
				case 'm': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
                                          else
                                          {
						arg++;
						if (arg>=argc)
						{
							printf ("\nMissing number after -sm\n");
							handler (-1);
						}
						if (sscanf(argv[arg], "%d", &membits)!=1)
						{
							printf ("\nBad number after -sm\n");
							handler (-1);
						}
						if ((membits < 16) || (membits > 31))
						{
							printf ("\nMemory bits should be in range [16,31] (64K,2048M)!\n");
							handler (-1);
						}
                                          }
					  break;
				case 'n': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
                                          else
                                          {
						arg++;
						if (arg>=argc)
						{
							printf ("\nMissing number after -sn\n");
							handler (-1);
						}
						if (sscanf(argv[arg], "%d", &nodeid)!=1)
						{
							printf ("\nBad number after -sn\n");
							handler (-1);
						}
                                                if (nodeid < 0)
                                                {
                                                        /* Network startup */
                                                        nxputers = -nodeid;
                                                        if ((nxputers < 1) || (nxputers > 1024))
                                                        {
							        printf ("\nNumber of transputers should be in range [1,1024]!\n");
							        handler (-1);
                                                        }
                                                        nodeid = 0;
                                                        if (!want_threads)
                                                                shlinks = TRUE;
                                                }
						if ((nodeid < 0) || (nodeid > 1024))
						{
							printf ("\nNode ID  should be in range [0,1023]!\n");
							handler (-1);
						}
                                          }
					  break;
				case 'r': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else reset=TRUE;
					  break;
				case 'p': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						arg++;
						if (arg>=argc)
						{
							printf("\nMissing number after -sp\n");
							handler (-1);
						}
						if (sscanf(argv[arg], "%d", &peeksize)!=1)
						{
							printf("\nBad number after -sp\n");
							handler (-1);
						}
						if (peeksize > 16)
						{
							printf("\nCannot peek that much memory.\n");
							handler (-1);
						}
					  }
					  break;
				case 's': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else serve=TRUE;
					  break;
#ifdef T4PROFILE
				case 'u': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else profiling=true;
					  break;
#endif
                                case 'v': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						if (arg + 3 >= argc)
						{
							printf("\nMissing parameters after -sv\n");
							handler (-1);
						}
                                                strcpy (CopyFileName, argv[arg + 1]);
                                                strcpy (InpFileName, argv[arg + 2]);
                                                strcpy (OutFileName, argv[arg + 3]);
                                                arg += 3;
                                                usetvs = TRUE;
                                                Txxx   = 1144;
                                                CoreSize    = 4 * 1024;
                                                MemStart    = 0x80000070;
                                                ExtMemStart = 0x80001000;
					  }
					  break;
#ifdef EMUDEBUG
				case 'w': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						arg++;
						if (arg>=argc)
						{
							printf("\nMissing string after -sw\n");
							handler (-1);
						}
                                                dbgtrigger = argv[arg];
					  }
					  break;
				case 'x': if (argv[arg][3]!='\0')
					  {
						cmdline_cat (CommandLineMost, argv[arg]);
						cmdline_cat (CommandLineMost, " ");
					  }
					  else
					  {
						arg++;
						if (arg >= argc)
                                                        tracing = 1;
                                                else if (isswopt(argv[arg][0]))
                                                {
                                                        tracing = 1;
                                                        arg--;
                                                }
						else 
						{
                                                        if (sscanf(argv[arg], "%d", &tracing)!=1)
                                                        {
							        printf("\nBad number after -sx\n");
							        handler (-1);
                                                        }
						}
					  }
					  break;
#endif
				default : cmdline_cat (CommandLineMost, argv[arg]);
					  cmdline_cat (CommandLineMost, " ");
					  break;
			}
		}
		else
		{
			cmdline_cat (CommandLineMost, argv[arg]);
			cmdline_cat (CommandLineMost, " ");
		}
	}
	if (strlen(CommandLineMost))
		CommandLineMost[strlen(CommandLineMost)-1] = '\0';

        /* Recalculate external memory settings, when membits set. */
        if (membits)
        {
                MemSize     = 1U << membits;
                MemByteMask = MemSize - 1;
                MemWordMask = MemByteMask & (~3U);
                t4_relocate_mmio_for_ram (MemSize);
        }

        sprintf (IBoardSize, "IBOARDSIZE=#%X", MemSize);
        putenv (IBoardSize);

	{
		const char *e = getenv ("T4_T9000");
		if (e && *e && *e != '0') {
			t9k.enabled = 1;
			t9k_init ();
			printf ("-I-EMU: T9000 instruction-grouper model enabled "
			        "(estimate; see transputer/t4/t9000_group.h)\n");
		}
	}
	{
		const char *e = getenv ("T4_EST_CYCLES");
		if (e && (*e == '1' || *e == '2')) {
			t8c_enabled = *e - '0';
			printf ("-I-EMU: T800 cycle estimate enabled, ldtimer "
			        "counts %s (see transputer/t4/t800_cycles.h)\n",
			        t8c_enabled == 1 ? "cycles" : "instructions");
		}
	}
        CLineTagsSize = ((MemSize >> T4CLINEBITS) >> 3);
	if (cpu_setup_ram(&t4_cpu0) < 0)
	{
		printf ("\nFailed to allocate emulator memory!\n");
		exit (-1);
	}


#ifdef EMUDEBUG
        if (tracing && (dbgtrigger == NULL))
                set_debug ();

        if (emudebug)
        {
	        printf("Most command line is : %s\n", CommandLineMost);
	        printf("T%d (%d/%dKB); analyse %d; copy %d; exit %d; verbose %d; reset %d; peek %d; serve %d; node %d\n",
                        Txxx,CoreSize/1024,MemSize/1024,analyse,copy,exitonerror,verbose,reset,peeksize,serve,nodeid);
        }
#endif

	/* Initialise profiling array. */
	for (temp=0; temp<10; temp++)
		profile[temp] = 0;

        /* Initialize memory with 'random' values. */
        init_memory ();

        /* Read network config file. */
        if (nodeid >= 0)
        {
                FILE *NetIn;
                char *spyNet;

                if (verbose)
                        printf ("-I-EMU414: Node ID = %d.\n", nodeid);
                spyNet = getenv ("SPYNET");
                strcpy (NetConfigName, spyNet ? spyNet : "spy.net");
                if ((NetIn = fopen (NetConfigName, "r")) == NULL)
                {
                        char *p = strchr (CopyFileName, '.');
                        if (p)
                        {
                                temp = p - CopyFileName;
                                strncpy (NetConfigName, CopyFileName, temp);
                                NetConfigName[temp] = '\0';
                        }
                        else
                        {
                                strcpy (NetConfigName, CopyFileName);
                        }
                        strcat (NetConfigName, ".map");
                        if ((NetIn = fopen (NetConfigName, "r")) == NULL)
                        {
                                printf ("Failed to open network config file %s!\n", NetConfigName);
                                handler (-1);
                        }
                }
                if (verbose)
                        printf ("-I-EMU414: Reading network configuration from %s...\n", NetConfigName);
                if (readNetConfig (NetIn) < 0)
                {
                        handler (-1);
                }
                if (verbose)
                        printf ("-I-EMU414: Done.\n");
                fclose (NetIn);

                if (nxputers)
                {
                        char *te = getenv("T4_THREADS");
                        if (te && *te && *te != '0')
                                want_threads = TRUE;
                        if (want_threads && shlinks) {
                                printf ("-E-EMU414: -st cannot be combined with -sl\n");
                                handler (-1);
                        }
#if !defined(__linux__) && !defined(__APPLE__)
                        if (want_threads) {
                                printf ("-E-EMU414: -st is not supported on this OS\n");
                                handler (-1);
                        }
#endif
                        if (!want_threads && netRouterCount ()) {
                                printf ("-E-EMU414: Router nodes need threaded nodes (-st)\n");
                                handler (-1);
                        }
                        sprintf (SpyNet, "SPYNET=%s", NetConfigName);
                        putenv (SpyNet);
                        if (want_threads)
                                t4_spawn_thread_nodes (nxputers);
                        else {
                        char *cmdargs[8];
                        char snid[8];
                        int  i;

                        for (i = 1; i < 1025; i++)
                                xputers[i] = -1;

                        for (i = 1; i < nxputers; i++)
                        {
                                sprintf (snid, "%d", i);

                                cmdargs[0] = "t4";
                                cmdargs[1] = "-sl";
                                cmdargs[2] = "-sn";
                                cmdargs[3] = snid;
                                cmdargs[4] = (Txxx == 9000) ? "-s9" :
                                             (Txxx == 806) ? "-s806" : (Txxx == 800) ? "-s8" :
                                             (t4_isa_strict == 414) ? "-s414" :
                                             (t4_isa_strict == 425) ? "-s425" : NULL;
                                cmdargs[5] = NULL;
                                if ((xputers[i] = SpawnVP ("t4", cmdargs)) < 0)
                                {
                                        printf ("-E-EMU414: Failed to start processor %d!\n", i);
                                        handler (-1);
                                }
                                printf ("-I-EMU414: Started processor %d (pid=%d)\n", i, xputers[i]);
                                if (1 == i)
                                        sleep (1);
                        }
                        if (nxputers > 2)
                                sleep (1);
                        }
                }
                else if (netRouterCount ())
                {
                        /* a node started on its own (-sn without a count)
                         * would never see the routers of its map */
                        printf ("-E-EMU414: Router nodes need threaded nodes (-st -sn -N)\n");
                        handler (-1);
                }
        }

        /* Initialize processor. */
        init_processor ();

#ifdef T4_SDL_FB
        display_init();
#endif

        if (t4_sd_boot_enabled())
        {
                /* T9000 boot-from-ROM (TIS 10.5.2): after power-up the
                 * control unit peeks BootWptr (#7FFFFFFC) then BootIptr
                 * (#7FFFFFF8) from the top of store — served here from
                 * the SD boot bank — and the CPU executes in place. */
                extern uint32_t word_int (uint32_t);
                WPtr = word_int (0x7FFFFFFC);
                IPtr = word_int (0x7FFFFFF8);
                copy = FALSE;
                fprintf (stderr,
                         "-I-EMUBOOT: T9000 SD boot bank: BootIptr=0x%08X BootWptr=0x%08X\n",
                         IPtr, WPtr);
        }
        else if ((FALSE == serve) && (nodeid >= 0))
        {
                /* A node of a network of processes waits up to
                 * T4_BOOT_WAIT seconds (default 600) for its boot code: a
                 * transputer waits for ever (TCWG 9), but a process whose
                 * node 0 has gone would wait for ever too, and only the
                 * nodes node 0 itself started hear that it has
                 * (SharedEvents).  It was 60 s, and -sl's polled links
                 * took longer than that to boot the 60th node of 80. */
                extern u_char *SharedEvents;
                extern volatile sig_atomic_t t4_sig_pending;
                extern void t4_signal_exit (int);
                const char *bw = getenv ("T4_BOOT_WAIT");
                int maxwait = (bw && atoi (bw) > 0) ? atoi (bw) : 600;

                temp = 1;
                do
                {
                        if (verbose)
                                printf ("-I-EMU414: Node%d waiting for bootstrap code... (%d)\n", nodeid < 0 ? 0 : nodeid, temp);
                        if (t9k_ctl_busy ())    /* configuring over CLink0 */
                                temp = 1;
                        if (SharedEvents && SharedEvents[nodeid])
                                temp = maxwait + 1;     /* node 0 has stopped */
                        /* handler() only notes a signal outside mainloop() */
                        if (t4_sig_pending)
                        {
                                int sig = (int) t4_sig_pending;

                                t4_sig_pending = 0;
                                t4_signal_exit (sig);
                        }
                } while ((temp++ < maxwait) && (0 == linkcomms ("boot", TRUE, LTO_BOOT)));
                if (temp > maxwait)
                {
                        printf ("-E-EMU414: Failed to get bootstrap code!\n");
                        handler (-1);
                }
        }
        else
        {
                /* Open boot file. */
                if ((CopyIn = fopen(CopyFileName, "rb"))==NULL)
	        {
		        printf("Failed to open file %s for copy to link!\n", CopyFileName);
		        handler (-1);
	        }

	        /* Load bootstrap into emulator. */
                if (usetvs)
                {
                        printf ("-I-EMUTVS: Running test %s\n", CopyFileName);
                        /* Optional InpFileName. */
                        if ((0 != strlen (InpFileName)) &&
                            (InpFile = fopen (InpFileName, "rb")) == NULL)
                        {
                                printf ("-E-EMUTVS: Error - failed to open file %s for input!\n", InpFileName);
                                handler (-1);
                        }
                        if ((OutFile = fopen (OutFileName, "wb")) == NULL)
                        {
                                printf ("-E-EMUTVS: Error - failed to open file %s for output!\n", OutFileName);
                                handler (-1);
                        }
                }

                /* TVS tbo code is bootstrap code. */
	        temp = getc (CopyIn);
	        if (temp == 0)
	        {
		        /* Allow a zero-length bootstrap for netload-generated images. */
		        fprintf(stderr, "-I-EMUBOOT: No bootstrap present; loading payload at MemStart\n");
	        }
	        else if (temp < 2)
	        {
		        printf ("\nFile does not start with bootstrap code!\n");
		        handler (-1);
	        }

	        for (temp2=0; temp2<temp; temp2++)
	        {
		        writebyte_int ((MemStart+temp2), getc (CopyIn));
	        }
	        WPtr = MemStart + temp2;

		fprintf(stderr, "-I-EMUBOOT: Loaded bootstrap: %d bytes at 0x%08X\n", temp, MemStart);
		fprintf(stderr, "-I-EMUBOOT: Full bootstrap dump (%d bytes):\n", temp);
		for (int i=0; i<temp; i++) {
			if (i % 16 == 0) fprintf(stderr, "  %04X: ", i);
			fprintf(stderr, "%02X ", byte_int(MemStart + i));
			if (i % 16 == 15 || i == temp-1) fprintf(stderr, "\n");
		}

	        /* Decode chunked BTL data after bootstrap when no inline bootstrap was used. */
	        if (temp == 0) {
	            long current_pos, end_pos, remaining;
	            unsigned long load_addr = MemStart + temp2;
	            int chunk_len;
	            unsigned char buffer[256];

	            current_pos = ftell(CopyIn);
	            fseek(CopyIn, 0, SEEK_END);
	            end_pos = ftell(CopyIn);
	            fseek(CopyIn, current_pos, SEEK_SET);
	            remaining = end_pos - current_pos;

	            fprintf(stderr, "-I-EMUBOOT-DEBUG: current_pos=%ld end_pos=%ld remaining=%ld\n",
	                   current_pos, end_pos, remaining);

	            if (remaining > 0) {
	                fprintf(stderr, "-I-EMUBOOT: Decoding chunked BTL to memory at 0x%08lx (%ld bytes remaining)\n",
	                       load_addr, remaining);
	            }

	            while ((chunk_len = getc(CopyIn)) != EOF) {
	                if (chunk_len == 0)
	                    break;
	                if (fread(buffer, 1, chunk_len, CopyIn) != (size_t)chunk_len)
	                    break;
	                for (temp = 0; temp < chunk_len; temp++) {
	                    writebyte_int(load_addr++, buffer[temp]);
	                }
	            }
		            fprintf(stderr, "-I-EMUBOOT: Loaded %ld bytes total to memory\n", load_addr - MemStart);
		            WPtr = load_addr;
		            copy = FALSE;  /* Don't use link protocol */
		        }

	        }

        if (!t9k_ctl_started_cpu ())    /* Run/Reboot set Wdesc and Iptr */
        {
	        while ((WPtr & 0x00000003) != 0x00000000)
		        WPtr++;
                UpdateWdescReg (WPtr | LoPriority);
        }
        fprintf(stderr, "-I-EMUBOOT: Entering mainloop WPtr=0x%08X copy=%d\n",
                WPtr, copy ? 1 : 0);

#ifdef CURTERM
        /* Initialise terminal settings. */
        prepterm (1);
#else
#ifndef __MWERKS__
	/* Initialise t_init terminal settings. */
	if ((ioctl (0, TCGETS, &t_init)) != 0)
	{
		printf("\nBad ioctl - initialising t_init.\n");
		handler (-1);
	}
	t_state = INIT;

	/* Initialise t_getk terminal settings. */
	if ((ioctl (0, TCGETS, &t_getk)) != 0)
	{
		printf("\nBad ioctl - initialising t_getk.\n");
		handler (-1);
	}
	/* Turn off buffering and echoing. */
	t_getk.c_iflag &= ~ICRNL;
	t_getk.c_lflag &= ~(ICANON | ECHO);
	/* Get at least 1 char, wait as long as you have to. */
	t_getk.c_cc[VMIN] = 1;
	t_getk.c_cc[VTIME] = 0;

	/* Initialise t_poll terminal settings. */
	if ((ioctl (0, TCGETS, &t_poll)) != 0)
	{
		printf("\nBad ioctl - initialising t_poll.\n");
		handler (-1);
	}
	/* No buffering, no echoing. */
	t_poll.c_iflag &= ~ICRNL;
	t_poll.c_lflag &= ~(ICANON | ECHO);
	/* Don't necessarily get a charcter, wait 0.1s. */
	t_poll.c_cc[VMIN] = 0;
	t_poll.c_cc[VTIME] = 1;
#endif
#endif

#ifdef T4PROFILE
	if (profiling)
	{
		/* Open profiling file. */
                strcpy (ProfileName, "profile");
                if (nodeid >= 0)
                        sprintf (ProfileName, "profile.%d", nodeid);
		if ((ProfileFile = fopen(ProfileName, "w"))==NULL)
		{
			printf("Failed to open %s file!\n", ProfileName);
			handler (-1);
		}
	}
#endif


#if __profile__
	/* METROWERKS PROFILER. */
	if (ProfilerInit(collectDetailed,bestTimeBase,10,10) == noErr)
	{
#endif

	/* Now start the emulator; a T9000 control system may restart a
	 * stopped CPU (Run, Reboot). */
	t4_in_mainloop = 1;
	do {
	        t4_ls_unpark ();        /* T4_LOCKSTEP: running from here */
	        mainloop ();
	        t4_ls_park ();
	} while (t9k_ctl_after_halt ());
	t4_in_mainloop = 0;
        if (want_threads)
                t4_join_thread_nodes();
	t4_ls_report ();

#if __profile__
	/* METROWERKS PROFILER. */
		ProfilerDump("\pTutorial.prof");
		ProfilerTerm();
	}
#endif

	/* Close boot file. */
        if (CopyIn)
	        fclose (CopyIn);
        if (InpFile)
                fclose (InpFile);
        if (OutFile)
                fclose (OutFile);


        stop_xputers ();
        t4net_cleanup();

        close_channels ();

#ifdef T4PROFILE
	if (profiling)
	{
		/* Print out profile counts. */
                fprintf (ProfileFile, "-----Statistics-------------------\n");
                fprintf (ProfileFile, "Instructions           %llu\n", profile[PRO_INSTR]);
                fprintf (ProfileFile, "Elapsed time           %llums\n", profile[PRO_ELAPSEDMS]);
                fprintf (ProfileFile, "MIPS                   %.1f\n", (profile[PRO_INSTR] / (double)profile[PRO_ELAPSEDMS]) / 1000);
                fprintf (ProfileFile, "Cache Miss/Hit         %llu/%llu\n", profile[PRO_ICMISS], profile[PRO_ICHIT]);
                fprintf (ProfileFile, "Cache Hit Rate         %.1f\n", 100.0 * profile[PRO_ICHIT] / (profile[PRO_ICMISS] + (double)profile[PRO_ICHIT]));
                fprintf (ProfileFile, "Descheduling points    %llu\n", profile[PRO_DCHECK]);
                fprintf (ProfileFile, "Server calls           %llu\n", profile[PRO_ISERVER]);
                fprintf (ProfileFile, "StartProcess calls     %llu\n", profile[PRO_STARTP]);
                fprintf (ProfileFile, "-----Channels---------------------\n");
                fprintf (ProfileFile, "ChannelOut             %s\n", Humanoid (profile[PRO_CHANOUT]));
                fprintf (ProfileFile, "ChannelIn              %s\n", Humanoid (profile[PRO_CHANIN]));
                for (temp = 0; temp < 4; temp++)
                {
                        if (Link[temp].In.IOBytes)
                                fprintf (ProfileFile, "Link%dIn                %s\n", temp, Humanoid (Link[temp].In.IOBytes));
                        if (Link[temp].Out.IOBytes)
                                fprintf (ProfileFile, "Link%dOut               %s\n", temp, Humanoid (Link[temp].Out.IOBytes));
                }

#ifdef T4PROFILEINSTR
		print_profile ();
#endif

		fclose (ProfileFile);
	}
#endif

#ifdef CURTERM
        prepterm (0);
#else
#ifndef __MWERKS__
	if (t_state != INIT)
	{
		if ((ioctl (0, TCSETS, &t_init)) != 0)
		{
			printf("\nBad ioctl while resetting terminal.\n");
			handler (-1);
		}
	}
#endif
#endif

#ifdef __MWERKS__
	while (1)
		check_input ();
#endif
}


void mmio_early_printk_report(void);

/* Deferred-exit flag.  See the note in handler(). */
volatile sig_atomic_t t4_sig_pending = 0;

/* SIGINT/SIGTERM.
 *
 * This used to do all of its reporting - t9k_report(), fprintf(), fflush(),
 * then exit() - directly in the handler.  None of those are async-signal-safe,
 * and the emulator writes UART output through stdio continuously, so a signal
 * arriving while the main loop held the stdio lock deadlocked the handler on
 * the lock it was already holding.  The process then ignored SIGTERM forever:
 * `timeout 8s t4 ...` left emulators alive for hours, which silently stalled
 * the UART test runner and starved every other job on the machine.
 *
 * Now the handler only records the signal - the one thing that is safe - and
 * the interpreter loop performs the report and exits at its next instruction
 * boundary.  A second signal escalates to _exit(), which is async-signal-safe,
 * so an emulator wedged outside the loop can still be killed.
 *
 * timeout(1) delivers one kill twice, back to back: to the child and to its
 * process group.  Escalating on that duplicate lost the final report
 * (FINAL_STATE, T4_WATCH) on most timed-out runs, so a second signal only
 * escalates once the loop has had a couple of seconds to service the first.
 * time() is async-signal-safe. */
static time_t t4_sig_first;

void t4_signal_exit (int signal);

void handler (int signal)
{
	time_t now;

	/* handler(-1) is also T4's own stop on a fatal error.  Only mainloop()
	 * services the flag, so outside it - starting up, booting a node,
	 * shutting down - T4 would carry on past the error.  This is an
	 * ordinary call, not a signal, so the exit path runs here, as it did
	 * before signals were deferred. */
	if (-1 == signal && !t4_in_mainloop)
		t4_signal_exit (-1);
	now = time (NULL);
	if (t4_sig_pending) {
		if (now - t4_sig_first >= 2)
			_exit (-1);
		return;
	}
	t4_sig_first = now;
	t4_sig_pending = signal;
}

void t4_signal_exit (int signal)
{
#ifdef __MWERKS__
	DialogPtr dialog;
	short itemHit = 0;
#endif

        fflush (stdout);
        t9k_report ();
        t8c_report ();
        t4_ls_report ();
        {
                extern void t4_net_dump (void);
                const char *d = getenv ("T4_NET_DUMP");
                static _Atomic int dumped;

                if (d && *d && *d != '0' && !atomic_exchange (&dumped, 1))
                        t4_net_dump ();
        }
        fprintf(stderr, "[FINAL_STATE] signal=%d IPtr=0x%08X WPtr=0x%08X A=0x%08X B=0x%08X C=0x%08X\n",
                signal, IPtr, WPtr, AReg, BReg, CReg);
        /* T4_WATCH=addr[,words] dumps guest memory at exit.  A hang in a list
         * walk is a data problem, not a control-flow one, so the only way to
         * tell a corrupt list from a mis-compiled loop is to look at the
         * pointers.  Space-separated list of addresses is accepted. */
        {
                extern uint32_t word_int (uint32_t);
                const char *w = getenv("T4_WATCH");
                while (w && *w) {
                        char *end;
                        unsigned long addr = strtoul(w, &end, 0);
                        int n = 8, i;
                        if (end == w) break;
                        if (*end == ',') n = (int)strtol(end + 1, &end, 0);
                        fprintf(stderr, "[WATCH] 0x%08lX:", addr);
                        for (i = 0; i < n; i++)
                                fprintf(stderr, " %08X",
                                        word_int ((uint32_t)(addr + (unsigned long)i * 4)));
                        fprintf(stderr, "\n");
                        while (*end == ' ' || *end == ',') end++;
                        w = end;
                }
        }
        fflush(stderr);

	PROFILE(fclose (ProfileFile));

        if (CopyIn)
                fclose (CopyIn);
        if (InpFile)
                fclose (InpFile);
        if (OutFile)
                fclose (OutFile);

        stop_xputers ();
        t4net_cleanup();

        /* Save SD card image if file-backed */
        { extern void sd_save(void); sd_save(); }

        close_channels ();

#ifdef T4_SDL_FB
        if (png_save_filename) {
            save_framebuffer_png(png_save_filename);
        }
        /* Always dump framebuffer as BMP for headless/WSL use */
        vga_dump_framebuffer_bmp("framebuffer.bmp", 0);
        display_shutdown();
#endif

        mmio_early_printk_report();


#ifdef CURTERM
   	prepterm (0);
	exit (-1);
#else
#ifdef __MWERKS__
	/* Exit dialog. */
	dialog = GetNewDialog (129, nil, (void *) -1);
	
	while (itemHit == 0)
	{
		ModalDialog (nil, &itemHit);
	}
	
	DisposDialog (dialog);

	free (mem);
	ExitToShell ();
#else
	if (t_state != INIT)
	{
		if ((ioctl (0, TCSETS, &t_init)) != 0)
		{
			printf("\nBad ioctl while resetting terminal.\n");
			/* handler (-1); */
		}
	}

	exit (-1);
#endif
#endif
}
