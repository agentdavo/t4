/*{{{Comments*/
/*
 * File        : csecond.c
 * Author      : Stuart Menefy
 * Description : Transputer Minix secondary bootstrap.
 * Header      : $Id: csecond.c,v 1.6 1994/05/10 14:46:24 stuart Exp $
 *
 * This is the second block of code which is executed on the 32 bit transputer.
 * As such it will be loaded directly above the stack of the primary bootstrap
 * and so has very limited stack.
 *
 * It's purpose is initialise enough of the I/O system to be able to print
 * some messages while testing the size of memory, and load in the remainder
 * of the boot monitor.
 *
 * History:
 *   30/01/94  SIM  Created.
 */

/*}}}*/

/*{{{#include*/
#include <string.h>
#include "io.h"
/*}}}*/
/*{{{externs and globals*/
void* bootLinkIn;
void* bootLinkOut;

extern int check_B016(void);
extern unsigned int _ldtimer(void);
extern void load(int mem_size, void **code, int **data);

/*}}}*/

/*{{{memsize*/
/* Find out how much memory there is on the current system.
 * This code assumes that it is run from internal memory, and that the
 * whole of main memory can be corrupted.
 */

int memsize(void)
{
    int *base = (int*)0x80000800;
    int *top;
    int *test;
    int inc = 0x200;    /* 512 words = 2K bytes */
    unsigned int last_time, curr_time;
    char* flicks = "-\\|/";
    char* cur_flicks = flicks;

    /* Main test */
    printk("Sizing memory ");
    top = base;
    last_time = _ldtimer();
    while (1) {
        curr_time = _ldtimer();
        if ((int)(curr_time - last_time) > (15625/5)) {
            putchar(*cur_flicks);
            putchar('\b');
            if ((++cur_flicks) == &flicks[4])
                cur_flicks = flicks;
            last_time = curr_time;
        }

        if (top == (int*)0x80400000) {
            /* Because the B016 locks up when you try and access memory
             * off the end of the DRAM block, we detect B016s here
             * and don't attempt any further memory probes.
             */
            if (check_B016()) {
                printk("found B016.\n");
                return 0x400000;
            }
	}        

        *top = 0;
        if (*top != 0) return (int)top - 0x80000000;
        *top = -1;
        if (*top != -1) return (int)top - 0x80000000;

        *top = (int)top;
        for (test = base; test != top; test += inc) {
            if (*test != (int)test) {
                printk("finished.\n");
                return (int)top - 0x80000000;
            }
        }
        top += inc;
    }
}
/*}}}*/
/*{{{print_int*/
void print_int(int i)
{
    char buffer[9];
    int loop, val;

    for (loop=7; loop >= 0; loop--)
    {
        val = i & 0xf;
        buffer[loop] = (val > 9) ? val + 'a' - 10 : val + '0';
        i >>= 4;
    }

    buffer[8] = '\0';
    printk(buffer);
}
/*}}}*/
/*{{{main*/
main(int dummy, void* linkOutAddress, void* linkInAddress, int memStart,
     int oldWdesc, int oldIptr, int storedData0, int storedData1)
{
    void *code;
    int *data, *stack;
    int mem_size;

    bootLinkIn  = linkInAddress;
    bootLinkOut = linkOutAddress;
    
    term_init();
    printk("\n\nSecondary bootstrap entered.\n");

    mem_size = top_code();
    printk("Free mem starts at ");
    print_int(mem_size);
    printk("\n");
    if (mem_size > 0x80000800) {
        while (1)
            ;
    }

    mem_size = memsize();
    printk("Found ");
    print_int(mem_size);
    printk(" bytes.\n");

    (void)disk_init();
    printk("Loading boot monitor...");
    load(mem_size, &code, &data);
    printk("\b\b\b ok.\n");

    /* Set up the stack for the boot monitor, and jump into it. */
    stack = &((int*)code)[-12];
    stack[11] = (int)data;
    memcpy(&stack[7], &oldWdesc, 4*sizeof(int));
    stack[6]  = mem_size;
    stack[5]  = memStart;
    stack[4]  = (int)linkInAddress;
    stack[3]  = (int)linkOutAddress;

    start(code, stack);
        /* This will never return */
}
/*}}}*/
