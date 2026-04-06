/*{{{Comments*/
/*
 * File        : cmonitor.c
 * Author      : Stuart Menefy
 * Description : Main portion of boot monitor.
 * Header      : $Id: cmonitor.c,v 1.17 1994/10/11 10:22:10 stuart Exp stuart $
 *   
 * This is the generic C portion of the boot monitor.
 * 
 * It assumes that another module will provide all the I/O routines.
 * It's main function is to load the kernel, mm, fs and init into memory,
 * and then jump into the kernel.
 *
 * History:
 *   04/07/93  SIM  Created.
 *   07/02/94  SIM  Rewrite of module loading code.
 *   08/02/94  SIM  Added command line interface code.
 *   09/02/94  SIM  Pass memory size and boot pareameters onto Minix.
 *   10/02/94  SIM  Added basic module code.
 *   20/02/94  SIM  Added 'more' and 'source' commands.
 *   03/03/94  SIM  Added 'procnum' command and changed sizes generation.
 *   22/07/94  SIM  Added 'params' and 'terminate' commands.
 */
/*}}}*/

/*{{{#include*/
#include <a.out.h>
#include <string.h>
#include <minix/config.h>
#include <minix/const.h>
#include <sys/types.h>
#include <minix/boot.h>
#include <stdlib.h>
#include <transputer/channel.h>
#include <transputer/iserver.h>

#include "io.h"
#include "monitor.h"
/*}}}*/
/*{{{#defines*/
#define NAME "Minix boot monitor"
#define VERSION "0.11"
/*}}}*/

/*{{{globals*/
extern void* bootLinkIn;
extern void* bootLinkOut;
extern int   savedDetails[4];
extern int   memSize;

extern char* itoa(const int n);
extern void start(int* code, int *stack);

struct bparam_s boot_parameters = {
  DROOTDEV, DRAMIMAGEDEV, DRAMSIZE, DSCANCODE, DPROCESSOR,
};

command_t mcommands[];

#define MAX_MODULES 20
#if 0
static module_t modules[MAX_MODULES] = {
    {"kernel", "kernel", 0, data_first}, 
    {"mm",     "mm",     0, text_first}, 
    {"fs",     "fs",     0, text_first}, 
    {"init",   "init",   0, text_first}
};
static int num_modules = 4;
#else
static module_t modules[MAX_MODULES];
static int num_modules = 0;
#endif
static module_t *cur_module;

static int read_from_file = 0;
/*}}}*/

/*{{{print_header*/
void print_header(void)
{
    printk("\n"NAME" : Version "VERSION" (built "__DATE__" "__TIME__")\n\n");
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
/*{{{print_cmd*/
void print_cmd(command_t *cp, int help)
{
    int spaces;

    printk(cp->cmd);

    if (help) {
        for (spaces = 10 - strlen(cp->cmd) ; spaces > 0; spaces--)
            putchar(' ');
    }

    if (cp->args != NULL) {
        putchar(' ');
        printk(cp->args);
        spaces = 14 - strlen(cp->args);
    } else {
        spaces = 15;
    }

    if (help) {
        for (; spaces > 0; spaces--)
            putchar(' ');
        printk(cp->help);
    }

    putchar('\n');
}
/*}}}*/
/*{{{print_help*/
void print_help(command_t *commands)
{
    command_t *cp;

    for (cp = commands; cp->cmd != NULL; cp++) {
        print_cmd(cp, 1);
    }
}

/*}}}*/
/*{{{get_short*/
void get_short(const char* string, int min, int max, short* result)
{
    int tmp;
    char* end;

    tmp = (int)strtol(string, &end, 0);
    if (*end != '\0') {
        printk("Unrecognised number\n");
        return;
    }

    if ((tmp < min) || (tmp > max)) {
        printk("Number must be in the range ");
        printk(itoa(min));
        printk(" to ");
        printk(itoa(max));
        printk("\n");
        return;
    }
    
    *result = (short)tmp;
}
/*}}}*/

/*{{{mdatafirst*/
command_t *mdatafirst(int argc, char *argv[])
{
    cur_module->order = data_first;

    return mcommands;
}
/*}}}*/
/*{{{mend*/
command_t *mend(int argc, char *argv[])
{
    if (cur_module - modules == num_modules)
        num_modules++;
    return commands;
}
/*}}}*/
/*{{{mfile*/
command_t *mfile(int argc, char *argv[])
{
    strcpy(cur_module->filename, argv[1]);

    return mcommands;
}
/*}}}*/
/*{{{mhelp*/
command_t *mhelp(int argc, char *argv[])
{
    print_help(mcommands);
    return mcommands;
}
/*}}}*/
/*{{{mparams*/
command_t *mparams(int argc, char *argv[])
{
    int arg;
    char* dest;
    char* src;

    cur_module->argc = argc-1;

    dest = cur_module->args;
    while ((src = *(++argv)) != NULL) {
        while ((*dest++ = *src++) != '\0')
            ;
    }

    return mcommands;
}
/*}}}*/
/*{{{mprint*/
command_t *mprint(int argc, char *argv[])
{
    unsigned int arg;
    char* argp;
    char* quote;

    printk("Module  : ");
    printk(cur_module->name);
    printk("\nFilename: ");
    printk(cur_module->filename);
    printk("\nPriority: ");
    printk(itoa(cur_module->priority));
    printk("\nProc num: ");
    printk(itoa(cur_module->procnum));
    printk("\nOrder   : ");
    if (cur_module->order == data_first)
        printk("Data below text");
    else
        printk("Text below data");

    printk("\nParams  : ");
    argp = cur_module->args;
    for (arg = cur_module->argc; arg > 0; arg--) {
        quote = strchr(argp, ' ');
        if (quote != NULL)
            putchar('"');
        printk(argp);
        argp += strlen(argp) + 1;
        if (quote != NULL)
            putchar('"');
        putchar(' ');
    }
    putchar('\n');

    return mcommands;
}
/*}}}*/
/*{{{mpriority*/
command_t *mpriority(int argc, char *argv[])
{
    get_short(argv[1], 0, 255, (short*)&cur_module->priority);

    return mcommands;
}
/*}}}*/
/*{{{mprocnum*/
command_t *mprocnum(int argc, char *argv[])
{
    get_short(argv[1], -NR_TASKS, 255, &cur_module->procnum);

    return mcommands;
}
/*}}}*/

/*{{{mcommands*/
command_t mcommands[] = {
    {"datafirst",  1, 1, mdatafirst,  NULL,       "Place data below text"},
    {"end",        1, 1, mend,        NULL,       "Finish definition"},
    {"file",       2, 2, mfile,       "filename", "Module filename"},
    {"help",       1, 1, mhelp,       NULL,       "Print this text"},
    {"params",     1,10, mparams,     "arguments","Command line arguments"},
    {"print",      1, 1, mprint,      NULL,       "Print current module"},
    {"priority",   2, 2, mpriority,   "priority", "Module priority"},
    {"procnum",    2, 2, mprocnum,    "proc num", "Process number"},
    {NULL,         0, 0, NULL,        NULL,       NULL}
};

/*}}}*/

/*{{{boot*/
command_t *boot(int argc, char *argv[])
{
    if (num_modules == 0) {
        printk("ERROR: No modules to load\n");
        return commands;
    }

    printk("Booting\n");
    return NULL;
}
/*}}}*/
/*{{{help*/
command_t *help(int argc, char *argv[])
{
    print_header();
    print_help(commands);

    printk("\nReset Wdesc: ");
    print_int(savedDetails[0]);
    printk(" Iptr: ");
    print_int(savedDetails[1]);
    printk("\n\n");
    
    return commands;
}
/*}}}*/
/*{{{module*/
command_t *module(int argc, char *argv[])
{
    int i;

    for (i = 0; i<num_modules; i++) {
        if (strcmp(argv[1], modules[i].name) == 0) {
            printk("Editing existing module\n");
            cur_module = &modules[i];
            return mcommands;
        }
    }

    if (num_modules == MAX_MODULES) {
        printk("No more modules\n");
        return commands;
    }

    printk("Creating new module\n");
    cur_module = &modules[num_modules];
    strcpy(cur_module->name, argv[1]);
    cur_module->filename[0]  = '\0';
    cur_module->priority     = 0;
    cur_module->order        = text_first;
    cur_module->argc         = 0;

    return mcommands;
}
/*}}}*/
/*{{{more*/
command_t *more(int argc, char *argv[])
{
    char buffer[80];
    extern int file_inode;

    r_open(argv[1]);
    if (file_inode != -1) {
        while (r_readline(buffer, sizeof(buffer))) {
            printk(buffer);
            putchar('\n');
        }
        r_close();
    }

    return commands;
}
/*}}}*/
/*{{{ramsize*/
command_t *ramsize(int argc, char *argv[])
{
    if (argc == 1) {
        printk("RAM disk set to ");
        printk(itoa(boot_parameters.bp_ramsize));
        printk("K\n");
    } else {
        get_short(argv[1], 0, memSize / 1024, (short*)&boot_parameters.bp_ramsize);
    }
    return commands;
}
/*}}}*/
/*{{{rootdev*/
command_t *rootdev(int argc, char *argv[])
{
    if (argc == 1) {
        printk("Root device set to ");
        print_int(boot_parameters.bp_rootdev);
        printk("\n");
    } else {
        get_short(argv[1], 0, memSize / 1024, (short*)&boot_parameters.bp_rootdev);
    }
    return commands;
}
/*}}}*/
/*{{{source*/
command_t *source(int argc, char *argv[])
{
    extern int file_inode;

    r_open(argv[1]);
    read_from_file = (file_inode != -1);

    return commands;
}
/*}}}*/

/*{{{read_module*/
void read_module(module_t* mp, int address)
{
    char buffer[1024];
    int len_read;
    int text_len, data_len;
    int offset, part_len;

    r_open(mp->filename);
    len_read = r_read(buffer);

    if ((((struct exec*)buffer)->a_magic[0] != A_MAGIC0) ||
        (((struct exec*)buffer)->a_magic[1] != A_MAGIC1)) {
        printk("Error: Module ");
        printk(mp->filename);
        printk(" is not an executable file\n");
        while (1)
            ;
    }
    
    text_len = (int)((struct exec*)buffer)->a_text;
    data_len = (int)((struct exec*)buffer)->a_total;

    if (! (((struct exec*)buffer)->a_flags & A_SEP)) {
        data_len -= text_len;
    }
        
    printk("Module ");
    printk(mp->filename);
    printk(" text size ");
    print_int(text_len);
    printk(" data size ");
    print_int(data_len);
    printk("\n");

    mp->text_clicks = (text_len + (CLICK_SIZE-1)) >> CLICK_SHIFT;
    if (mp->order == data_first) {
        /* This is the kernel, the first module loaded, so we have to allow
         * some padding for the transputers' own use of low memory. */
        mp->data_clicks = (data_len + 0x70 + (CLICK_SIZE-1)) >> CLICK_SHIFT;
        address += (mp->data_clicks << CLICK_SHIFT);
    } else {
        mp->data_clicks = (data_len + (CLICK_SIZE-1)) >> CLICK_SHIFT;
    }

    printk("Loaded from ");
    print_int(address);

    offset = ((struct exec*)buffer)->a_hdrlen;
    while (1) {
        part_len = len_read - offset;
        if (part_len > text_len)
            part_len = text_len;
        memcpy((void*)address, &buffer[offset], part_len);
        address  += part_len;
        text_len -= part_len;
        if (text_len == 0)
            break;
        len_read = r_read(buffer);
        offset = 0;
    }

    printk(" to ");
    print_int(address);
    printk("\n");

    r_close();
}

/*}}}*/
/*{{{getline*/
static void getline(const char prompt, char* buffer, int max_len)
{
    int len = 0;
    char c;

    max_len--;  /* Allow space for nul character */

    putchar(prompt);
    putchar(' ');
    while (1) {
        c = getchar();
        switch (c) {
            case '\r':
            case '\n':
                buffer[len] = '\0';
                putchar('\n');
                return;

            case 0x7f:
            case 8:
                if (len > 0) {
                    printk("\b \b");
                    len--;
                }
                break;

            case 0x15:  /* CTRL U */
                for ( ; len > 0; len --) {
                    printk("\b \b");
                }
                break;

            default:
                if ((c > 31) && (len < max_len)) {
                    putchar(c);
                    buffer[len++] = c;
                } else {
                    putchar('\a');
                }
                break;
	}
    }
}
/*}}}*/
/*{{{cli*/
#define MAX_ARGS 10
void cli(void)
{
    char buffer[80];
    int argc;
    command_t *cur_commands;
    command_t *cp;
    char* argv[MAX_ARGS];
    char* ptr;
    int len;
    extern int file_inode;
    char terminate;
    extern void r_cmdline(char* buffer, int len);

    if (read_from_file) {
        r_cmdline(buffer, 80);
        if (buffer[0] == '\0') {
            strcpy(buffer, "minix.cf");
        }
        printk("read \"");
        printk(buffer);
        printk("\"\n");
        r_open(buffer);
        read_from_file = (file_inode != -1);
    }

    cur_commands = commands;
    do {
        if (read_from_file) {
            read_from_file = r_readline(buffer, sizeof(buffer));
            if (! read_from_file) {
                /* Should also check at end if still in a nested command */
                r_close();
            }
        } else {
            getline((cur_commands == commands) ? '>' : ':',
                    buffer, sizeof(buffer));
        }

        /* Break the buffer up into arguments. */
        ptr = buffer;
        argc = 0;
        terminate = ' ';
        while (1) {
            while (*ptr == ' ')
                ptr++;
            if ((*ptr == '\0') || (*ptr == '#')) {
                break;
            }
            if (argc == MAX_ARGS)
                break;
            if (*ptr == '"') {
                terminate = '"';
                if (*(++ptr) == '\0')
                    break;
            }
            argv[argc++] = ptr;
            while ((*ptr != terminate) && (*ptr != '\0'))
                ptr++;
            if (*ptr != '\0') {
                *(ptr++) = '\0';
                terminate = ' ';
            }
	}
        if (argc == MAX_ARGS) {
            printk("Too many arguments\n");
            continue;
        }
        if (terminate != ' ') {
            printk("Unterminated string\n");
            continue;
        }
        argv[argc] = NULL;

        /* Check for nul commands and comments */
        if ((argc == 0) || (argv[0][0] == '#'))
            continue;

        /* Check for valid commands */
        len = strlen(argv[0]);
        for (cp = cur_commands; cp->cmd != NULL; cp++) {
            if (strncmp(cp->cmd, argv[0], len) == 0) {
                if (argc < cp->min_args) {
                    printk("Insufficient arguments : ");
                    print_cmd(cp, 0);
                } else if (argc > cp->max_args) {
                    printk("Too many arguments : ");
                    print_cmd(cp, 0);
                } else {
                    cur_commands = cp->fn(argc, argv);
                }
                break;
            }
        }

        if (cp->cmd == NULL) {
            printk("Unrecognised command\n");
        }
    } while (cur_commands != NULL);
}
/*}}}*/
/*{{{cmonitor*/
void c_monitor(void)
{
    int component;
    int base;
    int sizes[8];
    int* stack;
    int* kernel_start;
    module_t *mp;
    
    term_init();    
    print_header();
    read_from_file = disk_init();
    cli();
    
    base = 0x80000000;
    for (component = 0, mp = &modules[0];
         component < num_modules;
         component++, mp++) {
        read_module(mp, base);
        base += (mp->text_clicks + mp->data_clicks) << CLICK_SHIFT;

        if (component < 4) {
            /* Still set up the sizes array for backward compatibility */
            sizes[ component * 2     ] = mp->text_clicks;
            sizes[(component * 2) + 1] = mp->data_clicks;
        }
    }

    disk_finish();
    
    /*
     * Set up the kernel's stack, and jump into it.
     *
     *   18 num_modules
     *   17 load_struct
     *   16 boot_parameters
     *
     *   15 savedReg3
     *   14 savedReg2
     *   13 oldIptr
     *   12 oldWdesc
     *
     *   11 initDataSize
     *   10 initCodeSize
     *    9 fsDataSize  
     *    8 fsCodeSize  
     *    7 mmDataSize  
     *    6 mmCodeSize  
     *    5 kernelDataSize
     *    4 kernelCodeSize
     *  
     *    3 memSize
     *    2 memStart
     *    1 linkInAddress
     *    0 linkOutAddress
     *
     * OK. Lets doooo it!
     */    

    /**********************************************************************
     * REMEMBER!! The current kernel uses the whole of the data space for *
     * static, and so chmem =0 does not work, as there is no initial      *
     * stack for it to perform initialisation on. chmem =100 works OK.    *
     **********************************************************************/

    kernel_start = (int*)(0x80000000 + (sizes[1] << CLICK_SHIFT));
    stack = &kernel_start[-19];

    stack[0] = (int)bootLinkOut;
    stack[1] = (int)bootLinkIn;
    stack[2] = 0x80000070;
    stack[3] = (int)memSize;

    memcpy(&stack[4], sizes, 8*sizeof(int));
    memcpy(&stack[12], savedDetails, 4*sizeof(int));

    stack[16] = (int)&boot_parameters;
    stack[17] = (int)&modules[0];
    stack[18] = num_modules;

    start(kernel_start, stack);
        /* This will never return */
}
/*}}}*/
