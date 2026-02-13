/*{{{Comments*/
/*
 * File        : monitord.c
 * Author      : Stuart Menefy
 * Description : Boot monitor file system support functions.
 * Header      : $Id: monitord.c,v 1.9 1994/08/05 11:10:00 stuart Exp stuart $
 *
 * Routines for the boot monitor when booting from a file system.
 * Provides file system specific commands for the boot monitor.
 *
 * History:
 *   06/02/94  SIM  Created.
 */
/*}}}*/

/*{{{#include*/
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <minix/config.h>
#include <minix/const.h>
#include "rawfs.h"
#include "io.h"
#include "monitor.h"

/*}}}*/

#if 0
/*{{{m_read*/
static int m_read(int buffer)
{
    static char data1[] = {
        6               /* ReadSector() */
    };
    int block;
    
    block = r_vir2abs(offset);

    printk("Read block ");
    print_int(block + 2);
    printk(" into buffer ");
    print_int(buffer);
    printk("\n");
    
    select_block(block + 2);
    select_buffer(buffer);
    ChanOut(bootLinkOut, data1, sizeof(data1));
    check_error(block+2);
    
    offset++;

    return BLOCK_SIZE;
}
/*}}}*/
/*{{{execute_buffer*/
static void execute_buffer(int start_buffer, int entry_offset, int stack_req)
{
    static char data[] = {
        3, 0x1d, 0,     /* WriteParam(DesiredSectorBuffer, n) */
        3, 0x10, 0,     /* WriteParam(RWCCylinderBy4 = EntryOffsetLSB, n) */
        3, 0x11, 0,     /* WriteParam(PCCylinderBy4  = EntryOffsetMSB, n) */
        3, 0x12, 0,     /* WriteParam(SectorRetries  = WorkspaceReqLSB, n) */
        3, 0x13, 0,     /* WriteParam(SeekRetries    = WorkspaceReqMSB, n) */
        0xf             /* Boot */
    };

    printk("Execute (buffer ");
    print_int(start_buffer);
    printk(", Entry offset ");
    print_int(entry_offset);
    printk(", stack requirement ");
    print_int(stack_req);
    printk(")\n");
    
    data[2] = start_buffer;
    data[5] = entry_offset % 256;
    data[8] = entry_offset / 256;
    data[11] = stack_req % 256;
    data[14] = stack_req / 256;
    ChanOut(bootLinkOut, data, sizeof(data));

    printk("Started OK\n");
}
/*}}}*/
/*{{{load_multimon*/
void load_multimon(void)
{
    int buffer_num, start_buffer;
    char buffer[1024];
    int tmp;
    int stack_req, vector_req, entry_offset, code_size, hdr_len;
    int sectors;

    printk("Loading multimon...\n");

    r_open("multimon.rsc");
    r_read(buffer);

    hdr_len = ((int*)buffer)[0];
    memcpy(&tmp, &buffer[4 + hdr_len], 4);
    hdr_len = 4 + hdr_len + 4 + tmp;

    memcpy(&tmp, &buffer[hdr_len], 4);
    if (tmp != 2) {
        printk("Multimon not compiled for T2\n");
        return;
    }

    memcpy(&tmp, &buffer[hdr_len+4], 4);
    if (tmp != 10) {
        printk("Illegal version number ");
        print_int(tmp);
        printk("\n");
        return;
    }
    
    memcpy(&stack_req,    &buffer[hdr_len +  8], 4);
    memcpy(&vector_req,   &buffer[hdr_len + 12], 4);
    memcpy(&entry_offset, &buffer[hdr_len + 16], 4);
    memcpy(&code_size,    &buffer[hdr_len + 20], 4);
    hdr_len += 24;

    printk("Stack req ");
    print_int(stack_req);
    printk(" words\nVector req ");
    print_int(vector_req);
    printk(" words\nEntry offset ");
    print_int(entry_offset);
    printk(" bytes\nCode size ");
    print_int(code_size);
    printk(" bytes\n");

    offset = 0; /* reset the file pointer back to the start */
    start_buffer = buffer_num = (stack_req - 356 + 511) / 512;
    for (sectors = (code_size + hdr_len + 1023) / 1024; sectors > 0; sectors--) {
        m_read(buffer_num++);
    }

    r_close();    

    /* Start multimon */
    execute_buffer(start_buffer, entry_offset + hdr_len, stack_req);

    /* Test Multimon is loaded */
    {
        int i = 4;
        printk("About to send");
        ChanOut(bootLinkOut, &i, 2);
    /*  printk("Sent, about to receive");
        ChanIn(bootLinkIn, &i, 2);
        printk("Value returned is ");
        print_int(i); */
        printk("\n");
    }

    {
        static char data2[] = {
            2, 0x20         /* ReadParam(Error) */
        };
        char error;

        printk("About to read errro\n");
        ChanOut(bootLinkOut, data2, sizeof(data2));
        printk("Send req OK, get result\n");
        ChanIn(bootLinkIn, &error, 1);
        printk("Got result OK of");
        print_int(error);
    }
    
    
    /* Call disk_init() again, this time for the multimon */
    disk_init();
}
/*}}}*/
#endif

/*{{{disk_finish*/
void disk_finish(void)
{
    /* Nothing to do for the hard disk. */
    /* Could do a restore, make booting quicker... */
}
/*}}}*/
/*{{{r_cmdline*/
void r_cmdline(char* buffer, int max_len)
{
    buffer[0] = '\0';
}
/*}}}*/

/*{{{cd*/
command_t *cd(int argc, char* argv[])
{
    ino_t   inode;
    extern int errno;
    extern ino_t dir_inode;

    if (dir_inode == 0) {
        printk("Must select a drive first\n");
        return commands;
    }

    if ((argv[1][0] == '/') && (argv[1][1] == '\0')) {
        dir_inode = ROOT_INO;
    } else {
        inode = r_lookup(dir_inode, argv[1]);
        if (inode == 0) {
            printk("Unable to find directory (");
            print_int(errno);
            printk(")\n");
        } else {
            dir_inode = inode;
        }
    }

    return commands;
}
/*}}}*/
/*{{{drive*/
command_t *drive(int argc, char* argv[])
{
    short d, p;
    extern drive_select(int, int);

    get_short(argv[1], 1, 4, &d);
    if (argc > 2)
        get_short(argv[2], 0, 3, &p);
    drive_select(d, p);

    return commands;
}
/*}}}*/
/*{{{ls*/
command_t *ls(int argc, char* argv[])
{
    struct stat sbuf;
    int inode;
    char name[20];
    int spaces;
    extern int errno;
    extern ino_t dir_inode;
    
    if (dir_inode == 0) {
        printk("Must select a drive first\n");
        return commands;
    }

    r_stat(dir_inode, &sbuf);
    printk("Stat size ");
    print_int((int)sbuf.st_size);
    printk("\n");

    while ((inode = r_readdir(name)) > (ino_t)0) {
        printk(name);
        for (spaces = 20 - strlen(name); spaces > 0; spaces--)
            printk(" ");
        print_int(inode);
        printk("\n");
    }

    if (inode == (ino_t)-1) {
        printk("Unable to read directory ");
        print_int(errno);
        printk("\n");
    }

    return commands;
}
/*}}}*/

/*{{{commands list*/
command_t commands[] = {
    {"boot",     1, 1, boot,     NULL,          "Boot Minix"},
    {"cd",       2, 2, cd,       "directory",   "Change current directory"},
    {"drive",    2, 3, drive,    "drive [part]","Select drive and partition"},
    {"help",     1, 1, help,     NULL,          "Print this text"},
    {"ls",       1, 1, ls,       NULL,          "List current directory"},
    {"module",   2, 2, module,   "name",        "Define a new module"},
    {"more",     2, 2, more,     "filename",    "Display a file"},
    {"ramsize",  1, 2, ramsize,  "[size in K]", "Set RAM disk size"},
    {"rootdev",  1, 2, rootdev,  "[device]",    "Set root file system"},
    {"source",   2, 2, source,   "filename",    "Read commands from file"},
    {NULL,       0, 0, NULL,     NULL,          NULL}
};
/*}}}*/
