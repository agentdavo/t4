/*{{{Comments*/
/*
 * File        : monitorl.c
 * Author      : Stuart Menefy
 * Description : Boot monitor link support functions.
 * Header      : $Id: monitorl.c,v 1.11 1994/10/11 10:24:16 stuart Exp stuart $
 *
 * Routines for the boot monitor when booting from an IServer.
 * Provides IServer specific commands for the boot monitor.
 *
 * History:
 *   06/02/94  SIM  Created.
 *   20/02/94  SIM  Added r_readline() command for 'more' and 'source'.
 */
/*}}}*/

/*{{{#include*/
#include <string.h>
#include <transputer/channel.h>
#include <transputer/iserver.h>
#include "io.h"
#include "monitor.h"

#undef BLOCK_SIZE
#define BLOCK_SIZE 507
/*}}}*/
/*{{{externs and globals*/
extern void* bootLinkIn;
extern void* bootLinkOut;

static enum {CONTINUE, TERMINATE} atEnd = CONTINUE;
/*}}}*/

/*{{{disk_finish*/
void disk_finish(void)
{
    char hdr[8] = {35,                  /* Exit */
                   0xff,0xc9,0x9a,0x3b, /* Status (999999999) */
                   0,0,0};              /* Padding */
    int pkt_len = 8;                    /* 7 + padding */

    if (atEnd == CONTINUE)
        return;

    /* Send an IServer terminate request. */    
    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, pkt_len);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, &hdr, pkt_len);
}
/*}}}*/

/* These functions are here to prevent linkio.c growing too big. */
/*{{{r_readline*/
int r_readline(char* buffer, int max_len)
{
    char hdr[7] = {14,                  /* Fgets */
                   0,0,0,0,             /* Handle (patched) */
                   0,0};                /* Length (patched) */
    int pkt_len = 8;                    /* 7 + padding */
    int len = 0;                        /* Clear msb and init for eof */
    extern int file_inode;

    memcpy(&hdr[1], &file_inode, 4);
    memcpy(&hdr[5], &max_len, 2);

    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, pkt_len);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, &hdr, 1);
    pkt_len--;
    if (pkt_len >= 2) {
        ChanIn(bootLinkIn, &len, 2);
        pkt_len -= 2;
    }
    if (len > 0) {
        ChanIn(bootLinkIn, buffer, len);
        pkt_len -= len;
    }

    if (pkt_len > 0)
        ChanIn(bootLinkIn, &hdr[1], pkt_len);

    buffer[len] = '\0';    
    return (hdr[0] == 0);
}

/*}}}*/
/*{{{r_cmdline*/
void r_cmdline(char* buffer, int max_len)
{
    char hdr[2] = {40,                  /* CommandLine */
                   0};                  /* All */
    int pkt_len = 6;                    /* 2 + padding */
    int len = 0;                        /* Clear msb and init for eof */
    extern int file_inode;

    ChanOut(bootLinkOut, &pkt_len, 2);
    ChanOut(bootLinkOut, hdr, pkt_len);

    ChanIn(bootLinkIn, &pkt_len, 2);
    ChanIn(bootLinkIn, &len, 1);
    ChanIn(bootLinkIn, &len, 2);
    pkt_len -= 3;
    while ((max_len > 0) && (len > 0)) {
        ChanIn(bootLinkIn, buffer, 1);
        pkt_len--;
        len--;
        buffer++;
    }
    *buffer = '\0';

    while (pkt_len > 0) {
        ChanIn(bootLinkIn, &len, 1);
        pkt_len--;
    }
}


/*}}}*/

/*{{{rboot*/
command_t *rboot(int argc, char *argv[])
{
    short link;
    channel_t *linkOutAddr, *linkInAddr;
    char buffer[512];
    int size;
    extern int file_inode;
    int looping;

    get_short(argv[1], 0, 3, &link);
    linkOutAddr = &((channel_t*)0x80000000)[link];
    linkInAddr  = &((channel_t*)0x80000010)[link];

    if (argc == 2)
        argv[2] = "boot.btl";

    r_open(argv[2]);
    if (file_inode != -1) {
        while ((size = r_read(buffer)) != 0) {
            print_int(size);
            ChanOut(linkOutAddr, buffer, size);
        }

        printk("OK\n");
        r_close();

        looping = 1;
        while (looping) {
            size = ChanInInt16(linkInAddr);
            ChanIn(linkInAddr, buffer, size);

            switch (buffer[0]) {
                case SP_EXIT:
                    buffer[0] = ER_SUCCESS;
                    size = 1;
                    looping = 0;
                    break;

                case SP_COMMAND:
                    buffer[0] = ER_SUCCESS;
                    if (argc == 4) {
                        buffer[1] = strlen(argv[3]);
                        buffer[2] = 0;
                        strcpy(&buffer[3], argv[3]);
		    } else {
                        buffer[1] = buffer[2] = 0;
		    }
		    size = buffer[1] + 3;
                    break;

                default:
                    ChanOutInt16(bootLinkOut, size);
                    ChanOut(bootLinkOut, buffer, size);

                    size = ChanInInt16 (bootLinkIn);
                    ChanIn(bootLinkIn, buffer, size);
            }

            ChanOutInt16(linkOutAddr, size);
            ChanOut(linkOutAddr, buffer, size);
        }
    }

    return commands;
}
/*}}}*/
/*{{{terminate*/
static command_t *terminate(int argc, char *argv[])
{
    atEnd = TERMINATE;

    return commands;
}
/*}}}*/

/*{{{commands list*/
command_t commands[] = {
    {"boot",     1, 1, boot,     NULL,          "Boot Minix"},
    {"help",     1, 1, help,     NULL,          "Print this text"},
    {"module",   2, 2, module,   "name",        "Define a new module"},
    {"more",     2, 2, more,     "filename",    "Display a file"},
    {"ramsize",  1, 2, ramsize,  "[size in K]", "Set RAM disk size"},
    {"rboot",    2, 4, rboot,    "link [file]", "Boot processor off link"},
    {"rootdev",  1, 2, rootdev,  "[device]",    "Set root file system"},
    {"source",   2, 2, source,   "filename",    "Read commands from file"},
    {"terminate",1, 1, terminate,NULL,          "Terminate after loading"},
    {NULL,       0, 0, NULL,     NULL,          NULL}
};

/*}}}*/
