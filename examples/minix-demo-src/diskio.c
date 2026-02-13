/*{{{Comments*/
/*
 * File        : diskio.c
 * Author      : Stuart Menefy
 * Description : M212 disk reading routines.
 * Header      : $Id: diskio.c,v 1.4 1994/05/16 18:18:36 stuart Exp $
 *   
 * This file supplies the disk reading functions to support cmonitor.c.
 * This is used when running on a B016 and using a raw disk interface via
 * an M212.
 *
 * History:
 *   04/07/93  SIM  Created.
 *   06/02/94  SIM  Most of the previous functionality moved into
 *                  fileio.c and termio.c, rewritten to support csecond.c.
 */
/*}}}*/

/*{{{#include*/
#include "io.h"
/*}}}*/
/*{{{externs and globals*/
extern void* bootLinkIn;
extern void* bootLinkOut;

#define BLOCK_SIZE 256
/*}}}*/
/*{{{Channel communications macros*/
#define ChanOut(link, data, size) \
  __asm{ ldabc size, link, data; out; }
#define ChanIn(link, data, size) \
  __asm{ ldabc size, link, data; in; }
/*}}}*/

/*{{{disk_init*/
int disk_init(void)
{
    static char data[] = {
        3, 0x1e, 3,     /* WriteParam(DesiredDrive, 3) */
        0xb             /* Select drive */
    };

    ChanOut(bootLinkOut, data, sizeof(data));

    return 0;   /* Do not attempt to open minix.cf */
}
/*}}}*/
/*{{{r_open*/
void r_open(const char* filename)
{
    filename = filename;    /* Keep compiler happy about unused vars */
}
/*}}}*/
/*{{{r_read*/
int  r_read(char* ptr)
{
    static char data[] = {
        6,              /* Read sector */
        4               /* Read buffer */
    };

    ChanOut(bootLinkOut, data, sizeof(data));
    ChanIn(bootLinkIn, ptr, BLOCK_SIZE);
    return BLOCK_SIZE;
}
/*}}}*/
/*{{{r_close*/
void r_close(void)
{
    static char data[] = {
        3, 0x1e, 0,     /* WriteParam(DesiredDrive, 0) */
        0xb             /* Select drive */
    };

    ChanOut(bootLinkOut, data, sizeof(data));
}
/*}}}*/

