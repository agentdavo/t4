/*{{{Comments*/
/*
 * File        : secondl.c
 * Author      : Stuart Menefy
 * Description : Secondary bootstrap link support functions.
 * Header      : $Id: secondl.c,v 1.3 1994/04/11 16:40:29 stuart Exp $
 *
 * Routines for the secondary bootstrap when booting from an IServer.
 * Provides IServer specific commands for the secondary bootstrap.
 *
 * History:
 *   06/02/94  SIM  Created.
 *   11/04/94  SIM  Shortened monitor executable filename for MS-DOS.
 */
/*}}}*/

/*{{{#include*/
#include <string.h>
#include <a.out.h>
#include "io.h"
/*}}}*/
/*{{{#defines*/
#define BLOCK_SIZE 507
/*}}}*/


/*{{{load*/
void load(int memory, void **code, int **data)
{
    /* Because we cannot declare a buffer on the stack or in the static
     * area big enough, use the top of memory and then move it down. */
    char* buffer;
    char* ptr;
    int len;
    int text_len, data_len;

    buffer = &((char*)0x80000000)[memory - BLOCK_SIZE];
    buffer = (char*)((int)buffer & ~3);      /* Round down to word boundary */
    
    r_open("monitorl");
    len = r_read(buffer);
    if ((len < A_MINHDR) ||
        (((struct exec*)buffer)->a_magic[0] != A_MAGIC0) ||
        (((struct exec*)buffer)->a_magic[1] != A_MAGIC1)) {
        printk("\nError: monitorl is not an executable file\n");
        while (1)
            ;
    }

    text_len = (int)((struct exec*)buffer)->a_text;
    data_len = (int)((struct exec*)buffer)->a_total;
    if (! (((struct exec*)buffer)->a_flags & A_SEP)) {
        data_len -= text_len;
    }

    ptr = &((char*)0x80000000)[memory - text_len];
    ptr = (char*)((int)ptr & ~3);      /* Round down to word boundary */
    len -= ((struct exec*)buffer)->a_hdrlen;
    buffer += ((struct exec*)buffer)->a_hdrlen;

    memcpy(ptr, buffer, len);   /* This should be memmove, but memcpy is shorter */

    *code = ptr;
    *data = (int*)&ptr[-data_len];
    
    ptr += len;
    text_len -= len;
    while (text_len > 0) {
        len = r_read(ptr);
        ptr += len;
        text_len -= len;
    }
}
/*}}}*/

